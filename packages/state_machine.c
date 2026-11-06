          lambda: !lambda |-
            static const char*state_string[] = {"Idle", "Starting", "EarlyRun", "Running", "Defrosting", "Stopping", "Afterrun","Cooling","PCtlCooling"};
            enum States {Idle ,  Starting ,  EarlyRun ,  Running ,  Defrosting ,  Stopping ,  Afterrun, Cooling, CoolingPowerCtl};
            static States state = Idle;
            static States newstate = Idle;
            // mpa stands for minimum power avoidance
            static const char*mpa_state_string[] = {"Idle", "Initializing", "Stabilizing", "ReInitialize", "Active"};
            enum MpaStates {mpaIdle ,  mpaInitializing, mpaStabilizing, mpaReInitialize,  mpaActive };
            static MpaStates mpastate = mpaIdle;
            static MpaStates newmpastate = mpaIdle;
            static float mpa_compressorspeed = 0;

            static uint32_t timer = 0;
            static uint32_t mpatimer = 0;
            static float mpatarget = 0;
            static uint32_t statechange = 0; //timer value upon previous state change
            static uint32_t compressortime = 0; //timer value on last compressor start

            // power target control, see DESIGN.md "Power target control". Uses LG and power meter sensors only.
            static bool pt_active = false;        // power target control engaged in this run
            static float pt_target = NAN;         // target the held setpoint was calculated for
            static bool pt_calc_pending = false;  // setpoint for pt_target still has to be calculated
            static bool pt_step = false;          // new target: skip the ES2 settle time
            static float pt_setpoint = NAN;       // held water setpoint
            static float pt_floor_est = NAN;      // last valid estimated floor temperature
            static float pt_kin = NAN;            // last valid electrical power per K water dT (W/K)
            static int pt_es = 2;                 // energy state requested by the power controller
            static uint32_t pt_es_since = 0;      // timer value of last energy state change
            static uint32_t pt_trim_time = 0;     // timer value of last setpoint calculation or steadiness reset
            static float pt_trim_to = NAN;        // outlet temperature at pt_trim_time, to detect a steady LG
            const char *pt_status = "Off";

            static uint32_t dt = round(id(state_machine).get_update_interval()/1000);
            auto water_temp_call = id(water_temp_target_output).make_call();
            auto set_energy_state = [&](int value) {
              auto call = id(energy_state).make_call();
              call.set_value(value);
              call.perform();
              ESP_LOGD("power_target", "Set energy state to ES%d", value);
            };
            #define set_target_temp(x) if (fabs((x)-id(water_temp_target_output).state) > 0.7) {water_temp_call.set_value(round(x));water_temp_call.perform();ESP_LOGD("set_target_temp", "target set to: %f", round(x));}

            // Setpoint that gives power pt, from the current operating point (see DESIGN.md):
            //   Tset = To + (Pt/P - 1) * dT / eps
            // falls back to the last floor estimate when the current point is not usable (low power or dT).
            auto pt_feedforward = [&](float pt) -> float {
              const float to = id(water_temp_aanvoer).state;
              const float dtw = id(water_dt_corrected).state;
              const float p = id(lg_total_active_power_filtered).state;
              const float eps = id(floor_eps).state;
              if (isnan(eps) || isnan(to))
                return NAN;
              if (!isnan(p) && !isnan(dtw) && (p > 300.0f) && (dtw > 1.0f))
                return to + (pt / p - 1.0f) * dtw / eps;
              if (!isnan(pt_floor_est) && !isnan(pt_kin))
                return pt_floor_est + pt / (eps * pt_kin);
              return NAN;
            };

            const float requested_power = id(power_target_effective).state;
            if (pt_active && (((state != Running) && (state != Defrosting)) || !(requested_power >= 0.05))) {
              // power target ended or the run ended, hand control back
              pt_active = false;
              pt_target = NAN;
              pt_calc_pending = false;
              pt_setpoint = NAN;
              pt_es = 2;
              if (id(energy_state).state != 2)
                set_energy_state(2);
            }

            timer += dt;
            if (timer < 60) { // we just got booted, let's take some time to figure out what's happening before taking action
              if (id(operation_mode).current_option() == "Cooling") {
                if ((id(compressor_running).state) && (requested_power >= 0.05))
                  newstate = state = CoolingPowerCtl;
                else
                  newstate = state = Cooling;
              } else if (id(compressor_running).state) {
                newstate = state = Running;
              } else {
                if (id(water_temp_target_output).state > 20) // turn on if stooklijn asks more then absolute water minimum
                  newstate = state = Starting;
                else
                  newstate = state = Idle;
              }

              ESP_LOGD(state_string[state], "Since: %lus", timer - statechange);
              return;
            }



            switch (state) {
              case Cooling:
                // Cooling control only implemented for Target Power, otherwise manual operation possible
                if (id(operation_mode).current_option() == "Heating")
                  newstate = Idle;
                if (requested_power >= 0.05) { //Target power requested
                  newstate = CoolingPowerCtl;
                  id(force_run_end).turn_off();
                  id(modbus_enable_heat).turn_on();
                  set_target_temp(18.0);
                }

                break;
              case CoolingPowerCtl:
                if (requested_power < 0.05) { //Target power to 0
                  set_target_temp(19);
                  newstate = Cooling;
                  id(modbus_enable_heat).turn_off();
                } else { //Target power requested
                  double target = 0.0;
                  if (!(id(compressor_running).state)) { // Compressor not started. Need to set the target low enough to start the compressor
                    target = (double)((id(water_temp_aanvoer).state + (C->water_hyst_neg)));
                    set_target_temp(target);
                    compressortime = timer;
                  }
                  else if (((timer - compressortime)> (15*60))) { // regular running phase
                    /*
                       assuming target COP is equal to current COP, we get: (t = target, a = actual, i = input, o = output)
                       COPt = COPa
                       Pt/(Tt-Ti) = Pa/(To-Ti)
                       Pt*(To-Ti) = Pa*(Tt-Ti)
                       Pt/Pa*(To-Ti) = (Tt-Ti)
                       Pt/Pa*(To-Ti)+Ti = Tt
                     */
                    target = requested_power / (id(lg_total_active_power_filtered).state / (id(hp_temp_out).state-id(hp_temp_in).state)) + id(hp_temp_in).state;
                    target = max(target, 12.0);
                    target = min(target, 19.0);
                  } else { // still in startup phase, target is stable
                    target = id(water_temp_target_output).state;
                  }

                  // ensure we don't put the temperature so high it will turn off.
                  target = min((double)((id(water_temp_aanvoer).state + (C->water_hyst_pos))), target);
                  set_target_temp(target);

                }
                break;

              case Idle:
              //Nothing is going on, keep checking if something should be going on
                if (id(energy_state).state != 2)
                  set_energy_state(2);
                if (id(thermostat_wp_heat).state) {
                  newstate = Starting;
                  id(force_run_end).turn_off();
                  id(modbus_enable_heat).turn_on();
                  break;
                }
                // If the mode has switched to Cooling, we go to cooling operations. -- should this be a separate state machine? For now it's just other states. Also means we only switch from Idle to Cooling...
                if (id(operation_mode).current_option() == "Cooling")
                  newstate = Cooling;
                C->custom_idle_behavior();
                break;
              case Starting:
              //we want the compressor to start...
                if (id(energy_state).state != 2)
                  set_energy_state(2);
                if (id(compressor_running).state) {
                  // apparently the compressor started..
                  compressortime = timer;
                  newstate = EarlyRun;
                  break;
                } else if (!id(thermostat_wp_heat).state) {
                  // the compressor didn't start and the thermostat turned off
                  newstate = Idle;
                  break;
                }

                // set temperature high enough so compressor will start
                //set_target_temp(id(water_temp_aanvoer).state + 3.0); // old fixed hysteresis
                set_target_temp(id(water_temp_aanvoer).state + (C->water_hyst_pos + 1.0));
                break;
              case EarlyRun:
              // Heatpump operates along a predefined behavior. Just wait until it's done and keep the target as low as possible to limit overshoot
                //id(modbus_set_silent_mode).turn_off(); //silent mode off during start (is this right? Might cause bigger startup overshoot
                if (requested_power > id(power_target_silent_limit).state)
                  id(modbus_set_silent_mode).turn_off(); // high power target: overshoot is wanted
                else
                  id(modbus_set_silent_mode).turn_on(); // turn silent mode on at start, to get less overshoot
                if ((timer - statechange) > (15*60)) { // EarlyRun takes at most 15 minutes
                  newstate = Running;
                  newmpastate = mpaIdle;
                  break;
                } else {
                  // set_target_temp(id(water_temp_aanvoer).state - 3.0); // old fixed hysteresis
                  set_target_temp(id(water_temp_aanvoer).state + (C->water_hyst_neg + 1.0));
                }
                break;
              case Defrosting:
              // To identify defrost has (recently) happened
                if ((timer - statechange) > (8*60)) { // 8 minutes
                  newstate = Running;
                }
                // no break, because the logic is still in Running
                __attribute__((fallthrough));
              case Running:
              // Main Running behaviors
                {
                  // correct the stooklijn target based on flow rate. I noticed that the outflow temperature target of the heatpump
                  // is the same irrespective of the amount of flow. This doesn't make sense because at half the flow, half the heat
                  // is transferred (this is not completely accurate, but close enough).

                  // TODO: Argh! I should make the step to control the actual physics instead of this indirect stuff! The only thing
                  // that really matters is the temperature of the floor surface. That determines the heat transfer into the room and
                  // compensates the heat loss out of the house. With the high heat capacity of my floor that could be done much better!

                  // concept it that we correct the delta between stooklijn target and returning water temp by the flow rate. My stooklijn
                  // is more or less accurate for the minimal flow rate of 17.5 lpm, so that's the baseline flow. (i.e. the stooklijn
                  // temperatures are correct at 17.5 lpm, e.g. at 35 lpm, the delta is halved.
                  double corrected_stooklijn = (id(stooklijn_target) - id(water_temp_retour).state) * (C->base_stooklijn_flow) / id(current_flow_rate).state + id(water_temp_retour).state;

                  double target = corrected_stooklijn + clamp((double)(id(thermostat_error).state * id(thermostat_error_gain).state), C->max_stooklijn_correction_neg, C->max_stooklijn_correction_pos);
                  bool minimum_run_time_passed = ((timer - compressortime) > (id(minimum_run_time).state*60));

                  if ((state == Running) && (requested_power < 0.05)) { //only do minimum power stuff when running (and especially not when defrosting!) and not if there's a power target
                    switch (mpastate) {
                      case mpaIdle:
                        if ((id(compressor_speed).state <= 18) && (min(id(temp18_filtered).state,id(temp20_filtered).state) > -0.0)) { //compressor running at minimum speed and evaporator is not freezing!
                          newmpastate = mpaInitializing;
                          mpa_compressorspeed = id(compressor_speed).state;
                          mpatarget = max(id(doel_temp).state + 1, id(water_temp_aanvoer).state+1); // let's start with a new target, one above the current target temperature or one above current aanvoer temp, whichever is highest
                        } else {
                          mpatarget = 0;
                        }
                        break;
                      case mpaInitializing:
                        if (id(compressor_speed).state != mpa_compressorspeed) { //compressor is responding
                          newmpastate = mpaStabilizing;
                        } else if ((timer - mpatimer) > 16*60) { //compressor is not responding! increase the target...
                          newmpastate = mpaReInitialize;
                          mpatarget += 1.0;
                        }
                        break;
                      case mpaReInitialize:
                        newmpastate = mpaInitializing;
                        break;
                      case mpaStabilizing:
                        if ((timer - mpatimer) > 45*60) // been stabilizing for 45 minutes, we are active!
                          newmpastate = mpaActive;

                        break;
                      case mpaActive:
                        if (id(compressor_speed).state > 35) { //we're overdoing it!
                          newmpastate = mpaInitializing;
                          mpa_compressorspeed = id(compressor_speed).state;
                          mpatarget -= 1;
                        } else if (min(id(temp18_filtered).state,id(temp20_filtered).state) < -1.0) { // evaporator is freezing let's reduce or should we just shut down?
                          newmpastate = mpaInitializing;
                          mpa_compressorspeed = id(compressor_speed).state;
                          mpatarget -= 1;
                        } else if (id(compressor_speed).state <= 18) { //we're not doing enough!
                          newmpastate = mpaInitializing;
                          mpa_compressorspeed = id(compressor_speed).state;
                          mpatarget += 1;
                        }
                        break;
                    }
                  }
                  if (target > mpatarget)
                    newmpastate = mpaIdle;
                  else
                    target = mpatarget;
                  
                  if (requested_power >= 0.05) { //Target power requested
                    /*
                    The water setpoint is calculated once per target change and then held, because the heatpump
                    takes about an hour to settle on a new setpoint. The energy state does the fast part: it
                    pushes the compressor toward the target and is set back to ES2 inside the deadband.
                    */
                    const float p = id(lg_total_active_power_filtered).state;
                    const float dtw = id(water_dt_corrected).state;
                    const float to = id(water_temp_aanvoer).state;
                    const float deadband = id(power_target_deadband).state;

                    if (!pt_active) { // power target active at start of Running, or just set: handle as a step
                      pt_active = true;
                      pt_es = isnan(id(energy_state).state) ? 2 : (int) id(energy_state).state;
                      pt_es_since = timer;
                    }
                    if (isnan(pt_target) || (fabsf(requested_power - pt_target) >= 100.0f)) {
                      pt_target = requested_power;
                      pt_calc_pending = true;
                      pt_step = true;
                    }

                    if (state == Running) { // during defrost the measurements are meaningless, keep everything as is
                      if (!isnan(id(floor_temp_est).state))
                        pt_floor_est = id(floor_temp_est).state;
                      if ((p > 300.0f) && (dtw > 1.0f))
                        pt_kin = p / dtw;

                      if (pt_calc_pending) {
                        const float sp = pt_feedforward(pt_target);
                        if (!isnan(sp)) {
                          pt_setpoint = sp;
                          pt_calc_pending = false;
                          pt_trim_time = timer;
                          pt_trim_to = to;
                          ESP_LOGD("power_target", "Target %.0f W at %.0f W: setpoint %.1f", pt_target, p, sp);
                        }
                      } else if (fabsf(to - pt_trim_to) >= 0.5f) {
                        // outlet still moving, the LG is not steady yet
                        pt_trim_time = timer;
                        pt_trim_to = to;
                      } else if ((pt_es == 2) && ((timer - pt_es_since) >= 15*60) && ((timer - pt_trim_time) >= 15*60)) {
                        // LG steady in ES2: move the setpoint at most 1 K toward the calculated value
                        const float sp = pt_feedforward(requested_power);
                        if (!isnan(sp) && !isnan(pt_setpoint)) {
                          pt_setpoint += clamp(sp - pt_setpoint, -1.0f, 1.0f);
                          ESP_LOGD("power_target", "Re-trim at %.0f W: calculated %.1f, setpoint %.1f", p, sp, pt_setpoint);
                        }
                        pt_trim_time = timer;
                        pt_trim_to = to;
                      }

                      // energy state from filtered power against the target
                      const float err = requested_power - p;
                      int es = 2;
                      if (err > deadband)
                        es = ((err > 800.0f) || (id(compressor_speed).state > 45)) ? 5 : 6; // ES6 does nothing above ~55
                      else if (err < -deadband)
                        es = 7; // ES8 stops the compressor, never use it
                      const int es_wanted = es;
                      if ((es != 2) && (pt_es != 2) && ((es == 7) != (pt_es == 7)))
                        es = 2; // reverse direction only through ES2
                      if ((es != pt_es) && ((pt_es != 2) || pt_step || ((timer - pt_es_since) >= 5*60))) {
                        // leaving a boost/reduce is immediate, starting one waits until ES2 had 5 minutes to settle
                        pt_es = es;
                        pt_es_since = timer;
                      }
                      if (pt_es == es_wanted)
                        pt_step = false; // a target step keeps skipping the settle time until it is acted on

                      if (pt_es == 5 || pt_es == 6)
                        pt_status = (id(compressor_speed).state >= 80) ? "Limited" : "Increasing";
                      else if (pt_es == 7)
                        pt_status = "Decreasing";
                      else
                        pt_status = pt_calc_pending ? "Waiting for model" : "Holding";
                      ESP_LOGD("power_target", "Target %.0f W, power %.0f W, ES%d, setpoint %.1f", requested_power, p, pt_es, pt_setpoint);
                    } else {
                      pt_status = "Defrost";
                    }
                    if (id(energy_state).state != pt_es)
                      set_energy_state(pt_es);

                    if (isnan(pt_setpoint))
                      pt_setpoint = target; // no usable model yet, start from the stooklijn
                    pt_setpoint = clamp((float) round(pt_setpoint), 20.0f, (float) id(power_target_max_wtemp).state);
                    target = pt_setpoint;
                  }
                  double delta = id(water_temp_aanvoer).state - target;

                  ESP_LOGD(state_string[state], "Delta: %f, Stooklijn: %f, corrected stooklijn: %f, target: %f", delta, id(stooklijn_target), corrected_stooklijn, target);
  
                  // includes ugly hack so unit is not turned off when stooklijn_target is still high enough to keep house on target with minimum power
                  if (((!id(thermostat_wp_heat).state) && minimum_run_time_passed ) ||
                      (id(force_run_end).state)) {
                    id(modbus_enable_heat).turn_off();
                    id(force_run_end).turn_off();
                    id(thermostat_wp_heat).turn_off();
                    set_target_temp(20); //hard set at absolute minimum water temp, so unit wont turn on based on its hysteresis
                    newstate = Stopping;
                    break;
                  }

                  if (id(defrosting).state) {
                    newstate = Defrosting;
                    // No break, because the logic while defrosting is just Running logic
                  }
  
                  if (delta > 0) { //if the temperature is overshooting, don't pull down by reducing the target. But never lower than hysteresis below actual...
                    //set_target_temp(max((id(water_temp_aanvoer).state - 3.0), (target - delta)));
                    //set_target_temp(max((id(water_temp_aanvoer).state - 3.0), (target)));// old fixed hysteresis
                    set_target_temp(max((id(water_temp_aanvoer).state + (C->water_hyst_neg + 1.0)), (target)));
                  } else {
                    set_target_temp(target);
                  }
                  //Publish new stooklijn value to watertemp value sensor
                  id(watertemp_target).publish_state(target);
  
                  //Silent mode logic.
                  const bool pt_no_silent = pt_active && (requested_power > id(power_target_silent_limit).state);
                  if (pt_no_silent || (id(lg_total_active_power).state > 1200) || (min(id(temp18_filtered).state,id(temp20_filtered).state) < -0.0)) {
                    // high power target -- silent mode limits the compressor
                    // high power -- efficiency gain is not significant, then COP is better with silent mode off
                    // temp20 -- if the evaporator is freezing, silent mode off helps
                    ESP_LOGD(state_string[state], "silent mode off: Power %f, temp20: %f", id(lg_total_active_power).state, id(temp20_filtered).state);
                    id(modbus_set_silent_mode).turn_off();
                  }
                  if (!pt_no_silent && (id(lg_total_active_power).state < 1000) && (max(id(temp18_filtered).state, id(temp20_filtered).state) > 0.0) && (state != Defrosting)) {
                    // low power -- efficiency gain is interesting, but only if the evaporator is not close to freezing
                    ESP_LOGD(state_string[state], "silent mode on: Power %f, temp20: %f", id(lg_total_active_power).state, id(temp20_filtered).state);
                    id(modbus_set_silent_mode).turn_on();
                  }
                }
                break;
              case Stopping:
              // We want the heatpump to shut down. heat/cool should be off, target temp at minimum (20C)
                if (id(energy_state).state != 2)
                  set_energy_state(2);
                if (!id(compressor_running).state) {
                  newstate = Afterrun;
                  id(modbus_enable_heat).turn_on();
                  break;
                }
                break;
              case Afterrun:
              // run the waterpump for a while, allow the heatpump to do that by heat/cool enabled.
                if ((timer - statechange) > (id(minimum_off_time).state*60) || (id(thermostat_wp_heat).state)) { //updated to use state minimum off time
                  newstate = Idle;
                  break;
                }
              default:
                break;
            }

            // if the state is updated, handle that.
            if (state != newstate) {
              state = newstate;
              statechange = timer;
            }

            // if the mpastate is updated, handle that.
            if (mpastate != newmpastate) {
              mpastate = newmpastate;
              mpatimer = timer;
            }
            ESP_LOGD(mpa_state_string[mpastate], "Since: %lus", timer - mpatimer);
            ESP_LOGD(state_string[state], "Since: %lus", timer - statechange);
            if (id(compressor_running).state) ESP_LOGD(state_string[state], "Compressor running for: %02lu:%02lu:%02lu", (timer - compressortime) / 3600, ((timer - compressortime) % 3600) / 60, (timer - compressortime) % 60);
            id(lg_controller_state).publish_state(state_string[state]);
            id(lg_mpa_controller_state).publish_state(mpa_state_string[mpastate]);
            id(power_target_state).publish_state(pt_status);

            return;
