          lambda: !lambda |-
            static const char*state_string[] = {"Idle", "Starting", "EarlyRun", "Running", "Defrosting", "Stopping", "Afterrun","Cooling"};
            enum States {Idle ,  Starting ,  EarlyRun ,  Running ,  Defrosting ,  Stopping ,  Afterrun, Cooling};
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
            static uint32_t mpatarget = 0;
            static uint32_t statechange = 0; //timer value upon previous state change
            static uint32_t compressortime = 0; //timer value on last compressor start
            static uint32_t dt = round(id(state_machine).get_update_interval()/1000);
            auto water_temp_call = id(water_temp_target_output).make_call();
            #define set_target_temp(x) if (abs(x-id(water_temp_target_output).state) > 0.7) {water_temp_call.set_value(round(x));water_temp_call.perform();ESP_LOGD("set_target_temp", "target set to: %f", round(x));}

            timer += dt;
            if (timer < 60) { // we just got booted, let's take some time to figure out what's happening before taking action
              if (id(compressor_running).state) {
                newstate = state = Running;
              } else {
                if (id(water_temp_target_output).state > 20) // turn on if stooklijn asks more then absolute water minimum
                  newstate = state = Starting;
                else
                  newstate = state = Idle;
              }

              ESP_LOGD(state_string[state], "Since: %ds", timer - statechange);
              return;
            }

            if (id(operation_mode).state == 0.0)
              newstate = state = Cooling;


            switch (state) {
              case Cooling:
                //don't know yet what to do
                if (id(operation_mode).state == 4.0)
                  newstate = Idle;

                break;
              case Idle:
              //Nothing is going on, keep checking if something should be going on
                if (id(thermostat_wp_heat).state) {
                  newstate = Starting;
                  id(force_run_end).turn_off();
                  id(modbus_enable_heat).turn_on();
                  break;
                }
                C->custom_idle_behavior();
                break;
              case Starting:
              //we want the compressor to start...
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
                  // no break, because the logic is still in Running
                }
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

                  if (state == Running) { //only do minimum power stuff when running (and especially not when defrosting!)
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




                  double delta = id(water_temp_aanvoer).state - target;

                  ESP_LOGD(state_string[state], "Delta: %f, Stooklijn: %f, corrected stooklijn: %f, target: %f", delta, id(stooklijn_target), corrected_stooklijn, target);
  
                  // includes ugly hack so unit is not turned off when stooklijn_target is still high enough to keep house on target with minimum power
                  if (((!id(thermostat_wp_heat).state) && minimum_run_time_passed && (id(stooklijn_target) < 26.0)) ||
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
                  if ((id(lg_total_active_power).state > 1200) || (min(id(temp18_filtered).state,id(temp20_filtered).state) < -0.0)) {
                    // high power -- efficiency gain is not significant, then COP is better with silent mode off
                    // temp20 -- if the evaporator is freezing, silent mode off helps
                    ESP_LOGD(state_string[state], "silent mode off: Power %f, temp20: %f", id(lg_total_active_power).state, id(temp20_filtered).state);
                    id(modbus_set_silent_mode).turn_off();
                  }
                  if ((id(lg_total_active_power).state < 1000) && (max(id(temp18_filtered).state, id(temp20_filtered).state) > 0.0) && (state != Defrosting)) {
                    // low power -- efficiency gain is interesting, but only if the evaporator is not close to freezing
                    ESP_LOGD(state_string[state], "silent mode on: Power %f, temp20: %f", id(lg_total_active_power).state, id(temp20_filtered).state);
                    id(modbus_set_silent_mode).turn_on();
                  }
                }
                break;
              case Stopping:
              // We want the heatpump to shut down. heat/cool should be off, target temp at minimum (20C)
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
            ESP_LOGD(mpa_state_string[mpastate], "Since: %ds", timer - mpatimer);
            ESP_LOGD(state_string[state], "Since: %ds", timer - statechange);
            if (id(compressor_running).state) ESP_LOGD(state_string[state], "Compressor running for: %02u:%02u:%02u", (timer - compressortime) / 3600, ((timer - compressortime) % 3600) / 60, (timer - compressortime) % 60);
            id(lg_controller_state).publish_state(state_string[state]);
            id(lg_mpa_controller_state).publish_state(mpa_state_string[mpastate]);

            return;

