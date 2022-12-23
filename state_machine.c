          lambda: !lambda |-
            static const char*state_string[] =
                       {"Idle", "Starting", "EarlyRun", "Running", "Defrosting", "Stopping", "Afterrun"};
            enum States {Idle ,  Starting ,  EarlyRun ,  Running ,  Defrosting ,  Stopping ,  Afterrun};
            static States state = Idle;
            static States newstate = Idle;
            static float water_hyst_pos = 4.0; //still to be replaced by LG modbus state if we find how to connect this
            static float water_hyst_neg = -4.0; //still to be replaced by LG modbus state if we find how to connect this
            static uint32_t timer = 0;
            static uint32_t statechange = 0; //timer value upon previous state change
            static uint32_t compressortime = 0; //timer value on last compressor start
            static uint32_t dt = round(id(state_machine).get_update_interval()/1000);
            auto water_temp_call = id(water_temp_target_output).make_call();
            #define set_target_temp(x) water_temp_call.set_value(round(x));water_temp_call.perform();ESP_LOGD("set_target_temp", "target set to: %f", round(x));

            timer += dt;
            if (timer < 60) { // we just got booted, let's take some time to figure out what's happening before taking action
              if (id(compressor_running).state) {
                newstate = state = Running;
              } else {
                if (id(water_temp_target_output).state > 20)
                  newstate = state = Starting;
                else
                  newstate = state = Idle;
              }

              ESP_LOGD(state_string[state], "Since: %ds", timer - statechange);
              return;
            }

            switch (state) {
              case Idle:
              //Nothing is going on, keep checking if something should be going on
                if (id(thermostat_wp_heat).state) {
                  newstate = Starting;
                  id(modbus_enable_heat).turn_on();
                  break;
                }
                // when Idle, we run the pump to circulate warm water, if that's useful
                if ((id(huiskamer_vloer).state<20.0) || // vloer is koud, rondpompen heeft weinig zin
                    (id(huiskamer_lucht).state>id(huiskamer_vloer).state)) { //lucht is warmer dan de vloer, rondpompen heeft weinig zin
                  ESP_LOGD("modbus_enable_heat", "Turned controller off: %f %f", id(huiskamer_vloer).state, id(huiskamer_lucht).state);
                  id(modbus_enable_heat).turn_off();
                } else {
                  ESP_LOGD("modbus_enable_heat", "Turned controller on: %f %f", id(huiskamer_vloer).state, id(huiskamer_lucht).state);
                  id(modbus_enable_heat).turn_on();
                }
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
                set_target_temp(id(water_temp_aanvoer).state + (water_hyst_pos - 1.0));
                break;
              case EarlyRun:
              // Heatpump operates along a predefined behavior. Just wait until it's done and keep the target as low as possible to limit overshoot
                //id(modbus_set_silent_mode).turn_off(); //silent mode off during start (is this right? Might cause bigger startup overshoot
                id(modbus_set_silent_mode).turn_on(); // turn silent mode on at start, to get less overshoot
                if ((timer - statechange) > (15*60)) { // EarlyRun takes at most 15 minutes
                  newstate = Running;
                  break;
                } else {
                  // set_target_temp(id(water_temp_aanvoer).state - 3.0); // old fixed hysteresis
                  set_target_temp(id(water_temp_aanvoer).state + (water_hyst_neg + 1.0));
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

                  // concept it that we correct the delta between stooklijn target and room temp target by the flow rate. My stooklijn
                  // is more or less accurate for the minimal flow rate of 17.5 lpm, so that's the baseline flow. (i.e. the stooklijn
                  // temperatures are correct at 17.5 lpm, e.g. at 35 lpm, the delta is halved.
                  double corrected_stooklijn = (id(stooklijn_target) - id(huiskamer_thermostaat_target).state) * 17.5 / id(current_flow_rate).state + id(huiskamer_thermostaat_target).state;

                  double target = corrected_stooklijn + clamp((double)(id(thermostat_error).state * id(thermostat_error_gain).state), -10.0, 4.0);
                  double delta = id(water_temp_aanvoer).state - target;
                  bool minimum_run_time_passed = ((timer - compressortime) > (id(minimum_run_time).state*60));
  
                  ESP_LOGD(state_string[state], "Delta: %f, Stooklijn: %f, corrected stooklijn: %f, target: %f", delta, id(stooklijn_target), corrected_stooklijn, target);
  
                  if ((!id(thermostat_wp_heat).state) && minimum_run_time_passed) {
                    id(modbus_enable_heat).turn_off();
                    set_target_temp(20.0);
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
                    set_target_temp(max((id(water_temp_aanvoer).state + (water_hyst_neg + 1.0)), (target)));
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
                if ((timer - statechange) > (30*60) || (id(thermostat_wp_heat).state)) {
                  newstate = Idle;
                  break;
                }
              default:
                break;
            }

            // if the state is updated, handle that.
            if (state != newstate) {
              state = newstate;
              ESP_LOGD("new state is %f", newstate)
              statechange = timer;
            }

            ESP_LOGD(state_string[state], "Since: %ds", timer - statechange);
            if (id(compressor_running).state) ESP_LOGD(state_string[state], "Compressor running for: %ds", timer - compressortime);
            id(lg_controller_state).publish_state(state_string[state]);

            return;

