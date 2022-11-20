          lambda: !lambda |-
            static const char*state_string[] =
                       {"Idle", "Starting", "EarlyRun", "Running", "Stopping", "Afterrun"};
            enum States {Idle ,  Starting ,  EarlyRun ,  Running ,  Stopping ,  Afterrun};
            static States state = Idle;
            static States newstate = Idle;
            static uint32_t timer = 0;
            static uint32_t statechange = 0; //timer value upon previous state change
            static uint32_t compressortime = 0; //timer value on last compressor start
            static uint32_t dt = round(id(state_machine).get_update_interval()/1000);
            auto water_temp_call = id(water_temp_target_output).make_call();
            #define set_target_temp(x) water_temp_call.set_value(round(x));water_temp_call.perform();ESP_LOGD("set_target_temp", "target set to: %f", round(x));

            timer += dt;
            if (timer < 60) { // we just got booted, let's take some time to figure out what's happening before taking action
              if (id(compressor_running).state) {
                state = Running;
              } else {
                if (id(water_temp_target_output).state > 20)
                  state = Starting;
                else
                  state = Idle;
              }
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
                set_target_temp(id(water_temp_aanvoer).state + 3.0);
                break;

              case EarlyRun:
                if ((timer - statechange) > (15*60)) { // EarlyRun takes at most 15 minutes
                  newstate = Running;
                  break;
                } else {
                  set_target_temp(id(water_temp_aanvoer).state - 3.0);
                }
                break;

              case Running:
              {
                float delta = id(water_temp_aanvoer).state - id(stooklijn_target);
                bool minimum_run_time_passed = ((timer - compressortime) > (id(minimum_run_time).state*60));

                ESP_LOGD(state_string[state], "Delta: %f", delta);

                if ((!id(thermostat_wp_heat).state) && minimum_run_time_passed) {
                  id(modbus_enable_heat).turn_off();
                  set_target_temp(20.0);
                  newstate = Stopping;
                  break;
                }

                if (delta > 0) { //if the temperature is overshooting, pull down by reducing the target. But never lower than hysteresis below actual...
                  set_target_temp(max(float(id(water_temp_aanvoer).state - 3.0), float(id(stooklijn_target) - delta)));
                } else {
                  set_target_temp(id(stooklijn_target));
                }
              }
              break;
              case Stopping:
                if (!id(compressor_running).state) {
                  newstate = Afterrun;
                  id(modbus_enable_heat).turn_on();
                  break;
                }
                break;
              case Afterrun:
                if ((timer - statechange) > (30*60) || (id(thermostat_wp_heat).state)) {
                  newstate = Idle;
                  break;
                }

              default:
                break;
            }

            if (state != newstate) {
              state = newstate;
              statechange = timer;
            }

            ESP_LOGD(state_string[state], "Since: %ds", timer - statechange);
            if (id(compressor_running).state) ESP_LOGD(state_string[state], "Compressor running for: %ds", timer - compressortime);
            id(lg_controller_state).publish_state(state_string[state]);

            return;

