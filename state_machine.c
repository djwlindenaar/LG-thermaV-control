          lambda: !lambda |-
            static const char*state_string[] =
                       {"Idle", "Starting", "EarlyRun", "Running", "Stopping", "Afterrun"};
            enum States {Idle ,  Starting ,  EarlyRun ,  Running ,  Stopping ,  Afterrun};
            static States state = Idle;
            auto water_temp_call = id(water_temp_target_output).make_call();
            
            switch (state) {
              case Idle:
                if (id(thermostat_wp_heat).state) {
                  state = Starting;
                  id(modbus_enable_heat).turn_on();
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
                if (id(compressor_running).state) {
                  state = EarlyRun;
                } else {
                  // set temperature high enough so compressor will start
                  water_temp_call.set_value(id(water_temp_aanvoer).state+3);
                  water_temp_call.perform();
                }
                break;
              default:
                break;
            }


            return;

