          lambda: !lambda |-
            static const char*state_string[] =
                       {"Idle", "Starting", "EarlyRun", "Running", "Stopping"};
            enum States {Idle ,  Starting ,  EarlyRun ,  Running ,  Stopping};
            static States state = Idle;
            auto water_temp_call = id(water_temp_target_output).make_call();
            
            switch (state) {
              case Idle:
                if (id(thermostat_wp_heat).state) {
                  state = Starting;
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

