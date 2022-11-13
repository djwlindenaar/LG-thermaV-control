          lambda: !lambda |-
            //Main program logic this loop runs every 'clock' cycle when modbus data is received for this sensor
            static int alive_timer = 0;
            
            //Set controller control mode to 'outlet' and set hysteresis to the setting you have on the controller (recommend 4)
            const static int hysteresis = 4;
            //The algoritm will allow an overshoot (over the hysteresis) for the first 15 minutes to not kill an ongoing run that is about to start modulating
            //Rather settle at a slightly higher temperature than start 'pendeling'
            const static int max_overshoot = 4; //So maximum overshoot of hysteresis + max_overshoot
            static bool overshooting = false;
            //Set to value that anti-pendel script will track (outlet/inlet) (recommend outlet)
            const float tracking_value = id(water_temp_aanvoer).state;
            if(alive_timer == 0){
              //Post an 'alive' debug message in the logs every 300 seconds
              alive_timer = 16; //300 seconds
              ESP_LOGD("anti-pendel", "**alive** oat: %f inlet: %f outlet: %f ",id(buiten_temp).state,id(water_temp_retour).state,id(water_temp_aanvoer).state);
            }
            alive_timer--;
            //Wait for valid oat reading
            if(isnan(id(buiten_temp).state)) return;
            //Do not run script if working mode = 0
            ESP_LOGD("working_mode", "working_mode %d ",id(working_mode));
            if(id(working_mode) == 0) {
              auto call = id(water_temp_target_output).make_call();
              call.set_value(20.0);
              call.perform();
              //id(water_temp_target_output).publish_state(20.0);
              return;
            }
            
            //Hold previous script run ota value to check if it changed since last run
            static float prev_oat = 0;
            //Pump pre run time
            static int pre_run_time = 0;
            //System logic (to switch off agressive target setting after target is reached)
            static bool target_reached = false;
            
            float temp_new_target = id(stooklijn_target);
            //Calculate new (anti pendel) target
            //Very basic algorithm
            //System initializes at the start of a run (when working mode changes to 1 or 2)
            //Then sets initial target to 'real' target, to allow HP to start up (if water temp below HP hysteresis setting)
            //When the compressor turns on the logic starts running, setting a new target at 2 degrees below actual return temp when actual within hysteresis degrees of target
            //When the new target equals the target temperature the logic stops, fixating the target so the HP can take over again an run its own logic
            //System resets at heating mode stop (system working mode 0) for example when the room thermostat stops the heating
            if(id(run_start)){
              //Initialize values. run_start is set to true by the switches that control heating mode
              pre_run_time = 5; // equals 100 seconds. Based on 20s 'clock'
              //Activate system
              target_reached = false;
              temp_new_target = id(stooklijn_target);
              overshooting = false;
              ESP_LOGD("anti-pendel", "Run start initial values set; target: %f inlet: %f outlet: %f",id(stooklijn_target),id(water_temp_retour).state,id(water_temp_aanvoer).state);
              id(run_start) = 0;
            }
            
            //System will run with (very) agressive settings when delta return vs target is < 0.5 to prevent stabilising on too low temperature
            const float delta = id(stooklijn_target)-tracking_value;
            //Run when compressor is running and target not reached and actual return temp within 3 degrees of target
            if(id(compressor_running).state && !target_reached && delta < 0.5){
              //Agressive target setting
              temp_new_target = ((tracking_value-3)*10)/10;
              if(temp_new_target > id(stooklijn_target)) temp_new_target = id(stooklijn_target);
              ESP_LOGD("anti-pendel", "RETURN_TEMP: %f OUTLET_TEMP: %f TARGET: %f PENDEL_TARGET: %f TRACKING_VALUE: %f OVERSHOOTING: %d",id(water_temp_retour).state,id(water_temp_aanvoer).state,id(stooklijn_target),temp_new_target,tracking_value,overshooting);
            } else if(id(compressor_running).state && !target_reached) {
              //Waiting for delta te become within range, less agressive target setting
              temp_new_target = round((tracking_value-1)*10)/10;
              if(temp_new_target > id(stooklijn_target)) temp_new_target = id(stooklijn_target);
              ESP_LOGD("anti-pendel", "Waiting for delta RETURN_TEMP: %f OUTLET_TEMP: %f DELTA: %f PENDEL_TARGET: %f",id(water_temp_retour).state,id(water_temp_aanvoer).state,delta,temp_new_target);
            } else if(!target_reached && pre_run_time > 0) {
              //Make sure target remains at stooklijn target to ensure compressor start (if ok according to WP own logic)
              temp_new_target = id(stooklijn_target);
              //Update pre_run_time counter
              ESP_LOGD("anti-pendel", "Waiting for pre_run_time: %i",pre_run_time);
              pre_run_time--;
            }
            
            //Make sure changes in stooklijn are processed when target reached
            if(target_reached){
              //Do not set a lower target if it kills the run
              if(!(tracking_value > id(stooklijn_target) + hysteresis)) temp_new_target = id(stooklijn_target);
              temp_new_target = id(stooklijn_target);
            }
            if(delta < 0 && id(compressor_running).state && !target_reached){
              //Deactive agressive target setting until next run
              temp_new_target = id(stooklijn_target);
              target_reached = true;
              ESP_LOGD("anti-pendel", "Target reached, maintaining target: %f pendel_target: %f",id(stooklijn_target),id(pendel_watertemp_target).state);
            }
            //Allow extra hysteresis during first 15 minutes of heating run
            //Once overshooting = true the extra overshoot is allowed for the remainder of the run
            if((target_reached && !id(modulation_started) && id(working_mode) == 2)||overshooting){
              if(tracking_value > temp_new_target + hysteresis){
                if(tracking_value < (temp_new_target + hysteresis + max_overshoot)){
                  //Overshoot as needed (as it is below max overshoot)
                  temp_new_target = (tracking_value - hysteresis + 0.5);
                } else {
                  //Settle on max overshoot
                  temp_new_target = temp_new_target + max_overshoot;
                }
                overshooting = true;
                ESP_LOGD("anti-pendel", "Overshooting: target: %f pendel_target: %f tracking_value: %f",id(stooklijn_target),temp_new_target,tracking_value);
              }
            } 
            if(temp_new_target != id(var_pendel_watertemp_target) || id(var_pendel_watertemp_target) != id(water_temp_target_output).state){
              //Set new target
              id(var_pendel_watertemp_target) = temp_new_target;
              //Update sensor
              id(pendel_watertemp_target).publish_state(id(var_pendel_watertemp_target));
              //Update target through modbus (set level needs value between 0-1, so divide by 100)
              //id(water_temp_target_output).set_level(id(var_pendel_watertemp_target)*0.01);
              //id(water_temp_target_output).publish_state(id(var_pendel_watertemp_target));
              auto call = id(water_temp_target_output).make_call();
              call.set_value(round(id(var_pendel_watertemp_target)));
              call.perform();
              ESP_LOGD("anti-pendel", "New pendel_watertemp_target set: %f outlet: %f",id(pendel_watertemp_target).state,id(water_temp_aanvoer).state);
            }
