
Customize::Customize() 
{
    water_hyst_pos = 2.0; //still to be replaced by LG modbus state if we find how to connect this
    water_hyst_neg = -4.0; //still to be replaced by LG modbus state if we find how to connect this
    max_stooklijn_correction_pos = 4.0;
    max_stooklijn_correction_neg = -10.0;
    base_stooklijn_flow = 17.5;
}

void Customize::custom_idle_behavior()
{
  // when Idle, we run the pump to circulate warm water, if that's useful
  if ((id(huiskamer_vloer).state<20.0) || // vloer is koud, rondpompen heeft weinig zin
      (id(huiskamer_lucht).state>id(huiskamer_vloer).state)) { //lucht is warmer dan de vloer, rondpompen heeft weinig zin
    ESP_LOGD("modbus_enable_heat", "Turned controller off: %f %f", id(huiskamer_vloer).state, id(huiskamer_lucht).state);
    id(modbus_enable_heat).turn_off();
  } else {
    ESP_LOGD("modbus_enable_heat", "Turned controller on: %f %f", id(huiskamer_vloer).state, id(huiskamer_lucht).state);
    id(modbus_enable_heat).turn_on();
  }
}
