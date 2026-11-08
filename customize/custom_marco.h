
Customize::Customize() 
{
    water_hyst_pos = 4.0; //still to be replaced by LG modbus state if we find how to connect this
    water_hyst_neg = -4.0; //still to be replaced by LG modbus state if we find how to connect this
    max_stooklijn_correction_pos = 3.0;
    max_stooklijn_correction_neg = -5.0;
    base_stooklijn_flow = 30.0;
    // power target model (Daniel values, Marco has no power target)
    cop_base_w = 346.7;
    cop_q = 0.0300;
    cop_lift = 0.00869;
    cop_silent_w = -231.4;
    lg_cmp_a = 15.6;
    lg_cmp_b = 1.54;
}


void Customize::custom_idle_behavior() {};
