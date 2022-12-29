#include <custom.h>

Customize::Customize() 
{
    water_hyst_pos = 4.0; //still to be replaced by LG modbus state if we find how to connect this
    water_hyst_neg = -4.0; //still to be replaced by LG modbus state if we find how to connect this
    max_stooklijn_correction_pos = 3.0;
    max_stooklijn_correction_neg = -5.0;
}


void Customize::custom_idle_behavior() {};
