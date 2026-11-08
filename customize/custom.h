// this file contains a class to define customizations

class Customize {
  public:
    Customize();
    
    float water_hyst_pos; //still to be replaced by LG modbus state if we find how to connect this
    float water_hyst_neg; //still to be replaced by LG modbus state if we find how to connect this
    double max_stooklijn_correction_pos;
    double max_stooklijn_correction_neg;
    float base_stooklijn_flow;

    // power target model, see DESIGN.md "Power target control"
    // electrical power P = cop_base_w + Q * (cop_q + cop_lift * (outlet - outdoor)) + cop_silent_w * silent
    float cop_base_w;
    float cop_q;
    float cop_lift;
    float cop_silent_w;
    // LG compressor speed it aims for: lg_cmp_a + lg_cmp_b * (setpoint - outdoor)
    float lg_cmp_a;
    float lg_cmp_b;

    void custom_idle_behavior(); //custom behavior during Idle e.g. enable/disable the pump


};

Customize *C;
