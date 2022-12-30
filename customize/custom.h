// this file contains a class to define customizations

class Customize {
  public:
    Customize();
    
    float water_hyst_pos; //still to be replaced by LG modbus state if we find how to connect this
    float water_hyst_neg; //still to be replaced by LG modbus state if we find how to connect this
    double max_stooklijn_correction_pos;
    double max_stooklijn_correction_neg;

    void custom_idle_behavior(); //custom behavior during Idle e.g. enable/disable the pump


};

Customize *C;
