//
// Created by SamScott on 20/03/2026.
//

#ifndef MAXWELL_VOLTAGESENSORS_H
#define MAXWELL_VOLTAGESENSORS_H


#include "Arduino.h"
#include "../RCFilter/RCFilter.h"
#include "../../include/pin_definitions.h"



class VoltageSensors {
private:
    uint32_t _pin_a;
    uint32_t _pin_b;
    uint32_t _pin_c;
    uint32_t _pin_supply;

    // Divider correction factors: V_real = V_adc * divider_gain
    // e.g. for Rtop=47k, Rbot=2.2k => divider_gain = (47k+2.2k)/2.2k
    float _divider_gain_a;
    float _divider_gain_b;
    float _divider_gain_c;
    float _divider_gain_supply;

    float _vref;
    float _adc_lsb;

    RCFilter* _filter_a;
    RCFilter* _filter_b;
    RCFilter* _filter_c;
    RCFilter* _filter_supply;

    ADC_HandleTypeDef* hadc1;
    ADC_HandleTypeDef* hadc2;
    ADC_HandleTypeDef* hadc3;

public:
    bool filtered = false;

    float phase_a_v = 0.0f;
    float phase_b_v = 0.0f;
    float phase_c_v = 0.0f;
    float supply_v  = 0.0f;

    explicit VoltageSensors(
        uint32_t pin_a,
        uint32_t pin_b,
        uint32_t pin_c,
        uint32_t pin_supply,
        float divider_gain_a,
        float divider_gain_b,
        float divider_gain_c,
        float divider_gain_supply,
        float vref = 3.3f
    );

    void setup_injected_adc();
    void read_phase_voltages();
    void read_supply_voltage();

    float get_phase_a_voltage() const;
    float get_phase_b_voltage() const;
    float get_phase_c_voltage() const;
    float get_supply_voltage() const;
};



#endif //MAXWELL_VOLTAGESENSORS_H