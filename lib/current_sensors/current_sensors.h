//
// Created by robosam on 2/3/25.
//

#include "Arduino.h"
#include "../RCFilter/RCFilter.h"
#include "../../include/config.h"
#include "../../include/pin_definitions.h"
#include "DRV8323_constants.h"

#ifndef CURRENT_SENSORS_H
#define CURRENT_SENSORS_H

class CurrentSensors {
private:
    double R_SENSE = 0.004;
	float ANOMALY_THRESHOLD = 300;
    uint32_t _pin_a;
    uint32_t _pin_b;
    uint32_t _pin_c;
    double _offset_a;  // in AMPS
    double _offset_b;
    double _offset_c;

    RCFilter* _filter_a;
    RCFilter* _filter_b;
    RCFilter* _filter_c;
    DRV8323::CSA_GAIN _csa_gain;
	ADC_HandleTypeDef* hadc1;


public:
	double v_a;
	double v_b;
	double v_c;
    double _current_a;
    double _current_b;
    double _current_c;
	bool filtered = false;
	bool inverted = true;

    explicit CurrentSensors(uint32_t pin_a,
                          uint32_t pin_b,
                          uint32_t pin_c,
                          DRV8323::CSA_GAIN gain);

    void setup_injected_adc();

    void set_csa_gain(DRV8323::CSA_GAIN gain);

    void calibrate_offsets();

	void read();

    double get_current_a();
    double get_current_b();
    double get_current_c();
    double* get_currents();
    double get_total_current();
};

#endif //CURRENT_SENSORS_H
