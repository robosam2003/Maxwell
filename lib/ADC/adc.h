//
// Created by robosam on 2/3/25.
//

#include "Arduino.h"
#include "stm32f4xx_hal_adc.h"
#include "../RCFilter/RCFilter.h"
#include "../../include/config.h"
#include "../../include/pin_definitions.h"
#include "../../src/maxwell_utils.h"
#include "DRV8323_constants.h"

#ifndef CURRENT_SENSORS_H
#define CURRENT_SENSORS_H

class Adc {
private:

	// Current Sensing
    double R_SENSE = 5e-4; // 0.5 milliohms
	float ANOMALY_THRESHOLD = 300;
    uint32_t _curr_pin_a;
    uint32_t _curr_pin_b;
    uint32_t _curr_pin_c;
    double _curr_offset_a;  // in AMPS
    double _curr_offset_b;
    double _curr_offset_c;

    RCFilter* _curr_filter_a;
    RCFilter* _curr_filter_b;
    RCFilter* _curr_filter_c;

    int _csa_gain = PHASE_CSA_GAIN;

	// Voltage Sensing
	uint32_t _volt_pin_a;
	uint32_t _volt_pin_b;
	uint32_t _volt_pin_c;

	RCFilter* _volt_filter_a;
	RCFilter* _volt_filter_b;
	RCFilter* _volt_filter_c;

	uint32_t _supply_sense_pin;



	ADC_HandleTypeDef* hadc1;
	ADC_HandleTypeDef* hadc2;
	ADC_HandleTypeDef* hadc3;


public:
    float _current_a;
    float _current_b;
    float _current_c;
	float _voltage_a;
	float _voltage_b;
	float _voltage_c;

	bool volt_filtered = false;
	bool curr_filtered = false; // ENSURE THIS IS OFF FOR STABLE D AND Q CONTROL!
	bool curr_inverted = true;

    explicit Adc(uint32_t curr_pin_a,
                  uint32_t curr_pin_b,
                  uint32_t curr_pin_c,
                  uint32_t volt_pin_a,
                  uint32_t volt_pin_b,
                  uint32_t volt_pin_c,
                  uint32_t supply_sense_pin);

    void setup_injected_adc();

    void calibrate_current_offsets();

	void read();

	PhaseVoltages get_phase_voltages();

	PhaseCurrents get_phase_currents();

	float get_supply_voltage();

	float get_bemf_angle();
};

#endif //CURRENT_SENSORS_H
