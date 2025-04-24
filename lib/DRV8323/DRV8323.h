//
// Created by robos on 20/09/2023.
//

#ifndef MAXWELL_DRV8323_H
#define MAXWELL_DRV8323_H

#include "Arduino.h"
#include "DRV8323_constants.h"
#include <SPI.h>
#include "../../include/pin_definitions.h"
#include "../../include/config.h"
#include "current_sensors.h"
#include "HardwareTimer.h"




namespace DRV8323 {

struct PWM_3x_Struct {
    uint8_t PIN_A;
    uint8_t PIN_B;
    uint8_t PIN_C;
};



class DRV8323 {
public:
    uint16_t _READ_WORD = 0x8000;
    uint16_t _WRITE_WORD = 0x0000;
    byte _CS;
    SPIClass _spi;
    SPISettings _settings;
    byte address_length = 4;
    byte word_length = 16;
    byte gate_en_pin;

    // PWM_3x_Struct pwm_3_x = {DRV8323_HI_A_PIN,
    //                          DRV8323_HI_B_PIN,
    //                          DRV8323_HI_C_PIN};

    bool enabled = false;

    CSA_GAIN _csa_gain = CSA_GAIN::GAIN_40_V_V;
    CurrentSensors* current_sensors;
    // Constructor
    DRV8323(byte CS, SPIClass& spi, uint32_t spiFreq, uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, byte gate_enable_pin);

    void enable(bool enable);

    void setup_pwm();

    // Helper functions
    uint16_t read_reg(REGISTER regAddress);  // SPI read
    void write_reg(REGISTER regAddress, uint16_t data);  // SPI write

    // Fault status
    uint16_t get_fault_status_1();
    String get_fault_status_1_string();
    uint16_t get_fault_status_2();
    String get_fault_status_2_string();

    // Driver mode (6x, 3x, 1x, independent)
    void set_pwm_mode(PWM_MODE mode);

    void set_SYNC_rectification(bool enable);

    void set_current_gain(CSA_GAIN gain);

    void set_gate_drive_source_current(IDRIVE_P_CURRENT current);

    void set_gate_drive_sink_current(IDRIVE_N_CURRENT current);

    void set_peak_gate_drive_time(TDRIVE_TIME time);

    void set_dead_time(DEAD_TIMES time);

    void perform_current_sense_calibration();

    void enable_CPUV_Fault(bool enable);

    void enable_GDF(bool enable);

    void clear_fault();

    void default_configuration();
};

} // namespace DRV8323

#endif //MAXWELL_DRV8323_H
