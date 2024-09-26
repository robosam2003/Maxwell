//
// Created by robos on 20/09/2023.
//

#ifndef MAXWELL_DRV8323_H
#define MAXWELL_DRV8323_H

#include "Arduino.h"
#include "DRV8323_constants.h"
#include <SPI.h>

namespace DRV8323 {

class DRV8323 {
private:
    uint16_t _READ_WORD = 0x8000;
    uint16_t _WRITE_WORD = 0x0000;
    byte _CS;
    SPIClass _spi;
    SPISettings _settings;
    byte address_length = 4;
    byte word_length = 16;
    byte gate_en_pin;

    bool enabled = false;

public:
    // Constructor
    DRV8323(byte CS, SPIClass& spi, uint32_t spiFreq, byte gate_enable_pin);

    void enable(bool enable);

    uint16_t read_reg(REGISTER regAddress);  // SPI read

    void write_reg(REGISTER regAddress, uint16_t data);  // SPI write

    uint16_t get_fault_status_1();

    String get_fault_status_1_string();

    uint16_t get_fault_status_2();

    String get_fault_status_2_string();

    void set_pwm_mode(PWM_MODE mode);

    void set_SYNC_rectification(bool enable);

    void enable_CPUV_Fault(bool enable);

    void enable_GDF(bool enable);

    void clear_fault();


    void default_configuration();

};

} // namespace DRV8323

#endif //MAXWELL_DRV8323_H
