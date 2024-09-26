//
// Created by robos on 10/07/2024.
//
#include "Arduino.h"
#include <SPI.h>
#include "AS5047P_constants.h"

#ifndef MAXWELL_AS5047P_H
#define MAXWELL_AS5047P_H

namespace AS5047P {

class AS5047P {
private:
    byte _CS;
    byte READ_BYTE = 0b01000000;
    byte WRITE_BYTE = 0b00000000;
    SPIClass _spi;
    SPISettings _settings;

public:
    AS5047P(byte CS, SPIClass& spi, uint32_t spiFreq);

    uint16_t read_reg(REGISTER regAddress);  // SPI read

    void write_reg(REGISTER regAddress, uint16_t data);  // SPI write

    uint16_t get_angle(bool comp = true);

    uint16_t get_mag_strength();










};


#endif //MAXWELL_AS5047P_H

} // namespace AS5047P