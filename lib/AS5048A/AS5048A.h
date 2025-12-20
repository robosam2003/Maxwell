//
// Created by robos on 20/12/2025.
//

#ifndef MAXWELL_AS5048A_H
#define MAXWELL_AS5048A_H

#include "PositionSensor.h"
#include <SPI.h>
#include "../../include/config.h"


class AS5048A : public PositionSensor {
private:
    uint8_t _CS;
    SPIClass _spi;
    SPISettings _settings;

    const uint8_t READ_BYTE = 0x40;
    const uint8_t WRITE_BYTE = 0x00;

    enum REGISTER : uint16_t {
        CLEAR_ERROR = 0x0001,
        PROGRAMMING_CONTROL = 0x0003,
        ZEROMSB = 0x0016,
        ZEROLSB = 0x0017,
        AGC = 0x3FFD,
        MAG = 0x3FFE,
        ANGLE = 0x3FFF
    };

    uint32_t prev_time_us = 0; // Timestamp of the previous angle reading

    uint16_t read_reg(REGISTER regAddress);
    void write_reg(REGISTER regAddress, uint16_t data);

public:
    AS5048A(byte CS, SPIClass& spi, uint32_t spiFreq);
    void update() override;
    float get_angle() override; // in radians
    float get_velocity() override; // in radians per second
    void set_offset(float angle);
};


#endif //MAXWELL_AS5048A_H