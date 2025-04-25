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
    public:

        byte _CS;
        byte READ_BYTE = 0b01000000;
        byte WRITE_BYTE = 0b00000000;
        SPIClass _spi;
        SPISettings _settings;
        DIRECTION _direction;
        float prev_rel_angle = 0.0; // Previous relative angle (0 to 2*pi)
        long full_rotations = 0; // Number of full rotations
        float absolute_angle = 0.0; // Absolute angle factoring in full rotations (radians)

        uint32_t prev_time_us = 0; // Timestamp of the previous angle reading
        float velocity = 0.0; // Angular velocity (radians/s)
        bool comp = true;

        AS5047P(byte CS, SPIClass& spi, uint32_t spiFreq);

        uint16_t read_reg(REGISTER regAddress);  // SPI read

        void write_reg(REGISTER regAddress, uint16_t data);  // SPI write

        void update();

        float get_angle();

        float get_velocity();


        uint16_t get_mag_strength();

        void set_direction(DIRECTION direction);

    };

} // namespace AS5047P

#endif //MAXWELL_AS5047P_H
