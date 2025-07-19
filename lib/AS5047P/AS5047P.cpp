//
// Created by robos on 10/07/2024.
//

#include "AS5047P.h"

#include "../../include/config.h"


namespace AS5047P {


    AS5047P::AS5047P(byte CS, SPIClass &spi, uint32_t spiFreq) {
        _CS = CS;
        _spi = spi;
        // Set up SPI settings - SPI MODE 1 because data is captured on the falling edge of the clock
        // and propagated on the rising edge - https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
        _settings = SPISettings(spiFreq, MSBFIRST, SPI_MODE1);
        _direction = CCW; // Default - may be changed by the init sequence.
        // Set up CS pin
        pinMode(_CS, OUTPUT);
        digitalWrite(_CS, HIGH);
        // Begin the SPI bus.
        _spi.begin();

        // update();
        // float _offset = get_angle(); // Set the offset to the current angle
        // set_offset(_offset);

    }

    uint16_t AS5047P::read_reg(REGISTER regAddress) {
        uint16_t result = 0;
        uint16_t word = (READ_BYTE << 8) | regAddress; // add read bit
        // Add parity bit using xor
        uint16_t parityBit = 0;
        for (int i = 0; i < 15; i++) {
            parityBit ^= (word >> i) & 0x1;
        }
        // first bit is parity bit
        word |= parityBit << 15;
        _spi.beginTransaction(_settings); // Begin the SPI transaction
        digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
        result = _spi.transfer16(word); // Send the address byte
        digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
        _spi.endTransaction(); // End the SPI transaction
        return result;
    }

    void AS5047P::write_reg(REGISTER regAddress, uint16_t data) {
        uint16_t word = (WRITE_BYTE << 8) | regAddress; // add write bit
        // Add parity bit using xor
        uint16_t parityBit = 0;
        for (int i = 0; i < 15; i++) {
            parityBit ^= (word >> i) & 0x1;
        }
        // first bit is parity bit
        word |= parityBit << 15;
        _spi.beginTransaction(_settings); // Begin the SPI transaction
        digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
        _spi.transfer16(word); // Send the address byte
        _spi.transfer16(data); // Send the data byte
        digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
        _spi.endTransaction(); // End the SPI transaction
    }

    void AS5047P::update() {
        prev_absolute_angle = absolute_angle; // Store the angle from previous update

        uint16_t angle = 0;
        if (comp) {
            angle = read_reg(REGISTER::ANGLECOM);
        }
        else {
            angle = read_reg(REGISTER::ANGLEUNCOMP);
        }
        angle = angle & 0b0011111111111111; // Mask the 2 MSB bits
        float angle_raw_val = (static_cast<float>(angle) / 16384.0) * _2PI;
        float d_angle = angle_raw_val - prev_raw_angle;
        if (abs(d_angle) > 0.5f*_2PI) { // This relies on the update() method being called frequently enough
            full_rotations += (d_angle > 0) ? -1 : 1;
        }
        prev_raw_angle = angle_raw_val;
        absolute_angle = (static_cast<float>(full_rotations) * _2PI) + prev_raw_angle;

        // Velocity calculation
    }

    float AS5047P::get_angle() {
        // update();
        if (_direction == CCW) {
            return -absolute_angle + offset; // Return the absolute angle in radians, adjusted for direction and offset
        }
        return absolute_angle + offset; // Return the absolute angle in radians, adjusted for direction and offset
    }

    float AS5047P::get_velocity() {
        // update();
        uint32_t current_time = micros();
        float Ts = (current_time - prev_time_us) * 1e-6f; // Convert microseconds to seconds
        if (Ts < 100e-6) { // 100 microseconds
            return velocity;
        }
        velocity = (absolute_angle - prev_absolute_angle) / ((current_time - prev_time_us) * 1e-6f); // rad/s
        if (_direction == CCW) {
            velocity = -velocity; // Adjust for direction
        }
        prev_time_us = current_time;
        return velocity;
    }

    float AS5047P::set_offset(float angle) {
        offset = angle;
    }

    uint16_t AS5047P::get_mag_strength() {
        uint16_t mag = read_reg(REGISTER::MAG);
        mag = mag & 0b0011111111111111; // Mask the 2 MSB bits
        return mag;
    }

    void AS5047P::set_direction(DIRECTION direction) {
        _direction = direction;
    }


} // namespace AS5047P