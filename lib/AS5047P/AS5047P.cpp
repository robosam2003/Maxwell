//
// Created by robos on 10/07/2024.
//

#include "AS5047P.h"


namespace AS5047P {


    AS5047P::AS5047P(byte CS, SPIClass &spi, uint32_t spiFreq) {
        _CS = CS;
        _spi = spi;

        // Set up SPI settings - SPI MODE 1 because data is captured on the falling edge of the clock
        // and propagated on the rising edge - https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
        _settings = SPISettings(spiFreq, MSBFIRST, SPI_MODE1);

        // Set up CS pin
        pinMode(_CS, OUTPUT);
        digitalWrite(_CS, HIGH);

        // Begin the SPI bus.
        _spi.begin();
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

    uint16_t AS5047P::get_angle(bool comp) {
        uint16_t angle = 0;
        if (comp) {
            angle = read_reg(REGISTER::ANGLECOM);
        }
        else {
            angle = read_reg(REGISTER::ANGLEUNCOMP);
        }
        angle = angle & 0b0011111111111111; // Mask the 2 MSB bits
        return angle;
    }

    uint16_t AS5047P::get_mag_strength() {
        uint16_t mag = read_reg(REGISTER::MAG);
        mag = mag & 0b0011111111111111; // Mask the 2 MSB bits
        return mag;

    }


} // namespace AS5047P