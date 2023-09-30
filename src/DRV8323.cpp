//
// Created by robos on 20/09/2023.
//

#include "DRV8323.h"

namespace DRV8323 {

    /* Bit manipulation */
    byte getBit(uint16_t bits, int bitIndex) {
        return (bits & (1 << (bitIndex))) ? 1 : 0;
    }

    void setBit(uint16_t * bits, int bitIndex, int val) { // index starts from lsb (at 0) and goes left
        // This way, bits is ORed a one that is shifted left by bitIndex, which sets the bit to 1
        // likewise bit is ANDed with a byte like 0b11101111 for example, if the bit index was 4. This sets ONLY the 5th bit to 0;
        (val) ? (*bits |= (1 << (bitIndex))) : (*bits &= ~(1 << (bitIndex)));
    }


    // constructor
    DRV8323::DRV8323(byte CS, SPIClass &spi, uint32_t spiFreq) {
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

    void DRV8323::enable(bool enable) {
        enabled = enable;
        if (enable) {
            digitalWrite(DRV8323_GATE_EN_PIN, HIGH);
        } else {
            digitalWrite(DRV8323_GATE_EN_PIN, LOW);
        }
        delay(1);  // Wake-up time for DRV8323 is maximum 1ms
    }

    uint16_t DRV8323::read_reg(REGISTER regAddress) {
        // NOTE: DRV8323 must be ENABLED to run SPI commands
        if (!enabled) {
            enable(true);
        }
        uint16_t result = 0;
        uint16_t addressWord = _READ_WORD | (regAddress << ((word_length - 1) - address_length)); // Set up the address byte

        _spi.beginTransaction(_settings); // Begin the SPI transaction
        digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction

        result = _spi.transfer16(addressWord); // Send the address byte
        result = result & 0b0000011111111111; // Mask the first 5 bits of the result, which are don't care bits

        digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
        _spi.endTransaction(); // End the SPI transaction

        return result;
    }

    void DRV8323::write_reg(REGISTER regAddress, uint16_t data) {
        uint16_t word = _WRITE_WORD | (regAddress << ((word_length - 1) - address_length)) | data; // Set up the address byte

        _spi.beginTransaction(_settings); // Begin the SPI transaction
        digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction

        _spi.transfer16(word); // Send the address byte, don't care about the result

        digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
        _spi.endTransaction(); // End the SPI transaction
    }

    uint16_t DRV8323::get_fault_status_1() {
        return read_reg(REGISTER::FAULT_STATUS_1);
    }

    uint16_t DRV8323::get_fault_status_2() {
        return read_reg(REGISTER::VGS_STATUS_2);
    }

    void DRV8323::set_pwm_mode(PWM_MODE mode) {
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        data = data & 0b11110011111; // Mask the PWM_MODE bits
        data = data | (mode << 5);   // Set the PWM_MODE bits
        write_reg(REGISTER::DRIVER_CONTROL, data);
    }

    void DRV8323::default_configuration() {
        enable(true);
        set_pwm_mode(PWM_MODE::PWM_INDEPENDENT);
        enable_CPUV_Fault(false);
        enable_GDF(false);
        set_SYNC_rectification(true);
    }

    void DRV8323::enable_CPUV_Fault(bool enable) {
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        setBit(&data, 9, enable ? 0 : 1);
        write_reg(REGISTER::DRIVER_CONTROL, data);
    }

    void DRV8323::enable_GDF(bool enable) {
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        setBit(&data, 3, enable ? 0 : 1);
        write_reg(REGISTER::DRIVER_CONTROL, data);

    }

    void DRV8323::set_SYNC_rectification(bool enable) {
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        setBit(&data, 8, enable ? 0 : 1);
        write_reg(REGISTER::DRIVER_CONTROL, data);

    }


} // namespace DRV8323