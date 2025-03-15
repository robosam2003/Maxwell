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


    // Constructor
    DRV8323::DRV8323(byte CS, SPIClass &spi, uint32_t spiFreq, uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, byte gate_enable_pin) {
        _CS = CS;
        _spi = spi;

        pinMode(pin_a, OUTPUT);
        pinMode(pin_b, OUTPUT);
        pinMode(pin_c, OUTPUT);


        // TODO: Deal with this properly - this is a quick hack
        pinMode(DRV8323_LO_A_PIN, OUTPUT);
        pinMode(DRV8323_LO_B_PIN, OUTPUT);
        pinMode(DRV8323_LO_C_PIN, OUTPUT);
        // Tie all low side pins HIGH to avoid hi-z state
        digitalWrite(DRV8323_LO_A_PIN, HIGH);
        digitalWrite(DRV8323_LO_B_PIN, HIGH);
        digitalWrite(DRV8323_LO_C_PIN, HIGH);

        gate_en_pin = gate_enable_pin;



        // Set up SPI settings - SPI MODE 1 because data is captured on the falling edge of the clock
        // and propagated on the rising edge - https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
        _settings = SPISettings(spiFreq, MSBFIRST, SPI_MODE1);

        // Set up CS pin
        pinMode(_CS, OUTPUT);
        digitalWrite(_CS, HIGH);

        // Begin the SPI bus.
        _spi.begin();

        current_sensors = new CurrentSensors(DRV8323_CURR_SENSE_A_PIN,
                                            DRV8323_CURR_SENSE_B_PIN,
                                            DRV8323_CURR_SENSE_C_PIN,
                                            _csa_gain);
    }

    void DRV8323::enable(bool enable) {
        enabled = enable;
        if (enable) {
            digitalWrite(gate_en_pin, HIGH);
        } else {
            digitalWrite(gate_en_pin, LOW);
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

    void DRV8323::write_reg(REGISTER regAddress, uint16_t data) {  // Any way to get feedback from this?
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

    String DRV8323::get_fault_status_1_string() {
        uint16_t fault_code = get_fault_status_1();
        String fault_string = "";
        if (getBit(fault_code, 0)) {
            fault_string += "VDS_LC ";
        }
        if (getBit(fault_code, 1)) {
            fault_string += "VDS_HC ";
        }
        if (getBit(fault_code, 2)) {
            fault_string += "VDS_LB ";
        }
        if (getBit(fault_code, 3)) {
            fault_string += "VDS_HB ";
        }
        if (getBit(fault_code, 4)) {
            fault_string += "VDS_LA ";
        }
        if (getBit(fault_code, 5)) {
            fault_string += "VDS_HA ";
        }
        if (getBit(fault_code, 6)) {
            fault_string += "OTSD ";
        }
        if (getBit(fault_code, 7)) {
            fault_string += "UVLO ";
        }
        if (getBit(fault_code, 8)) {
            fault_string += "GDF ";
        }
        if (getBit(fault_code, 9)) {
            fault_string += "VDS_OCP ";
        }
        if (getBit(fault_code, 10)) {
            fault_string += "FAULT ";
        }
        return fault_string;
    }

    uint16_t DRV8323::get_fault_status_2() {
        return read_reg(REGISTER::VGS_STATUS_2);
    }

    String DRV8323::get_fault_status_2_string() {
        uint16_t fault_code = get_fault_status_2();
        String fault_string = "";
        if (getBit(fault_code, 0)) {
            fault_string += "VGS_LC ";
        }
        if (getBit(fault_code, 1)) {
            fault_string += "VGS_HC ";
        }
        if (getBit(fault_code, 2)) {
            fault_string += "VGS_LB ";
        }
        if (getBit(fault_code, 3)) {
            fault_string += "VGS_HB ";
        }
        if (getBit(fault_code, 4)) {
            fault_string += "VGS_LA ";
        }
        if (getBit(fault_code, 5)) {
            fault_string += "VGS_HA ";
        }
        if (getBit(fault_code, 6)) {
            fault_string += "CPUV ";
        }
        if (getBit(fault_code, 7)) {
            fault_string += "OTW ";
        }
        if (getBit(fault_code, 8)) {
            fault_string += "SC_OC ";
        }
        if (getBit(fault_code, 9)) {
            fault_string += "SB_OC ";
        }
        if (getBit(fault_code, 10)) {
            fault_string += "SA_OC ";
        }
        return fault_string;
    }

    void DRV8323::set_pwm_mode(PWM_MODE mode) {
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        data = data & 0b11110011111; // Mask the PWM_MODE bits
        data = data | (mode << 5);   // Set the PWM_MODE bits
        write_reg(REGISTER::DRIVER_CONTROL, data);
    }

    void DRV8323::set_current_gain(CSA_GAIN gain) {
        _csa_gain = gain;
        uint16_t data = read_reg(REGISTER::CSA_CONTROL);
        data = data & 0b11100111111; // Mask the CSA_GAIN bits
        data = data | (gain << 5);   // Set the CSA_GAIN bits
        write_reg(REGISTER::CSA_CONTROL, data);
        current_sensors->set_csa_gain(gain);
    }

    void DRV8323::set_gate_drive_source_current(IDRIVE_P_CURRENT current) {
        uint16_t data_1_hs = read_reg(REGISTER::GATE_DRIVE_HS);
        data_1_hs = data_1_hs & 0b11100001111; // Mask the IDRIVEP_HS bits
        data_1_hs = data_1_hs | (current << 4);   // Set the IDRIVEP_HS bits
        write_reg(REGISTER::GATE_DRIVE_HS, data_1_hs);
        delay(1);
        uint16_t data_2_ls = read_reg(REGISTER::GATE_DRIVE_LS);
        data_2_ls = data_2_ls & 0b11100001111; // Mask the IDRIVEP_LS bits
        data_2_ls = data_2_ls | (current << 4);   // Set the IDRIVEP_LS bits
        write_reg(REGISTER::GATE_DRIVE_LS, data_2_ls);
    }

    void DRV8323::set_gate_drive_sink_current(IDRIVE_N_CURRENT current) {
        uint16_t data_1_hs = read_reg(REGISTER::GATE_DRIVE_HS);
        data_1_hs = data_1_hs & 0b11111110000; // Mask the IDRIVEN_HS bits
        data_1_hs = data_1_hs | current;   // Set the IDRIVEN_HS bits
        write_reg(REGISTER::GATE_DRIVE_HS, data_1_hs);
        delay(1);
        uint16_t data_2_ls = read_reg(REGISTER::GATE_DRIVE_LS);
        data_2_ls = data_2_ls & 0b11111110000; // Mask the IDRIVEN_LS bits
        data_2_ls = data_2_ls | current;   // Set the IDRIVEN_LS bits
        write_reg(REGISTER::GATE_DRIVE_LS, data_2_ls);
    }

    void DRV8323::set_peak_gate_drive_time(TDRIVE_TIME time) {
        uint16_t data = read_reg(REGISTER::GATE_DRIVE_LS);
        data = data & 0b10011111111; // Mask the TDRIVE bits
        data = data | (time << 8);   // Set the TDRIVE bits
        write_reg(REGISTER::GATE_DRIVE_LS, data);
    }

    void DRV8323::perform_current_sense_calibration() {
        uint16_t data = read_reg(REGISTER::CSA_CONTROL);
        setBit(&data, 4, 1);  // Channel A
        setBit(&data, 3, 1);  // Channel B
        setBit(&data, 2, 1);  // Channel C
        write_reg(REGISTER::CSA_CONTROL, data);
        delay(100);  // Wait for calibration to complete
        setBit(&data, 4, 0);  // Channel A
        setBit(&data, 3, 0);  // Channel B
        setBit(&data, 2, 0);  // Channel C
        write_reg(REGISTER::CSA_CONTROL, data);
        set_current_gain(_csa_gain);  // Reset the CSA gain to whatever it was.
    }


    void DRV8323::default_configuration() {
        enable(true);
        set_pwm_mode(PWM_MODE::PWM_3x);
        enable_CPUV_Fault(false);
        enable_GDF(false);
        set_current_gain(CSA_GAIN::GAIN_40_V_V);
        clear_fault();
    }

    void DRV8323::enable_CPUV_Fault(bool enable) {
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        setBit(&data, 9, enable ? 0 : 1);
        write_reg(REGISTER::DRIVER_CONTROL, data);
    }

    void DRV8323::enable_GDF(bool enable) { // Gate drive fault
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        setBit(&data, 3, enable ? 0 : 1);
        write_reg(REGISTER::DRIVER_CONTROL, data);

    }

    void DRV8323::set_SYNC_rectification(bool enable) {
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        setBit(&data, 8, enable ? 0 : 1);
        write_reg(REGISTER::DRIVER_CONTROL, data);
    }

    void DRV8323::clear_fault() {
        uint16_t data = read_reg(REGISTER::DRIVER_CONTROL);
        setBit(&data, 0, 1);
        write_reg(REGISTER::DRIVER_CONTROL, data);
    }


} // namespace DRV8323