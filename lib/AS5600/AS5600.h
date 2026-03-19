//
// Created by robos on 20/12/2025.
//

#ifndef MAXWELL_AS5600_H
#define MAXWELL_AS5600_H

#include "PositionSensor.h"
#include <SPI.h>
#include "../../include/config.h"
#include "HardwareTimer.h"
#include "pid_controller.h"
#include "Wire.h"


class AS5600 : public PositionSensor {
private:
    TwoWire* _pipe; // I2C bus
    const uint8_t READ_BYTE = 0x40;
    const uint8_t WRITE_BYTE = 0x00;

    const uint8_t I2C_ADDRESS = 0x36;

    enum REGISTER : uint16_t {
        RAW_ANGLE_H = 0x0C,
        RAW_ANGLE_L = 0x0D
    };

    int prev_time_counts = 0; // Timestamp of the previous angle reading
    uint32_t prev_micros = 0; // Timestamp of the previous angle reading in microseconds
    // HardwareTimer* timer;
    float integral_observer = 0; // Integral term for the tracking observer

    uint8_t read_reg(REGISTER regAddress) const;
    uint16_t read_reg16(REGISTER regAddress) const; // Atomic burst read of two consecutive registers
    // void read_regs(byte regAddress, byte* outputPointer, uint length);
    void write_reg(REGISTER regAddress, uint8_t data);
    float K = 30.0; // Gain for the tracking observer - this can be tuned based on the expected noise and dynamics of the system

public:
    // PIDController angle_observer = PIDController(50.0,
    //                                              50.0,
    //                                             0,
    //                                             0,
    //                                             100000,
    //                                             100000); // PID controller for the tracking observer

    AS5600(uint8_t SDA, uint8_t SCL, uint32_t freq);
    void update() override;
    float get_angle() override; // in radians
    float get_velocity() override; // in radians per second
    float get_velocity_estimate(float angle_meas, float Ts); // Tracking observer
    void set_offset(float angle) override;
};


#endif //MAXWELL_AS5600_H