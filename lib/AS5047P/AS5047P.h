//
// Created by robos on 20/12/2025.
//

#ifndef MAXWELL_AS5047P_H
#define MAXWELL_AS5047P_H

#include "PositionSensor.h"
#include <SPI.h>
#include "../../include/config.h"
#include "HardwareTimer.h"
#include "pid_controller.h"


class AS5047P : public PositionSensor {
private:
    uint8_t _CS;
    SPIClass _spi;
    SPISettings _settings;

    const uint8_t READ_BYTE = 0x40;
    const uint8_t WRITE_BYTE = 0x00;
    const float MAX_DELTA_ANGLE = 0.5; // Maximum allowed change in angle between updates (radians) - used for error detection
public:
    enum REGISTER {
        NOOP        = 0x0000,
        ERRFL       = 0x0001,
        PROG        = 0x0003,
        DIAAGC      = 0x3FFC,
        MAG         = 0x3FFD,
        ANGLEUNCOMP = 0x3FFE,
        ANGLECOM    = 0x3FFF
    };
    enum ERROR: uint8_t {
        NO_ERROR = 0x00,
        FRAMING_ERROR = 0x1,
        COMMAND_INVALID = 0x2,
        PARITY_ERROR = 0x3,
    };

    int prev_time_counts = 0; // Timestamp of the previous angle reading
    uint32_t prev_micros = 0; // Timestamp of the previous angle reading in microseconds
    HardwareTimer* timer;
    float integral_observer = 0; // Integral term for the tracking observer

    uint16_t read_reg(REGISTER regAddress);
    void write_reg(REGISTER regAddress, uint16_t data);
    float K = 30.0; // Gain for the tracking observer - this can be tuned based on the expected noise and dynamics of the system


    PIDController angle_observer = PIDController(50.0,
                                                 50.0,
                                                0,
                                                0,
                                                100000,
                                                100000); // PID controller for the tracking observer

    AS5047P(byte CS, SPIClass& spi, uint32_t spiFreq);

    void update() override;
    uint16_t read_angle_reg();
    float get_angle() override; // in radians
    float get_velocity() override; // in radians per second
    float get_velocity_estimate(float angle_meas, float Ts); // Tracking observer
    ERROR get_error();
    void set_offset(float angle) override;
};


#endif //MAXWELL_AS5047P_H