//
// Created by robos on 17/12/2025.
//

#ifndef MAXWELL_POSITION_SENSOR_H
#define MAXWELL_POSITION_SENSOR_H

enum SENSOR_DIRECTION {
    CW = 1,
    CCW = -1
};

#include "RCFilter.h"
#include "pid_controller.h"

struct error_struct {
    uint32_t parity_error_cnt;
    uint32_t error_flag_cnt;
    uint16_t error_flag;
    uint16_t delta_jump_error_cnt;
};

class PositionSensor {
public:

    virtual ~PositionSensor() = default;

    float prev_raw_angle        = 0.0; // Previous relative angle (0 to 2*pi)
    float prev_absolute_angle   = 0.0;
    float absolute_angle        = 0.0; // Absolute angle factoring in full rotations (radians)

    long long full_rotations    = 0;   // Number of full rotations
    float offset                = 0.0; // Motor-sensor offset (radians)
    float velocity              = 0.0; // radians per second

    bool calibrated             = false; // Flag to indicate if the sensor has been calibrated
    float pos_lpf_cutoff = 10.0;
    RCFilter* pos_lpf = new RCFilter(pos_lpf_cutoff);
    bool pos_filtered = false;
    float theta_est = 0; // Estimated angle for velocity estimation
    float velocity_estimate; // Velocity estimate from tracking observer
    PIDController angle_observer = PIDController(0.0, 0.0, 0, 0, 0, 0); // PID controller for the tracking observer
    error_struct errors;

    SENSOR_DIRECTION _direction; // CW or CCW

    virtual void update() = 0;
    virtual float get_angle() = 0; // in radians
    virtual float get_velocity() = 0; // in radians per second
    virtual void set_offset(float angle) = 0;
};


#endif