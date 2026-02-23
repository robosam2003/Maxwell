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

class PositionSensor {
public:
    float prev_raw_angle        = 0.0; // Previous relative angle (0 to 2*pi)
    float prev_absolute_angle   = 0.0;
    float absolute_angle        = 0.0; // Absolute angle factoring in full rotations (radians)

    long long full_rotations         = 0; // Number of full rotations
    float offset                = 0.0; // Motor-sensor offset (radians)

    float velocity              = 0.0; // radians per second

    float pos_lpf_cutoff = 10.0;
    RCFilter* pos_lpf = new RCFilter(pos_lpf_cutoff);
    bool pos_filtered = false;


    SENSOR_DIRECTION _direction; // CW or CCW

    virtual void update() = 0;
    virtual float get_angle() = 0; // in radians
    virtual float get_velocity() = 0; // in radians per second
    virtual void set_offset(float angle) = 0;
};


#endif