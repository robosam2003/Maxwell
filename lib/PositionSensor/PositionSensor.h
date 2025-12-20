//
// Created by robos on 17/12/2025.
//

#ifndef MAXWELL_POSITION_SENSOR_H
#define MAXWELL_POSITION_SENSOR_H

enum SENSOR_DIRECTION {
    CW = 1,
    CCW = -1
};


class PositionSensor {
public:
    float prev_raw_angle        = 0.0; // Previous relative angle (0 to 2*pi)
    float prev_absolute_angle   = 0.0;
    float absolute_angle        = 0.0; // Absolute angle factoring in full rotations (radians)

    long long full_rotations         = 0; // Number of full rotations
    float offset                = 0.0; // Motor-sensor offset (radians)

    float velocity              = 0.0; // radians per second


    SENSOR_DIRECTION _direction; // CW or CCW

    virtual void update() = 0;
    virtual float get_angle() = 0; // in radians
    virtual float get_velocity() = 0; // in radians per second
};


#endif