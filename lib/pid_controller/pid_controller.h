//
// Created by robosam on 1/24/25.
//

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include "Arduino.h"

class PIDController {
private:
    float _kp;
    float _ki;
    float _kd;
    float _setpoint;
    float _error;
    float _prev_error;
    float _integral;
    float _derivative;
    float _output;
    uint32_t _prev_input_time;
    float _prev_input;

    float _max_output;
    float _max_integral;  // Anti-windup control

public:
    PIDController(float kp, float ki, float kd, float setpoint, float max_output, float max_integral);

    float update(float input);

    void set_setpoint(float sp);

    void print_state();



};


#endif //PID_CONTROLLER_H
