//
// Created by robosam on 1/24/25.
//

#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float setpoint, float max_output, float max_integral) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _setpoint = setpoint;
    _max_output = max_output;
    _max_integral = max_integral;
    _prev_input_time = micros();

    _integral = 0.0;
}

void PIDController::set_setpoint(float sp) {
    _setpoint = sp;
}

float PIDController::update(const float input) {
    uint32_t current_time = micros();
    float dt = (current_time - _prev_input_time) / 1e6;
    _error = _setpoint - input;
    // Serial.println((current_time - _prev_input_time) / 1e6);
    _integral += _error * dt;
    // Serial.println(_integral);
    _integral = constrain(_integral, -_max_integral, _max_integral);
    _derivative = (_error - _prev_error) / dt;
    _output = _kp * _error + _ki * _integral + _kd * _derivative;
    _output = constrain(_output, -_max_output, _max_output);
    _prev_error = _error;
    _prev_input_time = current_time;
    _prev_input = input;
    return _output;
}

void PIDController::print_state() {
    Serial.print("Setpoint: ");
    Serial.print(_setpoint);
    Serial.print(" Error: ");
    Serial.print(_error);
    Serial.print(" Integral: ");
    Serial.print(_integral);
    Serial.print(" Input: ");
    Serial.print(_prev_input);
    Serial.print(" Output: ");
    Serial.println(_output);
}