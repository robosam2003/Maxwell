//
// Created by robosam on 1/25/25.
//

#include "PWMInput.h"

PWMInput::PWMInput(int input_pin, INPUT_MODE input_mode, DIRECTION direction) {
    _input_pin = input_pin;
    _input_mode = input_mode;
    _direction = direction;
    _pwm_value = 0;
    pinMode(_input_pin, INPUT);
}

uint32_t PWMInput::read() {
    _pwm_value = pulseIn(_input_pin, HIGH);
    if (_pwm_value > 1000000) { // if the pulse is too long, the signal is invalid.
        _pwm_value = 0;
    }
    return _pwm_value;
}

uint32_t PWMInput::read_percentage() {
    _pwm_value = pulseIn(_input_pin, HIGH);
    return map(_pwm_value, _bottom_threshold, _top_threshold, 0, 100);
}