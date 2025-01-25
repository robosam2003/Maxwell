//
// Created by robosam on 1/25/25.
//

#include "PWMInput.h"

PWMInput::PWMInput(int input_pin, INPUT_MODE input_mode, DIRECTION direction) {
    _input_pin = input_pin;
    _input_mode = input_mode;
    _direction = direction;
    prev_rise_time = micros();
    prev_fall_time = micros();
    _pwm_value = 0;
    pinMode(_input_pin, INPUT_PULLDOWN);
}

void PWMInput::pwm_callback() {
    bool state = digitalRead(_input_pin);
    uint32_t current_time = micros();
    if (state == HIGH) {
        frequency = 1 / (current_time - prev_rise_time);
        prev_rise_time = current_time;
    }
    else {
        _pwm_value = current_time - prev_rise_time;
        if (_pwm_value > 1000000) { // if the pulse is too long, the signal is invalid.
            _pwm_value = 0;
        }
    }
}

void PWMInput::set_callback(void (*interrupt_callback)()) {
    attachInterrupt(digitalPinToInterrupt(_input_pin), interrupt_callback, CHANGE);
}


uint32_t PWMInput::read() {
    return _pwm_value;
}

uint32_t PWMInput::read_percentage() {
    _pwm_value = constrain(_pwm_value, _bottom_threshold, _top_threshold);
    return map(_pwm_value, _bottom_threshold, _top_threshold, 0, 100);
}