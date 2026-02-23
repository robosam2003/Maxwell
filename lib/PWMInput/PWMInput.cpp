//
// Created by robosam on 1/25/25.
//

#include "PWMInput.h"
#include "stm32f4xx.h"

PWMInput::PWMInput(int input_pin, INPUT_MODE input_mode, DIRECTION direction) {
    _input_pin = input_pin;
    _input_mode = input_mode;
    _direction = direction;
    prev_rise_time = micros();
    prev_fall_time = micros();
    _pwm_value = 0;

    TimerInstance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(_input_pin), PinMap_PWM);
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(_input_pin), PinMap_PWM));

    PWMTimer = new HardwareTimer(TimerInstance);
    PWMTimer->setMode(channel, TIMER_INPUT_CAPTURE_BOTHEDGE, _input_pin);
    PWMTimer->attachInterrupt(channel,  [this] { pwm_callback(); }); // This will call every time there is a change on the pin
    PWMTimer->setPrescaleFactor(1);
    PWMTimer->setOverflow(0xFFFF); // Max overflow
    PWMTimer->resume();

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

float PWMInput::read() {
    return read_percentage() * command_gain;
}

float PWMInput::read_percentage() {
    _pwm_value = constrain(_pwm_value, _bottom_threshold, _top_threshold);
    return ((_pwm_value - _bottom_threshold) * 1.0f / (_top_threshold - _bottom_threshold));
}