//
// Created by robosam on 1/25/25.
//

#ifndef PWMINPUT_H
#define PWMINPUT_H

#include "Arduino.h"

enum INPUT_MODE {
  UNIDIRECTIONAL,
  BIDIRECTIONAL
};

enum DIRECTION {
  FORWARD,
  REVERSE
};


class PWMInput {
private:
    int _input_pin;
    INPUT_MODE _input_mode;
    uint32_t prev_rise_time;
    uint32_t prev_fall_time;
    float frequency;
    DIRECTION _direction;
    uint32_t _pwm_value;

public:
    int _bottom_threshold = 1000;
    int _top_threshold = 2000;

    PWMInput(int input_pin, INPUT_MODE input_mode, DIRECTION direction);

    void set_callback(void (*interrupt_callback)());

    void pwm_callback();

    uint32_t read();

    uint32_t read_percentage();
};




#endif //PWMINPUT_H
