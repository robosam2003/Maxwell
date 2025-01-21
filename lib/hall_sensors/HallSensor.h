//
// Created by robosam on 1/21/25.
//

#ifndef HALLSENSOR_H
#define HALLSENSOR_H
#include <Arduino.h>

namespace Maxwell {


class HallSensor {
private:
    int hall_pin_a;
    int hall_pin_b;
    int hall_pin_c;

// 0, 1, 2, 3, 4, 5 - INDEX
// 1, 3, 2, 6, 4, 5 - HALL SEQUENCE
//                               000 001 010 011 100 101 110 111
// int8_t hallcode_to_index[8] =   { -1,  0,  2,  1,  4,  5,  3, -1};
int8_t hallcode_to_index[8] =   { -1,  3,  5,  4,  1,  2,  0, -1};

    // int hall_to_index[6] = {1, 3, 2, 6, 4, 5};
// e.g. index = hall_to_index[rotor_position]

    // For debouncing hall sensor signals
    int prev_callback_time_a;
    int prev_callback_time_b;
    int prev_callback_time_c;
    int debounce_time_ms = 5;

    bool hall_a_state;
    bool hall_b_state;
    bool hall_c_state;

public:
    // Rotor position
    uint8_t hall_code;
    int8_t rotor_sector;


    HallSensor(int pin_a, int pin_b, int pin_c);

    void setup(bool interrupts_enable, void (*callback_a)(), void (*callback_b)(), void (*callback_c)());

    void callback_a();
    void callback_b();
    void callback_c();

    void read_hall_state();


};

} // Maxwell

#endif //HALLSENSOR_H
