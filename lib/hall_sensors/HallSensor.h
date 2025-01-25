//
// Created by robosam on 1/21/25.
//

#ifndef HALLSENSOR_H
#define HALLSENSOR_H
#include <Arduino.h>
#include <RCFilter.h>

namespace Maxwell {


enum MOTOR_DIRECTION {
    CW = 1,
    CCW = -1
};


class HallSensor {
private:
    int hall_pin_a;
    int hall_pin_b;
    int hall_pin_c;

// 0, 1, 2, 3, 4, 5 - INDEX
// 1, 3, 2, 6, 4, 5 - HALL SEQUENCE
//                               000 001 010 011 100 101 110 111
// int8_t hallcode_to_index_cw[8] =   { -1,  0,  2,  1,  4,  5,  3, -1};  // wrong way?
int8_t hallcode_to_index[8] =   { -1,  3,  5,  4,  1,  2,  0, -1};

    // int hall_to_index[6] = {1, 3, 2, 6, 4, 5};
// e.g. index = hall_to_index[rotor_position]

    // For debouncing hall sensor signals
    // uint32_t prev_callback_time_a;
    // uint32_t prev_callback_time_b;
    // uint32_t prev_callback_time_c;
    uint32_t prev_callback_time;
    uint32_t prev_callback_sensor;  // 0=A, 1=B, 2=C
    MOTOR_DIRECTION direction;

    uint32_t debounce_time_us = 1000;

    bool hall_a_state;
    bool hall_b_state;
    bool hall_c_state;



public:
    // Rotor position
    uint8_t hall_code;
    int8_t rotor_sector;
    float electrical_velocity;
    RCFilter* velocity_filter;



    HallSensor(int pin_a, int pin_b, int pin_c);

    void setup(bool interrupts_enable, void (*callback_a)(), void (*callback_b)(), void (*callback_c)());

    void callback_a();
    void callback_b();
    void callback_c();

    void update();

    void read_hall_state();


};

} // Maxwell

#endif //HALLSENSOR_H
