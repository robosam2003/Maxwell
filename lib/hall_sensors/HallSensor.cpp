//
// Created by robosam on 1/21/25.
//

#include "HallSensor.h"

namespace Maxwell {

    HallSensor::HallSensor(int pin_a, int pin_b, int pin_c) {
        hall_pin_a = pin_a;
        hall_pin_b = pin_b;
        hall_pin_c = pin_c;
    }

    void HallSensor::setup(bool interrupts_enable, void (*callback_a)(), void (*callback_b)(), void (*callback_c)()) {
        pinMode(hall_pin_a, INPUT_PULLUP); // Enable pullup resistor
        pinMode(hall_pin_b, INPUT_PULLUP); // Enable pullup resistor
        pinMode(hall_pin_c, INPUT_PULLUP); // Enable pullup resistor

        if (interrupts_enable) {
            attachInterrupt(digitalPinToInterrupt(hall_pin_a), callback_a, CHANGE);
            attachInterrupt(digitalPinToInterrupt(hall_pin_b), callback_b, CHANGE);
            attachInterrupt(digitalPinToInterrupt(hall_pin_c), callback_c, CHANGE);
        }
        hall_a_state = digitalRead(hall_pin_a);
        hall_b_state = digitalRead(hall_pin_b);
        hall_c_state = digitalRead(hall_pin_c);
        hall_code = hall_a_state << 2 | hall_b_state << 1 | hall_c_state; // 0-7
        rotor_sector = hallcode_to_index[hall_code];
    }


    void HallSensor::callback_a() {
        hall_a_state = digitalRead(hall_pin_a);
        update();
    }

    void HallSensor::callback_b() {
        hall_b_state = digitalRead(hall_pin_b);
        update();
    }

    void HallSensor::callback_c() {
        hall_c_state = digitalRead(hall_pin_c);
        update();
    }


    void HallSensor::update() {
        hall_code = hall_a_state << 2 | hall_b_state << 1 | hall_c_state; // 0-7
        int8_t new_rotor_secotor = hallcode_to_index[hall_code];
        if ((new_rotor_secotor == -1) or new_rotor_secotor == rotor_sector) return; // Eliminate invalid hall codes
        int8_t rotor_sector_diff = new_rotor_secotor - rotor_sector;
        rotor_sector = new_rotor_secotor;
        if (rotor_sector_diff > 3) { // 5 - 0 - underflow
            direction = MOTOR_DIRECTION::CCW;
        }
        else if (rotor_sector_diff < -3) { // 0 - 5 - overflow
            direction = MOTOR_DIRECTION::CW;
        }
        else {
            direction = rotor_sector_diff > 0 ? MOTOR_DIRECTION::CW : MOTOR_DIRECTION::CCW;
        }
        // Calculate electrical velocity
        uint32_t current_time = micros();
        electrical_velocity = (direction) * PI /3 / (current_time - prev_callback_time) * 1e6;
        prev_callback_time = current_time;
        // Serial.print(hallcode_to_index[hall_code]); Serial.print(" ");
        // Serial.println(hall_code, BIN);
        // Serial.println(electrical_velocity);
    }


} // Maxwell