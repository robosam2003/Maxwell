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
    }


    void HallSensor::callback_a() {
        // Debounce
        int current_time = millis();
        if (current_time - prev_callback_time_a > debounce_time_ms) {
            hall_a_state = digitalRead(hall_pin_a);
            hall_code = hall_a_state << 2 | hall_b_state << 1 | hall_c_state; // 0-7
            if (hallcode_to_index[hall_code] != -1) {
                rotor_sector = hallcode_to_index[hall_code];
            }
        }
        prev_callback_time_a = current_time;
        Serial.print(hallcode_to_index[hall_code]); Serial.print(" ");
        Serial.println(hall_code, BIN);
    }

    void HallSensor::callback_b() {
        // Debounce
        int current_time = millis();
        if (current_time - prev_callback_time_b > debounce_time_ms) {
            hall_b_state = digitalRead(hall_pin_b);
            hall_code = hall_a_state << 2 | hall_b_state << 1 | hall_c_state; // 0-7
            if (hallcode_to_index[hall_code] != -1) {
                rotor_sector = hallcode_to_index[hall_code];
            }
        }
        prev_callback_time_b = current_time;
        Serial.print(hallcode_to_index[hall_code]); Serial.print(" ");
        Serial.println(hall_code, BIN);

    }

    void HallSensor::callback_c() {
        // Debounce
        int current_time = millis();
        if (current_time - prev_callback_time_c > debounce_time_ms) {
            hall_c_state = digitalRead(hall_pin_c);
            hall_code = hall_a_state << 2 | hall_b_state << 1 | hall_c_state; // 0-7
            if (hallcode_to_index[hall_code] != -1) {
                rotor_sector = hallcode_to_index[hall_code];
            }
        }
        prev_callback_time_c = current_time;
        Serial.print(hallcode_to_index[hall_code]); Serial.print(" ");
        Serial.println(hall_code, BIN);
    }


} // Maxwell