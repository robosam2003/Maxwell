//
// Created by robos on 11/09/2023.
//

#include "Maxwell.h"

namespace Maxwell {
    Maxwell::Maxwell() = default;

    void Maxwell::setup() {
        pinMode(led_pin, OUTPUT);


        // Breakout to a driver class?
        pinMode(DRV8323_HI_A_PIN, OUTPUT);
        pinMode(DRV8323_HI_B_PIN, OUTPUT);
        pinMode(DRV8323_HI_C_PIN, OUTPUT);
        pinMode(DRV8323_LO_A_PIN, OUTPUT);
        pinMode(DRV8323_LO_B_PIN, OUTPUT);
        pinMode(DRV8323_LO_C_PIN, OUTPUT);
        pinMode(DRV8323_GATE_EN_PIN, OUTPUT);
        pinMode(DRV8323_DRIVE_CAL_PIN, OUTPUT);

        digitalWrite(DRV8323_DRIVE_CAL_PIN, LOW);
    }

    void Maxwell::toggle_led() {
        digitalToggle(led_pin);
    }

}
