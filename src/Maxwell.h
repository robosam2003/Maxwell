//
// Created by robos on 11/09/2023.
//

#ifndef MAXWELL_MAXWELL_H
#define MAXWELL_MAXWELL_H

#include <pin_definitions.h>
#include "DRV8323.h"

namespace Maxwell {
    class Maxwell {
    private:
        uint32_t led_pin = GREEN_LED_PIN;
        uint32_t gate_en_pin = DRV8323_GATE_EN_PIN;

    public:
        Maxwell(); // Constructor

        void setup();

        void toggle_led();
    };
}
#endif //MAXWELL_MAXWELL_H
