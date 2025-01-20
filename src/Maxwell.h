//
// Created by robos on 11/09/2023.
//

#ifndef MAXWELL_MAXWELL_H
#define MAXWELL_MAXWELL_H

#include <pin_definitions.h>
#include "DRV8323.h"
#include "AS5047P.h"
#include "FreeRTOS.h"
#include "task.h"

namespace Maxwell {

    struct triggered {
        bool phases[3];
        bool zero_cross[3];
    };

    class Maxwell {
    public:
        DRV8323::DRV8323* driver;
        // AS5047P::AS5047P* encoder;
        triggered* trigger;


        Maxwell(); // Constructor

        void setup();

        void drive_hall_velocity(int velocity, int duration);

        void drive_velocity(int velocity, int duration);



        // static void driver_error_task(void *pvParameters);
    };
}
#endif //MAXWELL_MAXWELL_H
