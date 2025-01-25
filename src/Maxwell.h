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
#include "HallSensor.h"
#include "pid_controller.h"
#include "PWMInput.h"

namespace Maxwell {

    struct triggered {
        bool phases[3];
        bool zero_cross[3];
    };

    class Maxwell {
    public:
        DRV8323::DRV8323* driver;
        HallSensor* hall_sensor;
        // AS5047P::AS5047P* encoder;
        PIDController* pid_controller;
        triggered* trigger;
        PWMInput* pwm_input;



        Maxwell(); // Constructor

        void setup();

        void drive_hall_velocity();

        void drive_velocity(int velocity, int duration);



        // static void driver_error_task(void *pvParameters);
    };
}
#endif //MAXWELL_MAXWELL_H
