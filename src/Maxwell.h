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
#include "current_sensors.h"

namespace Maxwell {

    struct triggered {
        bool phases[3];
        bool zero_cross[3];
    };

    struct FOC {
        PIDController* d_pid;
        PIDController* q_pid;
        PIDController* position_pid;
    };

    struct alpha_beta_struct {
        float alpha;
        float beta;
    };

    struct dq_struct {
        float d;
        float q;
    };

    struct PhaseCurrents {
        float current_a;
        float current_b;
        float current_c;
    };

    struct Currents {
        PhaseCurrents phase_currents;
        alpha_beta_struct alpha_beta;
        dq_struct dq;
    };


    class Maxwell {
    public:
        DRV8323::DRV8323* driver;
        HallSensor* hall_sensor;
        AS5047P::AS5047P* encoder;
        PIDController* pid_controller;
        triggered* trigger;
        PWMInput* pwm_input;
        FOC* foc;
        Currents* curr_struct;

        Maxwell(); // Constructor

        void setup();

        void state_feedback();

        void foc_position_control();

        alpha_beta_struct clarke_transform(PhaseCurrents currents);  // Currents to alpha-beta

        dq_struct park_transform(alpha_beta_struct ab_vec);  // Alpha-beta to dq

        alpha_beta_struct reverse_park_transform(dq_struct dq_vec);  // dq to alpha-beta

        PhaseCurrents reverse_clarke_transform(alpha_beta_struct ab_vec);   // alpha-beta to currents

        void drive_hall_velocity();

        void drive_velocity(int velocity, int duration);



        // static void driver_error_task(void *pvParameters);
    };
}
#endif //MAXWELL_MAXWELL_H
