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

    struct pwm_3x_struct {
        HardwareTimer *TIM_A;
        HardwareTimer *TIM_B;
        HardwareTimer *TIM_C;
        uint8_t channel_a;
        uint8_t channel_b;
        uint8_t channel_c;
        uint32_t FREQ;
        uint32_t RESOLUTION;
        uint32_t MAX_COMPARE_VALUE; // 2^RESOLUTION - 1

        uint8_t PIN_A;
        uint8_t PIN_B;
        uint8_t PIN_C;
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
        pwm_3x_struct* pwm_3x;
        float max_voltage = 4;

        static Maxwell* instance;



        static void Compare_A_callback();
        static void Compare_B_callback();
        static void Compare_C_callback();


        Maxwell(); // Constructor

        void setup();

        void init_pwm();

        void sync_pwm();

        void set_pwm(uint32_t Ua, uint32_t Ub, uint32_t Uc, uint32_t resolution);

        void set_phase_voltages(float Va, float Vb, float Vc);

        void all_off();

        void state_feedback();

        void foc_position_control();

        void svpwm_position_control();

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
