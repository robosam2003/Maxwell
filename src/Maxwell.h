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
        double alpha;
        double beta;
    };

    struct dq_struct {
        double d;
        double q;
    };

    struct PhaseCurrents {
        double current_a;
        double current_b;
        double current_c;
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

        uint8_t PIN_A_STATE;
        uint8_t PIN_B_STATE;
        uint8_t PIN_C_STATE;

        uint32_t prev_cnt_a;
        uint32_t prev_cnt_b;
        uint32_t prev_cnt_c;

        bool rising_a;
        bool rising_b;
        bool rising_c;
    };


    class Maxwell {
    public:

        float_frame command_voltage_frame = {
            .name = "Command Voltage",
            .values = {0.0, 0.0, 0.0}
        };
        float_frame phase_current_frame = {
            .name = "Phase Current",
            .values = {0.0, 0.0, 0.0}
        };
        float_frame rotor_position_frame = {
            .name = "Rotor Position",
            .values = {0.0}
        };
        float_frame rotor_velocity_frame = {
            .name = "Rotor Velocity",
            .values = {0.0}
        };
        // float_frame
        // float_frame

        float_frame alpha_beta_frame = {
            .name = "Alpha-Beta",
            .values = {0.0, 0.0}
        };
        float_frame dq_frame = {
            .name = "dq",
            .values = {0.0, 0.0}
        };
        float_frame command_dq_frame = {
            .name = "command dq",
            .values = {0.0, 0.0}
        };
        float_frame command_alpha_beta_frame = {
            .name = "command alpha-beta",
            .values = {0.0, 0.0}
        };



        DRV8323::DRV8323* driver;
        HallSensor* hall_sensor;
        AS5047P::AS5047P* encoder;
        PIDController* pid_controller;
        triggered* trigger;
        PWMInput* pwm_input;
        Currents* curr_struct;
        pwm_3x_struct* pwm_3x;
        uint32_t pwm_frequency = 20000;
        float max_voltage = 1.5;   // V
        float max_current = 2;   // A
        float align_max_voltage = 1.5;
        float offset = 0.1; // V

        static Maxwell* instance;

        Maxwell(); // Constructor

        void setup();

        void init_pwm_3x();

        void init_pwm_6x();

        void sync_timer_frequencies(long pwm_frequency);

        void sync_pwm();

        void set_pwm(uint32_t Ua, uint32_t Ub, uint32_t Uc, uint32_t resolution);

        void set_phase_voltages(float Va, float Vb, float Vc);

        void all_off();

        void state_feedback();

        void foc_init_sequence();

        void foc_position_control();

        void sinusoidal_position_control();

        void voltage_torque_control();

        void voltage_position_control();

        void foc_current_torque_control();



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
