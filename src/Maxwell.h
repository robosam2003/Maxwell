//
// Created by robos on 11/09/2023.
//

#ifndef MAXWELL_MAXWELL_H
#define MAXWELL_MAXWELL_H

#include <pin_definitions.h>
#include "DRV8323.h"
// #include "AS5047P.h"
#include "PositionSensor.h"
#include "AS5048.h"
#include "AS5600.h"

// #include "FreeRTOS.h"
// #include "task.h"
#include "HallSensor.h"
#include "pid_controller.h"
#include "PWMInput.h"
#include "adc.h"
#include "maxwell_config.h"
#include "maxwell_utils.h"
#include "TelemetryTarget.h"
#include "USBTarget.h"
#include "CommandSource.h"
#include "CANBus.h"
#include "ConfigManager.h"


namespace Maxwell {
    class Maxwell {

        // The default configuration
        maxwell_config default_config_struct = {
            COMMAND_SOURCE::PWM,
            TELEMETRY_TARGET::TELEMETRY_USB,
            MOTOR_TYPE::BLDC,
            CONTROL_MODE::VELOCITY,
            SENSOR_TYPE::MAGNETIC,
            TORQUE_CONTROL_MODE::CURRENT,
            SENSOR_LOCATION::INTERNAL,
            POLE_PAIRS_6374,
            0.0,
            {2.0, 0, 0, limits.max_current, 1},
            {2.0, 0, 0, limits.max_current, 3},
            {0.05, 1.0, 0, 20, 20},
            {20, 0, 0, limits.max_velocity, limits.max_velocity},
            {true, 2.0},
            {true, 2.0},
            {false, 2.0},
            {true, 10.0}
        };
    public:
        DRV8323::DRV8323* driver;
        HallSensor* hall_sensor;
        PositionSensor* encoder;
        Adc* adc;

        // Command Sources / Telemetry Targets
        CommandSource* command_source;
        PWMInput* pwm_input;
        CANBus* can_bus;
        TelemetryTarget* telemetry;
        USBTarget* usb_target;

        // Control Config
        ConfigManager config_manager;
        maxwell_config config;
        test_config t_config;

        bool pos_homed = false;
        float homed_position_offset = 0.0;
        FOC foc;
        float input_voltage = 24.0;
        limits_struct limits = { // Absorb into config struct?
            .max_voltage = 23.0,
            .max_current = 10.0,
            .align_voltage = 1.5,
            .max_velocity = 300.0, // in radians per second
            .max_position = _2PI * 70/2 // 50mm stroke, with 2mm lead
        };
        // Motor params
        float kv_rating = 330;
        float flux_linkage;
        float Rs = 90e-3;
        float L  = 222e-6; // Average of Ld and Lq
        float Ld = 186e-6;
        float Lq = 257e-6;

        float prev_velocity = 0.0;
        float velocity = 0.0;
        float theta = 0.0;
        float theta_est = 0.0;
        float integral_observer = 0.0;
        uint32_t prev_observer_micros = 0;

        int bemf_full_elec_rotations = 0;
        int bemf_full_mech_rotations = 0;
        float prev_bemf_angle = 0.0;
        float raw_bemf_angle = 0.0;
        float absolute_bemf_angle = 0.0;

        MOTOR_DIRECTION motor_direction = MOTOR_DIRECTION::CCW;
        pwm_3x_struct* pwm_3x;
        uint32_t pwm_frequency = 20000;
        // float align_voltage = 2; // V
        float offset = 0.1; // V
        int _csa_gain = 20;

        static Maxwell* instance;

        Maxwell(); // Constructor

        void setup();

        void init_pwm_3x();

        void init_pwm_6x();

        void sync_pwm();

        void set_pwm(uint32_t Ua, uint32_t Ub, uint32_t Uc, uint32_t resolution);

        void set_phase_voltages(float Va, float Vb, float Vc);

        void set_phase_voltages(const dq_struct &command_dq);

        void set_phase_voltages(const dq_struct &command_dq, float theta);

        void all_off();

        void sensor_offset_calibration();

        float find_resistance(float voltage);

        float find_flux_linkage();

        float find_inductance(float v_d, float v_q);

        float estimate_bemf_angle();

        void update_observer(float angle_meas, uint32_t current_time_us);

        void motor_calibration();

        void load_control_config();

        void motor_control();

        void bldc_control();



    };
}
#endif //MAXWELL_MAXWELL_H
