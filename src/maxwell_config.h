//
// Created by robos on 18/12/2025.
//

#ifndef MAXWELL_MAXWELL_CONFIG_H
#define MAXWELL_MAXWELL_CONFIG_H
#include <cstdint>

/*  Maxwell control architecture
 *  The main control loop of Maxwell requires a single configuration / schema to be input to it to run things
 *
 *  control_mode | Distinguishes motor control modes
 *      POSITION
 *      VELOCITY
 *      TORQUE
 *  sensor_type | Distinguishes position sensor
 *      SENSORLESS
 *      MAGNETIC
 *      HALL_SENSOR
 *      A_B
 *  torque_control_mode | Torque control mode being used
 *      VOLTAGE
 *      CURRENT
 *  motor_type | The type of motor being controlled
 *      BLDC
 *      DC
 *  command_source | Where to read the reference from
 *      PWM
 *      USB
 *      UART
 *      CAN
 *  telemetry_destination | Where to send telemetry data to
 *    UART
 *    CAN
 *    USB
 */

enum MOTOR_TYPE {
    BLDC,
    DC
};

enum CONTROL_MODE {
    POSITION,
    VELOCITY,
    TORQUE,
};

enum SENSOR_TYPE {
    SENSORLESS,
    MAGNETIC,
    HALL_SENSOR,
    A_B,
};

enum SENSOR_LOCATION {
    INTERNAL,
    EXTERNAL_PORT
};

enum TORQUE_CONTROL_MODE {
    VOLTAGE,
    CURRENT
};

enum COMMAND_SOURCE {
    PWM,
    UART,
    CAN,
    USB
};

enum TELEMETRY_TARGET {
    TELEMETRY_UART,
    TELEMETRY_CAN,
    TELEMETRY_USB
};

struct pid_config {
    float kp;
    float ki;
    float kd;
    float max_output;
    float max_integral;
};

struct lpf_config {
    bool enabled;
    double cutoff_frequency;
};
enum MOTOR_DIRECTION {
    MOTOR_CW = 1,
    MOTOR_CCW = -1
};

struct motorTypeSchema {


public:
    MOTOR_DIRECTION direction;
};

struct DCmotorSchema : public motorTypeSchema {
public:

};

struct BLDCmotorSchema : public motorTypeSchema {
public:
    uint32_t pole_pairs;
    uint32_t phase_resistance;
    uint32_t phase_inductance;
};

struct maxwell_config {

    // Top level configuration
    COMMAND_SOURCE command_source;
    TELEMETRY_TARGET telemetry_target;
    MOTOR_TYPE motor_type;
    CONTROL_MODE control_mode;
    SENSOR_TYPE sensor_type;
    TORQUE_CONTROL_MODE torque_control_mode;
    SENSOR_LOCATION sensor_location;
    uint32_t pole_pairs;
    float encoder_offset = 0;

    // PID loops
    pid_config d_pid_config;
    pid_config q_pid_config;
    pid_config velocity_pid_config;
    pid_config position_pid_config;
    // LPF configs
    lpf_config d_lpf_config;
    lpf_config q_lpf_config;
    lpf_config velocity_lpf_config;
    lpf_config input_lpf_config;
};




#endif //MAXWELL_MAXWELL_CONFIG_H