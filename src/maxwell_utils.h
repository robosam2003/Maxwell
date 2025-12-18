//
// Created by robos on 18/12/2025.
//

#ifndef MAXWELL_MAXWELL_UTILS_H
#define MAXWELL_MAXWELL_UTILS_H
#include <cstdint>
#include "current_sensors.h"

/*  Maxwell control architecture
 *  The main control loop of Maxwell requires a single configuration / schema to be input to it to run things
 *   control_mode | Distinguishes motor control modes
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
 *      UART
 *      CAN
 *
 */

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

enum TORQUE_CONTROL_MODE {
    VOLTAGE,
    CURRENT
};

enum MOTOR_TYPE {
    BLDC,
    DC
};

enum COMMAND_SOURCE {
    PWM,
    UART,
    CAN,
    USB
};

struct sensorTypeSchema {
public:
    uint8_t sensor_type;
};

struct sensoredSchema : public sensorTypeSchema {
public:

};




struct motorTypeSchema {
public:
    uint32_t pole_pairs;
    uint32_t phase_resistance;
    uint32_t phase_inductance;
};

struct DCmotorSchema : public motorTypeSchema {
public:

};

struct BLDCmotorSchema : public motorTypeSchema {
public:

};

struct controlConfig {
    CONTROL_MODE control_mode;
    SENSOR_TYPE sensor_type;
    TORQUE_CONTROL_MODE torque_control_mode;
    MOTOR_TYPE motor_type;
    COMMAND_SOURCE command_source;


};





#endif //MAXWELL_MAXWELL_UTILS_H