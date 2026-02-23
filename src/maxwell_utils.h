//
// Created by robos on 18/12/2025.
//

#ifndef MAXWELL_MAXWELL_UTILS_H
#define MAXWELL_MAXWELL_UTILS_H
#include <cstdint>
#include "current_sensors.h"

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


struct motorTypeSchema {
    enum MOTOR_DIRECTION {
        CW = 1,
        CCW = -1
    };

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

struct controlConfig {
    COMMAND_SOURCE command_source;
    TELEMETRY_TARGET telemetry_target;
    MOTOR_TYPE motor_type;
    CONTROL_MODE control_mode;
    SENSOR_TYPE sensor_type;
    TORQUE_CONTROL_MODE torque_control_mode;
    BLDCmotorSchema motor;
};





// #define DEBUG_U32(a) ITM->PORT[x].u32 = *(uint32_t*)&a; // type-punn to desired size: sizeof(float) = sizeof(uint32_t)




#endif //MAXWELL_MAXWELL_UTILS_H