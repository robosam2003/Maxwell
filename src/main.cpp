#define SERIAL_FEEDBACK_ENABLED
#define MAXWELL_STUDIO_OUTPUT
#define HARDWARE_V2_0

// -----------------------------------------------------------------------------------
#include <Arduino.h>
#include "Maxwell.h"


Maxwell::Maxwell maxwell;

float t = 2.123;
controlConfig config = {
    COMMAND_SOURCE::PWM,
    TELEMETRY_TARGET::TELEMETRY_USB,
    MOTOR_TYPE::BLDC,
    CONTROL_MODE::TORQUE,
    SENSOR_TYPE::MAGNETIC,
    TORQUE_CONTROL_MODE::VOLTAGE,
};


void setup() {
    pinMode(GREEN_LED_PIN, OUTPUT);
    delay(500);

    maxwell.config = config;

    maxwell.setup();
    // maxwell.init_pwm_3x();
    // maxwell.set_phase_voltages(0, 0, 0);
    // maxwell.current_sensors->calibrate_offsets();
    // maxwell.driver->clear_fault();
    //
    // maxwell.foc_init_sequence();
}


void loop() {
    maxwell.load_control_config();
}