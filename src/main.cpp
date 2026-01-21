#define SERIAL_FEEDBACK_ENABLED
#define MAXWELL_STUDIO_OUTPUT
#define HARDWARE_V2_0


// #define TEST
#ifdef TEST
#include "Arduino.h"
#include "config.h"


float_frame test_frame = {
    .name = "Measured Current",
    .values = {1.0, 2.0, 3.1}
};


void setup() {
    Serial.begin(115200);
}

void loop() {
    send_frame(test_frame);
}
#endif

#ifndef TEST
// -----------------------------------------------------------------------------------
#include <Arduino.h>
#include "SPI.h"
//
#include "pin_definitions.h"
#include "Maxwell.h"

#include "stm32f4xx.h"
#include <cstring> // For strlen

Maxwell::Maxwell maxwell;
#define SERIAL_FEEDBACK_ENABLED

controlConfig config = {
    CONTROL_MODE::POSITION,
    SENSOR_TYPE::MAGNETIC,
    TORQUE_CONTROL_MODE::VOLTAGE,
    MOTOR_TYPE::BLDC,
    COMMAND_SOURCE::PWM,
    TELEMETRY_DESTINATION::TELEMETRY_USB
};

uint8_t buffer[3];

void setup() {
    pinMode(GREEN_LED_PIN, OUTPUT);

    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);

    maxwell.setup();
    maxwell.init_pwm_3x();
    maxwell.set_phase_voltages(0, 0, 0);
    // //
    // pwm_input.set_callback(pwm_callback);
    maxwell.current_sensors->calibrate_offsets();
    maxwell.driver->clear_fault();

    maxwell.foc_init_sequence();
}


void loop() {
    digitalToggle(GREEN_LED_PIN);
    // maxwell.telemetry->send({TELEMETRY_PACKET_TYPE::GENERAL, {1.234f, 3.1415f, 2.718f}});
    // delay(10);
    // maxwell.voltage_torque_control();
    maxwell.foc_current_torque_control();
    // maxwell.encoder->update();
    // maxwell.telemetry->send({ROTOR_POSITION, {maxwell.encoder->get_angle()}});
    // delay(10);

}
#endif