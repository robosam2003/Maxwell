

#define MAXWELL_STUDIO_OUTPUT


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
#include <string>

#include "pin_definitions.h"
#include "Maxwell.h"
#include "PWMInput.h"
#include "stm32f4xx_hal_uart.h"
#include <cstring> // For strlen

UART_HandleTypeDef huart1;


Maxwell::Maxwell maxwell;
PWMInput pwm_input(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);

void pwm_callback() {
    pwm_input.pwm_callback();
}

void setup() {
    Serial.begin(921600);
    Serial.println("Setup Start");

    maxwell.setup();
    maxwell.init_pwm_3x();

    pwm_input.set_callback(pwm_callback);
    maxwell.pwm_input = &pwm_input;
    maxwell.driver->perform_current_sense_calibration();
    maxwell.driver->current_sensors->calibrate_offsets();
    maxwell.driver->clear_fault();

    // maxwell.foc_init_sequence();
}

#define SERIAL_FEEDBACK_ENABLED



void loop() {


    // maxwell.sinusoidal_position_control();
    // maxwell.foc_position_control();

    // maxwell.voltage_position_control();

    // maxwell.voltage_torque_control();
    // maxwell.dc_current_torque_control();
    maxwell.foc_current_torque_control();


    // Blink
    // digitalWrite(GREEN_LED_PIN, HIGH);
    // delay(100);
    // digitalWrite(GREEN_LED_PIN, LOW);
    // delay(100);

    // Serial.println(maxwell.driver->get_fault_status_1_string());

    // maxwell.voltage_position_control();





    // maxwell.encoder->update();
    // maxwell.rotor_position_frame.values = {maxwell.encoder->get_angle()};
    // send_frame(maxwell.rotor_position_frame);
    // Serial.println(maxwell.encoder->get_angle());
    // delay(10);

    // maxwell.foc_current_torque_control();

    // Serial.println(maxwell.encoder->get_angle());
    // delay(10);
    // Serial.println(maxwell.driver->get_fault_status_1_string());
    // Serial.println(maxwell.driver->get_fault_status_2_string());

}
#endif