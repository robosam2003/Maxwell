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
// #include "PWMInput.h"
#include "stm32f4xx_hal_uart.h"
#include <cstring> // For strlen

Maxwell::Maxwell maxwell;


controlConfig config = {
    CONTROL_MODE::POSITION,
    SENSOR_TYPE::MAGNETIC,
    TORQUE_CONTROL_MODE::VOLTAGE,
    MOTOR_TYPE::BLDC,
    COMMAND_SOURCE::PWM
};

// PWMInput pwm_input(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Start");
    pinMode(GREEN_LED_PIN, OUTPUT);

    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);



    maxwell.setup();
    maxwell.init_pwm_3x();
    // maxwell.set_phase_voltages(-3, 0, 3);
    // //
    // pwm_input.set_callback(pwm_callback);
    // maxwell.pwm_input = &pwm_input;
    // maxwell.driver->perform_current_sense_calibration();
    // maxwell.driver->current_sensors->calibrate_offsets();
    maxwell.driver->clear_fault();

    // maxwell.foc_init_sequence();
    analogReadResolution(12);
}



void loop() {
    digitalToggle(GREEN_LED_PIN);
    //
    delay(10);

    maxwell.driver->set_pwm_mode(DRV8323::PWM_3x);
    Serial.println(maxwell.driver->read_reg(DRV8323::REGISTER::DRIVER_CONTROL));
    Serial.println(maxwell.driver->get_fault_status_1_string());
    Serial.println(maxwell.driver->get_fault_status_2_string());
    Serial.println();


    // Serial.println(maxwell.driver->read_reg(DRV8323::REGISTER::GATE_DRIVE_HS));
    // Serial.println(maxwell.driver->read_reg(DRV8323::REGISTER::GATE_DRIVE_LS));
    // Serial.println(maxwell.driver->read_reg(DRV8323::REGISTER::DRIVER_CONTROL));



    // Serial.println(driver.read_reg(DRV8323::REGISTER::GATE_DRIVE_HS));
    // Serial.println(driver.read_reg(DRV8323::REGISTER::GATE_DRIVE_LS));
    // Serial.println(driver.read_reg(DRV8323::REGISTER::DRIVER_CONTROL));
    // Serial.println(driver.read_reg(DRV8323::REGISTER::CSA_CONTROL));

    // char strbuf[50]; sprintf(strbuf, "%lu, %lu, %lu",
    //                         analogRead(CURR_SENSE_A_PIN),
    //                         analogRead(CURR_SENSE_B_PIN),
    //                         analogRead(CURR_SENSE_C_PIN));
    // Serial.println(strbuf);


    // maxwell.sinusoidal_position_control();
    // maxwell.foc_position_control();

    // maxwell.voltage_position_control();

    // maxwell.voltage_torque_control();
    // maxwell.dc_current_torque_control();
    // maxwell.foc_current_torque_control();


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