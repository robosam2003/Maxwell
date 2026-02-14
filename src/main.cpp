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

#include "stm32f4xx_hal_can.h"

#include "STM32_CAN.h"


Maxwell::Maxwell maxwell;
#define SERIAL_FEEDBACK_ENABLED

STM32_CAN can(CAN1_RX_PIN, CAN1_TX_PIN);


CAN_message_t message;



#define maxwell_number 0


controlConfig config = {
    CONTROL_MODE::POSITION,
    SENSOR_TYPE::MAGNETIC,
    TORQUE_CONTROL_MODE::VOLTAGE,
    MOTOR_TYPE::BLDC,
    COMMAND_SOURCE::PWM,
    TELEMETRY_DESTINATION::TELEMETRY_USB
};

CAN_message_t tx_message = {
    .id = maxwell_number,
    .len = 3,
    .buf = {maxwell_number, 1, 2}
};


bool initer = maxwell_number;

uint8_t buffer[3];

void setup() {
    pinMode(GREEN_LED_PIN, OUTPUT);
    delay(500);

    can.begin();

    can.setBaudRate(500000);




    // pinMode(PC13, OUTPUT);
    // digitalWrite(PC13, HIGH);
    //
    // maxwell.setup();
    // maxwell.init_pwm_3x();
    // maxwell.set_phase_voltages(0, 0, 0);
    // // //
    // // pwm_input.set_callback(pwm_callback);
    // maxwell.current_sensors->calibrate_offsets();
    // maxwell.driver->clear_fault();
    //
    // maxwell.foc_init_sequence();
}


void loop() {
    if (initer) { // the one that inits
        delay(1000);
        can.write(tx_message);
        initer = false;
    }
    // if (can.read(message)) {
    //     if (message.id != maxwell_number) {
    //         for (int i=0; i<message.len; i++) {
    //             buffer[i] = message.buf[i];
    //         }
    //         Serial.print("Received message at Timestamp"); Serial.print(message.timestamp);
    //         Serial.print(": ID=0x"); Serial.print(message.id, HEX); Serial.print(", Data = ");
    //         for (int i=0; i<message.len; i++) {
    //             Serial.print(buffer[i], HEX);
    //             Serial.print(" ");
    //         }
    //         Serial.println();
    //         digitalToggle(GREEN_LED_PIN);
    //     }
    // }
    // delay(10);
    // can.write(tx_message);
    // delay(100);

    // Ping pong test
    if (can.read(message)) {
        if (message.id != maxwell_number) { // It's from the other maxwell
            // delay(200);
            can.write(tx_message);
        }
        digitalToggle(GREEN_LED_PIN);
    }

}


#endif