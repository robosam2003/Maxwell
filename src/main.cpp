#include <Arduino.h>
#include "pin_definitions.h"
#include "Maxwell.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "PWMInput.h"



Maxwell::Maxwell maxwell;
Maxwell::HallSensor hall_sensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN);



void hall_a_callback() {
    hall_sensor.callback_a();
}
void hall_b_callback() {
    hall_sensor.callback_b();
}
void hall_c_callback() {
    hall_sensor.callback_c();
}


void setup() {
    // xTaskCreate(
    //     drive_motor,
    //     "drive motor",
    //     1000,
    //     NULL,
    //     1,
    //     NULL
    // );

    Serial.begin(115200);
    maxwell.setup();
    hall_sensor.setup(true, hall_a_callback, hall_b_callback, hall_c_callback);
    maxwell.hall_sensor = &hall_sensor;
    // vTaskStartScheduler();
}

void loop() {
    maxwell.drive_hall_velocity(7000); // 2 seconds
    // delay(2000);
    // vTaskDelay(2000);
    // Serial.println(pwm_input.read_percentage());
}

