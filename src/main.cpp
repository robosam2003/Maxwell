#include <Arduino.h>
#include "pin_definitions.h"
#include "Maxwell.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "PWMInput.h"
// #include "AS5047P.h"
#include "current_sensors.h"


Maxwell::Maxwell maxwell;
Maxwell::HallSensor hall_sensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN);
PWMInput pwm_input(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);



void hall_a_callback() {
    hall_sensor.callback_a();
}
void hall_b_callback() {
    hall_sensor.callback_b();
}
void hall_c_callback() {
    hall_sensor.callback_c();
}
void pwm_callback() {
    pwm_input.pwm_callback();
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
    maxwell.driver->enable(true);
    hall_sensor.setup(true, hall_a_callback, hall_b_callback, hall_c_callback);
    pwm_input.set_callback(pwm_callback);
    maxwell.hall_sensor = &hall_sensor;
    maxwell.pwm_input = &pwm_input;

}

void loop() {
    // current_sensors.read();
    // Serial.print(current_sensors.get_current_a()); Serial.print(", ");
    // Serial.print(current_sensors.get_current_b()); Serial.print(", ");
    // Serial.println(current_sensors.get_current_c());
    // Serial.println(encoder.get_angle());
    // Serial.println(pwm_input.read_percentage());
    maxwell.drive_hall_velocity();
    // maxwell.driver->current_sensors->read();


    // delay(2000);
    // vTaskDelay(2000);
    // Serial.println(pwm_input.read_percentage());
}

