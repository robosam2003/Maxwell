#include <Arduino.h>
#include "pin_definitions.h"
#include "Maxwell.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "PWMInput.h"
// #include "AS5047P.h"
#include "current_sensors.h"
// #include "Maxwell.cpp"


Maxwell::Maxwell maxwell;
// Maxwell::HallSensor hall_sensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN);
PWMInput pwm_input(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);

// CurrentSensors current_sensors(DRV8323_CURR_SENSE_A_PIN,
//                                 DRV8323_CURR_SENSE_B_PIN,
//                                 DRV8323_CURR_SENSE_C_PIN,
//                                 DRV8323::CSA_GAIN::GAIN_5_V_V);

// void hall_a_callback() {
//     hall_sensor.callback_a();
// }
// void hall_b_callback() {
//     hall_sensor.callback_b();
// }
// void hall_c_callback() {
//     hall_sensor.callback_c();
// }
// void pwm_callback() {
//     pwm_input.pwm_callback();
// }


void setup() {
    // xTaskCreate(
    //     drive_motor,
    //     "drive motor",
    //     1000,
    //     NULL,
    //     1,
    //     NULL
    // );

    Serial.begin(921600);
    analogReadResolution(12);

    maxwell.setup();
    maxwell.driver->enable(true);
    // hall_sensor.setup(true, hall_a_callback, hall_b_callback, hall_c_callback);
    // pwm_input.set_callback(pwm_callback);
    // maxwell.hall_sensor = &hall_sensor;
    maxwell.pwm_input = &pwm_input;
    // Maxwell::all_off();
    maxwell.driver->enable(true);
    maxwell.driver->perform_current_sense_calibration();
    maxwell.driver->current_sensors->calibrate_offsets();
    // maxwell.driver->enable(false);


    // digitalWrite(DRV8323_LO_A_PIN, HIGH);
    // digitalWrite(DRV8323_LO_B_PIN, HIGH);
    // digitalWrite(DRV8323_LO_C_PIN, HIGH);
    // maxwell.driver->perform_current_sense_calibration();

}



void fake_state_feedback() {
    String text = "";
    double angle = fmod(maxwell.encoder->get_angle() * POLE_PAIRS_6374, 2*PI);
    text += static_cast<String>(angle);
    text += "//";
    maxwell.driver->current_sensors->read();
    // Perform the clarke transform on the currents
    double currents[3] = {maxwell.driver->current_sensors->get_current_a(),
                          maxwell.driver->current_sensors->get_current_b(),
                          maxwell.driver->current_sensors->get_current_c()};
    double c_current = -1 * (currents[0] + currents[1]); // Kirchoff's current law
    double I_alpha = 1.5 * currents[0];
    double I_beta = sqrt(3.0)/2 * (currents[1] - currents[2]);
    double magnitude = sqrt(I_alpha*I_alpha + I_beta*I_beta);

    text += static_cast<String>(I_alpha); text += "/";
    text += static_cast<String>(I_beta); text += "/";
    text += static_cast<String>(magnitude); text += "/";
    text += static_cast<String>(0.0); text += "/";

    // text += static_cast<String>(maxwell.driver->current_sensors->get_current_a()); text += "/";
    // text += static_cast<String>(maxwell.driver->current_sensors->get_current_b()); text += "/";
    // text += static_cast<String>(maxwell.driver->current_sensors->get_current_c()); text += "/";
    // text += static_cast<String>(0.0); text += "/";
    // text += static_cast<String>(analogRead(DRV8323_CURR_SENSE_A_PIN)); text += "/";
    // text += static_cast<String>(analogRead(DRV8323_CURR_SENSE_B_PIN)); text += "/";
    // text += static_cast<String>(analogRead(DRV8323_CURR_SENSE_C_PIN)); text += "/";
    // text += static_cast<String>(0.0); text += "/";
    text += "////";

    int checksum = 0;
    for (int i = 0; i < text.length(); i++) {
        checksum += text[i]; // add the ASCII value of each character
    }
    checksum = checksum % 256;
    text += static_cast<String>(checksum);
    Serial.println(text);
    delay(10);
}

void loop() {
    maxwell.foc_position_control();
    // delay(5000);

    // fake_state_feedback();


}
