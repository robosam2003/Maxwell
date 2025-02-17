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
Maxwell::HallSensor hall_sensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN);
PWMInput pwm_input(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);

// CurrentSensors current_sensors(DRV8323_CURR_SENSE_A_PIN,
//                                 DRV8323_CURR_SENSE_B_PIN,
//                                 DRV8323_CURR_SENSE_C_PIN,
//                                 DRV8323::CSA_GAIN::GAIN_5_V_V);

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
    analogReadResolution(12);

    maxwell.setup();
    maxwell.driver->enable(true);
    hall_sensor.setup(true, hall_a_callback, hall_b_callback, hall_c_callback);
    pwm_input.set_callback(pwm_callback);
    maxwell.hall_sensor = &hall_sensor;
    maxwell.pwm_input = &pwm_input;
    // Maxwell::all_off();
    maxwell.driver->enable(true);
    maxwell.driver->perform_current_sense_calibration();
    maxwell.driver->current_sensors->calibrate_offsets();



    digitalWrite(DRV8323_LO_A_PIN, HIGH);
    digitalWrite(DRV8323_LO_B_PIN, HIGH);
    digitalWrite(DRV8323_LO_C_PIN, HIGH);
    // maxwell.driver->perform_current_sense_calibration();
}

// typedef struct Stm32CurrentSenseParams {
//     int pins[3] = {(int)NOT_SET};
//     float adc_voltage_conv;
//     ADC_HandleTypeDef* adc_handle = NP;
//     HardwareTimer* timer_handle = NP;
// } Stm32CurrentSenseParams;
//
//
// float _readADCVoltageLowSide(const int pin, const void* cs_params){
//     for(int i=0; i < 3; i++){
//             return adc_val[_adcToIndex(((Stm32CurrentSenseParams*)cs_params)->adc_handle)][i] * ((Stm32CurrentSenseParams*)cs_params)->adc_voltage_conv;
//             }
//         }
//     }
//     return 0;
// }

void loop() {
    // maxwell.state_feedback();
    String text = "";
    text += "//";
    maxwell.driver->current_sensors->read();
    text += static_cast<String>(maxwell.driver->current_sensors->get_current_a()); text += "/";
    text += static_cast<String>(maxwell.driver->current_sensors->get_current_b()); text += "/";
    text += static_cast<String>(maxwell.driver->current_sensors->get_current_c()); text += "/";
    text += static_cast<String>(0.0); text += "/";
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

