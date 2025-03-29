
/*
#include <Arduino.h>
#include "pin_definitions.h"
#include "Maxwell.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "timers.h"
// #include "PWMInput.h"
// // #include "AS5047P.h"
// #include "current_sensors.h"
#include <stm32f4xx.h>

Maxwell::Maxwell maxwell;
// // Maxwell::HallSensor hall_sensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN);
// PWMInput pwm_input(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);

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
#define STM32F4xx
#define STM32F405xx
#define STM32F405RGT6
#define PWM_RESOLUTION 8

uint8_t pin_a = DRV8323_HI_A_PIN;
uint8_t pin_b = DRV8323_HI_B_PIN;
uint8_t pin_c = DRV8323_HI_C_PIN;
// SOLENOID = PA10 = TIM1_CH3
// pin_a = PB1  = TIM3_CH4
// pin_b = PA3  = TIM2_CH4
// pin_c = PA1  = TIM5_CH2

// SimpleFOC CONFIG:
// pin_a = PB1 = TIM3_CH4
// pin_b = PA3 = TIM2_CH4
// pin_c = PA1 = TIM2_CH2

// Suposed new config
// pin_a = PB1 = TIM1_CH3N
// pin_b = PA3 = TIM2_CH4
// pin_c = PA1 = TIM5_CH2



TIM_TypeDef *Instance_a = TIM1; //(TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_a), PinMap_PWM);
uint32_t channel_a = 3; //STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_a), PinMap_PWM));
HardwareTimer *TIM_A = new HardwareTimer(Instance_a);
//
TIM_TypeDef *Instance_b = TIM2; //(TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_b), PinMap_PWM);
uint32_t channel_b = 4; //STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_b), PinMap_PWM));
HardwareTimer *TIM_B = new HardwareTimer(Instance_b);
//
TIM_TypeDef *Instance_c = TIM5;//(TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_c), PinMap_PWM);
uint32_t channel_c = 2; //STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_c), PinMap_PWM));
HardwareTimer *TIM_C = new HardwareTimer(Instance_c);

uint32_t pwm_freq = 1000 * 2;


struct PWM_STATE {
    uint8_t PIN_A;
    uint8_t PIN_B;
    uint8_t PIN_C;
    uint32_t duty_a;
    uint32_t duty_b;
    uint32_t duty_c;
    uint8_t timer_period_a;
    uint8_t timer_period_b;
    uint8_t timer_period_c;
};

PWM_STATE pwm_state = {1, 0, 0, 0, 0, 0, 0};

void print_state() {
    Serial.print(pwm_state.PIN_A+3); Serial.print(" ");
    Serial.print(pwm_state.PIN_B+2); Serial.print(" ");
    Serial.print(pwm_state.PIN_C+1); Serial.print(" ");
    Serial.print(pwm_state.timer_period_a); Serial.print(" ");
    Serial.print(pwm_state.timer_period_b); Serial.print(" ");
    Serial.print(pwm_state.timer_period_c); Serial.print(" ");


    // print counter values
    float max_counter_value = static_cast<float>(TIM_A->getHandle()->Init.Period);
    float counter_a = static_cast<float>(TIM_A->getHandle()->Instance->CNT) / max_counter_value;
    // float counter_b = static_cast<float>(TIM_B->getHandle()->Instance->CNT) / max_counter_value;
    // float counter_c = static_cast<float>(TIM_C->getHandle()->Instance->CNT) / max_counter_value;
    // Serial.print(max_counter_value); Serial.print(" ");
    Serial.print(counter_a); Serial.print(" ");
    // Serial.print(counter_b); Serial.print(" ");
    // Serial.print(counter_c); Serial.print(" ");

    // print duty values
    float max_duty_value = pow(2, PWM_RESOLUTION) - 1;
    float duty_a = static_cast<float>(pwm_state.duty_a) / max_duty_value;
    // float duty_b = static_cast<float>(pwm_state.duty_b) / max_duty_value;
    // float duty_c = static_cast<float>(pwm_state.duty_c) / max_duty_value;
    // Serial.print(max_duty_value); Serial.print(" ");
    Serial.print(duty_a); Serial.print(" ");
    // Serial.print(duty_b); Serial.print(" ");
    // Serial.print(duty_c); Serial.print(" ");

    Serial.print("\n");
}

void Update_A_callback() {
    // pwm_state.PIN_A = 1;
    pwm_state.timer_period_a = (pwm_state.timer_period_a)? 0 : 1;
    // print_state();
}
void Compare_A_callback() {
    pwm_state.PIN_A = (pwm_state.PIN_A)? 0 : 1;
    // digitalWriteFast(static_cast<PinName>(pin_a), pwm_state.PIN_A);
    digitalWrite(pin_a, pwm_state.PIN_A);
    // pwm_state.PIN_A = 0;
    // print_state();
}

void Update_B_callback() {
    // pwm_state.PIN_B = 1;
    pwm_state.timer_period_b = (pwm_state.timer_period_b)? 0 : 1;

    // print_state();
}
void Compare_B_callback() {
    pwm_state.PIN_B = (pwm_state.PIN_B)? 0 : 1;
    digitalWrite(pin_b, pwm_state.PIN_B);
    // digitalWrite(pin_b, pwm_state.PIN_B);
    // pwm_state.PIN_B = 0;
    // print_state();
}

void Update_C_callback() {
    // pwm_state.PIN_C = 1;
    pwm_state.timer_period_c = (pwm_state.timer_period_c)? 0 : 1;
    // print_state();
}
void Compare_C_callback() {
    pwm_state.PIN_C = (pwm_state.PIN_C)? 0 : 1;
    digitalWrite(pin_c, pwm_state.PIN_C);
    // pwm_state.PIN_C = 0;
    // print_state();
}

void init_pwm() {
    uint32_t COUNTER_MODE = TIM_COUNTERMODE_CENTERALIGNED3;
    TimerModes_t PWM_MODE = TIMER_OUTPUT_DISABLED; //TIMER_OUTPUT_COMPARE_PWM1;

    TIM_A->pause();
    TIM_A->setMode(channel_a, PWM_MODE, NC);
    TIM_A->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_A->getHandle()->Init.RepetitionCounter = 1;
    TIM_A->setPWM(channel_a, pin_a, pwm_freq, 0);
    TIM_A->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_A->setCaptureCompare(channel_a, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));
    TIM_A->attachInterrupt(Update_A_callback);
    TIM_A->attachInterrupt(channel_a, Compare_A_callback);

    TIM_B->pause();
    TIM_B->setMode(channel_b, PWM_MODE, NC);
    TIM_B->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_B->getHandle()->Init.RepetitionCounter = 1;
    TIM_B->setPWM(channel_b, pin_b, pwm_freq, 0);
    TIM_B->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_B->setCaptureCompare(channel_b, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));
    TIM_B->attachInterrupt(Update_B_callback);
    TIM_B->attachInterrupt(channel_b, Compare_B_callback);

    TIM_C->pause();
    TIM_C->setMode(channel_c, PWM_MODE, NC);
    TIM_C->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_C->getHandle()->Init.RepetitionCounter = 1;
    TIM_C->setPWM(channel_c, pin_c, pwm_freq, 0);
    TIM_C->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_C->setCaptureCompare(channel_c, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));
    TIM_C->attachInterrupt(Update_C_callback);
    TIM_C->attachInterrupt(channel_c, Compare_C_callback);




    HAL_TIM_Base_Init(TIM_A->getHandle());
    HAL_TIM_Base_Init(TIM_B->getHandle());
    HAL_TIM_Base_Init(TIM_C->getHandle());

}

void set_pwm(uint32_t Ua, uint32_t Ub, uint32_t Uc) {
    pwm_state.duty_a = Ua;
    pwm_state.duty_b = Ub;
    pwm_state.duty_c = Uc;
    constrain(Ua, 0.0, pow(2, PWM_RESOLUTION));
    constrain(Ub, 0.0, pow(2, PWM_RESOLUTION));
    constrain(Uc, 0.0, pow(2, PWM_RESOLUTION));
    // constrain(Ua, 0, 100);
    // constrain(Ub, 0, 100);
    // constrain(Uc, 0, 100);
    TIM_A->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_B->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_C->setOverflow(pwm_freq, HERTZ_FORMAT);
    //
    TIM_A->setCaptureCompare(channel_a, Ua, RESOLUTION_8B_COMPARE_FORMAT);
    TIM_B->setCaptureCompare(channel_b, Ub, RESOLUTION_8B_COMPARE_FORMAT);
    TIM_C->setCaptureCompare(channel_c, Uc, RESOLUTION_8B_COMPARE_FORMAT);

    // TIM_A->setPWM(channel_a, pin_a, pwm_freq, Ua);
    // TIM_B->setPWM(channel_b, pin_b, pwm_freq, Ub);
    // TIM_C->setPWM(channel_c, pin_c, pwm_freq, Uc);


}

void sync_pwm() {
    TIM_A->pause();
    TIM_A->refresh();
    TIM_B->pause();
    TIM_B->refresh();
    TIM_C->pause();
    TIM_C->refresh();

    TIM_A->resume();
    TIM_B->resume();
    TIM_C->resume();
}


void setup() {
    init_pwm();
    sync_pwm();

    digitalWrite(pin_b, LOW);
    digitalWrite(pin_c, LOW);

    delay(1000);
    maxwell.setup();
    maxwell.driver->enable(true);
    maxwell.driver->perform_current_sense_calibration();
    Serial.begin(115200);
    Serial.println("MAXWELL SETUP COMPLETE");
    // delay(5000);
}


void loop() {


    set_pwm(127, 50, 10);
    // set_pwm(10, 50, 95);
    print_state();
    // Serial.print((uint32_t)TIM_A->getHandle()->Instance, HEX); ; Serial.print(" ");
    // Serial.print(channel_a); Serial.print(" ");
    // Serial.println(maxwell.driver->get_fault_status_1_string());
    // Serial.println(maxwell.driver->get_fault_status_2_string());
    // Serial.print(channel_b); Serial.print(" ");
    // Serial.print(channel_c); Serial.print(" ");
    // Serial.println();
    delay(10);

    // analogWrite(pin_c, 10);
    // analogWrite(pin_b, 50);
    // analogWrite(pin_a, 10);



}
*/



// -----------------------------------------------------------------------------------
#include <Arduino.h>
#include "pin_definitions.h"
#include "Maxwell.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "PWMInput.h"
// #include "AS5047P.h"
#include "current_sensors.h"
#include <stm32f4xx.h>

Maxwell::Maxwell maxwell;
// // Maxwell::HallSensor hall_sensor(HALL_A_PIN, HALL_B_PIN, HALL_C_PIN);
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
void pwm_callback() {
    pwm_input.pwm_callback();
}

void setup() {
    delay(100);
    // analogWriteResolution(8);
    // analogWriteFrequency(1000);
    Serial.begin(921600);

    maxwell.setup();
    maxwell.init_pwm();
    // maxwell.sync_pwm();
    // hall_sensor.setup(true, hall_a_callback, hall_b_callback, hall_c_callback);
    // maxwell.hall_sensor = &hall_sensor;
    pwm_input.set_callback(pwm_callback);
    maxwell.pwm_input = &pwm_input;
    maxwell.driver->perform_current_sense_calibration();
    // maxwell.driver->current_sensors->calibrate_offsets();
}



void loop() {
    // analogWrite(DRV8323_HI_A_PIN, 70);
    // analogWrite(DRV8323_HI_B_PIN, 0);
    // analogWrite(DRV8323_HI_C_PIN, 0);
    // delay(100);
    // analogWrite(DRV8323_HI_A_PIN, 0);
    // analogWrite(DRV8323_HI_B_PIN, 0);
    // analogWrite(DRV8323_HI_C_PIN, 0);
    // delay(2000);
    // Serial.println(`

    maxwell.svpwm_position_control();
    // Serial.println(maxwell.driver->get_fault_status_1_string());
    // Serial.println(maxwell.driver->get_fault_status_2_string());
    // Serial.println(pwm_input.read_percentage());
}
