#include <Arduino.h>
#include "pin_definitions.h"
// #include "Maxwell.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "timers.h"
// #include "PWMInput.h"
// // #include "AS5047P.h"
// #include "current_sensors.h"
#include <stm32f4xx.h>

#define STM32F405xx
// Maxwell::Maxwell maxwell;
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

uint8_t pin_a = DRV8323_HI_A_PIN;
uint8_t pin_b = DRV8323_HI_B_PIN;
uint8_t pin_c = DRV8323_HI_C_PIN;
// pin_a = PB1  = TIM3_CH4
// pin_b = PA3  = TIM5_CH4
// pin_c = PA1  = TIM2_CH2
TIM_TypeDef *Instance_a = TIM3;
uint8_t channel_a = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_a), PinMap_PWM));
HardwareTimer *TIM_A = new HardwareTimer(Instance_a);

TIM_TypeDef *Instance_b = TIM2;
uint8_t channel_b = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_b), PinMap_PWM));
HardwareTimer *TIM_B = new HardwareTimer(Instance_b);

TIM_TypeDef *Instance_c = TIM5;
uint8_t channel_c = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_c), PinMap_PWM));
HardwareTimer *TIM_C = new HardwareTimer(Instance_c);

uint32_t pwm_freq = 10;


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

PWM_STATE pwm_state = {1, 1, 1, 0, 0, 0, 0, 0, 0};

void print_state() {
    Serial.print(pwm_state.PIN_A+1); Serial.print(" ");
    Serial.print(pwm_state.PIN_B+2); Serial.print(" ");
    Serial.print(pwm_state.PIN_C+3); Serial.print(" ");
    Serial.print(pwm_state.timer_period_a); Serial.print(" ");
    // Serial.print(pwm_state.timer_period_b); Serial.print(" ");
    // Serial.print(pwm_state.timer_period_c); Serial.print(" ");


    // print counter values
    float max_counter_value = static_cast<float>(TIM_A->getHandle()->Init.Period);
    float counter_a = static_cast<float>(TIM_A->getHandle()->Instance->CNT) / max_counter_value;
    float counter_b = static_cast<float>(TIM_B->getHandle()->Instance->CNT) / max_counter_value;
    float counter_c = static_cast<float>(TIM_C->getHandle()->Instance->CNT) / max_counter_value;
    // Serial.print(max_counter_value); Serial.print(" ");
    Serial.print(counter_a); Serial.print(" ");
    // Serial.print(counter_b); Serial.print(" ");
    // Serial.print(counter_c); Serial.print(" ");

    // print duty values
    float max_duty_value = 100;
    float duty_a = static_cast<float>(pwm_state.duty_a) / max_duty_value;
    float duty_b = static_cast<float>(pwm_state.duty_b) / max_duty_value;
    float duty_c = static_cast<float>(pwm_state.duty_c) / max_duty_value;
    // Serial.print(max_duty_value); Serial.print(" ");
    Serial.print(duty_a); Serial.print(" ");
    Serial.print(duty_b); Serial.print(" ");
    Serial.print(duty_c); Serial.print(" ");

    Serial.print("\n");
}

void Update_A_callback() {
    // pwm_state.PIN_A = 1;
    pwm_state.timer_period_a = (pwm_state.timer_period_a)? 0 : 1;
    // print_state();
}
void Compare_A_callback() {
    pwm_state.PIN_A = (pwm_state.PIN_A)? 0 : 1;
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
    // pwm_state.PIN_B = 0;
    // print_state();
}

void Update_C_callback() {
    pwm_state.timer_period_c = (pwm_state.timer_period_c)? 0 : 1;

}
void Compare_C_callback() {
    pwm_state.PIN_C = (pwm_state.PIN_C)? 0 : 1;
}

void init_pwm() {

    uint32_t COUNTER_MODE = TIM_COUNTERMODE_CENTERALIGNED3;
    TimerModes_t PWM_MODE = TIMER_OUTPUT_COMPARE_PWM1;

    TIM_A->setMode(channel_a, PWM_MODE, pin_a);
    TIM_A->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_A->getHandle()->Init.RepetitionCounter = 1;
    TIM_A->setPWM(channel_a, pin_a, pwm_freq, 0, Update_A_callback, Compare_A_callback);

    TIM_B->setMode(channel_b, PWM_MODE, pin_b);
    TIM_B->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_B->getHandle()->Init.RepetitionCounter = 1;
    TIM_B->setPWM(channel_b, pin_b, pwm_freq, 0, Update_B_callback, Compare_B_callback);

    TIM_C->setMode(channel_c, PWM_MODE, pin_c);
    TIM_C->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_C->getHandle()->Init.RepetitionCounter = 1;
    TIM_C->setPWM(channel_c, pin_c, pwm_freq, 0, Update_C_callback, Compare_C_callback);


    HAL_TIM_Base_Init(TIM_A->getHandle());
    HAL_TIM_Base_Init(TIM_B->getHandle());
    HAL_TIM_Base_Init(TIM_C->getHandle());

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

void set_pwm(uint32_t Ua, uint32_t Ub, uint32_t Uc) {
    pwm_state.duty_a = Ua;
    pwm_state.duty_b = Ub;
    pwm_state.duty_c = Uc;
    constrain(Ua, 0.0, 100);
    constrain(Ub, 0.0, 100);
    constrain(Uc, 0.0, 100);
    TIM_A-> setPWM(channel_a, pin_a, pwm_freq, Ua);
    TIM_B-> setPWM(channel_b, pin_b, pwm_freq, Ub);
    TIM_C-> setPWM(channel_c, pin_c, pwm_freq, Uc);
}


void setup() {
    // // Set the pwm mode to centre aligned
    // TIM1->CR1 |= TIM_CR1_CMS_1; // Center-aligned mode 1
    //
    // TIM2->CR1 |= TIM_CR1_CMS_1; // Center-aligned mode 1
    // TIM3->CR1 |= TIM_CR1_CMS_1; // Center-aligned mode 1
    // TIM4->CR1 |= TIM_CR1_CMS_1; // Center-aligned mode 1
    // TIM5->CR1 |= TIM_CR1_CMS_1; // Center-aligned mode 1
    // TIM6->CR1 |= TIM_CR1_CMS_1; // Center-aligned mode 1
    // TIM8->CR1 |= TIM_CR1_CMS_1; // Center-aligned mode 1

    Serial.begin(115200);
    delay(1000);
    // Serial.print("Starting\n");
    // Serial.print("Channels: "); Serial.println(channel_a);
    // Serial.print("Channels: "); Serial.println(channel_b);
    // Serial.print("Channels: "); Serial.println(channel_c);


    init_pwm();
    sync_pwm();
}

void loop() {

    set_pwm(10, 50, 90);
    print_state();
    delay(10);




}
