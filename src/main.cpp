
/*
// =============== 3X PWM testing =================
#include <Arduino.h>
#include <Maxwell.h>

Maxwell::Maxwell maxwell;
// function finds the appropriate timer source trigger for the master/slave timer combination
// returns -1 if no trigger source is found
// currently supports the master timers to be from TIM1 to TIM4 and TIM8
int _getInternalSourceTrigger(HardwareTimer* master, HardwareTimer* slave) {
  // put master and slave in temp variables to avoid arrows
  TIM_TypeDef *TIM_master = master->getHandle()->Instance;
  TIM_TypeDef *TIM_slave = slave->getHandle()->Instance;
  #if defined(TIM1) && defined(LL_TIM_TS_ITR0)
    if (TIM_master == TIM1){
      #if defined(TIM2)
      if(TIM_slave == TIM2) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR0;
      #endif
    }
  #endif
  #if defined(TIM2) &&  defined(LL_TIM_TS_ITR1)
    else if (TIM_master == TIM2){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM5)
      else if(TIM_slave == TIM5) return LL_TIM_TS_ITR0;
      #endif
    }
  #endif
  #if defined(TIM3) &&  defined(LL_TIM_TS_ITR2)
    else if (TIM_master == TIM3){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM2)
      else if(TIM_slave == TIM2) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM5)
      else if(TIM_slave == TIM5) return LL_TIM_TS_ITR1;
      #endif
    }
  #endif
  #if defined(TIM4) &&  defined(LL_TIM_TS_ITR3)
    else if (TIM_master == TIM4){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM2)
      else if(TIM_slave == TIM2) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM5)
      else if(TIM_slave == TIM5) return LL_TIM_TS_ITR1;
      #endif
    }
  #endif
  #if defined(TIM5)
    else if (TIM_master == TIM5){
      #if !defined(STM32L4xx) // only difference between F4,F1 and L4
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR2;
      #endif
      #endif
      #if defined(TIM8)
      if(TIM_slave == TIM8) return LL_TIM_TS_ITR3;
      #endif
    }
  #endif
  #if defined(TIM8)
    else if (TIM_master == TIM8){
      #if defined(TIM2)
      if(TIM_slave==TIM2) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM5)
      else if(TIM_slave == TIM5) return LL_TIM_TS_ITR3;
      #endif
    }
  #endif
  return -1; // combination not supported
}


PinName pin_a = DRV8323_HI_A_PIN;
PinName pin_b = DRV8323_HI_B_PIN;
PinName pin_c = DRV8323_HI_C_PIN;

TIM_TypeDef *Instance_a = TIM1;// (TIM_TypeDef *)pinmap_peripheral(pin_a, PinMap_PWM);
uint32_t channel_a = STM_PIN_CHANNEL(pinmap_function(pin_a, PinMap_PWM));
HardwareTimer *TIM_A = new HardwareTimer(Instance_a);

TIM_TypeDef *Instance_b = TIM2; //(TIM_TypeDef *)pinmap_peripheral(pin_b, PinMap_PWM);
uint32_t channel_b = STM_PIN_CHANNEL(pinmap_function(pin_b, PinMap_PWM));
HardwareTimer *TIM_B = new HardwareTimer(Instance_b);

TIM_TypeDef *Instance_c = TIM2;//(TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_c), PinMap_PWM);
uint32_t channel_c = STM_PIN_CHANNEL(pinmap_function(pin_c, PinMap_PWM));
HardwareTimer *TIM_C = new HardwareTimer(Instance_c);

// PARAMS
uint32_t pwm_freq = 20000 * 2;
uint32_t pwm_resolution = 8;



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

void init_pwm() {
    // Set APB1 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    // Set APB2 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;

    uint32_t COUNTER_MODE = TIM_COUNTERMODE_CENTERALIGNED3;  // Counter Rise and then fall
    TimerModes_t PWM_MODE = TIMER_OUTPUT_COMPARE_PWM1;

    TIM_A->setMode(channel_a, PWM_MODE, pin_a);
    TIM_A->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_A->getHandle()->Init.RepetitionCounter = 1;
    TIM_A->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_A->setCaptureCompare(channel_a, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));

    TIM_B->setMode(channel_b, PWM_MODE, pin_b);
    TIM_B->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_B->getHandle()->Init.RepetitionCounter = 1;
    TIM_B->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_B->setCaptureCompare(channel_b, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));

    TIM_C->setMode(channel_c, PWM_MODE, pin_c);
    TIM_C->getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_C->getHandle()->Init.RepetitionCounter = 1;
    TIM_C->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_C->setCaptureCompare(channel_c, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));

    HAL_TIM_Base_Init(TIM_A->getHandle());
    HAL_TIM_Base_Init(TIM_B->getHandle());
    HAL_TIM_Base_Init(TIM_C->getHandle());
}

void set_pwm(uint32_t Ua, uint32_t Ub, uint32_t Uc, uint32_t resolution) {
    // constrain(Ua, 0.0, static_cast<float>(pow(2, resolution)) - 1);
    // constrain(Ub, 0.0, static_cast<float>(pow(2, resolution)) - 1);
    // constrain(Uc, 0.0, static_cast<float>(pow(2, resolution)) - 1);
    // (Ua < 2.0)? Ua = 0 : Ua;
    // (Ub < 2.0)? Ub = 0 : Ub;
    // (Uc < 2.0)? Uc = 0 : Uc;
    TIM_A->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_B->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_C->setOverflow(pwm_freq, HERTZ_FORMAT);
    TIM_A->setCaptureCompare(channel_a, Ua, static_cast<TimerCompareFormat_t>(resolution));
    TIM_B->setCaptureCompare(channel_b, Ub, static_cast<TimerCompareFormat_t>(resolution));
    TIM_C->setCaptureCompare(channel_c, Uc, static_cast<TimerCompareFormat_t>(resolution));
}

void set_master_slave_mode() {
    LL_TIM_SetSlaveMode(TIM_A->getHandle()->Instance, LL_TIM_SLAVEMODE_DISABLED);
    LL_TIM_SetTriggerOutput(TIM_A->getHandle()->Instance, LL_TIM_TRGO_ENABLE);

    // Configure the other two timers to get their input trigger from the master timer:
    for (auto timer : {TIM_B, TIM_C}) {
        LL_TIM_SetTriggerInput(timer->getHandle()->Instance, _getInternalSourceTrigger(TIM_A, timer));
        LL_TIM_SetSlaveMode(timer->getHandle()->Instance, LL_TIM_SLAVEMODE_TRIGGER);
    }
}

void setup() {
    Serial.begin(9600);
    maxwell.setup();
    maxwell.driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);
    init_pwm();
    set_pwm(200, 127, 60, pwm_resolution);

    set_master_slave_mode();
    sync_pwm();

    // TIM_A->setPWM(channel_a, pin_a, 20000, 50);
    // TIM_B->setPWM(channel_b, pin_b, 20000, 50);
    // TIM_C->setPWM(channel_c, pin_c, 20000, 20);
    maxwell.driver->clear_fault();
}


void loop() {
    Serial.println(maxwell.driver->get_fault_status_1_string());
    Serial.println(maxwell.driver->get_fault_status_2_string());
}




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

PinName pin_a = DRV8323_HI_A_PIN;
PinName pin_b = DRV8323_HI_B_PIN;
PinName pin_c = DRV8323_HI_C_PIN;
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



TIM_TypeDef *Instance_a = TIM1; //(TIM_TypeDef *)pinmap_peripheral(pin_a, PinMap_PWM);
uint32_t channel_a = STM_PIN_CHANNEL(pinmap_function(pin_a, PinMap_PWM));
HardwareTimer *TIM_A = new HardwareTimer(Instance_a);
//
// TIM_TypeDef *Instance_b = TIM2; //(TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_b), PinMap_PWM);
// uint32_t channel_b = 4; //STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_b), PinMap_PWM));
// HardwareTimer *TIM_B = new HardwareTimer(Instance_b);
//
// TIM_TypeDef *Instance_c = TIM5;//(TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_c), PinMap_PWM);
// uint32_t channel_c = 2; //STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_c), PinMap_PWM));
// HardwareTimer *TIM_C = new HardwareTimer(Instance_c);

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
    TimerModes_t PWM_MODE = TIMER_OUTPUT_COMPARE_PWM2;

    // TIM_A->pause();
    // TIM_A->setMode(channel_a, PWM_MODE, pin_a);
    // TIM_A->getHandle()->Init.CounterMode = COUNTER_MODE;
    // TIM_A->getHandle()->Init.RepetitionCounter = 1;
    TIM_A->setPWM(channel_a, pin_a, pwm_freq, 0);
    // TIM_A->setOverflow(pwm_freq, HERTZ_FORMAT);
    // TIM_A->setCaptureCompare(channel_a, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));
    // TIM_A->attachInterrupt(Update_A_callback);
    // TIM_A->attachInterrupt(channel_a, Compare_A_callback);

    // TIM_B->pause();
    // TIM_B->setMode(channel_b, PWM_MODE, NC);
    // TIM_B->getHandle()->Init.CounterMode = COUNTER_MODE;
    // TIM_B->getHandle()->Init.RepetitionCounter = 1;
    // TIM_B->setPWM(channel_b, pin_b, pwm_freq, 0);
    // TIM_B->setOverflow(pwm_freq, HERTZ_FORMAT);
    // TIM_B->setCaptureCompare(channel_b, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));
    // TIM_B->attachInterrupt(Update_B_callback);
    // TIM_B->attachInterrupt(channel_b, Compare_B_callback);
    //
    // TIM_C->pause();
    // TIM_C->setMode(channel_c, PWM_MODE, NC);
    // TIM_C->getHandle()->Init.CounterMode = COUNTER_MODE;
    // TIM_C->getHandle()->Init.RepetitionCounter = 1;
    // TIM_C->setPWM(channel_c, pin_c, pwm_freq, 0);
    // TIM_C->setOverflow(pwm_freq, HERTZ_FORMAT);
    // TIM_C->setCaptureCompare(channel_c, 0, static_cast<TimerCompareFormat_t>(PWM_RESOLUTION));
    // TIM_C->attachInterrupt(Update_C_callback);
    // TIM_C->attachInterrupt(channel_c, Compare_C_callback);




    // HAL_TIM_Base_Init(TIM_A->getHandle());
    // HAL_TIM_Base_Init(TIM_B->getHandle());
    // HAL_TIM_Base_Init(TIM_C->getHandle());

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
    // TIM_B->setOverflow(pwm_freq, HERTZ_FORMAT);
    // TIM_C->setOverflow(pwm_freq, HERTZ_FORMAT);
    //
    TIM_A->setCaptureCompare(channel_a, Ua, RESOLUTION_8B_COMPARE_FORMAT);
    // TIM_B->setCaptureCompare(channel_b, Ub, RESOLUTION_8B_COMPARE_FORMAT);
    // TIM_C->setCaptureCompare(channel_c, Uc, RESOLUTION_8B_COMPARE_FORMAT);

    // TIM_A->setPWM(channel_a, pin_a, pwm_freq, Ua);
    // TIM_B->setPWM(channel_b, pin_b, pwm_freq, Ub);
    // TIM_C->setPWM(channel_c, pin_c, pwm_freq, Uc);


}

void sync_pwm() {
    TIM_A->pause();
    TIM_A->refresh();
    // TIM_B->pause();
    // TIM_B->refresh();
    // TIM_C->pause();
    // TIM_C->refresh();

    TIM_A->resume();
    // TIM_B->resume();
    // TIM_C->resume();
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
    TIM_A->setPWM(channel_a, pin_a, pwm_freq, 50);
    // set_pwm(127, 50, 10);
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


// Goal is to read the adc with dma
#include "Arduino.h"
#include "pin_definitions.h"



DMA_HandleTypeDef hdma_adc1;
ADC_HandleTypeDef hadc1;
uint32_t adc_value = 0;



void setup() {
    Serial.begin(9600);
    delay(1000);
    // PB0 = ADC1_IN8 or ADC2_IN8

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    pinMode(V_SENSE_A_PIN, INPUT);

    // Configure the adc
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    // Configure the adc channel
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_10; // Replace with the channel corresponding to the pin
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Configure the DMA
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);

    // Link DMA
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    // Enable DMA interrupt
    // HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // Start the ADC in DMA mode
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_value, 1);

    Serial.println("Setup complete! ADC and DMA initialized.");
}

void loop() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_value, 1);
    // while (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK);

    Serial.println(HAL_ADC_GetValue(&hadc1));
    delay(10);
    HAL_ADC_Stop_DMA(&hadc1);
}

// extern "C" void DMA2_Stream0_IRQHandler() {
//     HAL_DMA_IRQHandler(&hdma_adc1);
//
//     Serial.println(adc_value);
// }


/*
// Define the pin and configure it for analog input
__HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_0; // Replace with the desired pin
GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// Configure the ADC channel
hadc1.Instance = ADC1;
hadc1.Init.Resolution = ADC_RESOLUTION_12B;
hadc1.Init.ScanConvMode = DISABLE;
hadc1.Init.ContinuousConvMode = DISABLE;
hadc1.Init.DiscontinuousConvMode = DISABLE;
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
hadc1.Init.NbrOfConversion = 1;
HAL_ADC_Init(&hadc1);

ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_0; // Replace with the channel corresponding to the pin
sConfig.Rank = ADC_REGULAR_RANK_1;
sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
HAL_ADC_ConfigChannel(&hadc1, &sConfig);
*/









/*


// -----------------------------------------------------------------------------------
#include <Arduino.h>
#include "pin_definitions.h"
#include "Maxwell.h"
#include "PWMInput.h"


Maxwell::Maxwell maxwell;
PWMInput pwm_input(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);

void pwm_callback() {
    pwm_input.pwm_callback();
}

void setup() {
    delay(100);
    Serial.begin(921600);

    maxwell.setup();
    maxwell.init_pwm_3x();

    pwm_input.set_callback(pwm_callback);
    maxwell.pwm_input = &pwm_input;
    maxwell.driver->perform_current_sense_calibration();
    // maxwell.driver->current_sensors->calibrate_offsets();
    maxwell.driver->clear_fault();
}



void loop() {
    // maxwell.sinusoidal_position_control();
    // maxwell.foc_position_control();
    maxwell.voltage_torque_control();

    // Serial.println(maxwell.driver->get_fault_status_1_string());
    // Serial.println(maxwell.driver->get_fault_status_2_string());

}

*/