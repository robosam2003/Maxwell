
#define TEST
#ifdef TEST
// Goal is to read the adc with dma
#include "Arduino.h"
#include "pin_definitions.h"
#include "stm32f4xx.h"

PinName pin_a = DRV8323_CURR_SENSE_A_PIN; // PB_0 - ADC1_IN8
PinName pin_b = DRV8323_CURR_SENSE_B_PIN; // PC_5 - ADC1_IN15
PinName pin_c = DRV8323_CURR_SENSE_C_PIN; // PC_4 - ADC1_IN14

TIM_TypeDef *Instance_a = TIM1;// (TIM_TypeDef *)pinmap_peripheral(pin_a, PinMap_PWM);
uint32_t channel_a = STM_PIN_CHANNEL(pinmap_function(DRV8323_HI_A_PIN, PinMap_PWM));
HardwareTimer TIM_A = HardwareTimer(Instance_a);

#define FREQ 20000*2
#define TIMER_RESOLUTION 8

ADC_HandleTypeDef* hadc1 = new ADC_HandleTypeDef();

void timer_setup() {
    uint32_t COUNTER_MODE = TIM_COUNTERMODE_CENTERALIGNED3;  // Counter Rise and then fall
    TimerModes_t PWM_MODE = TIMER_OUTPUT_COMPARE_PWM1;

    TIM_A.setMode(channel_a, PWM_MODE, DRV8323_HI_A_PIN);
    TIM_A.getHandle()->Init.CounterMode = COUNTER_MODE;
    TIM_A.getHandle()->Init.RepetitionCounter = 1;
    TIM_A.setOverflow(FREQ, HERTZ_FORMAT);
    TIM_A.setCaptureCompare(channel_a, 127, static_cast<TimerCompareFormat_t>(TIMER_RESOLUTION));
    // pwm_3x->TIM_A->attachInterrupt(Update_A_callback);
    LL_TIM_SetTriggerOutput(TIM_A.getHandle()->Instance, LL_TIM_TRGO_ENABLE);

    LL_TIM_SetTriggerOutput(TIM_A.getHandle()->Instance, LL_TIM_TRGO_UPDATE);

    HAL_TIM_Base_Init(TIM_A.getHandle());
    TIM_A.resume();

    TIM_A.setOverflow(FREQ, HERTZ_FORMAT);
    TIM_A.setCaptureCompare(channel_a, 127, static_cast<TimerCompareFormat_t>(TIMER_RESOLUTION));
}


void setup_injected_adc() {
        // Assuming all pins belong to the same ADC (they do)
    // Enable clocks
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    pinMode(pin_a, INPUT_ANALOG); // PB_0 - ADC1_IN8
    pinMode(pin_b, INPUT_ANALOG); // PC_5 - ADC1_IN15
    pinMode(pin_c, INPUT_ANALOG); // PC_4 - ADC1_IN14

    hadc1->Instance = ADC1;
    hadc1->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1->Init.Resolution = ADC_RESOLUTION_12B;
    hadc1->Init.ScanConvMode = ENABLE;
    hadc1->Init.ContinuousConvMode = ENABLE; // For injected mode
    hadc1->Init.DiscontinuousConvMode = DISABLE;
    hadc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; //
    hadc1->Init.ExternalTrigConv = ADC_SOFTWARE_START; // for now
    hadc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1->Init.NbrOfConversion = 3;
    hadc1->Init.DMAContinuousRequests = DISABLE;
    hadc1->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    uint32_t adc_code = HAL_ADC_Init(hadc1);
    if (adc_code != HAL_OK) {
      for (int i=0; i<10; i++) {
        Serial.println("ADC Initialization Error");
        Serial.println(adc_code);
        delay(100);
      }
    }

    ADC_InjectionConfTypeDef sConfigInjected;
    sConfigInjected.InjectedNbrOfConversion = 3;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;

    // first channel
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedChannel = ADC_CHANNEL_4; // PB_0 - ADC1_IN8
    if (HAL_ADCEx_InjectedConfigChannel(hadc1, &sConfigInjected) != HAL_OK){
        Serial.println("Error configuring ADC channel 1");
    }
    // second channel
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    sConfigInjected.InjectedChannel = ADC_CHANNEL_15; // PC_5 - ADC1_IN15
    if (HAL_ADCEx_InjectedConfigChannel(hadc1, &sConfigInjected) != HAL_OK){
        Serial.println("Error configuring ADC channel 2");
    }
    // third channel
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    sConfigInjected.InjectedChannel = ADC_CHANNEL_14; // PC_4 - ADC1_IN14
    if (HAL_ADCEx_InjectedConfigChannel(hadc1, &sConfigInjected) != HAL_OK){
        Serial.println("Error configuring ADC channel 3");
    }

    // Start the injected conversion
    if (HAL_ADCEx_InjectedStart(hadc1) != HAL_OK) {
        Serial.println("Error starting injected conversion");
    }
    Serial.println("SUCCESSFULLY SETUP INJECTED ADC");
}


void setup() {
    pinMode(DRV8323_GATE_EN_PIN, OUTPUT);
    pinMode(DRV8323_LO_A_PIN, OUTPUT);
    digitalWrite(DRV8323_LO_A_PIN, HIGH);
    digitalWrite(DRV8323_GATE_EN_PIN, HIGH);

    delay(3000);
    Serial.begin(9600);
    Serial.println("Setup started");
    timer_setup();
    setup_injected_adc();
}


void loop() {
    // if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
    //     Serial.println("Error starting injected conversion");
    // }
    // Wait for the conversion to complete

    // Read the converted values
    uint32_t start = micros();
    double adc_value1 = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_1) * CURRENT_SENSE_CONVERSION_FACTOR;
    double adc_value2 = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_2) * CURRENT_SENSE_CONVERSION_FACTOR;
    double adc_value3 = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_3) * CURRENT_SENSE_CONVERSION_FACTOR;
    uint32_t end = micros();

    Serial.print(adc_value1); Serial.print(" ");
    Serial.print(adc_value2); Serial.print(" ");
    Serial.println(adc_value3);
    // Serial.println(end - start);

    // delay(10); // Delay for readability
}
#endif




#ifndef TEST
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
    delay(1000);
    Serial.begin(921600);

    maxwell.setup();
    maxwell.init_pwm_3x();
    // maxwell.driver->current_sensors->setup_injected_adc();

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
#endif