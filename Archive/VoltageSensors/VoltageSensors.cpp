// lib/VoltageSensors/VoltageSensors.cpp
#include "VoltageSensors.h"

VoltageSensors::VoltageSensors(
    uint32_t pin_a,
    uint32_t pin_b,
    uint32_t pin_c,
    uint32_t pin_supply,
    float divider_gain_a,
    float divider_gain_b,
    float divider_gain_c,
    float divider_gain_supply,
    float vref
) {
    _pin_a = pin_a;
    _pin_b = pin_b;
    _pin_c = pin_c;
    _pin_supply = pin_supply;

    _divider_gain_a = divider_gain_a;
    _divider_gain_b = divider_gain_b;
    _divider_gain_c = divider_gain_c;
    _divider_gain_supply = divider_gain_supply;

    _vref = vref;
    _adc_lsb = _vref / 4095.0f; // 12-bit ADC

    // Slightly lower cutoff for voltage smoothing (adjust as needed)
    _filter_a = new RCFilter(20.0);
    _filter_b = new RCFilter(20.0);
    _filter_c = new RCFilter(20.0);
    _filter_supply = new RCFilter(10.0);

    hadc1 = new ADC_HandleTypeDef();
    hadc2 = new ADC_HandleTypeDef();
    hadc3 = new ADC_HandleTypeDef();

    setup_injected_adc();
}

void VoltageSensors::setup_injected_adc() {
    // Clocks
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Analog pins
    pinMode(_pin_a, INPUT_ANALOG);      // Expected: PA3
    pinMode(_pin_b, INPUT_ANALOG);      // Expected: PC1
    pinMode(_pin_c, INPUT_ANALOG);      // Expected: PC0
    pinMode(_pin_supply, INPUT_ANALOG); // Expected: PA4

    ADC_HandleTypeDef* adcs[] = {hadc1, hadc2, hadc3};
    ADC_TypeDef* instances[] = {ADC1, ADC2, ADC3};

    for (int i = 0; i < 3; i++) {
        adcs[i]->Instance = instances[i];
        adcs[i]->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
        adcs[i]->Init.Resolution = ADC_RESOLUTION_12B;
        adcs[i]->Init.ScanConvMode = DISABLE;
        adcs[i]->Init.ContinuousConvMode = DISABLE;
        adcs[i]->Init.DiscontinuousConvMode = DISABLE;
        adcs[i]->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        adcs[i]->Init.ExternalTrigConv = ADC_SOFTWARE_START;
        adcs[i]->Init.DataAlign = ADC_DATAALIGN_RIGHT;
        adcs[i]->Init.NbrOfConversion = 1;
        adcs[i]->Init.DMAContinuousRequests = DISABLE;
        adcs[i]->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

        if (HAL_ADC_Init(adcs[i]) != HAL_OK) {
            Serial.println("Voltage ADC Init Error");
        }
    }

    // Triple injected simultaneous mode (same pattern as CurrentSensors)
    ADC_MultiModeTypeDef multimode = {0};
    multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;

    if (HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode) != HAL_OK) {
        Serial.println("Voltage MultiMode Config Error");
    }

    ADC_InjectionConfTypeDef sConfigInjected = {0};
    sConfigInjected.InjectedNbrOfConversion = 1;
    // Divider sources are typically higher impedance than CSA output; use longer sample time.
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;

    // Master trigger on ADC1 (same as current path)
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;

    // STM32F405 mapping for your pin defs:
    // PA3 -> ADC_CHANNEL_3
    // PC1 -> ADC_CHANNEL_11
    // PC0 -> ADC_CHANNEL_10
    sConfigInjected.InjectedChannel = ADC_CHANNEL_3;   // Phase A
    HAL_ADCEx_InjectedConfigChannel(hadc1, &sConfigInjected);

    sConfigInjected.InjectedChannel = ADC_CHANNEL_11;  // Phase B
    HAL_ADCEx_InjectedConfigChannel(hadc2, &sConfigInjected);

    sConfigInjected.InjectedChannel = ADC_CHANNEL_10;  // Phase C
    HAL_ADCEx_InjectedConfigChannel(hadc3, &sConfigInjected);

    // Start slaves first, then master
    HAL_ADCEx_InjectedStart(hadc3);
    HAL_ADCEx_InjectedStart(hadc2);
    HAL_ADCEx_InjectedStart(hadc1);

    Serial.println("VoltageSensors triple injected ADC ready");
}

void VoltageSensors::read_phase_voltages() {
    uint32_t raw_a = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_b = HAL_ADCEx_InjectedGetValue(hadc2, ADC_INJECTED_RANK_1);
    uint32_t raw_c = HAL_ADCEx_InjectedGetValue(hadc3, ADC_INJECTED_RANK_1);

    float va = raw_a * _adc_lsb * _divider_gain_a;
    float vb = raw_b * _adc_lsb * _divider_gain_b;
    float vc = raw_c * _adc_lsb * _divider_gain_c;

    if (filtered) {
        uint32_t now_us = micros();
        phase_a_v = _filter_a->update(va, now_us);
        phase_b_v = _filter_b->update(vb, now_us);
        phase_c_v = _filter_c->update(vc, now_us);
    } else {
        phase_a_v = va;
        phase_b_v = vb;
        phase_c_v = vc;
    }
}

void VoltageSensors::read_supply_voltage() {
    // Simpler path for low-rate bus voltage telemetry
    int raw = analogRead(_pin_supply);
    float vs = raw * _adc_lsb * _divider_gain_supply;

    if (filtered) {
        supply_v = _filter_supply->update(vs, micros());
    } else {
        supply_v = vs;
    }
}

float VoltageSensors::get_phase_a_voltage() const { return phase_a_v; }
float VoltageSensors::get_phase_b_voltage() const { return phase_b_v; }
float VoltageSensors::get_phase_c_voltage() const { return phase_c_v; }
float VoltageSensors::get_supply_voltage() const { return supply_v; }
