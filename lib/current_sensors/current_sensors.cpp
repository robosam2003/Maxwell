//
// Created by robosam on 2/3/25.
//

#include "current_sensors.h"

CurrentSensors::CurrentSensors(uint32_t pin_a, uint32_t pin_b, uint32_t pin_c, int gain) {
    _pin_a = pin_a;
    _pin_b = pin_b;
    _pin_c = pin_c;
    // _conversion_factor = conversion_factor;
    _current_a = 0.0;
    _current_b = 0.0;
    _current_c = 0.0;
    _offset_a = 0.0;
    _offset_b = 0.0;
    _offset_c = 0.0;

    double cuttoff_freq = 30;
    _filter_a = new RCFilter(cuttoff_freq);
    _filter_b = new RCFilter(cuttoff_freq);
    _filter_c = new RCFilter(cuttoff_freq);

    _csa_gain = gain;
    hadc1 = new ADC_HandleTypeDef();
    hadc2 = new ADC_HandleTypeDef();
    hadc3 = new ADC_HandleTypeDef();

    // analogReadResolution(12);


    setup_injected_adc();
}

void CurrentSensors::setup_injected_adc() {
// 1. Enable Clocks for all three ADCs and GPIOs
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure Pins: PA0 (ADC1_IN0), PA1 (ADC2_IN1), PA2 (ADC3_IN2)
    pinMode(_pin_a, INPUT_ANALOG);
    pinMode(_pin_b, INPUT_ANALOG);
    pinMode(_pin_c, INPUT_ANALOG);

    // 2. Basic Configuration for all three ADCs
    ADC_HandleTypeDef* adcs[] = {hadc1, hadc2, hadc3};
    ADC_TypeDef* instances[] = {ADC1, ADC2, ADC3};

    for(int i = 0; i < 3; i++) {
        adcs[i]->Instance = instances[i];
        adcs[i]->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
        adcs[i]->Init.Resolution = ADC_RESOLUTION_12B;
        adcs[i]->Init.ScanConvMode = DISABLE;           // Only 1 channel per ADC
        adcs[i]->Init.ContinuousConvMode = DISABLE;     // Triggered by Timer
        adcs[i]->Init.DiscontinuousConvMode = DISABLE;
        adcs[i]->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        adcs[i]->Init.ExternalTrigConv = ADC_SOFTWARE_START;
        adcs[i]->Init.DataAlign = ADC_DATAALIGN_RIGHT;
        adcs[i]->Init.NbrOfConversion = 1;
        adcs[i]->Init.DMAContinuousRequests = DISABLE;
        adcs[i]->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

        if (HAL_ADC_Init(adcs[i]) != HAL_OK) {
            Serial.println("ADC Init Error");
        }
    }

    // 3. Multi-ADC Mode Configuration (The "Triple" part)
    ADC_MultiModeTypeDef multimode = {0};
    multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT; // Synchronize Injected conversions
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;

    if (HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode) != HAL_OK) {
        Serial.println("MultiMode Config Error");
    }

    // 4. Configure Injected Channels (1 per ADC)
    ADC_InjectionConfTypeDef sConfigInjected = {0};
    sConfigInjected.InjectedNbrOfConversion = 1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;

    // Only the Master (ADC1) needs the trigger source defined
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;

    // Phase C -> ADC1 Channel 0
    sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
    HAL_ADCEx_InjectedConfigChannel(hadc1, &sConfigInjected);

    // Phase B -> ADC2 Channel 1
    sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
    HAL_ADCEx_InjectedConfigChannel(hadc2, &sConfigInjected);

    // Phase A -> ADC3 Channel 2
    sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
    HAL_ADCEx_InjectedConfigChannel(hadc3, &sConfigInjected);

    // 5. Start the ADCs (Slaves first, then Master)
    HAL_ADCEx_InjectedStart(hadc3);
    HAL_ADCEx_InjectedStart(hadc2);
    HAL_ADCEx_InjectedStart(hadc1);

    Serial.println("SUCCESSFULLY SETUP TRIPLE SIMULTANEOUS INJECTED ADC");
}

void CurrentSensors::set_csa_gain(int gain) {
  _csa_gain = gain;
}

void CurrentSensors::calibrate_offsets() {
  uint32_t num_samples = 100;
  double sum_a = 0;
  double sum_b = 0;
  double sum_c = 0;
  _offset_a = 0.0;
  _offset_b = 0.0;
  _offset_c = 0.0;
  _filter_a->prev_value = 0.0;
  _filter_b->prev_value = 0.0;
  _filter_c->prev_value = 0.0;
  bool old_filtered = filtered;
  filtered = false; // Ensure no filtering for this bit

  for (int i = 0; i< num_samples; i++) {
    read();
    // Let the value settle
    get_current_a();
    get_current_b();
    get_current_c();
    delay(1);
  }
  for (int i = 0; i < num_samples; i++) {
    read();
    sum_a += get_current_a();
    sum_b += get_current_b();
    sum_c += get_current_c();
    delay(1);
  }
  _offset_a = sum_a / num_samples;
  _offset_b = sum_b / num_samples;
  _offset_c = sum_c / num_samples;
  filtered = old_filtered;
  }

void CurrentSensors::read() {
    // 2. Fetch raw values from the Injected Data Registers (JDR1)
    // Rank 1 for all because each ADC only handles one channel in this setup
    uint32_t raw_a = HAL_ADCEx_InjectedGetValue(hadc3, ADC_INJECTED_RANK_1);
    uint32_t raw_b = HAL_ADCEx_InjectedGetValue(hadc2, ADC_INJECTED_RANK_1);
    uint32_t raw_c = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_1);

    // 3. Convert to Volts
    v_a = raw_a * CURRENT_SENSE_CONVERSION_FACTOR;
    v_b = raw_b * CURRENT_SENSE_CONVERSION_FACTOR;
    v_c = raw_c * CURRENT_SENSE_CONVERSION_FACTOR;

    uint32_t current_time_us = micros();

    // 4. Calculate actual Amps with Offsets
    // Formula: (Vref/2 - Vread) / (Gain * Rsense)
    float a_calc = (3.3/2.0 - v_a) / (_csa_gain * R_SENSE) + (inverted ? _offset_a : -_offset_a);
    float b_calc = (3.3/2.0 - v_b) / (_csa_gain * R_SENSE) + (inverted ? _offset_b : -_offset_b);
    float c_calc = (3.3/2.0 - v_c) / (_csa_gain * R_SENSE) + (inverted ? _offset_c : -_offset_c);

    if (inverted) {
        a_calc *= -1; b_calc *= -1; c_calc *= -1;
    }

    // 5. Apply Filtering and Anomaly Detection
    if (filtered) {
        _current_a = (abs(a_calc) < ANOMALY_THRESHOLD) ? _filter_a->update(a_calc, current_time_us) : _current_a;
        _current_b = (abs(b_calc) < ANOMALY_THRESHOLD) ? _filter_b->update(b_calc, current_time_us) : _current_b;
        _current_c = (abs(c_calc) < ANOMALY_THRESHOLD) ? _filter_c->update(c_calc, current_time_us) : _current_c;
    } else {
        _current_a = (abs(a_calc) < ANOMALY_THRESHOLD) ? a_calc : _current_a;
        _current_b = (abs(b_calc) < ANOMALY_THRESHOLD) ? b_calc : _current_b;
        _current_c = (abs(c_calc) < ANOMALY_THRESHOLD) ? c_calc : _current_c;
    }

}

double CurrentSensors::get_current_a() {
    return _current_a;
}

double CurrentSensors::get_current_b() {
    return _current_b;
}

double CurrentSensors::get_current_c() {
    return _current_c;
}

double* CurrentSensors::get_currents() {
  double currents[3] = {_current_a, _current_b, _current_c};
  return currents;
}

double CurrentSensors::get_total_current() {
  return _current_a + _current_b + _current_c;
}