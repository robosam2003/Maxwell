//
// Created by robosam on 2/3/25.
//

#include "adc.h"

Adc::Adc(uint32_t curr_pin_a,
         uint32_t curr_pin_b,
         uint32_t curr_pin_c,
         uint32_t volt_pin_a,
         uint32_t volt_pin_b,
         uint32_t volt_pin_c,
         uint32_t supply_sense_pin) {
    // Current Sensing
    _curr_pin_a = curr_pin_a;
    _curr_pin_b = curr_pin_b;
    _curr_pin_c = curr_pin_c;

    _current_a = 0.0;
    _current_b = 0.0;
    _current_c = 0.0;
    _curr_offset_a = 0.0;
    _curr_offset_b = 0.0;
    _curr_offset_c = 0.0;

    double curr_cuttoff_freq = 30;
    _curr_filter_a = new RCFilter(curr_cuttoff_freq);
    _curr_filter_b = new RCFilter(curr_cuttoff_freq);
    _curr_filter_c = new RCFilter(curr_cuttoff_freq);

    // Voltage Sensing
    _volt_pin_a = volt_pin_a;
    _volt_pin_b = volt_pin_b;
    _volt_pin_c = volt_pin_c;

    double volt_cuttoff_freq = 30;
    _volt_filter_a = new RCFilter(volt_cuttoff_freq);
    _volt_filter_b = new RCFilter(volt_cuttoff_freq);
    _volt_filter_c = new RCFilter(volt_cuttoff_freq);

    // Supply Sensing
    _supply_sense_pin = supply_sense_pin;

    analogReadResolution(12);

    // ADC Handles
    hadc1 = new ADC_HandleTypeDef();
    hadc2 = new ADC_HandleTypeDef();
    hadc3 = new ADC_HandleTypeDef();

    setup_injected_adc();
}

void Adc::setup_injected_adc() {
    // 1. Enable Clocks for all three ADCs and GPIOs
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure Pins: PA0 (ADC1_IN0), PA1 (ADC2_IN1), PA2 (ADC3_IN2)
    pinMode(_curr_pin_a, INPUT_ANALOG);
    pinMode(_curr_pin_b, INPUT_ANALOG);
    pinMode(_curr_pin_c, INPUT_ANALOG);
    pinMode(_volt_pin_a, INPUT_ANALOG);
    pinMode(_volt_pin_b, INPUT_ANALOG);
    pinMode(_volt_pin_c, INPUT_ANALOG);
    pinMode(_supply_sense_pin, INPUT_ANALOG);

    // 2. Basic Configuration for all three ADCs
    ADC_HandleTypeDef* adcs[] = {hadc1, hadc2, hadc3};
    ADC_TypeDef* instances[] = {ADC1, ADC2, ADC3};

    for(int i = 0; i < 3; i++) {
        adcs[i]->Instance = instances[i];
        adcs[i]->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
        adcs[i]->Init.Resolution = ADC_RESOLUTION_12B; //
        adcs[i]->Init.ScanConvMode = DISABLE; // Multiple Ranks now
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
    // ADC_MultiModeTypeDef multimode = {0};
    // multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT; // Synchronize Injected conversions
    // multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
    // multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
    //
    // if (HAL_ADCEx_MultiModeConfigChannel(hadc1, &multimode) != HAL_OK) {
    //     Serial.println("MultiMode Config Error");
    // }

    // 4. Configure Injected Channels (2 per ADC)
    ADC_InjectionConfTypeDef inj = {0};
    inj.InjectedNbrOfConversion = 1;
    inj.InjectedOffset = 0;
    inj.InjectedDiscontinuousConvMode = DISABLE;
    inj.AutoInjectedConv = DISABLE;
    inj.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;   // On rising edge of trigger
    inj.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;          // Use T1 TRGO trigger


    // ========= Rank 1: Current ===========
    inj.InjectedRank = ADC_INJECTED_RANK_1;
    inj.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
    // Phase C -> ADC1 Channel 0
    inj.InjectedChannel = ADC_CHANNEL_0;  HAL_ADCEx_InjectedConfigChannel(hadc1, &inj);
    // Phase B -> ADC2 Channel 1
    inj.InjectedChannel = ADC_CHANNEL_1;  HAL_ADCEx_InjectedConfigChannel(hadc2, &inj);
    // Phase A -> ADC3 Channel 2
    inj.InjectedChannel = ADC_CHANNEL_2;  HAL_ADCEx_InjectedConfigChannel(hadc3, &inj);

    // ======== Rank 2: Voltage ==========
    // inj.InjectedRank = ADC_INJECTED_RANK_2;
    // inj.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES; // What is appropriate for voltage?
    // // Phase C -> ADC1 Channel 10
    // inj.InjectedChannel = ADC_CHANNEL_10; HAL_ADCEx_InjectedConfigChannel(hadc1, &inj);
    // // Phase B -> ADC2 Channel 11
    // inj.InjectedChannel = ADC_CHANNEL_11; HAL_ADCEx_InjectedConfigChannel(hadc2, &inj);
    // // Phase A -> ADC3 Channel 3
    // inj.InjectedChannel = ADC_CHANNEL_3;  HAL_ADCEx_InjectedConfigChannel(hadc3, &inj);


    // 5. Start the ADCs (Slaves first, then Master)
    HAL_ADCEx_InjectedStart(hadc3);
    HAL_ADCEx_InjectedStart(hadc2);
    HAL_ADCEx_InjectedStart(hadc1);

    // Assuming you are using HAL under the hood for the ADC setup:
    // HAL_NVIC_SetPriority(ADC_IRQn, 0, 0); // Set highest priority (0)
    // HAL_NVIC_EnableIRQ(ADC_IRQn);         // Enable the interrupt
    //
    // // Enable the "Injected End of Sequence" (JEOS) interrupt on your specific ADC
    // __HAL_ADC_ENABLE_IT(hadc1, LL_ADC_IT_JEOS);
}

void Adc::calibrate_current_offsets() {
  uint32_t num_samples = 100;
  double sum_a = 0;
  double sum_b = 0;
  double sum_c = 0;
  _curr_offset_a = 0.0;
  _curr_offset_b = 0.0;
  _curr_offset_c = 0.0;
  _curr_filter_a->prev_value = 0.0;
  _curr_filter_b->prev_value = 0.0;
  _curr_filter_c->prev_value = 0.0;
  bool old_filtered = curr_filtered;
  curr_filtered = false; // Ensure no filtering for this bit

  for (int i = 0; i< num_samples; i++) {
    read();
    // Let the value settle
    delay(1);
  }
  for (int i = 0; i < num_samples; i++) {
    read();
    sum_a += _current_a;
    sum_b += _current_b;
    sum_c += _current_c;
    delay(1);
  }
  _curr_offset_a = sum_a / num_samples;
  _curr_offset_b = sum_b / num_samples;
  _curr_offset_c = sum_c / num_samples;
  curr_filtered = old_filtered;
  }

void Adc::read() {
    uint32_t current_time_us = micros();

    // =============== Voltage Sensing ================
    // Read raw valued from injected ADC
    // uint32_t raw_volt_v_a = HAL_ADCEx_InjectedGetValue(hadc3, ADC_INJECTED_RANK_2); // analogRead(_volt_pin_a);
    // uint32_t raw_volt_v_b = HAL_ADCEx_InjectedGetValue(hadc2, ADC_INJECTED_RANK_2); // analogRead(_volt_pin_b);
    // uint32_t raw_volt_v_c = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_2); // analogRead(_volt_pin_c);
    //
    // _voltage_a = raw_volt_v_a * VOLTAGE_SENSE_CONVERSION_FACTOR;
    // _voltage_b = raw_volt_v_b * VOLTAGE_SENSE_CONVERSION_FACTOR;
    // _voltage_c = raw_volt_v_c * VOLTAGE_SENSE_CONVERSION_FACTOR;
    //
    // if (volt_filtered) {
    //     _voltage_a = _volt_filter_a->update(_voltage_a, current_time_us);
    //     _voltage_b = _volt_filter_b->update(_voltage_b, current_time_us);
    //     _voltage_c = _volt_filter_c->update(_voltage_c, current_time_us);
    // }
    // ================== CURRENT SENSING ==================
    // 1. Fetch raw values from the Injected Data Registers
    uint32_t raw_curr_v_a = HAL_ADCEx_InjectedGetValue(hadc3, ADC_INJECTED_RANK_1);
    uint32_t raw_curr_v_b = HAL_ADCEx_InjectedGetValue(hadc2, ADC_INJECTED_RANK_1);
    uint32_t raw_curr_v_c = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_1);

    // 2. Convert to Volts
    float curr_v_a = raw_curr_v_a * CURRENT_SENSE_CONVERSION_FACTOR;
    float curr_v_b = raw_curr_v_b * CURRENT_SENSE_CONVERSION_FACTOR;
    float curr_v_c = raw_curr_v_c * CURRENT_SENSE_CONVERSION_FACTOR;


    // 3. Calculate actual Amps with Offsets
    // Formula: (Vref/2 - Vread) / (Gain * Rsense)
    float curr_a = (3.3/2.0 - curr_v_a) / (_csa_gain * R_SENSE) + (curr_inverted ? _curr_offset_a : -_curr_offset_a);
    float curr_b = (3.3/2.0 - curr_v_b) / (_csa_gain * R_SENSE) + (curr_inverted ? _curr_offset_b : -_curr_offset_b);
    float curr_c = (3.3/2.0 - curr_v_c) / (_csa_gain * R_SENSE) + (curr_inverted ? _curr_offset_c : -_curr_offset_c);

    // Invert if necessary
    if (curr_inverted) {
        curr_a *= -1; curr_b *= -1; curr_c *= -1;
    }

    // 4. Apply Filtering and Anomaly Detection
    if (curr_filtered) {
        _current_a = (abs(curr_a) < ANOMALY_THRESHOLD) ? _curr_filter_a->update(curr_a, current_time_us) : _current_a;
        _current_b = (abs(curr_b) < ANOMALY_THRESHOLD) ? _curr_filter_b->update(curr_b, current_time_us) : _current_b;
        _current_c = (abs(curr_c) < ANOMALY_THRESHOLD) ? _curr_filter_c->update(curr_c, current_time_us) : _current_c;
    } else {
        _current_a = (abs(curr_a) < ANOMALY_THRESHOLD) ? curr_a : _current_a;
        _current_b = (abs(curr_b) < ANOMALY_THRESHOLD) ? curr_b : _current_b;
        _current_c = (abs(curr_c) < ANOMALY_THRESHOLD) ? curr_c : _current_c;
    }
}

PhaseCurrents Adc::get_phase_currents() {
    return {_current_a, _current_b, _current_c};
}

PhaseVoltages Adc::get_phase_voltages() {
    return {_voltage_a, _voltage_b, _voltage_c};
}

float Adc::get_supply_voltage() {
    // Read raw value from supply sense pin
    return (analogRead(_supply_sense_pin) * VOLTAGE_SENSE_CONVERSION_FACTOR);
}

float Adc::get_bemf_angle() {
    // Estimate electrical angle from voltage measurements using atan2
    // This is a very rough estimate and assumes we're in a state where voltage measurements are meaningful
    PhaseVoltages voltages = get_phase_voltages();
    ab_struct ab = clarke_transform({voltages.voltage_a, voltages.voltage_b, voltages.voltage_c});
    float theta = atan2(ab.beta, ab.alpha);
    return theta;
}