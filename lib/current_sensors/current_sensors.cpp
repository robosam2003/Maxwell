//
// Created by robosam on 2/3/25.
//

#include "current_sensors.h"

CurrentSensors::CurrentSensors(PinName pin_a, PinName pin_b, PinName pin_c, DRV8323::CSA_GAIN gain) {
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

    double cuttoff_freq = 10;
    _filter_a = new RCFilter(cuttoff_freq);
    _filter_b = new RCFilter(cuttoff_freq);
    _filter_c = new RCFilter(cuttoff_freq);

    _csa_gain = gain;
    hadc1 = new ADC_HandleTypeDef();


  setup_injected_adc();
}

void CurrentSensors::setup_injected_adc() {
    // Assuming all pins belong to the same ADC (they do)
    // Enable clocks
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    pinMode(_pin_a, INPUT_ANALOG); // PB_0 - ADC1_IN8
    pinMode(_pin_b, INPUT_ANALOG); // PC_5 - ADC1_IN15
    pinMode(_pin_c, INPUT_ANALOG); // PC_4 - ADC1_IN14

    hadc1->Instance = ADC1;
    hadc1->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
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
        delay(1000);
      }
    }

    ADC_InjectionConfTypeDef sConfigInjected;
    sConfigInjected.InjectedNbrOfConversion = 3;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;

    // first channel
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedChannel = ADC_CHANNEL_8; // PB_0 - ADC1_IN8
    if (HAL_ADCEx_InjectedConfigChannel(hadc1, &sConfigInjected) != HAL_OK){
        Serial.println("Error configuring ADC channel 1");
    }
    // second channel
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedChannel = ADC_CHANNEL_15; // PC_5 - ADC1_IN15
    if (HAL_ADCEx_InjectedConfigChannel(hadc1, &sConfigInjected) != HAL_OK){
        Serial.println("Error configuring ADC channel 2");
    }
    // third channel
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    sConfigInjected.InjectedChannel = ADC_CHANNEL_14; // PC_4 - ADC1_IN14
    if (HAL_ADCEx_InjectedConfigChannel(hadc1, &sConfigInjected) != HAL_OK){
        Serial.println("Error configuring ADC channel 3");
    }

    // Start the injected conversion
    int start_code = HAL_ADCEx_InjectedStart(hadc1);
    if (start_code != HAL_OK) {
      for (int i=0; i<10; i++) {
        Serial.println("Error starting injected conversion");
        Serial.println(start_code);
        delay(1000);
      }

    }
    Serial.println("SUCCESSFULLY SETUP INJECTED ADC");
}

void CurrentSensors::set_csa_gain(DRV8323::CSA_GAIN gain) {
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
  for (int i = 0; i< num_samples; i++) {
    // Let the value settle
    get_current_a();
    get_current_b();
    get_current_c();
    delay(1);
  }
  for (int i = 0; i < num_samples; i++) {
    sum_a += get_current_a();
    sum_b += get_current_b();
    sum_c += get_current_c();
    delay(1);
  }
  _offset_a = sum_a / num_samples;
  _offset_b = sum_b / num_samples;
  _offset_c = sum_c / num_samples;
  }

void CurrentSensors::read() {
  // HAL_ADCEx_InjectedPollForConversion(hadc1, 100);
  v_a = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_1) * CURRENT_SENSE_CONVERSION_FACTOR;
  v_b = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_2) * CURRENT_SENSE_CONVERSION_FACTOR;
  v_c = HAL_ADCEx_InjectedGetValue(hadc1, ADC_INJECTED_RANK_3) * CURRENT_SENSE_CONVERSION_FACTOR;

  if (filtered) {
    _current_a = _filter_a->update((3.3/2 - v_a) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_a, micros());
    _current_b = _filter_b->update((3.3/2 - v_b) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_b, micros());
    _current_c = _filter_c->update((3.3/2 - v_c) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_c, micros());
  }
  else {
    _current_a = (3.3/2 - v_a) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_a;
    _current_b = (3.3/2 - v_b) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_b;
    _current_c = (3.3/2 - v_c) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_a;
  }
}


double CurrentSensors::get_current_a() {
  return v_a;
}

double CurrentSensors::get_current_b() {
  return v_b;
}

double CurrentSensors::get_current_c() {
  return v_c;
}

double* CurrentSensors::get_currents() {
  double currents[3] = {_current_a, _current_b, _current_c};
  return currents;
}

double CurrentSensors::get_total_current() {
  return _current_a + _current_b + _current_c;
}



