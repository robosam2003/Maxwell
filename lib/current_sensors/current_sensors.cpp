//
// Created by robosam on 2/3/25.
//

#include "current_sensors.h"

CurrentSensors::CurrentSensors(int pin_a, int pin_b, int pin_c, DRV8323::CSA_GAIN gain) {
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

  double cuttoff_freq = 0.5;
  _filter_a = new RCFilter(cuttoff_freq);
  _filter_b = new RCFilter(cuttoff_freq);
  _filter_c = new RCFilter(cuttoff_freq);

  _csa_gain = gain;

  // Set analog read resolution to 12 bits

  // set pins
  pinMode(_pin_a, INPUT_ANALOG);
  pinMode(_pin_b, INPUT_ANALOG);
  pinMode(_pin_c, INPUT_ANALOG);

  // Set the ADCs to continuous sampling


  // calibrate_offsets();
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
    this->read();
    delay(1);
  }
  for (int i = 0; i < num_samples; i++) {
    this->read();
    sum_a += _current_a;
    sum_b += _current_b;
    sum_c += _current_c;
    delay(1);
  }
  _offset_a = sum_a / num_samples;
  _offset_b = sum_b / num_samples;
  _offset_c = sum_c / num_samples;
  }

void CurrentSensors::read() {
  double v_a = (analogRead(_pin_a)) * CURRENT_SENSE_CONVERSION_FACTOR;
  double v_b = (analogRead(_pin_b)) * CURRENT_SENSE_CONVERSION_FACTOR;
  double v_c = (analogRead(_pin_c)) * CURRENT_SENSE_CONVERSION_FACTOR;
  uint32_t current_time_us = micros();

  _current_a = _filter_a->update((3.3/2 - v_a) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_a, current_time_us);
  _current_b = _filter_b->update((3.3/2 - v_b) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_b, current_time_us);
  _current_c = _filter_c->update((3.3/2 - v_c) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_c, current_time_us);
  // _current_a = (3.3/2 - v_a) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_a;
  // _current_b = (3.3/2 - v_b) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_b;
  // _current_c = (3.3/2 - v_c) / (DRV8323::csa_gain_to_int[_csa_gain] * R_SENSE) - _offset_c;
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

double CurrentSensors::get_total_current() {
  return _current_a + _current_b + _current_c;
}



