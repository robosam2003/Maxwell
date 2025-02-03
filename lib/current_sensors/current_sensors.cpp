//
// Created by robosam on 2/3/25.
//

#include "current_sensors.h"

CurrentSensors::CurrentSensors(int pin_a, int pin_b, int pin_c, float conversion_factor) {
  _pin_a = pin_a;
  _pin_b = pin_b;
  _pin_c = pin_c;
  _conversion_factor = conversion_factor;
  _current_a = 0;
  _current_b = 0;
  _current_c = 0;

  pinMode(_pin_a, INPUT);
  pinMode(_pin_b, INPUT);
  pinMode(_pin_c, INPUT);
  calibrate_offsets();
}

void CurrentSensors::calibrate_offsets() {
  int num_samples = 100;
  int sum_a = 0;
  int sum_b = 0;
  int sum_c = 0;
  for (int i = 0; i < num_samples; i++) {
    sum_a += analogRead(_pin_a);
    sum_b += analogRead(_pin_b);
    sum_c += analogRead(_pin_c);
    delay(1);
  }
  _offset_a = sum_a / num_samples;
  _offset_b = sum_b / num_samples;
  _offset_c = sum_c / num_samples;
  }

void CurrentSensors::read() {
  _current_a = (analogRead(_pin_a) - _offset_a) * _conversion_factor;
  _current_b = (analogRead(_pin_b) - _offset_b) * _conversion_factor;
  _current_c = (analogRead(_pin_c) - _offset_c) * _conversion_factor;
}

float CurrentSensors::get_current_a() {
  return _current_a;
}

float CurrentSensors::get_current_b() {
  return _current_b;
}

float CurrentSensors::get_current_c() {
  return _current_c;
}




