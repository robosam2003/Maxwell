//
// Created by robosam on 2/3/25.
//

#include "Arduino.h"
#ifndef CURRENT_SENSORS_H
#define CURRENT_SENSORS_H

class CurrentSensors {
private:
  int _pin_a;
  int _pin_b;
  int _pin_c;
  float _offset_a;
  float _offset_b;
  float _offset_c;
//#define CURRENT_SENSE_CONVERSION_FACTOR ((3.3 / 1024))
  float _conversion_factor;
public:

  float _current_a;
  float _current_b;
  float _current_c;

  CurrentSensors(int pin_a, int pin_b, int pin_c, float conversion_factor);
  void calibrate_offsets();
  void read();
  float get_current_a();
  float get_current_b();
  float get_current_c();
};

#endif //CURRENT_SENSORS_H
