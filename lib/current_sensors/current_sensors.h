//
// Created by robosam on 2/3/25.
//

#include "Arduino.h"
#include "RCFilter.h"
#ifndef CURRENT_SENSORS_H
#define CURRENT_SENSORS_H

class CurrentSensors {
// TODO: ARE THESE EVEN SET UP PROPERLY IN THE DRV8323 LIBRARY?
private:
  double R_SENSE = 400e-6;
  double G_CA = 40; // gain of current amplifier - V/V

  int _pin_a;
  int _pin_b;
  int _pin_c;
  double _offset_a;  // in AMPS
  double _offset_b;
  double _offset_c;

//#define CURRENT_SENSE_CONVERSION_FACTOR ((3.3 / 1024))
  double _conversion_factor;
  RCFilter* _filter_a;
  RCFilter* _filter_b;
  RCFilter* _filter_c;

public:
  double _current_a;
  double _current_b;
  double _current_c;

  CurrentSensors(int pin_a, int pin_b, int pin_c, double conversion_factor);
  void calibrate_offsets();
  void read();
  float get_current_a();
  float get_current_b();
  float get_current_c();
  float get_total_current();
};

#endif //CURRENT_SENSORS_H
