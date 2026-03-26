#define SERIAL_FEEDBACK_ENABLED
#define MAXWELL_STUDIO_OUTPUT
#define HARDWARE_V2_0

#include <Arduino.h>
#include "Maxwell.h"
#include "stm32f4xx_hal_flash.h"


Maxwell::Maxwell maxwell;

void setup() {
    delay(500);

    maxwell.setup();
    // maxwell.init_pwm_3x();
    // maxwell.set_phase_voltages(0, 0, 0);
    // maxwell.adc->calibrate_current_offsets();
    maxwell.driver->clear_fault();
}


void loop() {
    // Serial.println(maxwell.driver->read_reg(DRV8323::DRIVER_CONTROL), HEX);
    // delay(10);
    // maxwell.motor_control();
    maxwell.motor_control();

    //
    // maxwell.encoder->update();
    // Serial.println(maxwell.encoder->get_angle());
}
