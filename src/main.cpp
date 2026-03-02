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
    // maxwell.current_sensors->calibrate_offsets();
    // maxwell.driver->clear_fault();
    //
}


void loop() {
    maxwell.motor_control();
}
