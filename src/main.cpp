#define SERIAL_FEEDBACK_ENABLED
#define MAXWELL_STUDIO_OUTPUT
#define HARDWARE_V2_0

#include <Arduino.h>
#include "Maxwell.h"
#include "stm32f4xx_hal_flash.h"


Maxwell::Maxwell maxwell;
// SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK
//
// AS5047P encoder(
//     AS5048A_CS_PIN,
//     SPI_2,
//     1000000); // 1 MHz SPI frequency


void setup() {
    delay(500);

    maxwell.setup();
    // // maxwell.init_pwm_3x();
    // // maxwell.set_phase_voltages(0, 0, 0);
    maxwell.adc->calibrate_current_offsets();
    maxwell.driver->clear_fault();
}


void loop() {
    // Serial.println(maxwell.driver->read_reg(DRV8323::DRIVER_CONTROL), HEX);
    // delay(10);
    maxwell.motor_control();
    // Serial.println(pwm_input.read_percentage());
    // delay(100);
    // maxwell.motor_control();
    // maxwell.adc->read();
    // Serial.println("Supply Voltage: " + String(maxwell.adc->get_supply_voltage()));
    // delay(100);

    //
    // maxwell.encoder->update();
    // Serial.println("Angle: " + String(maxwell.encoder->absolute_angle));
    // delay(10);

}



// maxwell_config default_config_struct = {
//     COMMAND_SOURCE::PWM,
//     TELEMETRY_TARGET::TELEMETRY_USB,
//     MOTOR_TYPE::BLDC,
//     CONTROL_MODE::POSITION,
//     SENSOR_TYPE::MAGNETIC,
//     TORQUE_CONTROL_MODE::CURRENT,
//     SENSOR_LOCATION::EXTERNAL_PORT,
//     POLE_PAIRS_6374,
//     0.0,
//     {2.0, 0, 0, 0.0, 1},
//     {2.0, 0, 0, 0.0, 3},
//     {0.05, 1.0, 0, 20, 20},
//     {20, 0, 0, 0.0, 0.0},
//     {true, 2.0},
//     {true, 2.0},
//     {true, 10.0},
//     {true, 10.0}
// };
// void setup() {
//     Serial.begin(115200);
// }
//
// void loop() {
// // Wait for serial command:
//     // Check if data is available
//     if (Serial.available() > 0) {
//         String result = Serial.readStringUntil('\n');
//         if (result[0] == ('0')+COMMANDS::READ_CONFIG) { // If the command is to read the config, send the default config struct as bytes
//             for (size_t i = 0; i < sizeof(maxwell_config); i++) {
//                 Serial.write(((uint8_t*)&default_config_struct)[i]);
//             }
//             Serial.println();
//         }
//         else {
//             Serial.println(result);
//         }
//         // Serial.println("Received command: " + String(read_buffer));
//         // Serial.flush();
//     }
// }