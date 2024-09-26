#include "pin_definitions.h"
#include "Maxwell.h"
#include "DRV8323.h"
#include "FreeRTOS.h"
#include "AS5047P.h"



SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK
SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK

DRV8323::DRV8323 drv8323(DRV8323_CS_PIN, SPI_1, 1000000, DRV8323_GATE_EN_PIN);

AS5047P::AS5047P encoder(AS5047P_CS_PIN, SPI_2, 1000000);


double prev_cross_voltage = -1;

void setup() {
    SerialUSB.begin(9600);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(DRV8323_HI_A_PIN, OUTPUT);
    pinMode(DRV8323_HI_B_PIN, OUTPUT);
    pinMode(DRV8323_HI_C_PIN, OUTPUT);
    pinMode(DRV8323_LO_A_PIN, OUTPUT);
    pinMode(DRV8323_LO_B_PIN, OUTPUT);
    pinMode(DRV8323_LO_C_PIN, OUTPUT);

    pinMode(DRV8323_GATE_EN_PIN, OUTPUT);
    pinMode(DRV8323_DRIVE_CAL_PIN, OUTPUT);

    digitalWrite(DRV8323_DRIVE_CAL_PIN, LOW);

    drv8323.default_configuration(); // set up the gate driver


}


void sense_print_voltages() {
    uint32_t v_a = analogRead(V_SENSE_A_PIN);
    uint32_t v_b = analogRead(V_SENSE_B_PIN);
    uint32_t v_c = analogRead(V_SENSE_C_PIN);
    uint32_t v_supply = analogRead(V_SUPPLY_SENSE_PIN);

    // Convert to voltages:
    float v_a_v = v_a * SENSE_CONVERSION_FACTOR;
    float v_b_v = v_b * SENSE_CONVERSION_FACTOR;
    float v_c_v = v_c * SENSE_CONVERSION_FACTOR;
    float v_supply_v = v_supply * SENSE_CONVERSION_FACTOR;

    // Print the voltages:
    Serial.print(v_a_v); Serial.print(" ");
    Serial.print(v_b_v); Serial.print(" ");
    Serial.println(v_c_v);
//    Serial.println(v_supply_v);
}

void sense_print_currents() {
    float G = 20;
    float R_sense = 0.4e-3;
    float VREF = 3.3;
    auto V_SOa = ((double) analogRead(DRV8323_CURR_SENSE_A_PIN)) * CURRENT_SENSE_CONVERSION_FACTOR;
    auto V_SOb = ((double) analogRead(DRV8323_CURR_SENSE_B_PIN)) * CURRENT_SENSE_CONVERSION_FACTOR;
    auto V_SOc = ((double) analogRead(DRV8323_CURR_SENSE_C_PIN)) * CURRENT_SENSE_CONVERSION_FACTOR;

    // Convert to currents:
    double i_a = ((VREF/2 - V_SOa) / (G * R_sense))- 205.85;
    double i_b = ((VREF/2 - V_SOb) / (G * R_sense))-  141.80 ;
    double i_c = ((VREF/2 - V_SOc) / (G * R_sense))-  114.00;


    // Print the voltages:
    Serial.print(i_a); Serial.print(" ");
    Serial.print(i_b); Serial.print(" ");
    Serial.print(i_c); Serial.print(" ");
    Serial.println();
}

uint8_t six_step_commutation_states[6][3] = {
        {HIGH, HIGH, LOW},
        {HIGH, LOW, LOW},
        {HIGH, LOW, HIGH},
        {LOW, LOW, HIGH},
        {LOW, HIGH, HIGH},
        {LOW, HIGH, LOW}
};
uint8_t six_step_high_impedance_phase[6] = {
        V_SENSE_A_PIN,
        V_SENSE_B_PIN,
        V_SENSE_C_PIN,
        V_SENSE_A_PIN,
        V_SENSE_B_PIN,
        V_SENSE_C_PIN
};

void delay_while_excecuting_func(int del, void (*func)(uint8_t arg), uint8_t arg) {
    unsigned long start_time = micros();
    while (micros() - start_time < del) {
        func(arg);
    }
}



bool check_zero_cross(uint8_t pin) {
    uint32_t v_p = analogRead(pin);
    uint32_t v_supply = analogRead(V_SUPPLY_SENSE_PIN);
    if (prev_cross_voltage == -1) {
        prev_cross_voltage = v_p;
        return NULL;
    }
    if ((v_p > prev_cross_voltage && v_p > 0.5 * v_supply && prev_cross_voltage < 0.5 * v_supply)
        || (v_p < prev_cross_voltage && v_p < 0.5 * v_supply && prev_cross_voltage > 0.5 * v_supply)) {
        Serial.println("Zero cross detected!!!!!!!!!!!");
        prev_cross_voltage = -1;
        return true;
    }
    prev_cross_voltage = v_p;
    return false;
}   

void delay_while_checking_zero_cross(int del, uint8_t pin) {
    unsigned long start_time = millis();
    while (millis() - start_time < del) {
        bool cross = check_zero_cross(pin);
        if (cross) {
            break;
        }
    }
}

void six_step_commutation_loop(int del, int level, int num_loops) {
    drv8323.default_configuration(); // set up the gate driver
    digitalWrite(DRV8323_BRAKE_PIN, HIGH); // set the brake pin to high - disables brake.
    digitalWrite(DRV8323_DIR_PIN, HIGH); // set the direction pin to high

//    // STOP and align procedure
//    // STOP
//    digitalWrite(DRV8323_STATE0_PIN, LOW);
//    digitalWrite(DRV8323_STATE1_PIN, LOW);
//    digitalWrite(DRV8323_STATE2_PIN, LOW);
//    delay(100);
//
////    // ALIGN
    digitalWrite(DRV8323_STATE0_PIN, HIGH);
    digitalWrite(DRV8323_STATE1_PIN, HIGH);
    digitalWrite(DRV8323_STATE2_PIN, HIGH);
    delay(del*10);

    analogWrite(DRV8323_PWM1X_PIN, level); // set the pwm pin to the pwm level out of 255

    for (int i = 0; i < num_loops; i++) {
        for (int j = 0; j < 6; j++) {
            digitalWrite(DRV8323_STATE0_PIN, six_step_commutation_states[j][0]);
            digitalWrite(DRV8323_STATE1_PIN, six_step_commutation_states[j][1]);
            digitalWrite(DRV8323_STATE2_PIN, six_step_commutation_states[j][2]);
            Serial.print(six_step_commutation_states[j][0]);
            Serial.print(six_step_commutation_states[j][1]);
            Serial.println(six_step_commutation_states[j][2]);
            sense_print_voltages();
            sense_print_currents();
            Serial.println(drv8323.get_fault_status_1_string());
            Serial.println(drv8323.get_fault_status_2_string());
//            drv8323.clear_fault();
            uint8_t arg = six_step_high_impedance_phase[j];
            delay_while_excecuting_func(del, reinterpret_cast<void (*)(uint8_t)>(sense_print_voltages), 0);
//            delay_while_checking_zero_cross(del, six_step_high_impedance_phase[j]);

//            delay(del);
        }
        Serial.println("----------------------");
    }

    digitalWrite(DRV8323_STATE0_PIN, LOW);
    digitalWrite(DRV8323_STATE1_PIN, LOW);
    digitalWrite(DRV8323_STATE2_PIN, LOW);
    delay(3*del);
}

void motor_start(int level) {
    drv8323.default_configuration(); // set up the gate driver
    digitalWrite(DRV8323_BRAKE_PIN, HIGH); // set the brake pin to high - disables brake.
    digitalWrite(DRV8323_DIR_PIN, HIGH); // set the direction pin to high
    analogWrite(DRV8323_PWM1X_PIN, level); // set the pwm pin to the pwm level out of 255


    digitalWrite(DRV8323_STATE0_PIN, HIGH);
    digitalWrite(DRV8323_STATE1_PIN, HIGH);
    digitalWrite(DRV8323_STATE2_PIN, HIGH);
    delay(100);

//    analogWrite(DRV8323_PWM1X_PIN, level); // set the pwm pin to the pwm level out of 255
}

void open_loop_acceleration(int level, int start_del_us, int end_del_us, int step_del_us) {
    analogWrite(DRV8323_PWM1X_PIN, level); // set the pwm pin to the pwm level out of 255

    for (int i = start_del_us; i > end_del_us; i -= step_del_us) {
        int del_us = i;
        for (int j = 0; j < 6; j++) {
            digitalWrite(DRV8323_STATE0_PIN, six_step_commutation_states[j][0]);
            digitalWrite(DRV8323_STATE1_PIN, six_step_commutation_states[j][1]);
            digitalWrite(DRV8323_STATE2_PIN, six_step_commutation_states[j][2]);
            Serial.print(six_step_commutation_states[j][0]);
            Serial.print(six_step_commutation_states[j][1]);
            Serial.println(six_step_commutation_states[j][2]);
            sense_print_voltages();
            sense_print_currents();
            Serial.println(drv8323.get_fault_status_1_string());
            Serial.println(drv8323.get_fault_status_2_string());
//            drv8323.clear_fault();
            uint8_t arg = six_step_high_impedance_phase[j];
            delay_while_excecuting_func(del_us, reinterpret_cast<void (*)(uint8_t)>(sense_print_voltages), 0);
//            delay_while_checking_zero_cross(del, six_step_high_impedance_phase[j]);

//            delay(del);
        }
        Serial.println("----------------------");
    }
}

void closed_loop_control(int level, int del_us, int num_loops) {
    for (int i = 0; i < num_loops; i++) {
        for (int j = 0; j < 6; j++) {
            digitalWrite(DRV8323_STATE0_PIN, six_step_commutation_states[j][0]);
            digitalWrite(DRV8323_STATE1_PIN, six_step_commutation_states[j][1]);
            digitalWrite(DRV8323_STATE2_PIN, six_step_commutation_states[j][2]);
            Serial.print(six_step_commutation_states[j][0]);
            Serial.print(six_step_commutation_states[j][1]);
            Serial.println(six_step_commutation_states[j][2]);
            sense_print_voltages();
//            sense_print_currents();
            Serial.println(drv8323.get_fault_status_1_string());
            Serial.println(drv8323.get_fault_status_2_string());
//            drv8323.clear_fault();
            uint8_t arg = six_step_high_impedance_phase[j];
            delay_while_excecuting_func(del_us, reinterpret_cast<void (*)(uint8_t)>(sense_print_voltages), 0);
        }
        Serial.println("----------------------");
    }
}

void motor_stop() {
    analogWrite(DRV8323_PWM1X_PIN, 0); // set the pwm pin to the pwm level out of 255

    digitalWrite(DRV8323_BRAKE_PIN, LOW); // set the brake pin to LOW - enables brake

//    digitalWrite(DRV8323_STATE0_PIN, LOW);
//    digitalWrite(DRV8323_STATE1_PIN, LOW);
//    digitalWrite(DRV8323_STATE2_PIN, LOW);
    delay(100);
}

void loop() {
//    six_step_commutation_loop(8, 5, 100);
//    Serial.println("=====================================");
//    delay(2000);
//    motor_start(10);
//    open_loop_acceleration(7, 10000, 100, 100);
////    closed_loop_control(5, 100, 2000);
//    motor_stop();
//    delay(5000);
    uint16_t angle = encoder.get_angle();
    uint16_t mag = encoder.get_mag_strength();
    Serial.printf("Angle: %d, Mag: %d\n", angle, mag);
    delay(100);
}