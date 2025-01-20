//
// Created by robos on 11/09/2023.
//

#include "Maxwell.h"

namespace Maxwell {




    Maxwell::Maxwell()  {
        SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK
        // SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK

        driver = new DRV8323::DRV8323(
            DRV8323_CS_PIN,
            SPI_1,
            1000000,
            DRV8323_GATE_EN_PIN);

        // encoder = new AS5047P::AS5047P(
        //     AS5047P_CS_PIN,
        //     SPI_2,
        //     1000000);

        trigger = new triggered{false, false, false};
    }

    void read_voltage_state(void* pvParameters) {
        triggered* trigger = (triggered*) pvParameters;

        for (;;) {
        //     double voltage_a = analogRead(V_SENSE_A_PIN) * SENSE_CONVERSION_FACTOR;
        //     double voltage_b = analogRead(V_SENSE_B_PIN) * SENSE_CONVERSION_FACTOR;
        //     double voltage_c = analogRead(V_SENSE_C_PIN) * SENSE_CONVERSION_FACTOR;
        //     double supply_voltage = analogRead(V_SUPPLY_SENSE_PIN) * SENSE_CONVERSION_FACTOR;
        //     // Serial.print(voltage_a); Serial.print(", ");
        //     // Serial.print(voltage_b); Serial.print(", ");
        //     // Serial.print(voltage_c); Serial.print(", ");
        //     // Serial.print(supply_voltage); Serial.print("\n");
        //
        //     double half_supply_voltage = supply_voltage / 3;
        //     (trigger->phases[0] != (voltage_a > half_supply_voltage)) ? trigger->zero_cross[0] = true : trigger->zero_cross[0] = false;
        //     (trigger->phases[1] != (voltage_b > half_supply_voltage)) ? trigger->zero_cross[1] = true : trigger->zero_cross[1] = false;
        //     (trigger->phases[2] != (voltage_c > half_supply_voltage)) ? trigger->zero_cross[2] = true : trigger->zero_cross[2] = false;
        //     (voltage_a > half_supply_voltage) ? trigger->phases[0] = true : trigger->phases[0] = false;
        //     (voltage_b > half_supply_voltage) ? trigger->phases[1] = true : trigger->phases[1] = false;
        //     (voltage_c > half_supply_voltage) ? trigger->phases[2] = true : trigger->phases[2] = false;
        //     if (trigger->zero_cross[0] | trigger->zero_cross[1] | trigger->zero_cross[2]) {
        //         Serial.print(trigger->zero_cross[0]); Serial.print(", ");
        //         Serial.print(trigger->zero_cross[1]); Serial.print(", ");
        //         Serial.print(trigger->zero_cross[2]); Serial.print("\n");
        //     }
        //
        //     digitalToggle(GREEN_LED_PIN);
        //     // Serial.println("HELOOOO");
        //     // vTaskDelay(1);
            uint32_t hall_a = digitalRead(HALL_A_PIN);
            uint32_t hall_b = digitalRead(HALL_B_PIN);
            uint32_t hall_c = digitalRead(HALL_C_PIN);
            Serial.print(hall_a); Serial.print(", ");
            Serial.print(hall_b); Serial.print(", ");
            Serial.print(hall_c); Serial.print("\n");
            vTaskDelay(1);
            digitalToggle(GREEN_LED_PIN);
        }

    }

    void Maxwell::setup() {
        pinMode(HALL_A_PIN, INPUT);
        pinMode(HALL_B_PIN, INPUT);
        pinMode(HALL_C_PIN, INPUT);

        pinMode(GREEN_LED_PIN, OUTPUT);
        // Breakout to a driver class?
        pinMode(DRV8323_HI_A_PIN, OUTPUT);
        pinMode(DRV8323_HI_B_PIN, OUTPUT);
        pinMode(DRV8323_HI_C_PIN, OUTPUT);
        pinMode(DRV8323_LO_A_PIN, OUTPUT);
        pinMode(DRV8323_LO_B_PIN, OUTPUT);
        pinMode(DRV8323_LO_C_PIN, OUTPUT);
        pinMode(DRV8323_GATE_EN_PIN, OUTPUT);
        pinMode(DRV8323_DRIVE_CAL_PIN, OUTPUT);

        pinMode(V_SENSE_A_PIN, INPUT);
        pinMode(V_SENSE_B_PIN, INPUT);
        pinMode(V_SENSE_C_PIN, INPUT);
        pinMode(V_SUPPLY_SENSE_PIN, INPUT);

        pinMode(DRV8323_CURR_SENSE_A_PIN, INPUT);
        pinMode(DRV8323_CURR_SENSE_B_PIN, INPUT);
        pinMode(DRV8323_CURR_SENSE_C_PIN, INPUT);


        digitalWrite(DRV8323_DRIVE_CAL_PIN, LOW);

        pinMode(HALL_A_PIN, INPUT_PULLUP);
        pinMode(HALL_B_PIN, INPUT_PULLUP);
        pinMode(HALL_C_PIN, INPUT_PULLUP);
        // pinMode(HALL_TEMP_PIN, INPUT);

        // // Configure stm32 adc watchdog
        // ADC1->CR1 |= ADC_CR1_AWDEN;



        // xTaskCreate(
        //     read_voltage_state,
        //     "Read Voltage State",
        //     1000,
        //     (void*) trigger,
        //     1,
        //     NULL
        // );
    }

    void Maxwell::drive_hall_velocity(int velocity, int duration) { // velocity is in ms, duration is in ms
        uint8_t six_step_commutation_states[6][6] = {
            {0, 0, 1, 0, 0, 1}, // 1 - A - B->C
            {1, 0, 0, 0, 0, 1}, // 2 - B - A->C
            {1, 0, 0, 1, 0, 0}, // 3 - C - A->B
            {0, 0, 0, 1, 1, 0}, // 4 - A - C->B
            {0, 1, 0, 0, 1, 0}, // 5 - B - C->A
            {0, 1, 1, 0, 0, 0}  // 6 - C - B->A
        };

        uint8_t hall_states[6][3] = {
            {0, 0, 1},
            {0, 1, 1},
            {0, 1, 0},
            {1, 1, 0},
            {1, 0, 0},
            {1, 0, 1}
        };

        // Running in 6x PWM mode
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_6x);

        // Enable the driver
        driver->enable(true);
        digitalWrite(DRV8323_BRAKE_PIN, HIGH); // set the brake pin to high - disables brake.
        digitalWrite(DRV8323_DIR_PIN, HIGH);    // set the direction pin to high

        int level = 40;
        int step = 0;
        uint32_t start = millis();
        int i = 0;
        for (; millis() - start < duration/2;) {
            uint32_t hall_a = digitalRead(HALL_A_PIN);
            uint32_t hall_b = digitalRead(HALL_B_PIN);
            uint32_t hall_c = digitalRead(HALL_C_PIN);
            Serial.print(hall_a); Serial.print(", ");
            Serial.print(hall_b); Serial.print(", ");
            Serial.print(hall_c); Serial.print("\n");

            uint32_t hall_state[3] = {hall_a, hall_b, hall_c};

            // Match the state of the hall sensors to the commutation states
            for (int j = 0; j < 6; j++) {
                if (hall_state[0] == hall_states[j][0] && hall_state[1] == hall_states[j][1] && hall_state[2] == hall_states[j][2]) {
                    i = j;
                    break;
                }
            }
            step = i % 6;
            analogWrite(DRV8323_HI_A_PIN, six_step_commutation_states[step][0] * level);
            analogWrite(DRV8323_LO_A_PIN, six_step_commutation_states[step][1] * level);
            analogWrite(DRV8323_HI_B_PIN, six_step_commutation_states[step][2] * level);
            analogWrite(DRV8323_LO_B_PIN, six_step_commutation_states[step][3] * level);
            analogWrite(DRV8323_HI_C_PIN, six_step_commutation_states[step][4] * level);
            analogWrite(DRV8323_LO_C_PIN, six_step_commutation_states[step][5] * level);
            // i++;
        }


    }


    void Maxwell::drive_velocity(int velocity, int duration) { // velocity is in ms, duration is in ms
        // constrain velocity to 0-255
        velocity = constrain(velocity, 0, 255);

        uint8_t six_step_commutation_states[6][6] = {
            //  AH,AL,BH,BL,CH,CL
            {1, 0, 0, 0, 0, 1}, // 1 - B - A->C
            {0, 0, 1, 0, 0, 1}, // 2 - A - B->C
            {0, 1, 1, 0, 0, 0}, // 3 - C - B->A
            {0, 1, 0, 0, 1, 0}, // 4 - B - C->A
            {0, 0, 0, 1, 1, 0}, // 5 - A - C->B
            {1, 0, 0, 1, 0, 0}  // 6 - C - A->B
        };
        // Running in 6x PWM mode
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_6x);

        // Enable the driver
        driver->enable(true);
        digitalWrite(DRV8323_BRAKE_PIN, HIGH); // set the brake pin to high - disables brake.
        digitalWrite(DRV8323_DIR_PIN, HIGH);    // set the direction pin to high

        int level = 40;
        int on_time = 15;
        int off_time = 1;
        int step = 0;
        int high_impedance_phase = 0;

        // double align
        analogWrite(DRV8323_HI_A_PIN, 70);
        analogWrite(DRV8323_LO_B_PIN, 70);
        vTaskDelay(100);

        analogWrite(DRV8323_HI_A_PIN, 70);
        analogWrite(DRV8323_LO_C_PIN, 70);
        vTaskDelay(100);

        uint32_t start = millis();
        int i = 0;
        for (; millis() - start < duration/2;) {
            on_time = constrain(on_time, 5, 100);
            step = i % 6;
            // high_impedance_phase = (step+2) % 3;
            analogWrite(DRV8323_HI_A_PIN, six_step_commutation_states[step][0] * level);
            analogWrite(DRV8323_LO_A_PIN, six_step_commutation_states[step][1] * level);
            analogWrite(DRV8323_HI_B_PIN, six_step_commutation_states[step][2] * level);
            analogWrite(DRV8323_LO_B_PIN, six_step_commutation_states[step][3] * level);
            analogWrite(DRV8323_HI_C_PIN, six_step_commutation_states[step][4] * level);
            analogWrite(DRV8323_LO_C_PIN, six_step_commutation_states[step][5] * level);
            vTaskDelay(on_time);

            // analogWrite(DRV8323_HI_A_PIN, 0);
            // analogWrite(DRV8323_HI_B_PIN, 0);
            // analogWrite(DRV8323_HI_C_PIN, 0);
            // analogWrite(DRV8323_LO_A_PIN, 0);
            // analogWrite(DRV8323_LO_B_PIN, 0);
            // analogWrite(DRV8323_LO_C_PIN, 0);
            // vTaskDelay(off_time);
            if (i % 6 == 0) {
                on_time--;
            }
            i++;
        }
        // start = millis();
        // for (; millis() - start < duration/2; ) {
        //     // Serial.println(driver->get_fault_status_1_string());
        //     // Serial.println(driver->get_fault_status_2_string());
        //
        //     step = i % 6;
        //     high_impedance_phase = (step+1) % 3;
        //     analogWrite(DRV8323_HI_A_PIN, six_step_commutation_states[step][0] * level);
        //     analogWrite(DRV8323_LO_A_PIN, six_step_commutation_states[step][1] * level);
        //     analogWrite(DRV8323_HI_B_PIN, six_step_commutation_states[step][2] * level);
        //     analogWrite(DRV8323_LO_B_PIN, six_step_commutation_states[step][3] * level);
        //     analogWrite(DRV8323_HI_C_PIN, six_step_commutation_states[step][4] * level);
        //     analogWrite(DRV8323_LO_C_PIN, six_step_commutation_states[step][5] * level);
        //     // vTaskDelay(on_time);
        //
        //     while (!trigger->zero_cross[high_impedance_phase]) {
        //
        //     }
        //
        //     // // vTaskDelay(on_time);
        //     // analogWrite(DRV8323_HI_A_PIN, 0);
        //     // analogWrite(DRV8323_HI_B_PIN, 0);
        //     // analogWrite(DRV8323_HI_C_PIN, 0);
        //     // vTaskDelay(off_time);
        //     //
        //     i++;
        // }



        analogWrite(DRV8323_HI_A_PIN, 0);
        analogWrite(DRV8323_HI_B_PIN, 0);
        analogWrite(DRV8323_HI_C_PIN, 0);
        analogWrite(DRV8323_LO_A_PIN, 0);
        analogWrite(DRV8323_LO_B_PIN, 0);
        analogWrite(DRV8323_LO_C_PIN, 0);



    }




}
