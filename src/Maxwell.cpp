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

        // Running in 6x PWM mode
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_6x);

        // Enable the driver
        driver->enable(true);
        digitalWrite(DRV8323_BRAKE_PIN, HIGH); // set the brake pin to high - disables brake.
        digitalWrite(DRV8323_DIR_PIN, HIGH);    // set the direction pin to high

        // double align
        analogWrite(DRV8323_HI_B_PIN, 80);
        analogWrite(DRV8323_LO_A_PIN, 80);
        delay(100);

        // analogWrite(DRV8323_HI_A_PIN, 100);
        // analogWrite(DRV8323_LO_C_PIN, 100);
        // vTaskDelay(100);

        int level = 50;
        uint8_t step = 0;
        uint32_t start = millis();
        int i = 0;
        for (; millis() - start < duration;) {
            // Match the state of the hall sensors to the commutation states
            step = (hall_sensor->rotor_sector + 2) % 6;
            analogWrite(DRV8323_HI_A_PIN, six_step_commutation_states[step][0] * level);
            analogWrite(DRV8323_LO_A_PIN, six_step_commutation_states[step][1] * level);
            analogWrite(DRV8323_HI_B_PIN, six_step_commutation_states[step][2] * level);
            analogWrite(DRV8323_LO_B_PIN, six_step_commutation_states[step][3] * level);
            analogWrite(DRV8323_HI_C_PIN, six_step_commutation_states[step][4] * level);
            analogWrite(DRV8323_LO_C_PIN, six_step_commutation_states[step][5] * level);

            delay(10);
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
