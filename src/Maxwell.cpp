//
// Created by robos on 11/09/2023.
//

#include "Maxwell.h"

#include <FreeRTOS/Source/include/FreeRTOS.h>

namespace Maxwell {
    #define MAX_LEVEL 80  // out of 255
    #define MAX_SPEED 2000 // electrical rads/s



    Maxwell::Maxwell()  {
        SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK

        driver = new DRV8323::DRV8323(
            DRV8323_CS_PIN,
            SPI_1,
            1000000,
            DRV8323_GATE_EN_PIN);

        trigger = new triggered{false, false, false};
        pid_controller = new PIDController(0.2, 2, 0,
            0,
            static_cast<float>(MAX_LEVEL),
            30);
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

    void all_off() {
        digitalWrite(DRV8323_HI_A_PIN, LOW);
        digitalWrite(DRV8323_HI_B_PIN, LOW);
        digitalWrite(DRV8323_HI_C_PIN, LOW);
        digitalWrite(DRV8323_LO_A_PIN, LOW);
        digitalWrite(DRV8323_LO_B_PIN, LOW);
        digitalWrite(DRV8323_LO_C_PIN, LOW);
    }

    void Maxwell::state_feedback() {
        String text = "";
        // text += static_cast<String>(hall_sensor->hall_code); text += "/";
        text += static_cast<String>(hall_sensor->rotor_sector); text += "/";
        text += static_cast<String>(hall_sensor->electrical_velocity); text += "/";
        current_sensors->read();
        text += static_cast<String>(current_sensors->get_current_a()); text += "/";
        text += static_cast<String>(current_sensors->get_current_b()); text += "/";
        text += static_cast<String>(current_sensors->get_current_c()); text += "/";
        text += static_cast<String>(current_sensors->get_total_current()); text += "/";
        text += static_cast<String>(pwm_input->read_percentage()); text += "/";
        text += static_cast<String>(0.0); text += "/";
        // text += static_cast<String>(pid_controller->_output); text += "/";

        text += static_cast<String>(driver->get_fault_status_1_string()); text += "/";
        text += static_cast<String>(driver->get_fault_status_2_string()); text += "/";

        // calculate checksum
        int checksum = 0;
        for (int i = 0; i < text.length(); i++) {
            checksum += text[i]; // add the ASCII value of each character
        }
        checksum = checksum % 256;
        text += static_cast<String>(checksum);
        Serial.println(text);
    }


    void Maxwell::drive_hall_velocity() { // velocity is in electrical rads/s, duration is in ms
        uint8_t six_step_commutation_states_ccw[6][6] = {
            {0, 0, 1, 0, 0, 1}, // 0 - B->C // 110
            {1, 0, 0, 0, 0, 1}, // 1 - A->C // 100
            {1, 0, 0, 1, 0, 0}, // 2 - A->B // 101
            {0, 0, 0, 1, 1, 0}, // 3 - C->B // 001
            {0, 1, 0, 0, 1, 0}, // 4 - C->A // 011
            {0, 1, 1, 0, 0, 0}  // 5 - B->A // 010

        };

        // Running in 6x PWM mode
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_6x);

        // Enable the driver
        driver->enable(true);
        digitalWrite(DRV8323_BRAKE_PIN, HIGH); // set the brake pin to high - disables brake.
        digitalWrite(DRV8323_DIR_PIN, HIGH);    // set the direction pin to high

        int dir = MOTOR_DIRECTION::CW;
        // // ALIGN
        // analogWrite(DRV8323_HI_A_PIN, 100);
        // digitalWrite(DRV8323_LO_B_PIN, HIGH);
        // digitalWrite(DRV8323_LO_C_PIN, HIGH);
        // delay(100);

        all_off();



        int level = 0;
        uint8_t step = 0;
        uint32_t start = millis();
        bool aligned = false;
        int8_t old_rotor_sector = hall_sensor->rotor_sector;
        current_sensors->calibrate_offsets();

        while (true) {
            if (pwm_input->read_percentage() > 5) {
                uint32_t velocity = map(pwm_input->read_percentage(), 0, 100, 0, MAX_SPEED);
                pid_controller->set_setpoint(static_cast<float>(velocity));



                // if (not aligned) {
                //     // ALIGN
                //     analogWrite(DRV8323_HI_A_PIN, 100);
                //     digitalWrite(DRV8323_LO_B_PIN, HIGH);
                //     digitalWrite(DRV8323_LO_C_PIN, HIGH);
                //     delay(100);
                //     Serial.println("Aligned");
                //     aligned = true;
                // }

                float speed = hall_sensor->electrical_velocity;
                float output = pid_controller->update(speed);
                level = constrain(output, 0, MAX_LEVEL);
                // Match the state of the hall sensors to the commutation states
                // rotor_sector goes from 0-5
                step = dir == MOTOR_DIRECTION::CW ? (hall_sensor->rotor_sector + 3) % 6 : hall_sensor->rotor_sector;
                analogWrite(DRV8323_HI_A_PIN, six_step_commutation_states_ccw[step][0] * level);
                digitalWrite(DRV8323_LO_A_PIN, six_step_commutation_states_ccw[step][1]);
                analogWrite(DRV8323_HI_B_PIN, six_step_commutation_states_ccw[step][2] * level);
                digitalWrite(DRV8323_LO_B_PIN, six_step_commutation_states_ccw[step][3]);
                analogWrite(DRV8323_HI_C_PIN, six_step_commutation_states_ccw[step][4] * level);
                digitalWrite(DRV8323_LO_C_PIN, six_step_commutation_states_ccw[step][5]);

                // if (hall_sensor->rotor_sector != old_rotor_sector) {
                //     pid_controller->print_state();
                // }
                old_rotor_sector = hall_sensor->rotor_sector;
            }
            else {
                aligned = false;
                all_off();
                pid_controller->set_setpoint(0);
                // current_sensors->calibrate_offsets();
            }
            current_sensors->read();
            // Serial.print(current_sensors->get_current_a()); Serial.print(", ");
            // Serial.print(current_sensors->get_current_b()); Serial.print(", ");
            // Serial.println(current_sensors->get_current_c());
            // float total_current = current_sensors->get_total_current();
            // Serial.println(total_current);
            state_feedback();
        }

        // STOP
        digitalWrite(DRV8323_HI_A_PIN, 0);
        digitalWrite(DRV8323_LO_A_PIN, 0);
        digitalWrite(DRV8323_HI_B_PIN, 0);
        digitalWrite(DRV8323_LO_B_PIN, 0);
        digitalWrite(DRV8323_HI_C_PIN, 0);
        digitalWrite(DRV8323_LO_C_PIN, 0);

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
