//
// Created by robos on 11/09/2023.
//

#include "Maxwell.h"

#include <FreeRTOS/Source/include/FreeRTOS.h>

namespace Maxwell {
    #define MAX_LEVEL 80  // out of 255
    #define MAX_SPEED 2000 // electrical rads/s
    Maxwell *Maxwell::instance = nullptr;




    Maxwell::Maxwell()  {
        SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK
        SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK
        driver = new DRV8323::DRV8323(
            DRV8323_CS_PIN,
            SPI_1,
            1000000,
            DRV8323_HI_A_PIN,
            DRV8323_HI_B_PIN,
            DRV8323_HI_C_PIN,
            DRV8323_GATE_EN_PIN);
        encoder = new AS5047P::AS5047P(
            AS5047P_CS_PIN,
            SPI_2,
            1000000);

        trigger = new triggered{false, false, false};
        pid_controller = new PIDController(0.2, 2, 0,
            0,
            static_cast<float>(MAX_LEVEL),
            30);

        foc = new FOC{
            new PIDController(0.1, 2, 0,
            0.0,
            5.0,
            2.0),

            new PIDController(0.1, 2, 0,
            0.0,
            5.0,
            2.0),

            new PIDController(20, 0.1, 0,
            0.0,
            5.0,
            2.0)
        };
        curr_struct = new Currents{0, 0, 0, 0, 0, 0, 0};

        pwm_3x = new pwm_3x_struct();
        pwm_3x->PIN_A_STATE = 0; pwm_3x->PIN_B_STATE = 0; pwm_3x->PIN_C_STATE = 0;

        instance = this;
    }

    volatile void Maxwell::Compare_A_callback() {
        instance->pwm_3x->PIN_A_STATE = (instance->pwm_3x->PIN_A_STATE)? 0 : 1;
        digitalWriteFast(DRV8323_HI_A_PIN, instance->pwm_3x->PIN_A_STATE);
        digitalWriteFast(DRV8323_LO_A_PIN, not (instance->pwm_3x->PIN_A_STATE));
    }

    volatile void Maxwell::Compare_B_callback() {
        instance->pwm_3x->PIN_B_STATE = (instance->pwm_3x->PIN_B_STATE)? 0 : 1;
        digitalWriteFast(DRV8323_HI_B_PIN, instance->pwm_3x->PIN_B_STATE);
        digitalWriteFast(DRV8323_LO_B_PIN, not (instance->pwm_3x->PIN_B_STATE));
    }

    volatile void Maxwell::Compare_C_callback() {
        instance->pwm_3x->PIN_C_STATE = (instance->pwm_3x->PIN_C_STATE) ? 0 : 1;
        // digitalWriteFast(PC_13, instance->pwm_3x->PIN_C);
        digitalWriteFast(DRV8323_HI_C_PIN, instance->pwm_3x->PIN_C_STATE);
        digitalWriteFast(DRV8323_LO_C_PIN, not (instance->pwm_3x->PIN_C_STATE));
    }

    volatile void Maxwell::Update_A_callback() {
        instance->pwm_3x->PIN_A_STATE = 0;
        // digitalWriteFast(DRV8323_HI_A_PIN, instance->pwm_3x->PIN_A_STATE);
        // digitalWriteFast(DRV8323_LO_A_PIN, not (instance->pwm_3x->PIN_A_STATE));
    }

    volatile void Maxwell::Update_B_callback() {
        instance->pwm_3x->PIN_B_STATE = 0;
        // digitalWriteFast(DRV8323_HI_B_PIN, instance->pwm_3x->PIN_B_STATE);
        // digitalWriteFast(DRV8323_LO_B_PIN, not (instance->pwm_3x->PIN_B_STATE));
    }

    volatile void Maxwell::Update_C_callback() {
        instance->pwm_3x->PIN_C_STATE = 0;
        // digitalWriteFast(DRV8323_HI_C_PIN, instance->pwm_3x->PIN_C_STATE);
        // digitalWriteFast(DRV8323_LO_C_PIN, not (instance->pwm_3x->PIN_C_STATE));
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

        driver->default_configuration();
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_6x);
        driver->set_gate_drive_source_current(DRV8323::IDRIVE_P_CURRENT::IDRIVEP_1000mA);
        driver->set_gate_drive_sink_current(DRV8323::IDRIVE_N_CURRENT::IDRIVEN_2000mA);
        driver->set_peak_gate_drive_time(DRV8323::TDRIVE_TIME::TDRIVE_1000ns);
        driver->enable(true);

        // Tie the low side pins HIGH to avoid hi-z state - Only in PWM_3X mode
        // digitalWrite(DRV8323_LO_A_PIN, HIGH);
        // digitalWrite(DRV8323_LO_B_PIN, HIGH);
        // digitalWrite(DRV8323_LO_C_PIN, HIGH);


        pinMode(DRV8323_GATE_EN_PIN, OUTPUT);
        pinMode(DRV8323_DRIVE_CAL_PIN, OUTPUT);

        pinMode(V_SENSE_A_PIN, INPUT);
        pinMode(V_SENSE_B_PIN, INPUT);
        pinMode(V_SENSE_C_PIN, INPUT);
        pinMode(V_SUPPLY_SENSE_PIN, INPUT);

        pinMode(DRV8323_CURR_SENSE_A_PIN, INPUT);
        pinMode(DRV8323_CURR_SENSE_B_PIN, INPUT);
        pinMode(DRV8323_CURR_SENSE_C_PIN, INPUT);



        Serial.println("Maxwell setup complete");
    }

    void Maxwell::init_pwm() {
        pwm_3x->RESOLUTION = 16;
        pwm_3x->MAX_COMPARE_VALUE = static_cast<uint32_t>(pow(2, pwm_3x->RESOLUTION)) - 1;
        pwm_3x->FREQ = 1000 * 2;

        uint8_t pin_a = DRV8323_HI_A_PIN;
        uint8_t pin_b = DRV8323_HI_B_PIN;
        uint8_t pin_c = DRV8323_HI_C_PIN;
        uint32_t prescale_value = 4;
        // TIM_A = TIM1_CH3
        // TIM_B = TIM2_CH4
        // TIM_C = TIM5_CH2
        TIM_TypeDef *Instance_a = TIM4;
        pwm_3x->channel_a = 1; //STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_a), PinMap_PWM));
        pwm_3x->TIM_A = new HardwareTimer(Instance_a);
        pwm_3x->TIM_A->setPrescaleFactor(prescale_value);
        // pwm_3x->TIM_A->attachInterrupt(pwm_3x->channel_a, Compare_A_callback);

        TIM_TypeDef *Instance_b = TIM2;
        pwm_3x->channel_b = 4;  //STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_b), PinMap_PWM));
        pwm_3x->TIM_B = new HardwareTimer(Instance_b);
        pwm_3x->TIM_B->setPrescaleFactor(prescale_value);
        // pwm_3x->TIM_B->attachInterrupt(pwm_3x->channel_b, Compare_B_callback);

        TIM_TypeDef *Instance_c = TIM3;
        pwm_3x->channel_c = 2; //STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_c), PinMap_PWM));
        pwm_3x->TIM_C = new HardwareTimer(Instance_c);
        pwm_3x->TIM_C->setPrescaleFactor(prescale_value);
        // pwm_3x->TIM_C->attachInterrupt(pwm_3x->channel_c, Compare_C_callback);

        sync_pwm();

        uint32_t COUNTER_MODE = TIM_COUNTERMODE_CENTERALIGNED3;  // Counter Rise and then fall
        TimerModes_t PWM_MODE = TIMER_OUTPUT_DISABLED;
        pwm_3x->TIM_A->pause();
        pwm_3x->TIM_A->setMode(pwm_3x->channel_a, PWM_MODE, NC);
        pwm_3x->TIM_A->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_A->getHandle()->Init.RepetitionCounter = 1;
        pwm_3x->TIM_A->setPWM(pwm_3x->channel_a, PB6, pwm_3x->FREQ, 0);  // This clearly sets something up that I've forgot - No pwm output without it LOL
        pwm_3x->TIM_A->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_A->setCaptureCompare(pwm_3x->channel_a, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));

        pwm_3x->TIM_B->pause();
        pwm_3x->TIM_B->setMode(pwm_3x->channel_b, PWM_MODE, NC);
        pwm_3x->TIM_B->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_B->getHandle()->Init.RepetitionCounter = 1;
        pwm_3x->TIM_B->setPWM(pwm_3x->channel_b, PA3, pwm_3x->FREQ, 0);
        pwm_3x->TIM_B->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_B->setCaptureCompare(pwm_3x->channel_b, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));

        pwm_3x->TIM_C->pause();
        pwm_3x->TIM_C->setMode(pwm_3x->channel_c, PWM_MODE, NC);
        pwm_3x->TIM_C->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_C->getHandle()->Init.RepetitionCounter = 1;
        pwm_3x->TIM_C->setPWM(pwm_3x->channel_c, PB5, pwm_3x->FREQ, 0);
        pwm_3x->TIM_C->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_C->setCaptureCompare(pwm_3x->channel_c, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));

        // pwm_3x->TIM_A->attachInterrupt(Update_A_callback);
        // pwm_3x->TIM_B->attachInterrupt(Update_B_callback);
        // pwm_3x->TIM_C->attachInterrupt(Update_C_callback);


        pwm_3x->TIM_A->attachInterrupt(pwm_3x->channel_a, Compare_A_callback);
        pwm_3x->TIM_B->attachInterrupt(pwm_3x->channel_b, Compare_B_callback);
        pwm_3x->TIM_C->attachInterrupt(pwm_3x->channel_c, Compare_C_callback);

        HAL_TIM_Base_Init(pwm_3x->TIM_A->getHandle());;
        HAL_TIM_Base_Init(pwm_3x->TIM_B->getHandle());
        HAL_TIM_Base_Init(pwm_3x->TIM_C->getHandle());


    }

    void Maxwell::sync_pwm() {
        noInterrupts();
        pwm_3x->TIM_A->pause();
        pwm_3x->TIM_A->refresh();
        pwm_3x->TIM_B->pause();
        pwm_3x->TIM_B->refresh();
        pwm_3x->TIM_C->pause();
        pwm_3x->TIM_C->refresh();

        pwm_3x->TIM_A->resume();
        pwm_3x->TIM_B->resume();
        pwm_3x->TIM_C->resume();
        interrupts();
    }

    void Maxwell::set_pwm(uint32_t Ua, uint32_t Ub, uint32_t Uc, uint32_t resolution) {
        constrain(Ua, 0.0, pwm_3x->MAX_COMPARE_VALUE);
        constrain(Ub, 0.0, pwm_3x->MAX_COMPARE_VALUE);
        constrain(Uc, 0.0, pwm_3x->MAX_COMPARE_VALUE);
        (Ua < 2.0)? Ua = 0 : Ua;
        (Ub < 2.0)? Ub = 0 : Ub;
        (Uc < 2.0)? Uc = 0 : Uc;
        pwm_3x->TIM_A->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_B->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_C->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_A->setCaptureCompare(pwm_3x->channel_a, Ua, static_cast<TimerCompareFormat_t>(resolution));
        pwm_3x->TIM_B->setCaptureCompare(pwm_3x->channel_b, Ub, static_cast<TimerCompareFormat_t>(resolution));
        pwm_3x->TIM_C->setCaptureCompare(pwm_3x->channel_c, Uc, static_cast<TimerCompareFormat_t>(resolution));
    }

    void Maxwell::set_phase_voltages(float Va, float Vb, float Vc) {
        float input_voltage = 12;
        float offset = 0.1; // V
        Va = constrain(Va, -max_voltage/2, max_voltage/2) + max_voltage / 2 + offset;
        Vb = constrain(Vb, -max_voltage/2, max_voltage/2) + max_voltage / 2 + offset;
        Vc = constrain(Vc, -max_voltage/2, max_voltage/2) + max_voltage / 2 + offset;

        Va = constrain(Va, 0, max_voltage) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);
        Vb = constrain(Vb, 0, max_voltage) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);
        Vc = constrain(Vc, 0, max_voltage) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);

        Serial.print(Va); Serial.print(" "); Serial.print(Vb); Serial.print(" "); Serial.println(Vc);

        set_pwm(static_cast<uint32_t>(Va),
            static_cast<uint32_t>(Vb),
            static_cast<uint32_t>(Vc),
            pwm_3x->RESOLUTION);
    }

    void Maxwell::all_off() {
        set_pwm(0, 0, 0, pwm_3x->RESOLUTION);
    }

    void Maxwell::state_feedback() {
        String text = "";
        // text += static_cast<String>(hall_sensor->hall_code); text += "/";
        text += static_cast<String>(fmod(encoder->get_angle() * POLE_PAIRS_6374, 2*PI)); text += "/";
        // text += static_cast<String>(hall_sensor->electrical_velocity);
        // text += "/";
        driver->current_sensors->read();
        text += static_cast<String>(driver->current_sensors->get_current_a()); text += "/";
        text += static_cast<String>(driver->current_sensors->get_current_b()); text += "/";
        text += static_cast<String>(driver->current_sensors->get_current_c()); text += "/";
        text += static_cast<String>(curr_struct->dq.d); text += "/";
        text += static_cast<String>(curr_struct->dq.q); text += "/";
        text += static_cast<String>(curr_struct->alpha_beta.alpha); text += "/";
        text += static_cast<String>(curr_struct->alpha_beta.beta); text += "/";
        text += static_cast<String>(pwm_input->read_percentage()); text += "/";
        double voltages[4] = {analogRead(V_SENSE_A_PIN) * SENSE_CONVERSION_FACTOR,
                             analogRead(V_SENSE_B_PIN) * SENSE_CONVERSION_FACTOR,
                             analogRead(V_SENSE_C_PIN) * SENSE_CONVERSION_FACTOR,
                                analogRead(V_SUPPLY_SENSE_PIN) * SENSE_CONVERSION_FACTOR};
        text += static_cast<String>(voltages[0]); text += "/";
        text += static_cast<String>(voltages[1]); text += "/";
        text += static_cast<String>(voltages[2]); text += "/";
        text += static_cast<String>(voltages[3]); text += "/";

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

    void actuate_currents(PhaseCurrents command_currents, String &text) {
        // 3X pwm mode

        double current_a = command_currents.current_a;
        double current_b = command_currents.current_b;
        double current_c = command_currents.current_c;

        double input_voltage = 12.0; // 12V
        double voltage_limit = 3; // V
        voltage_limit = constrain(voltage_limit, 0, input_voltage);
        float centre = voltage_limit / 2;
        double current_limit = 4.0; // 4A
        double phase_resistance = 0.15; //
        int max_level = (int)voltage_limit / input_voltage * 255;
        // int max_level = 15;

        float voltage_a = (current_a * phase_resistance);
        float voltage_b = (current_b * phase_resistance);
        float voltage_c = (current_c * phase_resistance);

        text += "      ";
        text += static_cast<String>(voltage_a); text += "/";
        text += static_cast<String>(voltage_b); text += "/";
        text += static_cast<String>(voltage_c); text += "/";

        int level_a = constrain(level_a, 0, max_level);  // Cannot actuate a negative current
        int level_b = constrain(level_b, 0, max_level);
        int level_c = constrain(level_c, 0, max_level);


        // running in 3x PWM mode
        analogWrite(DRV8323_HI_A_PIN, level_a);
        analogWrite(DRV8323_HI_B_PIN, level_b);
        analogWrite(DRV8323_HI_C_PIN, level_c);
        text += "      ";
        text += static_cast<String>(level_a); text += "/";
        text += static_cast<String>(level_b); text += "/";
        text += static_cast<String>(level_c); text += "/";




        /*
        if (current_a > 0) {
            analogWrite(DRV8323_HI_A_PIN, level_a);
            digitalWrite(DRV8323_LO_A_PIN, LOW);
        }
        else {
            analogWrite(DRV8323_HI_A_PIN, LOW);
            digitalWrite(DRV8323_LO_A_PIN, level_a);
        }
        if (current_b > 0) {
            analogWrite(DRV8323_HI_B_PIN, level_b);
            digitalWrite(DRV8323_LO_B_PIN, LOW);
        }
        else {
            analogWrite(DRV8323_HI_B_PIN, LOW);
            digitalWrite(DRV8323_LO_B_PIN, level_b);
        }
        if (current_c > 0) {
            analogWrite(DRV8323_HI_C_PIN, level_c);
            digitalWrite(DRV8323_LO_C_PIN, LOW);
        }
        else {
            analogWrite(DRV8323_HI_C_PIN, LOW);
            digitalWrite(DRV8323_LO_C_PIN, level_c);
        } */
    }

    void Maxwell::foc_position_control() {
        // Running in 6x PWM mode
        // driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_6x);

        // Enable the driver
        driver->enable(true);

        int dir = MOTOR_DIRECTION::CW;
        // // ALIGN
        analogWrite(DRV8323_HI_A_PIN, 50);
        digitalWrite(DRV8323_HI_B_PIN, 0);
        digitalWrite(DRV8323_HI_C_PIN, 0);
        delay(100);

        all_off();

        foc->position_pid->set_setpoint(180);
        foc->d_pid->set_setpoint(0);
        foc->q_pid->set_setpoint(1);
        while (true) {
            float theta = encoder->get_angle();
            // float q_set_point = foc->position_pid->update(theta);
            // // foc->position_pid->print_state();
            // foc->q_pid->set_setpoint(q_set_point);
            foc->q_pid->print_state();


            // Get motor currents
            driver->current_sensors->read();
            PhaseCurrents phase_currents = {driver->current_sensors->get_current_a(),
                                            driver->current_sensors->get_current_b(),
                                            driver->current_sensors->get_current_c()};

            String text = "";
            text += static_cast<String>(phase_currents.current_a); text += "/";
            text += static_cast<String>(phase_currents.current_b); text += "/";
            text += static_cast<String>(phase_currents.current_c); text += "/";
            text += "    ";

            alpha_beta_struct ab_vec = clarke_transform(phase_currents);
            text += static_cast<String>(ab_vec.alpha); text += "/";
            text += static_cast<String>(ab_vec.beta); text += "/";
            text += "    ";
            dq_struct dq_vec = park_transform(ab_vec);
            text += static_cast<String>(dq_vec.d); text += "/";
            text += static_cast<String>(dq_vec.q); text += "/";


            float command_d = foc->d_pid->update(dq_vec.d);
            float command_q = foc->q_pid->update(dq_vec.q);
            // foc->d_pid->print_state();
            // foc->q_pid->print_state();
            dq_struct command_dq = {command_d, command_q};
            text += static_cast<String>(command_d); text += "/";
            text += static_cast<String>(command_q); text += "/";
            text += "    ";

            alpha_beta_struct command_alpha_beta = reverse_park_transform(command_dq);
            text += static_cast<String>(command_alpha_beta.alpha); text += "/";
            text += static_cast<String>(command_alpha_beta.beta); text += "/";
            text += "    ";

            PhaseCurrents command_currents = reverse_clarke_transform(command_alpha_beta);
            text += static_cast<String>(command_currents.current_a); text += "/";
            text += static_cast<String>(command_currents.current_b); text += "/";
            text += static_cast<String>(command_currents.current_c); text += "/";

            actuate_currents(command_currents, text);
            Serial.println(text);
            // state_feedback();

            // delay(1);
        }

        all_off();
    }

    void Maxwell::svpwm_position_control() {
        driver->enable(true);

        // digitalWrite(DRV8323_LO_A_PIN, HIGH);
        // digitalWrite(DRV8323_LO_B_PIN, HIGH);
        // digitalWrite(DRV8323_LO_C_PIN, HIGH);

        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_6x);

        while (true) {
            // pwm_input->read();
            float theta = static_cast<float>(pwm_input->read_percentage()) / 100 * 2 * PI * 4;
            // Serial.println(theta);

            // Generate three sin waves, offset by 120 degrees
            float U_a = sin(theta)          * max_voltage/2;
            float U_b = sin(theta - 2*PI/3) * max_voltage/2;
            float U_c = sin(theta + 2*PI/3) * max_voltage/2;

            set_phase_voltages(U_a, U_b, U_c);
            Serial.println(driver->get_fault_status_1_string());
            Serial.println(driver->get_fault_status_2_string());
            driver->clear_fault();
        }

        // set_pwm(127, 20, 127, pwm_3x->RESOLUTION);



        // Debugging output
        //Get the PWM output value:
        // uint32_t a = pwm_3x->TIM_A->getHandle()->Instance->CCR3;
        // uint32_t b = pwm_3x->TIM_B->getHandle()->Instance->CCR4;
        // uint32_t c = pwm_3x->TIM_C->getHandle()->Instance->CCR2;
        //
        // uint32_t cnt_a = pwm_3x->TIM_A->getHandle()->Instance->CNT;
        //
        //
        // Serial.print(instance->pwm_3x->PIN_A + 1); Serial.print(" ");
        // Serial.print(instance->pwm_3x->PIN_B + 2); Serial.print(" ");
        // Serial.print(instance->pwm_3x->PIN_C + 3); Serial.print(" ");
        // // Serial.print(out_b); Serial.print(" ");
        // // Serial.print(out_c); Serial.print(" ");
        // Serial.print(static_cast<float>(cnt_a)); Serial.print(" ");

        // Serial.println();
        // delay(10);
    }

    alpha_beta_struct Maxwell::clarke_transform(PhaseCurrents currents) { // currents to alpha-beta
        float I_alpha = currents.current_a;
        float I_beta = (currents.current_b - currents.current_c) / sqrt(3);
        alpha_beta_struct ab = {I_alpha, I_beta};
        curr_struct->alpha_beta = ab;
        return ab;
    }

    dq_struct Maxwell::park_transform(alpha_beta_struct ab_vec) { // alpha-beta to dq
        // the park transform

        float theta = encoder->get_angle(); // Assuming we're aligned with the encoder!
        float electrical_theta = fmod(theta * POLE_PAIRS_6374, 2*PI);
        float d = ab_vec.alpha * cos(electrical_theta)  + ab_vec.beta * sin(electrical_theta);
        float q = -ab_vec.alpha * sin(electrical_theta) + ab_vec.beta * cos(electrical_theta);
        dq_struct dq = {d, q};
        curr_struct->dq = dq;
        return dq;
    }

    alpha_beta_struct Maxwell::reverse_park_transform(dq_struct dq_vec) {  // dq to alpha-beta
        float theta = encoder->get_angle(); // Assuming we're aligned with the encoder!
        float electrical_theta = fmod(theta * POLE_PAIRS_6374, 2*PI);
        float alpha = dq_vec.d * cos(electrical_theta) - dq_vec.q * sin(electrical_theta);
        float beta  = dq_vec.d * sin(electrical_theta) + dq_vec.q * cos(electrical_theta);
        alpha_beta_struct alpha_beta = {alpha, beta};
        curr_struct->alpha_beta = alpha_beta;
        return alpha_beta;
    }

    PhaseCurrents Maxwell::reverse_clarke_transform(alpha_beta_struct ab_vec) {
        // PhaseCurrents currents = {(2/3)*ab_vec.alpha,
        //                     (-1/3)*ab_vec.alpha + (sqrt(3)/3)*ab_vec.beta,
        //                     (-1/3)*ab_vec.alpha - (sqrt(3)/3)*ab_vec.beta};
        PhaseCurrents currents;
        currents.current_a = 0.3333*ab_vec.alpha;
        currents.current_b = (-1/3)*ab_vec.alpha + (sqrt(3)/3)*ab_vec.beta;
        currents.current_c = (-1/3)*ab_vec.alpha - (sqrt(3)/3)*ab_vec.beta;
        curr_struct->phase_currents = currents;
        return currents;
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
        driver->current_sensors->calibrate_offsets();

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
