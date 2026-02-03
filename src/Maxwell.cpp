//
// Created by robos on 11/09/2023.
//

#include "Maxwell.h"

#include <ratio>
#include <FreeRTOS/Source/include/FreeRTOS.h>

#include "FreeRTOS/Source/include/FreeRTOS.h"

namespace Maxwell {
    #define MAX_LEVEL 80  // out of 255
    #define MAX_SPEED 2000 // electrical rads/s
    Maxwell *Maxwell::instance = nullptr;

    Maxwell::Maxwell()  {
        telemetry = new USBTarget();
        telemetry->initialize();

        SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK
        SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK
        driver = new DRV8323::DRV8323(
            DRV8323_CS_PIN,
            SPI_1,
            1000000, // 1 MHz SPI frequency - This is the max frequency for HARDWARE_V2_0 due to parasitics on the SDO line.
            DRV8323_HI_A_PIN,
            DRV8323_HI_B_PIN,
            DRV8323_HI_C_PIN,
            DRV8323_GATE_EN_PIN);
        // encoder = new AS5048A ( // Internal encoder
        //     AS5047P_CS_PIN,
        //     SPI_1,
        //     1000000); // 1 MHz SPI frequency
        encoder = new AS5048A ( // External encoder
            EXTERNAL_ENCODER_CS_PIN,
            SPI_2,
            10000); // 1 MHz SPI frequency

        pwm_input = new PWMInput(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);

        trigger = new triggered{false, false, false};
        pid_controller = new PIDController(0.2, 2, 0,
            0,
            static_cast<float>(MAX_LEVEL),
            30);
        current_sensors = new CurrentSensors(CURR_SENSE_A_PIN,
                                            CURR_SENSE_B_PIN,
                                            CURR_SENSE_C_PIN,
                                            _csa_gain);

        pwm_3x = new pwm_3x_struct();
        pwm_3x->PIN_A_STATE = 0; pwm_3x->PIN_B_STATE = 0; pwm_3x->PIN_C_STATE = 0;
        encoder->_direction = SENSOR_DIRECTION::CW; // Confirmed via testing

        instance = this;
    }

    void Maxwell::setup() {
        // pinMode(HALL_A_PIN, INPUT);
        // pinMode(HALL_B_PIN, INPUT);
        // pinMode(HALL_C_PIN, INPUT);

        pinMode(GREEN_LED_PIN, OUTPUT);
        // Breakout to a driver class?
        pinMode(DRV8323_HI_A_PIN, OUTPUT);
        pinMode(DRV8323_HI_B_PIN, OUTPUT);
        pinMode(DRV8323_HI_C_PIN, OUTPUT);
        pinMode(DRV8323_LO_A_PIN, OUTPUT);
        pinMode(DRV8323_LO_B_PIN, OUTPUT);
        pinMode(DRV8323_LO_C_PIN, OUTPUT);

        driver->enable(true);
        // driver->default_configuration();
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);
        // driver->set_gate_drive_source_current(DRV8323::IDRIVE_P_CURRENT::IDRIVEP_1000mA);
        // driver->set_gate_drive_sink_current(DRV8323::IDRIVE_N_CURRENT::IDRIVEN_2000mA);
        // driver->set_peak_gate_drive_time(DRV8323::TDRIVE_TIME::TDRIVE_4000ns);

        driver->set_dead_time(DRV8323::DEAD_TIMES::DEAD_TIME_400NS);

        // Tie the low side pins HIGH to avoid hi-z state - Only in PWM_3X mode
        digitalWriteFast(DRV8323_LO_A_PIN, 1);
        digitalWriteFast(DRV8323_LO_B_PIN, 1);
        digitalWriteFast(DRV8323_LO_C_PIN, 1);


        pinMode(DRV8323_GATE_EN_PIN, OUTPUT);

        pinMode(V_SENSE_A_PIN, INPUT);
        pinMode(V_SENSE_B_PIN, INPUT);
        pinMode(V_SENSE_C_PIN, INPUT);
        pinMode(V_SUPPLY_SENSE_PIN, INPUT);

        // __HAL_RCC_ADC1_CLK_ENABLE();
        // __HAL_RCC_GPIOB_CLK_ENABLE();
        // __HAL_RCC_GPIOC_CLK_ENABLE();
        // __HAL_RCC_GPIOA_CLK_ENABLE();

        // pinMode(DRV8323_CURR_SENSE_A_PIN, INPUT); // PB_0 - ADC1_IN8
        // pinMode(DRV8323_CURR_SENSE_B_PIN, INPUT); // PC_5 - ADC1_IN15
        // pinMode(DRV8323_CURR_SENSE_C_PIN, INPUT); // PC_4 - ADC1_IN14

        Serial.println("Maxwell setup complete");
    }

    void Maxwell::init_pwm_3x() {
        pwm_3x->RESOLUTION = 16;
        pwm_3x->MAX_COMPARE_VALUE = static_cast<uint32_t>(pow(2, pwm_3x->RESOLUTION)) - 1;
        pwm_3x->FREQ = pwm_frequency * 2;

        // Set APB1 prescaler
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
        // Set APB2 prescaler
        RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;

        PinName pin_a = DRV8323_HI_A_PIN;
        PinName pin_b = DRV8323_HI_B_PIN;
        PinName pin_c = DRV8323_HI_C_PIN;
        uint32_t prescale_value = 4;
        // TIM_A = TIM1_CH1
        // TIM_B = TIM2_CH4
        // TIM_C = TIM5_CH2
        TIM_TypeDef *Instance_a = TIM1;// (TIM_TypeDef *)pinmap_peripheral(pin_a, PinMap_PWM);
        pwm_3x->channel_a = STM_PIN_CHANNEL(pinmap_function(pin_a, PinMap_PWM));
        pwm_3x->TIM_A = new HardwareTimer(Instance_a);

        TIM_TypeDef *Instance_b = TIM1; //(TIM_TypeDef *)pinmap_peripheral(pin_b, PinMap_PWM);
        pwm_3x->channel_b = STM_PIN_CHANNEL(pinmap_function(pin_b, PinMap_PWM));
        pwm_3x->TIM_B = new HardwareTimer(Instance_b);

        TIM_TypeDef *Instance_c = TIM1;//(TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_c), PinMap_PWM);
        pwm_3x->channel_c = STM_PIN_CHANNEL(pinmap_function(pin_c, PinMap_PWM));
        pwm_3x->TIM_C = new HardwareTimer(Instance_c);


        sync_pwm();

        uint32_t COUNTER_MODE = TIM_COUNTERMODE_CENTERALIGNED3;  // Counter Rise and then fall
        TimerModes_t PWM_MODE = TIMER_OUTPUT_COMPARE_PWM1;

        pwm_3x->TIM_A->setMode(pwm_3x->channel_a, PWM_MODE, pin_a);
        pwm_3x->TIM_A->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_A->getHandle()->Init.RepetitionCounter = 1;
        pwm_3x->TIM_A->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_A->setCaptureCompare(pwm_3x->channel_a, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));
        // pwm_3x->TIM_A->attachInterrupt(Update_A_callback);

        pwm_3x->TIM_B->setMode(pwm_3x->channel_b, PWM_MODE, pin_b);
        pwm_3x->TIM_B->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_B->getHandle()->Init.RepetitionCounter = 1;
        pwm_3x->TIM_B->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_B->setCaptureCompare(pwm_3x->channel_b, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));

        pwm_3x->TIM_C->setMode(pwm_3x->channel_c, PWM_MODE, pin_c);
        pwm_3x->TIM_C->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_C->getHandle()->Init.RepetitionCounter = 1;
        pwm_3x->TIM_C->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_C->setCaptureCompare(pwm_3x->channel_c, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));

        HAL_TIM_Base_Init(pwm_3x->TIM_A->getHandle());;
        HAL_TIM_Base_Init(pwm_3x->TIM_B->getHandle());
        HAL_TIM_Base_Init(pwm_3x->TIM_C->getHandle());

        // Syncronise the timers with a master timer.
        // LL_TIM_SetSlaveMode(pwm_3x->TIM_A->getHandle()->Instance, LL_TIM_SLAVEMODE_DISABLED);
        // LL_TIM_SetTriggerOutput(pwm_3x->TIM_A->getHandle()->Instance, LL_TIM_TRGO_ENABLE);
        // LL_TIM_SetTriggerOutput(pwm_3x->TIM_A->getHandle()->Instance, LL_TIM_TRGO_UPDATE);


        // Configure the other two timers to get their input trigger from the master timer:
        // for (auto timer : {pwm_3x->TIM_B, pwm_3x->TIM_C}) {
        //     LL_TIM_SetTriggerInput(timer->getHandle()->Instance, _getInternalSourceTrigger(pwm_3x->TIM_A, timer));
        //     LL_TIM_SetSlaveMode(timer->getHandle()->Instance, LL_TIM_SLAVEMODE_TRIGGER);
        // }

        // LL_TIM_SetTriggerOutput(pwm_3x->TIM_B->getHandle()->Instance, LL_TIM_TRGO_UPDATE);

        sync_pwm();
    }

    void Maxwell::init_pwm_6x() {
        pwm_3x->RESOLUTION = 16;
        pwm_3x->MAX_COMPARE_VALUE = static_cast<uint32_t>(pow(2, pwm_3x->RESOLUTION)) - 1;
        pwm_3x->FREQ = pwm_frequency * 2;

        // Set APB1 prescaler
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
        // Set APB2 prescaler
        RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;

        PinName pin_a = DRV8323_LO_A_PIN; //DRV8323_HI_A_PIN;
        PinName pin_b = DRV8323_LO_B_PIN; //DRV8323_HI_B_PIN;
        PinName pin_c = DRV8323_LO_C_PIN; //DRV8323_HI_C_PIN;
        uint32_t prescale_value = 4;
        // TIM_A = TIM1_CH3
        // TIM_B = TIM2_CH4
        // TIM_C = TIM5_CH2
        TIM_TypeDef *Instance_a = TIM1; //(TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_a), PinMap_PWM); //TIM1;
        pwm_3x->channel_a = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_a), PinMap_PWM));
        pwm_3x->TIM_A = new HardwareTimer(Instance_a);
        // pwm_3x->TIM_A->setPrescaleFactor(prescale_value);

        TIM_TypeDef *Instance_b = TIM2;
        pwm_3x->channel_b = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_b), PinMap_PWM));
        pwm_3x->TIM_B = new HardwareTimer(Instance_b);
        // pwm_3x->TIM_B->setPrescaleFactor(prescale_value);

        TIM_TypeDef *Instance_c = TIM4;
        pwm_3x->channel_c = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_c), PinMap_PWM));
        pwm_3x->TIM_C = new HardwareTimer(Instance_c);
        // pwm_3x->TIM_C->setPrescaleFactor(prescale_value);


        sync_pwm();

        uint32_t COUNTER_MODE = TIM_COUNTERMODE_CENTERALIGNED3;  // Counter Rise and then fall
        TimerModes_t PWM_MODE = TIMER_OUTPUT_COMPARE_PWM2;
        // pwm_3x->TIM_A->pause();
        pwm_3x->TIM_A->setMode(pwm_3x->channel_a, PWM_MODE, pin_a);
        pwm_3x->TIM_A->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_A->getHandle()->Init.RepetitionCounter = 1;
        // pwm_3x->TIM_A->setPWM(pwm_3x->channel_a, PB6, pwm_3x->FREQ, 0);  // This clearly sets something up that I've forgot - No pwm output without it LOL
        pwm_3x->TIM_A->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_A->setCaptureCompare(pwm_3x->channel_a, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));

        // pwm_3x->TIM_B->pause();
        pwm_3x->TIM_B->setMode(pwm_3x->channel_b, PWM_MODE, pin_b);
        pwm_3x->TIM_B->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_B->getHandle()->Init.RepetitionCounter = 1;
        // pwm_3x->TIM_B->setPWM(pwm_3x->channel_b, PA3, pwm_3x->FREQ, 0);
        pwm_3x->TIM_B->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_B->setCaptureCompare(pwm_3x->channel_b, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));

        // pwm_3x->TIM_C->pause();
        pwm_3x->TIM_C->setMode(pwm_3x->channel_c, PWM_MODE, pin_c);
        pwm_3x->TIM_C->getHandle()->Init.CounterMode = COUNTER_MODE;
        pwm_3x->TIM_C->getHandle()->Init.RepetitionCounter = 1;
        // pwm_3x->TIM_C->setPWM(pwm_3x->channel_c, PB5, pwm_3x->FREQ, 0);
        pwm_3x->TIM_C->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_C->setCaptureCompare(pwm_3x->channel_c, 0, static_cast<TimerCompareFormat_t>(pwm_3x->RESOLUTION));

        // pwm_3x->TIM_A->attachInterrupt(Update_A_callback);
        // pwm_3x->TIM_B->attachInterrupt(Update_B_callback);
        // pwm_3x->TIM_C->attachInterrupt(Update_C_callback);


        HAL_TIM_Base_Init(pwm_3x->TIM_A->getHandle());;
        HAL_TIM_Base_Init(pwm_3x->TIM_B->getHandle());
        HAL_TIM_Base_Init(pwm_3x->TIM_C->getHandle());


        // sync_timer_frequencies(pwm_3x->FREQ);
        sync_pwm();

        // Syncronise the timers with a master timer.
        LL_TIM_SetSlaveMode(pwm_3x->TIM_A->getHandle()->Instance, LL_TIM_SLAVEMODE_DISABLED);
        LL_TIM_SetTriggerOutput(pwm_3x->TIM_A->getHandle()->Instance, LL_TIM_TRGO_ENABLE);

        // Configure the other two timers to get their input trigger from the master timer:
        for (auto timer : {pwm_3x->TIM_B, pwm_3x->TIM_C}) {
            LL_TIM_SetTriggerInput(timer->getHandle()->Instance, _getInternalSourceTrigger(pwm_3x->TIM_A, timer));
            LL_TIM_SetSlaveMode(timer->getHandle()->Instance, LL_TIM_SLAVEMODE_TRIGGER);
        }
        sync_pwm();

        // pwm_3x->TIM_A->resume();
        // pwm_3x->TIM_B->resume();
        // pwm_3x->TIM_C->resume();

    }

    void Maxwell::sync_timer_frequencies(long pwm_frequency) {
        auto timers = {pwm_3x->TIM_A, pwm_3x->TIM_B, pwm_3x->TIM_C};
        uint32_t min_freq = -1;
        uint32_t max_freq = 0;
        for (auto timer : {pwm_3x->TIM_A, pwm_3x->TIM_B, pwm_3x->TIM_C}) {
            uint32_t freq = timer->getTimerClkFreq();
            if (freq > max_freq) {
                max_freq = freq;
            }
            if (freq < min_freq) {
                min_freq = freq;
            }
        }
        uint32_t overflow = min_freq / pwm_frequency;


        for (auto timer : {pwm_3x->TIM_A, pwm_3x->TIM_B, pwm_3x->TIM_C}) {
            timer->setPrescaleFactor(timer->getTimerClkFreq()/min_freq);
            timer->setOverflow(overflow, TICK_FORMAT);
            timer->refresh();
        }
    }

    void Maxwell::sync_pwm() {
        pwm_3x->TIM_A->pause();
        pwm_3x->TIM_A->refresh();
        pwm_3x->TIM_B->pause();
        pwm_3x->TIM_B->refresh();
        pwm_3x->TIM_C->pause();
        pwm_3x->TIM_C->refresh();

        pwm_3x->TIM_A->resume();
        pwm_3x->TIM_B->resume();
        pwm_3x->TIM_C->resume();
    }

    void Maxwell::set_pwm(uint32_t Ua, uint32_t Ub, uint32_t Uc, uint32_t resolution) {
        Ua = constrain(Ua, 0.0, pwm_3x->MAX_COMPARE_VALUE);
        Ub = constrain(Ub, 0.0, pwm_3x->MAX_COMPARE_VALUE);
        Uc = constrain(Uc, 0.0, pwm_3x->MAX_COMPARE_VALUE);
        Ua = (Ua < 2.0)? 0 : Ua;
        Ub = (Ub < 2.0)? 0 : Ub;
        Uc = (Uc < 2.0)? 0 : Uc;
        pwm_3x->TIM_A->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_B->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_C->setOverflow(pwm_3x->FREQ, HERTZ_FORMAT);
        pwm_3x->TIM_A->setCaptureCompare(pwm_3x->channel_a, Ua, static_cast<TimerCompareFormat_t>(resolution));
        pwm_3x->TIM_B->setCaptureCompare(pwm_3x->channel_b, Ub, static_cast<TimerCompareFormat_t>(resolution));
        pwm_3x->TIM_C->setCaptureCompare(pwm_3x->channel_c, Uc, static_cast<TimerCompareFormat_t>(resolution));
    }

    void Maxwell::set_phase_voltages(float Va, float Vb, float Vc) {
        float input_voltage = 12;
        Va = constrain(Va, -max_voltage/2, max_voltage/2) + max_voltage / 2 + offset;
        Vb = constrain(Vb, -max_voltage/2, max_voltage/2) + max_voltage / 2 + offset;
        Vc = constrain(Vc, -max_voltage/2, max_voltage/2) + max_voltage / 2 + offset;

        Va = constrain(Va, offset, max_voltage+offset) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);
        Vb = constrain(Vb, offset, max_voltage+offset) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);
        Vc = constrain(Vc, offset, max_voltage+offset) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);

        set_pwm(static_cast<uint32_t>(Va),
            static_cast<uint32_t>(Vb),
            static_cast<uint32_t>(Vc),
            pwm_3x->RESOLUTION);
    }

    void Maxwell::set_phase_voltages(const dq_struct &command_dq) {
        encoder->update();
        alpha_beta_struct command_ab = reverse_park_transform(command_dq, encoder->get_angle());
        PhaseCurrents command_voltages = reverse_clarke_transform(command_ab);
        set_phase_voltages(command_voltages.current_a,
                            command_voltages.current_b,
                            command_voltages.current_c);
    }

    void Maxwell::set_phase_voltages(const dq_struct &command_dq, float theta) {
        alpha_beta_struct command_ab = reverse_park_transform(command_dq, theta/POLE_PAIRS_6374);
        PhaseCurrents command_voltages = reverse_clarke_transform(command_ab);
        set_phase_voltages(command_voltages.current_a,
                            command_voltages.current_b,
                            command_voltages.current_c);
    }

    void Maxwell::all_off() {
        set_pwm(0, 0, 0, pwm_3x->RESOLUTION);
    }

    void Maxwell::foc_init_sequence() {
        float old_max_voltage = max_voltage;

        driver->enable(true);
        // Ensure 3x PWM setup
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);
        digitalWriteFast(DRV8323_LO_A_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_B_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_C_PIN, HIGH);

        encoder->update();

        float zero_angle = encoder->get_angle();

        float theta = 0;
        for (long i=0; i<10000; i++) {
            theta += 0.001;
            // theta = fmod(theta, _2PI);
            encoder->update();
            if (i%1000==0) {
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {theta,encoder->get_angle() * POLE_PAIRS_6374}});}
            set_phase_voltages({align_max_voltage, 0}, theta);
        }
        encoder->update();
        float top_angle = encoder->get_angle();
        for (long i=0; i<10000; i++) {
            theta -= 0.001;
            // theta = fmod(theta, _2PI);
            encoder->update();
            if (i%1000==0) {
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {theta, encoder->get_angle() * POLE_PAIRS_6374}});
            }
            set_phase_voltages({align_max_voltage, 0}, theta);
        }
        float bottom_angle = encoder->get_angle();
        // Offset is the bottom angle (we should now be at zero electrical angle)
        encoder->set_offset(bottom_angle - theta);


        // Calculate CW or CCW encoder configuration
        float diff = top_angle - zero_angle;

        if (diff > 0) {
            encoder->_direction = SENSOR_DIRECTION::CW;
        }
        else if (diff < 0) {
            encoder->_direction = SENSOR_DIRECTION::CCW;
        }
        else {
            Serial.println("DID NOT DETECT MOVEMENT");
        }
        // Current sensor calibration:
        // set_phase_voltages(-0.75, 0, 0.75);
        // for (int i=0; i<10; i++) {
        //     current_sensors->read();
        //     double currents[3] = {current_sensors->get_current_a(),
        //                             current_sensors->get_current_b(),
        //                             current_sensors->get_current_c()};
        //     alpha_beta_struct ab_vec = clarke_transform(
        //         PhaseCurrents{static_cast<float>(currents[0]),
        //                         static_cast<float>(currents[1]),
        //                         static_cast<float>(currents[2])});
        //     encoder->update();
        //     dq_struct dq_vec = park_transform(ab_vec, encoder->get_angle());
        //     telemetry->send({TELEMETRY_PACKET_TYPE::PHASE_CURRENTS, {static_cast<float>(currents[0]),
        //                                                                             static_cast<float>(currents[1]),
        //                                                                             static_cast<float>(currents[2])}});
        //     // telemetry->send({TELEMETRY_PACKET_TYPE::DQ_CURRENTS, {static_cast<float>(dq_vec.d),
        //     //                                                                     static_cast<float>(dq_vec.q)}});
        //     telemetry->send({TELEMETRY_PACKET_TYPE::COMMAND_VOLTAGES, {-0.75, 0, 0.75}});
        //
        //     delay(10);
        // }

        // set_phase_voltages({align_max_voltage, 0}, 0);
        // delay(700);
        // encoder->update();
        // encoder->set_offset(encoder->get_angle());
        telemetry->send({GENERAL, {zero_angle, top_angle, bottom_angle,
                                                        static_cast<float>(encoder->_direction),
                                                        static_cast<float>(encoder->offset)}});
        set_phase_voltages(0, 0, 0);

        // // wait until current it low
        // double current_a = current_sensors->get_current_a();
        // double current_b = current_sensors->get_current_b();
        // double current_c = current_sensors->get_current_c();
        // while ((abs(current_a) > 2.0) || (abs(current_b) > 2.0) || (abs(current_c) > 2.0)) {
        //     current_a = current_sensors->get_current_a();
        //     current_b = current_sensors->get_current_b();
        //     current_c = current_sensors->get_current_c();
        //     telemetry->send({TELEMETRY_PACKET_TYPE::PHASE_CURRENTS, {static_cast<float>(current_a),
        //                                                                 static_cast<float>(current_b),
        //                                                                 static_cast<float>(current_c)}});
        //     delay(10);
        // }
        //
        // // Home the linear actuator by checking when current draw increases
        // bool homed = false;
        // theta = 0;
        // int i = 0;
        // float thresh = 10.0; // Amps
        // while (!homed) {
        //     theta += 0.001;
        //     set_phase_voltages({1.0, 0.0}, theta);
        //     current_a = current_sensors->get_current_a();
        //     current_b = current_sensors->get_current_b();
        //     current_c = current_sensors->get_current_c();
        //     encoder->update();
        //     if (i%500==0) {
        //         telemetry->send({TELEMETRY_PACKET_TYPE::PHASE_CURRENTS, {static_cast<float>(current_a),
        //                                                                                     static_cast<float>(current_b),
        //                                                                                     static_cast<float>(current_c)}});
        //         telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {theta,
        //                                                                                     encoder->get_angle()}});
        //     }
        //     if ((abs(current_a) > thresh || abs(current_b) > thresh || abs(current_c) > thresh) & (i > 5000)) {
        //         homed = true;
        //     }
        //     i++;
        // }
        // set_phase_voltages(0, 0, 0);
    }

    void Maxwell::sinusoidal_position_control() {
        driver->enable(true);
        // Ensure 3x PWM setup
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);
        digitalWriteFast(DRV8323_LO_A_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_B_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_C_PIN, HIGH);

        PIDController sinusoidal_pid_controller =
            PIDController(1,
                            0.1,
                            0.0,
                            0.0,
                            1,
                            1);
        float theta = 0;
        int i = 0;
        while (true) {
            // theta = static_cast<float>(pwm_input->read_percentage()) / 100 * 2 * PI * POLE_PAIRS_6374;
            theta -= 0.0001;
            theta = fmod(theta, _2PI);
            // sinusoidal_pid_controller.set_setpoint(theta);
            // float current_angle = encoder->get_angle();
            // sinusoidal_pid_controller.update(current_angle);
            // sinusoidal_pid_controller.print_state();

            // Generate three sin waves, offset by 120 degrees
            float U_a = _sin(theta)          * max_voltage/2;
            float U_b = _sin(theta - 2*_PI_3) * max_voltage/2;
            float U_c = _sin(theta + 2*_PI_3) * max_voltage/2;

            // if (i% 1000 == 0) Serial.println(U_b);

            set_phase_voltages(U_a, U_b, U_c);
            // double* currents = current_sensors->get_currents();
            if (i % 1500 == 0) {
                // current_sensors->read();
                double currents[3] = {current_sensors->get_current_a(),
                                        current_sensors->get_current_b(),
                                        current_sensors->get_current_c()};
                double average = (currents[0] + currents[1] + currents[2]) / 3;
                double rel_currents[3] = {
                    (currents[0] - average),
                    (currents[1] - average),
                    (currents[2] - average)
                };
                Serial.print(rel_currents[0]); Serial.print(" ");
                Serial.print(rel_currents[1]); Serial.print(" ");
                Serial.println(rel_currents[2]);
            }
            i++;
        }
    }

    void Maxwell::voltage_torque_control() {
        driver->enable(true);
        // Ensure 3x PWM setup
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);
        digitalWriteFast(DRV8323_LO_A_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_B_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_C_PIN, HIGH);

        int i = 0;
        uint32_t prev_millis = millis();
        while (true) {
            // Read pwm input and map as a voltage:
            float U_q = pwm_input->read_percentage() / 100.0 * max_voltage;
            // U_q = -U_q;

            float theta = encoder->get_angle();

            dq_struct command_dq = {0, U_q};
            set_phase_voltages(command_dq);


            uint32_t current_time = millis();

            if (current_time - prev_millis >= 30) { // Cannot be too fast, otherwise it messes up results (cannot handle serial buffer speed)
                double currents[3] = {current_sensors->get_current_a(),
                                     current_sensors->get_current_b(),
                                     current_sensors->get_current_c()};

                encoder->update();
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {encoder->get_angle()}});
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_VELOCITY, {encoder->get_velocity()}});
                telemetry->send({TELEMETRY_PACKET_TYPE::PHASE_CURRENTS, {static_cast<float>(currents[0]),
                                                                                        static_cast<float>(currents[1]),
                                                                                        static_cast<float>(currents[2])}});

                prev_millis = current_time;
            }
        }

    }

    void Maxwell::voltage_position_control() {
        driver->enable(true);
        // Ensure 3x PWM setup
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);
        digitalWriteFast(DRV8323_LO_A_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_B_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_C_PIN, HIGH);

        PIDController position_pid_controller =
            PIDController(  20,
                            0.0,
                            0.0,
                            0.0,
                            60,
                            60);
        PIDController velocity_pid_controller =
            PIDController(  0.2,
                            2,
                            0.0,
                            0.0,
                            20,
                            20);

        RCFilter velocity_lpf = RCFilter(2);
        RCFilter input_lpf = RCFilter(5);
        // Step function generator
        float step_period = 3; // seconds
        uint32_t step_period_start = millis();
        float high_angle = 2 * PI * 10; // 15 revolutions
        float low_angle = - high_angle; // 0 revolutions
        float desired_ramp_to_angle = low_angle; // radians
        float ramp_step_percentage = 0.5; // 10% of the step period
        float desired_angle = low_angle;


        uint32_t prev_millis = millis();
        while (true) {
            uint32_t current_time_us = micros();
            uint32_t current_time_ms = millis();
            // double desired_angle = input_lpf.update(pwm_input->read_percentage() / 100.0 * 2 * PI * 15, current_time_us); // Desired_angle is from the pwm_input

            // Trapezoidal reference generator
            if (current_time_ms - step_period_start >= (step_period * 1000)) {
                step_period_start = current_time_ms;
                desired_ramp_to_angle = (desired_ramp_to_angle == low_angle) ? high_angle : low_angle;
            }
            else {
                float ramp_duration = step_period * ramp_step_percentage;
                float elapsed = (current_time_ms - step_period_start) / 1000.0; // Convert to seconds
                float ramp_progress = elapsed / ramp_duration; // 0 to 1
                if (elapsed <= ramp_duration) {
                    if (desired_ramp_to_angle == low_angle) {
                        desired_angle = high_angle - (high_angle - low_angle) * ramp_progress;
                    } else if (desired_ramp_to_angle == high_angle) {
                        desired_angle = low_angle + (high_angle - low_angle) * ramp_progress;

                    }
                } else {
                    desired_angle = desired_ramp_to_angle;
                }
            }

            position_pid_controller.set_setpoint(desired_angle);

            encoder->update();
            double rotor_theta = encoder->get_angle();
            double rotor_velocity = velocity_lpf.update(encoder->get_velocity(), current_time_us);


            double desired_velocity = position_pid_controller.update(rotor_theta);

            velocity_pid_controller.set_setpoint(desired_velocity);
            // double rotor_velocity_error = desired_velocity - rotor_velocity;
            double I_q = velocity_pid_controller.update(rotor_velocity);
            // I_q = constrain(I_q, -max_current, max_current);

            // position_pid_controller.print_state();
            // velocity_pid_controller.print_state();

            dq_struct command_dq = {0, I_q};
            set_phase_voltages(command_dq);

            if (current_time_ms - prev_millis >= 30) {
                // Cannot be too fast, otherwise it messes up results (cannot handle serial buffer speed)
                // current_sensors->read();
                double currents[3] = {current_sensors->get_current_a(),
                                     current_sensors->get_current_b(),
                                     current_sensors->get_current_c()};
                PhaseCurrents phase_currents = {static_cast<float>(currents[0]),
                                                static_cast<float>(currents[1]),
                                                static_cast<float>(currents[2])};

                alpha_beta_struct ab_vec = clarke_transform(phase_currents);
                dq_struct dq_vec = park_transform(ab_vec, rotor_theta);
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {static_cast<float>(desired_angle),
                                                                                        static_cast<float>(rotor_theta)}});

                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_VELOCITY, {static_cast<float>(desired_velocity),
                                                                                        static_cast<float>(rotor_velocity)}});
                telemetry->send({TELEMETRY_PACKET_TYPE::PHASE_CURRENTS, {static_cast<float>(currents[0]),
                                                                                        static_cast<float>(currents[1]),
                                                                                        static_cast<float>(currents[2])}});




                // position_pid_controller.print_state();
                // velocity_pid_controller.print_state();

                prev_millis = current_time_ms;
            }
        }
    }

    void Maxwell::dc_current_torque_control() {
        driver->enable(true);
        // Ensure 3x PWM setup
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);
        digitalWriteFast(DRV8323_LO_A_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_B_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_C_PIN, HIGH);

        PIDController q_pid_controller =
            PIDController(  5,
                            0.1,
                            0.0,
                            0.0,
                            max_current,
                            1);


        auto I_DC_LPF = RCFilter(1);
        uint32_t prev_millis = 0;
        uint32_t current_time_ms = millis();
        uint32_t current_time_us = micros();
        while (true) {
            current_time_us = micros();
            current_time_ms = millis();
            float I_q = pwm_input->read_percentage() / 100.0 * max_current;
            q_pid_controller.set_setpoint(I_q);

            encoder->update();
            float theta = encoder->get_angle();

            current_sensors->read();
            PhaseCurrents currents = {
                current_sensors->get_current_a(),
                current_sensors->get_current_b(),
                current_sensors->get_current_c()
            };
            alpha_beta_struct ab_vec = clarke_transform(currents);
            dq_struct dq_meas = park_transform(ab_vec, theta);

            // Low pass filter the dq measurements
            float I_DC = sqrt(ab_vec.alpha*ab_vec.alpha + ab_vec.beta*ab_vec.beta) * (dq_meas.q > 0) ? 1 : -1;
            I_DC = I_DC_LPF.update(I_DC, current_time_us);

            // Update the PID controllers
            dq_struct command_dq = {0, 0};
            command_dq.q = q_pid_controller.update(I_DC);

            alpha_beta_struct command_ab = reverse_park_transform(command_dq, theta);
            PhaseCurrents command_voltages = reverse_clarke_transform(command_ab);
            set_phase_voltages(command_voltages.current_a,
                                command_voltages.current_b,
                                command_voltages.current_c);
#ifdef SERIAL_FEEDBACK_ENABLED
            if (current_time_ms - prev_millis >= 30) {
                // Cannot be too fast, otherwise it messes up results (cannot handle serial buffer speed)
                phase_current_frame.values = {currents.current_a,
                                            currents.current_b,
                                            currents.current_c};

                alpha_beta_frame.values = {ab_vec.alpha, ab_vec.beta};
                dq_frame.values = {I_DC, command_dq.q, I_q};
                command_voltage_frame.values = {command_voltages.current_a,
                                                    command_voltages.current_b,
                                                    command_voltages.current_c};

                rotor_position_frame.values = {encoder->get_angle()};

                send_frame(phase_current_frame);
                send_frame(alpha_beta_frame);
                send_frame(dq_frame);
                send_frame(rotor_position_frame);
                send_frame(command_voltage_frame);
                prev_millis = current_time_ms;
            }
#endif
        }
    }

    void Maxwell::foc_current_torque_control() {
        driver->enable(true);
        // Ensure 3x PWM setup
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);
        digitalWriteFast(DRV8323_LO_A_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_B_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_C_PIN, HIGH);

        PIDController d_pid_controller =
            PIDController(  2.0,
                            0.0,
                            0.0,
                            0.0,
                            max_current,
                            1);

        PIDController q_pid_controller =
            PIDController(  2.0,
                            0.0,
                            0.0,
                            0.0,
                            max_current,
                            3);
        PIDController position_pid_controller =
            PIDController(  20,
                            0.0,
                            0.0,
                            0.0,
                            250,
                            250);
        PIDController velocity_pid_controller =
            PIDController(  0.05,
                            1.0,
                            0.000,
                            0.0,
                            20,
                            20);


        d_pid_controller.set_setpoint(0.0);

        auto q_lpf = RCFilter(0.5);
        auto d_lpf = RCFilter(0.5);
        auto command_q_lpf = RCFilter(40.0);
        auto command_d_lpf = RCFilter(40.0);
        auto velocity_lpf = RCFilter(2.0);
        auto input_lpf = RCFilter(2.0);


        double step_period = 0.3; // seconds
        uint32_t step_period_start = millis();
        double high_angle = -0.5; // 3 revolutions
        double low_angle = -2.0 * PI * 6.0; // 0 revolutions
        double desired_ramp_to_angle = low_angle; // radians


        double desired_angle = 0.0;
        float thresh = 20.0; // 20A current threshold for homing
        float position_offset = 0.0;


        uint32_t prev_millis = 0;
        uint32_t current_time_ms = millis();
        uint32_t current_time_us = micros();
        bool homed = false;
        while (true) {
            current_time_us = micros();
            current_time_ms = millis();


            if (homed) {
                desired_angle = input_lpf.update(-pwm_input->read_percentage() / 100.0 * 2*PI * 5 + position_offset, current_time_us);

                // Trapezoidal reference generator
            //     if (current_time_ms - step_period_start >= (step_period * 1000.0)) {
            //         step_period_start = current_time_ms;
            //         desired_ramp_to_angle = (desired_ramp_to_angle == low_angle) ? high_angle : low_angle;
            //     }
            //     else {
            //         double ramp_step_percentage = 1.0;
            //         double ramp_duration = step_period * ramp_step_percentage;
            //         double elapsed = (current_time_ms - step_period_start) / 1000.0; // Convert to seconds
            //         double ramp_progress = elapsed / ramp_duration; // 0 to 1
            //         if (elapsed <= ramp_duration) {
            //             if (desired_ramp_to_angle == low_angle) {
            //                 desired_angle = high_angle - (high_angle - low_angle) * ramp_progress;
            //             } else if (desired_ramp_to_angle == high_angle) {
            //                 desired_angle = low_angle + (high_angle - low_angle) * ramp_progress;
            //             }
            //         } else {
            //             desired_angle = desired_ramp_to_angle;
            //         }
            //     }
            //     desired_angle += position_offset;
            //     desired_angle = input_lpf.update(desired_angle, current_time_us);
            }
            position_pid_controller.set_setpoint(desired_angle);

            encoder->update();
            float theta = encoder->get_angle();
            float rotor_velocity = encoder->get_velocity();
            rotor_velocity = velocity_lpf.update(rotor_velocity, current_time_us);
            if (rotor_velocity < 0.2 && rotor_velocity > -0.2) {
                rotor_velocity = 0; // Prevents the PID controller from going crazy
            }
            float vel_ref = position_pid_controller.update(theta);
            // float vel_ref = pwm_input->read_percentage() / 100.0f * 100;

            velocity_pid_controller.set_setpoint(vel_ref);

            float I_q = velocity_pid_controller.update(rotor_velocity);

            // float I_q = pwm_input->read_percentage() / 100.0f * max_current;
            q_pid_controller.set_setpoint(I_q);

            // current_sensors->read();
            PhaseCurrents currents = {
                current_sensors->get_current_a(),
                current_sensors->get_current_b(),
                current_sensors->get_current_c()
            };
            alpha_beta_struct ab_vec = clarke_transform(currents);
            dq_struct dq_meas = park_transform(ab_vec, theta);

            // Low pass filter the dq measurements
            dq_meas.d = d_lpf.update(dq_meas.d, current_time_us);
            dq_meas.q = q_lpf.update(dq_meas.q, current_time_us);

            // Update the PID controllers
            dq_struct command_dq = {0, I_q}; // Running in pure voltage mode - Not very energy efficient
            // command_dq.d = command_d_lpf.update(d_pid_controller.update(dq_meas.d), current_time_us);
            // command_dq.q = command_q_lpf.update(q_pid_controller.update(dq_meas.q), current_time_us);
            alpha_beta_struct command_ab = reverse_park_transform(command_dq, theta);
            PhaseCurrents command_voltages = reverse_clarke_transform(command_ab);
            set_phase_voltages(command_voltages.current_a,
                                command_voltages.current_b,
                                command_voltages.current_c);
            // float loop_freq = 1 / ((micros() - current_time_us) / 1000000.0);
            // Serial.println(loop_freq);
            if (!homed) {
                desired_angle += 0.01;

                if (abs(currents.current_a) > thresh || abs(currents.current_b) > thresh || abs(currents.current_c) > thresh) {
                    homed = true;
                    position_offset = theta - 0.5;

                    // encoder->set_offset(theta);
                    desired_angle = position_offset;
                }
            }

            if (current_time_ms - prev_millis >= 30) {
                // Cannot be too fast, otherwise it messes up results (cannot handle serial buffer speed)
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {static_cast<float>(desired_angle),
                                                                                        static_cast<float>(theta)}});
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_VELOCITY, {static_cast<float>(vel_ref),
                                                                                        static_cast<float>(rotor_velocity)}});
                telemetry->send({TELEMETRY_PACKET_TYPE::PHASE_CURRENTS, {static_cast<float>(currents.current_a),
                                                                                          static_cast<float>(currents.current_b),
                                                                                          static_cast<float>(currents.current_c)}});
                telemetry->send({TELEMETRY_PACKET_TYPE::DQ_CURRENTS, {static_cast<float>(dq_meas.d),
                                                                                       static_cast<float>(dq_meas.q),
                                                                                        static_cast<float>(command_dq.d),
                                                                                        static_cast<float>(command_dq.q),
                                                                                        static_cast<float>(I_q)
                }});

                prev_millis = current_time_ms;
            }
        }
    }

    alpha_beta_struct Maxwell::clarke_transform(PhaseCurrents currents) { // currents to alpha-beta
        // float I_alpha = currents.current_a;
        // float I_beta = (currents.current_a  + 2*currents.current_b) / _SQRT3;

        // Altenative Amplitude-invariant version
        float I_alpha = currents.current_a;
        float I_beta = (currents.current_b - currents.current_c) / _SQRT3;

        alpha_beta_struct ab = {I_alpha, I_beta};
        return ab;
    }

    dq_struct Maxwell::park_transform(alpha_beta_struct ab_vec, float theta) { // alpha-beta to dq
        // the park transform
        float electrical_theta = theta * POLE_PAIRS_6374; // Assuming we're aligned with the encoder!
        float d = ab_vec.alpha * _cos(electrical_theta)  + ab_vec.beta * _sin(electrical_theta);
        float q = -ab_vec.alpha * _sin(electrical_theta) + ab_vec.beta * _cos(electrical_theta);
        dq_struct dq = {d, q};
        return dq;
    }

    alpha_beta_struct Maxwell::reverse_park_transform(dq_struct dq_vec, float theta) {  // dq to alpha-beta
        float electrical_theta = theta * POLE_PAIRS_6374; // Assuming we're aligned with the encoder!
        double c = _cos(electrical_theta);
        double s = _sin(electrical_theta);
        float alpha = dq_vec.d * c - dq_vec.q * s;
        float beta  = dq_vec.d * s + dq_vec.q * c;
        alpha_beta_struct alpha_beta = {alpha, beta};
        return alpha_beta;
    }

    PhaseCurrents Maxwell::reverse_clarke_transform(alpha_beta_struct ab_vec) {
        // PhaseCurrents currents = {(2/3)*ab_vec.alpha,
        //                     (-1/3)*ab_vec.alpha + (sqrt(3)/3)*ab_vec.beta,
        //                     (-1/3)*ab_vec.alpha - (sqrt(3)/3)*ab_vec.beta};
        PhaseCurrents currents;
        currents.current_a = ab_vec.alpha;
        currents.current_b = (-ab_vec.alpha + M_SQRT3*ab_vec.beta) / 2;
        currents.current_c = (-ab_vec.alpha - M_SQRT3*ab_vec.beta) / 2;
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
        // vTaskDelay(100);

        analogWrite(DRV8323_HI_A_PIN, 70);
        analogWrite(DRV8323_LO_C_PIN, 70);
        // vTaskDelay(100);

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
            // vTaskDelay(on_time);

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
