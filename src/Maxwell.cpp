//
// Created by robos on 11/09/2023.
//

#include "Maxwell.h"

#include <ratio>
#include <FreeRTOS/Source/include/FreeRTOS.h>

#include "AS5047P.h"
#include "FreeRTOS/Source/include/FreeRTOS.h"

namespace Maxwell {
    #define MAX_LEVEL 80  // out of 255
    #define MAX_SPEED 2000 // electrical rads/s
    Maxwell *Maxwell::instance = nullptr;

    Maxwell::Maxwell() {
        SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK
        SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK
        driver = new DRV8323::DRV8323(
            DRV8323_CS_PIN,
            SPI_1,
            1000000,
            // 1 MHz SPI frequency - This is the max frequency for HARDWARE_V2_0 due to parasitics on the SDO line.
            DRV8323_HI_A_PIN,
            DRV8323_HI_B_PIN,
            DRV8323_HI_C_PIN,
            DRV8323_GATE_EN_PIN);

        // TODO: Make the choice between internal and external encoder a config option
        encoder = new AS5048A( // Internal encoder
            AS5048A_CS_PIN,
            SPI_1,
            1000000); // 1 MHz SPI frequency
        // encoder = new AS5048A ( // External encoder
        //     EXTERNAL_ENCODER_CS_PIN,
        //     SPI_2,
        //     10000); // 1 MHz SPI frequency


        /*
         * Instantiate all Command Sources and Telemetry Targets
         */
        // Command Sources
        pwm_input = new PWMInput(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);
        can_bus = new CANBus(500000); // Also a Telemetry target

        // Telemetry Targets
        usb_target = new USBTarget();

        current_sensors = new CurrentSensors(CURR_SENSE_A_PIN,
                                             CURR_SENSE_B_PIN,
                                             CURR_SENSE_C_PIN,
                                             _csa_gain);

        pwm_3x = new pwm_3x_struct();
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
        digitalWriteFast(DRV8323_LO_A_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_B_PIN, HIGH);
        digitalWriteFast(DRV8323_LO_C_PIN, HIGH);


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

        // Default config - only if we're not loading from flash
        config = default_config_struct;
        load_control_config();

        t_config = config_manager.read_config(); // Try to read the config from flash

        if (not t_config.configured) {
            sensor_offset_calibration();
        }
        else {
            encoder->set_offset(t_config.offset);
        }

        telemetry->send({TELEMETRY_PACKET_TYPE::GENERAL, {t_config.offset}});


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
        Va = constrain(Va, -limits.max_voltage/2, limits.max_voltage/2) + limits.max_voltage / 2 + offset;
        Vb = constrain(Vb, -limits.max_voltage/2, limits.max_voltage/2) + limits.max_voltage / 2 + offset;
        Vc = constrain(Vc, -limits.max_voltage/2, limits.max_voltage/2) + limits.max_voltage / 2 + offset;

        Va = constrain(Va, offset, limits.max_voltage+offset) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);
        Vb = constrain(Vb, offset, limits.max_voltage+offset) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);
        Vc = constrain(Vc, offset, limits.max_voltage+offset) / input_voltage * static_cast<float>(pwm_3x->MAX_COMPARE_VALUE);

        set_pwm(static_cast<uint32_t>(Va),
            static_cast<uint32_t>(Vb),
            static_cast<uint32_t>(Vc),
            pwm_3x->RESOLUTION);
    }

    void Maxwell::set_phase_voltages(const dq_struct &command_dq) {
        encoder->update();
        ab_struct command_ab = reverse_park_transform(command_dq, encoder->get_angle());
        PhaseCurrents command_voltages = reverse_clarke_transform(command_ab);
        set_phase_voltages(command_voltages.current_a,
                            command_voltages.current_b,
                            command_voltages.current_c);
    }

    void Maxwell::set_phase_voltages(const dq_struct &command_dq, float theta) {
        ab_struct command_ab = reverse_park_transform(command_dq, theta/POLE_PAIRS_6374);
        PhaseCurrents command_voltages = reverse_clarke_transform(command_ab);
        set_phase_voltages(command_voltages.current_a,
                            command_voltages.current_b,
                            command_voltages.current_c);
    }

    void Maxwell::all_off() {
        set_pwm(0, 0, 0, pwm_3x->RESOLUTION);
    }

    void Maxwell::sensor_offset_calibration() {
        // Initialise 3x PWM generation on the STM32
        init_pwm_3x();

        // Setup components for bldc_control
        driver->enable(true); // Enable driver in 3x mode
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);  // Ensure 3x PWM setup
        // set_phase_voltages(0, 0, 0); // Ensure we're not sending any voltage to the motor

        encoder->update();
        float zero_angle = encoder->get_angle();

        float theta = 0;
        for (long i=0; i<50000; i++) {
            theta += 0.0005;
            // theta = fmod(theta, _2PI);
            encoder->update();
            if (i%1000==0) {
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {theta,encoder->get_angle() * config.pole_pairs}});
            }
            set_phase_voltages({limits.align_voltage, 0}, theta);
        }
        encoder->update();
        float top_angle = encoder->get_angle();
        for (long i=0; i<50000; i++) {
            theta -= 0.0005;
            // theta = fmod(theta, _2PI);
            encoder->update();
            if (i%1000==0) {
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {theta, encoder->get_angle() * config.pole_pairs}});
            }
            set_phase_voltages({limits.align_voltage, 0}, theta);
        }
        float bottom_angle = encoder->get_angle();
        // Offset is the bottom angle (we should now be at zero electrical angle)
        float off = bottom_angle - theta;
        encoder->set_offset(off);

        t_config.configured = true;
        t_config.magic_number = 0xDEADBEEF;
        t_config.offset = off;
        config_manager.write_config(t_config);


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

        // set_phase_voltages({limits.align_voltage, 0}, 0);
        // delay(700);
        // encoder->update();
        // encoder->set_offset(encoder->get_angle());
        // telemetry->send({GENERAL, {zero_angle, top_angle, bottom_angle,
        //                                                 static_cast<float>(encoder->_direction),
        //                                                 static_cast<float>(encoder->offset)}});
        // set_phase_voltages(0, 0, 0);

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

    void Maxwell::load_control_config() {
        // Populate FOC struct from config
        foc.d_pid = new PIDController( config.d_pid_config.kp,
                                        config.d_pid_config.ki,
                                        config.d_pid_config.kd,
                                        0.0,
                                        config.d_pid_config.max_output,
                                        config.d_pid_config.max_integral);
        foc.q_pid = new PIDController( config.q_pid_config.kp,
                                        config.q_pid_config.ki,
                                        config.q_pid_config.kd,
                                        0.0,
                                        config.q_pid_config.max_output,
                                        config.q_pid_config.max_integral);
        foc.position_pid = new PIDController(  config.position_pid_config.kp,
                                                config.position_pid_config.ki,
                                                config.position_pid_config.kd,
                                                0.0,
                                                config.position_pid_config.max_output,
                                                config.position_pid_config.max_integral);
        foc.velocity_pid = new PIDController(  config.velocity_pid_config.kp,
                                                config.velocity_pid_config.ki,
                                                config.velocity_pid_config.kd,
                                                0.0,
                                                config.velocity_pid_config.max_output,
                                                config.velocity_pid_config.max_integral);

        // LPFs
        foc.d_lpf = new RCFilter(config.d_lpf_config.cutoff_frequency);
        foc.q_lpf = new RCFilter(config.q_lpf_config.cutoff_frequency);
        foc.velocity_lpf = new RCFilter(config.velocity_lpf_config.cutoff_frequency);
        foc.input_lpf = new RCFilter(config.input_lpf_config.cutoff_frequency);


        switch (config.command_source) {
            case (COMMAND_SOURCE::PWM): {
                command_source = pwm_input;
                // command_source->command_gain = 1.0;
                break;
            }
            case (COMMAND_SOURCE::CAN): {
                command_source = can_bus;
                break;
            }
            default: break;
        }
        switch (config.telemetry_target) {
            case (TELEMETRY_TARGET::TELEMETRY_USB): {
                telemetry = usb_target;
                break;
            }
            case (TELEMETRY_TARGET::TELEMETRY_CAN): {
                telemetry = can_bus;
                break;
            }
            default: break;
        }
    }

    void Maxwell::motor_control() {
        // Interpret the current config, and call the relevant control loop (e.g. sinusoidal position control, voltage control, current control etc.)
        switch (config.motor_type) {
            case (MOTOR_TYPE::DC): {
                // dc_control();
                break;
            }
            case (MOTOR_TYPE::BLDC): {
                bldc_control();
                break;
            }
        }
    }

    void Maxwell::bldc_control() {
        // Initialise 3x PWM generation on the STM32
        init_pwm_3x();

        // Setup components for bldc_control
        driver->enable(true); // Enable driver in 3x mode
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);  // Ensure 3x PWM setup
        PIDController d_pid =
                    PIDController(  2.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    limits.max_current,
                                    1);
        PIDController q_pid =
            PIDController(  2.0,
                            0.0,
                            0.0,
                            0.0,
                            limits.max_current,
                            3);




        uint32_t prev_millis = millis();
        switch (config.control_mode) {
            // Torque control mode: i.e. The reference command is a torque.
            case (CONTROL_MODE::TORQUE): {
                while (true) {
                    // Read torque command from command source and convert to voltage command
                    float reference = command_source->read();
                    encoder->update();
                    float angle = encoder->get_angle();
                    float velocity = encoder->get_velocity();
                    uint32_t current_time_ms = millis();
                    uint32_t current_time_us = micros();


                    switch (config.torque_control_mode) {
                        case (TORQUE_CONTROL_MODE::VOLTAGE): {
                            // TORQUE CONTROL, with TORQUE CONTROL MODE
                            float U_q = reference * limits.max_voltage;
                            set_phase_voltages({0, U_q});
                            break;
                        }


                        case (TORQUE_CONTROL_MODE::CURRENT): {
                            // Read reference and set q-axis set point
                            float I_q = reference * limits.max_current;
                            d_pid.set_setpoint(0.0);
                            q_pid.set_setpoint(I_q);

                            // // Measure current and transform
                            foc.phase_current_meas = {current_sensors->get_current_a(),
                                                        current_sensors->get_current_b(),
                                                        current_sensors->get_current_c()};
                            foc.ab_meas = clarke_transform(foc.phase_current_meas);
                            foc.dq_meas = park_transform(foc.ab_meas, angle);

                            // Filter measured currents
                            foc.dq_meas.d = foc.d_lpf->update(foc.dq_meas.d, current_time_us);
                            foc.dq_meas.q = foc.q_lpf->update(foc.dq_meas.q, current_time_us);

                            // // Update PID controllers
                            foc.command_dq.d = d_pid.update(foc.dq_meas.d);
                            foc.command_dq.q = q_pid.update(foc.dq_meas.q);
                            //
                            // // Generate voltage command from PID output
                            set_phase_voltages(foc.command_dq);
                            break;
                        }
                        default: break;
                    }

                    if (current_time_ms - prev_millis >= 30) {
                        telemetry->send({TELEMETRY_PACKET_TYPE::COMMAND, {reference}});
                        telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {angle}});
                        telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_VELOCITY, {velocity}});
                        telemetry->send({TELEMETRY_PACKET_TYPE::PHASE_CURRENTS, {static_cast<float>(foc.phase_current_meas.current_a),
                                                                                        static_cast<float>(foc.phase_current_meas.current_b),
                                                                                        static_cast<float>(foc.phase_current_meas.current_c)}});
                        telemetry->send({TELEMETRY_PACKET_TYPE::DQ_CURRENTS, {static_cast<float>(foc.dq_meas.d),
                                                                                                        static_cast<float>(foc.dq_meas.q),
                                                                                                        static_cast<float>(foc.command_dq.d),
                                                                                                        static_cast<float>(foc.command_dq.q)}});
                        prev_millis = current_time_ms;
                    }
                }
                break;
            };
            case (CONTROL_MODE::VELOCITY): {
                while (true) {

                }
                break;
            };
            default: break;
        }
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
            float U_a = _sin(theta)          * limits.max_voltage/2;
            float U_b = _sin(theta - 2*_PI_3) * limits.max_voltage/2;
            float U_c = _sin(theta + 2*_PI_3) * limits.max_voltage/2;

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
            float U_q = pwm_input->read_percentage() / 100.0 * limits.max_voltage;
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

                ab_struct ab_vec = clarke_transform(phase_currents);
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
                            limits.max_current,
                            1);


        auto I_DC_LPF = RCFilter(1);
        uint32_t prev_millis = 0;
        uint32_t current_time_ms = millis();
        uint32_t current_time_us = micros();
        while (true) {
            current_time_us = micros();
            current_time_ms = millis();
            float I_q = pwm_input->read_percentage() / 100.0 * limits.max_current;
            q_pid_controller.set_setpoint(I_q);

            encoder->update();
            float theta = encoder->get_angle();

            current_sensors->read();
            PhaseCurrents currents = {
                current_sensors->get_current_a(),
                current_sensors->get_current_b(),
                current_sensors->get_current_c()
            };
            ab_struct ab_vec = clarke_transform(currents);
            dq_struct dq_meas = park_transform(ab_vec, theta);

            // Low pass filter the dq measurements
            float I_DC = sqrt(ab_vec.alpha*ab_vec.alpha + ab_vec.beta*ab_vec.beta) * (dq_meas.q > 0) ? 1 : -1;
            I_DC = I_DC_LPF.update(I_DC, current_time_us);

            // Update the PID controllers
            dq_struct command_dq = {0, 0};
            command_dq.q = q_pid_controller.update(I_DC);

            ab_struct command_ab = reverse_park_transform(command_dq, theta);
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

        // Tuning method:
        // Tune q and d axis controllers properly, then assume they are ideal
        // Tune


        PIDController d_pid_controller =
            PIDController(  2.0,
                            0.0,
                            0.0,
                            0.0,
                            limits.max_current,
                            1);

        PIDController q_pid_controller =
            PIDController(  2.0,
                            0.0,
                            0.0,
                            0.0,
                            limits.max_current,
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
            ab_struct ab_vec = clarke_transform(currents);
            dq_struct dq_meas = park_transform(ab_vec, theta);

            // Low pass filter the dq measurements
            dq_meas.d = d_lpf.update(dq_meas.d, current_time_us);
            dq_meas.q = q_lpf.update(dq_meas.q, current_time_us);

            // Update the PID controllers
            dq_struct command_dq = {0, I_q}; // Running in pure voltage mode - Not very energy efficient
            // command_dq.d = command_d_lpf.update(d_pid_controller.update(dq_meas.d), current_time_us);
            // command_dq.q = command_q_lpf.update(q_pid_controller.update(dq_meas.q), current_time_us);
            ab_struct command_ab = reverse_park_transform(command_dq, theta);
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




}
