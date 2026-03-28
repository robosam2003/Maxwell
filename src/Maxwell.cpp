//
// Created by robos on 11/09/2023.
//

#include "Maxwell.h"

#include <ratio>
#include <Wire.h>
#include <FreeRTOS/Source/include/FreeRTOS.h>

#include "FreeRTOS.h"
#include "FreeRTOS/Source/include/FreeRTOS.h"

namespace Maxwell {
    #define MAX_LEVEL 80  // out of 255
    #define MAX_SPEED 2000 // electrical rads/s
    Maxwell *Maxwell::instance = nullptr;

    Maxwell::Maxwell() {
        SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK
        SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK

        // *ALL* CS pins on a given bus must be pulled high before we try to communicate anything
        pinMode(AS5048A_CS_PIN, OUTPUT);
        digitalWrite(AS5048A_CS_PIN, HIGH); // Ensure the CS pin is high

        driver = new DRV8323::DRV8323(
            DRV8323_CS_PIN,
            SPI_1,
            1000000,
            // 1 MHz SPI frequency - This is the max frequency for HARDWARE_V2_0 due to parasitics on the SDO line.
            DRV8323_HI_A_PIN,
            DRV8323_HI_B_PIN,
            DRV8323_HI_C_PIN,
            DRV8323_GATE_EN_PIN);
        driver->default_configuration();

        // TODO: Make the choice between internal and external encoder a config option
        // if (config.sensor_location == SENSOR_LOCATION::INTERNAL) {

        // }
        // else {
        //     encoder = new AS5048 ( // External encoder
        //         EXTERNAL_ENCODER_CS_PIN,
        //         SPI_2,
        //         10000); // 1 MHz SPI frequency
        // }
        // encoder = new AS5048 ( // External encoder
        //         EXTERNAL_ENCODER_CS_PIN,
        //         SPI_2,
        //         10000000); // 1 MHz SPI frequency
        // encoder = new AS5048( // Internal encoder
        //         AS5048A_CS_PIN,
        //         SPI_1,
        //         1000000); // 1 MHz SPI frequency
                // encoder = new AS5048( // Internal encoder
        //         AS5048A_CS_PIN,
        //         SPI_1,
        //         1000000); // 1 MHz SPI frequency


        encoder = new AS5047P ( // External encoder
                EXTERNAL_ENCODER_CS_PIN,
                SPI_2,
                1000000); // 10 MHz SPI frequency


        // encoder = new AS5048(AS5048::INT); // DMA Mode

        // External encoder
        // encoder = new AS5600(CS_SDA2_RX3_EXTI_PIN, SCK2_SCL2_TX3_EXTI_PIN, 10000); // 10 kHz I2C frequency

        /*
         * Instantiate all Command Sources and Telemetry Targets
         */
        // Command Sources
        pwm_input = new PWMInput(PWM_IN_PIN, UNIDIRECTIONAL, FORWARD);
        can_bus = new CANBus(500000); // Also a Telemetry target

        // Telemetry Targets
        usb_target = new USBTarget();

        adc = new Adc(CURR_SENSE_A_PIN,
                     CURR_SENSE_B_PIN,
                     CURR_SENSE_C_PIN,
                     V_SENSE_A_PIN,
                     V_SENSE_B_PIN,
                     V_SENSE_C_PIN,
                     V_SUPPLY_SENSE_PIN);

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

        // supply_voltage_watchdog(); // Populate the initial supply voltage reading

        t_config = config_manager.read_config(); // Try to read the config from flash
        sensor_offset_calibration();

        // if (not t_config.configured) {
        //     sensor_offset_calibration();
        //
        // }
        // else {
        //     encoder->set_offset(t_config.offset);
        // }

        // telemetry->send({TELEMETRY_PACKET_TYPE::GENERAL, {t_config.offset}});
        telemetry->DEBUG("Sensor Offset: " + String(encoder->offset));
        telemetry->DEBUG("Maxwell Setup complete");

        // Let velocity estimate stabilise before starting control loop
        for (int i = 0; i < 500; i++) {
            encoder->update();
            update_observer(encoder->get_angle(), micros());
            // telemetry->DEBUG("Reading: " + String(encoder->get_angle()));
            delay(1);
        }
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
        // pwm_3x->TIM_A->getHandle()->Instance->CCR1
        // Set timer direction
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

        LL_TIM_SetTriggerOutput(pwm_3x->TIM_A->getHandle()->Instance, LL_TIM_TRGO_UPDATE);

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
        PhaseVoltages command_voltages = reverse_clarke_transform(command_ab);
        set_phase_voltages(command_voltages.voltage_a,
                            command_voltages.voltage_b,
                            command_voltages.voltage_c);
    }

    void Maxwell::set_phase_voltages(const dq_struct &command_dq, float theta) {
        ab_struct command_ab = reverse_park_transform(command_dq, theta / POLE_PAIRS_6374);
        PhaseVoltages command_voltages = reverse_clarke_transform(command_ab);
        set_phase_voltages(command_voltages.voltage_a,
                            command_voltages.voltage_b,
                            command_voltages.voltage_c);
    }

    void Maxwell::all_off() {
        set_pwm(0, 0, 0, pwm_3x->RESOLUTION);
    }

    void Maxwell::sensor_offset_calibration() {
        // supply_voltage_watchdog(); // Ensure we have a valid supply voltage reading before starting the calibration
        // while (!driver_active) {
        //     telemetry->DEBUG("Insufficient supply voltage: " + String(input_voltage));
        //     delay(500);
        //     supply_voltage_watchdog(); // Ensure we have a valid supply voltage reading before starting the calibration
        // }
        // Initialise 3x PWM generation on the STM32
        init_pwm_3x();

        // Setup components for bldc_control
        driver->enable(true); // Enable driver in 3x mode
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);  // Ensure 3x PWM setup
        // set_phase_voltages(0, 0, 0); // Ensure we're not sending any voltage to the motor

        encoder->update();
        float zero_angle = encoder->get_angle();

        float average_diff = 0;
        float theta = 0;
        float theta_inc = 0.001;
        int theta_inc_len = int(_2PI / theta_inc);

        for (long i=0; i<theta_inc_len; i++) {
            theta += theta_inc;
            // theta = fmod(theta, _2PI);
            encoder->update();
            if (i%1000==0) {
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {theta,encoder->get_angle()}});
            }
            average_diff = (average_diff * i + (encoder->get_angle()*config.pole_pairs - theta)) / (i+1);
            set_phase_voltages({limits.align_voltage, 0}, theta);
        }
        encoder->update();
        float top_angle = encoder->get_angle();
        for (long i=0; i<theta_inc_len; i++) {
            theta -= theta_inc;
            // theta = fmod(theta, _2PI);
            encoder->update();
            if (i%1000==0) {
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {theta, encoder->get_angle()}});
            }
            average_diff = (average_diff * i + (encoder->get_angle()*config.pole_pairs - theta)) / (i+1);
            set_phase_voltages({limits.align_voltage, 0}, theta);
        }
        float bottom_angle = encoder->get_angle();
        // Offset is the bottom angle (we should now be at zero electrical angle)
        float off = bottom_angle;
        encoder->set_offset(off);
        // set_phase_voltages({limits.align_voltage, 0}, 0);
        // delay(700);
        // encoder->update();
        // float off = encoder->get_angle();
        encoder->set_offset(off);
        telemetry->send({GENERAL, {off}});

        set_phase_voltages(0, 0, 0);

        // write to config

        t_config.configured = true;
        t_config.magic_number = 0xDEADBEEF;
        t_config.offset = off;
        config_manager.write_config(t_config);

        // // Calculate CW or CCW encoder configuration
        // float diff = top_angle - zero_angle;
        //
        // if (diff > 0) {
        //     encoder->_direction = SENSOR_DIRECTION::CW;
        // }
        // else if (diff < 0) {
        //     encoder->_direction = SENSOR_DIRECTION::CCW;
        // }
        // else {
        //     Serial.println("DID NOT DETECT MOVEMENT");
        // }
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
        //
        //
        //
        //
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

    bool Maxwell::supply_voltage_watchdog() {
        adc->read();
        input_voltage = adc->get_supply_voltage();
        if (input_voltage < min_voltage) {
            driver_active = false;
        }
        else {
            driver_active = true;
        }
        return driver_active;
    }

    float Maxwell::find_resistance(float voltage) {
        constexpr int buf_size = 128;

        float d_axis_voltage = voltage;
        float d_axis_currents[buf_size];
        set_phase_voltages({d_axis_voltage, 0}, 0);
        delay(20); // Let value settle
        for (int i=0; i<buf_size; i++) {
            encoder->update();
            adc->read();
            PhaseCurrents c = adc->get_phase_currents();
            ab_struct ab = clarke_transform(c);
            dq_struct dq = park_transform(ab, encoder->get_angle());
            d_axis_currents[i] = dq.d;

            delay(1);
        }
        set_phase_voltages(0, 0, 0); // Turn off voltage
        // Average the current readings to reduce noise
        float average_d_axis_current = mean(d_axis_currents, buf_size);
        float resistance = d_axis_voltage / average_d_axis_current;
        Rs = resistance;
        telemetry->DEBUG("Resistance: (mOhm) " + String(resistance*1e3, 7));
        return resistance;
    }

    float Maxwell::find_flux_linkage() {
        // Spin the motor at a constant speed, observe the results
        // CAN be calculated from KV rating
        float fl = (1/kv_rating) * 60 / (2 * PI); // use kv_rating from
        flux_linkage = fl;
        return fl;
    }

    float Maxwell::find_inductance(float v_d, float v_q) {
        // Perform motor calibration sequence to find Ld and Lq
        constexpr int buf_size = 128*5;
        float d_currs[buf_size];
        float q_currs[buf_size];
        float vels[buf_size];

        for (int i=0; i<500; i++) {
            set_phase_voltages({v_d, v_q});
            encoder->update();
            update_observer(encoder->get_angle(), micros());
            delay(1);
        }
        // delay(200); // Allow values to settle
        for (int i=0; i<buf_size; i++) {
            set_phase_voltages({v_d, v_q});
            float angle = encoder->get_angle();
            update_observer(angle, micros());
            vels[i] = velocity;
            adc->read();
            PhaseCurrents c = adc->get_phase_currents();
            ab_struct ab = clarke_transform(c);
            dq_struct dq = park_transform(ab, angle);
            // Store d and q axis currents for later processing
            d_currs[i] = dq.d;
            q_currs[i] = dq.q;
            delay(1);
        }
        set_phase_voltages(0, 0, 0); // Turn off voltage
        float Id = mean(d_currs, buf_size);
        float Iq = mean(q_currs, buf_size);
        float average_omega = mean(vels, buf_size) * config.pole_pairs; // convert to electrical velocity

        Ld = (v_q - Rs*Iq - flux_linkage*average_omega) / (average_omega*Id);
        Lq = (Rs*Id - v_d) / (average_omega*Iq);
        telemetry->DEBUG("Average Omega: " + String(average_omega, 7)); delay(10);
        telemetry->DEBUG("Ld: (uH) " + String(Ld*1e6, 7)); delay(10);
        telemetry->DEBUG("Lq: (uH) " + String(Lq*1e6, 7));
    }

    float Maxwell::estimate_flux_angle(uint32_t current_time_us) {
        OBSERVER_TYPE observer = OBSERVER_TYPE::GRIFFO_OBSERVER;
        float Ts = (current_time_us - prev_flux_estimator_micros) * 1e-6;
        if (Ts > 1e-6) {
            switch (observer) {
                case OBSERVER_TYPE::ORTEGA_OBSERVER: {
                    flux_full_elec_rotations = floor(absolute_flux_angle * config.pole_pairs / _2PI);
                    prev_flux_angle = absolute_flux_angle * config.pole_pairs - flux_full_elec_rotations * _2PI; // Get the angle within the current electrical rotation
                    // High omega_c at low speed to stop drift, low at high speed for phase accuracy
                    float omega_c = 50.0f;

                    float gamma = 100.0;
                    if (velocity > 25.0) {
                        gamma = velocity * 4.0f;
                        gamma = constrain(gamma, 100.0, 2000.0);
                    }
                    float L_ia = L * foc.ab_meas.alpha;
                    float L_ib = L * foc.ab_meas.beta;

                    float err = SQ(flux_linkage) - (SQ(Psi_alpha - L_ia) + SQ(Psi_beta - L_ib));

                    if (err > 0.0) err = 0.0;

                    float p_a_dot =  (foc.command_ab.alpha - Rs*foc.ab_meas.alpha) + (gamma/2)*(Psi_alpha - L_ia)*err;
                    float p_b_dot =  (foc.command_ab.beta  - Rs*foc.ab_meas.beta ) + (gamma/2)*(Psi_beta  - L_ib)*err;

                    Psi_alpha += (p_a_dot - omega_c * Psi_alpha) * Ts;
                    Psi_beta  += (p_b_dot - omega_c * Psi_beta ) * Ts;

                    raw_flux_angle = atan2(Psi_beta, Psi_alpha);  // Get the angle of the flux vector
                    // if (isnan(raw_flux_angle) | isinf(raw_flux_angle)) { // If the angle is not a number (or inf), use the previous angle
                    //     raw_flux_angle = prev_flux_angle;
                    // }
                    raw_flux_angle = raw_flux_angle - floor(raw_flux_angle / _2PI) * _2PI; // normalize angle to [0, 2*PI]

                    float d_angle = raw_flux_angle - prev_flux_angle;
                    if (abs(d_angle) > 0.5f*_2PI) { // wrap around
                        flux_full_elec_rotations += (d_angle > 0) ? -1 : 1;
                    }
                    // absolute_bemf_angle = (bemf_full_mech_rotations * _2PI) + ((bemf_full_elec_rotations * _2PI) + raw_bemf_angle) / config.pole_pairs;
                    absolute_flux_angle = ((_2PI * flux_full_elec_rotations) + raw_flux_angle) / config.pole_pairs;
                    prev_flux_estimator_micros = current_time_us;
                    break;
                }
                case OBSERVER_TYPE::GRIFFO_OBSERVER: {
                    // Extract elec_rotations, and prev_bemf angle from absolute_bemf_angle (so absolute_bemf_angle can be used as update mechanism)
                    flux_full_elec_rotations = floor(absolute_flux_angle * config.pole_pairs / _2PI);
                    prev_flux_angle = absolute_flux_angle * config.pole_pairs - flux_full_elec_rotations * _2PI; // Get the angle within the current electrical rotation
                    // prev_flux_angle = raw_flux_angle;
                    float omega_c = 50.0;
                    float observer_gain = 20.0;

                    // Integrate the voltage vector to get flux vector
                    Psi_alpha += (foc.command_ab.alpha - Rs*foc.ab_meas.alpha - omega_c*Psi_alpha) * Ts * observer_gain;
                    Psi_beta  += (foc.command_ab.beta -  Rs*foc.ab_meas.beta  - omega_c*Psi_beta ) * Ts * observer_gain;

                    // raw_flux_angle = atan2(Psi_beta, Psi_alpha); // Get the angle of the flux vector

                    raw_flux_angle = atan2(Psi_beta - L*foc.ab_meas.beta, Psi_alpha - L*foc.ab_meas.alpha); // Get the angle of the flux vector
                    raw_flux_angle = raw_flux_angle - floor(raw_flux_angle / _2PI) * _2PI; // normalize angle to [0, 2*PI]

                    float d_angle = raw_flux_angle - prev_flux_angle;
                    if (abs(d_angle) > 0.5f*_2PI) { // wrap around
                        flux_full_elec_rotations += (d_angle > 0) ? -1 : 1;
                    }
                    // absolute_bemf_angle = (bemf_full_mech_rotations * _2PI) + ((bemf_full_elec_rotations * _2PI) + raw_bemf_angle) / config.pole_pairs;
                    absolute_flux_angle = ((_2PI * flux_full_elec_rotations) + raw_flux_angle) / config.pole_pairs;
                    prev_flux_estimator_micros = current_time_us;
                    break;
                }
            }
        }
        return absolute_flux_angle;
    }

    void Maxwell::update_observer(float angle_meas, uint32_t current_time_us) {
        float Ts = (current_time_us - prev_observer_micros) * 1e-6;
        if (Ts > 1e-6) { // only update if there's a reasonable amount of time passed.
            // A tracking observer
            // // This is proportional to 2 times the error
            // float two_error = (_sin(angle_meas) * _cos(theta_est)) - (_cos(angle_meas) * _sin(theta_est));

            // Lightweight PID:
            float Kp = 100.0;
            float Ki = 1000.0;
            float error = angle_meas - theta_est;

            integral_observer += error * Ki * Ts;
            velocity = Kp * error + integral_observer;

            deriv_velocity = (angle_meas - prev_angle) / Ts;
            prev_angle = angle_meas;

            deriv_velocity = foc.velocity_lpf->update(deriv_velocity, current_time_us);

            // // Select deriv_velocity
            // velocity = deriv_velocity;
            // angle_observer.set_setpoint(angle_meas);
            // float omega = angle_observer.update(theta_est);

            // Theta est is the integral of the estimated velocity
            theta_est += (velocity * Ts);
            prev_observer_micros = current_time_us; // Update the previous timestamp
        }
    }

    void Maxwell::motor_calibration() {
        init_pwm_3x();

        // Setup components for bldc_control
        driver->enable(true); // Enable driver in 3x mode
        driver->set_pwm_mode(DRV8323::PWM_MODE::PWM_3x);  // Ensure 3x PWM setup

        find_resistance(2.5);
        // find_resistance(4.0);
        delay(500);
        find_inductance(1.5, 1.5);
        delay(100);


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
        switch (config.telemetry_target) {
            case (TELEMETRY_TARGET::TELEMETRY_USB): {
                telemetry = usb_target;
                telemetry->DEBUG("TELEMETRY_TARGET::TELEMETRY_USB");
                break;
            }
            case (TELEMETRY_TARGET::TELEMETRY_CAN): {
                telemetry = can_bus;
                telemetry->DEBUG("TELEMETRY_TARGET::TELEMETRY_CAN");
                break;
            }
            default: break;
        }

        switch (config.command_source) {
            case (COMMAND_SOURCE::PWM): {
                telemetry->DEBUG("COMMAND_SOURCE::PWM");
                command_source = pwm_input;
                // command_source->command_gain = 1.0;
                break;
            }
            case (COMMAND_SOURCE::CAN): {
                    telemetry->DEBUG("COMMAND_SOURCE::CAN");
                command_source = can_bus;
                break;
            }
            default: break;
        }
        delay(10);

    }

    void Maxwell::motor_control() {
        // motor_calibration();

        // Interpret the current config, and call the relevant control loop (e.g. sinusoidal position control, voltage control, current control etc.)
        switch (config.motor_type) {
            case (MOTOR_TYPE::DC): {
                // dc_control();
                break;
            }
            case (MOTOR_TYPE::BLDC): {
                telemetry->DEBUG("Starting BLDC control loop");
                telemetry->DEBUG("Control Mode: " + String(config.control_mode));
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

        float cl_bw = _2PI * 500;
        float dq_kp = 5;  //cl_bw * L; // about 0.02
        float dq_ki = 30; //cl_bw * Rs;

        telemetry->DEBUG("DQ KP: " + String(dq_kp, 7));
        telemetry->DEBUG("DQ KI: " + String(dq_ki, 7));


        // Must not be pointers!
        PIDController d_pid =
                    PIDController(  dq_kp,
                                    dq_ki,
                                    0.0,
                                    0.0,
                                    limits.max_voltage,
                                    limits.max_voltage/20);
        PIDController q_pid =
            PIDController(  dq_kp,
                            dq_ki,
                            0.0,
                            0.0,
                            limits.max_voltage,
                            limits.max_voltage/20);

        PIDController velocity_pid =
            PIDController(  0.01,
                            1.0,
                            0.0,
                            0.0,
                            limits.max_current,
                            limits.max_current);

        PIDController position_pid =
            PIDController(  20,
                            0,
                            0.01,
                            0.0,
                            limits.max_velocity,
                            limits.max_velocity);

        float pos_ref = 0.0;
        float vel_ref = 0.0;
        float torque_reference = 0.0;
        float homing_theta = 0.0;
        float prev_angle = 0.0;
        float angle = 0.0;
        float encoder_angle = 0.0;
        float use_flux_thresh_1 = 50.0;
        float use_flux_thresh_2 = 100.0;
        float v_d_ff = 0.0;
        float v_q_ff = 0.0;
        float v_d_ff_bemf = 0.0;
        bool using_bemf_estimate = false;
        float max_abs_angle = 0.0;
        uint32_t last_pos_vel_update = 0;
        uint32_t start = 0;
        uint32_t end = 0;
        uint32_t prev_millis = millis();
        uint32_t prev_micros = micros();
        float loop_frequency = 1;
        float pos_vel_loop_rate = 2500.0;
        while (true) { // Master control loop
            // For all control cases, the following things are read/measured at the start of the loop:
            uint32_t current_time_ms = millis();
            uint32_t current_time_us = micros();
            float reference = command_source->read();             // Read command from command source
            reference = foc.input_lpf->update(reference, current_time_us);  // Filter command
            reference = motor_direction * reference; // Apply motor direction from config
            //float Ts = (current_time_us - prev_micros) * 1e-6;
            // supply_voltage_watchdog();

            // Pure encoder
            encoder->update();
            angle = encoder->get_angle();
            update_observer(angle, current_time_us);
            // velocity = foc.velocity_lpf->update(velocity, current_time_us);

            // if ((last_pos_vel_update - current_time_us)*1e-6 >= (1/pos_vel_loop_rate))
            { // Position and velocity loop
                last_pos_vel_update = current_time_us;
                // switch control mode to generate correct torque reference for the torque control loop
                switch (config.control_mode) {
                    case (CONTROL_MODE::TORQUE): {
                        // Direct torque control - reference is directly obtained from command source
                        torque_reference = reference *
                            ((config.torque_control_mode== TORQUE_CONTROL_MODE::VOLTAGE) ?
                                limits.max_voltage : limits.max_current);
                        break;
                    };
                    case (CONTROL_MODE::VELOCITY): {
                        vel_ref = reference * limits.max_velocity;
                        velocity_pid.set_setpoint(vel_ref);
                        torque_reference = velocity_pid.update(velocity);
                        break;
                    };
                    case (CONTROL_MODE::POSITION) : {
                        if (pos_homed) {
                            pos_ref = reference * limits.max_position + homed_position_offset + 0.5 * motor_direction;
                        }
                        else {
                            homing_theta -= (motor_direction * 0.003);
                            pos_ref = homing_theta;
                        }

                        position_pid.set_setpoint(pos_ref);
                        vel_ref = position_pid.update(angle);
                        velocity_pid.set_setpoint(vel_ref);
                        torque_reference = velocity_pid.update(velocity);
                        break;
                    };
                    default: break;
                }
            }

            // All control modes use torque control
            switch (config.torque_control_mode) {
                case (TORQUE_CONTROL_MODE::VOLTAGE): {
                    // TORQUE CONTROL, with TORQUE CONTROL MODE

                    float U_q = torque_reference; // In voltage control mode, the torque reference is directly the q-axis voltage
                    set_phase_voltages({0, U_q}, angle*POLE_PAIRS_6374);
                    adc->read();

                    // // Measure current and transform
                    foc.phase_current_meas = adc->get_phase_currents();
                    foc.phase_voltage_meas = adc->get_phase_voltages();
                    foc.ab_meas = clarke_transform(foc.phase_current_meas);
                    foc.dq_meas = park_transform(foc.ab_meas, angle);

                    // Filter measured currents
                    foc.dq_meas.d = foc.d_lpf->update(foc.dq_meas.d, current_time_us);
                    foc.dq_meas.q = foc.q_lpf->update(foc.dq_meas.q, current_time_us);

                    break;
                }
                case (TORQUE_CONTROL_MODE::CURRENT): {
                    // Read reference and set q-axis set point
                    float I_q = torque_reference;
                    d_pid.set_setpoint(0.0);
                    q_pid.set_setpoint(I_q);


                    adc->read();

                    // // Measure current and transform
                    foc.phase_current_meas = adc->get_phase_currents();
                    // foc.phase_voltage_meas = adc->get_phase_voltages();
                    foc.ab_meas = clarke_transform(foc.phase_current_meas);
                    foc.dq_meas = park_transform(foc.ab_meas, angle);

                    if (config.control_mode == POSITION and !pos_homed) {
                        if (abs(foc.dq_meas.q) > 5.0) {
                            pos_homed = true;
                            homed_position_offset = angle;
                            telemetry->DEBUG("Position homed! Offset: " + String(homed_position_offset));
                        }
                    }

                    // Filter measured currents
                    foc.dq_meas.d = foc.d_lpf->update(foc.dq_meas.d, current_time_us);
                    foc.dq_meas.q = foc.q_lpf->update(foc.dq_meas.q, current_time_us);


                    // // Update PID controllers
                    foc.command_dq.d = d_pid.update(foc.dq_meas.d);
                    foc.command_dq.q = q_pid.update(foc.dq_meas.q);
                    foc.command_ab = reverse_park_transform(foc.command_dq, angle);

                    // calculate decoupling:
                    float w_e = velocity * config.pole_pairs; // Electrical angular velocity
                    v_d_ff = -w_e * Lq * foc.dq_meas.q; // D axis feedforward voltage
                    v_q_ff = w_e * Ld * foc.dq_meas.d;  // Q axis feedforward voltage
                    v_d_ff_bemf = w_e * flux_linkage; // BEMF decoupling - can add in ??
                    // Add feedforward
                    // foc.command_dq.d += v_d_ff;
                    // foc.command_dq.q += v_q_ff;



                    // // Generate voltage command from PID output
                    set_phase_voltages(foc.command_dq, angle*POLE_PAIRS_6374);

                    break;
                }
                default: break;
            }

            uint32_t end_time_us = micros();
            loop_frequency = (loop_frequency * 4 + 1/((end_time_us - current_time_us)*1e-6)) /5;
            if (current_time_ms - prev_millis >= 30) {
                // String a = driver->get_fault_status_1_string() +" / " + driver->get_fault_status_1_string();
                telemetry->send({TELEMETRY_PACKET_TYPE::COMMAND, {reference}});
                // telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {pos_ref, angle, encoder->theta_est, adc->get_bemf_angle()}});
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_POSITION, {pos_ref, encoder->absolute_angle}});
                telemetry->send({TELEMETRY_PACKET_TYPE::BUS_VOLTAGE, {input_voltage}});
                telemetry->send({TELEMETRY_PACKET_TYPE::ROTOR_VELOCITY, {vel_ref, velocity, deriv_velocity}});
                telemetry->send({TELEMETRY_PACKET_TYPE::PHASE_CURRENTS, {static_cast<float>(foc.phase_current_meas.current_a),
                                                                                static_cast<float>(foc.phase_current_meas.current_b),
                                                                                static_cast<float>(foc.phase_current_meas.current_c)}});
                telemetry->send({TELEMETRY_PACKET_TYPE::COMMAND_VOLTAGES, {static_cast<float>(foc.command_dq.d),
                                                                                        static_cast<float>(foc.command_dq.q)}});
                // telemetry->send({TELEMETRY_PACKET_TYPE::ALPHA_BETA_CURRENTS, {static_cast<float>(foc.ab_meas.alpha),
                //                                                                     static_cast<float>(foc.ab_meas.beta),
                //                                                                         static_cast<float>(foc.command_ab.alpha),
                //                                                                             static_cast<float>(foc.command_ab.beta)}});
                // telemetry->send({TELEMETRY_PACKET_TYPE::ALPHA_BETA_CURRENTS,
                //         {static_cast<float>(Psi_alpha),
                //             static_cast<float>(Psi_beta)}});
                telemetry->send({TELEMETRY_PACKET_TYPE::ALPHA_BETA_CURRENTS, {v_d_ff, v_q_ff, v_d_ff_bemf}});
                telemetry->send({TELEMETRY_PACKET_TYPE::DQ_CURRENTS, {static_cast<float>(foc.dq_meas.d),
                                                                                                static_cast<float>(foc.dq_meas.q),
                                                                                                    0.0,
                                                                                                    torque_reference}});
                telemetry->send({TELEMETRY_PACKET_TYPE::GENERAL, {loop_frequency}});
                // telemetry->DEBUG(end-start);
                // telemetry->DEBUG(String(angle) + " / " + String(absolute_flux_angle) + " / " + String(encoder->absolute_angle));
                // telemetry->DEBUG(digitalRead(DRV8323_MOTOR_FAULT_PIN));
                // telemetry->DEBUG(String(driver->get_fault_status_1_string()) + " / " + String(driver->get_fault_status_2_string()));
                // telemetry->DEBUG(static_cast<String>(encoder->errors.parity_error_cnt) + " / " +
                //                         static_cast<String>(encoder->errors.error_flag_cnt) + " / " +
                //                         static_cast<String>(encoder->errors.error_flag) + "/" +
                //                         static_cast<String>(encoder->errors.delta_jump_error_cnt) + " / " +
                //                             static_cast<String>(encoder->errors.magnitude));
                prev_millis = current_time_ms;
            }
            prev_micros = current_time_us;

        } // End of master control loop
    }






}
