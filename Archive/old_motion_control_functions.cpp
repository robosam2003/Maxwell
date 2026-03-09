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
                            3.0,
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