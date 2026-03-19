//
// Created by robos on 20/12/2025.
//

#include "AS5600.h"

uint8_t AS5600::read_reg(REGISTER regAddress) const {
    _pipe->beginTransmission(I2C_ADDRESS);
    _pipe->write(regAddress);
    _pipe->endTransmission(false); // repeated start — keeps bus held for the following requestFrom
    _pipe->requestFrom(I2C_ADDRESS, (uint8_t)1);
    return _pipe->available() ? _pipe->read() : 0;
}

uint16_t AS5600::read_reg16(REGISTER regAddress) const {
    // Burst read: set register pointer once, request 2 bytes atomically.
    // The AS5600 auto-increments its register pointer, so HIGH and LOW bytes
    // come from the same internal angle snapshot — preventing MSB/LSB tearing.
    _pipe->beginTransmission(I2C_ADDRESS);
    _pipe->write(regAddress);
    _pipe->endTransmission(false); // repeated start
    _pipe->requestFrom(I2C_ADDRESS, (uint8_t)2);
    uint16_t result = 0;
    if (_pipe->available()) result  = (uint16_t)_pipe->read() << 8;
    if (_pipe->available()) result |= (uint16_t)_pipe->read();
    return result;
}

void AS5600::write_reg(REGISTER regAddress, uint8_t data) {
    _pipe->beginTransmission(I2C_ADDRESS);
    _pipe->write(regAddress);
    _pipe->write(data);
    _pipe->endTransmission() ; // 0 means success
}

AS5600::AS5600(uint8_t SDA, uint8_t SCL, uint32_t freq) {
    _direction = SENSOR_DIRECTION::CCW; // Default - may be changed by the init sequence.

    _pipe = new TwoWire(SDA, SCL);
    _pipe->setClock(freq);
    _pipe->begin();


    // Timer
    // timer = new HardwareTimer(TIM3);
    // timer->setMode(1, TIMER_OUTPUT_DISABLED);
    // timer->setOverflow(84000000, HERTZ_FORMAT); // 84 MHz clock
    // timer->resume();

}

void AS5600::update() {
    prev_absolute_angle = absolute_angle; // Store the angle from previous update
    uint32_t current_time = micros();
    // int current_time_counts = static_cast<int>(timer->getCount()); // in microseconds
    // if (current_time_counts < prev_time_counts) {
    //     // Timer overflowed - adjust previous time accordingly
    //     prev_time_counts -= 0xFFFF;
    // }
    // double Ts = (current_time_counts - prev_time_counts) * 11.9e-9; // Convert to seconds

    uint16_t angle = read_reg16(REGISTER::RAW_ANGLE_H); // Atomic burst read — both bytes from the same sample
    // uint16_t angle = read_regs(REGISTER::RAW_ANGLE_H); // Read the raw angle from the sensor (0 to 4095)

    float angle_raw_val = (static_cast<float>(angle) / 4096.0f) * _2PI; // Convert to radians
    float d_angle = angle_raw_val - prev_raw_angle;
    if (abs(d_angle) > 0.5f*_2PI) { // This relies on the update() method being called frequently enough
        full_rotations += (d_angle > 0) ? -1 : 1;
    }
    prev_raw_angle = angle_raw_val;
    absolute_angle = (static_cast<float>(full_rotations) * _2PI) + prev_raw_angle;
    if (_direction == CCW) {
        absolute_angle = -absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
    }
    else {
        absolute_angle = absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
    }


    // // Calculate velocity here
    const float Ts = (current_time - prev_micros) * 1e-6;
    if (Ts > 100e-6) { // 100 microseconds - avoid calculating velocity if updates are too close together, which can cause noise
        velocity = (absolute_angle - prev_absolute_angle) / Ts;
        if (_direction == CCW) {
            velocity = -velocity;
        }
        // prev_time_counts = current_time_counts;
        // if (calibrated) {
        //     velocity_estimate = get_velocity_estimate(absolute_angle, Ts);
        //     velocity = velocity_estimate; // Use velocity estimate for velocity output.
        // }
        prev_micros = current_time;
    }
    else {
        velocity = velocity; // Return previous velocity if updates are too close together
    }
    // (pos_filtered) ? absolute_angle = pos_lpf->update(absolute_angle, current_time) : 0;
}

float AS5600::get_angle() {
    return absolute_angle;
    // // update();
    // if (_direction == CCW) {
    //     return -absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
    // }
    // return absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
}

float AS5600::get_velocity() {
    // // int current_time_counts = static_cast<int>(timer->getCount()); // in microseconds
    // // if (current_time_counts < prev_time_counts) {
    // //     // Timer overflowed - adjust previous time accordingly
    // //     prev_time_counts -= 0xFFFF;
    // // }
    // // 84 MHz timer -> 11.9 ns per count
    // uint32_t current_time = micros();
    //
    // // float Ts = (current_time_counts - prev_time_counts) * 11.9e-9; // Convert to seconds
    // const float Ts = (current_time - prev_micros) * 1e-6; // Convert to seconds
    // // if (Ts < 1e-6) { // 1 microseconds
    // //     return velocity;
    // // }
    // velocity = (absolute_angle - prev_absolute_angle) / Ts;
    // if (_direction == CCW) {
    //     velocity = -velocity; // Adjust for direction
    // }
    // // prev_time_counts = current_time_counts;
    // // prev_micros = current_time;
    return velocity;
}

float AS5600::get_velocity_estimate(float angle_meas, float Ts) {
    // A tracking observer
    // // This is proportional to 2 times the error
    // float two_error = (_sin(angle_meas) * _cos(theta_est)) - (_cos(angle_meas) * _sin(theta_est));

    // Lightweight PID:
    float Kp = 100.0;
    float Ki = 100.0;
    float error = angle_meas - theta_est;

    integral_observer += error * Ki * Ts;
    float omega = Kp * error + integral_observer;

    // angle_observer.set_setpoint(angle_meas);
    // float omega = angle_observer.update(theta_est);

    // Theta est is the integral of the estimated velocity
    theta_est += (omega * Ts);
    return omega;
}

void AS5600::set_offset(float angle) {
    calibrated = true;
    offset = angle;
}
