//
// Created by robos on 20/12/2025.
//

#include "AS5048A.h"


uint16_t AS5048A::read_reg(REGISTER regAddress) {
    uint16_t result = 0;
    uint16_t word = (READ_BYTE << 8) | regAddress; // add read bit
    // Add parity bit using xor (even parity)
    uint16_t parityBit = 0;
    for (int i = 0; i < 15; i++) {
        parityBit ^= (word >> i) & 0x1;
    }
    // first bit is parity bit
    word |= parityBit << 15;
    _spi.beginTransaction(_settings); // Begin the SPI transaction
    digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
    result = _spi.transfer16(word); // Send the address byte
    digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
    _spi.endTransaction(); // End the SPI transaction
    result = result & 0x3FFF; // Mask the 2 MSB bits - data is 14 bits
    return result;
}

void AS5048A::write_reg(REGISTER regAddress, uint16_t value) {
    uint16_t word = (WRITE_BYTE << 8) | regAddress; // add write bit
    // Add parity bit using xor (even parity)
    uint16_t parityBit = 0;
    for (int i = 0; i < 15; i++) {
        parityBit ^= (word >> i) & 0x1;
    }
    // first bit is parity bit
    word |= parityBit << 15;
    _spi.beginTransaction(_settings); // Begin the SPI transaction
    digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
    _spi.transfer16(word); // Send the address byte
    _spi.transfer16(value); // Send the data byte
    digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
    _spi.endTransaction(); // End the SPI transaction
}

AS5048A::AS5048A(byte CS, SPIClass& spi, uint32_t spiFreq) {
    _CS = CS;
    _spi = spi;
    // Set up SPI settings - SPI MODE 1 because data is captured on the falling edge of the clock
    // and propagated on the rising edge - https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
    _settings = SPISettings(spiFreq, MSBFIRST, SPI_MODE1);
    _direction = SENSOR_DIRECTION::CCW; // Default - may be changed by the init sequence.
    // Set up CS pin
    pinMode(_CS, OUTPUT);
    digitalWrite(_CS, HIGH);


    // Timer
    timer = new HardwareTimer(TIM3);
    timer->setMode(1, TIMER_OUTPUT_DISABLED);
    timer->setOverflow(84000000, HERTZ_FORMAT); // 84 MHz clock
    timer->resume();

    // Begin the SPI bus.
    _spi.begin();

}

void AS5048A::update() {
    prev_absolute_angle = absolute_angle; // Store the angle from previous update
    uint32_t current_time = micros();
    // int current_time_counts = static_cast<int>(timer->getCount()); // in microseconds
    // if (current_time_counts < prev_time_counts) {
    //     // Timer overflowed - adjust previous time accordingly
    //     prev_time_counts -= 0xFFFF;
    // }
    // double Ts = (current_time_counts - prev_time_counts) * 11.9e-9; // Convert to seconds


    uint16_t angle = read_reg(REGISTER::ANGLE);
    float angle_raw_val = (static_cast<float>(angle) / 16384.0f) * _2PI; // Convert to radians
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
        // velocity = (absolute_angle - prev_absolute_angle) / Ts;
        // if (_direction == CCW) {
        //     velocity = -velocity;
        // }
        // prev_time_counts = current_time_counts;
        if (calibrated) {
            velocity_estimate = get_velocity_estimate(absolute_angle, Ts);
            velocity = velocity_estimate; // Use velocity estimate for velocity output.
        }
        prev_micros = current_time;
    }
    else {
        velocity = velocity; // Return previous velocity if updates are too close together
    }
    // (pos_filtered) ? absolute_angle = pos_lpf->update(absolute_angle, current_time) : 0;
}

float AS5048A::get_angle() {
    return absolute_angle;
    // // update();
    // if (_direction == CCW) {
    //     return -absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
    // }
    // return absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
}

float AS5048A::get_velocity() {
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

float AS5048A::get_velocity_estimate(float angle_meas, float Ts) {
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

void AS5048A::set_offset(float angle) {
    calibrated = true;
    offset = angle;
}
