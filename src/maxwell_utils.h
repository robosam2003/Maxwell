//
// Created by SamScott on 02/03/2026.
//

#ifndef MAXWELL_MAXWELL_UTILS_H
#define MAXWELL_MAXWELL_UTILS_H

#include "../lib/RCFilter/RCFilter.h"
#include "../lib/pid_controller/pid_controller.h"


struct ab_struct {
    double alpha;
    double beta;
};

struct dq_struct {
    double d;
    double q;
};

struct PhaseCurrents {
    double current_a;
    double current_b;
    double current_c;
};

struct PhaseVoltages {
    float voltage_a;
    float voltage_b;
    float voltage_c;
};

struct Currents {
    PhaseCurrents phase_currents;
    ab_struct alpha_beta;
    dq_struct dq;
};

struct FOC { // Everything that needs initialised must be a pointer
    RCFilter* q_lpf;
    RCFilter* d_lpf;
    RCFilter* velocity_lpf;
    RCFilter* input_lpf;
    PIDController* d_pid;
    PIDController* q_pid;
    PIDController* position_pid;
    PIDController* velocity_pid;

    PhaseCurrents phase_current_meas;
    ab_struct ab_meas;
    dq_struct dq_meas;

    dq_struct command_dq;
    ab_struct command_ab;
    PhaseVoltages command_voltages;  // Are these voltages or currents?

    PhaseVoltages phase_voltage_meas;
};

struct pwm_3x_struct {
    HardwareTimer *TIM_A; HardwareTimer *TIM_B; HardwareTimer *TIM_C;
    uint8_t channel_a; uint8_t channel_b; uint8_t channel_c;
    uint32_t FREQ;
    uint32_t RESOLUTION;
    uint32_t MAX_COMPARE_VALUE; // 2^RESOLUTION - 1
};

struct limits_struct {
    float max_voltage;
    float max_current;
    float align_voltage;
    float max_velocity; // in electrical radians per second
    float max_position; // in radians
};

struct params_struct {
    float offset;
    int csa_gain;
    uint32_t pwm_frequency;
};


// Clarke and Park transforms
inline ab_struct clarke_transform(const PhaseCurrents &currents) { // currents to alpha-beta
    // float I_alpha = currents.current_a;
    // float I_beta = (currents.current_a  + 2*currents.current_b) / _SQRT3;

    // Altenative Amplitude-invariant version
    float I_alpha = currents.current_a;
    float I_beta = (currents.current_b - currents.current_c) / _SQRT3;

    ab_struct ab = {I_alpha, I_beta};
    return ab;
}


inline dq_struct park_transform(const ab_struct &ab_vec, float electrical_theta) { // alpha-beta to dq
    // the park transform
    // float electrical_theta = theta * POLE_PAIRS_6374; // Assuming we're aligned with the encoder!
    float d = ab_vec.alpha * _cos(electrical_theta)  + ab_vec.beta * _sin(electrical_theta);
    float q = -ab_vec.alpha * _sin(electrical_theta) + ab_vec.beta * _cos(electrical_theta);
    dq_struct dq = {d, q};
    return dq;
}

inline ab_struct reverse_park_transform(const dq_struct &dq_vec, float electrical_theta) {  // dq to alpha-beta
    // float electrical_theta = theta * POLE_PAIRS_6374; // Assuming we're aligned with the encoder!
    double c = _cos(electrical_theta);
    double s = _sin(electrical_theta);
    float alpha = dq_vec.d * c - dq_vec.q * s;
    float beta  = dq_vec.d * s + dq_vec.q * c;
    ab_struct alpha_beta = {alpha, beta};
    return alpha_beta;
}

inline PhaseVoltages reverse_clarke_transform(const ab_struct &ab_vec) {
    // PhaseCurrents currents = {(2/3)*ab_vec.alpha,
    //                     (-1/3)*ab_vec.alpha + (sqrt(3)/3)*ab_vec.beta,
    //                     (-1/3)*ab_vec.alpha - (sqrt(3)/3)*ab_vec.beta};
    PhaseVoltages voltages;
    voltages.voltage_a = ab_vec.alpha;
    voltages.voltage_b = (-ab_vec.alpha + M_SQRT3*ab_vec.beta) / 2;
    voltages.voltage_c = (-ab_vec.alpha - M_SQRT3*ab_vec.beta) / 2;
    return voltages;
}


inline float sum(float* arr, int size) {
    float s = 0;
    for (int i=0; i<size; i++) {
        s += arr[i];
    }
    return s;
}

inline float mean(float* arr, int size) {
    return sum(arr, size) / size;
}

inline float norm(float* arr, int size) { // norm of a vector
    float s = 0;
    for (int i=0; i<size; i++) {
        s += arr[i]*arr[i];
    }
    s = sqrt(s);
    return s;
}



#define SQ(x)       ((x)*(x))









#endif //MAXWELL_MAXWELL_UTILS_H