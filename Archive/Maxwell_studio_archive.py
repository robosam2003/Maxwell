@dataclass
class State:
    time:                float  = 0.0
    rotor_sector:        float  = 0.0
    electrical_velocity: float  = 0.0
    mechanical_velocity: float  = 0.0
    current_a:           float  = 0.0
    current_b:           float  = 0.0
    current_c:           float  = 0.0
    d_current:           float  = 0.0
    q_current:           float  = 0.0
    alpha_current:       float  = 0.0
    beta_current:        float  = 0.0
    total_current:       float  = 0.0
    voltage_a:           float  = 0.0
    voltage_b:           float  = 0.0
    voltage_c:           float  = 0.0
    input_voltage:       float  = 0.0
    temperature:         float  = 0.0
    pwm_duty:            float  = 0.0
    pid_output:          float  = 0.0
    fault_1:             str    = ''
    fault_2:             str    = ''