//
// Created by robos on 20/09/2023.
//

#ifndef MAXWELL_DRV8323_CONSTANTS_H
#define MAXWELL_DRV8323_CONSTANTS_H

namespace DRV8323 {

enum REGISTER {
    FAULT_STATUS_1 = 0x00,
    VGS_STATUS_2 = 0x01,
    DRIVER_CONTROL = 0x02,
    GATE_DRIVE_HS = 0x03,
    GATE_DRIVE_LS = 0x04,
    OCP_CONTROL = 0x05,
    CSA_CONTROL = 0x06
};

enum FAULT1 {
    FAULT =     0b10000000000,
    VDS_OCP =   0b01000000000,
    GDF =       0b00100000000,
    UVLO =      0b00010000000,
    OTSD =      0b00001000000,
    VDS_HA =    0b00000100000,
    VDS_LA =    0b00000010000,
    VDS_HB =    0b00000001000,
    VDS_LB =    0b00000000100,
    VDS_HC =    0b00000000010,
    VDS_LC =    0b00000000001,
};

enum VGSFAULT2 {
    SA_OC =     0b10000000000,
    SB_OC =     0b01000000000,
    SC_OC =     0b00100000000,
    OTW =       0b00010000000,
    CPUV =      0b00001000000,
    VGS_HA =    0b00000100000,
    VGS_LA =    0b00000010000,
    VGS_HB =    0b00000001000,
    VGS_LB =    0b00000000100,
    VGS_HC =    0b00000000010,
    VGS_LC =    0b00000000001
};

enum PWM_MODE {
    PWM_6x =            0b00,
    PWM_3x =            0b01,
    PWM_1x =            0b10,
    PWM_INDEPENDENT =   0b11
};

enum CSA_GAIN {
    GAIN_5_V_V =   0b00,
    GAIN_10_V_V =  0b01,
    GAIN_20_V_V =  0b10,
    GAIN_40_V_V =  0b11
};

inline uint8_t csa_gain_to_int[4] = {5, 10, 20, 40};



enum DIRECTION {
    FORWARD =   0b0,
    REVERSE =   0b1
};



}

#endif //MAXWELL_DRV8323_CONSTANTS_H
