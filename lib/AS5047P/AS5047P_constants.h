//
// Created by robos on 10/07/2024.
//

#ifndef MAXWELL_AS5047P_CONSTANTS_H
#define MAXWELL_AS5047P_CONSTANTS_H

namespace AS5047P {

enum REGISTER {
    NOOP        = 0x0000,
    ERRFL       = 0x0001,
    PROG        = 0x0003,
    DIAAGC      = 0x3FFC,
    MAG         = 0x3FFD,
    ANGLEUNCOMP = 0x3FFE,
    ANGLECOM    = 0x3FFF
};

}

#endif //MAXWELL_AS5047P_CONSTANTS_H
