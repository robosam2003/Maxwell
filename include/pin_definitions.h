//
// Created by robos on 07/09/2023.
//
#include "config.h"

#ifndef MAXWELL_PIN_DEFINITIONS_H
#define MAXWELL_PIN_DEFINITIONS_H

#ifdef HARDWARE_V1_0

// AS5047P Magnetic Encoder
#define AS5047P_SPI_BUS             SPI2
#define AS5047P_CS_PIN              PB9
#define AS5047P_I_PIN               PC13
#define AS5047P_A_PIN               PC14
#define AS5047P_B_PIN               PC15

// DRV8323SR Gate driver
#define DRV8323_SPI_BUS             SPI1
#define DRV8323_CS_PIN              PB7

#define DRV8323_MOTOR_FAULT_PIN     PC7
#define DRV8323_DRIVE_CAL_PIN       PC8
#define DRV8323_GATE_EN_PIN         PC9

#define DRV8323_HI_A_PIN            PB_1  // Also called INHA
#define DRV8323_HI_B_PIN            PA_3
#define DRV8323_HI_C_PIN            PA_1


#define DRV8323_LO_A_PIN            PB_15
#define DRV8323_LO_B_PIN            PA_2
#define DRV8323_LO_C_PIN            PB_8

#define DRV8323_PWM1X_PIN           DRV8323_HI_A_PIN
#define DRV8323_STATE0_PIN          DRV8323_LO_A_PIN
#define DRV8323_STATE1_PIN          DRV8323_HI_B_PIN
#define DRV8323_STATE2_PIN          DRV8323_LO_B_PIN
#define DRV8323_DIR_PIN             DRV8323_HI_C_PIN
#define DRV8323_BRAKE_PIN           DRV8323_LO_C_PIN

#define DRV8323_CURR_SENSE_A_PIN    PB0
#define DRV8323_CURR_SENSE_B_PIN    PC5
#define DRV8323_CURR_SENSE_C_PIN    PC4

// CAN
#define CAN2_TX_PIN                 PB13
#define CAN2_RX_PIN                 PB12`
#define CAN_S_PIN                   PB14
#define CAN_FAULT_PIN               PC6

// Sense
#define V_SENSE_A_PIN               PC0
#define V_SENSE_B_PIN               PA0
#define V_SENSE_C_PIN               PC1

#define V_SUPPLY_SENSE_PIN          PA4

// Other
#define GREEN_LED_PIN               PB2
#define SOLENOID_EN_PIN             PA10
#define PWM_IN_PIN                  PA8


// Hall sensors
#define HALL_A_PIN                  PB10
#define HALL_B_PIN                  PB11
#define HALL_C_PIN                  PC3
#define HALL_TEMP_PIN               PC2




#endif //HARDWARE_V1_0



#endif //MAXWELL_PIN_DEFINITIONS_H
