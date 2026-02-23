//
// Created by SamScott on 16/02/2026.
//

#ifndef MAXWELL_CANBUS_H
#define MAXWELL_CANBUS_H


#include "STM32_CAN.h"
#include "stm32f4xx_hal_can.h"
#include "../../include/pin_definitions.h"
#include "CommandSource.h"
#include "TelemetryTarget.h"


class CANBus : public CommandSource, public TelemetryTarget { // Multiple Inheritance
private:
    STM32_CAN* can;



public:
    CANBus(uint32_t baud_rate);

    float read() override;

    void send(const telemetry_packet &packet) override;
};


#endif //MAXWELL_CANBUS_H