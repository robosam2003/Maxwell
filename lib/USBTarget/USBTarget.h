//
// Created by robos on 22/12/2025.
//

#ifndef MAXWELL_USBTARGET_H
#define MAXWELL_USBTARGET_H
#include <USBSerial.h>
#include <WSerial.h>

#include "TelemetryTarget.h"

#define USB_SERIAL_BUFFER_SIZE 64

class USBTarget : public TelemetryTarget {
private:
    USBSerial serial = Serial;
    uint8_t buffer[512] = {};
    uint8_t buffer_pointer = 0;
    int baud_rate = 921600;
public:
    USBTarget();
    void send(const telemetry_packet& packet) override;
    static void generate_checksum(uint8_t* data, size_t length);

};


#endif //MAXWELL_USBTARGET_H