//
// Created by SamScott on 16/02/2026.
//

#include "CANBus.h"

union {
    char uint8_t[4];
    float f;
} float_to_bytes;

CANBus::CANBus(uint32_t baud_rate) {
    // Implement can bus
    can = new STM32_CAN(CAN1_RX_PIN, CAN1_TX_PIN);
    can->begin();

    can->setBaudRate(baud_rate);
}

float CANBus::read() {
    // Needs implementing
}

void CANBus::send(const telemetry_packet &packet) {
    // Needs implementing

    CAN_message_t message;
    message.id = 0x01;
    message.len = packet.data.size() * sizeof(float) + 1; // +1 for the packet type
    message.buf[0] = packet.type;
    for (size_t i = 0; i < packet.data.size(); i++) {
        float_to_bytes.f = packet.data[i];
        for (int j = 1; j < 5; j++) {
            message.buf[i*4 + j] = float_to_bytes.uint8_t[j-1];
        }
    }
    can->write(message);
    // if (message.len > 8) { // CAN messages can only carry up to 8 bytes of data - Split into multiple messages if necessary
    //     // Handle splitting the packet into multiple messages if needed
    // } else {
    //     memccpy(message.buf, packet.data.data(), 0, message.len);
    //     // Send the CAN message using your CAN library's send function
    //     can->write(message);
    // }
    digitalToggle(GREEN_LED_PIN);
}
