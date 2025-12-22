//
// Created by robos on 22/12/2025.
//

#include "USBTarget.h"

void USBTarget::initialize() {
    serial.begin(baud_rate);
}

void USBTarget::send(const telemetry_packet &packet) {
    size_t num_data_points = packet.data.size(); // size() returns the number of elements in the vector
    size_t byte_length = num_data_points * sizeof(float);
    if (serial.availableForWrite() < (1 + byte_length + 1)) {
        // Not enough space in write buffer, skip sending
        return;
    }
    // Simple protocol: [type (1 byte)] [data (length bytes)]
    buffer[0] = packet.type;
    memcpy(&buffer[1], packet.data.data(), byte_length); // Copy data into buffer
    buffer[1 + byte_length] = '\n';
    serial.write(buffer, 1 + byte_length + 1); // Send type + data + newline
}
