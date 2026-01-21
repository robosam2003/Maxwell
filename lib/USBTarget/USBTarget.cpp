//
// Created by robos on 22/12/2025.
//

#include "USBTarget.h"

void USBTarget::initialize() {
    serial.begin(baud_rate);
}

void USBTarget::send(const telemetry_packet &packet) {
    size_t num_data_points = packet.data.size(); // size() returns the number of elements in the vector
    size_t data_byte_length = num_data_points * sizeof(float);
    size_t packet_length = 1 + data_byte_length + 2; // type (1 byte) + data + checksum + newline (2 bytes)
    if (serial.availableForWrite() < packet_length) {
        // Not enough space in write buffer, skip sending
        return;
    }
    // Simple protocol: [type (1 byte)] [data (length bytes)]
    buffer[0] = packet.type;
    memcpy(&buffer[1], packet.data.data(), data_byte_length); // Copy data into buffer
    generate_checksum(buffer, 1+data_byte_length); // Generate checksum for type + data
    buffer[2 + data_byte_length] = '\n'; // Newline as end of packet
    serial.write(buffer, packet_length); // Send type + data + newline
}

void USBTarget::generate_checksum(uint8_t *data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    data[length] = checksum; // Append checksum at the end
}
