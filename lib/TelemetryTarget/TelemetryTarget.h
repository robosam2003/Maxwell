//
// Created by robos on 22/12/2025.
//

#ifndef MAXWELL_TELEMETRYTARGET_H
#define MAXWELL_TELEMETRYTARGET_H
#include <cstdint>
#include <vector>


enum TELEMETRY_PACKET_TYPE : uint8_t {
    GENERAL,
    ROTOR_POSITION,
    ROTOR_VELOCITY,
    PHASE_CURRENTS,
    ALPHA_BETA_CURRENTS,
    DQ_CURRENTS,
    BUS_VOLTAGE,
    COMMAND_VOLTAGES,
    COMMAND,
};

struct telemetry_packet {
    TELEMETRY_PACKET_TYPE type;
    std::vector<float> data;
};

class TelemetryTarget {

public:
    virtual void send(const telemetry_packet& packet);
    virtual ~TelemetryTarget() = default;
};


#endif //MAXWELL_TELEMETRYTARGET_H