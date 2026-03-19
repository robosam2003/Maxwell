//
// Created by robos on 20/12/2025.
//

#ifndef MAXWELL_AS5048A_H
#define MAXWELL_AS5048A_H

#include "PositionSensor.h"
#include <SPI.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal_dma.h"
#include "../../include/config.h"
#include "HardwareTimer.h"
#include "pid_controller.h"
#include "../../include/pin_definitions.h"



class AS5048 : public PositionSensor {
public:
    enum REGISTER : uint16_t {
        CLEAR_ERROR = 0x0001,
        PROGRAMMING_CONTROL = 0x0003,
        ZEROMSB = 0x0016,
        ZEROLSB = 0x0017,
        AGC = 0x3FFD,
        MAG = 0x3FFE,
        ANGLE = 0x3FFF
    };
    enum ERROR: uint8_t {
        NO_ERROR = 0x00,
        FRAMING_ERROR = 0x1,
        COMMAND_INVALID = 0x2,
        PARITY_ERROR = 0x3,
    };
    enum SENSOR_LOCATION : uint8_t {
        INT = 0x00,
        EXT = 0x01
    };

    uint8_t _CS;
    SPIClass _spi;
    SPISettings _settings;

    SPI_HandleTypeDef hspi;
    DMA_HandleTypeDef hdma_spi_rx;
    DMA_HandleTypeDef hdma_spi_tx;

    volatile bool dma_done = false;
    volatile bool dma_error = false;
    volatile uint32_t dma_error_code = 0;

    uint8_t tx_buf[2] = {0, 0};
    uint8_t rx_buf[2] = {0, 0};

    void init_hal_spi_dma();
    // bool spi_txrx16_dma(uint16_t tx_word, uint16_t &rx_word, uint32_t timeout_us = 200);
    static constexpr uint16_t NOP_WORD = 0x0000;
    static constexpr size_t DMA_WORD_COUNT = 64;   // must be even
    static_assert((DMA_WORD_COUNT % 2) == 0, "DMA_WORD_COUNT must be even");

    uint16_t tx_ring[DMA_WORD_COUNT];
    volatile uint16_t rx_ring[DMA_WORD_COUNT];

    volatile bool stream_running = false;
    volatile bool new_angle_ready = false;
    volatile uint16_t latest_angle_word = 0;
    volatile uint32_t dma_angle_count = 0;
    volatile size_t last_parsed_idx = 0;

    uint16_t build_read_word(REGISTER reg);
    void prepare_tx_ring();
    bool start_angle_stream_dma();
    void stop_angle_stream_dma();
    void parse_rx_range(size_t start, size_t count);

    // Callbacks
    void dma_rx_irq_handler();
    void dma_tx_irq_handler();
    void on_spi_dma_complete();
    void on_spi_dma_error(uint32_t err);
    void on_spi_dma_half_complete();





    const uint8_t READ_BYTE = 0x40;
    const uint8_t WRITE_BYTE = 0x00;
    const float MAX_DELTA_ANGLE = 0.1; // Maximum allowed change in angle between updates (radians) - used for error detection


    SENSOR_LOCATION _location;
    int prev_time_counts = 0; // Timestamp of the previous angle reading
    uint32_t prev_micros = 0; // Timestamp of the previous angle reading in microseconds
    HardwareTimer* timer;
    float integral_observer = 0; // Integral term for the tracking observer

    uint16_t read_reg(REGISTER regAddress);
    void write_reg(REGISTER regAddress, uint16_t data);
    float K = 30.0; // Gain for the tracking observer - this can be tuned based on the expected noise and dynamics of the system


    PIDController angle_observer = PIDController(50.0,
                                                 50.0,
                                                0,
                                                0,
                                                100000,
                                                100000); // PID controller for the tracking observer

    AS5048(SENSOR_LOCATION location);

    void update() override;
    // uint16_t read_angle_reg();
    float get_angle() override; // in radians
    // float get_magnitude();
    float get_velocity() override; // in radians per second
    float get_velocity_estimate(float angle_meas, float Ts); // Tracking observer
    // ERROR get_error();
    void set_offset(float angle) override;
};


#endif //MAXWELL_AS5048A_H