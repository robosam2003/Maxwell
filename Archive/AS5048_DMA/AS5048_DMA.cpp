//
// Created by robos on 20/12/2025.
//

#include "AS5048.h"

// Single-owner bridge for C HAL callbacks/IRQs
static AS5048* g_as5048_dma_owner = nullptr;


void AS5048::init_hal_spi_dma() {
    // ---- Clocks ----
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // ---- SPI1 pins: PA5=SCK, PA6=MISO, PA7=MOSI (AF5) ----
    GPIO_InitTypeDef gpio = {0};
    if (_location == INT) {
        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
    else {
        gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3; // MISO: PC2, MOSI: PC3
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOC, &gpio);

        gpio.Pin = GPIO_PIN_10; // SCK : PB10
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &gpio);
    }

    // ---- SPI config (AS5048 uses mode 1: CPOL=0, CPHA=2EDGE) ----
    hspi.Instance = (_location == INT) ? SPI1 : SPI2; // SPI2 is external encoder
    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_8BIT; // use 2-byte DMA for each 16-bit frame
    hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi.Init.NSS = SPI_NSS_SOFT;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // tune as needed
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.CRCPolynomial = 7;

    if (HAL_SPI_Init(&hspi) != HAL_OK) {
        // Optional: set a class error flag/log here
        return;
    }

    // ---- DMA config ----
    if (_location == INT) {  // SPI1 on DMA2
        hdma_spi_rx.Instance = DMA2_Stream0;
        hdma_spi_rx.Init.Channel = DMA_CHANNEL_3;
        hdma_spi_tx.Instance = DMA2_Stream3;
        hdma_spi_tx.Init.Channel = DMA_CHANNEL_3;

        HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
        HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    } else {  // SPI2 on DMA1
        hdma_spi_rx.Instance = DMA1_Stream3;
        hdma_spi_rx.Init.Channel = DMA_CHANNEL_0;
        hdma_spi_tx.Instance = DMA1_Stream4;
        hdma_spi_tx.Init.Channel = DMA_CHANNEL_0;

        HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
        HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    }

    // RX DMA (circular, halfword)
    hdma_spi_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_spi_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_spi_rx);
    __HAL_LINKDMA(&hspi, hdmarx, hdma_spi_rx);

    // TX DMA (circular, halfword)
    hdma_spi_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_spi_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_spi_tx);
    __HAL_LINKDMA(&hspi, hdmatx, hdma_spi_tx);

    g_as5048_dma_owner = this;
}

uint16_t AS5048::build_read_word(REGISTER reg) {
    uint16_t word = (READ_BYTE << 8) | reg;
    uint16_t parity = 0;
    for (int i = 0; i < 15; i++) parity ^= (word >> i) & 0x1;
    word |= (parity << 15);
    return word;
}

void AS5048::prepare_tx_ring() {
    const uint16_t rd_angle = build_read_word(REGISTER::ANGLE);
    // Ring pattern: [CMD, NOP, CMD, NOP, ...]
    for (size_t i = 0; i < DMA_WORD_COUNT; i += 2) {
        tx_ring[i] = rd_angle;      // command frame
        tx_ring[i + 1] = NOP_WORD;  // fetch response
    }
}

bool AS5048::start_angle_stream_dma() {
    prepare_tx_ring();
    new_angle_ready = false;
    dma_angle_count = 0;
    last_parsed_idx = 0;

    // Start with CS LOW for first frame
    digitalWrite(_CS, LOW);

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(
        &hspi,
        reinterpret_cast<uint8_t*>(tx_ring),
        reinterpret_cast<uint8_t*>(const_cast<uint16_t*>(rx_ring)),
        DMA_WORD_COUNT
    );
    stream_running = (st == HAL_OK);
    return stream_running;
}

void AS5048::stop_angle_stream_dma() {
    if (!stream_running) return;
    HAL_SPI_DMAStop(&hspi);
    digitalWrite(_CS, HIGH);
    stream_running = false;
}

void AS5048::parse_rx_range(size_t start, size_t count) {
    // Parse pairs: even indices are command echo (discard), odd indices are NOP responses (actual data)
    for (size_t i = start + 1; i < start + count; i += 2) {
        uint16_t raw = rx_ring[i];
        latest_angle_word = raw;
        new_angle_ready = true;
        dma_angle_count++;
    }
}

void AS5048::on_spi_dma_half_complete() {
    // Half-complete: first half of ring is done
    // Before parsing second half: pulse CS HIGH to latch the NOP response
    digitalWrite(_CS, HIGH);
    delayMicroseconds(1);
    digitalWrite(_CS, LOW);

    // Parse the second half (which was just completed)
    parse_rx_range(DMA_WORD_COUNT / 2, DMA_WORD_COUNT / 2);
}

void AS5048::on_spi_dma_complete() {
    // Full completion: second half of ring is done
    // Pulse CS HIGH to latch
    digitalWrite(_CS, HIGH);
    delayMicroseconds(1);
    digitalWrite(_CS, LOW);

    // Parse the first half
    parse_rx_range(0, DMA_WORD_COUNT / 2);
}

void AS5048::on_spi_dma_error(uint32_t err) {
    dma_error = true;
    dma_error_code = err;
    stop_angle_stream_dma();
}

void AS5048::dma_rx_irq_handler() {
    HAL_DMA_IRQHandler(&hdma_spi_rx);
}

void AS5048::dma_tx_irq_handler() {
    HAL_DMA_IRQHandler(&hdma_spi_tx);
}

// C linkage IRQ shims
extern "C" void DMA2_Stream0_IRQHandler(void) {
    if (g_as5048_dma_owner) g_as5048_dma_owner->dma_rx_irq_handler();
}

extern "C" void DMA2_Stream3_IRQHandler(void) {
    if (g_as5048_dma_owner) g_as5048_dma_owner->dma_tx_irq_handler();
}

extern "C" void DMA1_Stream3_IRQHandler(void) {
    if (g_as5048_dma_owner) g_as5048_dma_owner->dma_rx_irq_handler();
}

extern "C" void DMA1_Stream4_IRQHandler(void) {
    if (g_as5048_dma_owner) g_as5048_dma_owner->dma_tx_irq_handler();
}

// HAL weak callback overrides
extern "C" void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
    if (g_as5048_dma_owner && hspi == &g_as5048_dma_owner->hspi) {
        g_as5048_dma_owner->on_spi_dma_half_complete();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (g_as5048_dma_owner && hspi == &g_as5048_dma_owner->hspi) {
        g_as5048_dma_owner->on_spi_dma_complete();
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (g_as5048_dma_owner && hspi == &g_as5048_dma_owner->hspi) {
        g_as5048_dma_owner->on_spi_dma_error(hspi->ErrorCode);
    }
}

// uint16_t AS5048::read_reg(REGISTER regAddress) {
//     const uint16_t NOP = 0x0000;
//     uint16_t result = 0;
//     uint16_t word = (READ_BYTE << 8) | regAddress; // add read bit
//     // Add parity bit using xor (even parity)
//     uint16_t parityBit = 0;
//     for (int i = 0; i < 15; i++) {
//         parityBit ^= (word >> i) & 0x1;
//     }
//     // first bit is parity bit
//     word |= parityBit << 15;
//     _spi.beginTransaction(_settings); // Begin the SPI transaction
//     digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
//     result = _spi.transfer16(word); // Send the command
//     digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
//
//     delayMicroseconds(1); // Short delay between transactions
//
//     // read the response
//     digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
//     result = _spi.transfer16(NOP); // Send the address byte
//     digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
//
//
//     _spi.endTransaction(); // End the SPI transaction
//
//     return result;
// }
//
//
// void AS5048::write_reg(REGISTER regAddress, uint16_t value) {
//     uint16_t word = (WRITE_BYTE << 8) | regAddress; // add write bit
//     // Add parity bit using xor (even parity)
//     uint16_t parityBit = 0;
//     for (int i = 0; i < 15; i++) {
//         parityBit ^= (word >> i) & 0x1;
//     }
//     // first bit is parity bit
//     word |= parityBit << 15;
//     _spi.beginTransaction(_settings); // Begin the SPI transaction
//     digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
//     _spi.transfer16(word); // Send the address byte
//     _spi.transfer16(value); // Send the data byte
//     digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
//     _spi.endTransaction(); // End the SPI transaction
// }
//
// uint16_t AS5048::read_angle_reg() {
//     // errors.magnitude = get_magnitude(); // Update the magnitude error for diagnostics
//
//     uint16_t result = read_reg(REGISTER::ANGLE);
//     bool error_flag = false;
//
//     // Calculate Parity
//     uint16_t recieve_parity = (result >> 15) & 0x1; // Extract the parity bit from the received word
//     uint16_t calculated_parity = 0;
//     uint16_t recieved_word = result & 0x7FFF; // Mask out the parity bit to get the original 15 bits
//     for (int i = 0; i < 15; i++) {
//         calculated_parity ^= (recieved_word >> i) & 0x1;
//     }
//
//     // Handle parity error
//     if (calculated_parity != recieve_parity) { // Parity error
//         error_flag = true;
//         errors.parity_error_cnt++;
//     }
//
//     // errors.error_flag = get_error();
//     if ((result >> 14) & 0x1) {  // Check the error bit (bit 14)
//         error_flag = true;
//         errors.error_flag_cnt++;
//         errors.error_flag = get_error();
//     }
//     else {
//         errors.error_flag = ERROR::NO_ERROR;
//     }
//
//     // if (error_flag) {
//     //     // Handle error - return 0;
//     //     return 0;
//     // }
//     // Otherwise, return the actual angle
//     result = result & 0x3FFF; // Mask the 2 MSB bits - data is 14 bits
//     return result;
// }

// AS5048::AS5048(byte CS, SPIClass& spi, uint32_t spiFreq) {
AS5048::AS5048(SENSOR_LOCATION location) {
    _location = location;
    _CS = (_location == INT) ? AS5048A_CS_PIN : CS_SDA2_RX3_EXTI_PIN;


    // _CS = CS;
    // _spi = spi;
    // // Set up SPI settings - SPI MODE 1 because data is captured on the falling edge of the clock
    // // and propagated on the rising edge - https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
    // _settings = SPISettings(spiFreq, MSBFIRST, SPI_MODE1);
    // _direction = SENSOR_DIRECTION::CCW; // Default - may be changed by the init sequence.
    // // Set up CS pin
    pinMode(_CS, OUTPUT);
    digitalWrite(_CS, HIGH);


    // Timer
    timer = new HardwareTimer(TIM3);
    timer->setMode(1, TIMER_OUTPUT_DISABLED);
    timer->setOverflow(84000000, HERTZ_FORMAT); // 84 MHz clock
    timer->resume();

    // // Begin the SPI bus.
    // _spi.begin();

    errors.parity_error_cnt = 0;
    errors.error_flag_cnt = 0;
    errors.delta_jump_error_cnt = 0;

    init_hal_spi_dma();
    start_angle_stream_dma();

    // errors.magnitude = get_magnitude();
    // errors.error_flag = get_error(); // Initialise error flag
}

void AS5048::update() {
    prev_absolute_angle = absolute_angle;
    uint32_t current_time = micros();

    // If no new sample ready from DMA, use previous angle
    if (!new_angle_ready) {
        return; // skip this update, wait for fresh sample
    }

    // Consume the latest DMA sample
    uint16_t angle = latest_angle_word;
    new_angle_ready = false;

    // ---- Parity check ----
    uint16_t recieve_parity = (angle >> 15) & 0x1;
    uint16_t calculated_parity = 0;
    uint16_t recieved_word = angle & 0x7FFF;
    for (int i = 0; i < 15; i++) {
        calculated_parity ^= (recieved_word >> i) & 0x1;
    }

    bool error_flag = false;
    if (calculated_parity != recieve_parity) {
        error_flag = true;
        errors.parity_error_cnt++;
    }

    // ---- Error flag check (bit 14) ----
    if ((angle >> 14) & 0x1) {
        error_flag = true;
        errors.error_flag_cnt++;
        // errors.error_flag = get_error();
    } else {
        errors.error_flag = ERROR::NO_ERROR;
    }

    if (error_flag) {
        return; // Skip this sample if error detected
    }

    // ---- Extract angle data (14 bits) ----
    angle = angle & 0x3FFF;
    float angle_raw_val = (static_cast<float>(angle) / 16384.0f) * _2PI;
    float d_angle = angle_raw_val - prev_raw_angle;

    // ---- Handle multi-turn wrapping ----
    if (abs(d_angle) > 0.5f * _2PI) {
        full_rotations += (d_angle > 0) ? -1 : 1;
    }
    prev_raw_angle = angle_raw_val;
    absolute_angle = (static_cast<float>(full_rotations) * _2PI) + prev_raw_angle;

    // ---- Apply offset and direction ----
    if (_direction == CCW) {
        absolute_angle = -absolute_angle - offset;
    } else {
        absolute_angle = absolute_angle - offset;
    }

    // ---- Delta check ----
    if (abs(absolute_angle - prev_absolute_angle) > MAX_DELTA_ANGLE) {
        errors.delta_jump_error_cnt++;
    }

    // ---- Velocity estimation ----
    const float Ts = (current_time - prev_micros) * 1e-6;
    if (Ts > 100e-6) {  // only update if enough time has passed
        if (calibrated) {
            velocity_estimate = get_velocity_estimate(absolute_angle, Ts);
            velocity = velocity_estimate;
        }
        prev_micros = current_time;
    }
}

//
// void AS5048::update() {
//     prev_absolute_angle = absolute_angle; // Store the angle from previous update
//     uint32_t current_time = micros();
//     // int current_time_counts = static_cast<int>(timer->getCount()); // in microseconds
//     // if (current_time_counts < prev_time_counts) {
//     //     // Timer overflowed - adjust previous time accordingly
//     //     prev_time_counts -= 0xFFFF;
//     // }
//     // double Ts = (current_time_counts - prev_time_counts) * 11.9e-9; // Convert to seconds
//
//
//     uint16_t angle = read_angle_reg();
//     if (errors.error_flag != ERROR::NO_ERROR) {
//         // Handle error - for now, just return previous angle and velocity (could extrapolate?)
//         return;
//     }
//     float angle_raw_val = (static_cast<float>(angle) / 16384.0f) * _2PI; // Convert to radians
//     float d_angle = angle_raw_val - prev_raw_angle;
//
//     if (abs(d_angle) > 0.5f*_2PI) { // This relies on the update() method being called frequently enough
//         full_rotations += (d_angle > 0) ? -1 : 1;
//     }
//     prev_raw_angle = angle_raw_val;
//     absolute_angle = (static_cast<float>(full_rotations) * _2PI) + prev_raw_angle;
//
//     if (_direction == CCW) {
//         absolute_angle = -absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
//     }
//     else {
//         absolute_angle = absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
//     }
//
//     if (abs(absolute_angle - prev_absolute_angle) > MAX_DELTA_ANGLE) {
//         // Handle error - this is likely a glitch in the reading, so ignore it and return previous angle and velocity
//         // absolute_angle = prev_absolute_angle;
//         // return;
//         errors.delta_jump_error_cnt++;
//     }
//
//     // // Calculate velocity
//     const float Ts = (current_time - prev_micros) * 1e-6;
//     if (Ts > 100e-6) { // 100 microseconds - avoid calculating velocity if updates are too close together, which can cause noise
//         // velocity = (absolute_angle - prev_absolute_angle) / Ts;
//         // if (_direction == CCW) {
//         //     velocity = -velocity;
//         // }
//         // prev_time_counts = current_time_counts;
//         if (calibrated) {
//             velocity_estimate = get_velocity_estimate(absolute_angle, Ts);
//             velocity = velocity_estimate; // Use velocity estimate for velocity output.
//         }
//         prev_micros = current_time;
//     }
//     else {
//         velocity = velocity; // Return previous velocity if updates are too close together
//     }
//     // (pos_filtered) ? absolute_angle = pos_lpf->update(absolute_angle, current_time) : 0;
//
// }

float AS5048::get_angle() {
    return absolute_angle;
    // // update();
    // if (_direction == CCW) {
    //     return -absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
    // }
    // return absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
}

// float AS5048::get_magnitude() {
//     uint16_t mag_reg = read_reg(REGISTER::MAG);
//     return static_cast<float>(mag_reg & 0x3FFF); // mask off top two bits
// }

float AS5048::get_velocity() {
    // // int current_time_counts = static_cast<int>(timer->getCount()); // in microseconds
    // // if (current_time_counts < prev_time_counts) {
    // //     // Timer overflowed - adjust previous time accordingly
    // //     prev_time_counts -= 0xFFFF;
    // // }
    // // 84 MHz timer -> 11.9 ns per count
    // uint32_t current_time = micros();
    //
    // // float Ts = (current_time_counts - prev_time_counts) * 11.9e-9; // Convert to seconds
    // const float Ts = (current_time - prev_micros) * 1e-6; // Convert to seconds
    // // if (Ts < 1e-6) { // 1 microseconds
    // //     return velocity;
    // // }
    // velocity = (absolute_angle - prev_absolute_angle) / Ts;
    // if (_direction == CCW) {
    //     velocity = -velocity; // Adjust for direction
    // }
    // // prev_time_counts = current_time_counts;
    // // prev_micros = current_time;
    return velocity;
}

float AS5048::get_velocity_estimate(float angle_meas, float Ts) {
    // A tracking observer
    // // This is proportional to 2 times the error
    // float two_error = (_sin(angle_meas) * _cos(theta_est)) - (_cos(angle_meas) * _sin(theta_est));

    // Lightweight PID:
    float Kp = 100.0;
    float Ki = 300.0;
    float error = angle_meas - theta_est;

    integral_observer += error * Ki * Ts;
    float omega = Kp * error + integral_observer;

    // angle_observer.set_setpoint(angle_meas);
    // float omega = angle_observer.update(theta_est);

    // Theta est is the integral of the estimated velocity
    theta_est += (omega * Ts);
    return omega;
}

// AS5048::ERROR AS5048::get_error() {
//     // Bit 0 is framing error
//     // Bit 1 is command invalid
//     // Bit 2 is parity error
//     uint16_t error_reg = read_reg(REGISTER::CLEAR_ERROR);
//     ERROR error = ERROR::NO_ERROR;
//     (error_reg & 0x1) ? error = ERROR::FRAMING_ERROR   : 0;
//     (error_reg & 0x2) ? error = ERROR::COMMAND_INVALID : 0;
//     (error_reg & 0x4) ? error = ERROR::PARITY_ERROR    : 0;
//     return error;
// }

void AS5048::set_offset(float angle) {
    calibrated = true;
    offset = angle;
}
