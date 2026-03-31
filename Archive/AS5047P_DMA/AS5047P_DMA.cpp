//
// Created by robos on 20/12/2025.
//

#include "AS5047P.h"


uint16_t AS5047P::read_reg(REGISTER regAddress) {
    const uint16_t NOP = 0x0000;
    uint16_t result = 0;
    uint16_t word = (READ_BYTE << 8) | regAddress; // add read bit
    // Add parity bit using xor (even parity)
    uint16_t parityBit = 0;
    for (int i = 0; i < 15; i++) {
        parityBit ^= (word >> i) & 0x1;
    }
    // first bit is parity bit
    word |= parityBit << 15;
    _spi.beginTransaction(_settings); // Begin the SPI transaction
    digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
    result = _spi.transfer16(word); // Send the address byte
    digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
    _spi.endTransaction(); // End the SPI transaction

    // Now send NOP to read result
    _spi.beginTransaction(_settings);
    digitalWrite(_CS, LOW);
    result = _spi.transfer16(NOP);
    digitalWrite(_CS, HIGH);
    _spi.endTransaction();

    return result;
}


void AS5047P::write_reg(REGISTER regAddress, uint16_t value) {
    uint16_t word = (WRITE_BYTE << 8) | regAddress; // add write bit
    // Add parity bit using xor (even parity)
    uint16_t parityBit = 0;
    for (int i = 0; i < 15; i++) {
        parityBit ^= (word >> i) & 0x1;
    }
    // first bit is parity bit
    word |= parityBit << 15;
    _spi.beginTransaction(_settings); // Begin the SPI transaction
    digitalWrite(_CS, LOW); // Pull CS low to start the SPI transaction
    _spi.transfer16(word); // Send the address byte
    _spi.transfer16(value); // Send the data byte
    digitalWrite(_CS, HIGH); // Pull CS high to end the SPI transaction
    _spi.endTransaction(); // End the SPI transaction
}

uint16_t AS5047P::read_angle_reg() {
    uint16_t result = read_reg(REGISTER::ANGLEUNCOMP);
    bool error_flag = false;

    // Calculate Parity
    uint16_t recieve_parity = (result >> 15) & 0x1; // Extract the parity bit from the received word
    uint16_t calculated_parity = 0;
    uint16_t recieved_word = result & 0x7FFF; // Mask out the parity bit to get the original 15 bits
    for (int i = 0; i < 15; i++) {
        calculated_parity ^= (recieved_word >> i) & 0x1;
    }

    // Handle parity error
    if (calculated_parity != recieve_parity) { // Parity error
        error_flag = true;
        errors.parity_error_cnt++;
    }

    // errors.error_flag = get_error();
    if ((result >> 14) & 0x1) {  // Check the error bit (bit 14)
        error_flag = true;
        errors.error_flag_cnt++;
        // errors.error_flag = get_error();
    }
    else {
        errors.error_flag = ERROR::NO_ERROR;
    }

    // if (error_flag) {
    //     // Handle error - return 0;
    //     return 0;
    // }
    // Otherwise, return the actual angle
    result = result & 0x3FFF; // Mask the 2 MSB bits - data is 14 bits
    return result;
}

AS5047P::AS5047P(byte CS, SPIClass& spi, uint32_t spiFreq) {
    // _CS = CS;
    // _spi = spi;
    // Set up SPI settings - SPI MODE 1 because data is captured on the falling edge of the clock
    // and propagated on the rising edge - https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
    // _settings = SPISettings(spiFreq, MSBFIRST, SPI_MODE1);
    _direction = SENSOR_DIRECTION::CCW; // Default - may be changed by the init sequence.
    // Set up CS pin

    // Setup SPI2 pins - MISO2: PC2, MOSI2: PC3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; 	// Enable the clock for port C
    GPIO_C_params.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_C_params.Mode = GPIO_MODE_AF_PP;
    GPIO_C_params.Pull = GPIO_PULLUP;
    GPIO_C_params.Speed = GPIO_SPEED_FREQ_VERY_HIGH;							// Very high speed
    GPIO_C_params.Alternate = GPIO_AF5_SPI2; // Alternate function 5 for SPI2
    HAL_GPIO_Init(GPIOC, &GPIO_C_params);

    // SCK2: PB10, CS2: PB11
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable the clock for port B
    GPIO_B_params.Pin = GPIO_PIN_10;
    GPIO_B_params.Mode = GPIO_MODE_AF_PP;
    GPIO_B_params.Pull = GPIO_PULLUP;
    GPIO_B_params.Speed = GPIO_SPEED_FREQ_VERY_HIGH;							// Very high speed
    GPIO_B_params.Alternate = GPIO_AF5_SPI2; // Alternate function 5 for SPI2
    HAL_GPIO_Init(GPIOB, &GPIO_B_params);

    // Setup CS pin in push pull mode
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable the clock for port B
    GPIO_B_params.Pin = GPIO_PIN_11;
    GPIO_B_params.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_B_params.Pull = GPIO_PULLUP;
    GPIO_B_params.Speed = GPIO_SPEED_FREQ_VERY_HIGH;							// Very high speed
    HAL_GPIO_Init(GPIOB, &GPIO_B_params);

    // pinMode(_CS, OUTPUT);
    // digitalWrite(_CS, HIGH);

    // Setup SPI instance
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // Enable I2S3 (SPI3) peripheral clock
    __HAL_RCC_SPI2_CLK_ENABLE();
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    // hspi2.Init.TIMode = SPI_TIMODE_ENABLE; // Pulsing CS between 16-bit words
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi2);
    // FORCE the hardware to take the pin before enabling
    SET_BIT(hspi2.Instance->CR2, SPI_CR2_SSOE);
    // Ensure Master mode is reinforced (MODF can clear this)
    SET_BIT(hspi2.Instance->CR1, SPI_CR1_MSTR);

    __HAL_SPI_ENABLE(&hspi2); // Ensure peripheral is ON


    // SET_BIT(hspi2.Instance->CR2, SPI_CR2_TXDMAEN); // Enable TX DMA Request
    // SET_BIT(hspi2.Instance->CR2, SPI_CR2_RXDMAEN); // Enable RX DMA Request

    init_dma();

    // 1. Initial trigger to start the chain
    digitalWrite(PB11, LOW);
    HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)spi_tx_buf, (uint8_t*)spi_rx_buf, 1);

    // Timer
    // timer = new HardwareTimer(TIM3);
    // timer->setMode(1, TIMER_OUTPUT_DISABLED);
    // timer->setOverflow(84000000, HERTZ_FORMAT); // 84 MHz clock
    // timer->resume();

    // Begin the SPI bus.
    // _spi.begin();

    errors.parity_error_cnt = 0;
    errors.error_flag_cnt = 0;
    errors.delta_jump_error_cnt = 0;
    // errors.error_flag = get_error(); // Initialise error flag
}

// In AS5047P constructor:
void AS5047P::init_dma() {
    // 1. Enable DMA Clocks
    __HAL_RCC_DMA1_CLK_ENABLE();

    // External encoder is on SPI2:
    // - SPI2_TX: DMA1, Stream 4, Channel 0
    // - SPI2_RX: DMA1, Stream 3, Channel 0
    // 2. Configure TX Stream (Memory to Peripheral)
    // --- TX Stream: Memory -> SPI2 ---
    hdma_spi_tx.Instance = DMA1_Stream4;
    hdma_spi_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_tx.Init.MemInc = DMA_MINC_ENABLE;        // Increment through our 2-word buffer
    hdma_spi_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 16-bit
    hdma_spi_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    // 16-bit
    hdma_spi_tx.Init.Mode = DMA_NORMAL;
    hdma_spi_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi_tx) != HAL_OK) { /* Handle Error */ }

    __HAL_LINKDMA(&hspi2, hdmatx, hdma_spi_tx);

    // --- RX Stream: SPI2 -> Memory ---
    hdma_spi_rx.Instance = DMA1_Stream3;
    hdma_spi_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi_rx.Init.Mode = DMA_NORMAL;
    hdma_spi_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi_rx) != HAL_OK) { /* Handle Error */ }

    __HAL_LINKDMA(&hspi2, hdmarx, hdma_spi_rx);

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0); // RX Stream (Priority 5 is mid-range)
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0); // TX Stream
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    // HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

extern "C" void DMA1_Stream4_IRQHandler(void) {
    // Serial.println("DMA TX Interrupt");
    HAL_DMA_IRQHandler(&hdma_spi_tx);
}

extern "C" void DMA1_Stream3_IRQHandler(void) {
    // Serial.println("DMA RX Interrupt");
    HAL_DMA_IRQHandler(&hdma_spi_rx);
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI2) {
        // 1. Latch data
        digitalWrite(PB11, HIGH);

        // 2. Minimal Delay
        // A delay(1) is 1000us - way too long for FOC.
        // Use a few NOPs or delayMicroseconds(1)
        asm volatile("nop; nop; nop; nop; nop; nop;");
        // delayMicroseconds(1);

        // 3. Start Next Frame
        digitalWrite(PB11, LOW);

        // 4. Request 2 words (Address + NOP)
        // This ensures the sensor shifts out the full 14-bit data packet
        HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t*)spi_tx_buf, (uint8_t*)spi_rx_buf, 1);
    }
}

void AS5047P::update() {
    prev_absolute_angle = absolute_angle; // Store the angle from previous update
    // Decompose absolute angle into full rotations and raw angle for easier handling of position update (from external source e.g. flux observer)
    // full_rotations = floor(absolute_angle / _2PI);
    // prev_raw_angle = (absolute_angle + offset) - (full_rotations * _2PI); // Add offset back to raw angle for easier handling of direction and offset in the angle reading

    // if (_direction == CCW) {
    //     prev_raw_angle = -(absolute_angle + offset) - (full_rotations * _2PI); // Add offset back to raw angle for easier handling of direction and offset in the angle reading
    // }
    // else {
    //     prev_raw_angle = (absolute_angle + offset) - (full_rotations * _2PI); // Add offset back to raw angle for easier handling of direction and offset in the angle reading
    // }


    uint32_t current_time = micros();
    // int current_time_counts = static_cast<int>(timer->getCount()); // in microseconds
    // if (current_time_counts < prev_time_counts) {
    //     // Timer overflowed - adjust previous time accordingly
    //     prev_time_counts -= 0xFFFF;
    // }
    // double Ts = (current_time_counts - prev_time_counts) * 11.9e-9; // Convert to seconds


    // 4. Use the data from the buffer
    uint16_t result = spi_rx_buf[0];

    uint16_t angle = result & 0x3FFF; // Mask bottom 14 bits

    // if (errors.error_flag != ERROR::NO_ERROR) {
    //     // Handle error - for now, just return previous angle and velocity (could extrapolate?)
    //     return;
    // }
    float angle_raw_val = (static_cast<float>(angle) / 16384.0f) * _2PI; // Convert to radians
    float d_angle = angle_raw_val - prev_raw_angle;

    if (abs(d_angle) > 0.5f*_2PI) { // This relies on the update() method being called frequently enough
        full_rotations += (d_angle > 0) ? -1 : 1;
    }
    prev_raw_angle = angle_raw_val;
    absolute_angle = (static_cast<float>(full_rotations) * _2PI) + prev_raw_angle;

    if (_direction == CCW) {
        absolute_angle = -absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
    }
    else {
        absolute_angle = absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
    }

    if (abs(absolute_angle - prev_absolute_angle) > MAX_DELTA_ANGLE) {
        // Handle error - this is likely a glitch in the reading, so ignore it and return previous angle and velocity
        // absolute_angle = prev_absolute_angle;
        // return;
        errors.delta_jump_error_cnt++;
    }

    // // Calculate velocity
    // const float Ts = (current_time - prev_micros) * 1e-6;
    // if (Ts > 100e-6) { // 100 microseconds - avoid calculating velocity if updates are too close together, which can cause noise
    //     // velocity = (absolute_angle - prev_absolute_angle) / Ts;
    //     // if (_direction == CCW) {
    //     //     velocity = -velocity;
    //     // }
    //     // prev_time_counts = current_time_counts;
    //     if (calibrated) {
    //         velocity_estimate = get_velocity_estimate(absolute_angle, Ts);
    //         velocity = velocity_estimate; // Use velocity estimate for velocity output.
    //     }
    //     prev_micros = current_time;
    // }
    // else {
    //     velocity = velocity; // Return previous velocity if updates are too close together
    // }
    // (pos_filtered) ? absolute_angle = pos_lpf->update(absolute_angle, current_time) : 0;

}

float AS5047P::get_angle() {
    return absolute_angle;
    // // update();
    // if (_direction == CCW) {
    //     return -absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
    // }
    // return absolute_angle - offset; // Return the absolute angle in radians, adjusted for direction and offset
}

float AS5047P::get_velocity() {
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

float AS5047P::get_velocity_estimate(float angle_meas, float Ts) {
    // // A tracking observer
    // // // This is proportional to 2 times the error
    // // float two_error = (_sin(angle_meas) * _cos(theta_est)) - (_cos(angle_meas) * _sin(theta_est));
    //
    // // Lightweight PID:
    // float Kp = 100.0;
    // float Ki = 300.0;
    // float error = angle_meas - theta_est;
    //
    // integral_observer += error * Ki * Ts;
    // float omega = Kp * error + integral_observer;
    //
    // // angle_observer.set_setpoint(angle_meas);
    // // float omega = angle_observer.update(theta_est);
    //
    // // Theta est is the integral of the estimated velocity
    // theta_est += (omega * Ts);
    // return omega;
}

AS5047P::ERROR AS5047P::get_error() {
    // Bit 0 is framing error
    // Bit 1 is command invalid
    // Bit 2 is parity error
    uint16_t error_reg = read_reg(REGISTER::ERRFL);
    ERROR error = ERROR::NO_ERROR;
    (error_reg & 0x1) ? error = ERROR::FRAMING_ERROR   : 0;
    (error_reg & 0x2) ? error = ERROR::COMMAND_INVALID : 0;
    (error_reg & 0x4) ? error = ERROR::PARITY_ERROR    : 0;
    return error;
}

void AS5047P::set_offset(float angle) {
    calibrated = true;
    offset = angle;
}
