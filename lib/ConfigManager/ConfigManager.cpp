//
// Created by SamScott on 02/03/2026.
//

#include "ConfigManager.h"


ConfigManager::ConfigManager() {
    // Constructor
}

void ConfigManager::write_config(const test_config& config) {
    // Unlock the sector before writing
    HAL_FLASH_Unlock();

    // split the config struct into 32-bit words and write to flash
    uint32_t* config_ptr = (uint32_t*)&config;
    for (size_t i = 0; i < sizeof(test_config) / sizeof(uint32_t); i++) {
        // Write consecutive 32-bit words to flash
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CONFIG_FLASH_ADDRESS + i * sizeof(uint32_t), config_ptr[i]);
    }

    // Re-lock
    HAL_FLASH_Lock();
}

test_config ConfigManager::read_config() {
    test_config config;
    uint32_t* config_ptr = (uint32_t*)&config;
    for (size_t i = 0; i < sizeof(test_config) / sizeof(uint32_t); i++) {
        config_ptr[i] = *(uint32_t*)(CONFIG_FLASH_ADDRESS + i * sizeof(uint32_t));
    }
    return config;
}