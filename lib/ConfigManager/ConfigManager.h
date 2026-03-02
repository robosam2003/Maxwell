//
// Created by SamScott on 02/03/2026.
//

#ifndef MAXWELL_CONFIGMANAGER_H
#define MAXWELL_CONFIGMANAGER_H

#include "Arduino.h"
// #include "../../src/maxwell_utils.h"
// #include "../../src/maxwell_config.h"

struct test_config {
    bool configured;
    uint32_t magic_number;
    float offset;
};


class ConfigManager {
#define CONFIG_FLASH_ADDRESS 0x080E0000 // Start of last sector on STM32F405RG (128 KB flash, 16 sectors of 8 KB each)
public:
    ConfigManager();

    // Change test_config to maxwell_config once the config struct is finalised
    void write_config(const test_config& config);

    test_config read_config();
};


#endif //MAXWELL_CONFIGMANAGER_H