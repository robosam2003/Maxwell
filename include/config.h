//
// Created by robos on 07/09/2023.
//

#ifndef MAXWELL_CONFIG_H
#define MAXWELL_CONFIG_H


#define HARDWARE_V1_0
#define SENSE_CONVERSION_FACTOR ((3.3 / 1024) * 19.0)
#define CURRENT_SENSE_CONVERSION_FACTOR ((3.3 / 1024))



// ============ ARCHIVE ============
//extern "C" void SystemClock_Config(void)
//{
//    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//    /** Configure the main internal regulator output voltage
//    */
//    __HAL_RCC_PWR_CLK_ENABLE();
//    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//    /** Initializes the RCC Oscillators according to the specified parameters
//    * in the RCC_OscInitTypeDef structure.
//    */
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//    RCC_OscInitStruct.PLL.PLLM = 8;
//    RCC_OscInitStruct.PLL.PLLN = 72;
//    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//    RCC_OscInitStruct.PLL.PLLQ = 3;
//
//    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
//    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//    {
//        Error_Handler();
//    }
//
//    /** Initializes the CPU, AHB and APB buses clocks
//    */
//    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//    {
//        Error_Handler();
//    }
//}

//void JumpToBootloader(void) {
//    void (*SysMemBootJump)(void);
//
//    /**
//     * Step: Set system memory address.
//     *
//     *       For STM32F429, system memory is on 0x1FFF 0000
//     *       For other families, check AN2606 document table 110 with descriptions of memory addresses
//     */
//    volatile uint32_t addr = 0x1FFF0000;
//
//    /**
//     * Step: Disable RCC, set it to default (after reset) settings
//     *       Internal clock, no PLL, etc.
//     */
//#if defined(USE_HAL_DRIVER)
//    HAL_RCC_DeInit();
//#endif /* defined(USE_HAL_DRIVER) */
//#if defined(USE_STDPERIPH_DRIVER)
//    RCC_DeInit();
//#endif /* defined(USE_STDPERIPH_DRIVER) */
//
//    /**
//     * Step: Disable systick timer and reset it to default values
//     */
//    SysTick->CTRL = 0;
//    SysTick->LOAD = 0;
//    SysTick->VAL = 0;
//
//    /**
//     * Step: Disable all interrupts
//     */
//    __disable_irq();
//
//    /**
//     * Step: Remap system memory to address 0x0000 0000 in address space
//     *       For each family registers may be different.
//     *       Check reference manual for each family.
//     *
//     *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
//     *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
//     *       For others, check family reference manual
//     */
//    //Remap by hand... {
//#if defined(STM32F4)
//    SYSCFG->MEMRMP = 0x01;
//#endif
//#if defined(STM32F0)
//    SYSCFG->CFGR1 = 0x01;
//#endif
//    //} ...or if you use HAL drivers
//    //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();    //Call HAL macro to do this for you
//
//    /**
//     * Step: Set jump memory location for system memory
//     *       Use address with 4 bytes offset which specifies jump location where program starts
//     */
//    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
//
//    /**
//     * Step: Set main stack pointer.
//     *       This step must be done last otherwise local variables in this function
//     *       don't have proper value since stack pointer is located on different position
//     *
//     *       Set direct address location which specifies stack pointer in SRAM location
//     */
//    __set_MSP(*(uint32_t *)addr);
//
//    /**
//     * Step: Actually call our function to jump to set location
//     *       This will start system memory execution
//     */
//    SysMemBootJump();
//
//    /**
//     * Step: Connect USB<->UART converter to dedicated USART pins and test
//     *       and test with bootloader works with STM32 Flash Loader Demonstrator software
//     */
//}
//
//
//
//#define BOOTLOADER_MAGIC_NUMBER "4242"

#endif //MAXWELL_CONFIG_H
