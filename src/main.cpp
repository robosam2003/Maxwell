#include <Arduino.h> 
#include "pin_definitions.h"
#include "Maxwell.h"

void JumpToBootloader(void) {
    void (*SysMemBootJump)(void);

    /**
     * Step: Set system memory address.
     *
     *       For STM32F429, system memory is on 0x1FFF 0000
     *       For other families, check AN2606 document table 110 with descriptions of memory addresses
     */
    volatile uint32_t addr = 0x1FFF0000;

    /**
     * Step: Disable RCC, set it to default (after reset) settings
     *       Internal clock, no PLL, etc.
     */
#if defined(USE_HAL_DRIVER)
    HAL_RCC_DeInit();
#endif /* defined(USE_HAL_DRIVER) */
#if defined(USE_STDPERIPH_DRIVER)
    RCC_DeInit();
#endif /* defined(USE_STDPERIPH_DRIVER) */

    /**
     * Step: Disable systick timer and reset it to default values
     */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /**
     * Step: Disable all interrupts
     */
    __disable_irq();

    /**
     * Step: Remap system memory to address 0x0000 0000 in address space
     *       For each family registers may be different.
     *       Check reference manual for each family.
     *
     *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
     *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
     *       For others, check family reference manual
     */
    //Remap by hand... {
#if defined(STM32F4)
    SYSCFG->MEMRMP = 0x01;
#endif
#if defined(STM32F0)
    SYSCFG->CFGR1 = 0x01;
#endif
    //} ...or if you use HAL drivers
    //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();    //Call HAL macro to do this for you

    /**
     * Step: Set jump memory location for system memory
     *       Use address with 4 bytes offset which specifies jump location where program starts
     */
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

    /**
     * Step: Set main stack pointer.
     *       This step must be done last otherwise local variables in this function
     *       don't have proper value since stack pointer is located on different position
     *
     *       Set direct address location which specifies stack pointer in SRAM location
     */
    __set_MSP(*(uint32_t *)addr);

    /**
     * Step: Actually call our function to jump to set location
     *       This will start system memory execution
     */
    SysMemBootJump();

    /**
     * Step: Connect USB<->UART converter to dedicated USART pins and test
     *       and test with bootloader works with STM32 Flash Loader Demonstrator software
     */
}

#define BOOTLOADER_MAGIC_NUMBER "4242"


void start_up_noise() {

}

void setup() {
// write your initialization code here
    SerialUSB.begin(9600);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(DRV8323_HI_A_PIN, OUTPUT);
    pinMode(DRV8323_HI_B_PIN, OUTPUT);
    pinMode(DRV8323_HI_C_PIN, OUTPUT);
    pinMode(DRV8323_LO_A_PIN, OUTPUT);
    pinMode(DRV8323_LO_B_PIN, OUTPUT);
    pinMode(DRV8323_LO_C_PIN, OUTPUT);

    pinMode(DRV8323_GATE_EN_PIN, OUTPUT);
    pinMode(DRV8323_DRIVE_CAL_PIN, OUTPUT);

    digitalWrite(DRV8323_DRIVE_CAL_PIN, LOW);


}


void sense_print_voltages() {
    uint32_t v_a = analogRead(V_SENSE_A_PIN);
    uint32_t v_b = analogRead(V_SENSE_B_PIN);
    uint32_t v_c = analogRead(V_SENSE_C_PIN);
    uint32_t v_supply = analogRead(V_SUPPLY_SENSE_PIN);

    // Convert to voltages:
    float v_a_v = v_a * SENSE_CONVERSION_FACTOR;
    float v_b_v = v_b * SENSE_CONVERSION_FACTOR;
    float v_c_v = v_c * SENSE_CONVERSION_FACTOR;
    float v_supply_v = v_supply * SENSE_CONVERSION_FACTOR;

    // Print the voltages:
    SerialUSB.print(v_a_v); SerialUSB.print(" ");
    SerialUSB.print(v_b_v); SerialUSB.print(" ");
    SerialUSB.print(v_c_v); SerialUSB.print(" ");
    SerialUSB.println(v_supply_v);
}

void sense_print_currents() {
    uint32_t i_a = analogRead(DRV8323_CURR_SENSE_A_PIN);
    uint32_t i_b = analogRead(DRV8323_CURR_SENSE_B_PIN);
    uint32_t i_c = analogRead(DRV8323_CURR_SENSE_C_PIN);

    // Convert to voltages:
    double i_a_v = (double)i_a * SENSE_CONVERSION_FACTOR;
    double i_b_v = (double)i_b * SENSE_CONVERSION_FACTOR;
    double i_c_v = (double)i_c * SENSE_CONVERSION_FACTOR;

    // Print the voltages:
    SerialUSB.print(i_a_v); SerialUSB.print(" ");
    SerialUSB.print(i_b_v); SerialUSB.print(" ");
    SerialUSB.print(i_c_v); SerialUSB.print(" ");
    SerialUSB.println();
}



uint8_t six_step_commutation_states[6][6] = {  // AH, AL, BH, BL, CH, CL
        {1, 0, 0, 0, 0, 1},
        {0, 0, 1, 0, 0, 1},
        {0, 1, 1, 0, 0, 0},
        {0, 1, 0, 0, 1, 0},
        {0, 0, 0, 1, 1, 0},
        {1, 0, 0, 1, 0, 0},
};



void six_step_commutation_loop(int del, int level) {


    for (int i = 0; i < 6; i++) {
        analogWrite(DRV8323_HI_A_PIN, level * six_step_commutation_states[i][0]);
        analogWrite(DRV8323_LO_A_PIN, level * six_step_commutation_states[i][1]);
        analogWrite(DRV8323_HI_B_PIN, level * six_step_commutation_states[i][2]);
        analogWrite(DRV8323_LO_B_PIN, level * six_step_commutation_states[i][3]);
        analogWrite(DRV8323_HI_C_PIN, level * six_step_commutation_states[i][4]);
        analogWrite(DRV8323_LO_C_PIN, level * six_step_commutation_states[i][5]);
        sense_print_currents();
        delay(del);
    }


}

//void ramp_run_ramp() {
//    int level = 25;
//    for (int i=0; i<255; i++) {
//        six_step_commutation_loop(200 - i, level);
//    }
//    for (int i = 255; i > 0; i--) {
//        six_step_commutation_loop(200 - i, level);
//    }
//
//}


void loop() {
    digitalWrite(DRV8323_GATE_EN_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);

    for (int i = 0; i < 10; i++) {
        six_step_commutation_loop(15-i, 25);
    }
    for (int i = 0; i < 50; i++) {
        six_step_commutation_loop(2, 25);
    }
    digitalWrite(DRV8323_GATE_EN_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(2000);

}
