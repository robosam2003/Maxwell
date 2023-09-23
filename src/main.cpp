#include <Arduino.h> 
#include "pin_definitions.h"
#include "Maxwell.h"
#include "DRV8323.h"
#include "AS5047P.h"

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

SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK
SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK

DRV8323::DRV8323 drv8323(DRV8323_CS_PIN, SPI_1, 1000000);

//SPIProtocol as5047p(AS5047P_CS_PIN, SPI_2, SPISettings(4000000, MSBFIRST, SPI_MODE1), 0x4000, 0x0000, 16);


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

    drv8323.default_configuration(); // set up the gate driver

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
    float G = 20;
    float R_sense = 0.4e-3;
    float VREF = 3.3;
    auto V_SOa = ((double) analogRead(DRV8323_CURR_SENSE_A_PIN)) * CURRENT_SENSE_CONVERSION_FACTOR;
    auto V_SOb = ((double) analogRead(DRV8323_CURR_SENSE_B_PIN)) * CURRENT_SENSE_CONVERSION_FACTOR;
    auto V_SOc = ((double) analogRead(DRV8323_CURR_SENSE_C_PIN)) * CURRENT_SENSE_CONVERSION_FACTOR;

    // Convert to currents:
    double i_a = ((VREF/2 - V_SOa) / (G * R_sense)) - 206;
    double i_b = ((VREF/2 - V_SOb) / (G * R_sense)) - 139;
    double i_c = ((VREF/2 - V_SOc) / (G * R_sense)) - 110;


    // Print the voltages:
    SerialUSB.print(i_a); SerialUSB.print(" ");
    SerialUSB.print(i_b); SerialUSB.print(" ");
    SerialUSB.print(i_c); SerialUSB.print(" ");
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

void delayWhileRunningFunction(int del, void (*func)()) {
    uint32_t start = millis();
    while (millis() - start < del) {
        func();
    }
}


void six_step_commutation_loop(int del, int level) {

}


void low_side_on() {
    digitalWrite(DRV8323_GATE_EN_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(DRV8323_HI_A_PIN, LOW);
    digitalWrite(DRV8323_LO_A_PIN, HIGH);
    digitalWrite(DRV8323_HI_B_PIN, LOW);
    digitalWrite(DRV8323_LO_B_PIN, HIGH);
    digitalWrite(DRV8323_HI_C_PIN, LOW);
    digitalWrite(DRV8323_LO_C_PIN, HIGH);

}




void loop() {
//    digitalWrite(DRV8323_GATE_EN_PIN, HIGH);
//    digitalWrite(GREEN_LED_PIN, HIGH);//
//    for (int i = 0; i < 20; i++) {
//        digitalWrite(GREEN_LED_PIN, HIGH);
//        six_step_commutation_loop(20, 25);
//        digitalWrite(GREEN_LED_PIN, LOW);
//    }
//    digitalWrite(DRV8323_GATE_EN_PIN, LOW);
//    digitalWrite(GREEN_LED_PIN, LOW);
//    delay(2000);

//    uint16_t gate_drive_hs = drv8323.get_gate_drive_hs();
//    SerialUSB.println(gate_drive_hs, BIN);
//    uint16_t ocp_control = drv8323.get_gate_drive_hs();
//    SerialUSB.println(ocp_control, BIN);
//    delay(10);


//    digitalWrite(DRV8323_HI_A_PIN, LOW);
//    digitalWrite(DRV8323_LO_A_PIN, LOW);
//    digitalWrite(DRV8323_HI_B_PIN, HIGH);
//    digitalWrite(DRV8323_LO_B_PIN, LOW);
//    digitalWrite(DRV8323_HI_C_PIN, LOW);
//    digitalWrite(DRV8323_LO_C_PIN, HIGH);
//
//    delay(1000);
//    drv8323.enable(true);

    uint16_t a = drv8323.read_reg(DRV8323::REGISTER::OCP_CONTROL);
    SerialUSB.println(a, BIN);
    delay(100);
}
