#include "pin_definitions.h"
#include "Maxwell.h"
#include "DRV8323.h"
#include "FreeRTOS.h"
#include "task.h"
#include "AS5047P.h"

SPIClass SPI_1(PA7, PA6, PA5); // MOSI, MISO, SCK
SPIClass SPI_2(PC3, PC2, PB10); // MOSI, MISO, SCK

DRV8323::DRV8323 drv8323(DRV8323_CS_PIN, SPI_1, 1000000, DRV8323_GATE_EN_PIN);



double prev_cross_voltage = -1;


void TaskBlink1( void *pvParameters );
void Taskprint( void *pvParameters );



void setup() {
    pinMode(GREEN_LED_PIN, OUTPUT);

    xTaskCreate(TaskBlink1,
        "Blink1",
        128,
        NULL,
        1,
        NULL);

    xTaskCreate(Taskprint,
        "print",
        512,
        NULL,
        2,
        NULL);

    vTaskStartScheduler();
}

void TaskBlink1( void *pvParameters ) {
    for (;;) {
        digitalToggle(GREEN_LED_PIN);
        Serial.printf("Task 1\n");
        vTaskDelay(100);
    }
}

void Taskprint( void *pvParameters ) {
    AS5047P::AS5047P encoder(AS5047P_CS_PIN, SPI_2, 1000000);

    for (;;) {
        // Serial.println("Task 2 ========");
        Serial.printf("Encoder angle: %d\n", encoder.get_angle());
        vTaskDelay(500);
    }
}


void loop() {

}

