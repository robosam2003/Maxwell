#include <Arduino.h>
#include "pin_definitions.h"
#include "Maxwell.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"



void drive_motor(void *pvParameters) {
    Maxwell::Maxwell maxwell;
    maxwell.setup();

    for (;;) {
        maxwell.drive_hall_velocity(10, 7000); // 2 seconds
        vTaskDelay(2000);
    }
}

void setup() {

    xTaskCreate(
        drive_motor,
        "drive motor",
        1000,
        NULL,
        1,
        NULL
    );




    vTaskStartScheduler();

}

void loop() {

}

