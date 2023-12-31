#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include "motor.h"
#include "encoder.h"
#include "switch.h"

void app_main(void)
{
    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_task_wdt_deinit();
    motor_init();
    encoder_init();
    switch_init();

    set_motor(0, 0);
    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);

    for(int i = 0; i < 4; i++) {
        home_motor_sw(i);
    }

    while (1) {
        printf("%d, %d, %d, %d\n", get_count(0), get_count(1), get_count(2), get_count(3));
        // printf("%x %x %x %x %x %x %x %x\n", read_switch(0), read_switch(1), read_switch(2), read_switch(3), read_switch(4), read_switch(5), read_switch(6), read_switch(7));
        vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}



