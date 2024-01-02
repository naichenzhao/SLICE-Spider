#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include "motor.h"
#include "encoder.h"
#include "switch.h" 
#include "I2C.h"

uint8_t data_rd[16];

void app_main(void) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_task_wdt_deinit();
    


    // motor_init();
    // encoder_init();

    i2c_init();
    printf("---------- Finished Initialization ---------- \n\n");

    // for(int i = 0; i < 4; i++) {
    //     home_motor_sw(i);
    // }
    uint8_t count = 0;

    while (1) {
        // printf("%d, %d, %d, %d\n", get_count(0), get_count(1), get_count(2), get_count(3));
        // printf("%x %x %x %x %x %x %x %x\n", read_switch(0), read_switch(1), read_switch(2), read_switch(3), read_switch(4), read_switch(5), read_switch(6), read_switch(7));
        printf("count: %d, recieved_data: %d %d %d %d\n", (int)count, (int)data_rd[0], (int)data_rd[1], (int)data_rd[2], (int)data_rd[3]);
        esp_WR_RD(count, count + 1, count - 1, count + 2, data_rd);
        count++;

        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        
    }
}



