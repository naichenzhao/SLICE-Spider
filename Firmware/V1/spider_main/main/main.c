#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include "motor.h"
#include "encoder.h"
#include "switch.h" 
#include "I2C.h"


uint8_t motor_pos[8];
uint8_t encoder_pos[8];

void init_position();

void app_main(void) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_task_wdt_deinit();
    
    motor_init();
    encoder_init();
    i2c_init();
    for(int i = 0; i < 4; i++) {
        motor_pos[i] = 0;
    }
    
    init_position();

    printf("---------- Finished Initialization ---------- \n\n");

    

    while (1) {
        // Refresh all the sensor readings
        read_switches();
        get_counts(encoder_pos);
        update_positions(motor_pos);
        esp_WR_RD(motor_pos + 4, encoder_pos + 4);

        // Run anything in the main loop
        printf("encoders: %d %d %d %d\n", (int)encoder_pos[0], (int)encoder_pos[1], (int)encoder_pos[2], (int)encoder_pos[3]);

        // Some delay stuff
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
    }
}

void init_position() {
    home_motor_enc(0);
    goto_pos(0, 65);
    home_motor_enc(1);
    goto_pos(1, 65);
    home_motor_enc(2);
    goto_pos(2, 65);
    home_motor_enc(3);
    goto_pos(3, 65);
}
