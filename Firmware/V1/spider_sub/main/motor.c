#include "driver/ledc.h"
#include "driver/gpio.h"


#include "motor.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY (4000) // Frequency in Hertz. Set frequency at 4 kHz
#define MAX_SPEED (8192)      // Set duty to 50%. (2 ** 13) * 50% = 4096

/*  Motor 0: UPPER FRONT
    Motor 1: LOWER FRONT
    Motor 2: UPPER BACK
    Motor 3: LOWER BACK
*/

ledc_channel_t CHANNELS[4] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
int PWM_PINS[4] = {GPIO_NUM_0, GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_6};
int DIR_PINS[4] = {GPIO_NUM_1, GPIO_NUM_3, GPIO_NUM_5, GPIO_NUM_7};

int DIR[4] = {1, 1, 1, -1};

void init_single_motor(int num) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = CHANNELS[num],
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_PINS[num],
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    gpio_reset_pin(DIR_PINS[num]);
    gpio_set_direction(DIR_PINS[num], GPIO_MODE_OUTPUT);
}

void motor_init(void) {
    init_single_motor(0);
    init_single_motor(1);
    init_single_motor(2);
    init_single_motor(3);
}

void set_motor(int num, float speed) {
    if (num >= 4) {
        printf("Sending to other ESP\n");

    } else {
        speed = DIR[num] * speed;
        speed = speed > MAX_SPEED ? MAX_SPEED : speed;
        speed = speed < -MAX_SPEED ? -MAX_SPEED : speed;

        if (speed > 0) {
            gpio_set_level(DIR_PINS[num], 0);
            ledc_set_duty(LEDC_MODE, CHANNELS[num], (int)speed);
        } else {
            gpio_set_level(DIR_PINS[num], 1);
            ledc_set_duty(LEDC_MODE, CHANNELS[num], MAX_SPEED + (int)speed);
        }
        ledc_update_duty(LEDC_MODE, CHANNELS[num]);
    }
}
