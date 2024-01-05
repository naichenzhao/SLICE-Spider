#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "motor.h"
#include "encoder.h"

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
int PWM_PINS[4] = {GPIO_NUM_3, GPIO_NUM_0, GPIO_NUM_4, GPIO_NUM_6};
int DIR_PINS[4] = {GPIO_NUM_2, GPIO_NUM_1, GPIO_NUM_5, GPIO_NUM_7};

int DIR[4] = {1, -1, -1, 1};

uint8_t motor_positions[4] = {0, 0, 0, 0};

float Kp = 40;
float Ki = 1;
float Kd = 6;


float MAX_SAT = 700;

uint64_t last_time[4] = {0, 0, 0, 0};
float error_i[4] = {0, 0, 0, 0};
uint8_t last_pos[4] = {0, 0, 0, 0};

// ---------------------------------
// Setup Functions
// ---------------------------------

    void
    init_single_motor(int num)
{
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
    printf("Motor Setup done\n");
}

// ---------------------------------
// Movement Functions
// ---------------------------------

void update_positions(uint8_t* pos) {
    motor_positions[0] = pos[0];
    motor_positions[1] = pos[1];
    motor_positions[2] = pos[2];
    motor_positions[3] = pos[3];
}

void set_motor(int num, float speed) {
    if (num >= 4) {
        printf("Invalid number for this ESPP\n");
        return ;
    }

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

void move_pos(int num, uint8_t curr_pos) {
    if (num >= 4) {
        printf("Invalid number for this ESPP\n");
        return ;
    }

    // get current state
    uint64_t curr_time = esp_timer_get_time();
    int dt = (int) (curr_time - last_time[num]);

    // get errors
    float error_p = (float) (motor_positions[num] - curr_pos);
    float error_d =  ((float) (curr_pos - last_pos[num])) / dt;

    if ((error_p + error_i[num]) > MAX_SAT) {
        error_i[num] = MAX_SAT;
    } else if ((error_p + error_i[num]) < -MAX_SAT) {
        error_i[num] = -MAX_SAT;
    } else {
        error_i[num] = error_i[num] + error_p;
    }
    

    // calculate final output
    float pid_out = Kp * error_p + Ki * error_i[num] + Kd * error_d;
    printf("motor speed: %f\n", pid_out);

    // update state
    last_time[num] = curr_time;
    last_pos[num] = curr_pos;

    set_motor(num, pid_out);
}


void goto_pos(int num, int target) {
    motor_positions[num] = target;
    uint8_t current = get_angle(num);
    while (motor_positions[num] - current != 0) {
        move_pos(num, current);
        current = get_count(num);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    set_motor(num, 0);
}







// ---------------------------------
// Homing Functions
// ---------------------------------

void home_motor_enc(int num) {
    printf("Homing motor: %d\n", num);

    int last_count = 360;
    int counter = 0;
    while (counter < 40) {
        if ((last_count - get_count(num)) == 0) {
            counter++;
        } else {
            counter = 0;
        }
        set_motor(num, -6000);
        last_count = get_count(num);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    set_motor(num, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    reset_encoder(num);
    

    printf("Finished: %d\n", num);
}