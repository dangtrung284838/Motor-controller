#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define MOTOR_EN    14   // GPIO pin to activate the motor
#define MOTOR_IN1   12   // GPIO pin to control the motor direction
#define MOTOR_IN2   13   // GPIO pin to control the motor direction

#define LEDC_HS_TIMER          LEDC_TIMER_0          // Select timer for LEDC
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE  // High-speed mode for LEDC

void motor_control(int speed, int direction) {
    gpio_set_level(MOTOR_IN1, direction);            // Set motor rotation direction
    gpio_set_level(MOTOR_IN2, !direction);           // Set motor rotation direction (inverted)
    ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, speed);  // Set motor duty cycle
    ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);      // Update duty cycle
}

void init(void) {
    // Configure GPIO
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << MOTOR_EN) | (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&io_conf);

    // Initialize LEDC
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_HS_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t pwm_conf = {
        .gpio_num = MOTOR_EN,
        .speed_mode = LEDC_HS_MODE,
        .channel = LEDC_HS_CH0_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_HS_TIMER,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&pwm_conf);
}

void app_main(void) {
    // Initialize devices and configuration
    init();

    // Stop the motor initially
    gpio_set_level(MOTOR_EN, 0);  // Turn off power to the motor

    int direction = 1;  // Initially, the control direction is forward

    while (1) {
        // Toggle the control direction of the motor after each cycle
        direction = !direction;

        for (int duty_cycle = 0; duty_cycle <= MAX_SPEED; duty_cycle += 500)
        {
            motor_control(duty_cycle, 1); // Control the motor with increasing speed
        }

        // // Run the motor with the new direction
        // motor_control(2000, direction);

        // Wait for a period of time before performing the next cycle
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 2 seconds
    }
}
