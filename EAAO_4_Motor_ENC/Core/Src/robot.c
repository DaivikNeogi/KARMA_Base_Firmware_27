#include "robot.h"
#include "main.h"
#include "encoder.h"
#include "motor.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

static uint32_t s_last_tick_50ms;
static char txbuf[128];
static char rx_line_buffer[32];
static uint8_t rx_line_idx = 0;
static int16_t current_duty_percent = 0;

void robot_init(void)
{
    // 1. Encoders are on TIM4, TIM3, and TIM2
    encoder_init(&htim4, &htim3, &htim2);

    // 2. Motors are on TIM5 (CH1..CH4) with DIR on PB0, PB1, PB2, PB12
    motor_config_t my_motors[4] = {
        { TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin },   // PWM: PA0 | DIR: PB0
        { TIM_CHANNEL_2, DIR2_GPIO_Port, DIR2_Pin },   // PWM: PA1 | DIR: PB1
        { TIM_CHANNEL_3, DIR3_GPIO_Port, DIR3_Pin },   // PWM: PA2 | DIR: PB2
        { TIM_CHANNEL_4, DIR4_GPIO_Port, DIR4_Pin }    // PWM: PA3 | DIR: PB12
    };
    motor_init(&htim5, my_motors);

    s_last_tick_50ms = HAL_GetTick();
}
void robot_process_rx(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        char c = (char)buf[i];

        if (c == '\r' || c == '\n') {
            if (rx_line_idx > 0) {
                rx_line_buffer[rx_line_idx] = '\0';

                // Parse integer from -100 to 100
                int val = atoi(rx_line_buffer);
                if (val > 100) val = 100;
                if (val < -100) val = -100;

                current_duty_percent = (int16_t)val;

                // Scale percentage (-100..100) to motor speed (-1000..1000)
                int16_t speed_scaled = current_duty_percent * 10;
                for (int m = 0; m < 4; m++) {
                    motor_set(m, speed_scaled);
                }
                rx_line_idx = 0;
            }
        } else if (rx_line_idx < (sizeof(rx_line_buffer) - 1)) {
            // Append incoming character
            rx_line_buffer[rx_line_idx++] = c;
        }
    }
}

void robot_loop(void)
{
    if (HAL_GetTick() - s_last_tick_50ms >= 50) {
        s_last_tick_50ms = HAL_GetTick();

        encoder_update();

        // Print active PWM % alongside total count and instantaneous delta
        int elen = snprintf(txbuf, sizeof(txbuf),
            "PWM:%d%% | E1:[Tot:%ld, D:%d] | E2:[Tot:%ld, D:%d] | E3:[Tot:%ld, D:%d]\r\n",
            (int)current_duty_percent,
            (long)encoder_get_count(0), (int)encoder_get_delta(0),
            (long)encoder_get_count(1), (int)encoder_get_delta(1),
            (long)encoder_get_count(2), (int)encoder_get_delta(2));

        CDC_Transmit_FS((uint8_t*)txbuf, elen);
    }
}
