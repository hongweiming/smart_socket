#include "led.h"
#include <stdio.h>
#include "gpio.h"

void app_led_turn_off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void app_led_turn_on(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void led_user_test(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
