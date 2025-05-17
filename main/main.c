/*
 * LED blink with FreeRTOS
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"

#define UART_ID uart0
#define BAUD_RATE 115200
// Definições de pinos
const int ldr1= 26; // GPIO 26 = canal 0
const int ldr2 = 27; // GPIO 27 = canal 1

const int servoPinOne = 15;
const int servoPinTwo = 14;

QueueHandle_t xQueueADC;

int map_adc_to_us(uint16_t adc_value) {
    int pos_us = ((adc_value * (2400 - 400)) / 4000) + 400;
    if (pos_us < 400) pos_us = 400;
    if (pos_us > 2400) pos_us = 2400;
    return pos_us;
}

void setMillis(int servoPin, float millis)
{
    pwm_set_gpio_level(servoPin, (millis/20000.f)*39062);
}

void setServo(int servoPin, float startMillis)
{
    float clockDiv = 64;
    float wrap = 39062;
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servoPin);

    pwm_config config = pwm_get_default_config();
    
    uint64_t clockspeed = clock_get_hz(5);
    clockDiv = 64;
    wrap = 39062;

    while (clockspeed/clockDiv/50 > 65535 && clockDiv < 256) clockDiv += 64; 
    wrap = clockspeed/clockDiv/50;

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    setMillis(servoPin, startMillis);
}
void ldr1_task(void *p) {
    adc_gpio_init(ldr1);

    while (true) {
        adc_select_input(0);  
        uint16_t adc_value = adc_read();

        int pos_us = map_adc_to_us(adc_value);
        setMillis(servoPinOne, pos_us);


        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task para ler LDR2 e controlar Servo 2
void ldr2_task(void *p) {
    adc_gpio_init(ldr2);

    while (true) {
        adc_select_input(1); 
        uint16_t adc_value = adc_read();

        int pos_us = map_adc_to_us(adc_value);
        setMillis(servoPinTwo, pos_us);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main() {
    stdio_init_all();
    adc_init();
    
    setServo(servoPinOne, 1400);
    setServo(servoPinTwo, 1400);

    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
 
    xTaskCreate(ldr1_task, "LDR1 TASK", 4095, NULL, 1, NULL);
    xTaskCreate(ldr2_task, "LDR2 TASK", 4095, NULL, 1, NULL);
 
    vTaskStartScheduler();
 
    while (true){
    } 
 }
 
