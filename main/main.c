/*
 * LED blink with FreeRTOS, usando filas e semáforo
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

#define UART_ID    uart0
#define BAUD_RATE  115200

// Definições de pinos
static const int ldr1       = 26; // GPIO 26 = canal 0
static const int ldr2       = 27; // GPIO 27 = canal 1
static const int servoPinOne = 15;
static const int servoPinTwo = 14;

// Handles de filas e semáforo
static QueueHandle_t xQueueServo1;
static QueueHandle_t xQueueServo2;
static SemaphoreHandle_t xPWMmutex;

// Converte valor ADC (0–4095) para microsegundos (400–2400 µs)
static int map_adc_to_us(uint16_t adc_value) {
    int pos_us = ((adc_value * (2400 - 400)) / 4000) + 400;
    if (pos_us < 400)  pos_us = 400;
    if (pos_us > 2400) pos_us = 2400;
    return pos_us;
}

// Inicializa PWM no servoPin e posiciona em startMillis
static void setServo(int servoPin, float startMillis) {
    gpio_set_function(servoPin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(servoPin);
    pwm_config cfg = pwm_get_default_config();

    uint64_t clk_hz = clock_get_hz(5);
    float clk_div = 64.0f;
    while (clk_hz/clk_div/50 > 65535 && clk_div < 256.0f) {
        clk_div += 64.0f;
    }
    uint32_t wrap = clk_hz/clk_div/50;

    pwm_config_set_clkdiv(&cfg, clk_div);
    pwm_config_set_wrap  (&cfg, wrap);
    pwm_init(slice, &cfg, true);

    pwm_set_gpio_level(servoPin, (startMillis/20000.0f) * wrap);
}

// Ajusta apenas o duty-cycle (pulso) do PWM
static void setMillis(int servoPin, float millis) {
    uint64_t clk_hz = clock_get_hz(5);
    float clk_div = 64.0f;
    while (clk_hz/clk_div/50 > 65535 && clk_div < 256.0f) {
        clk_div += 64.0f;
    }
    uint32_t wrap = clk_hz/clk_div/50;

    pwm_set_gpio_level(servoPin, (millis/20000.0f) * wrap);
}

// Task que lê LDR1 e envia posição para servo 1
static void ldr1_task(void *pv) {
    (void)pv;
    adc_gpio_init(ldr1);
    for (;;) {
        adc_select_input(0);
        uint16_t val = adc_read();
        int pos = map_adc_to_us(val);
        xQueueSend(xQueueServo1, &pos, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task que lê LDR2 e envia posição para servo 2
static void ldr2_task(void *pv) {
    (void)pv;
    adc_gpio_init(ldr2);
    for (;;) {
        adc_select_input(1);
        uint16_t val = adc_read();
        int pos = map_adc_to_us(val);
        xQueueSend(xQueueServo2, &pos, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task que recebe da fila e atualiza servo 1 (com mutua exclusão)
static void servo1_task(void *pv) {
    (void)pv;
    int pos;
    for (;;) {
        if (xQueueReceive(xQueueServo1, &pos, portMAX_DELAY) == pdPASS) {
            xSemaphoreTake(xPWMmutex, portMAX_DELAY);
            setMillis(servoPinOne, pos);
            xSemaphoreGive(xPWMmutex);
        }
    }
}

// Task que recebe da fila e atualiza servo 2 (com mutua exclusão)
static void servo2_task(void *pv) {
    (void)pv;
    int pos;
    for (;;) {
        if (xQueueReceive(xQueueServo2, &pos, portMAX_DELAY) == pdPASS) {
            xSemaphoreTake(xPWMmutex, portMAX_DELAY);
            setMillis(servoPinTwo, pos);
            xSemaphoreGive(xPWMmutex);
        }
    }
}

int main() {
    stdio_init_all();
    adc_init();
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // cria filas de inteiros (10 elementos cada)
    xQueueServo1 = xQueueCreate(10, sizeof(int));
    xQueueServo2 = xQueueCreate(10, sizeof(int));

    // cria mutex para proteger acesso ao PWM
    xPWMmutex = xSemaphoreCreateMutex();

    // configura servos na posição inicial
    setServo(servoPinOne, 1400);
    setServo(servoPinTwo, 1400);

    // cria tasks
    xTaskCreate(ldr1_task,    "LDR1",   256, NULL, 1, NULL);
    xTaskCreate(servo1_task,  "SV1",    256, NULL, 2, NULL);
    xTaskCreate(ldr2_task,    "LDR2",   256, NULL, 1, NULL);
    xTaskCreate(servo2_task,  "SV2",    256, NULL, 2, NULL);

    // inicia scheduler
    vTaskStartScheduler();

    // nunca chega aqui
    for (;;) { }
    return 0;
}
