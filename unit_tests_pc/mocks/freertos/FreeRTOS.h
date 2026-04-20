/**
 * @file FreeRTOS.h
 * @brief Mock FreeRTOS zakladnich typu pro PC unit testy.
 *
 * Nahrada hlavicky z ESP-IDF FreeRTOS. Exportuje jen to, co potrebuje
 * isotp.c — zejmena TickType_t, portTICK_PERIOD_MS a pdMS_TO_TICKS().
 * Na realnem ESP-IDF je 1 tick = 1 ms, my to napodobujeme identicky.
 */
#ifndef MOCK_FREERTOS_H
#define MOCK_FREERTOS_H

#include <stdint.h>

typedef uint32_t TickType_t;
typedef uint32_t BaseType_t;

/* 1 tick = 1 ms (jako v default ESP-IDF konfiguraci). */
#define portTICK_PERIOD_MS   1u

/* pdMS_TO_TICKS je identita — milisekundy jsou ticky. */
#define pdMS_TO_TICKS(ms)    ((TickType_t)(ms))

#define pdTRUE               1
#define pdFALSE              0
#define pdPASS               1
#define pdFAIL               0

#endif /* MOCK_FREERTOS_H */
