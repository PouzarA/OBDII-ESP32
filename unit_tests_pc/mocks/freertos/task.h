/**
 * @file task.h
 * @brief Mock FreeRTOS task.h pro PC unit testy.
 *
 * Prohlasuje minimalni API pro praci s casem, ktere pouziva isotp.c:
 *   - xTaskGetTickCount() — vraci simulovany tick (v ms)
 *   - vTaskDelay()        — posouva simulovany cas dopredu
 *
 * Implementace je v mock_twai.c, protoze sdili stejnou "simulovanou
 * nastenku casu" s mockem TWAI (je dulezite, aby se RX timeouty v TWAI
 * a casomira v isotp_can_recv aktualizovaly synchronne).
 */
#ifndef MOCK_FREERTOS_TASK_H
#define MOCK_FREERTOS_TASK_H

#include "FreeRTOS.h"

/* Vraci aktualni simulovany tick (= ms od startu testu). */
TickType_t xTaskGetTickCount(void);

/* Posune simulovany cas o zadany pocet ticku dopredu. */
void vTaskDelay(TickType_t xTicksToDelay);

#endif /* MOCK_FREERTOS_TASK_H */
