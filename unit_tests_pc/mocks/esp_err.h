/**
 * @file esp_err.h
 * @brief Mock ESP-IDF error codes pro host-side unit testy.
 *
 * Nahrada hlavicky z ESP-IDF pro kompilaci na PC bez hardwaru.
 * Obsahuje jen ty konstanty, ktere realne pouziva isotp.c.
 */
#ifndef MOCK_ESP_ERR_H
#define MOCK_ESP_ERR_H

#include <stdint.h>

typedef int32_t esp_err_t;

#define ESP_OK                    0
#define ESP_FAIL                 -1

#define ESP_ERR_TIMEOUT           0x107
#define ESP_ERR_INVALID_STATE     0x103
#define ESP_ERR_INVALID_ARG       0x102
#define ESP_ERR_NO_MEM            0x101

#endif /* MOCK_ESP_ERR_H */
