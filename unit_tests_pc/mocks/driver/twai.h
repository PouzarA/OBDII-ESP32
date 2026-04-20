/**
 * @file twai.h
 * @brief Mock ESP-IDF TWAI driveru pro host-side unit testy.
 *
 * Nahradni hlavicka, ktera poskytuje pouze to, co pouziva isotp.c:
 *   - Datove typy: twai_message_t, twai_general_config_t,
 *     twai_timing_config_t, twai_filter_config_t, twai_status_info_t
 *   - Funkce: twai_driver_install(), twai_driver_uninstall(),
 *     twai_start(), twai_stop(), twai_transmit(), twai_receive(),
 *     twai_get_status_info(), twai_initiate_recovery()
 *   - Konfiguracni makra: TWAI_GENERAL_CONFIG_DEFAULT,
 *     TWAI_TIMING_CONFIG_500KBITS, TWAI_TIMING_CONFIG_250KBITS,
 *     TWAI_FILTER_CONFIG_ACCEPT_ALL
 *   - Rezimy a stavy driveru: TWAI_MODE_NORMAL, TWAI_STATE_*
 *
 * Implementace vsech funkci je v mock_twai.c. Pomocne API pro testy
 * (injektovani RX ramcu, cteni zachycenych TX ramcu, posouvani
 * simulovaneho casu, simulace chyb) je v mock_twai.h.
 */
#ifndef MOCK_DRIVER_TWAI_H
#define MOCK_DRIVER_TWAI_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* gpio_num_t — normalne je v driver/gpio.h, my ji mame tady, kvuli
 * jednoduchosti (isotp.c dela jen pretypovani int -> gpio_num_t). */
typedef int gpio_num_t;

/* Rezimy TWAI driveru (isotp.c pouziva jen TWAI_MODE_NORMAL). */
typedef enum {
    TWAI_MODE_NORMAL       = 0,
    TWAI_MODE_NO_ACK       = 1,
    TWAI_MODE_LISTEN_ONLY  = 2
} twai_mode_t;

/* Stavy TWAI driveru — isotp.c kontroluje BUS_OFF a STOPPED behem recovery. */
typedef enum {
    TWAI_STATE_STOPPED     = 0,
    TWAI_STATE_RUNNING     = 1,
    TWAI_STATE_BUS_OFF     = 2,
    TWAI_STATE_RECOVERING  = 3
} twai_state_t;

/**
 * @brief Struktura CAN ramce (napodobuje twai_message_t z ESP-IDF).
 *
 * isotp.c pouziva pouze: identifier, data_length_code, data[8], extd.
 * Ostatni flagy (rtr, ss, self, dlc_non_comp) nechavame pro kompatibilitu.
 */
typedef struct {
    /* Flagy — v ESP-IDF jsou jako bitove pole uvnitr anonymniho unionu;
     * pro mock stacilo pretypovat v kodu, tak je vystavime jako uint8_t. */
    uint32_t extd:          1;  /**< 1 = 29-bit extended ID */
    uint32_t rtr:           1;  /**< Remote transmission request */
    uint32_t ss:            1;  /**< Single shot */
    uint32_t self:          1;  /**< Self reception */
    uint32_t dlc_non_comp:  1;  /**< DLC non-compliant */
    uint32_t reserved:      27;

    uint32_t identifier;        /**< CAN ID (11 nebo 29 bit) */
    uint8_t  data_length_code;  /**< Pocet platnych bajtu v data[] (0..8) */
    uint8_t  data[8];           /**< Uzitecna data CAN ramce */
} twai_message_t;

/** Konfigurace driveru — isotp.c nastavuje jen tx_io, rx_io, mode, rx_queue_len. */
typedef struct {
    twai_mode_t   mode;
    gpio_num_t    tx_io;
    gpio_num_t    rx_io;
    gpio_num_t    clkout_io;
    gpio_num_t    bus_off_io;
    uint32_t      tx_queue_len;
    uint32_t      rx_queue_len;
    uint32_t      alerts_enabled;
    uint32_t      clkout_divider;
    int           intr_flags;
} twai_general_config_t;

/** Casovaci konfigurace — realne ESP-IDF ma vic poli, tady stačí prazdna. */
typedef struct {
    uint32_t brp;
    uint8_t  tseg_1;
    uint8_t  tseg_2;
    uint8_t  sjw;
    bool     triple_sampling;
} twai_timing_config_t;

/** Filter konfigurace. isotp_init pouziva ACCEPT_ALL — pole zde jsou nule. */
typedef struct {
    uint32_t acceptance_code;
    uint32_t acceptance_mask;
    bool     single_filter;
} twai_filter_config_t;

/** Stav driveru za behu (isotp.c cte state, tx_error_counter, rx_error_counter). */
typedef struct {
    twai_state_t state;
    uint32_t     msgs_to_tx;
    uint32_t     msgs_to_rx;
    uint32_t     tx_error_counter;
    uint32_t     rx_error_counter;
    uint32_t     tx_failed_count;
    uint32_t     rx_missed_count;
    uint32_t     rx_overrun_count;
    uint32_t     arb_lost_count;
    uint32_t     bus_error_count;
} twai_status_info_t;

/* Konfiguracni makra — mock vraci jen nejake rozumne defaulty. */
#define TWAI_GENERAL_CONFIG_DEFAULT(_tx, _rx, _mode) \
    { .mode = (_mode), .tx_io = (_tx), .rx_io = (_rx), \
      .clkout_io = -1, .bus_off_io = -1, \
      .tx_queue_len = 5, .rx_queue_len = 5, \
      .alerts_enabled = 0, .clkout_divider = 0, .intr_flags = 0 }

#define TWAI_TIMING_CONFIG_500KBITS() \
    { .brp = 8, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false }

#define TWAI_TIMING_CONFIG_250KBITS() \
    { .brp = 16, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false }

#define TWAI_FILTER_CONFIG_ACCEPT_ALL() \
    { .acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true }

/* Alert bity (hodnoty kopiruji ESP-IDF, abychom mohli pripadne sdilet
 * konstanty s realnym kodem). Mock je pouziva jen jako bitove priznaky. */
#define TWAI_ALERT_TX_IDLE              0x00000001
#define TWAI_ALERT_TX_SUCCESS           0x00000002
#define TWAI_ALERT_RX_DATA              0x00000004
#define TWAI_ALERT_BELOW_ERR_WARN       0x00000008
#define TWAI_ALERT_ERR_ACTIVE           0x00000010
#define TWAI_ALERT_RECOVERY_IN_PROGRESS 0x00000020
#define TWAI_ALERT_BUS_RECOVERED        0x00000040
#define TWAI_ALERT_ARB_LOST             0x00000080
#define TWAI_ALERT_ABOVE_ERR_WARN       0x00000100
#define TWAI_ALERT_BUS_ERROR            0x00000200
#define TWAI_ALERT_TX_FAILED            0x00000400
#define TWAI_ALERT_RX_QUEUE_FULL        0x00000800
#define TWAI_ALERT_ERR_PASS             0x00001000
#define TWAI_ALERT_BUS_OFF              0x00002000
#define TWAI_ALERT_NONE                 0x00000000
#define TWAI_ALERT_ALL                  0x00003FFF

/* ------------------------------------------------------------------------- */
/*  TWAI API — implementace v mock_twai.c                                    */
/* ------------------------------------------------------------------------- */

esp_err_t twai_driver_install(const twai_general_config_t *g_config,
                              const twai_timing_config_t  *t_config,
                              const twai_filter_config_t  *f_config);

esp_err_t twai_driver_uninstall(void);

esp_err_t twai_start(void);
esp_err_t twai_stop(void);

esp_err_t twai_transmit(const twai_message_t *message, TickType_t ticks_to_wait);
esp_err_t twai_receive(twai_message_t *message, TickType_t ticks_to_wait);

esp_err_t twai_get_status_info(twai_status_info_t *status_info);
esp_err_t twai_initiate_recovery(void);

/**
 * @brief Precte aktualni alert priznaky (mock pouze vraci hodnotu nastavenou
 *        pres mock_twai_set_pending_alerts() a pak ji vynuluje).
 */
esp_err_t twai_read_alerts(uint32_t *alerts, TickType_t ticks_to_wait);

#endif /* MOCK_DRIVER_TWAI_H */
