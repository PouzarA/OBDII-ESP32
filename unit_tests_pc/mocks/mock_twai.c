/**
 * @file mock_twai.c
 * @brief Implementace mocku TWAI driveru a FreeRTOS casovaciho API.
 *
 * Navrh: mock udrzuje dve fronty ramcu (RX = prichozi od "ECU",
 * TX = zachycene odchozi z kodu pod testem) a simulovany cas.
 *
 * Klicove chovani:
 *   - twai_transmit() zkopiruje ramec do TX fronty a vrati ESP_OK.
 *     (Pokud je nastaveno mock_twai_next_tx_returns(), vrati se nastavena
 *      chyba a priznak se smaze.)
 *   - twai_receive() vyjmouti ramec z RX fronty. Pokud je fronta prazdna,
 *     posune simulovany cas o ticks_to_wait a vrati ESP_ERR_TIMEOUT.
 *   - xTaskGetTickCount() vraci simulovany cas v ticku (1 tick = 1 ms).
 *   - vTaskDelay() posune simulovany cas dopredu.
 *
 * Diky tomu testy probehnou deterministicky a okamzite — neni treba
 * realne cekat na timeoutu.
 */

#include <string.h>
#include <stdio.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mock_twai.h"

/* ========================================================================= */
/*  Interni stav mocku                                                       */
/* ========================================================================= */

/* RX fronta (ramce, ktere chci aby kod "prijal"). */
static twai_message_t _rx_queue[MOCK_TWAI_MAX_RX_FRAMES];
static int            _rx_head = 0;    /* index dalsiho cteni */
static int            _rx_tail = 0;    /* index dalsiho zapisu */

/* TX zachytovac (co kod odeslal). */
static twai_message_t _tx_queue[MOCK_TWAI_MAX_TX_FRAMES];
static int            _tx_count = 0;

/* Simulovany cas v ms. */
static uint32_t _sim_time_ms = 0;

/* Flagy pro simulaci chyb. */
static bool      _next_tx_fail      = false;
static esp_err_t _next_tx_err       = ESP_OK;
static twai_state_t _driver_state   = TWAI_STATE_STOPPED;
static int       _recovery_count    = 0;
static uint32_t  _pending_alerts    = 0;
static bool      _recovery_keeps_busoff = false;

/* ========================================================================= */
/*  Pomocne API pro testy                                                    */
/* ========================================================================= */

void mock_twai_reset(void)
{
    memset(_rx_queue, 0, sizeof(_rx_queue));
    memset(_tx_queue, 0, sizeof(_tx_queue));
    _rx_head        = 0;
    _rx_tail        = 0;
    _tx_count       = 0;
    _sim_time_ms    = 0;
    _next_tx_fail   = false;
    _next_tx_err    = ESP_OK;
    _driver_state   = TWAI_STATE_STOPPED;
    _recovery_count = 0;
    _pending_alerts = 0;
    _recovery_keeps_busoff = false;
}

void mock_twai_set_pending_alerts(uint32_t alerts)
{
    _pending_alerts = alerts;
}

void mock_twai_set_recovery_keeps_busoff(bool keeps_busoff)
{
    _recovery_keeps_busoff = keeps_busoff;
}

void mock_twai_inject_rx(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (_rx_tail >= MOCK_TWAI_MAX_RX_FRAMES) {
        fprintf(stderr, "[mock_twai] RX queue overflow — test error?\n");
        return;
    }
    twai_message_t *m = &_rx_queue[_rx_tail++];
    memset(m, 0, sizeof(*m));
    m->identifier       = id;
    m->data_length_code = dlc;
    if (data && dlc > 0) {
        memcpy(m->data, data, (dlc > 8) ? 8 : dlc);
    }
}

void mock_twai_inject_rx_frame(uint32_t id, uint8_t b0, uint8_t b1,
                               uint8_t b2, uint8_t b3, uint8_t b4,
                               uint8_t b5, uint8_t b6, uint8_t b7)
{
    uint8_t buf[8] = { b0, b1, b2, b3, b4, b5, b6, b7 };
    mock_twai_inject_rx(id, buf, 8);
}

int mock_twai_pending_rx_count(void)
{
    return _rx_tail - _rx_head;
}

int mock_twai_get_tx_count(void)
{
    return _tx_count;
}

const twai_message_t *mock_twai_get_tx_frame(int index)
{
    if (index < 0 || index >= _tx_count) return NULL;
    return &_tx_queue[index];
}

uint32_t mock_twai_get_time_ms(void)
{
    return _sim_time_ms;
}

void mock_twai_advance_time(uint32_t ms)
{
    _sim_time_ms += ms;
}

void mock_twai_next_tx_returns(esp_err_t err)
{
    _next_tx_fail = true;
    _next_tx_err  = err;
}

void mock_twai_set_state(twai_state_t state)
{
    _driver_state = state;
}

int mock_twai_get_recovery_count(void)
{
    return _recovery_count;
}

/* ========================================================================= */
/*  ESP-IDF TWAI API — volane z kodu pod testem                              */
/* ========================================================================= */

esp_err_t twai_driver_install(const twai_general_config_t *g_config,
                              const twai_timing_config_t  *t_config,
                              const twai_filter_config_t  *f_config)
{
    (void)g_config; (void)t_config; (void)f_config;
    _driver_state = TWAI_STATE_STOPPED;
    return ESP_OK;
}

esp_err_t twai_driver_uninstall(void)
{
    _driver_state = TWAI_STATE_STOPPED;
    return ESP_OK;
}

esp_err_t twai_start(void)
{
    _driver_state = TWAI_STATE_RUNNING;
    return ESP_OK;
}

esp_err_t twai_stop(void)
{
    _driver_state = TWAI_STATE_STOPPED;
    return ESP_OK;
}

esp_err_t twai_transmit(const twai_message_t *message, TickType_t ticks_to_wait)
{
    (void)ticks_to_wait;

    /* Test muze pres mock_twai_next_tx_returns() vynutit, ze dalsi
     * pokus vrati chybu (typicky ESP_ERR_INVALID_STATE pro BUS_OFF). */
    if (_next_tx_fail) {
        _next_tx_fail = false;
        return _next_tx_err;
    }

    if (message == NULL) return ESP_ERR_INVALID_ARG;

    if (_tx_count >= MOCK_TWAI_MAX_TX_FRAMES) {
        fprintf(stderr, "[mock_twai] TX queue overflow — test error?\n");
        return ESP_FAIL;
    }
    memcpy(&_tx_queue[_tx_count++], message, sizeof(*message));
    return ESP_OK;
}

esp_err_t twai_receive(twai_message_t *message, TickType_t ticks_to_wait)
{
    if (message == NULL) return ESP_ERR_INVALID_ARG;

    /*
     * ticks_to_wait == 0 znamena "neblokovaci poll hardware bufferu prave teď".
     * V testech tim `isotp_flush_rx_queue()` zjistuje stare ramce z
     * predchozi transakce. Tyto stare ramce v ramci testu neexistuji —
     * testy predvyplnuji RX frontu jako "odpoved ECU, ktera prijde AZ kod
     * zacne cekat", tedy NE jako nevytezena historicka data. Vracime tedy
     * ESP_ERR_TIMEOUT, aby flush neodstranil ocekavane odpovedi.
     *
     * Pozn.: V realnem TWAI driveru by tohle chovani odpovidalo prazdnemu
     * RX FIFO, coz je nejcastejsi stav mezi transakcemi.
     */
    if (ticks_to_wait == 0) {
        return ESP_ERR_TIMEOUT;
    }

    /* Pokud je v RX fronte ramec, vratime ho ihned a cas neposouvame. */
    if (_rx_head < _rx_tail) {
        memcpy(message, &_rx_queue[_rx_head++], sizeof(*message));
        return ESP_OK;
    }

    /* Fronta prazdna — simulujeme blokaci a nasledny timeout. */
    _sim_time_ms += (uint32_t)ticks_to_wait;
    return ESP_ERR_TIMEOUT;
}

esp_err_t twai_get_status_info(twai_status_info_t *status_info)
{
    if (status_info == NULL) return ESP_ERR_INVALID_ARG;
    memset(status_info, 0, sizeof(*status_info));
    status_info->state = _driver_state;
    return ESP_OK;
}

esp_err_t twai_initiate_recovery(void)
{
    _recovery_count++;
    /* Po spustenem recovery po nejake dobe ocekavame stav STOPPED —
     * ale isotp.c na to pollem ceka, takze tady rovnou prepneme.
     * Test si muze pres mock_twai_set_recovery_keeps_busoff(true) vynutit,
     * aby driver v BUS_OFF zustal — overuje cestu, kde se recovery nepovede. */
    if (!_recovery_keeps_busoff) {
        _driver_state = TWAI_STATE_STOPPED;
    }
    return ESP_OK;
}

esp_err_t twai_read_alerts(uint32_t *alerts, TickType_t ticks_to_wait)
{
    if (alerts == NULL) return ESP_ERR_INVALID_ARG;
    *alerts = _pending_alerts;
    _pending_alerts = 0;
    if (*alerts == 0 && ticks_to_wait > 0) {
        /* Realny driver by blokoval az do timeoutu — simulujeme posunem casu. */
        _sim_time_ms += (uint32_t)ticks_to_wait;
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/* ========================================================================= */
/*  FreeRTOS task.h — xTaskGetTickCount(), vTaskDelay()                      */
/* ========================================================================= */

TickType_t xTaskGetTickCount(void)
{
    return (TickType_t)_sim_time_ms;
}

void vTaskDelay(TickType_t xTicksToDelay)
{
    _sim_time_ms += (uint32_t)xTicksToDelay;
}
