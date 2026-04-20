/**
 * @file mock_twai.h
 * @brief Pomocne API mocku TWAI — ovladaci panel pro unit testy.
 *
 * Toto NENI ESP-IDF rozhrani (to je v driver/twai.h). Tyto funkce vola
 * test, aby:
 *   1) pripravil odpoved ECU (naplni RX frontu ramci, ktere jakoby
 *      dorazily po CAN sbernici),
 *   2) zkontroloval, ze isotp.c skutecne odeslal ramec se spravnym
 *      CAN ID, PCI bajtem a daty (prohleda TX zachytovaci pole),
 *   3) simuloval ubehnuti casu (pro testovani timeoutu bez realneho
 *      cekani),
 *   4) simuloval chyby (bus-off, TX error, RX error) a overil chovani
 *      recovery vetve.
 *
 * Vsechna vnitrni data jsou v static promennych uvnitr mock_twai.c,
 * proto je nutne volat mock_twai_reset() na zacatku kazdeho testu.
 */
#ifndef MOCK_TWAI_H
#define MOCK_TWAI_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"

/* Maximalni pocet ramcu v RX a TX frontach — vetsi nez jakykoli realny
 * scenario OBD-II (VIN = 20 B ≈ 4 ramce, 126 DTC = 252 B ≈ 37 ramcu). */
#define MOCK_TWAI_MAX_RX_FRAMES     512
#define MOCK_TWAI_MAX_TX_FRAMES     512

/* ------------------------------------------------------------------------- */
/*  Priprava testu                                                           */
/* ------------------------------------------------------------------------- */

/**
 * @brief Resetuje cely stav mocku: RX fronta, TX zachytovac, cas, chyby.
 *
 * Musi se volat v setUp() kazdeho testu, jinak se stav z predchoziho
 * testu prolije do dalsiho (napr. zustanou nepouzite TX ramce).
 */
void mock_twai_reset(void);

/* ------------------------------------------------------------------------- */
/*  Injektovani prichozich CAN ramcu (RX smer)                               */
/* ------------------------------------------------------------------------- */

/**
 * @brief Prida jeden CAN ramec do fronty, kterou bude cist twai_receive().
 *
 * Ramce jsou vydavany v FIFO poradi. Kazde volani twai_receive() z kodu
 * pod testem vytahne jeden ramec. Pokud je fronta prazdna, twai_receive
 * posune simulovany cas a vrati ESP_ERR_TIMEOUT.
 *
 * @param id    11-bit nebo 29-bit CAN ID (napr. 0x7E8 pro odpoved ECU #1)
 * @param data  Ukazatel na data CAN ramce
 * @param dlc   Data Length Code (0..8)
 */
void mock_twai_inject_rx(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * @brief Zkratka pro injektovani 8-bajtoveho ramce (typicke pro OBD-II).
 */
void mock_twai_inject_rx_frame(uint32_t id, uint8_t b0, uint8_t b1,
                               uint8_t b2, uint8_t b3, uint8_t b4,
                               uint8_t b5, uint8_t b6, uint8_t b7);

/**
 * @brief Pocet ramcu cekajicich v RX fronte (jeste neprectenych kodem).
 */
int mock_twai_pending_rx_count(void);

/* ------------------------------------------------------------------------- */
/*  Zachycene odchozi CAN ramce (TX smer)                                    */
/* ------------------------------------------------------------------------- */

/**
 * @brief Pocet ramcu, ktere kod pod testem odeslal pres twai_transmit().
 */
int mock_twai_get_tx_count(void);

/**
 * @brief Vrati ukazatel na zachyceny TX ramec dle indexu (0..count-1).
 *
 * @return NULL pokud je index mimo rozsah.
 */
const twai_message_t *mock_twai_get_tx_frame(int index);

/* ------------------------------------------------------------------------- */
/*  Simulace casu                                                            */
/* ------------------------------------------------------------------------- */

/**
 * @brief Vrati aktualni simulovany cas v ms (stejne jako xTaskGetTickCount).
 */
uint32_t mock_twai_get_time_ms(void);

/**
 * @brief Posune simulovany cas dopredu o zadany pocet ms.
 *
 * Bezne to volat nemusite — twai_receive() na prazdne fronte si cas
 * posune sam (simuluje blokaci na timeoutu). Hodi se ale, pokud chcete
 * explicitne overit, ze kod neceka zbytecne dlouho.
 */
void mock_twai_advance_time(uint32_t ms);

/* ------------------------------------------------------------------------- */
/*  Simulace chyb                                                            */
/* ------------------------------------------------------------------------- */

/**
 * @brief Dalsi volani twai_transmit() vrati danou chybu.
 *
 * Priznak se automaticky resetuje po jednom pouziti (next-call-only).
 * Pro trvale selhani volejte mezi pokusy znovu.
 *
 * @param err Napr. ESP_ERR_INVALID_STATE pro simulaci BUS_OFF.
 */
void mock_twai_next_tx_returns(esp_err_t err);

/**
 * @brief Nastavi stav driveru vraceny z twai_get_status_info().
 *
 * Pouzijte v kombinaci s mock_twai_next_tx_returns(ESP_ERR_INVALID_STATE),
 * abyste simulovali BUS_OFF a ověřili, že isotp.c spusti recovery.
 */
void mock_twai_set_state(twai_state_t state);

/**
 * @brief Pocet volani twai_initiate_recovery() (pro overeni BUS_OFF recovery).
 */
int mock_twai_get_recovery_count(void);

/**
 * @brief Nastavi bitovou masku alertu, kterou vrati nejblizsi twai_read_alerts().
 *
 * Po precteni se maska v mocku vynuluje (stejne jako se chova ESP-IDF).
 */
void mock_twai_set_pending_alerts(uint32_t alerts);

/**
 * @brief Pokud true, twai_initiate_recovery() ponecha driver v BUS_OFF
 *        misto bezneho prepnuti do STOPPED. Slouzi k testu, ze isotp.c
 *        spravne vrati ISOTP_ERR_CAN_TX, pokud se recovery nepovede.
 */
void mock_twai_set_recovery_keeps_busoff(bool keeps_busoff);

#endif /* MOCK_TWAI_H */
