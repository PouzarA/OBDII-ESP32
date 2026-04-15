/**
 * @file obd2_internal.h
 * @brief Interni sdilene deklarace pro OBD-II vrstvu
 *
 * Tento hlavickovy soubor obsahuje interni typy a deklarace sdilene
 * mezi soubory obd2.c, obd2_pids.c a obd2_diag.c.
 * NENI soucasti verejneho API -- neincludovat z aplikacniho kodu.
 */

#ifndef OBD2_INTERNAL_H
#define OBD2_INTERNAL_H

#include "obd2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Interni kontext OBD-II vrstvy.
 *
 * Jedina staticka instance -- bez dynamicke alokace.
 * Obsahuje konfiguraci ECU adres, timeout, cache podporovanych PIDu
 * a posledni negativni odpoved.
 */
typedef struct {
    uint32_t    tx_id;              /**< CAN ID pro fyzicky pozadavek (vychozi 0x7E0) */
    uint32_t    rx_id;              /**< CAN ID pro fyzickou odpoved (vychozi 0x7E8) */
    uint32_t    timeout_ms;         /**< Timeout pro odpoved v ms */
    bool        initialized;        /**< Priznak: byl volan obd2_init()? */
    uint32_t    supported_pids[8];  /**< Bitmaska: 8 x 32 = 256 PIDu */
    bool        pids_queried;       /**< Priznak: byla provedena query supported PIDs? */
    obd2_nrc_info_t last_nrc;       /**< Posledni negativni odpoved (NRC) */
} _obd2_ctx_t;

/** Globalni kontext -- definovan v obd2.c */
extern _obd2_ctx_t _ctx;

/**
 * @brief Interni pomocna funkce pro OBD-II ISO-TP transakci.
 *
 * Odesle pozadavek pres ISO-TP a ceka na odpoved. Zpracovava negativni
 * odpovedi (NRC) a uklada je do _ctx.last_nrc.
 * Definovana v obd2.c, pouzivana v obd2_pids.c a obd2_diag.c.
 *
 * @param req           Ukazatel na data pozadavku (SID + parametry)
 * @param req_len       Delka pozadavku v bytech
 * @param resp          Vystupni buffer pro odpoved
 * @param resp_len      Vstup: velikost bufferu, vystup: skutecna delka odpovedi
 * @param use_broadcast true = pouzit broadcast (funkcni) adresu, false = fyzicka adresa
 * @return OBD2_OK pri uspechu, jinak chybovy kod
 */
extern obd2_status_t _obd2_request(const uint8_t *req, uint8_t req_len,
                                    uint8_t *resp, uint16_t *resp_len,
                                    bool use_broadcast);

#ifdef __cplusplus
}
#endif

#endif /* OBD2_INTERNAL_H */
