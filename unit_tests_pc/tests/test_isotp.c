/**
 * @file test_isotp.c
 * @brief Unit testy transportniho protokolu ISO 15765-2 (ISO-TP).
 *
 * Testy pokryvaji:
 *   - Single Frame (SF) transakce: spravny PCI bajt, padding, echo dat.
 *   - Multi-frame transakce: FF parsovani, Flow Control odpoved, skladani CF.
 *   - Chybove stavy: timeout, overflow (FF_DL > max), nespravne sekvencni cislo.
 *   - Broadcast: sber odpovedi od vicero ECU, ignorace non-OBD ramcu.
 *   - Validace argumentu: NULL ukazatele, delka 0, nezinicializovana vrstva.
 *
 * Vsechny testy bezi deterministicky — mock TWAI simuluje cas a RX ramce,
 * takze netreba cekat na skutecne timeouty.
 *
 * Inspirace: openxc/isotp-c/tree/master/tests (openXC ISO-TP test suite).
 */

#include <stdio.h>
#include <string.h>
#include "unity_lite.h"
#include "mock_twai.h"
#include "isotp.h"

/* ========================================================================= */
/*  Pomocne funkce a setup                                                   */
/* ========================================================================= */

/* Inicializace ISO-TP vrstvy pred kazdym testem. Pouzivaji defaultni piny. */
static void setup_initialized_isotp(void)
{
    mock_twai_reset();
    isotp_set_log_level(ISOTP_LOG_NONE);  /* Ztlumi vypisy behem testu */
    isotp_status_t s = isotp_init(500000, 5, 4);
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
}

/* ========================================================================= */
/*  Test 1: Inicializace a deinicializace                                    */
/* ========================================================================= */

void test_isotp_init_success(void)
{
    mock_twai_reset();
    isotp_set_log_level(ISOTP_LOG_NONE);
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, isotp_init(500000, 5, 4));
    isotp_deinit();
}

void test_isotp_init_250k_also_ok(void)
{
    mock_twai_reset();
    isotp_set_log_level(ISOTP_LOG_NONE);
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, isotp_init(250000, 5, 4));
    isotp_deinit();
}

void test_isotp_init_unsupported_baudrate(void)
{
    mock_twai_reset();
    isotp_set_log_level(ISOTP_LOG_NONE);
    /* 1 Mbit/s neni v OBD-II standardni, vrstva to musi odmitnout. */
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_INVALID_ARG, isotp_init(1000000, 5, 4));
}

/* ========================================================================= */
/*  Test 2: Single Frame transakce (request + response v jednom ramci)       */
/* ========================================================================= */

/*
 * Cely scenar Mode 01 PID $0C (otacky motoru):
 *   Request:  0x7E0 -> [02 01 0C AA AA AA AA AA]  (SF, len=2)
 *   Response: 0x7E8 -> [04 41 0C 1A F8 AA AA AA]  (SF, len=4, RPM=1726)
 */
void test_isotp_transaction_sf_request_has_correct_pci(void)
{
    setup_initialized_isotp();

    /* Pripravim response, aby transakce neskoncila timeoutem. */
    mock_twai_inject_rx_frame(0x7E8,
        0x04, 0x41, 0x0C, 0x1A, 0xF8, 0xCC, 0xCC, 0xCC);

    uint8_t req[] = { 0x01, 0x0C };
    uint8_t resp[16];
    uint16_t resp_len = 0;

    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);

    /* Zkontroluju, co kod odeslal. */
    TEST_ASSERT_EQUAL_INT(1, mock_twai_get_tx_count());
    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_EQUAL_HEX(0x7E0, tx->identifier);
    TEST_ASSERT_EQUAL_INT(8, tx->data_length_code);
    TEST_ASSERT_EQUAL_HEX(0x02, tx->data[0]);  /* SF PCI | len=2 */
    TEST_ASSERT_EQUAL_HEX(0x01, tx->data[1]);  /* SID 01 */
    TEST_ASSERT_EQUAL_HEX(0x0C, tx->data[2]);  /* PID 0C */
    /* Zbyvajici bajty by mely byt padding (0xCC dle ISOTP_PADDING_BYTE). */
    TEST_ASSERT_EQUAL_HEX(0xCC, tx->data[3]);
    TEST_ASSERT_EQUAL_HEX(0xCC, tx->data[7]);
}

void test_isotp_transaction_sf_response_is_extracted(void)
{
    setup_initialized_isotp();

    mock_twai_inject_rx_frame(0x7E8,
        0x04, 0x41, 0x0C, 0x1A, 0xF8, 0xCC, 0xCC, 0xCC);

    uint8_t req[] = { 0x01, 0x0C };
    uint8_t resp[16];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(4, resp_len);
    TEST_ASSERT_EQUAL_HEX(0x41, resp[0]);
    TEST_ASSERT_EQUAL_HEX(0x0C, resp[1]);
    TEST_ASSERT_EQUAL_HEX(0x1A, resp[2]);
    TEST_ASSERT_EQUAL_HEX(0xF8, resp[3]);
}

void test_isotp_transaction_sf_max_7byte_request(void)
{
    setup_initialized_isotp();

    /* Pripravim nejakou validni odpoved. */
    mock_twai_inject_rx_frame(0x7E8,
        0x03, 0x7F, 0x01, 0x12, 0xCC, 0xCC, 0xCC, 0xCC);

    uint8_t req[7] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
    uint8_t resp[16];
    uint16_t resp_len = 0;
    (void)isotp_transaction(0x7E0, 0x7E8, req, 7, resp, &resp_len, 1000);

    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_EQUAL_HEX(0x07, tx->data[0]);  /* SF, len=7 */
    TEST_ASSERT_EQUAL_MEMORY(req, &tx->data[1], 7);
}

/* ========================================================================= */
/*  Test 3: Timeout kdyz ECU neodpovi                                        */
/* ========================================================================= */

void test_isotp_transaction_timeout_on_no_response(void)
{
    setup_initialized_isotp();
    /* RX fronta zustava prazdna — kod by mel dobehnout s TIMEOUT. */

    uint8_t req[] = { 0x01, 0x0C };
    uint8_t resp[16];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 100);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT, s);
}

/* ========================================================================= */
/*  Test 4: Validace argumentu                                               */
/* ========================================================================= */

void test_isotp_transaction_not_initialized(void)
{
    mock_twai_reset();
    isotp_set_log_level(ISOTP_LOG_NONE);
    /* Predchozi testy mohly zanechat inicializovanou vrstvu — vycistime ji. */
    isotp_deinit();
    /* A po tomto bode zamerne NEzavolame isotp_init(). */
    uint8_t req[] = { 0x01 };
    uint8_t resp[16];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 1,
                                          resp, &resp_len, 100);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_NOT_INITIALIZED, s);
}

void test_isotp_transaction_null_request(void)
{
    setup_initialized_isotp();
    uint8_t resp[16];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, NULL, 1,
                                          resp, &resp_len, 100);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_INVALID_ARG, s);
}

void test_isotp_transaction_null_response(void)
{
    setup_initialized_isotp();
    uint8_t req[] = { 0x01 };
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 1,
                                          NULL, &resp_len, 100);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_INVALID_ARG, s);
}

/* ========================================================================= */
/*  Test 5: Multi-frame prijem (FF + CF)                                     */
/* ========================================================================= */

/*
 * Scenar: ECU vraci 10 bajtu dat (napr. 8 bajtu uzitecnych po [SID+PID]).
 * Sekvence:
 *   FF :  [0x10 0x0A D1 D2 D3 D4 D5 D6]  — FF, FF_DL=10, 6 prvnich bajtu
 *   <kod odesle FC: [0x30 00 00 CC CC CC CC CC]>
 *   CF1:  [0x21 D7 D8 D9 D10 CC CC CC]   — SN=1, 4 zbyvajici bajty + padding
 *
 * Pro nas test pouzijeme jednoduche bajty 0xA1..0xAA, aby se lehce overovalo.
 */
void test_isotp_transaction_multiframe_response(void)
{
    setup_initialized_isotp();

    /* FF: 10 bajtu payloadu, prvnich 6 = 0xA1..0xA6 */
    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x0A, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6);
    /* CF1: SN=1, zbylych 4 bajtu = 0xA7..0xAA */
    mock_twai_inject_rx_frame(0x7E8,
        0x21, 0xA7, 0xA8, 0xA9, 0xAA, 0xCC, 0xCC, 0xCC);

    uint8_t req[] = { 0x09, 0x02 };  /* Mode 09 PID 02 = VIN */
    uint8_t resp[32];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(10, resp_len);

    uint8_t expected[10] = {
        0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA
    };
    TEST_ASSERT_EQUAL_MEMORY(expected, resp, 10);
}

void test_isotp_transaction_multiframe_sends_flow_control(void)
{
    setup_initialized_isotp();

    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x0A, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6);
    mock_twai_inject_rx_frame(0x7E8,
        0x21, 0xA7, 0xA8, 0xA9, 0xAA, 0xCC, 0xCC, 0xCC);

    uint8_t req[] = { 0x09, 0x02 };
    uint8_t resp[32];
    uint16_t resp_len = 0;
    (void)isotp_transaction(0x7E0, 0x7E8, req, 2, resp, &resp_len, 1000);

    /* Ocekavame 2 odeslane ramce: SF request + FC */
    TEST_ASSERT_EQUAL_INT(2, mock_twai_get_tx_count());

    const twai_message_t *fc = mock_twai_get_tx_frame(1);
    TEST_ASSERT_EQUAL_HEX(0x7E0, fc->identifier);   /* FC jde na request ID */
    TEST_ASSERT_EQUAL_HEX(0x30, fc->data[0]);        /* FC PCI | FS=CTS(0) */
    TEST_ASSERT_EQUAL_HEX(0x00, fc->data[1]);        /* BS=0 (bez omezeni) */
    TEST_ASSERT_EQUAL_HEX(0x00, fc->data[2]);        /* STmin=0 */
}

/* ========================================================================= */
/*  Test 6: Chyby multi-frame prenosu                                        */
/* ========================================================================= */

void test_isotp_multiframe_sequence_error(void)
{
    setup_initialized_isotp();

    /* FF OK, ale CF ma SN=2 misto ocekavaneho SN=1 */
    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x0A, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6);
    mock_twai_inject_rx_frame(0x7E8,
        0x22, 0xA7, 0xA8, 0xA9, 0xAA, 0xCC, 0xCC, 0xCC);  /* SN=2 !! */

    uint8_t req[] = { 0x09, 0x02 };
    uint8_t resp[32];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_SEQUENCE, s);
}

void test_isotp_multiframe_overflow_rejects_oversized_ff(void)
{
    setup_initialized_isotp();

    /* FF_DL = 500 > ISOTP_MAX_PAYLOAD (256). Mel by se poslat FC OVFLW. */
    mock_twai_inject_rx_frame(0x7E8,
        0x11, 0xF4, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6);
    /* 0x11 = FF | high nibble FF_DL = 1, 0xF4 = low byte FF_DL = 500 (0x1F4) */

    uint8_t req[] = { 0x09, 0x02 };
    uint8_t resp[32];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_OVERFLOW, s);

    /* Musi byt odeslan FC s FS=OVERFLOW (0x32). */
    int tx_count = mock_twai_get_tx_count();
    TEST_ASSERT_TRUE(tx_count >= 2);
    const twai_message_t *fc = mock_twai_get_tx_frame(tx_count - 1);
    TEST_ASSERT_EQUAL_HEX(0x32, fc->data[0]);  /* FC | FS=OVERFLOW */
}

void test_isotp_multiframe_cf_timeout(void)
{
    setup_initialized_isotp();

    /* FF je tady, ale ocekavany CF nikdy neprijde. */
    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x0A, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6);

    uint8_t req[] = { 0x09, 0x02 };
    uint8_t resp[32];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT, s);
}

void test_isotp_multiframe_zero_ff_dl_is_invalid(void)
{
    setup_initialized_isotp();

    /* FF_DL = 0 — porusuje ISO 15765-2; musi vratit UNEXPECTED. */
    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

    uint8_t req[] = { 0x09, 0x02 };
    uint8_t resp[32];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_UNEXPECTED, s);
}

/* ========================================================================= */
/*  Test 7: Broadcast transakce (0x7DF)                                      */
/* ========================================================================= */

void test_isotp_broadcast_single_ecu_response(void)
{
    setup_initialized_isotp();

    mock_twai_inject_rx_frame(0x7E8,
        0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0xCC);

    uint8_t req[] = { 0x01, 0x00 };
    isotp_result_t result;
    isotp_status_t s = isotp_transaction_broadcast(req, 2, &result, 200);

    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(1, result.count);
    TEST_ASSERT_EQUAL_HEX(0x7E8, result.responses[0].rx_id);
    TEST_ASSERT_TRUE(result.responses[0].valid);
    TEST_ASSERT_EQUAL_INT(6, result.responses[0].len);

    /* Request mel byt odeslan na 0x7DF */
    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_EQUAL_HEX(0x7DF, tx->identifier);
}

void test_isotp_broadcast_multiple_ecus(void)
{
    setup_initialized_isotp();

    /* Dve ECU odpovi stejny PID dotaz (motor + prevodovka). */
    mock_twai_inject_rx_frame(0x7E8,
        0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0xCC);
    mock_twai_inject_rx_frame(0x7E9,
        0x06, 0x41, 0x00, 0x80, 0x00, 0x00, 0x00, 0xCC);

    uint8_t req[] = { 0x01, 0x00 };
    isotp_result_t result;
    isotp_status_t s = isotp_transaction_broadcast(req, 2, &result, 200);

    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(2, result.count);
    TEST_ASSERT_EQUAL_HEX(0x7E8, result.responses[0].rx_id);
    TEST_ASSERT_EQUAL_HEX(0x7E9, result.responses[1].rx_id);
}

void test_isotp_broadcast_ignores_non_obd_ids(void)
{
    setup_initialized_isotp();

    /* Airbag/ABS ramce (ID 0x200, 0x300) — nejsou v rozsahu 0x7E8..0x7EF. */
    mock_twai_inject_rx_frame(0x200,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    mock_twai_inject_rx_frame(0x300,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    /* Teprve pak skutecna odpoved ECU #1 */
    mock_twai_inject_rx_frame(0x7E8,
        0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0xCC);

    uint8_t req[] = { 0x01, 0x00 };
    isotp_result_t result;
    isotp_status_t s = isotp_transaction_broadcast(req, 2, &result, 200);

    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(1, result.count);
    TEST_ASSERT_EQUAL_HEX(0x7E8, result.responses[0].rx_id);
}

void test_isotp_broadcast_no_responses_is_timeout(void)
{
    setup_initialized_isotp();

    uint8_t req[] = { 0x01, 0x00 };
    isotp_result_t result;
    isotp_status_t s = isotp_transaction_broadcast(req, 2, &result, 100);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT, s);
    TEST_ASSERT_EQUAL_INT(0, result.count);
}

/* ========================================================================= */
/*  Test 8: BUS_OFF recovery                                                 */
/* ========================================================================= */

/*
 * Scenar: CAN sbernice spadne do BUS_OFF (napr. zkrat CAN-H a CAN-L).
 * Prvni twai_transmit() vrati ESP_ERR_INVALID_STATE. Kod v isotp_can_send()
 * musi:
 *   1) Zavolat twai_get_status_info() a zjistit BUS_OFF
 *   2) Zavolat twai_initiate_recovery()
 *   3) Pockat na TWAI_STATE_STOPPED
 *   4) Zavolat twai_start()
 *   5) Zkusit znovu twai_transmit() — tentokrat OK (mock uz neselze)
 *
 * Mock: mock_twai_next_tx_returns(ESP_ERR_INVALID_STATE) = jednorazove selhani.
 *       mock_twai_set_state(TWAI_STATE_BUS_OFF) = stav driveru.
 *       Po twai_initiate_recovery mock automaticky prepne do STOPPED.
 *       Druhy pokus twai_transmit uz projde (priznak se smazal).
 *
 * Overujeme, ze:
 *   - Transakce nakonec uspe (ISOTP_OK) diky recovery
 *   - twai_initiate_recovery() byla zavolana prave 1x
 */
void test_isotp_bus_off_recovery_succeeds(void)
{
    setup_initialized_isotp();

    /* Pripravime odpoved ECU — bude dorucena az po uspesnem retry TX. */
    mock_twai_inject_rx_frame(0x7E8,
        0x04, 0x41, 0x0C, 0x0C, 0x80, 0xCC, 0xCC, 0xCC);

    /* Prvni TX selze s BUS_OFF. */
    mock_twai_set_state(TWAI_STATE_BUS_OFF);
    mock_twai_next_tx_returns(ESP_ERR_INVALID_STATE);

    uint8_t req[] = { 0x01, 0x0C };
    uint8_t resp[16];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);

    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(4, resp_len);

    /* Recovery musi byt volana prave jednou. */
    TEST_ASSERT_EQUAL_INT(1, mock_twai_get_recovery_count());
}

/*
 * Scenar: TX selze s generickou chybou (ne BUS_OFF, napr. driver neni spusten).
 * isotp.c by mel vratit ISOTP_ERR_CAN_TX bez recovery pokusu.
 */
void test_isotp_tx_fail_non_bus_off_returns_can_tx_error(void)
{
    setup_initialized_isotp();

    /* TX selze, ale stav driveru NENI bus_off — napr. STOPPED. */
    mock_twai_set_state(TWAI_STATE_STOPPED);
    mock_twai_next_tx_returns(ESP_ERR_INVALID_STATE);

    uint8_t req[] = { 0x01, 0x0C };
    uint8_t resp[16];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 100);

    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_CAN_TX, s);
    /* Recovery nesmi byt zavolana — BUS_OFF to nebylo. */
    TEST_ASSERT_EQUAL_INT(0, mock_twai_get_recovery_count());
}

/* ========================================================================= */
/*  Test 9: SN wrap-around (vice nez 15 CF ramcu)                            */
/* ========================================================================= */

/*
 * ISO 15765-2 rika: Sequence Number (SN) se pocita 1,2,...,F,0,1,2,...
 * Po SN=0xF nasleduje SN=0x0 (wrap-around). Mnoho implementaci na tom
 * selhava kvuli off-by-one chybe.
 *
 * Pro vynuceni wrap-around potrebujeme payload > 6 + 15*7 = 111 bajtu.
 * Zvolime FF_DL = 120 bajtu:
 *   FF:       6 bajtu payloadu    (SN neexistuje)
 *   CF1..CF15: 15*7 = 105 bajtu  (SN = 1..0xF)
 *   CF16:     9 bajtu zbytku      (SN = 0x0 ← wrap-around!)
 *   CF17:     ale jen 120 - 6 - 15*7 = 9 zbyvajicich bajtu, takze
 *             staci CF16 (7 B) + CF17 (2 B), SN=0x0 a SN=0x1
 *
 * Prepocet: 120 - 6 = 114 bajtu v CF. 114/7 = 16.28 → 17 CF ramcu.
 *   CF 1..15:  SN=1..0xF  (105 B)
 *   CF 16:     SN=0x0     (7 B)   ← wrap-around!
 *   CF 17:     SN=0x1     (2 B)   ← jen zbytek
 *
 * Payload: byty 0x01, 0x02, ..., 0x78 (120 bajtu).
 */
void test_isotp_multiframe_sn_wrap_around(void)
{
    setup_initialized_isotp();

    /* Generujeme predvypocitany payload 120 bajtu: 0x01..0x78 */
    uint8_t expected_payload[120];
    for (int i = 0; i < 120; i++) expected_payload[i] = (uint8_t)(i + 1);

    /* FF: FF_DL = 120 = 0x0078 → [10 78 d0 d1 d2 d3 d4 d5] */
    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x78,
        expected_payload[0], expected_payload[1], expected_payload[2],
        expected_payload[3], expected_payload[4], expected_payload[5]);

    /* CF 1..15: SN = 0x1..0xF, po 7 bajtu */
    uint16_t off = 6;
    for (uint8_t sn = 1; sn <= 0x0F; sn++) {
        uint8_t frame[8];
        frame[0] = 0x20 | sn;
        for (int j = 0; j < 7; j++) {
            frame[1 + j] = (off + j < 120) ? expected_payload[off + j] : 0xCC;
        }
        mock_twai_inject_rx(0x7E8, frame, 8);
        off += 7;
    }
    /* off = 6 + 15*7 = 111 */

    /* CF 16: SN = 0x0 (wrap-around!) */
    {
        uint8_t frame[8];
        frame[0] = 0x20;  /* SN = 0 */
        for (int j = 0; j < 7; j++) {
            frame[1 + j] = (off + j < 120) ? expected_payload[off + j] : 0xCC;
        }
        mock_twai_inject_rx(0x7E8, frame, 8);
        off += 7;
    }
    /* off = 118 */

    /* CF 17: SN = 0x1, zbyvaji 2 bajty */
    {
        uint8_t frame[8];
        frame[0] = 0x21;  /* SN = 1 */
        for (int j = 0; j < 7; j++) {
            frame[1 + j] = (off + j < 120) ? expected_payload[off + j] : 0xCC;
        }
        mock_twai_inject_rx(0x7E8, frame, 8);
    }

    uint8_t req[] = { 0x09, 0x02 };
    uint8_t resp[256];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 2000);

    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(120, resp_len);
    TEST_ASSERT_EQUAL_MEMORY(expected_payload, resp, 120);
}

/* ========================================================================= */
/*  Test 10: Ochrana proti preteceni bufferu (malicious ECU)                  */
/* ========================================================================= */

/*
 * Scenar: ECU ohlasi FF_DL = 10 (10 bajtu payloadu), ale v CF posle
 * vice dat nez bylo deklarovano. Overujeme, ze isotp vrstva:
 *   1) Po prijmu 10 bajtu (6 z FF + 4 z CF1) ZASTAVI prijem
 *   2) NEPREPISE pamet za bufferem
 *   3) Vraci spravne ISOTP_OK s resp_len = 10
 *
 * Toto je bezpecnostni test — v praxi muze ECU odeslat vadny firmware
 * nebo utocnik na CAN sbernici (viz: Charlie Miller & Chris Valasek,
 * "Remote Exploitation of an Unaltered Passenger Vehicle", 2015).
 */
void test_isotp_multiframe_stops_at_declared_ff_dl(void)
{
    setup_initialized_isotp();

    /* FF: FF_DL = 10, prvnich 6 bajtu (0xA1..0xA6) */
    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x0A, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6);
    /* CF1: SN=1, zbyvaji 4 bajty (0xA7..0xAA) + 3 bajty "navic" */
    mock_twai_inject_rx_frame(0x7E8,
        0x21, 0xA7, 0xA8, 0xA9, 0xAA, 0xBB, 0xBB, 0xBB);
    /* CF2: Tenhle ramec by NEMEL byt zpracovan — FF_DL uz je splneno. */
    mock_twai_inject_rx_frame(0x7E8,
        0x22, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC);

    uint8_t req[] = { 0x09, 0x02 };
    uint8_t resp[32];
    memset(resp, 0xFE, sizeof(resp)); /* Sentinel pro detekci preteceni */
    uint16_t resp_len = 0;

    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);

    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(10, resp_len);  /* Presne 10 bajtu, ne vic */

    /* Zkontrolujeme, ze prvnich 10 bajtu je spravnych. */
    uint8_t expected[10] = {
        0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA
    };
    TEST_ASSERT_EQUAL_MEMORY(expected, resp, 10);

    /* Bajty ZA koncem payloadu NESMI byt prepsany — sentinel 0xFE. */
    TEST_ASSERT_EQUAL_HEX(0xFE, resp[10]);
    TEST_ASSERT_EQUAL_HEX(0xFE, resp[11]);
}

/* ========================================================================= */
/*  Test 11: Idempotence init / bezpecny deinit                              */
/* ========================================================================= */

/*
 * Defenzivni init: vrstva si pri init poradi i s tim, ze uz je inicializovana
 * (interni _isotp_force_cleanup_twai sebere stary driver). Test: dvakrat
 * po sobe init bez deinit — oba musi vratit ISOTP_OK.
 */
void test_isotp_init_idempotent_repeated_call(void)
{
    mock_twai_reset();
    isotp_set_log_level(ISOTP_LOG_NONE);

    TEST_ASSERT_EQUAL_INT(ISOTP_OK, isotp_init(500000, 5, 4));
    /* Bez deinit zavolam init znovu — defenzivni cleanup musi proběhnout. */
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, isotp_init(500000, 5, 4));

    isotp_deinit();
}

/*
 * isotp_deinit() je dle hlavickoveho komentare bezpecne volat i bez
 * predchoziho init. Test: rovnou deinit a overit, ze nedojde k padu.
 */
void test_isotp_deinit_safe_when_not_initialized(void)
{
    mock_twai_reset();
    isotp_set_log_level(ISOTP_LOG_NONE);
    /* Zatim nezavolane isotp_init — deinit musi byt no-op bez padu */
    isotp_deinit();

    /* A jeste jednou — porad bezpecne */
    isotp_deinit();

    /* Nasledny init musi normalne uspet */
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, isotp_init(500000, 5, 4));
    isotp_deinit();
}

/* ========================================================================= */
/*  Test 12: Hranice OBD-II ID filtru pri broadcastu                         */
/* ========================================================================= */

/*
 * Broadcast filter v isotp_can_recv() akceptuje rozsah 0x7E8..0x7EF
 * (= ISOTP_OBD_PHYS_RESP_BASE + ISOTP_MAX_ECU_RESPONSES).
 * Overujeme:
 *   - 0x7E7 (jeden pod) ignorovan
 *   - 0x7EF (horni hranice vcetne) prijat
 *   - 0x7F0 (jeden nad) ignorovan
 */
void test_isotp_broadcast_filter_rejects_id_below_obd_range(void)
{
    setup_initialized_isotp();

    /* 0x7E7 musi byt ignorovan (mimo dolni hranici 0x7E8). */
    mock_twai_inject_rx_frame(0x7E7,
        0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0xCC);

    uint8_t req[] = { 0x01, 0x00 };
    isotp_result_t result;
    isotp_status_t s = isotp_transaction_broadcast(req, 2, &result, 100);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT, s);
    TEST_ASSERT_EQUAL_INT(0, result.count);
}

void test_isotp_broadcast_filter_accepts_id_at_upper_boundary(void)
{
    setup_initialized_isotp();

    /* 0x7EF je posledni platne OBD response ID — musi byt prijato. */
    mock_twai_inject_rx_frame(0x7EF,
        0x06, 0x41, 0x00, 0x80, 0x00, 0x00, 0x00, 0xCC);

    uint8_t req[] = { 0x01, 0x00 };
    isotp_result_t result;
    isotp_status_t s = isotp_transaction_broadcast(req, 2, &result, 100);
    TEST_ASSERT_EQUAL_INT(ISOTP_OK, s);
    TEST_ASSERT_EQUAL_INT(1, result.count);
    TEST_ASSERT_EQUAL_HEX(0x7EF, result.responses[0].rx_id);
}

void test_isotp_broadcast_filter_rejects_id_above_obd_range(void)
{
    setup_initialized_isotp();

    /* 0x7F0 je o jeden mimo horni hranici 0x7EF — ignorovan. */
    mock_twai_inject_rx_frame(0x7F0,
        0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0xCC);

    uint8_t req[] = { 0x01, 0x00 };
    isotp_result_t result;
    isotp_status_t s = isotp_transaction_broadcast(req, 2, &result, 100);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT, s);
    TEST_ASSERT_EQUAL_INT(0, result.count);
}

/* ========================================================================= */
/*  Test 13: Padding bajty Flow Control ramce                                */
/* ========================================================================= */

/*
 * FC ramec ma fixni format: [0x30 BS STmin pad pad pad pad pad].
 * isotp_send_fc() pred zapisem PCI/BS/STmin nastavi cely buffer na
 * ISOTP_PADDING_BYTE (0xCC). Test overuje, ze bajty 3..7 obsahuji 0xCC
 * a ne nahodne hodnoty (memset musel projit pres cely ramec).
 */
void test_isotp_fc_padding_bytes_are_full(void)
{
    setup_initialized_isotp();

    /* Nasimulujeme FF, ktery vyvola odeslani FC. */
    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x0A, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6);
    mock_twai_inject_rx_frame(0x7E8,
        0x21, 0xA7, 0xA8, 0xA9, 0xAA, 0xCC, 0xCC, 0xCC);

    uint8_t req[] = { 0x09, 0x02 };
    uint8_t resp[32];
    uint16_t resp_len = 0;
    (void)isotp_transaction(0x7E0, 0x7E8, req, 2, resp, &resp_len, 1000);

    /* TX[1] musi byt FC ramec na 0x7E0, vsechny padding bajty 3..7 = 0xCC. */
    TEST_ASSERT_TRUE(mock_twai_get_tx_count() >= 2);
    const twai_message_t *fc = mock_twai_get_tx_frame(1);
    TEST_ASSERT_EQUAL_HEX(0x7E0, fc->identifier);
    TEST_ASSERT_EQUAL_HEX(0x30, fc->data[0]);
    TEST_ASSERT_EQUAL_HEX(0xCC, fc->data[3]);
    TEST_ASSERT_EQUAL_HEX(0xCC, fc->data[4]);
    TEST_ASSERT_EQUAL_HEX(0xCC, fc->data[5]);
    TEST_ASSERT_EQUAL_HEX(0xCC, fc->data[6]);
    TEST_ASSERT_EQUAL_HEX(0xCC, fc->data[7]);
}

/* ========================================================================= */
/*  Test 14: BUS_OFF recovery selze (recovery nikdy neprejde do STOPPED)      */
/* ========================================================================= */

/*
 * Scenar: prvni twai_transmit() vrati ESP_ERR_INVALID_STATE, isotp_can_send()
 * detekuje BUS_OFF a spusti recovery. Mock vsak nechava driver v BUS_OFF,
 * cili recovery nikdy neprejde do STOPPED. isotp.c po 500ms timeoutu
 * musi vratit ISOTP_ERR_CAN_TX a NEpokouset se o retransmit.
 */
void test_isotp_bus_off_recovery_failure_returns_can_tx(void)
{
    setup_initialized_isotp();

    /* TX selze a driver tvrdi BUS_OFF. */
    mock_twai_set_state(TWAI_STATE_BUS_OFF);
    mock_twai_next_tx_returns(ESP_ERR_INVALID_STATE);

    /* A recovery nepomuze — driver zustane v BUS_OFF. */
    mock_twai_set_recovery_keeps_busoff(true);

    uint8_t req[] = { 0x01, 0x0C };
    uint8_t resp[16];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 1000);

    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_CAN_TX, s);
    /* Recovery byla pokusene jednou (a selhala). */
    TEST_ASSERT_EQUAL_INT(1, mock_twai_get_recovery_count());
}

/* ========================================================================= */
/*  Test 15: N_As TX timeout (twai_transmit vrati ESP_ERR_TIMEOUT)            */
/* ========================================================================= */

/*
 * Realna ESP-IDF muze vratit ESP_ERR_TIMEOUT, kdyz TX fronta zustane plna
 * po N_As (25 ms). To NENI BUS_OFF — kod musi vratit ISOTP_ERR_CAN_TX
 * bez pokusu o recovery (recovery je vyhrazena pouze pro BUS_OFF).
 */
void test_isotp_tx_n_as_timeout_returns_can_tx_no_recovery(void)
{
    setup_initialized_isotp();

    /* Driver bezi normalne, jen prvni TX vrati TIMEOUT. */
    mock_twai_set_state(TWAI_STATE_RUNNING);
    mock_twai_next_tx_returns(ESP_ERR_TIMEOUT);

    uint8_t req[] = { 0x01, 0x0C };
    uint8_t resp[16];
    uint16_t resp_len = 0;
    isotp_status_t s = isotp_transaction(0x7E0, 0x7E8, req, 2,
                                          resp, &resp_len, 100);

    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_CAN_TX, s);
    /* Recovery NESMI byt zavolana — TIMEOUT neni BUS_OFF. */
    TEST_ASSERT_EQUAL_INT(0, mock_twai_get_recovery_count());
}

/* ========================================================================= */
/*  Test 16: status_str                                                      */
/* ========================================================================= */

void test_isotp_status_str_known_codes(void)
{
    TEST_ASSERT_STRING_EQUAL("OK",        isotp_status_str(ISOTP_OK));
    TEST_ASSERT_STRING_EQUAL("TIMEOUT",   isotp_status_str(ISOTP_ERR_TIMEOUT));
    TEST_ASSERT_STRING_EQUAL("OVERFLOW",  isotp_status_str(ISOTP_ERR_OVERFLOW));
    TEST_ASSERT_STRING_EQUAL("SEQUENCE",  isotp_status_str(ISOTP_ERR_SEQUENCE));
}

void test_isotp_status_str_unknown_code(void)
{
    TEST_ASSERT_STRING_EQUAL("UNKNOWN", isotp_status_str((isotp_status_t)99));
}

/* ========================================================================= */
/*  Hromadny registr testu pro test_main.c                                   */
/* ========================================================================= */

void run_isotp_tests(void)
{
    RUN_TEST(test_isotp_init_success);
    RUN_TEST(test_isotp_init_250k_also_ok);
    RUN_TEST(test_isotp_init_unsupported_baudrate);

    RUN_TEST(test_isotp_transaction_sf_request_has_correct_pci);
    RUN_TEST(test_isotp_transaction_sf_response_is_extracted);
    RUN_TEST(test_isotp_transaction_sf_max_7byte_request);

    RUN_TEST(test_isotp_transaction_timeout_on_no_response);

    RUN_TEST(test_isotp_transaction_not_initialized);
    RUN_TEST(test_isotp_transaction_null_request);
    RUN_TEST(test_isotp_transaction_null_response);

    RUN_TEST(test_isotp_transaction_multiframe_response);
    RUN_TEST(test_isotp_transaction_multiframe_sends_flow_control);

    RUN_TEST(test_isotp_multiframe_sequence_error);
    RUN_TEST(test_isotp_multiframe_overflow_rejects_oversized_ff);
    RUN_TEST(test_isotp_multiframe_cf_timeout);
    RUN_TEST(test_isotp_multiframe_zero_ff_dl_is_invalid);

    RUN_TEST(test_isotp_broadcast_single_ecu_response);
    RUN_TEST(test_isotp_broadcast_multiple_ecus);
    RUN_TEST(test_isotp_broadcast_ignores_non_obd_ids);
    RUN_TEST(test_isotp_broadcast_no_responses_is_timeout);

    /* BUS_OFF recovery */
    RUN_TEST(test_isotp_bus_off_recovery_succeeds);
    RUN_TEST(test_isotp_tx_fail_non_bus_off_returns_can_tx_error);

    /* SN wrap-around (>15 CF ramcu) */
    RUN_TEST(test_isotp_multiframe_sn_wrap_around);

    /* Buffer overflow ochrana */
    RUN_TEST(test_isotp_multiframe_stops_at_declared_ff_dl);

    /* Idempotence init / deinit */
    RUN_TEST(test_isotp_init_idempotent_repeated_call);
    RUN_TEST(test_isotp_deinit_safe_when_not_initialized);

    /* Hranice OBD-II ID filtru */
    RUN_TEST(test_isotp_broadcast_filter_rejects_id_below_obd_range);
    RUN_TEST(test_isotp_broadcast_filter_accepts_id_at_upper_boundary);
    RUN_TEST(test_isotp_broadcast_filter_rejects_id_above_obd_range);

    /* Padding FC ramce */
    RUN_TEST(test_isotp_fc_padding_bytes_are_full);

    /* Recovery / N_As */
    RUN_TEST(test_isotp_bus_off_recovery_failure_returns_can_tx);
    RUN_TEST(test_isotp_tx_n_as_timeout_returns_can_tx_no_recovery);

    RUN_TEST(test_isotp_status_str_known_codes);
    RUN_TEST(test_isotp_status_str_unknown_code);
}
