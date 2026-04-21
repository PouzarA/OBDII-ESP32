/**
 * @file test_obd2_modes.c
 * @brief Integracni testy pokryvajici dalsi OBD-II sluzby a stavovy automat.
 *
 * Doplnuje test_obd2.c o:
 *   - Mode 02 (Freeze frame): obd2_get_freeze_frame_raw
 *   - Mode 07 (Pending DTC):  obd2_read_pending_dtc
 *   - Mode 0A (Permanent DTC): obd2_read_permanent_dtc
 *   - NRC 0x78 (responsePending) — ECU si rika o vic casu
 *   - ECU binding: obd2_set_ecu_address, obd2_bind_active_ecu, obd2_get_active_ecu
 *   - PID discovery: obd2_query_supported_pids, obd2_is_pid_supported
 *   - Multi-ECU agregatory: obd2_read_dtc_multi, obd2_read_vin_all, obd2_read_infotype_all
 *   - Raw query: obd2_query_raw, obd2_query_raw_ex (broadcast vs unicast)
 *
 * Klicova idea zustava stejna: mock TWAI dodava ramce presne tak,
 * jak by prisly z realne ECU, a my overujeme pretiznost vstupu i vystupu.
 */

#include <stdio.h>
#include <string.h>
#include "unity_lite.h"
#include "mock_twai.h"
#include "obd2.h"
#include "isotp.h"

/* ========================================================================= */
/*  Pomocne funkce                                                           */
/* ========================================================================= */

static void setup_obd2(void)
{
    mock_twai_reset();
    isotp_set_log_level(ISOTP_LOG_NONE);
    obd2_set_log_level(ISOTP_LOG_NONE);
    obd2_status_t s = obd2_init(500000, 5, 4);
    TEST_ASSERT_EQUAL_INT(OBD2_OK, s);
    obd2_set_timeout(200);
}

static void teardown_obd2(void)
{
    obd2_deinit();
}

/* ========================================================================= */
/*  Mode 02: Freeze Frame                                                    */
/* ========================================================================= */

/*
 * Request:  [03 02 0C 00 ...]   = SF|len=3, SID 02, PID 0C, frame# 00
 * Response: [05 42 0C 00 1A F8 CC CC]
 *           = SF|len=5, SID 42, PID 0C, frame# 00, data = 0x1AF8 = 1726 RPM
 *
 * Overujeme:
 *   - funkce vraci OBD2_OK
 *   - result->pid = 0x0C
 *   - data = 0x1A 0xF8
 *   - request byl odeslan na fyzickou adresu 0x7E0 s SID 02 a frame#=0
 */
void test_obd2_freeze_frame_raw_happy_path(void)
{
    setup_obd2();

    mock_twai_inject_rx_frame(0x7E8,
        0x05, 0x42, 0x0C, 0x00, 0x1A, 0xF8, 0xCC, 0xCC);

    obd2_pid_raw_t raw;
    memset(&raw, 0, sizeof(raw));
    obd2_status_t st = obd2_get_freeze_frame_raw(0x0C, 0x00, &raw);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_HEX(0x0C, raw.pid);
    TEST_ASSERT_TRUE(raw.data_len >= 2);
    TEST_ASSERT_EQUAL_HEX(0x1A, raw.data[0]);
    TEST_ASSERT_EQUAL_HEX(0xF8, raw.data[1]);

    /* Request byl unicast na 0x7E0 s [03 02 0C 00] */
    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_HEX(0x7E0, tx->identifier);
    TEST_ASSERT_EQUAL_HEX(0x03, tx->data[0]);   /* SF|len=3 */
    TEST_ASSERT_EQUAL_HEX(0x02, tx->data[1]);   /* SID Mode 02 */
    TEST_ASSERT_EQUAL_HEX(0x0C, tx->data[2]);   /* PID 0C */
    TEST_ASSERT_EQUAL_HEX(0x00, tx->data[3]);   /* frame# 0 */

    teardown_obd2();
}

void test_obd2_freeze_frame_raw_pid_mismatch_is_malformed(void)
{
    setup_obd2();

    /* ECU vrati spravne SID, ale jiny PID v echu — implementace
     * to musi detekovat a vratit OBD2_ERR_RESPONSE_MALFORMED. */
    mock_twai_inject_rx_frame(0x7E8,
        0x05, 0x42, 0x0D, 0x00, 0x1A, 0xF8, 0xCC, 0xCC);

    obd2_pid_raw_t raw;
    obd2_status_t st = obd2_get_freeze_frame_raw(0x0C, 0x00, &raw);
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_RESPONSE_MALFORMED, st);

    teardown_obd2();
}

void test_obd2_freeze_frame_raw_null_result(void)
{
    setup_obd2();
    obd2_status_t st = obd2_get_freeze_frame_raw(0x0C, 0x00, NULL);
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG, st);
    teardown_obd2();
}

void test_obd2_freeze_frame_raw_not_initialized(void)
{
    mock_twai_reset();
    obd2_set_log_level(ISOTP_LOG_NONE);
    obd2_deinit();

    obd2_pid_raw_t raw;
    obd2_status_t st = obd2_get_freeze_frame_raw(0x0C, 0x00, &raw);
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_NOT_INITIALIZED, st);
}

/* ========================================================================= */
/*  Mode 07: Pending DTC                                                     */
/* ========================================================================= */

/*
 * Mode 07 ma stejnou podobu jako Mode 03, jen SID je 0x07 (echo 0x47).
 * Request: SF|len=1, [01 07 ...]
 * Response: [04 47 01 03 00 ...] = SF|len=4, SID 47, count=1, DTC P0300
 */
void test_obd2_read_pending_dtc_one_dtc(void)
{
    setup_obd2();

    mock_twai_inject_rx_frame(0x7E8,
        0x04, 0x47, 0x01, 0x03, 0x00, 0xCC, 0xCC, 0xCC);

    obd2_dtc_t dtcs[10];
    uint8_t count = 0;
    obd2_status_t st = obd2_read_pending_dtc(dtcs, 10, &count);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_INT(1, count);
    TEST_ASSERT_STRING_EQUAL("P0300", dtcs[0].code);

    /* Request musi byt broadcast na 0x7DF se SID 0x07 */
    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_EQUAL_HEX(0x7DF, tx->identifier);
    TEST_ASSERT_EQUAL_HEX(0x01, tx->data[0]);  /* SF|len=1 */
    TEST_ASSERT_EQUAL_HEX(0x07, tx->data[1]);  /* SID Mode 07 */

    teardown_obd2();
}

void test_obd2_read_pending_dtc_zero_dtcs(void)
{
    setup_obd2();

    mock_twai_inject_rx_frame(0x7E8,
        0x02, 0x47, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC);

    obd2_dtc_t dtcs[10];
    uint8_t count = 0xFF;
    obd2_status_t st = obd2_read_pending_dtc(dtcs, 10, &count);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_INT(0, count);

    teardown_obd2();
}

void test_obd2_read_pending_dtc_null_pointer(void)
{
    setup_obd2();
    uint8_t count = 0;
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG,
                          obd2_read_pending_dtc(NULL, 10, &count));
    teardown_obd2();
}

/* ========================================================================= */
/*  Mode 0A: Permanent DTC                                                    */
/* ========================================================================= */

/*
 * Mode 0A funguje stejne jako 03/07, jen SID = 0x0A (echo 0x4A).
 * Permanentni DTC jsou ulozeny v OBD regulaci tak, ze nejdou smazat pres Mode 04.
 */
void test_obd2_read_permanent_dtc_one_dtc(void)
{
    setup_obd2();

    /* Response: [04 4A 01 P0420 ...]; P0420 = [04 20] */
    mock_twai_inject_rx_frame(0x7E8,
        0x04, 0x4A, 0x01, 0x04, 0x20, 0xCC, 0xCC, 0xCC);

    obd2_dtc_t dtcs[10];
    uint8_t count = 0;
    obd2_status_t st = obd2_read_permanent_dtc(dtcs, 10, &count);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_INT(1, count);
    TEST_ASSERT_STRING_EQUAL("P0420", dtcs[0].code);

    /* Request musi byt broadcast s SID 0x0A */
    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_EQUAL_HEX(0x7DF, tx->identifier);
    TEST_ASSERT_EQUAL_HEX(0x01, tx->data[0]);
    TEST_ASSERT_EQUAL_HEX(0x0A, tx->data[1]);

    teardown_obd2();
}

void test_obd2_read_permanent_dtc_zero_dtcs(void)
{
    setup_obd2();

    mock_twai_inject_rx_frame(0x7E8,
        0x02, 0x4A, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC);

    obd2_dtc_t dtcs[10];
    uint8_t count = 0xFF;
    obd2_status_t st = obd2_read_permanent_dtc(dtcs, 10, &count);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_INT(0, count);

    teardown_obd2();
}

/* ========================================================================= */
/*  NRC 0x78 (responsePending)                                               */
/* ========================================================================= */

/*
 * Response Pending je specialni NRC, ktery ECU pouziva, kdyz potrebuje
 * vice casu na zpracovani pozadavku (typicky mazani DTC, ulozeni do EEPROM).
 *
 * V soucasne implementaci je NRC 0x78 zachycen do last_nrc a vrati
 * OBD2_ERR_NEGATIVE_RESP — stejne jako kazdy jiny NRC. Tester musi vedet,
 * ze 0x78 nezpusobil chybu, ale pouze odlozeni odpovedi (a muze pozadavek
 * opakovat). Tento test dokumentuje aktualni chovani pro ucely defense.
 */
void test_obd2_nrc_0x78_response_pending_is_captured(void)
{
    setup_obd2();

    /* Negativni odpoved s NRC 0x78 na Mode 04 (clear DTC). */
    mock_twai_inject_rx_frame(0x7E8,
        0x03, 0x7F, 0x04, 0x78, 0xCC, 0xCC, 0xCC, 0xCC);

    obd2_status_t st = obd2_clear_dtc();
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_NEGATIVE_RESP, st);

    /* Last NRC musi obsahovat 0x78 a SID 0x04. */
    obd2_nrc_info_t nrc = obd2_get_last_nrc();
    TEST_ASSERT_EQUAL_HEX(0x04, nrc.request_sid);
    TEST_ASSERT_EQUAL_HEX(OBD2_NRC_RESPONSE_PENDING, nrc.nrc);
    TEST_ASSERT_STRING_EQUAL("responsePending", obd2_nrc_str(nrc.nrc));

    teardown_obd2();
}

/* ========================================================================= */
/*  ECU binding state machine                                                */
/* ========================================================================= */

void test_obd2_set_ecu_address_redirects_unicast(void)
{
    setup_obd2();

    /* Prepneme na ECU #2 (TCM, prevodovka): tx=0x7E1 / rx=0x7E9. */
    obd2_set_ecu_address(0x7E1, 0x7E9);

    /* Pripravime odpoved od TCM (0x7E9). */
    mock_twai_inject_rx_frame(0x7E9,
        0x04, 0x41, 0x0C, 0x0C, 0x80, 0xCC, 0xCC, 0xCC);

    obd2_pid_raw_t raw;
    obd2_status_t st = obd2_get_pid_raw(0x0C, &raw);
    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);

    /* Request musel jit na 0x7E1, ne na vychozi 0x7E0. */
    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_EQUAL_HEX(0x7E1, tx->identifier);

    /* Active ECU se nastavila. */
    uint32_t tx_id = 0, rx_id = 0;
    bool bound = obd2_get_active_ecu(&tx_id, &rx_id);
    TEST_ASSERT_TRUE(bound);
    TEST_ASSERT_EQUAL_HEX(0x7E1, tx_id);
    TEST_ASSERT_EQUAL_HEX(0x7E9, rx_id);

    teardown_obd2();
}

/*
 * Bind active ECU: pokud zadny discovery jeste neprobehl, akceptujeme
 * libovolne ID v rozsahu 0x7E8..0x7EF. Mimo rozsah => INVALID_ARG.
 */
void test_obd2_bind_active_ecu_rejects_id_outside_range(void)
{
    setup_obd2();

    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG, obd2_bind_active_ecu(0x7E7));
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG, obd2_bind_active_ecu(0x7F0));
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG, obd2_bind_active_ecu(0x123));

    teardown_obd2();
}

void test_obd2_bind_active_ecu_accepts_valid_id_when_no_discovery(void)
{
    setup_obd2();

    /* Bez predchozi discovery (count==0) — vyber 0x7E9 musi projit. */
    obd2_status_t st = obd2_bind_active_ecu(0x7E9);
    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);

    uint32_t tx_id = 0, rx_id = 0;
    TEST_ASSERT_TRUE(obd2_get_active_ecu(&tx_id, &rx_id));
    TEST_ASSERT_EQUAL_HEX(0x7E1, tx_id);  /* tx = rx - 8 */
    TEST_ASSERT_EQUAL_HEX(0x7E9, rx_id);

    teardown_obd2();
}

/* ========================================================================= */
/*  PID discovery (Mode 01 PID $00)                                          */
/* ========================================================================= */

/*
 * obd2_query_supported_pids posila broadcast PID $00 a pripadne $20/$40 podle
 * "next range" bitu v odpovedi.
 *
 * Scenar: jedna ECU odpovi na PID $00 bitmaskou, kde:
 *   - bit pro PID $0C nastaven  (RPM)
 *   - bit pro PID $20 nenastaven (zadny dalsi rozsah)
 *
 * PID $0C ma index 12 v 1..32, takze v bitmasce 4-bajt:
 *   bit pos = 31 - ((12 - 1) % 32) = 20
 *   bit 20 v 32-bit: 0x00100000 (high byte ma bit pos 7 = bit 31)
 * Rozlozene do 4 bajtu A B C D (MSB first):
 *   A = bity 31..24 (PID 1..8)   -> 0x00 (zadny z 1..8)
 *   B = bity 23..16 (PID 9..16)  -> 0x10 (PID 12 = bit 20 = bit 4 v B)
 *
 * Pocitejme bit pos pro PID 12: 31 - 11 = 20. byte = (31-20)/8 = 1, bit_in_byte = 20 % 8 = 4
 * Takze A=0x00, B=0x10, C=0x00, D=0x00.
 *
 * Zaroven nastavim bit pro PID $20: 31 - ((32 - 1) % 32) = 31 - 31 = 0 → bit 0 = LSB of D.
 * Pro tento test bit $20 NEnastavovat (chceme, aby discovery skoncila po prvnim rozsahu).
 *
 * Response: [06 41 00 00 10 00 00 CC]
 */
void test_obd2_query_supported_pids_marks_supported_bits(void)
{
    setup_obd2();

    /* Bitmask: PID $0C podporovan, ostatni ne, zadny dalsi rozsah */
    mock_twai_inject_rx_frame(0x7E8,
        0x06, 0x41, 0x00, 0x00, 0x10, 0x00, 0x00, 0xCC);

    obd2_status_t st = obd2_query_supported_pids();
    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);

    TEST_ASSERT_TRUE(obd2_is_pid_supported(0x0C));
    /* Sousedni PIDy nejsou podporovany. */
    TEST_ASSERT_FALSE(obd2_is_pid_supported(0x0B));
    TEST_ASSERT_FALSE(obd2_is_pid_supported(0x0D));
    /* PID, ktery ECU rozhodne nepodporuje. */
    TEST_ASSERT_FALSE(obd2_is_pid_supported(0x5A));

    teardown_obd2();
}

void test_obd2_is_pid_supported_returns_false_before_query(void)
{
    setup_obd2();

    /* Zadny query_supported_pids nebyl zavolan -> bitmask nebyl naplnen. */
    TEST_ASSERT_FALSE(obd2_is_pid_supported(0x0C));
    TEST_ASSERT_FALSE(obd2_is_pid_supported(0x0D));

    teardown_obd2();
}

/* ========================================================================= */
/*  Multi-ECU agregatory                                                     */
/* ========================================================================= */

/*
 * obd2_read_dtc_multi posle broadcast Mode 03 (nebo 07) a uchova DTC
 * separatne pro kazdou odpovidajici ECU.
 *
 * Scenar: ECU 0x7E8 odpovi 1 DTC (P0300), ECU 0x7E9 odpovi 0 DTC.
 *   ECU1: [04 43 01 03 00 CC CC CC]
 *   ECU2: [02 43 00 CC CC CC CC CC]
 *
 * Pozn.: read_dtc_multi pouziva interne timeout 1000 ms (viz obd2_diag.c).
 * Mock ale nepotrebuje cekat skutecny cas — RX fronta ma odpovedi pripravene.
 */
void test_obd2_read_dtc_multi_two_ecus(void)
{
    setup_obd2();

    mock_twai_inject_rx_frame(0x7E8,
        0x04, 0x43, 0x01, 0x03, 0x00, 0xCC, 0xCC, 0xCC);
    mock_twai_inject_rx_frame(0x7E9,
        0x02, 0x43, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC);

    obd2_multi_ecu_dtc_t result;
    memset(&result, 0, sizeof(result));
    obd2_status_t st = obd2_read_dtc_multi(OBD2_SID_READ_DTC, &result);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_INT(2, result.ecu_count);

    /* ECU 0x7E8 musi mit prave 1 DTC (P0300) */
    bool found_ecu1 = false, found_ecu2 = false;
    for (int i = 0; i < result.ecu_count; i++) {
        if (result.ecus[i].ecu_id == 0x7E8) {
            found_ecu1 = true;
            TEST_ASSERT_EQUAL_INT(1, result.ecus[i].count);
            TEST_ASSERT_STRING_EQUAL("P0300", result.ecus[i].dtcs[0].code);
        } else if (result.ecus[i].ecu_id == 0x7E9) {
            found_ecu2 = true;
            TEST_ASSERT_EQUAL_INT(0, result.ecus[i].count);
        }
    }
    TEST_ASSERT_TRUE(found_ecu1);
    TEST_ASSERT_TRUE(found_ecu2);

    teardown_obd2();
}

void test_obd2_read_dtc_multi_null_result(void)
{
    setup_obd2();
    obd2_status_t st = obd2_read_dtc_multi(OBD2_SID_READ_DTC, NULL);
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG, st);
    teardown_obd2();
}

void test_obd2_read_dtc_multi_not_initialized(void)
{
    mock_twai_reset();
    obd2_set_log_level(ISOTP_LOG_NONE);
    obd2_deinit();

    obd2_multi_ecu_dtc_t result;
    obd2_status_t st = obd2_read_dtc_multi(OBD2_SID_READ_DTC, &result);
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_NOT_INITIALIZED, st);
}

/*
 * obd2_read_vin_all: broadcast Mode 09 InfoType 02 a sber VIN ze vsech ECU.
 *
 * Scenar: jedna ECU (0x7E8) posle multi-frame VIN "WVWZZZ3CZWE123456".
 *   FF :  [10 14 49 02 01 W V W]
 *   CF1:  [21 Z Z Z 3 C Z W]
 *   CF2:  [22 E 1 2 3 4 5 6]
 */
void test_obd2_read_vin_all_single_ecu(void)
{
    setup_obd2();
    obd2_set_timeout(1000);

    mock_twai_inject_rx_frame(0x7E8,
        0x10, 0x14, 0x49, 0x02, 0x01, 'W', 'V', 'W');
    mock_twai_inject_rx_frame(0x7E8,
        0x21, 'Z', 'Z', 'Z', '3', 'C', 'Z', 'W');
    mock_twai_inject_rx_frame(0x7E8,
        0x22, 'E', '1', '2', '3', '4', '5', '6');

    obd2_vin_list_t list;
    memset(&list, 0, sizeof(list));
    obd2_status_t st = obd2_read_vin_all(&list);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_INT(1, list.count);
    TEST_ASSERT_EQUAL_HEX(0x7E8, list.items[0].rx_id);
    TEST_ASSERT_STRING_EQUAL("WVWZZZ3CZWE123456", list.items[0].vin);

    teardown_obd2();
}

void test_obd2_read_vin_all_null_list(void)
{
    setup_obd2();
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG, obd2_read_vin_all(NULL));
    teardown_obd2();
}

/*
 * obd2_read_infotype_all je generic kanal pro vsechny Mode 09 InfoTypes.
 * Test: InfoType $00 (supported InfoTypes bitmask) — bez NODI.
 */
void test_obd2_read_infotype_all_supported_bitmap(void)
{
    setup_obd2();

    /* Response: [06 49 00 BE 3F A8 13 CC] = SID 49, InfoType 00, 4-bajt bitmap */
    mock_twai_inject_rx_frame(0x7E8,
        0x06, 0x49, 0x00, 0xBE, 0x3F, 0xA8, 0x13, 0xCC);

    obd2_infotype_list_t list;
    memset(&list, 0, sizeof(list));
    obd2_status_t st = obd2_read_infotype_all(0x00, &list);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_INT(1, list.count);
    TEST_ASSERT_EQUAL_HEX(0x7E8, list.items[0].rx_id);
    TEST_ASSERT_EQUAL_HEX(0x00, list.items[0].infotype);
    /* InfoType $00 nema NODI — payload jsou 4 bitmask bajty. */
    TEST_ASSERT_EQUAL_INT(4, list.items[0].data_len);
    TEST_ASSERT_EQUAL_HEX(0xBE, list.items[0].data[0]);

    teardown_obd2();
}

/* ========================================================================= */
/*  Raw query (obd2_query_raw, obd2_query_raw_ex)                            */
/* ========================================================================= */

/*
 * obd2_query_raw posila pozadavek na FYZICKOU adresu (0x7E0) a vraci
 * surova data odpovedi vcetne SID echo.
 */
void test_obd2_query_raw_unicast_returns_full_response(void)
{
    setup_obd2();

    mock_twai_inject_rx_frame(0x7E8,
        0x04, 0x41, 0x0C, 0x0C, 0x80, 0xCC, 0xCC, 0xCC);

    obd2_raw_response_t res;
    memset(&res, 0, sizeof(res));
    obd2_status_t st = obd2_query_raw(0x01, 0x0C, &res);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_FALSE(res.is_negative);
    TEST_ASSERT_TRUE(res.data_len >= 4);
    TEST_ASSERT_EQUAL_HEX(0x41, res.data[0]);  /* SID echo */
    TEST_ASSERT_EQUAL_HEX(0x0C, res.data[1]);  /* PID echo */
    TEST_ASSERT_EQUAL_HEX(0x0C, res.data[2]);  /* A */
    TEST_ASSERT_EQUAL_HEX(0x80, res.data[3]);  /* B */

    /* Default unicast — request musel jit na 0x7E0. */
    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_EQUAL_HEX(0x7E0, tx->identifier);

    teardown_obd2();
}

void test_obd2_query_raw_ex_broadcast_uses_7df(void)
{
    setup_obd2();

    mock_twai_inject_rx_frame(0x7E8,
        0x04, 0x41, 0x0C, 0x0C, 0x80, 0xCC, 0xCC, 0xCC);

    obd2_raw_response_t res;
    obd2_status_t st = obd2_query_raw_ex(0x01, 0x0C, &res, true);

    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_EQUAL_HEX(0x7E8, res.rx_id);

    /* Broadcast — request musel jit na 0x7DF. */
    const twai_message_t *tx = mock_twai_get_tx_frame(0);
    TEST_ASSERT_EQUAL_HEX(0x7DF, tx->identifier);

    teardown_obd2();
}

void test_obd2_query_raw_negative_response_is_captured(void)
{
    setup_obd2();

    /* ECU vrati NRC 0x12 (subFunctionNotSupported) na PID $5A. */
    mock_twai_inject_rx_frame(0x7E8,
        0x03, 0x7F, 0x01, 0x12, 0xCC, 0xCC, 0xCC, 0xCC);

    obd2_raw_response_t res;
    obd2_status_t st = obd2_query_raw(0x01, 0x5A, &res);

    /* obd2_query_raw "zaobaluje" NRC do uspesneho navratu pro terminal. */
    TEST_ASSERT_EQUAL_INT(OBD2_OK, st);
    TEST_ASSERT_TRUE(res.is_negative);
    TEST_ASSERT_EQUAL_HEX(0x12, res.nrc_code);

    teardown_obd2();
}

void test_obd2_query_raw_null_pointer(void)
{
    setup_obd2();
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG, obd2_query_raw(0x01, 0x0C, NULL));
    teardown_obd2();
}

/* ========================================================================= */
/*  Registr testu pro test_main.c                                            */
/* ========================================================================= */

void run_obd2_modes_tests(void)
{
    /* Mode 02 — Freeze frame */
    RUN_TEST(test_obd2_freeze_frame_raw_happy_path);
    RUN_TEST(test_obd2_freeze_frame_raw_pid_mismatch_is_malformed);
    RUN_TEST(test_obd2_freeze_frame_raw_null_result);
    RUN_TEST(test_obd2_freeze_frame_raw_not_initialized);

    /* Mode 07 — Pending DTC */
    RUN_TEST(test_obd2_read_pending_dtc_one_dtc);
    RUN_TEST(test_obd2_read_pending_dtc_zero_dtcs);
    RUN_TEST(test_obd2_read_pending_dtc_null_pointer);

    /* Mode 0A — Permanent DTC */
    RUN_TEST(test_obd2_read_permanent_dtc_one_dtc);
    RUN_TEST(test_obd2_read_permanent_dtc_zero_dtcs);

    /* NRC 0x78 — responsePending */
    RUN_TEST(test_obd2_nrc_0x78_response_pending_is_captured);

    /* ECU binding state machine */
    RUN_TEST(test_obd2_set_ecu_address_redirects_unicast);
    RUN_TEST(test_obd2_bind_active_ecu_rejects_id_outside_range);
    RUN_TEST(test_obd2_bind_active_ecu_accepts_valid_id_when_no_discovery);

    /* PID discovery */
    RUN_TEST(test_obd2_query_supported_pids_marks_supported_bits);
    RUN_TEST(test_obd2_is_pid_supported_returns_false_before_query);

    /* Multi-ECU agregatory */
    RUN_TEST(test_obd2_read_dtc_multi_two_ecus);
    RUN_TEST(test_obd2_read_dtc_multi_null_result);
    RUN_TEST(test_obd2_read_dtc_multi_not_initialized);
    RUN_TEST(test_obd2_read_vin_all_single_ecu);
    RUN_TEST(test_obd2_read_vin_all_null_list);
    RUN_TEST(test_obd2_read_infotype_all_supported_bitmap);

    /* Raw query */
    RUN_TEST(test_obd2_query_raw_unicast_returns_full_response);
    RUN_TEST(test_obd2_query_raw_ex_broadcast_uses_7df);
    RUN_TEST(test_obd2_query_raw_negative_response_is_captured);
    RUN_TEST(test_obd2_query_raw_null_pointer);
}
