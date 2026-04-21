/**
 * @file test_obd2_diag.c
 * @brief Unit testy pro cisty dekoder DTC retezcu (SAE J2012).
 *
 * Testy pokryvaji:
 *   - obd2_decode_dtc_string(): vsechny ctyri typove prefixy (P/C/B/U)
 *   - Hranicni pripady: NULL vstup, NULL vystup, nulova hodnota
 *   - Specificke priklady z normy ISO 15031-5 / SAE J2012
 *
 * Tato funkce je "pure" -- nevolaje zadnou komunikaci, zkousi jen
 * bitove rozkladani. Integracni testy pro cteni DTC z ECU jsou
 * v test_obd2.c (s mockovanym ISO-TP).
 *
 * Reference:
 *   SAE J2012:2007 -- Diagnostic Trouble Codes Definitions
 *   ISO 15031-6:2015 -- DTC definitions (nasledovala SAE J2012)
 */

#include <string.h>
#include "unity_lite.h"
#include "obd2.h"

/* ========================================================================= */
/*  Test 1: P-kody (Powertrain -- motor, prevodovka)                         */
/* ========================================================================= */

/*
 * [0x01, 0x00] -> "P0100"
 * Byte 0 = 0x01:
 *   bity 7-6 = 00 -> 'P'
 *   bity 5-4 = 00 -> digit2 = '0'
 *   bity 3-0 = 0x1 -> digit3 = '1'
 * Byte 1 = 0x00:
 *   bity 7-4 = 0 -> digit4 = '0'
 *   bity 3-0 = 0 -> digit5 = '0'
 * Vyznam P0100: "Mass Air Flow Circuit Malfunction"
 */
void test_decode_dtc_P0100_maf_circuit(void)
{
    uint8_t raw[2] = { 0x01, 0x00 };
    char out[6];
    memset(out, 0xAA, sizeof(out));

    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("P0100", out);
}

/*
 * [0x01, 0x31] -> "P0131"
 * P0131: "O2 Sensor Circuit Low Voltage (Bank 1, Sensor 1)"
 */
void test_decode_dtc_P0131_o2_sensor(void)
{
    uint8_t raw[2] = { 0x01, 0x31 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("P0131", out);
}

/*
 * [0x03, 0x00] -> "P0300"
 * P0300: "Random/Multiple Cylinder Misfire Detected"
 */
void test_decode_dtc_P0300_misfire(void)
{
    uint8_t raw[2] = { 0x03, 0x00 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("P0300", out);
}

/*
 * [0x00, 0x00] -> "P0000" -- technicky platny, v praxi znamena padding.
 */
void test_decode_dtc_P0000_zero(void)
{
    uint8_t raw[2] = { 0x00, 0x00 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("P0000", out);
}

/*
 * [0x01, 0xFF] -> "P01FF"
 * Byte1 = 0xFF -> digit4 = 'F', digit5 = 'F'
 */
void test_decode_dtc_hex_digits_upper(void)
{
    uint8_t raw[2] = { 0x01, 0xFF };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("P01FF", out);
}

/* ========================================================================= */
/*  Test 2: C-kody (Chassis -- ABS, ESP, rizeni)                             */
/* ========================================================================= */

/*
 * [0x43, 0x00] -> "C0300"
 * Byte 0 = 0x43 = 0100_0011:
 *   bity 7-6 = 01 -> 'C'
 *   bity 5-4 = 00 -> digit2 = '0'
 *   bity 3-0 = 0x3 -> digit3 = '3'
 */
void test_decode_dtc_C0300_chassis(void)
{
    uint8_t raw[2] = { 0x43, 0x00 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("C0300", out);
}

/*
 * [0x51, 0x10] -> "C1110"
 * Byte 0 = 0x51 = 0101_0001:
 *   bity 7-6 = 01 -> 'C'
 *   bity 5-4 = 01 -> digit2 = '1' (manufacturer-specific)
 *   bity 3-0 = 0x1 -> digit3 = '1'
 * Byte 1 = 0x10 -> digit4='1', digit5='0'
 */
void test_decode_dtc_C1110_manufacturer(void)
{
    uint8_t raw[2] = { 0x51, 0x10 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("C1110", out);
}

/* ========================================================================= */
/*  Test 3: B-kody (Body -- airbag, klima, osvetleni)                        */
/* ========================================================================= */

/*
 * [0x80, 0x00] -> "B0000"
 * Byte 0 = 0x80 = 1000_0000:
 *   bity 7-6 = 10 -> 'B'
 *   bity 5-4 = 00 -> digit2 = '0'
 *   bity 3-0 = 0x0 -> digit3 = '0'
 */
void test_decode_dtc_B0000(void)
{
    uint8_t raw[2] = { 0x80, 0x00 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("B0000", out);
}

/*
 * [0xA7, 0x53] -> "B2753"
 * Byte 0 = 0xA7 = 1010_0111:
 *   bity 7-6 = 10 -> 'B'
 *   bity 5-4 = 10 -> digit2 = '2'
 *   bity 3-0 = 0x7 -> digit3 = '7'
 * Byte 1 = 0x53 -> '5','3'
 */
void test_decode_dtc_B2753(void)
{
    uint8_t raw[2] = { 0xA7, 0x53 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("B2753", out);
}

/* ========================================================================= */
/*  Test 4: U-kody (Network -- komunikacni sbernice)                         */
/* ========================================================================= */

/*
 * [0xC1, 0x23] -> "U0123"
 * Byte 0 = 0xC1 = 1100_0001:
 *   bity 7-6 = 11 -> 'U'
 *   bity 5-4 = 00 -> digit2 = '0'
 *   bity 3-0 = 0x1 -> digit3 = '1'
 * Byte 1 = 0x23 -> '2','3'
 * Vyznam U0123: "Lost Communication with Vehicle Dynamics Control Module"
 */
void test_decode_dtc_U0123_comm_lost(void)
{
    uint8_t raw[2] = { 0xC1, 0x23 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("U0123", out);
}

/*
 * [0xC0, 0x10] -> "U0010" (communication bus A off)
 */
void test_decode_dtc_U0010(void)
{
    uint8_t raw[2] = { 0xC0, 0x10 };
    char out[6];
    obd2_decode_dtc_string(raw, out);
    TEST_ASSERT_STRING_EQUAL("U0010", out);
}

/* ========================================================================= */
/*  Test 5: Null-terminace a velikost                                        */
/* ========================================================================= */

/*
 * Vystup MUSI byt presne 5 znaku + '\0' = 6 bytu.
 * Kontrolujeme, ze za '\0' v pozici [5] nic neprepise.
 */
void test_decode_dtc_null_terminated(void)
{
    uint8_t raw[2] = { 0x01, 0x23 };
    /* Pridame straznou hodnotu za bufferem pro detekci preteceni. */
    char out[10];
    memset(out, 0xFE, sizeof(out));

    obd2_decode_dtc_string(raw, out);

    TEST_ASSERT_EQUAL_INT('P',  out[0]);
    TEST_ASSERT_EQUAL_INT('0',  out[1]);
    TEST_ASSERT_EQUAL_INT('1',  out[2]);
    TEST_ASSERT_EQUAL_INT('2',  out[3]);
    TEST_ASSERT_EQUAL_INT('3',  out[4]);
    TEST_ASSERT_EQUAL_INT('\0', out[5]);
    /* Straznice za null-terminatorem musi zustat nedotcena. */
    TEST_ASSERT_EQUAL_INT((char)0xFE, out[6]);
}

/* ========================================================================= */
/*  Test 6: Hranicni pripady (NULL ukazatele)                                */
/* ========================================================================= */

/*
 * Pokud raw == NULL, funkce musi bezpecne zkrotit: zapise '\0' na out[0]
 * a vrati se bez chyby (ma void navratovy typ). Neni to undefined behavior.
 */
void test_decode_dtc_null_raw_writes_empty_string(void)
{
    char out[6];
    memset(out, 0xAA, sizeof(out));

    obd2_decode_dtc_string(NULL, out);
    TEST_ASSERT_EQUAL_INT('\0', out[0]);
}

/*
 * Oba NULL musi byt bezpecne zvladnuto (zadny crash).
 */
void test_decode_dtc_both_null_safe(void)
{
    /* Test spociva v tom, ze tohle vubec nespadne. */
    obd2_decode_dtc_string(NULL, NULL);
    /* Pokud jsme se sem dostali, je to ok. */
    TEST_ASSERT_TRUE(1);
}

/*
 * Pokud raw != NULL ale out == NULL, funkce musi rovnez bezpecne projit.
 */
void test_decode_dtc_null_out_safe(void)
{
    uint8_t raw[2] = { 0x01, 0x23 };
    obd2_decode_dtc_string(raw, NULL);
    /* Zadny crash = pass. */
    TEST_ASSERT_TRUE(1);
}

/* ========================================================================= */
/*  Test 7: Vsechny ctyri typove prefixy pro stejnou "hodnotu" 0x0123         */
/* ========================================================================= */

/*
 * Kontrola, ze jen dva horni bity prvniho bytu meni typovy prefix.
 * Pro "0123" v hex:
 *   digit2 = 0 -> bity 5-4 = 00
 *   digit3 = 1 -> bity 3-0 = 0x1
 *   byte1  = 0x23
 * Typ se meni pouze v bitech 7-6.
 */
void test_decode_dtc_all_prefixes(void)
{
    char out[6];

    /* P: 00 xx xxxx */
    uint8_t p[2] = { 0x01, 0x23 };
    obd2_decode_dtc_string(p, out);
    TEST_ASSERT_STRING_EQUAL("P0123", out);

    /* C: 01 xx xxxx  (0x01 + 0x40 = 0x41) */
    uint8_t c[2] = { 0x41, 0x23 };
    obd2_decode_dtc_string(c, out);
    TEST_ASSERT_STRING_EQUAL("C0123", out);

    /* B: 10 xx xxxx  (0x01 + 0x80 = 0x81) */
    uint8_t b[2] = { 0x81, 0x23 };
    obd2_decode_dtc_string(b, out);
    TEST_ASSERT_STRING_EQUAL("B0123", out);

    /* U: 11 xx xxxx  (0x01 + 0xC0 = 0xC1) */
    uint8_t u[2] = { 0xC1, 0x23 };
    obd2_decode_dtc_string(u, out);
    TEST_ASSERT_STRING_EQUAL("U0123", out);
}

/* ========================================================================= */
/*  Registr testu pro test_main.c                                            */
/* ========================================================================= */

void run_obd2_diag_tests(void)
{
    /* P-kody */
    RUN_TEST(test_decode_dtc_P0100_maf_circuit);
    RUN_TEST(test_decode_dtc_P0131_o2_sensor);
    RUN_TEST(test_decode_dtc_P0300_misfire);
    RUN_TEST(test_decode_dtc_P0000_zero);
    RUN_TEST(test_decode_dtc_hex_digits_upper);

    /* C-kody */
    RUN_TEST(test_decode_dtc_C0300_chassis);
    RUN_TEST(test_decode_dtc_C1110_manufacturer);

    /* B-kody */
    RUN_TEST(test_decode_dtc_B0000);
    RUN_TEST(test_decode_dtc_B2753);

    /* U-kody */
    RUN_TEST(test_decode_dtc_U0123_comm_lost);
    RUN_TEST(test_decode_dtc_U0010);

    /* Formatovaci vlastnosti */
    RUN_TEST(test_decode_dtc_null_terminated);

    /* NULL bezpecnost */
    RUN_TEST(test_decode_dtc_null_raw_writes_empty_string);
    RUN_TEST(test_decode_dtc_both_null_safe);
    RUN_TEST(test_decode_dtc_null_out_safe);

    /* Prefixy */
    RUN_TEST(test_decode_dtc_all_prefixes);
}
