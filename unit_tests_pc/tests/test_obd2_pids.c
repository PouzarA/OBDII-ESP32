/**
 * @file test_obd2_pids.c
 * @brief Unit testy pro cistou dekodovaci vrstvu OBD-II (zadna komunikace).
 *
 * Testy pokryvaji:
 *   - obd2_decode_pid_value(): vsechny formaty (LINEAR_1B, LINEAR_2B,
 *     SIGNED_OFFSET_1B, SIGNED_2B, BIT_ENCODED, O2_CONV, O2_WIDE_EQ_V/I, CONFIG)
 *   - obd2_decode_pid_secondary(): O2 senzorove PIDy ($14-$1B, $24-$2B, $34-$3B)
 *   - Hranicni pripady: NULL data, data_len == 0, neznamy PID, neznamy format
 *   - obd2_get_monitor_status() parsing s predanymi raw daty (bez ECU)
 *
 * Vsechny tyto funkce jsou "pure" -- nevolaji ISO-TP ani TWAI, takze
 * netreba mockovat komunikaci. Testuji se primo vzorce z ISO 15031-5 Priloha B.
 *
 * Reference: ISO 15031-5:2006 Priloha B, tabulky B.4 -- B.72
 */

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "unity_lite.h"
#include "obd2.h"

/* Tolerance pro porovnavani floatu -- odpovida max. 1 LSB presnosti vzorce. */
#define FLT_TOL     0.01f

/* ========================================================================= */
/*  Test 1: LINEAR_1B format (PID $05 teplota chladici kapaliny)             */
/* ========================================================================= */

/*
 * Vzorec: val = A * 1.0 + (-40)  -> rozsah -40 az +215 stupnu C.
 * Priklady:
 *   A = 40  -> 0 stupnu C (bod mrazu)
 *   A = 80  -> 40 stupnu C (provozni teplota)
 *   A = 130 -> 90 stupnu C (zahrata teplota)
 */
void test_decode_pid_05_coolant_temp_at_zero(void)
{
    uint8_t data[1] = { 40 };
    float v = obd2_decode_pid_value(0x05, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v, FLT_TOL);
}

void test_decode_pid_05_coolant_temp_at_90deg(void)
{
    uint8_t data[1] = { 130 };
    float v = obd2_decode_pid_value(0x05, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, v, FLT_TOL);
}

void test_decode_pid_05_coolant_temp_minimum(void)
{
    /* A=0 -> -40 stupnu C (nejnizsi mozna hodnota dle ISO) */
    uint8_t data[1] = { 0 };
    float v = obd2_decode_pid_value(0x05, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(-40.0f, v, FLT_TOL);
}

/* ========================================================================= */
/*  Test 2: LINEAR_2B format (PID $0C otacky motoru)                         */
/* ========================================================================= */

/*
 * Vzorec: val = (256*A + B) * 0.25
 * Priklady z ISO 15031-5 Priloha B:
 *   [0x0C, 0x80] = 0x0C80 = 3200 -> 800 RPM (volnobeh)
 *   [0x1A, 0xF8] = 0x1AF8 = 6904 -> 1726 RPM (mirna zatez)
 *   [0x44, 0x00] = 0x4400 = 17408 -> 4352 RPM (vysoke otacky)
 */
void test_decode_pid_0C_rpm_idle(void)
{
    uint8_t data[2] = { 0x0C, 0x80 };
    float v = obd2_decode_pid_value(0x0C, data, 2);
    TEST_ASSERT_EQUAL_FLOAT(800.0f, v, FLT_TOL);
}

void test_decode_pid_0C_rpm_1726(void)
{
    uint8_t data[2] = { 0x1A, 0xF8 };
    float v = obd2_decode_pid_value(0x0C, data, 2);
    TEST_ASSERT_EQUAL_FLOAT(1726.0f, v, FLT_TOL);
}

void test_decode_pid_0C_rpm_zero(void)
{
    /* Motor vypnuty -> 0 RPM */
    uint8_t data[2] = { 0x00, 0x00 };
    float v = obd2_decode_pid_value(0x0C, data, 2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v, FLT_TOL);
}

/* ========================================================================= */
/*  Test 3: LINEAR_1B (PID $0D rychlost vozidla)                             */
/* ========================================================================= */

/*
 * Vzorec: val = A * 1.0 + 0  -> rozsah 0 az 255 km/h, primy byte.
 */
void test_decode_pid_0D_speed_50kmh(void)
{
    uint8_t data[1] = { 50 };
    float v = obd2_decode_pid_value(0x0D, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, v, FLT_TOL);
}

void test_decode_pid_0D_speed_max(void)
{
    uint8_t data[1] = { 255 };
    float v = obd2_decode_pid_value(0x0D, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(255.0f, v, FLT_TOL);
}

/* ========================================================================= */
/*  Test 4: SIGNED_OFFSET_1B (PID $06 STFT bank 1)                           */
/* ========================================================================= */

/*
 * Vzorec: val = (A - 128) * 100/128
 * Priklady:
 *   A = 128 -> 0 % (zadna korekce, idealni smes)
 *   A = 0   -> -100 % (maximalne ochuzena smes)
 *   A = 255 -> +99.2 % (maximalne obohacena smes)
 *   A = 64  -> -50 % (znacne ochuzeni)
 */
void test_decode_pid_06_stft_zero(void)
{
    uint8_t data[1] = { 128 };
    float v = obd2_decode_pid_value(0x06, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v, FLT_TOL);
}

void test_decode_pid_06_stft_minus_100(void)
{
    uint8_t data[1] = { 0 };
    float v = obd2_decode_pid_value(0x06, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(-100.0f, v, FLT_TOL);
}

void test_decode_pid_06_stft_positive(void)
{
    /* (192 - 128) * 100/128 = 50 % */
    uint8_t data[1] = { 192 };
    float v = obd2_decode_pid_value(0x06, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, v, FLT_TOL);
}

/* ========================================================================= */
/*  Test 5: SIGNED_2B (PID $32 tlak par EVAP)                                */
/* ========================================================================= */

/*
 * Vzorec: val = (int16_t)(256*A + B) * 0.25
 * Priklady:
 *   [0x00, 0x00] = 0 -> 0 Pa
 *   [0xFF, 0xFF] = -1 -> -0.25 Pa
 *   [0x01, 0x00] = +256 -> +64 Pa
 *   [0xFF, 0x00] = (int16_t)0xFF00 = -256 -> -64 Pa
 */
void test_decode_pid_32_evap_zero(void)
{
    uint8_t data[2] = { 0x00, 0x00 };
    float v = obd2_decode_pid_value(0x32, data, 2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v, FLT_TOL);
}

void test_decode_pid_32_evap_positive(void)
{
    uint8_t data[2] = { 0x01, 0x00 };  /* 256 * 0.25 = 64 */
    float v = obd2_decode_pid_value(0x32, data, 2);
    TEST_ASSERT_EQUAL_FLOAT(64.0f, v, FLT_TOL);
}

void test_decode_pid_32_evap_negative(void)
{
    /* 0xFF00 jako int16_t = -256, * 0.25 = -64 Pa */
    uint8_t data[2] = { 0xFF, 0x00 };
    float v = obd2_decode_pid_value(0x32, data, 2);
    TEST_ASSERT_EQUAL_FLOAT(-64.0f, v, FLT_TOL);
}

/* ========================================================================= */
/*  Test 6: BIT_ENCODED (PID $01 monitor status)                             */
/* ========================================================================= */

/*
 * BIT_ENCODED vraci surovych 32 bitu pretypovanych na float.
 * Klient je musi pretypovat zpet na uint32_t pro bitovou analyzu.
 */
void test_decode_pid_01_bitencoded_preserves_bytes(void)
{
    /* [A=0x83, B=0x07, C=0xFF, D=0x00] = 0x8307FF00 = 2198339328 */
    uint8_t data[4] = { 0x83, 0x07, 0xFF, 0x00 };
    float v = obd2_decode_pid_value(0x01, data, 4);

    /*
     * Float neni presne reprezentovatelny pro velka 32bit cisla — float
     * ma pouze ~24 bitu mantissy. Povolime toleranci 1024, coz odpovida
     * ztrate presnosti u hodnot blizkych 2^31.
     *   0x8307FF00 = 131*2^24 + 7*2^16 + 255*2^8 + 0 = 2198339328
     */
    TEST_ASSERT_EQUAL_FLOAT(2198339328.0f, v, 1024.0f);
}

/* ========================================================================= */
/*  Test 7: O2_CONV (PID $14 -- B1S1 napeti + STFT)                          */
/* ========================================================================= */

/*
 * Primarni hodnota: napeti = A * 0.005 V (rozsah 0..1.275 V)
 * Sekundarni hodnota: STFT = (B - 128) * 100/128 %
 *
 * Priklad: [A=0x90, B=0x80] -> 0.72 V, 0 %
 */
void test_decode_pid_14_o2_voltage(void)
{
    uint8_t data[2] = { 0x90, 0x80 };
    float v = obd2_decode_pid_value(0x14, data, 2);
    /* 0x90 = 144, 144 * 0.005 = 0.72 V */
    TEST_ASSERT_EQUAL_FLOAT(0.72f, v, FLT_TOL);
}

void test_decode_pid_14_o2_stft_secondary(void)
{
    uint8_t data[2] = { 0x90, 0x80 };
    float s = obd2_decode_pid_secondary(0x14, data, 2);
    /* B = 128 -> STFT = 0 % */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s, FLT_TOL);
}

void test_decode_pid_14_o2_stft_negative(void)
{
    /* B = 64 -> STFT = (64-128)*100/128 = -50 % */
    uint8_t data[2] = { 0x80, 64 };
    float s = obd2_decode_pid_secondary(0x14, data, 2);
    TEST_ASSERT_EQUAL_FLOAT(-50.0f, s, FLT_TOL);
}

/* ========================================================================= */
/*  Test 8: O2_WIDE_EQ_V (PID $24 -- sirokopasmovy, lambda + napeti)         */
/* ========================================================================= */

/*
 * Primarni: lambda = (256*A + B) * 2/65535  (rozsah 0..2.0)
 * Sekundarni: napeti = (256*C + D) * 8/65535  (rozsah 0..8.0 V)
 *
 * Priklad lambda = 1.0 (stechiometrie): 65535/2 = 0x8000 -> A=0x80, B=0x00
 */
void test_decode_pid_24_lambda_stoichiometric(void)
{
    uint8_t data[4] = { 0x80, 0x00, 0x00, 0x00 };
    float v = obd2_decode_pid_value(0x24, data, 4);
    /* (0x8000 * 2) / 65535 = 65536/65535 ~ 1.0 */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, v, FLT_TOL);
}

void test_decode_pid_24_voltage_secondary_zero(void)
{
    uint8_t data[4] = { 0x80, 0x00, 0x00, 0x00 };
    float s = obd2_decode_pid_secondary(0x24, data, 4);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s, FLT_TOL);
}

void test_decode_pid_24_voltage_secondary_half(void)
{
    /* CD = 0x8000 -> napeti = 0x8000 * 8/65535 ~ 4.0 V (polovina z 8 V) */
    uint8_t data[4] = { 0x80, 0x00, 0x80, 0x00 };
    float s = obd2_decode_pid_secondary(0x24, data, 4);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, s, FLT_TOL);
}

/* ========================================================================= */
/*  Test 9: O2_WIDE_EQ_I (PID $34 -- sirokopasmovy, lambda + proud)          */
/* ========================================================================= */

/*
 * Primarni: lambda stejne jako $24
 * Sekundarni: proud = (int16_t)(256*C + D) * 128/32768 mA
 *   Dle Annex B: $8000 = 0 mA; tj. CD je surove int16 signed.
 *   Pozn.: nase implementace pouziva (int16_t)cd * (128/32768),
 *   takze CD = 0 -> 0 mA, CD = 0x7FFF -> ~128 mA, CD = 0x8000 -> ~-128 mA.
 */
void test_decode_pid_34_lambda(void)
{
    uint8_t data[4] = { 0x80, 0x00, 0x00, 0x00 };
    float v = obd2_decode_pid_value(0x34, data, 4);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, v, FLT_TOL);
}

void test_decode_pid_34_current_zero(void)
{
    /* Annex B: $8000 = 0 mA. Vzorec: (CD/256) - 128.
     * CD = 0x8000 = 32768 -> 32768/256 - 128 = 128 - 128 = 0 mA. */
    uint8_t data[4] = { 0x80, 0x00, 0x80, 0x00 };
    float s = obd2_decode_pid_secondary(0x34, data, 4);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s, FLT_TOL);
}

void test_decode_pid_34_current_positive(void)
{
    /* CD = 0xC000 = 49152 -> 49152/256 - 128 = 192 - 128 = 64 mA. */
    uint8_t data[4] = { 0x80, 0x00, 0xC0, 0x00 };
    float s = obd2_decode_pid_secondary(0x34, data, 4);
    TEST_ASSERT_EQUAL_FLOAT(64.0f, s, FLT_TOL);
}

/* ========================================================================= */
/*  Test 10: CONFIG (PID $4F -- surovy bajt A)                               */
/* ========================================================================= */

void test_decode_pid_4F_config_raw_byte(void)
{
    uint8_t data[4] = { 0xAB, 0xCD, 0xEF, 0x01 };
    float v = obd2_decode_pid_value(0x4F, data, 4);
    /* Format CONFIG vraci primo A jako float. */
    TEST_ASSERT_EQUAL_FLOAT(171.0f, v, FLT_TOL);  /* 0xAB = 171 */
}

/* ========================================================================= */
/*  Test 11: Hranicni pripady                                                */
/* ========================================================================= */

void test_decode_pid_null_data_returns_nan(void)
{
    float v = obd2_decode_pid_value(0x0C, NULL, 2);
    TEST_ASSERT_NAN(v);
}

void test_decode_pid_zero_length_returns_nan(void)
{
    uint8_t data[1] = { 0 };
    float v = obd2_decode_pid_value(0x0C, data, 0);
    TEST_ASSERT_NAN(v);
}

void test_decode_pid_unknown_pid_returns_nan(void)
{
    /* PID 0xFE neni v tabulce deskriptoru. */
    uint8_t data[2] = { 0x12, 0x34 };
    float v = obd2_decode_pid_value(0xFE, data, 2);
    TEST_ASSERT_NAN(v);
}

void test_decode_pid_secondary_non_o2_returns_nan(void)
{
    /* PID $0C (otacky) nema sekundarni hodnotu. */
    uint8_t data[2] = { 0x0C, 0x80 };
    float s = obd2_decode_pid_secondary(0x0C, data, 2);
    TEST_ASSERT_NAN(s);
}

void test_decode_pid_secondary_null_data(void)
{
    float s = obd2_decode_pid_secondary(0x14, NULL, 2);
    TEST_ASSERT_NAN(s);
}

void test_decode_pid_secondary_short_data(void)
{
    /* data_len < 2 -> NAN (potrebuje alespon byt B) */
    uint8_t data[1] = { 0x80 };
    float s = obd2_decode_pid_secondary(0x14, data, 1);
    TEST_ASSERT_NAN(s);
}

/* ========================================================================= */
/*  Test 11b: LINEAR_4B (PID $A6 -- odometer)                                */
/* ========================================================================= */

/*
 * Vzorec: value = ((A<<24) | (B<<16) | (C<<8) | D) * 0.1 [km]
 * Pouziti: PID $A6 (Odometer, scaling 0.1 km / LSB).
 *
 * Priklad: raw = 0x000186A0 = 100000 -> 10000.0 km
 */
void test_decode_pid_A6_odometer_zero(void)
{
    uint8_t data[4] = { 0x00, 0x00, 0x00, 0x00 };
    float v = obd2_decode_pid_value(0xA6, data, 4);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, v, FLT_TOL);
}

void test_decode_pid_A6_odometer_10000km(void)
{
    /* 100000 raw * 0.1 = 10000 km */
    uint8_t data[4] = { 0x00, 0x01, 0x86, 0xA0 };
    float v = obd2_decode_pid_value(0xA6, data, 4);
    TEST_ASSERT_EQUAL_FLOAT(10000.0f, v, 1.0f);
}

void test_decode_pid_A6_odometer_short_data_returns_nan(void)
{
    /* Pouze 3 bajty - LINEAR_4B vyzaduje 4 -> NAN */
    uint8_t data[3] = { 0x00, 0x01, 0x86 };
    float v = obd2_decode_pid_value(0xA6, data, 3);
    TEST_ASSERT_NAN(v);
}

/* ========================================================================= */
/*  Test 11c: ENUM (PID $1C -- OBD standard, PID $51 -- typ paliva)          */
/* ========================================================================= */

/*
 * Vzorec: value = A (raw bajt jako float).
 *
 * PID $1C (OBD standard): A=01 -> OBD II California, A=06 -> EOBD, atd.
 * PID $51 (Fuel type):    A=01 -> Gasoline, A=04 -> Diesel, atd.
 */
void test_decode_pid_1C_obd_standard_eobd(void)
{
    uint8_t data[1] = { 0x06 };  /* EOBD */
    float v = obd2_decode_pid_value(0x1C, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, v, FLT_TOL);
}

void test_decode_pid_51_fuel_type_diesel(void)
{
    uint8_t data[1] = { 0x04 };  /* Diesel */
    float v = obd2_decode_pid_value(0x51, data, 1);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, v, FLT_TOL);
}

/* ========================================================================= */
/*  Test 11d: TEMP_4S (PID $78 -- EGT 4 senzory)                             */
/* ========================================================================= */

/*
 * Format dat (9 B): A B C D E F G H I
 *   A:    bity 0..3 = podpora senzoru 1..4
 *   B,C:  senzor 1 = (256B+C) * 0.1 - 40 stupnu C
 *   D,E:  senzor 2
 *   F,G:  senzor 3
 *   H,I:  senzor 4
 *
 * Primarni hodnota = senzor 1.
 * Sekundarni hodnota = senzor 2.
 * Senzory 3,4 -> obd2_decode_pid_extras().
 */
void test_decode_pid_78_temp_4s_sensor1(void)
{
    /* support = 0x0F (vsech 4 senzoru), senzor 1 = (0x0AF0)*0.1 - 40 = 280 - 40 = 240 stupnu */
    uint8_t data[9] = { 0x0F, 0x0A, 0xF0,
                              0x0A, 0xF0,
                              0x0A, 0xF0,
                              0x0A, 0xF0 };
    float v = obd2_decode_pid_value(0x78, data, 9);
    TEST_ASSERT_EQUAL_FLOAT(240.0f, v, FLT_TOL);
}

void test_decode_pid_78_temp_4s_sensor2(void)
{
    /* support = 0x0F, senzor 2 = (0x0BB8)*0.1 - 40 = 300 - 40 = 260 stupnu */
    uint8_t data[9] = { 0x0F, 0x00, 0x00,
                              0x0B, 0xB8,
                              0x00, 0x00,
                              0x00, 0x00 };
    float s = obd2_decode_pid_secondary(0x78, data, 9);
    TEST_ASSERT_EQUAL_FLOAT(260.0f, s, FLT_TOL);
}

void test_decode_pid_78_temp_4s_unsupported_sensor1_returns_nan(void)
{
    /* support = 0x0E (senzor 1 NEpodporovan, 2-4 ano) -> primarni = NAN */
    uint8_t data[9] = { 0x0E, 0x0A, 0xF0,
                              0x0A, 0xF0,
                              0x0A, 0xF0,
                              0x0A, 0xF0 };
    float v = obd2_decode_pid_value(0x78, data, 9);
    TEST_ASSERT_NAN(v);
}

void test_decode_pid_78_temp_4s_extras_for_sensors_3_4(void)
{
    /* support = 0x0F, senzor 3 = 250 stupnu, senzor 4 = 200 stupnu
     *   senzor 3: (256*0x0B + 0x54)*0.1 - 40 = (2900)*0.1 - 40 = 250
     *   senzor 4: (256*0x09 + 0x60)*0.1 - 40 = (2400)*0.1 - 40 = 200 */
    uint8_t data[9] = { 0x0F, 0x00, 0x00,
                              0x00, 0x00,
                              0x0B, 0x54,
                              0x09, 0x60 };
    float extra[2] = { 999.0f, 999.0f };
    uint8_t valid_count = obd2_decode_pid_extras(0x78, data, 9, extra);

    TEST_ASSERT_EQUAL_INT(4, valid_count);
    TEST_ASSERT_EQUAL_FLOAT(250.0f, extra[0], FLT_TOL);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, extra[1], FLT_TOL);
}

/* ========================================================================= */
/*  Test 11e: NOX_4S (PID $83 -- NOx 4 senzory)                              */
/* ========================================================================= */

/*
 * Format dat (9 B): A B C D E F G H I
 *   A:    bity 0..3 = podpora senzoru
 *   B,C:  senzor 1 = (256B+C) ppm  (0xFFFF = invalid)
 */
void test_decode_pid_83_nox_4s_sensor1(void)
{
    /* support = 0x01, senzor 1 = 0x01F4 = 500 ppm */
    uint8_t data[9] = { 0x01, 0x01, 0xF4,
                              0x00, 0x00,
                              0x00, 0x00,
                              0x00, 0x00 };
    float v = obd2_decode_pid_value(0x83, data, 9);
    TEST_ASSERT_EQUAL_FLOAT(500.0f, v, FLT_TOL);
}

void test_decode_pid_83_nox_4s_invalid_value_returns_nan(void)
{
    /* support bit nastaven, ale hodnota = 0xFFFF (invalid mereni) -> NAN */
    uint8_t data[9] = { 0x01, 0xFF, 0xFF,
                              0x00, 0x00,
                              0x00, 0x00,
                              0x00, 0x00 };
    float v = obd2_decode_pid_value(0x83, data, 9);
    TEST_ASSERT_NAN(v);
}

/* ========================================================================= */
/*  Test 11f: RAW format (PID $64 -- engine percent torque)                  */
/* ========================================================================= */

/*
 * RAW: hodnota se nedekoduje (vraci NAN). Frontend zobrazi raw bajty primo.
 * Test: verify that decode returns NAN even with valid-looking data.
 */
void test_decode_pid_64_raw_returns_nan(void)
{
    uint8_t data[5] = { 0x80, 0x90, 0xA0, 0xB0, 0xC0 };
    float v = obd2_decode_pid_value(0x64, data, 5);
    TEST_ASSERT_NAN(v);
}

/* ========================================================================= */
/*  Test 11g: obd2_get_pid_descriptor                                        */
/* ========================================================================= */

/*
 * Lookup PID deskriptoru musi vratit non-NULL pro standardni PIDy a NULL
 * pro zcela neznamy PID (mimo Annex B tabulky).
 */
void test_get_pid_descriptor_known_pid_returns_non_null(void)
{
    const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(0x0C);
    TEST_ASSERT_NOT_NULL(desc);
    TEST_ASSERT_EQUAL_HEX(0x0C, desc->pid);
    TEST_ASSERT_EQUAL_INT(2, desc->data_len);
}

void test_get_pid_descriptor_format_for_a6_is_linear_4b(void)
{
    const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(0xA6);
    TEST_ASSERT_NOT_NULL(desc);
    TEST_ASSERT_EQUAL_INT(OBD2_FMT_LINEAR_4B, desc->format);
    TEST_ASSERT_EQUAL_INT(4, desc->data_len);
}

/* ========================================================================= */
/*  Test 12: obd2_status_str (prevod stavovych kodu na retezce)              */
/* ========================================================================= */

void test_status_str_known_codes(void)
{
    TEST_ASSERT_STRING_EQUAL("OK",               obd2_status_str(OBD2_OK));
    TEST_ASSERT_STRING_EQUAL("TIMEOUT",          obd2_status_str(OBD2_ERR_TIMEOUT));
    TEST_ASSERT_STRING_EQUAL("NEGATIVE_RESP",    obd2_status_str(OBD2_ERR_NEGATIVE_RESP));
    TEST_ASSERT_STRING_EQUAL("NO_DATA",          obd2_status_str(OBD2_ERR_NO_DATA));
    TEST_ASSERT_STRING_EQUAL("INVALID_ARG",      obd2_status_str(OBD2_ERR_INVALID_ARG));
    TEST_ASSERT_STRING_EQUAL("NOT_INIT",         obd2_status_str(OBD2_ERR_NOT_INITIALIZED));
    TEST_ASSERT_STRING_EQUAL("UNSUPPORTED_PID",  obd2_status_str(OBD2_ERR_UNSUPPORTED_PID));
    TEST_ASSERT_STRING_EQUAL("MALFORMED",        obd2_status_str(OBD2_ERR_RESPONSE_MALFORMED));
}

void test_status_str_unknown_code(void)
{
    TEST_ASSERT_STRING_EQUAL("UNKNOWN", obd2_status_str((obd2_status_t)99));
}

/* ========================================================================= */
/*  Test 13: obd2_nrc_str (prevod NRC kodu na retezce)                       */
/* ========================================================================= */

void test_nrc_str_known_codes(void)
{
    TEST_ASSERT_STRING_EQUAL("generalReject",
                              obd2_nrc_str(OBD2_NRC_GENERAL_REJECT));
    TEST_ASSERT_STRING_EQUAL("serviceNotSupported",
                              obd2_nrc_str(OBD2_NRC_SERVICE_NOT_SUPPORTED));
    TEST_ASSERT_STRING_EQUAL("subFunctionNotSupported",
                              obd2_nrc_str(OBD2_NRC_SUB_FUNCTION_NOT_SUPPORTED));
    TEST_ASSERT_STRING_EQUAL("conditionsNotCorrect",
                              obd2_nrc_str(OBD2_NRC_CONDITIONS_NOT_CORRECT));
    TEST_ASSERT_STRING_EQUAL("requestOutOfRange",
                              obd2_nrc_str(OBD2_NRC_REQUEST_OUT_OF_RANGE));
    TEST_ASSERT_STRING_EQUAL("responsePending",
                              obd2_nrc_str(OBD2_NRC_RESPONSE_PENDING));
}

void test_nrc_str_unknown_code(void)
{
    TEST_ASSERT_STRING_EQUAL("unknownNRC", obd2_nrc_str(0xAB));
}

/* ========================================================================= */
/*  Test 14: obd2_get_monitor_status -- parsovani PID $01 (raw data)         */
/* ========================================================================= */

/*
 * Priklad odpovedi pro auto s MIL sviticim a 3 DTC:
 *   Byte A = 0x83 = 1000_0011:
 *     bit 7 = MIL ON
 *     bity 0-6 = 0x03 = 3 DTC
 *   Byte B = 0x07 = 0000_0111:
 *     podpora: misfire + fuel_sys + CCM
 *     horni nibble = 0 -> vsechny jsou ready
 *   Byte C = 0xFF: vsechny nekontinualni monitory podporovany
 *   Byte D = 0x00: vsechny nekontinualni monitory ready
 */
void test_monitor_status_mil_on_3_dtc(void)
{
    uint8_t raw[4] = { 0x83, 0x07, 0xFF, 0x00 };
    obd2_monitor_status_t st;
    memset(&st, 0xAA, sizeof(st));

    obd2_status_t ret = obd2_get_monitor_status(raw, &st);
    TEST_ASSERT_EQUAL_INT(OBD2_OK, ret);

    TEST_ASSERT_TRUE(st.mil_on);
    TEST_ASSERT_EQUAL_INT(3, st.dtc_count);

    TEST_ASSERT_TRUE(st.misfire_sup);
    TEST_ASSERT_TRUE(st.fuel_sys_sup);
    TEST_ASSERT_TRUE(st.ccm_sup);
    TEST_ASSERT_TRUE(st.misfire_rdy);
    TEST_ASSERT_TRUE(st.fuel_sys_rdy);
    TEST_ASSERT_TRUE(st.ccm_rdy);

    TEST_ASSERT_TRUE(st.cat_sup);
    TEST_ASSERT_TRUE(st.o2s_sup);
    TEST_ASSERT_TRUE(st.egr_sup);
    TEST_ASSERT_TRUE(st.cat_rdy);
    TEST_ASSERT_TRUE(st.o2s_rdy);
    TEST_ASSERT_TRUE(st.egr_rdy);
}

/*
 * Vozidlo bez zavad, vsechny monitory ready:
 *   A = 0x00 -> MIL OFF, 0 DTC
 *   B = 0x00 -> nic nepodporovano (edge case)
 *   C = 0x00, D = 0x00
 */
void test_monitor_status_no_mil_no_dtc(void)
{
    uint8_t raw[4] = { 0x00, 0x00, 0x00, 0x00 };
    obd2_monitor_status_t st;

    TEST_ASSERT_EQUAL_INT(OBD2_OK, obd2_get_monitor_status(raw, &st));
    TEST_ASSERT_FALSE(st.mil_on);
    TEST_ASSERT_EQUAL_INT(0, st.dtc_count);
    TEST_ASSERT_FALSE(st.misfire_sup);
    TEST_ASSERT_FALSE(st.cat_sup);
}

/*
 * Test invertovaneho bitu ready: bit v horni nibble B znamena
 * "test NE-dokoncen" (= not ready). Takze B = 0x77 = 0111_0111:
 *   dolni nibble 0x07: misfire+fuel_sys+CCM podporovany
 *   horni nibble 0x70: misfire+fuel_sys+CCM NE-ready (= bit=1)
 */
void test_monitor_status_continuous_not_ready(void)
{
    uint8_t raw[4] = { 0x00, 0x77, 0x00, 0x00 };
    obd2_monitor_status_t st;

    TEST_ASSERT_EQUAL_INT(OBD2_OK, obd2_get_monitor_status(raw, &st));
    TEST_ASSERT_TRUE(st.misfire_sup);
    TEST_ASSERT_TRUE(st.fuel_sys_sup);
    TEST_ASSERT_TRUE(st.ccm_sup);

    /* Bit v horni nibble nastaven -> test NEdokoncen -> rdy=false */
    TEST_ASSERT_FALSE(st.misfire_rdy);
    TEST_ASSERT_FALSE(st.fuel_sys_rdy);
    TEST_ASSERT_FALSE(st.ccm_rdy);
}

void test_monitor_status_null_status_returns_error(void)
{
    uint8_t raw[4] = { 0, 0, 0, 0 };
    TEST_ASSERT_EQUAL_INT(OBD2_ERR_INVALID_ARG,
                          obd2_get_monitor_status(raw, NULL));
}

/* ========================================================================= */
/*  Registr testu pro test_main.c                                            */
/* ========================================================================= */

void run_obd2_pids_tests(void)
{
    /* LINEAR_1B a LINEAR_2B */
    RUN_TEST(test_decode_pid_05_coolant_temp_at_zero);
    RUN_TEST(test_decode_pid_05_coolant_temp_at_90deg);
    RUN_TEST(test_decode_pid_05_coolant_temp_minimum);
    RUN_TEST(test_decode_pid_0C_rpm_idle);
    RUN_TEST(test_decode_pid_0C_rpm_1726);
    RUN_TEST(test_decode_pid_0C_rpm_zero);
    RUN_TEST(test_decode_pid_0D_speed_50kmh);
    RUN_TEST(test_decode_pid_0D_speed_max);

    /* SIGNED_OFFSET_1B a SIGNED_2B */
    RUN_TEST(test_decode_pid_06_stft_zero);
    RUN_TEST(test_decode_pid_06_stft_minus_100);
    RUN_TEST(test_decode_pid_06_stft_positive);
    RUN_TEST(test_decode_pid_32_evap_zero);
    RUN_TEST(test_decode_pid_32_evap_positive);
    RUN_TEST(test_decode_pid_32_evap_negative);

    /* BIT_ENCODED a CONFIG */
    RUN_TEST(test_decode_pid_01_bitencoded_preserves_bytes);
    RUN_TEST(test_decode_pid_4F_config_raw_byte);

    /* O2 senzory */
    RUN_TEST(test_decode_pid_14_o2_voltage);
    RUN_TEST(test_decode_pid_14_o2_stft_secondary);
    RUN_TEST(test_decode_pid_14_o2_stft_negative);
    RUN_TEST(test_decode_pid_24_lambda_stoichiometric);
    RUN_TEST(test_decode_pid_24_voltage_secondary_zero);
    RUN_TEST(test_decode_pid_24_voltage_secondary_half);
    RUN_TEST(test_decode_pid_34_lambda);
    RUN_TEST(test_decode_pid_34_current_zero);
    RUN_TEST(test_decode_pid_34_current_positive);

    /* Hranicni pripady */
    RUN_TEST(test_decode_pid_null_data_returns_nan);
    RUN_TEST(test_decode_pid_zero_length_returns_nan);
    RUN_TEST(test_decode_pid_unknown_pid_returns_nan);
    RUN_TEST(test_decode_pid_secondary_non_o2_returns_nan);
    RUN_TEST(test_decode_pid_secondary_null_data);
    RUN_TEST(test_decode_pid_secondary_short_data);

    /* LINEAR_4B (PID $A6 odometer) */
    RUN_TEST(test_decode_pid_A6_odometer_zero);
    RUN_TEST(test_decode_pid_A6_odometer_10000km);
    RUN_TEST(test_decode_pid_A6_odometer_short_data_returns_nan);

    /* ENUM (PID $1C OBD standard, $51 fuel type) */
    RUN_TEST(test_decode_pid_1C_obd_standard_eobd);
    RUN_TEST(test_decode_pid_51_fuel_type_diesel);

    /* TEMP_4S (PID $78 EGT bank 1) */
    RUN_TEST(test_decode_pid_78_temp_4s_sensor1);
    RUN_TEST(test_decode_pid_78_temp_4s_sensor2);
    RUN_TEST(test_decode_pid_78_temp_4s_unsupported_sensor1_returns_nan);
    RUN_TEST(test_decode_pid_78_temp_4s_extras_for_sensors_3_4);

    /* NOX_4S (PID $83) */
    RUN_TEST(test_decode_pid_83_nox_4s_sensor1);
    RUN_TEST(test_decode_pid_83_nox_4s_invalid_value_returns_nan);

    /* RAW (PID $64) */
    RUN_TEST(test_decode_pid_64_raw_returns_nan);

    /* PID descriptor lookup */
    RUN_TEST(test_get_pid_descriptor_known_pid_returns_non_null);
    RUN_TEST(test_get_pid_descriptor_format_for_a6_is_linear_4b);

    /* Prevod stavovych kodu a NRC */
    RUN_TEST(test_status_str_known_codes);
    RUN_TEST(test_status_str_unknown_code);
    RUN_TEST(test_nrc_str_known_codes);
    RUN_TEST(test_nrc_str_unknown_code);

    /* Monitor status */
    RUN_TEST(test_monitor_status_mil_on_3_dtc);
    RUN_TEST(test_monitor_status_no_mil_no_dtc);
    RUN_TEST(test_monitor_status_continuous_not_ready);
    RUN_TEST(test_monitor_status_null_status_returns_error);
}
