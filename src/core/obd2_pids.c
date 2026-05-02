/**
 * @file obd2_pids.c
 * @brief Cteni a dekodovani PID hodnot -- Mode 01 (aktualni data) a Mode 02 (freeze frame)
 *
 * Tento soubor je soucasti trojice:
 *   obd2.c       -- jadro (init, konfigurace, ISO-TP komunikace, PID tabulka)
 *   obd2_pids.c  -- cteni a dekodovani PID hodnot (TENTO SOUBOR)
 *   obd2_diag.c  -- diagnostika (DTC, VIN, CalID, ECU name)
 *
 * Implementuje:
 *   - Dekodovani PID hodnot z raw bytu dle ISO 15031-5 Annex B
 *   - Cteni jednoho PIDu (Mode 01 / Service $01)
 *   - Monitor status (PID $01 -- specialni 4-bytovy bit-encoded format)
 *   - Freeze frame (Mode 02 / Service $02)
 *
 * @author Ales Pouzar, vycházel jsem z ISO 15031-5/-6, SAE J1979, J2012, a dokumentace a wikipedie pro dostupne PIDs   
 */

#include "obd2_internal.h"

/* ========================================================================= */
/*  Dekodovani PID hodnot                                                    */
/* ========================================================================= */

/**
 * @brief Dekoduje primarni hodnotu PIDu z raw datovych bytu.
 *
 * Kazdy PID ma v tabulce prirazen format (obd2_pid_format_t), ktery urcuje
 * zpusob dekodovani. Podporovane formaty:
 *
 *  - LINEAR_1B:  hodnota = A * multiplier + offset.
 *    Priklad: PID $04 (zatizeni motoru) = A * 100/255, rozsah 0..100 %.
 *
 *  - LINEAR_2B:  hodnota = (256*A + B) * multiplier + offset.
 *    Priklad: PID $0C (otacky) = (256*A + B) / 4, rozsah 0..16383.75 RPM.
 *
 *  - SIGNED_OFFSET_1B: hodnota = (A - 128) * multiplier.
 *    Priklad: PID $06 (STFT bank 1) = (A - 128) * 100/128, rozsah -100..+99.2 %.
 *
 *  - SIGNED_2B: hodnota = (int16_t)(256*A + B) * mult + offset.
 *    Priklad: PID $32 (tlak par) = (int16_t)(256A+B) / 4, rozsah -8192..+8191.75 Pa.
 *
 *  - BIT_ENCODED: 4 byty sbalene do uint32, pretypovane na float (ztratove pro >24 bitu).
 *    Pouziti: PID $01 (monitor status), $03 (fuel system), $13 (O2 senzor pritomnost).
 *    Klient musi pretypovat zpet na uint32_t pro bitovou analyzu.
 *
 *  - O2_CONV: napeti = A * 0.005 V.
 *    Pouziti: PIDy $14--$1B (konvencni O2 senzory), rozsah 0..1.275 V.
 *
 *  - O2_WIDE_EQ_V: ekvivalencni pomer = (256*A + B) * 2/65536.
 *    Pouziti: PIDy $24--$2B (sirokopasme O2 -- napetovy vystup).
 *
 *  - O2_WIDE_EQ_I: ekvivalencni pomer = (256*A + B) * 2/65536.
 *    Pouziti: PIDy $34--$3B (sirokopasme O2 -- proudovy vystup).
 *
 *  - CONFIG: raw byte A jako float, bez skalovania.
 *    Pouziti: PIDy $12 (commanded secondary air), $1C (OBD standard).
 *
 * Hranicni pripady:
 *  - Pokud PID neni v tabulce deskriptoru, vraci NAN.
 *  - Pokud data == NULL nebo data_len == 0, vraci NAN.
 *  - BIT_ENCODED PIDy nelze smysluplne interpretovat jako cislo -- float
 *    representace je pouze pro prenos; klient musi pretypovat na uint32_t.
 *
 * @param pid       Cislo PIDu (0x00--0xFF)
 * @param data      Ukazatel na raw data byty (A, B, C, D) z ECU odpovedi
 * @param data_len  Pocet platnych bytu v data[]
 * @return Dekodovana hodnota, nebo NAN pri chybe (neznamy PID, neplatna data)
 */
float obd2_decode_pid_value(uint8_t pid, const uint8_t *data, uint8_t data_len)
{
    if (data == NULL || data_len == 0) {
        OBD2_LOGE("decode_pid_value: NULL data or len=0");
        return NAN;
    }

    const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(pid);
    if (desc == NULL) {
        OBD2_LOGD("decode_pid_value: pid 0x%02X not in table", pid);
        return NAN;
    }

    uint8_t a = data[0];
    uint8_t b = (data_len >= 2) ? data[1] : 0;
    uint8_t c = (data_len >= 3) ? data[2] : 0;
    uint8_t d = (data_len >= 4) ? data[3] : 0;
    uint16_t ab = ((uint16_t)a << 8) | b;

    switch (desc->format) {
    case OBD2_FMT_LINEAR_1B:
        return (float)a * desc->multiplier + desc->offset;

    case OBD2_FMT_LINEAR_2B:
        return (float)ab * desc->multiplier + desc->offset;

    case OBD2_FMT_SIGNED_OFFSET_1B:
        /* hodnota = (A - 128) * multiplier */
        return ((float)a - 128.0f) * desc->multiplier;

    case OBD2_FMT_SIGNED_2B:
        /* hodnota = (int16_t)(256A+B) * multiplier + offset */
        return (float)(int16_t)ab * desc->multiplier + desc->offset;

    case OBD2_FMT_BIT_ENCODED:
        /* Sbaleni az 4 bytu do uint32, pretypovani na float (ztratove pro >24 bitu) */
        return (float)(((uint32_t)a << 24) | ((uint32_t)b << 16) |
                       ((uint32_t)c << 8) | (uint32_t)d);

    case OBD2_FMT_O2_CONV:
        /* Primarni hodnota = napeti: A * 0.005 V */
        return (float)a * 0.005f;

    case OBD2_FMT_O2_WIDE_EQ_V:
        /* Primarni hodnota = ekvivalencni pomer: (256A+B) * 2/65536 */
        return (float)ab * (2.0f / 65536.0f);

    case OBD2_FMT_O2_WIDE_EQ_I:
        /* Primarni hodnota = ekvivalencni pomer: (256A+B) * 2/65536 */
        return (float)ab * (2.0f / 65536.0f);

    case OBD2_FMT_CONFIG:
        /* Konfiguracni PIDy: raw A jako float */
        return (float)a;

    case OBD2_FMT_ENUM:
        /* Vyctovy typ: vraci primo prvni bajt bez posunu */
        return (float)a;

    case OBD2_FMT_LINEAR_4B: {
        /* 32-bit unsigned linearni:
         *   value = ((A<<24) | (B<<16) | (C<<8) | D) * mult + offset
         * Pouziti: $A6 odometer (km*10), $7F engine run time. */
        if (data_len < 4) return NAN;
        uint32_t v = ((uint32_t)a << 24) | ((uint32_t)b << 16) |
                     ((uint32_t)c << 8)  | (uint32_t)d;
        return (float)v * desc->multiplier + desc->offset;
    }

    case OBD2_FMT_TEMP_4S:
    case OBD2_FMT_NOX_4S:
        /* Multi-sensor formaty: primarni hodnota = senzor 1, ostatni
         * dekoduje obd2_get_pid() pres obd2_decode_pid_extras(). */
        if (data_len < 3) return NAN;
        if (desc->format == OBD2_FMT_TEMP_4S) {
            uint8_t support = a;
            if (!(support & 0x01)) return NAN; /* senzor 1 nepodporovan */
            uint16_t bc = ((uint16_t)b << 8) | c;
            return (float)bc * 0.1f - 40.0f;
        } else {
            /* NOX_4S: senzor 1 = (256B+C) ppm; 0xFFFF = invalid */
            uint8_t support = a;
            if (!(support & 0x01)) return NAN;
            uint16_t bc = ((uint16_t)b << 8) | c;
            if (bc == 0xFFFF) return NAN;
            return (float)bc;
        }

    case OBD2_FMT_RAW:
        /* RAW format: hodnota neni dekodovana. Frontend zobrazi raw bajty.
         * Vracime NAN — klient pozna podle isnan() ze nema hodnotu. */
        return NAN;

    default:
        OBD2_LOGE("decode_pid_value: unknown format %d for pid 0x%02X",
                  desc->format, pid);
        return NAN;
    }
}

/**
 * @brief Dekoduje 3. a 4. hodnotu pro multi-sensor PIDy ($78, $79, $83, $98...).
 *
 * Pro PIDy formatu TEMP_4S a NOX_4S vrati senzor 3 (extra[0]) a senzor 4
 * (extra[1]). Pokud senzor neni podporovan (support bit = 0) nebo je hodnota
 * neplatna (0xFFFF u NOx), vraci NAN.
 *
 * Vraci pocet validnich hodnot (1-4) — value, secondary, extra[0..1].
 *
 * @param pid       Cislo PIDu
 * @param data      Raw data byty (alespon 9 bytu pro multi-sensor)
 * @param data_len  Pocet validnich bytu
 * @param extra     Pole 2 floatu pro senzory 3 a 4 (musi byt non-NULL)
 * @return Pocet validnich hodnot (1 az 4) v decoded structuie
 */
uint8_t obd2_decode_pid_extras(uint8_t pid, const uint8_t *data,
                               uint8_t data_len, float *extra)
{
    if (data == NULL || extra == NULL) {
        if (extra) { extra[0] = NAN; extra[1] = NAN; }
        return 1;
    }

    extra[0] = NAN;
    extra[1] = NAN;

    const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(pid);
    if (desc == NULL) return 1;

    if (desc->format != OBD2_FMT_TEMP_4S && desc->format != OBD2_FMT_NOX_4S) {
        if (pid >= 0x55 && pid <= 0x58 && data_len >= 2) {
            return 2;
        }
        /* Skalarni / O2 formaty maji 1-2 hodnoty (value + secondary).
         * Skalarni: 1 hodnota. O2: 2 hodnoty (kdyz secondary neni NAN). */
        if (desc->format == OBD2_FMT_O2_CONV ||
            desc->format == OBD2_FMT_O2_WIDE_EQ_V ||
            desc->format == OBD2_FMT_O2_WIDE_EQ_I) {
            return 2;
        }
        return 1;
    }

    /* Multi-sensor: 9 bytu (1 byte podpora + 4× 2 byty hodnota) */
    if (data_len < 9) return 1;

    uint8_t support = data[0];
    uint8_t valid_count = (support & 0x01) ? 1 : 0;

    /* Senzor 2 — uz je v 'secondary' field, jen rozhodneme, jestli pocitat */
    if (support & 0x02) valid_count++;

    /* Senzor 3 = bajty F, G (data[5], data[6]) */
    if (support & 0x04) {
        uint16_t fg = ((uint16_t)data[5] << 8) | data[6];
        if (desc->format == OBD2_FMT_TEMP_4S) {
            extra[0] = (float)fg * 0.1f - 40.0f;
        } else if (fg != 0xFFFF) {
            extra[0] = (float)fg;
        }
        valid_count++;
    }

    /* Senzor 4 = bajty H, I (data[7], data[8]) */
    if (support & 0x08) {
        uint16_t hi = ((uint16_t)data[7] << 8) | data[8];
        if (desc->format == OBD2_FMT_TEMP_4S) {
            extra[1] = (float)hi * 0.1f - 40.0f;
        } else if (hi != 0xFFFF) {
            extra[1] = (float)hi;
        }
        valid_count++;
    }

    return valid_count > 0 ? valid_count : 1;
}

/**
 * @brief Dekoduje sekundarni hodnotu pro PIDy s dvema hodnotami.
 *
 * Nektere PIDy (predevsim O2 senzory) nesou v odpovedi dve nezavisle hodnoty.
 * Tato funkce dekoduje druhou (sekundarni) hodnotu:
 *
 *  - O2_CONV (PIDy $14--$1B):
 *    Primarni = napeti (A * 0.005 V),
 *    sekundarni = kratkodoby fuel trim STFT = (B - 128) * 100/128 [%].
 *    Rozsah: -100 az +99.2 %. Hodnota 0 % = zadna korekce.
 *
 *  - O2_WIDE_EQ_V (PIDy $24--$2B):
 *    Primarni = ekvivalencni pomer,
 *    sekundarni = napeti = (256*C + D) * 8/65536 [V].
 *    Rozsah: 0 az 8 V.
 *
 *  - O2_WIDE_EQ_I (PIDy $34--$3B):
 *    Primarni = ekvivalencni pomer,
 *    sekundarni = proud = (256*C + D) / 256 - 128 [mA].
 *    Dle Annex B: $8000 = 0 mA, rozsah -128 az +127.996 mA.
 *
 *  - Vsechny ostatni formaty: vraci NAN (nemaji sekundarni hodnotu).
 *
 * Hranicni pripady:
 *  - Pokud data == NULL nebo data_len < 2, vraci NAN.
 *  - Pokud PID neni v tabulce, vraci NAN.
 *
 * @param pid       Cislo PIDu (0x00--0xFF)
 * @param data      Ukazatel na raw data byty (A, B, C, D) z ECU odpovedi
 * @param data_len  Pocet platnych bytu v data[]
 * @return Dekodovana sekundarni hodnota, nebo NAN pokud PID nema sekundarni hodnotu
 */
float obd2_decode_pid_secondary(uint8_t pid, const uint8_t *data,
                                 uint8_t data_len)
{
    if (data == NULL || data_len < 2) return NAN;

    const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(pid);
    if (desc == NULL) return NAN;

    uint8_t b = data[1];
    uint8_t c = (data_len >= 3) ? data[2] : 0;
    uint8_t d = (data_len >= 4) ? data[3] : 0;
    uint16_t cd = ((uint16_t)c << 8) | d;

    switch (desc->format) {
    case OBD2_FMT_SIGNED_OFFSET_1B:
        if (pid >= 0x55 && pid <= 0x58 && data_len >= 2) {
            return ((float)b - 128.0f) * (100.0f / 128.0f);
        }
        return NAN;

    case OBD2_FMT_O2_CONV:
        /* Sekundarni = STFT: (B - 128) * 100/128 % */
        /* ISO 15031-5: If B == 0xFF, the sensor is not used in trim calculation */
        if (b == 0xFF) return NAN;
        return ((float)b - 128.0f) * (100.0f / 128.0f);

    case OBD2_FMT_O2_WIDE_EQ_V:
        /* Sekundarni = napeti: (256C+D) * 8/65536 V */
        return (float)cd * (8.0f / 65536.0f);

    case OBD2_FMT_O2_WIDE_EQ_I:
        /* Sekundarni = proud: (256C+D) / 256 - 128 mA */
        /* Dle Annex B: $8000 = 0 mA, rozsah -128 az +127.996 */
        return ((float)cd / 256.0f) - 128.0f;

    case OBD2_FMT_TEMP_4S:
    case OBD2_FMT_NOX_4S:
        /* Multi-sensor: senzor 2 = bajty D, E (data[3], data[4]).
         * Podporu indikuje bit 1 v bajtu A (data[0]). */
        if (data_len < 5) return NAN;
        if (!(data[0] & 0x02)) return NAN; /* senzor 2 nepodporovan */
        {
            uint16_t de = ((uint16_t)data[3] << 8) | data[4];
            if (desc->format == OBD2_FMT_TEMP_4S) {
                return (float)de * 0.1f - 40.0f;
            }
            if (de == 0xFFFF) return NAN;
            return (float)de;
        }

    default:
        return NAN;
    }
}

/* ========================================================================= */
/*  Mode 01: Aktualni data (Current Data)                                    */
/* ========================================================================= */

/**
 * @brief Precte raw data jednoho PIDu z ECU (Mode 01 / Service $01).
 *
 * Provede kompletni komunikacni cyklus pro cteni jednoho PIDu:
 *  1. Kontrola inicializace (_ctx.initialized) -- pokud ne, vraci NOT_INITIALIZED.
 *  2. Kontrola podpory PIDu v bitmaskce (_ctx.supported_pids) -- pokud PID neni
 *     v bitmaskce a bitmaska byla nactena, vraci UNSUPPORTED_PID.
 *     Pozn.: nektera ECU podporuji PIDy mimo standardni bitmasku.
 *  3. Sestaveni requestu [0x01, PID] a odeslani pres fyzickou adresu (unicast).
 *  4. Validace odpovedi:
 *     - Minimalni delka 3 byty: [0x41, PID, A]
 *     - Echovany PID v odpovedi musi odpovidat pozadovanemu PIDu
 *  5. Extrakce datovych bytu A, B, C, D (max 4 byty po SID+PID).
 *
 * Hranicni pripady:
 *  - result == NULL: vraci OBD2_ERR_INVALID_ARG.
 *  - PID neni v bitmaskce: vraci OBD2_ERR_UNSUPPORTED_PID (ale nektera ECU
 *    odpovi i tak -- klient muze PID zkusit primo pres _obd2_request).
 *  - Odpoved kratsi nez 3 byty: vraci OBD2_ERR_RESPONSE_MALFORMED.
 *  - Echovany PID neodpovida: vraci OBD2_ERR_RESPONSE_MALFORMED
 *    (muze nastat pri kolizi na CAN sbernici).
 *
 * @param pid     Cislo pozadovaneho PIDu (0x00--0xFF)
 * @param result  Ukazatel na strukturu pro vysledek (pid, data[], data_len)
 * @return OBD2_OK pri uspechu, jinak chybovy kod
 */
obd2_status_t obd2_get_pid_raw(uint8_t pid, obd2_pid_raw_t *result)
{
    OBD2_LOGI("get_pid_raw: pid=0x%02X", pid);

    if (result == NULL) {
        OBD2_LOGE("get_pid_raw: NULL result pointer");
        return OBD2_ERR_INVALID_ARG;
    }

    if (!_ctx.initialized) {
        OBD2_LOGE("get_pid_raw: not initialized");
        return OBD2_ERR_NOT_INITIALIZED;
    }

    /* Varovani pokud PID neni v bitmaskce podporovanych (presto se pokusi odeslat) */
    if (_ctx.pids_queried && !obd2_is_pid_supported(pid)) {
        OBD2_LOGW("get_pid_raw: pid 0x%02X not in supported bitmask", pid);
        return OBD2_ERR_UNSUPPORTED_PID;
    }

    uint8_t req[2] = { OBD2_SID_CURRENT_DATA, pid };
    uint8_t resp[ISOTP_MAX_PAYLOAD];
    uint16_t resp_len = sizeof(resp);

    obd2_status_t st = _obd2_request(req, 2, resp, &resp_len, false);
    if (st != OBD2_OK) {
        OBD2_LOGE("get_pid_raw: request failed: %s", obd2_status_str(st));
        return st;
    }

    /* Odpoved: [41, PID, data_A, (data_B), (data_C), (data_D)] */
    if (resp_len < 3) {
        OBD2_LOGE("get_pid_raw: response too short (%u bytes)", resp_len);
        return OBD2_ERR_RESPONSE_MALFORMED;
    }

    /* Validace echovaneho PIDu */
    if (resp[1] != pid) {
        OBD2_LOGE("get_pid_raw: PID mismatch: got 0x%02X expected 0x%02X",
                  resp[1], pid);
        return OBD2_ERR_RESPONSE_MALFORMED;
    }

    /* Extrakce datovych bytu (preskoceni SID + PID).
     * Buffer pokryva i multi-sensor a dlouhe J1979-DA PIDy. */
    result->pid = pid;
    uint16_t avail = (resp_len > 2) ? (resp_len - 2) : 0;
    result->data_len = (avail > OBD2_PID_MAX_DATA_BYTES)
                       ? OBD2_PID_MAX_DATA_BYTES
                       : (uint8_t)avail;
    memcpy(result->data, &resp[2], result->data_len);

    OBD2_LOGD("get_pid_raw: pid=0x%02X data_len=%u data=[%02X %02X %02X %02X%s]",
              pid, result->data_len,
              result->data[0],
              result->data_len > 1 ? result->data[1] : 0,
              result->data_len > 2 ? result->data[2] : 0,
              result->data_len > 3 ? result->data[3] : 0,
              result->data_len > 4 ? "..." : "");

    return OBD2_OK;
}

/**
 * @brief Precte a dekoduje PID -- vysokourovnova funkce.
 *
 * Kombinuje obd2_get_pid_raw() a obd2_decode_pid_value()/obd2_decode_pid_secondary()
 * do jednoho volani. Vysledek obsahuje:
 *  - pid: cislo PIDu
 *  - raw_data/raw_data_len: puvodni datove bajty A..N z ECU odpovedi
 *  - value: dekodovana primarni hodnota (napr. teplota ve stupnich, otacky v RPM)
 *  - secondary: sekundarni hodnota (pouze pro O2 PIDy, jinak NAN)
 *  - name: lidsky citelny nazev PIDu (napr. "Engine RPM")
 *  - unit: jednotka (napr. "rpm", "degC", "%")
 *
 * Pokud PID neni v tabulce deskriptoru, name = "Unknown PID" a unit = "".
 *
 * @param pid     Cislo pozadovaneho PIDu (0x00--0xFF)
 * @param result  Ukazatel na strukturu pro dekodovany vysledek
 * @return OBD2_OK pri uspechu, jinak chybovy kod (viz obd2_get_pid_raw)
 */
obd2_status_t obd2_get_pid(uint8_t pid, obd2_pid_decoded_t *result)
{
    OBD2_LOGI("get_pid: pid=0x%02X", pid);

    if (result == NULL) {
        OBD2_LOGE("get_pid: NULL result pointer");
        return OBD2_ERR_INVALID_ARG;
    }

    /* Nejprve precteme raw data */
    obd2_pid_raw_t raw;
    obd2_status_t st = obd2_get_pid_raw(pid, &raw);
    if (st != OBD2_OK) return st;

    /* Vyhledani deskriptoru v tabulce */
    const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(pid);

    memset(result, 0, sizeof(*result));
    result->pid = pid;
    result->raw_data_len = raw.data_len;
    memcpy(result->raw_data, raw.data, raw.data_len);
    result->value = obd2_decode_pid_value(pid, raw.data, raw.data_len);
    result->secondary = obd2_decode_pid_secondary(pid, raw.data, raw.data_len);
    result->value_count = obd2_decode_pid_extras(pid, raw.data, raw.data_len,
                                                  result->extra);
    result->name = desc ? desc->name : "Unknown PID";
    result->unit = desc ? desc->unit : "";

    OBD2_LOGI("get_pid: pid=0x%02X \"%s\" = %.3f %s (count=%u)",
              pid, result->name, result->value, result->unit,
              result->value_count);

    return OBD2_OK;
}

/* ========================================================================= */
/*  PID $01: Monitor Status                                                  */
/* ========================================================================= */

/**
 * @brief Parsuje monitor status z PID $01 (4 byty).
 *
 * PID $01 ma specialni 4-bytovy bit-encoded format, ktery nelze dekodovat
 * standardnim obd2_decode_pid_value(). Tato funkce rozklada jednotlive bity:
 *
 * Struktura 4 bytu (A, B, C, D):
 *
 *  Byte A (data[0]):
 *    bit 7     = MIL (Malfunction Indicator Lamp -- kontrolka motoru)
 *                1 = sviti, 0 = nesviti
 *    bity 6-0  = pocet potvrzenych DTC (0--127)
 *
 *  Byte B (data[1]):
 *    dolni nibble (bity 0-2) = podpora kontinualnich monitoru:
 *      bit 0 = misfire (vynechavani zapalovani)
 *      bit 1 = fuel_system (palivovy system)
 *      bit 2 = CCM (comprehensive component monitoring)
 *    horni nibble (bity 4-6) = stav kontinualnich monitoru (INVERTOVANY):
 *      0 = test dokoncen (ready), 1 = test nedokoncen (not ready)
 *
 *  Byte C (data[2]):
 *    podpora nekontinualnich monitoru:
 *      bit 0 = catalyst (katalyzator)
 *      bit 1 = heated catalyst
 *      bit 2 = EVAP (emise par paliva)
 *      bit 3 = secondary air (sekundarni vzduch)
 *      bit 4 = A/C refrigerant
 *      bit 5 = O2 sensor (lambda sonda)
 *      bit 6 = O2 sensor heater (ohrev lambda sondy)
 *      bit 7 = EGR (recirkulace vyfukovych plynu)
 *
 *  Byte D (data[3]):
 *    stav nekontinualnich monitoru (INVERTOVANY: 0 = hotovo, 1 = nehotovo)
 *    Bity odpovidaji byte C ve stejnem poradi.
 *
 * Priklad: data = [0x83, 0x07, 0xFF, 0x00]
 *   Byte A = 0x83 = 1000_0011: MIL=ON (bit7=1), pocet DTC = 3 (bity 6-0 = 0x03)
 *   Byte B = 0x07 = 0000_0111: misfire+fuel+CCM podporovany, horni nibble 0 = vsechny ready
 *   Byte C = 0xFF: vsechny nekontinualni monitory podporovany
 *   Byte D = 0x00: vsechny nekontinualni monitory hotove (ready)
 *
 * Pokud raw != NULL, parsuje predane data (4 byty). Pokud raw == NULL,
 * precte PID $01 z ECU pomoci obd2_get_pid_raw().
 *
 * @param raw     Ukazatel na 4 byty raw dat, nebo NULL pro cteni z ECU
 * @param status  Ukazatel na vystupni strukturu obd2_monitor_status_t
 * @return OBD2_OK pri uspechu, OBD2_ERR_INVALID_ARG pokud status == NULL,
 *         nebo chybovy kod z obd2_get_pid_raw() pokud raw == NULL
 */
obd2_status_t obd2_get_monitor_status(const uint8_t *raw,
                                       obd2_monitor_status_t *status)
{
    OBD2_LOGI("get_monitor_status");

    if (status == NULL) {
        OBD2_LOGE("get_monitor_status: NULL pointer");
        return OBD2_ERR_INVALID_ARG;
    }

    memset(status, 0, sizeof(*status));

    uint8_t data[4];

    if (raw != NULL) {
        memcpy(data, raw, 4);
    } else {
        /* Cteni z ECU */
        obd2_pid_raw_t pid_raw;
        obd2_status_t st = obd2_get_pid_raw(0x01, &pid_raw);
        if (st != OBD2_OK) return st;
        if (pid_raw.data_len < 4) return OBD2_ERR_RESPONSE_MALFORMED;
        memcpy(data, pid_raw.data, 4);
    }

    memcpy(status->raw, data, sizeof(status->raw));

    /* Byte A: pocet DTC (bity 0-6) a MIL (bit 7) */
    status->dtc_count = data[0] & 0x7F;
    status->mil_on    = (data[0] & 0x80) != 0;

    /* Byte B: common/continuous monitors.
     * B0-B2 = support, B3 = ignition type, B4-B6 = not complete (0=ready). */
    status->is_compression = (data[1] & 0x08) != 0;
    
    status->misfire_sup  = (data[1] & 0x01) != 0;
    status->fuel_sys_sup = (data[1] & 0x02) != 0;
    status->ccm_sup      = (data[1] & 0x04) != 0;

    status->misfire_rdy  = (data[1] & 0x10) == 0;
    status->fuel_sys_rdy = (data[1] & 0x20) == 0;
    status->ccm_rdy      = (data[1] & 0x40) == 0;

    /* Byte C & D: podpora a stav nekontinualnich monitoru (lisi se podle typu motoru) */
    if (!status->is_compression) {
        /* Zazehozy motor (Spark / Benzin) */
        status->cat_sup  = (data[2] & 0x01) != 0;
        status->hcat_sup = (data[2] & 0x02) != 0;
        status->evap_sup = (data[2] & 0x04) != 0;
        status->air_sup  = (data[2] & 0x08) != 0;
        status->acrf_sup = (data[2] & 0x10) != 0;
        status->o2s_sup  = (data[2] & 0x20) != 0;
        status->htr_sup  = (data[2] & 0x40) != 0;
        status->egr_sup  = (data[2] & 0x80) != 0;

        status->cat_rdy  = (data[3] & 0x01) == 0;
        status->hcat_rdy = (data[3] & 0x02) == 0;
        status->evap_rdy = (data[3] & 0x04) == 0;
        status->air_rdy  = (data[3] & 0x08) == 0;
        status->acrf_rdy = (data[3] & 0x10) == 0;
        status->o2s_rdy  = (data[3] & 0x20) == 0;
        status->htr_rdy  = (data[3] & 0x40) == 0;
        status->egr_rdy  = (data[3] & 0x80) == 0;
    } else {
        /* Vznetovy motor (Compression / Nafta) */
        /* Poznamka: Pole ve strukture jsou pojmenovana podle benzinu,
           ale norma prirazuje bity dieselu takto: */
        status->cat_sup  = (data[2] & 0x01) != 0; /* NMHC Catalyst */
        status->hcat_sup = (data[2] & 0x02) != 0; /* NOx/SCR Aftertreatment */
        /* evap_sup (bit 2) je u dieselu reserved */
        status->air_sup  = (data[2] & 0x08) != 0; /* Boost Pressure */
        /* acrf_sup (bit 4) je u dieselu reserved */
        status->o2s_sup  = (data[2] & 0x20) != 0; /* Exhaust Gas Sensor */
        status->htr_sup  = (data[2] & 0x40) != 0; /* PM Filter */
        status->egr_sup  = (data[2] & 0x80) != 0; /* EGR and/or VVT */

        status->cat_rdy  = (data[3] & 0x01) == 0;
        status->hcat_rdy = (data[3] & 0x02) == 0;
        status->air_rdy  = (data[3] & 0x08) == 0;
        status->o2s_rdy  = (data[3] & 0x20) == 0;
        status->htr_rdy  = (data[3] & 0x40) == 0;
        status->egr_rdy  = (data[3] & 0x80) == 0;
    }

    OBD2_LOGI("monitor_status: MIL=%s DTCs=%u",
              status->mil_on ? "ON" : "OFF", status->dtc_count);
    OBD2_LOGD("monitor_status: CAT=%c EVAP=%c O2S=%c EGR=%c",
              status->cat_rdy ? 'Y' : 'N', status->evap_rdy ? 'Y' : 'N',
              status->o2s_rdy ? 'Y' : 'N', status->egr_rdy  ? 'Y' : 'N');

    return OBD2_OK;
}

/**
 * @brief Multi-ECU varianta cteni readiness/monitor status pres broadcast.
 *
 * Posle Mode 01 PID 01 na 0x7DF a sesbira monitor status od vsech ECU.
 * Kazda ECU ma vlastni sadu readiness monitoru — napr. ECM hlasi stav
 * CAT/EVAP/O2S, TCM jen relevantni pro prevodovku. Hlavni use-case:
 * emisni kontrola (overit, ze VSECHNY emisne-relevantni ECU jsou ready).
 *
 * Dekodovaci logika je znovu pouzita z obd2_get_monitor_status() — predava
 * se 4-bajtovy buffer [A,B,C,D] z pozice [2] kazde odpovedi.
 *
 * @param list  Vystupni seznam per-ECU monitor status
 * @return OBD2_OK pri uspechu, OBD2_ERR_NO_DATA pokud zadna ECU neodpovedela
 */
obd2_status_t obd2_get_monitor_status_all(obd2_monitor_status_list_t *list)
{
    OBD2_LOGI("get_monitor_status_all");

    if (list == NULL) {
        OBD2_LOGE("get_monitor_status_all: NULL pointer");
        return OBD2_ERR_INVALID_ARG;
    }
    if (!_ctx.initialized) {
        OBD2_LOGE("get_monitor_status_all: not initialized");
        return OBD2_ERR_NOT_INITIALIZED;
    }

    memset(list, 0, sizeof(obd2_monitor_status_list_t));

    uint8_t req[2] = { OBD2_SID_CURRENT_DATA, 0x01 };
    static isotp_result_t bcast_result;

    obd2_status_t st = _obd2_request_multi(req, 2, &bcast_result, _ctx.timeout_ms);
    if (st != OBD2_OK) return st;

    uint8_t expected_resp_sid = OBD2_SID_CURRENT_DATA + OBD2_SID_RESPONSE_OFFSET;

    for (uint8_t i = 0; i < bcast_result.count && list->count < ISOTP_MAX_ECU_RESPONSES; i++) {
        isotp_response_t *r = &bcast_result.responses[i];
        /* Minimalni format: [41, 01, A, B, C, D] = 6 B */
        if (!r->valid || r->len < 6) continue;
        if (r->data[0] != expected_resp_sid || r->data[1] != 0x01) continue;

        obd2_ecu_monitor_status_item_t *item = &list->items[list->count];
        item->rx_id = r->rx_id;

        /* Dekodovaci logika je sdilena s obd2_get_monitor_status() */
        obd2_get_monitor_status(&r->data[2], &item->status);

        list->count++;
    }

    return (list->count > 0) ? OBD2_OK : OBD2_ERR_NO_DATA;
}

/* ========================================================================= */
/*  Mode 02: Freeze Frame                                                    */
/* ========================================================================= */

/**
 * @brief Precte freeze frame data pro dany PID (Mode 02 / Service $02).
 *
 * Freeze frame je "snimek" hodnot PIDu v okamziku, kdy ECU zaznamenala
 * diagnosticky chybovy kod (DTC). Umoznuje zpetne analyzovat podminky
 * pri vzniku zavady.
 *
 * Komunikacni protokol:
 *  - Request:  [0x02, PID, frame_number]
 *  - Response: [0x42, PID, frame_number, A, B, C, D]
 *
 * Parametr frame_number je typicky 0x00 (prvni ulozeny freeze frame).
 * Vetsinq ECU podporuje pouze frame 0. Nektera ECU (predevsim evropska)
 * mohou podporovat vice frame (0x01, 0x02...).
 *
 * Hranicni pripady:
 *  - Pokud ECU nema ulozeny freeze frame, odpovi NRC (negative response)
 *    nebo nedojde k odpovedi (timeout).
 *  - result == NULL: vraci OBD2_ERR_INVALID_ARG.
 *  - Neinicializovano: vraci OBD2_ERR_NOT_INITIALIZED.
 *  - Odpoved kratsi nez 4 byty: vraci OBD2_ERR_RESPONSE_MALFORMED.
 *  - Echovany PID neodpovida: vraci OBD2_ERR_RESPONSE_MALFORMED.
 *
 * @param pid     Cislo PIDu pro freeze frame (0x00--0xFF)
 * @param frame   Cislo freeze frame (typicky 0x00)
 * @param result  Ukazatel na strukturu pro vysledek (pid, data[], data_len)
 * @return OBD2_OK pri uspechu, jinak chybovy kod
 */
obd2_status_t obd2_get_freeze_frame_raw(uint8_t pid, uint8_t frame,
                                         obd2_pid_raw_t *result)
{
    OBD2_LOGI("get_freeze_frame_raw: pid=0x%02X frame=0x%02X", pid, frame);

    if (result == NULL) {
        OBD2_LOGE("get_freeze_frame_raw: NULL result pointer");
        return OBD2_ERR_INVALID_ARG;
    }
    if (!_ctx.initialized) {
        OBD2_LOGE("get_freeze_frame_raw: not initialized");
        return OBD2_ERR_NOT_INITIALIZED;
    }

    uint8_t req[3] = { OBD2_SID_FREEZE_FRAME, pid, frame };
    uint8_t resp[ISOTP_MAX_PAYLOAD];
    uint16_t resp_len = sizeof(resp);

    obd2_status_t st = _obd2_request(req, 3, resp, &resp_len, false);
    if (st != OBD2_OK) return st;

    /* Odpoved: [42, PID, frame#, data_A, ...] -- minimalne 4 byty */
    if (resp_len < 4) {
        OBD2_LOGE("get_freeze_frame_raw: response too short (%u bytes)", resp_len);
        return OBD2_ERR_RESPONSE_MALFORMED;
    }
    if (resp[1] != pid) {
        OBD2_LOGE("get_freeze_frame_raw: PID mismatch: got 0x%02X", resp[1]);
        return OBD2_ERR_RESPONSE_MALFORMED;
    }

    result->pid = pid;
    result->data_len = (resp_len - 3 > 4) ? 4 : (uint8_t)(resp_len - 3);
    memcpy(result->data, &resp[3], result->data_len);

    OBD2_LOGD("get_freeze_frame_raw: pid=0x%02X frame=0x%02X data_len=%u",
              pid, frame, result->data_len);
    return OBD2_OK;
}
