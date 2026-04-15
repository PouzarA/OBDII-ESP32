/**
 * @file obd2.c
 * @brief Jadro OBD-II diagnosticke vrstvy -- inicializace, konfigurace, ISO-TP komunikace
 *        a tabulka PID deskriptoru.
 *
 * Tento soubor je soucasti rozdeleni puvodni monoliticke implementace do tri souboru:
 *
 *   - obd2.c      (tento soubor) -- jadro: interni stav (_ctx), inicializace/deinicializace,
 *                  nastaveni ECU adresy a timeoutu, interni request helper (_obd2_request),
 *                  tabulka PID deskriptoru, vyhledavani v tabulce, zjistovani podporovanych
 *                  PIDu (Mode 01 PID $00/$20/$40...), prevod stavovych kodu na retezce.
 *
 *   - obd2_pids.c -- cteni a dekodovani PID hodnot (Mode 01/02): obd2_get_pid_raw,
 *                  obd2_get_pid, obd2_decode_pid_value, obd2_decode_pid_secondary,
 *                  obd2_get_monitor_status, obd2_get_freeze_frame_raw.
 *
 *   - obd2_diag.c -- diagnosticke funkce: cteni/mazani DTC (Mode 03/04/07),
 *                  VIN, nazev ECU, CalID (Mode 09), dekodovani DTC retezcu.
 *
 * Architektura sdileneho stavu:
 *   Arduino IDE kompiluje vsechny .c soubory ve slozce sketche do jednoho binarniho
 *   souboru. Promenne _ctx a funkce _obd2_request jsou deklarovany jako NE-staticke
 *   v tomto souboru a v obd2_pids.c / obd2_diag.c se k nim pristupuje pres extern.
 *
 * @see obd2.h pro verejne API a datove typy
 */

#include "obd2_internal.h"

/* ========================================================================= */
/*  Interni stav -- sdileny mezi obd2.c, obd2_pids.c, obd2_diag.c           */
/* ========================================================================= */

/**
 * @brief Aktualni uroven logovani pro OBD-II vrstvu.
 *
 * Vychozi hodnota ISOTP_LOG_TRACE zajisti, ze pri vyvoji jsou videt vsechny
 * zpravy. V produkci se typicky nastavi na ISOTP_LOG_WARN nebo ISOTP_LOG_ERROR
 * pomoci obd2_set_log_level(). Promennou pouzivaji makra OBD2_LOGx definovana
 * v obd2.h.
 */
isotp_log_level_t _obd2_runtime_log_level = ISOTP_LOG_TRACE;

/**
 * @brief Globalni kontext OBD-II vrstvy -- jedina instance, bez malloc.
 *
 * Struktura drzi veskerou konfiguraci a stav pro komunikaci s ECU:
 *
 *   tx_id          -- CAN ID pro odchozi pozadavky (vychozi 0x7E0 = fyzicke adresovani ECU #1).
 *   rx_id          -- CAN ID pro prijem odpovedi (vychozi 0x7E8 = odpoved od ECU #1).
 *   timeout_ms     -- maximalni doba cekani na odpoved v milisekundach.
 *   initialized    -- priznak, zda byla uspesne zavolana obd2_init().
 *   supported_pids -- pole 8 x uint32 = 256 bitu, bitmaska podporovanych PIDu.
 *   pids_queried   -- priznak, zda uz byla bitmaska nactena z ECU.
 *   last_nrc       -- posledni negativni odpoved (NRC) od ECU.
 *
 * Typedef _obd2_ctx_t je v obd2_internal.h. Promenna NENI static --
 * obd2_pids.c a obd2_diag.c k ni pristupuji pres extern deklaraci.
 */
_obd2_ctx_t _ctx = {
    .tx_id       = ISOTP_OBD_PHYS_REQ_BASE,   /* 0x7E0 */
    .rx_id       = ISOTP_OBD_PHYS_RESP_BASE,   /* 0x7E8 */
    .timeout_ms  = OBD2_DEFAULT_TIMEOUT_MS,
    .initialized = false,
    .pids_queried = false,
    .last_nrc    = { 0, 0 }
};

/* ========================================================================= */
/*  Stavove kody a NRC retezce                                               */
/* ========================================================================= */

/**
 * @brief Pole nazvu stavovych kodu OBD-II vrstvy pro prevod na retezec.
 *
 * Indexovano hodnotou obd2_status_t (0..9). Pouziva se ve funkci obd2_status_str().
 * Retezce jsou v anglictine, protoze jde o technicky identifikator pro logy a
 * diagnostiku (ne uzivatelske rozhrani).
 */
static const char *_obd2_status_names[] = {
    "OK", "TIMEOUT", "NEGATIVE_RESP", "NO_DATA", "INVALID_ARG",
    "NOT_INIT", "ISOTP_ERR", "UNSUPPORTED_PID", "DECODE_ERR", "MALFORMED"
};

/**
 * @brief Prevede stavovy kod OBD-II na citelny retezec.
 *
 * @param status Stavovy kod (napr. OBD2_OK, OBD2_ERR_TIMEOUT, ...)
 * @return Ukazatel na staticky retezec odpovidajici danemu kodu.
 *         Pokud je status mimo rozsah, vraci "UNKNOWN".
 *
 * Priklad pouziti:
 *   OBD2_LOGE("selhalo: %s", obd2_status_str(st));
 *   // Vypise napr. "selhalo: TIMEOUT"
 */
const char *obd2_status_str(obd2_status_t status)
{
    if (status >= 0 && status <= OBD2_ERR_RESPONSE_MALFORMED) {
        return _obd2_status_names[status];
    }
    return "UNKNOWN";
}

/**
 * @brief Prevede NRC (Negative Response Code) na citelny retezec dle ISO 14229 / ISO 15031-5.
 *
 * @param nrc Kod negativni odpovedi (napr. 0x10 = generalReject, 0x12 = subFunctionNotSupported)
 * @return Ukazatel na staticky retezec. Pro nezname NRC kody vraci "unknownNRC".
 *
 * NRC kody jsou definovany v ISO 14229-1 (UDS) a castecne prevzaty do OBD-II.
 * Nejcastejsi pripady v OBD-II:
 *   - 0x12 (subFunctionNotSupported) -- ECU nepodporuje dany PID/InfoType
 *   - 0x31 (requestOutOfRange)       -- parametr mimo povoleny rozsah
 *   - 0x22 (conditionsNotCorrect)    -- napr. motor nebezi, ale PID to vyzaduje
 */
const char *obd2_nrc_str(uint8_t nrc)
{
    switch (nrc) {
    case OBD2_NRC_GENERAL_REJECT:             return "generalReject";
    case OBD2_NRC_SERVICE_NOT_SUPPORTED:      return "serviceNotSupported";
    case OBD2_NRC_SUB_FUNCTION_NOT_SUPPORTED: return "subFunctionNotSupported";
    case OBD2_NRC_CONDITIONS_NOT_CORRECT:     return "conditionsNotCorrect";
    case OBD2_NRC_REQUEST_OUT_OF_RANGE:       return "requestOutOfRange";
    case OBD2_NRC_RESPONSE_PENDING:           return "responsePending";
    default:                                  return "unknownNRC";
    }
}

/* ========================================================================= */
/*  Tabulka PID deskriptoru -- ISO 15031-5:2006 Priloha B (kompletni)        */
/* ========================================================================= */

/*
 * Tabulka deskriptoru pro vsechny standardni PIDy Mode 01 ($00--$5E).
 *
 * Struktura kazdeho zaznamu (obd2_pid_desc_t):
 *   pid        -- ciselny identifikator PIDu (napr. 0x0C = otacky motoru)
 *   name       -- anglicky nazev dle ISO 15031-5 Prilohy B (nemenit -- pouziva se v JSON)
 *   unit       -- jednotka mereni (napr. "rpm", "kPa", "%", "\xC2\xB0C" pro stupen Celsia)
 *   data_len   -- pocet datovych bajtu v odpovedi ECU (1, 2 nebo 4)
 *   format     -- typ dekodovani (format vzorce):
 *                   OBD2_FMT_LINEAR_1B       -- jednobajtovy linearni: val = A * mult + offset
 *                                               Priklad: PID $05 teplota chlad. kapaliny:
 *                                               val = A * 1.0 + (-40), tj. 0..255 -> -40..215 stupnu C
 *                   OBD2_FMT_LINEAR_2B       -- dvoubajtovy linearni: val = (256*A+B) * mult + offset
 *                                               Priklad: PID $0C otacky: val = (256A+B) * 0.25 rpm
 *                   OBD2_FMT_SIGNED_OFFSET_1B -- se zapornymi hodnotami: val = (A-128) * mult
 *                                               Priklad: PID $06 STFT: val = (A-128) * 100/128 %
 *                   OBD2_FMT_SIGNED_2B       -- dvoubajtovy se znamenkem (int16_t):
 *                                               val = (int16_t)(256A+B) * mult + offset
 *                   OBD2_FMT_BIT_ENCODED     -- bitove pole (napr. PID $01 stav monitoru)
 *                   OBD2_FMT_O2_CONV         -- konvencni O2 senzor (napeti + STFT)
 *                   OBD2_FMT_O2_WIDE_EQ_V    -- sirokopasmovy O2 (ekvivalencni pomer + napeti)
 *                   OBD2_FMT_O2_WIDE_EQ_I    -- sirokopasmovy O2 (ekvivalencni pomer + proud)
 *                   OBD2_FMT_CONFIG           -- konfiguracni PID (surova hodnota bajtu A)
 *   multiplier -- nasobici koeficient ve vzorci
 *   offset     -- offset ve vzorci (posunuti nuloveho bodu)
 *
 * Vzorce byly overeny vuci originalni norme ISO 15031-5:2006, strany 115--147.
 *
 * Poznamky ke specifickyim skupinam PIDu:
 *   - PIDy $14--$1B: konvencni O2 senzory, prirazeni bank dle PID $13 (2 banky x 4 senzory)
 *   - PIDy $24--$2B: sirokopasmove O2 s ekvivalencnim pomerem a napetim
 *   - PIDy $34--$3B: sirokopasmove O2 s ekvivalencnim pomerem a proudem
 *   - PIDy $55--$58: mohou mit 1 nebo 2 datove bajty dle konfigurace bank;
 *     v tabulce uvadime data_len=1 (minimalni garantovana delka)
 *   - PIDy $5B--$5E: pochazi z pozdejsich revizi standardu (v ISO 15031-5:2006
 *     Priloha B tabulka B.72 jsou $5B--$FF oznaceny jako "reserved"), ale v praxi
 *     je podpora ze strany ECU bezna. Zahrnuty pro uplnost.
 */

static const obd2_pid_desc_t _pid_table[] = {
    /* ---- Bitmasky podporovanych PIDu (kazda pokryva rozsah 32 PIDu) ---- */
    { 0x00, "PIDs supported [01-20]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0 },
    { 0x20, "PIDs supported [21-40]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0 },
    { 0x40, "PIDs supported [41-60]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0 },
    { 0x60, "PIDs supported [61-80]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0 },
    { 0x80, "PIDs supported [81-A0]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0 },
    { 0xA0, "PIDs supported [A1-C0]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0 },
    { 0xC0, "PIDs supported [C1-E0]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PID $01: Stav monitoru (4 bajty, bitove pole) ---- */
    { 0x01, "Monitor status since DTCs cleared",    "",    4, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PID $02: Zmrazeny DTC (Freeze Frame DTC) ---- */
    { 0x02, "Freeze DTC",                          "",    2, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PID $03: Stav palivoveho systemu (bitove pole) ---- */
    { 0x03, "Fuel system status",                  "",    2, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PIDy $04--$05: jednobajtove linearni hodnoty ---- */
    { 0x04, "Calculated engine load",              "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x05, "Engine coolant temperature",          "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f },

    /* ---- PIDy $06--$09: Palive korekce (offset se znamenkem, stred na 128) ---- */
    { 0x06, "Short term fuel trim - Bank 1",       "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },
    { 0x07, "Long term fuel trim - Bank 1",        "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },
    { 0x08, "Short term fuel trim - Bank 2",       "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },
    { 0x09, "Long term fuel trim - Bank 2",        "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },

    /* ---- PID $0A: Tlak paliva ---- */
    { 0x0A, "Fuel pressure",                       "kPa", 1, OBD2_FMT_LINEAR_1B, 3.0f, 0 },

    /* ---- PID $0B: Absolutni tlak v sacim potrubi (MAP senzor) ---- */
    { 0x0B, "Intake manifold absolute pressure",   "kPa", 1, OBD2_FMT_LINEAR_1B, 1.0f, 0 },

    /* ---- PID $0C: Otacky motoru ---- */
    { 0x0C, "Engine RPM",                          "rpm", 2, OBD2_FMT_LINEAR_2B, 0.25f, 0 },

    /* ---- PID $0D: Rychlost vozidla ---- */
    { 0x0D, "Vehicle speed",                       "km/h", 1, OBD2_FMT_LINEAR_1B, 1.0f, 0 },

    /* ---- PID $0E: Predstiho zapalovania ---- */
    { 0x0E, "Timing advance",                      "\xC2\xB0", 1, OBD2_FMT_LINEAR_1B, 0.5f, -64.0f },

    /* ---- PID $0F: Teplota nasavaneho vzduchu ---- */
    { 0x0F, "Intake air temperature",              "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f },

    /* ---- PID $10: Hmotnostni prutok vzduchu (MAF) ---- */
    { 0x10, "MAF air flow rate",                   "g/s", 2, OBD2_FMT_LINEAR_2B, 0.01f, 0 },

    /* ---- PID $11: Poloha skrtici klapky ---- */
    { 0x11, "Throttle position",                   "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },

    /* ---- PID $12: Stav sekundarniho vzduchu (bitove pole) ---- */
    { 0x12, "Commanded secondary air status",      "",    1, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PID $13: Pritomnost O2 senzoru, 2 banky (bitove pole) ---- */
    { 0x13, "Oxygen sensors present (2 banks)",    "",    1, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PIDy $14--$1B: Konvencni O2 senzory (napeti + kratkodoba korekce STFT) ---- */
    { 0x14, "O2 Sensor B1S1 Voltage",             "V",   2, OBD2_FMT_O2_CONV, 0, 0 },
    { 0x15, "O2 Sensor B1S2 Voltage",             "V",   2, OBD2_FMT_O2_CONV, 0, 0 },
    { 0x16, "O2 Sensor B1S3 Voltage",             "V",   2, OBD2_FMT_O2_CONV, 0, 0 },
    { 0x17, "O2 Sensor B1S4 Voltage",             "V",   2, OBD2_FMT_O2_CONV, 0, 0 },
    { 0x18, "O2 Sensor B2S1 Voltage",             "V",   2, OBD2_FMT_O2_CONV, 0, 0 },
    { 0x19, "O2 Sensor B2S2 Voltage",             "V",   2, OBD2_FMT_O2_CONV, 0, 0 },
    { 0x1A, "O2 Sensor B2S3 Voltage",             "V",   2, OBD2_FMT_O2_CONV, 0, 0 },
    { 0x1B, "O2 Sensor B2S4 Voltage",             "V",   2, OBD2_FMT_O2_CONV, 0, 0 },

    /* ---- PID $1C: Standard OBD, kteremu vozidlo vyhovuje (stavove pole) ---- */
    { 0x1C, "OBD standards this vehicle conforms to", "", 1, OBD2_FMT_CONFIG, 0, 0 },

    /* ---- PID $1D: Pritomnost O2 senzoru, 4 banky (bitove pole) ---- */
    { 0x1D, "Oxygen sensors present (4 banks)",   "",    1, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PID $1E: Stav pomocneho vstupu (PTO status) ---- */
    { 0x1E, "Auxiliary input status",              "",    1, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PID $1F: Doba behu motoru od startu ---- */
    { 0x1F, "Run time since engine start",         "s",   2, OBD2_FMT_LINEAR_2B, 1.0f, 0 },

    /* ---- PID $21: Ujeta vzdalenost s rozsvicenou MIL kontrolkou ---- */
    { 0x21, "Distance traveled with MIL on",       "km",  2, OBD2_FMT_LINEAR_2B, 1.0f, 0 },

    /* ---- PID $22: Tlak v palivovem potrubi relativne k podtlaku saciku ---- */
    { 0x22, "Fuel rail pressure (relative)",       "kPa", 2, OBD2_FMT_LINEAR_2B, 0.079f, 0 },

    /* ---- PID $23: Tlak v palivovem potrubi -- primeho vstrikku (diesel/GDI) ---- */
    { 0x23, "Fuel rail gauge pressure",            "kPa", 2, OBD2_FMT_LINEAR_2B, 10.0f, 0 },

    /* ---- PIDy $24--$2B: Sirokopasmove O2 senzory (ekvivalencni pomer + napeti) ---- */
    { 0x24, "O2 Sensor B1S1 Equivalence Ratio",   "",    4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0 },
    { 0x25, "O2 Sensor B1S2 Equivalence Ratio",   "",    4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0 },
    { 0x26, "O2 Sensor B1S3 Equivalence Ratio",   "",    4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0 },
    { 0x27, "O2 Sensor B1S4 Equivalence Ratio",   "",    4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0 },
    { 0x28, "O2 Sensor B2S1 Equivalence Ratio",   "",    4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0 },
    { 0x29, "O2 Sensor B2S2 Equivalence Ratio",   "",    4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0 },
    { 0x2A, "O2 Sensor B2S3 Equivalence Ratio",   "",    4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0 },
    { 0x2B, "O2 Sensor B2S4 Equivalence Ratio",   "",    4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0 },

    /* ---- PIDy $2C--$2E: EGR a odparovani ---- */
    { 0x2C, "Commanded EGR",                       "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x2D, "EGR error",                           "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },
    { 0x2E, "Commanded evaporative purge",         "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },

    /* ---- PID $2F: Uroven paliva v nadrzi ---- */
    { 0x2F, "Fuel tank level input",               "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },

    /* ---- PID $30: Pocet zahrivacich cyklu od posledniho smazani kodu ---- */
    { 0x30, "Warm-ups since codes cleared",        "",    1, OBD2_FMT_LINEAR_1B, 1.0f, 0 },

    /* ---- PID $31: Ujeta vzdalenost od smazani kodu ---- */
    { 0x31, "Distance traveled since codes cleared", "km", 2, OBD2_FMT_LINEAR_2B, 1.0f, 0 },

    /* ---- PID $32: Tlak par v odparovacim systemu (se znamenkem) ---- */
    { 0x32, "Evap system vapor pressure",          "Pa",  2, OBD2_FMT_SIGNED_2B, 0.25f, 0 },

    /* ---- PID $33: Absolutni barometricky tlak ---- */
    { 0x33, "Absolute barometric pressure",        "kPa", 1, OBD2_FMT_LINEAR_1B, 1.0f, 0 },

    /* ---- PIDy $34--$3B: Sirokopasmove O2 senzory (ekvivalencni pomer + proud) ---- */
    { 0x34, "O2 Sensor B1S1 EQ Ratio (current)",  "",    4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0 },
    { 0x35, "O2 Sensor B1S2 EQ Ratio (current)",  "",    4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0 },
    { 0x36, "O2 Sensor B1S3 EQ Ratio (current)",  "",    4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0 },
    { 0x37, "O2 Sensor B1S4 EQ Ratio (current)",  "",    4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0 },
    { 0x38, "O2 Sensor B2S1 EQ Ratio (current)",  "",    4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0 },
    { 0x39, "O2 Sensor B2S2 EQ Ratio (current)",  "",    4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0 },
    { 0x3A, "O2 Sensor B2S3 EQ Ratio (current)",  "",    4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0 },
    { 0x3B, "O2 Sensor B2S4 EQ Ratio (current)",  "",    4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0 },

    /* ---- PIDy $3C--$3F: Teplota katalyzatoru ---- */
    { 0x3C, "Catalyst temperature B1S1",           "\xC2\xB0""C", 2, OBD2_FMT_LINEAR_2B, 0.1f, -40.0f },
    { 0x3D, "Catalyst temperature B2S1",           "\xC2\xB0""C", 2, OBD2_FMT_LINEAR_2B, 0.1f, -40.0f },
    { 0x3E, "Catalyst temperature B1S2",           "\xC2\xB0""C", 2, OBD2_FMT_LINEAR_2B, 0.1f, -40.0f },
    { 0x3F, "Catalyst temperature B2S2",           "\xC2\xB0""C", 2, OBD2_FMT_LINEAR_2B, 0.1f, -40.0f },

    /* ---- PID $41: Stav monitoru v aktualnim jezdnim cyklu (bitove pole) ---- */
    { 0x41, "Monitor status this drive cycle",     "",    4, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PID $42: Napeti ridici jednotky ---- */
    { 0x42, "Control module voltage",              "V",   2, OBD2_FMT_LINEAR_2B, 0.001f, 0 },

    /* ---- PID $43: Absolutni hodnota zateze ---- */
    { 0x43, "Absolute load value",                 "%",   2, OBD2_FMT_LINEAR_2B, 100.0f/255.0f, 0 },

    /* ---- PID $44: Prikazany ekvivalencni pomer (lambda) ---- */
    { 0x44, "Commanded equivalence ratio",         "",    2, OBD2_FMT_LINEAR_2B, 2.0f/65535.0f, 0 },

    /* ---- PIDy $45--$4C: Polohy skrtici klapky a pedalu ---- */
    { 0x45, "Relative throttle position",          "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x46, "Ambient air temperature",             "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f },
    { 0x47, "Absolute throttle position B",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x48, "Absolute throttle position C",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x49, "Accelerator pedal position D",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x4A, "Accelerator pedal position E",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x4B, "Accelerator pedal position F",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x4C, "Commanded throttle actuator",         "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },

    /* ---- PIDy $4D--$4E: Casove pocitadla ---- */
    { 0x4D, "Time run with MIL on",               "min", 2, OBD2_FMT_LINEAR_2B, 1.0f, 0 },
    { 0x4E, "Time since trouble codes cleared",    "min", 2, OBD2_FMT_LINEAR_2B, 1.0f, 0 },

    /* ---- PIDy $4F--$50: Konfiguracni PIDy (maximalni hodnoty, nejsou pro zobrazeni) ---- */
    { 0x4F, "Max EQ ratio / O2V / O2I / MAP",     "",    4, OBD2_FMT_CONFIG, 0, 0 },
    { 0x50, "Max MAF air flow rate",               "",    4, OBD2_FMT_CONFIG, 0, 0 },

    /* ---- PID $51: Typ paliva (stavove pole) ---- */
    { 0x51, "Fuel type",                           "",    1, OBD2_FMT_BIT_ENCODED, 0, 0 },

    /* ---- PID $52: Procentualni obsah ethanolu v palivu ---- */
    { 0x52, "Ethanol fuel percentage",             "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },

    /* ---- PID $53: Absolutni tlak par v odparovacim systemu ---- */
    { 0x53, "Absolute evap system vapor pressure", "kPa", 2, OBD2_FMT_LINEAR_2B, 0.005f, 0 },

    /* ---- PID $54: Tlak par v odparovacim systemu (se znamenkem, sirsi rozsah) ---- */
    { 0x54, "Evap system vapor pressure",          "Pa",  2, OBD2_FMT_SIGNED_2B, 1.0f, 0 },

    /* ---- PIDy $55--$58: Korekce sekundarniho O2 senzoru ---- */
    { 0x55, "Short term secondary O2 trim B1",    "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },
    { 0x56, "Long term secondary O2 trim B1",     "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },
    { 0x57, "Short term secondary O2 trim B2",    "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },
    { 0x58, "Long term secondary O2 trim B2",     "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0 },

    /* ---- PID $59: Absolutni tlak v palivovem potrubi ---- */
    { 0x59, "Fuel rail absolute pressure",         "kPa", 2, OBD2_FMT_LINEAR_2B, 10.0f, 0 },

    /* ---- PID $5A: Relativni poloha pedalu plynu ---- */
    { 0x5A, "Relative accelerator pedal position", "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },

    /* ---- PIDy $5B--$5E: Rozsirene PIDy (pozdejsi revize standardu, siroko podporovane) ---- */
    { 0x5B, "Hybrid battery pack remaining life",  "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0 },
    { 0x5C, "Engine oil temperature",              "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f },
    { 0x5D, "Fuel injection timing",               "\xC2\xB0", 2, OBD2_FMT_LINEAR_2B, 1.0f/128.0f, -210.0f },
    { 0x5E, "Engine fuel rate",                    "L/h", 2, OBD2_FMT_LINEAR_2B, 0.05f, 0 },
};

/** Pocet zaznamu v tabulce PID deskriptoru (vyhodnoceno v dobe kompilace) */
#define PID_TABLE_SIZE  (sizeof(_pid_table) / sizeof(_pid_table[0]))

/* ========================================================================= */
/*  Vyhledani v PID tabulce                                                  */
/* ========================================================================= */

/**
 * @brief Vyhleda deskriptor PIDu v interni tabulce podle cisla PIDu.
 *
 * Pouziva linearni vyhledavani (O(n), kde n ~ 70 zaznamu). Hashovaci tabulka
 * neni nutna -- pri 70 zaznamech je linearni prohledavani dostatecne rychle
 * a setri RAM na ESP32. Typicka doba vyhledani je pod 1 us.
 *
 * @param pid Cislo PIDu (napr. 0x0C pro otacky motoru)
 * @return Ukazatel na nalezeny deskriptor, nebo NULL pokud PID neni v tabulce.
 *         Vraci ukazatel primo do statickeho pole -- nevolat free()!
 *
 * Priklad:
 *   const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(0x0C);
 *   if (desc) printf("Nazev: %s, jednotka: %s\n", desc->name, desc->unit);
 */
const obd2_pid_desc_t *obd2_get_pid_descriptor(uint8_t pid)
{
    for (unsigned i = 0; i < PID_TABLE_SIZE; i++) {
        if (_pid_table[i].pid == pid) {
            return &_pid_table[i];
        }
    }
    return NULL;
}

/* ========================================================================= */
/*  Logovani                                                                 */
/* ========================================================================= */

/**
 * @brief Nastavi uroven logovani pro OBD-II vrstvu.
 *
 * Ovlivnuje, ktere zpravy se budou vypisovat pres OBD2_LOGx makra.
 * Urovne (od nejpodrobnejsi): TRACE, DEBUG, INFO, WARN, ERROR, NONE.
 *
 * @param level Nova uroven logovani (napr. ISOTP_LOG_INFO pro bezny provoz)
 */
void obd2_set_log_level(isotp_log_level_t level)
{
    _obd2_runtime_log_level = level;
    OBD2_LOGI("Log level set to %d", level);
}

/* ========================================================================= */
/*  Zivotni cyklus (inicializace, deinicializace, konfigurace)               */
/* ========================================================================= */

/**
 * @brief Inicializuje OBD-II vrstvu vcetne TWAI/CAN a ISO-TP.
 *
 * Posloupnost inicializace:
 *   1) Pokud uz bylo inicializovano, nejprve zavola obd2_deinit()
 *      (umozni opakovane volani bez memory leaku).
 *   2) Inicializuje ISO-TP vrstvu (ta inicializuje TWAI driver na ESP32).
 *   3) Nastavi vychozi adresy 0x7E0 (TX) / 0x7E8 (RX) -- fyzicke adresovani
 *      ECU #1 dle ISO 15765-4.
 *   4) Vynuluje cache podporovanych PIDu a NRC info.
 *
 * Vychozi adresy:
 *   0x7E0 = fyzicky pozadavek na ECU #1 (rozsah 0x7E0--0x7E7 pro ECU #1 az #8)
 *   0x7E8 = odpoved od ECU #1 (rozsah 0x7E8--0x7EF)
 *   0x7DF = broadcast (funkcni adresovani) -- pouziva se v _obd2_request s use_broadcast=true
 *
 * @param baudrate  Rychlost CAN sbernice v bit/s (typicky 500000 pro OBD-II)
 * @param tx_pin    GPIO pin pro CAN TX (na ESP32 napr. GPIO_NUM_5)
 * @param rx_pin    GPIO pin pro CAN RX (na ESP32 napr. GPIO_NUM_4)
 * @return OBD2_OK pri uspechu, OBD2_ERR_ISOTP pokud inicializace ISO-TP/TWAI selze
 */
obd2_status_t obd2_init(uint32_t baudrate, int tx_pin, int rx_pin)
{
    OBD2_LOGI("init: baudrate=%lu tx_pin=%d rx_pin=%d",
              (unsigned long)baudrate, tx_pin, rx_pin);

    if (_ctx.initialized) {
        OBD2_LOGW("init: already initialized, deinit first");
        obd2_deinit();
    }

    isotp_status_t ist = isotp_init(baudrate, tx_pin, rx_pin);
    if (ist != ISOTP_OK) {
        OBD2_LOGE("init: isotp_init failed: %s", isotp_status_str(ist));
        return OBD2_ERR_ISOTP;
    }

    _ctx.tx_id       = ISOTP_OBD_PHYS_REQ_BASE;
    _ctx.rx_id       = ISOTP_OBD_PHYS_RESP_BASE;
    _ctx.timeout_ms  = OBD2_DEFAULT_TIMEOUT_MS;
    _ctx.pids_queried = false;
    memset(_ctx.supported_pids, 0, sizeof(_ctx.supported_pids));
    memset(&_ctx.last_nrc, 0, sizeof(_ctx.last_nrc));
    _ctx.initialized = true;

    OBD2_LOGI("init: OK (tx=0x%03X rx=0x%03X timeout=%lums)",
              _ctx.tx_id, _ctx.rx_id, (unsigned long)_ctx.timeout_ms);
    return OBD2_OK;
}

/**
 * @brief Deinicializuje OBD-II vrstvu a uvolni ISO-TP/TWAI prostredky.
 *
 * Po zavolani je nutne pred dalsi komunikaci znovu zavolat obd2_init().
 * Bezpecne volani i pokud nebylo inicializovano (nic neprovede).
 */
void obd2_deinit(void)
{
    OBD2_LOGI("deinit");
    if (_ctx.initialized) {
        isotp_deinit();
        _ctx.initialized = false;
        _ctx.pids_queried = false;
    }
}

/**
 * @brief Nastavi CAN adresy pro komunikaci s konkretni ECU.
 *
 * Pouziti pro prepnuti z vychozi ECU #1 (0x7E0/0x7E8) na jinou ECU.
 * OBD-II definuje 8 fyzickych adres:
 *   ECU #1: TX=0x7E0, RX=0x7E8
 *   ECU #2: TX=0x7E1, RX=0x7E9
 *   ...
 *   ECU #8: TX=0x7E7, RX=0x7EF
 *
 * Broadcast adresa 0x7DF se nastavuje automaticky pri use_broadcast=true
 * v _obd2_request a neni potreba ji menit pres tuto funkci.
 *
 * @param tx_id CAN ID pro odeslani pozadavku (napr. 0x7E0)
 * @param rx_id CAN ID pro prijem odpovedi (napr. 0x7E8)
 */
void obd2_set_ecu_address(uint32_t tx_id, uint32_t rx_id)
{
    OBD2_LOGI("set_ecu_address: tx=0x%03X rx=0x%03X", tx_id, rx_id);
    _ctx.tx_id = tx_id;
    _ctx.rx_id = rx_id;
}

/**
 * @brief Nastavi timeout pro cekani na odpoved od ECU.
 *
 * Vychozi hodnota je OBD2_DEFAULT_TIMEOUT_MS (typicky 2000 ms).
 * Kratsi timeout muze byt vhodny pro rychle dotazovani v realnemu case,
 * delsi pro pomale ECU nebo pri cteni VIN (Mode 09, multi-frame odpoved).
 *
 * @param timeout_ms Timeout v milisekundach (napr. 1000 pro 1 sekundu)
 */
void obd2_set_timeout(uint32_t timeout_ms)
{
    OBD2_LOGD("set_timeout: %lu ms", (unsigned long)timeout_ms);
    _ctx.timeout_ms = timeout_ms;
}

/**
 * @brief Vrati informace o posledni negativni odpovedi (NRC) od ECU.
 *
 * Uzitecne po obdrzeni OBD2_ERR_NEGATIVE_RESP pro zjisteni presneho duvodu
 * odmitnuti pozadavku. Struktura obsahuje:
 *   request_sid -- SID pozadavku, ktery byl odmitnut
 *   nrc         -- kod NRC (napr. 0x12 = subFunctionNotSupported)
 *
 * @return Kopie struktury obd2_nrc_info_t s poslednimi hodnotami.
 *         Pokud dosud zadna NRC odpoved neprisla, oba cleny jsou 0.
 */
obd2_nrc_info_t obd2_get_last_nrc(void)
{
    return _ctx.last_nrc;
}

/* ========================================================================= */
/*  Interni request helper -- jadro ISO-TP komunikace                        */
/* ========================================================================= */

/**
 * @brief Odesle OBD-II pozadavek pres ISO-TP a validuje odpoved.
 *
 * Toto je centralni funkce pro veskerou komunikaci s ECU. Vsechny verejne
 * funkce (obd2_get_pid_raw, obd2_read_dtc, obd2_read_vin atd.) ji pouzivaji.
 *
 * Funkcionalita:
 *   1) Kontrola inicializace a validita parametru.
 *   2) Odeslani pozadavku pres ISO-TP (fyzicke nebo broadcast adresovani).
 *   3) Prijem a validace odpovedi:
 *      - Detekce negativni odpovedi (SID 0x7F) s extrakci NRC kodu.
 *      - Overeni, ze SID odpovedi odpovida pozadavku (SID + 0x40).
 *
 * Broadcast vs. fyzicke adresovani:
 *   - Broadcast (0x7DF): pouziva se pro zjisteni podporovanych PIDu, cteni DTC
 *     a mazani DTC. Vsechny ECU odpovedi, pouzijeme prvni validni.
 *   - Fyzicke (0x7E0/0x7E8): pouziva se pro cteni konkretniho PIDu, VIN atd.
 *
 * Bezpecnost zasobniku:
 *   Broadcast rezim pouziva static isotp_result_t (~2.1 KB), protoze alokace
 *   na zasobniku FreeRTOS tasku (typicky 4-8 KB) by zpusobila preteceni.
 *   Dusledek: funkce neni reentrantni, ale to je v poradku -- OBD-II pozadavky
 *   se zpracovavaji sekvencne z jednoho tasku.
 *
 * @param req            Ukazatel na pozadavek (SID + parametry), napr. {0x01, 0x0C} pro PID $0C
 * @param req_len        Delka pozadavku v bajtech (1--7)
 * @param resp           Buffer pro odpoved (musi byt dostatecne velky, typicky ISOTP_MAX_PAYLOAD)
 * @param resp_len       Vstup: velikost bufferu; vystup: skutecna delka odpovedi
 * @param use_broadcast  true = broadcast na 0x7DF, false = fyzicke adresovani na _ctx.tx_id
 * @return OBD2_OK pri uspechu, nebo prislusny chybovy kod
 */
/* NENI static -- pouzivana v obd2_pids.c a obd2_diag.c pres extern */
obd2_status_t _obd2_request(const uint8_t *req, uint8_t req_len,
                             uint8_t *resp, uint16_t *resp_len,
                             bool use_broadcast)
{
    if (!_ctx.initialized) {
        OBD2_LOGE("request: not initialized");
        return OBD2_ERR_NOT_INITIALIZED;
    }
    if (req == NULL || resp == NULL || resp_len == NULL || req_len == 0) {
        OBD2_LOGE("request: invalid argument");
        return OBD2_ERR_INVALID_ARG;
    }

    uint8_t expected_resp_sid = req[0] + OBD2_SID_RESPONSE_OFFSET;

    OBD2_LOGD("request: SID=0x%02X len=%u broadcast=%d",
              req[0], req_len, use_broadcast);

    isotp_status_t ist;

    if (use_broadcast) {
        /*
         * Broadcast: odeslani na 0x7DF, prijem odpovedi od vsech ECU.
         *
         * DULEZITE: isotp_result_t zabira ~2.1 KB (8 x isotp_response_t,
         * kazda s 256B bufferem). Alokace na stacku zpusobuje preteceni
         * v FreeRTOS tasku (typicky 4-8 KB stack), coz vede k
         * "Guru Meditation Error: StoreProhibited/LoadProhibited".
         *
         * Reseni: static promenna -- ulozena v .bss segmentu (globalni RAM),
         * nezabira misto na stacku. Pred kazdym pouzitim nulovana pres
         * memset, protoze static promenna si jinak drzi data z predchoziho
         * volani.
         *
         * Omezeni: funkce neni reentrantni (static buffer je sdileny).
         * To je v poradku -- OBD pozadavky se zpracovavaji sekvencne
         * z jednoho tasku a CAN sbernice je polo-duplexni.
         */
        static isotp_result_t bcast_result;
        memset(&bcast_result, 0, sizeof(bcast_result));

        ist = isotp_transaction_broadcast(req, req_len, &bcast_result,
                                          _ctx.timeout_ms);
        if (ist != ISOTP_OK) {
            OBD2_LOGE("request: broadcast isotp error: %s",
                      isotp_status_str(ist));
            return (ist == ISOTP_ERR_TIMEOUT) ? OBD2_ERR_TIMEOUT
                                              : OBD2_ERR_ISOTP;
        }
        if (bcast_result.count == 0) {
            OBD2_LOGW("request: broadcast got 0 responses");
            return OBD2_ERR_NO_DATA;
        }

        /* Pouzijeme prvni validni odpoved z broadcast vysledku */
        bool found = false;
        for (uint8_t i = 0; i < bcast_result.count; i++) {
            if (bcast_result.responses[i].valid &&
                bcast_result.responses[i].len > 0) {
                uint16_t copy_len = bcast_result.responses[i].len;
                if (copy_len > *resp_len) copy_len = *resp_len;
                memcpy(resp, bcast_result.responses[i].data, copy_len);
                *resp_len = copy_len;
                found = true;
                OBD2_LOGD("request: using response from ECU 0x%03X (%u bytes)",
                          bcast_result.responses[i].rx_id, copy_len);
                break;
            }
        }
        if (!found) {
            OBD2_LOGW("request: no valid broadcast response");
            return OBD2_ERR_NO_DATA;
        }
    } else {
        /* Fyzicke adresovani: primo na nakonfigurovanou ECU */
        ist = isotp_transaction(_ctx.tx_id, _ctx.rx_id,
                                req, req_len,
                                resp, resp_len,
                                _ctx.timeout_ms);
        if (ist != ISOTP_OK) {
            OBD2_LOGE("request: isotp error: %s", isotp_status_str(ist));
            return (ist == ISOTP_ERR_TIMEOUT) ? OBD2_ERR_TIMEOUT
                                              : OBD2_ERR_ISOTP;
        }
    }

    /* Kontrola negativni odpovedi (SID 0x7F = negative response) */
    if (*resp_len >= 3 && resp[0] == OBD2_SID_NEGATIVE_RESPONSE) {
        _ctx.last_nrc.request_sid = resp[1];
        _ctx.last_nrc.nrc = resp[2];
        OBD2_LOGW("request: negative response for SID 0x%02X, NRC=0x%02X (%s)",
                  resp[1], resp[2], obd2_nrc_str(resp[2]));
        return OBD2_ERR_NEGATIVE_RESP;
    }

    /* Validace SID odpovedi -- musi byt pozadovany SID + 0x40 */
    if (*resp_len < 1 || resp[0] != expected_resp_sid) {
        OBD2_LOGE("request: unexpected response SID=0x%02X (expected 0x%02X)",
                  resp[0], expected_resp_sid);
        return OBD2_ERR_RESPONSE_MALFORMED;
    }

    OBD2_LOGD("request: OK, resp_len=%u resp_SID=0x%02X",
              *resp_len, resp[0]);
    return OBD2_OK;
}

/* ========================================================================= */
/*  Zjistovani podporovanych PIDu (Mode 01, PID $00/$20/$40...)              */
/* ========================================================================= */

/**
 * @brief Dotaz na ECU pro zjisteni vsech podporovanych PIDu (Mode 01).
 *
 * Iteruje pres rozsahy PIDu: $00, $20, $40, $60, $80, $A0, $C0, $E0.
 * Kazdy rozsah vraci 4-bajtovou bitmasku pokryvajici 32 PIDu.
 *
 * Princip bitmasky:
 *   - Odpoved na PID $00: [41, 00, A, B, C, D]
 *   - Bit 7 bajtu A = PID $01 podporovan
 *   - Bit 6 bajtu A = PID $02 podporovan
 *   - ...
 *   - Bit 0 bajtu D (= bit D0) = PID $20 podporovan = "dalsi rozsah existuje"
 *
 * Pravidla iterace:
 *   - Prvni rozsah ($00) MUSI uspet, jinak se vraci chyba.
 *   - Pokud dalsi rozsah selze, je to normalni (ECU nepodporuje vyssi PIDy).
 *   - Pokud bit D0 (posledni bit) neni nastaven, dalsi rozsah neexistuje -- konec.
 *   - Vysledky se ukladaji do _ctx.supported_pids[0..7].
 *
 * Po uspesnem volani je mozne pouzit obd2_is_pid_supported() pro dotaz
 * na konkretni PID.
 *
 * @return OBD2_OK pri uspechu, chybovy kod pokud PID $00 selze
 */
obd2_status_t obd2_query_supported_pids(void)
{
    OBD2_LOGI("query_supported_pids: starting");

    if (!_ctx.initialized) {
        OBD2_LOGE("query_supported_pids: not initialized");
        return OBD2_ERR_NOT_INITIALIZED;
    }

    memset(_ctx.supported_pids, 0, sizeof(_ctx.supported_pids));
    _ctx.pids_queried = false;

    /*
     * Iterace pres rozsahy PIDu: $00, $20, $40, $60, $80, $A0, $C0, $E0.
     * Kazdy vraci 4-bajtovou bitmasku pro dalsich 32 PIDu.
     * Iterace konci, kdyz posledni bit (indikujici existenci dalsiho rozsahu) je 0.
     * ISO 15031-5 Priloha A: bit D0 kazdeho rozsahu = dalsi rozsah podporovan.
     */
    for (uint8_t range_pid = 0x00; range_pid <= 0xE0; range_pid += 0x20) {
        uint8_t req[2] = { OBD2_SID_CURRENT_DATA, range_pid };
        uint8_t resp[ISOTP_MAX_PAYLOAD];
        uint16_t resp_len = sizeof(resp);

        OBD2_LOGD("query_supported_pids: requesting PID 0x%02X", range_pid);

        obd2_status_t st = _obd2_request(req, 2, resp, &resp_len, true);

        if (st != OBD2_OK) {
            if (range_pid == 0x00) {
                /* Prvni rozsah musi uspet -- bez neho neni mozne zjistit cokoliv */
                OBD2_LOGE("query_supported_pids: PID $00 failed: %s",
                          obd2_status_str(st));
                return st;
            }
            /* Selhani dalsich rozsahu je normalni -- ECU je proste nepodporuje */
            OBD2_LOGD("query_supported_pids: range 0x%02X not available",
                      range_pid);
            break;
        }

        /* Odpoved: [41, range_pid, A, B, C, D] -- potrebujeme minimalne 6 bajtu */
        if (resp_len < 6) {
            OBD2_LOGW("query_supported_pids: short response for 0x%02X (%u bytes)",
                      range_pid, resp_len);
            if (range_pid == 0x00) return OBD2_ERR_RESPONSE_MALFORMED;
            break;
        }

        /* Overeni, ze ECU zopakovala spravny PID v odpovedi */
        if (resp[1] != range_pid) {
            OBD2_LOGW("query_supported_pids: PID mismatch: got 0x%02X, expected 0x%02X",
                      resp[1], range_pid);
            break;
        }

        /* Ulozeni 32-bitove bitmasky do prislusneho indexu pole */
        uint8_t idx = range_pid / 0x20;  /* 0, 1, 2, ... 7 */
        _ctx.supported_pids[idx] = ((uint32_t)resp[2] << 24) |
                                    ((uint32_t)resp[3] << 16) |
                                    ((uint32_t)resp[4] << 8)  |
                                    ((uint32_t)resp[5]);

        OBD2_LOGD("query_supported_pids: range 0x%02X bitmask=0x%08lX",
                  range_pid, (unsigned long)_ctx.supported_pids[idx]);

        /* Kontrola, zda bit 0 (= dalsi rozsah podporovan) je nastaven */
        if ((_ctx.supported_pids[idx] & 0x00000001) == 0) {
            OBD2_LOGD("query_supported_pids: no more ranges after 0x%02X",
                      range_pid);
            break;
        }
    }

    _ctx.pids_queried = true;

    /* Spocitani podporovanych PIDu pro informacni log */
    uint16_t count = 0;
    for (uint16_t pid = 0x01; pid <= 0xFF; pid++) {
        if (obd2_is_pid_supported((uint8_t)pid)) count++;
    }
    OBD2_LOGI("query_supported_pids: done, %u PIDs supported", count);

    return OBD2_OK;
}

/**
 * @brief Zjisti, zda je dany PID podporovan ECU (na zaklade drive nactene bitmasky).
 *
 * Pred volanim je nutne zavolat obd2_query_supported_pids(). Pokud nebyl
 * dotaz proveden (_ctx.pids_queried == false), vraci vzdy false.
 *
 * Vzorec pro urceni pozice bitu:
 *   range_idx  = (pid - 1) / 32    -- index do pole supported_pids[]
 *   bit_pos    = 31 - ((pid - 1) % 32)  -- pozice bitu v 32-bitovem slove
 *
 * Priklad pro PID $0C (otacky motoru = 12 desitkove):
 *   range_idx = (12 - 1) / 32 = 0      -- prvni rozsah (PID $00)
 *   bit_pos   = 31 - ((12 - 1) % 32) = 31 - 11 = 20
 *   -> testujeme bit 20 v supported_pids[0]
 *
 * Specialni pripad: PID $00 (dotaz na podporu) vraci vzdy false,
 * protoze nema smysl testovat jeho podporu -- je to sam dotazovaci mechanismus.
 *
 * @param pid Cislo PIDu (0x01--0xFF)
 * @return true pokud je PID podporovan, false pokud ne nebo pokud nebyl dotaz proveden
 */
bool obd2_is_pid_supported(uint8_t pid)
{
    if (!_ctx.pids_queried || pid == 0x00) {
        return false;
    }

    /*
     * PID $01 je bit 7 bajtu A bitmasky rozsahu $00.
     * PID $20 je bit 0 bajtu D bitmasky rozsahu $00 (= priznak "dalsi rozsah").
     *
     * Obecny vzorec:
     *   range_index = (pid - 1) / 32
     *   bit_position = 31 - ((pid - 1) % 32)
     */
    uint8_t range_idx = (pid - 1) / 32;
    uint8_t bit_pos   = 31 - ((pid - 1) % 32);

    if (range_idx >= 8) return false;

    return (_ctx.supported_pids[range_idx] & (1UL << bit_pos)) != 0;
}
