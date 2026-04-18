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
 * @author Ales Pouzar, vycházel jsem z ISO 15031-5/-6, SAE J1979, J2012, a dokumentace a wikipedie pro dostupne PIDs, viz take obd2.h      
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
    .baudrate    = 0,
    .tx_pin      = -1,
    .rx_pin      = -1,
    .timeout_ms  = OBD2_DEFAULT_TIMEOUT_MS,
    .initialized = false,
    .active_ecu_bound = false,
    .pids_queried = false,
    .last_nrc    = { 0, 0 },
    .init_diag   = { 0 }
};

static void _obd2_diag_refresh_twai(uint32_t alerts_hint)
{
    _ctx.init_diag.alerts |= alerts_hint;

    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
        _ctx.init_diag.twai_state = (uint32_t)st.state;
        _ctx.init_diag.tx_error_counter = (uint32_t)st.tx_error_counter;
        _ctx.init_diag.rx_error_counter = (uint32_t)st.rx_error_counter;
        _ctx.init_diag.msgs_to_tx = (uint32_t)st.msgs_to_tx;
        _ctx.init_diag.msgs_to_rx = (uint32_t)st.msgs_to_rx;
    }

    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) == ESP_OK) {
        _ctx.init_diag.alerts |= alerts;
    }
}

static bool _obd2_diag_needs_reinit(void)
{
    if (_ctx.init_diag.alerts & TWAI_ALERT_BUS_OFF) return true;
    return (_ctx.init_diag.twai_state == TWAI_STATE_BUS_OFF ||
            _ctx.init_diag.twai_state == TWAI_STATE_RECOVERING);
}

static obd2_status_t _obd2_reinit_transport_for_init(void)
{
    if (_ctx.baudrate == 0 || _ctx.tx_pin < 0 || _ctx.rx_pin < 0) {
        _ctx.init_diag.last_obd_status = OBD2_ERR_INVALID_ARG;
        return OBD2_ERR_INVALID_ARG;
    }

    OBD2_LOGW("init fallback: reinitializing ISO-TP/TWAI");
    _ctx.init_diag.reinit_performed = true;
    isotp_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));

    isotp_status_t ist = isotp_init(_ctx.baudrate, _ctx.tx_pin, _ctx.rx_pin);
    _ctx.init_diag.last_isotp_status = ist;
    if (ist != ISOTP_OK) {
        _ctx.initialized = false;
        _ctx.init_diag.last_obd_status = OBD2_ERR_ISOTP;
        _obd2_diag_refresh_twai(0);
        return OBD2_ERR_ISOTP;
    }

    _ctx.initialized = true;
    _ctx.init_diag.last_obd_status = OBD2_OK;
    _obd2_diag_refresh_twai(0);
    vTaskDelay(pdMS_TO_TICKS(150));
    return OBD2_OK;
}

static bool _obd2_pid_in_mask(const uint32_t *mask, uint8_t pid)
{
    if (mask == NULL || pid == 0x00) return false;

    uint8_t range_idx = (pid - 1) / 32;
    uint8_t bit_pos   = 31 - ((pid - 1) % 32);
    if (range_idx >= 8) return false;

    return (mask[range_idx] & (1UL << bit_pos)) != 0;
}

static uint16_t _obd2_count_pids_in_mask(const uint32_t *mask)
{
    uint16_t count = 0;
    if (mask == NULL) return 0;

    for (uint16_t pid = 0x01; pid <= 0xFF; pid++) {
        if (_obd2_pid_in_mask(mask, (uint8_t)pid)) count++;
    }
    return count;
}

static const obd2_detected_ecu_t *_obd2_find_detected_ecu(uint32_t rx_id)
{
    for (uint8_t i = 0; i < _ctx.detected_ecus.count; i++) {
        if (_ctx.detected_ecus.items[i].rx_id == rx_id) {
            return &_ctx.detected_ecus.items[i];
        }
    }
    return NULL;
}

static uint16_t _obd2_ecu_engine_score(const obd2_detected_ecu_t *ecu)
{
    if (ecu == NULL) return 0;

    uint16_t score = 0;
    if (_obd2_pid_in_mask(ecu->supported_pids, 0x0C)) score += 100; /* RPM */
    if (_obd2_pid_in_mask(ecu->supported_pids, 0x0D)) score += 80;  /* speed */
    if (_obd2_pid_in_mask(ecu->supported_pids, 0x05)) score += 60;  /* coolant */
    if (_obd2_pid_in_mask(ecu->supported_pids, 0x04)) score += 40;  /* load */
    score += _obd2_count_pids_in_mask(ecu->supported_pids);
    return score;
}

static void _obd2_bind_best_detected_ecu(void)
{
    if (_ctx.detected_ecus.count == 0) {
        _ctx.active_ecu_bound = false;
        return;
    }

    const obd2_detected_ecu_t *best = &_ctx.detected_ecus.items[0];
    uint16_t best_score = _obd2_ecu_engine_score(best);

    for (uint8_t i = 1; i < _ctx.detected_ecus.count; i++) {
        const obd2_detected_ecu_t *candidate = &_ctx.detected_ecus.items[i];
        uint16_t score = _obd2_ecu_engine_score(candidate);

        if (score > best_score ||
            (score == best_score && candidate->rx_id == ISOTP_OBD_PHYS_RESP_BASE)) {
            best = candidate;
            best_score = score;
        }
    }

    obd2_bind_active_ecu(best->rx_id);
}

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

/*
 * Konvence pro literaly v tabulce:
 *   "\xC2\xB0""C"  = UTF-8 kodovany znak "°C" (stupen Celsia)
 *   "\xC2\xB0"     = "°" samostatny stupen
 *   "\xCE\xBB"     = UTF-8 "λ" (lambda) — pro bezrozmerny ekvivalencni pomer
 *
 * Sloupec 'category' (osmy) slouzi pro UI segmentaci:
 *   TELEMETRY = continuous live data (DASH bublinky)
 *   STATUS    = bitove pole / kumulativni citace (Diag panel)
 *   CONFIG    = staticka data o vozidle (Vehicle Info na HOME)
 *   META      = bitmasky podpory ($00, $20, ...) — interni
 */

static const obd2_pid_desc_t _pid_table[] = {
    /* ---- META: Bitmasky podporovanych PIDu ---- */
    { 0x00, "PIDs supported [01-20]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_META },
    { 0x20, "PIDs supported [21-40]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_META },
    { 0x40, "PIDs supported [41-60]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_META },
    { 0x60, "PIDs supported [61-80]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_META },
    { 0x80, "PIDs supported [81-A0]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_META },
    { 0xA0, "PIDs supported [A1-C0]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_META },
    { 0xC0, "PIDs supported [C1-E0]",       "",      4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_META },

    /* ---- $01-$03: Diagnosticke stavy (STATUS) ---- */
    { 0x01, "Monitor status since DTCs cleared",    "",    4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },
    { 0x02, "Freeze DTC",                          "",    2, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },
    { 0x03, "Fuel system status",                  "",    2, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },

    /* ---- $04-$11: Zakladni telemetrie motoru (TELEMETRY) ---- */
    { 0x04, "Calculated engine load",              "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x05, "Engine coolant temperature",          "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f, OBD2_CAT_TELEMETRY },
    { 0x06, "Short term fuel trim - Bank 1",       "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x07, "Long term fuel trim - Bank 1",        "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x08, "Short term fuel trim - Bank 2",       "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x09, "Long term fuel trim - Bank 2",        "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x0A, "Fuel pressure",                       "kPa", 1, OBD2_FMT_LINEAR_1B, 3.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x0B, "Intake manifold absolute pressure",   "kPa", 1, OBD2_FMT_LINEAR_1B, 1.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x0C, "Engine RPM",                          "rpm", 2, OBD2_FMT_LINEAR_2B, 0.25f, 0, OBD2_CAT_TELEMETRY },
    { 0x0D, "Vehicle speed",                       "km/h", 1, OBD2_FMT_LINEAR_1B, 1.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x0E, "Timing advance",                      "\xC2\xB0", 1, OBD2_FMT_LINEAR_1B, 0.5f, -64.0f, OBD2_CAT_TELEMETRY },
    { 0x0F, "Intake air temperature",              "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f, OBD2_CAT_TELEMETRY },
    { 0x10, "MAF air flow rate",                   "g/s", 2, OBD2_FMT_LINEAR_2B, 0.01f, 0, OBD2_CAT_TELEMETRY },
    { 0x11, "Throttle position",                   "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $12-$13: Konfigurace O2 senzoru (STATUS) ---- */
    { 0x12, "Commanded secondary air status",      "",    1, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },
    { 0x13, "Oxygen sensors present (2 banks)",    "",    1, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },

    /* ---- $14-$1B: Konvencni O2 senzory (V + STFT) — TELEMETRY ---- */
    { 0x14, "O2 Sensor B1S1 (Voltage + STFT)",    "V",   2, OBD2_FMT_O2_CONV, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x15, "O2 Sensor B1S2 (Voltage + STFT)",    "V",   2, OBD2_FMT_O2_CONV, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x16, "O2 Sensor B1S3 (Voltage + STFT)",    "V",   2, OBD2_FMT_O2_CONV, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x17, "O2 Sensor B1S4 (Voltage + STFT)",    "V",   2, OBD2_FMT_O2_CONV, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x18, "O2 Sensor B2S1 (Voltage + STFT)",    "V",   2, OBD2_FMT_O2_CONV, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x19, "O2 Sensor B2S2 (Voltage + STFT)",    "V",   2, OBD2_FMT_O2_CONV, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x1A, "O2 Sensor B2S3 (Voltage + STFT)",    "V",   2, OBD2_FMT_O2_CONV, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x1B, "O2 Sensor B2S4 (Voltage + STFT)",    "V",   2, OBD2_FMT_O2_CONV, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $1C: OBD standard (CONFIG, statika po init) ---- */
    { 0x1C, "OBD standards this vehicle conforms to", "", 1, OBD2_FMT_ENUM, 0, 0, OBD2_CAT_CONFIG },

    /* ---- $1D-$1E: Dalsi stavy (STATUS) ---- */
    { 0x1D, "Oxygen sensors present (4 banks)",   "",    1, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },
    { 0x1E, "Auxiliary input status",              "",    1, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },

    /* ---- $1F: Doba behu motoru — TELEMETRY (kontinualne roste) ---- */
    { 0x1F, "Run time since engine start",         "s",   2, OBD2_FMT_LINEAR_2B, 1.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $21: Vzdalenost s MIL — STATUS (citac) ---- */
    { 0x21, "Distance traveled with MIL on",       "km",  2, OBD2_FMT_LINEAR_2B, 1.0f, 0, OBD2_CAT_STATUS },

    /* ---- $22-$23: Tlaky paliva — TELEMETRY ---- */
    { 0x22, "Fuel rail pressure (relative)",       "kPa", 2, OBD2_FMT_LINEAR_2B, 0.079f, 0, OBD2_CAT_TELEMETRY },
    { 0x23, "Fuel rail gauge pressure",            "kPa", 2, OBD2_FMT_LINEAR_2B, 10.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $24-$2B: Sirokopasmove O2 senzory (lambda + V) — TELEMETRY ---- */
    { 0x24, "O2 Sensor B1S1 (lambda + V)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x25, "O2 Sensor B1S2 (lambda + V)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x26, "O2 Sensor B1S3 (lambda + V)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x27, "O2 Sensor B1S4 (lambda + V)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x28, "O2 Sensor B2S1 (lambda + V)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x29, "O2 Sensor B2S2 (lambda + V)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x2A, "O2 Sensor B2S3 (lambda + V)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x2B, "O2 Sensor B2S4 (lambda + V)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_V, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $2C-$2E: EGR a odparovani — TELEMETRY ---- */
    { 0x2C, "Commanded EGR",                       "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x2D, "EGR error",                           "%",   1, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x2E, "Commanded evaporative purge",         "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $2F: Uroven paliva — TELEMETRY ---- */
    { 0x2F, "Fuel tank level input",               "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $30-$31: Citace — STATUS ---- */
    { 0x30, "Warm-ups since codes cleared",        "",    1, OBD2_FMT_LINEAR_1B, 1.0f, 0, OBD2_CAT_STATUS },
    { 0x31, "Distance traveled since codes cleared", "km", 2, OBD2_FMT_LINEAR_2B, 1.0f, 0, OBD2_CAT_STATUS },

    /* ---- $32-$33: Evap & barometricky tlak — TELEMETRY ---- */
    { 0x32, "Evap system vapor pressure",          "Pa",  2, OBD2_FMT_SIGNED_2B, 0.25f, 0, OBD2_CAT_TELEMETRY },
    { 0x33, "Absolute barometric pressure",        "kPa", 1, OBD2_FMT_LINEAR_1B, 1.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $34-$3B: Sirokopasmove O2 (lambda + I mA) — TELEMETRY ---- */
    { 0x34, "O2 Sensor B1S1 (lambda + I)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x35, "O2 Sensor B1S2 (lambda + I)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x36, "O2 Sensor B1S3 (lambda + I)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x37, "O2 Sensor B1S4 (lambda + I)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x38, "O2 Sensor B2S1 (lambda + I)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x39, "O2 Sensor B2S2 (lambda + I)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x3A, "O2 Sensor B2S3 (lambda + I)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x3B, "O2 Sensor B2S4 (lambda + I)",        "\xCE\xBB", 4, OBD2_FMT_O2_WIDE_EQ_I, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $3C-$3F: Teplota katalyzatoru — TELEMETRY ---- */
    { 0x3C, "Catalyst temperature B1S1",           "\xC2\xB0""C", 2, OBD2_FMT_LINEAR_2B, 0.1f, -40.0f, OBD2_CAT_TELEMETRY },
    { 0x3D, "Catalyst temperature B2S1",           "\xC2\xB0""C", 2, OBD2_FMT_LINEAR_2B, 0.1f, -40.0f, OBD2_CAT_TELEMETRY },
    { 0x3E, "Catalyst temperature B1S2",           "\xC2\xB0""C", 2, OBD2_FMT_LINEAR_2B, 0.1f, -40.0f, OBD2_CAT_TELEMETRY },
    { 0x3F, "Catalyst temperature B2S2",           "\xC2\xB0""C", 2, OBD2_FMT_LINEAR_2B, 0.1f, -40.0f, OBD2_CAT_TELEMETRY },

    /* ---- $41: Stav monitoru aktualniho cyklu — STATUS ---- */
    { 0x41, "Monitor status this drive cycle",     "",    4, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },

    /* ---- $42-$44: Napeti, zatez, lambda — TELEMETRY ---- */
    { 0x42, "Control module voltage",              "V",   2, OBD2_FMT_LINEAR_2B, 0.001f, 0, OBD2_CAT_TELEMETRY },
    { 0x43, "Absolute load value",                 "%",   2, OBD2_FMT_LINEAR_2B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x44, "Commanded equivalence ratio",         "\xCE\xBB", 2, OBD2_FMT_LINEAR_2B, 2.0f/65536.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $45-$4C: Polohy klapky, pedalu — TELEMETRY ---- */
    { 0x45, "Relative throttle position",          "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x46, "Ambient air temperature",             "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f, OBD2_CAT_TELEMETRY },
    { 0x47, "Absolute throttle position B",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x48, "Absolute throttle position C",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x49, "Accelerator pedal position D",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x4A, "Accelerator pedal position E",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x4B, "Accelerator pedal position F",        "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x4C, "Commanded throttle actuator",         "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $4D-$4E: Kumulativni casy — STATUS ---- */
    { 0x4D, "Time run with MIL on",               "min", 2, OBD2_FMT_LINEAR_2B, 1.0f, 0, OBD2_CAT_STATUS },
    { 0x4E, "Time since trouble codes cleared",    "min", 2, OBD2_FMT_LINEAR_2B, 1.0f, 0, OBD2_CAT_STATUS },

    /* ---- $4F-$51: Konfigurace vozidla — CONFIG ---- */
    { 0x4F, "Max EQ ratio / O2V / O2I / MAP",     "",    4, OBD2_FMT_CONFIG, 0, 0, OBD2_CAT_CONFIG },
    { 0x50, "Max MAF air flow rate",               "g/s", 4, OBD2_FMT_LINEAR_1B, 10.0f, 0, OBD2_CAT_CONFIG },
    { 0x51, "Fuel type",                           "",    1, OBD2_FMT_ENUM, 0, 0, OBD2_CAT_CONFIG },

    /* ---- $52-$5A: Dalsi telemetrie — TELEMETRY ---- */
    { 0x52, "Ethanol fuel percentage",             "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x53, "Absolute evap system vapor pressure", "kPa", 2, OBD2_FMT_LINEAR_2B, 0.005f, 0, OBD2_CAT_TELEMETRY },
    { 0x54, "Evap system vapor pressure (signed)", "Pa",  2, OBD2_FMT_SIGNED_2B, 1.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x55, "Short term secondary O2 trim B1/B3", "%",   2, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x56, "Long term secondary O2 trim B1/B3",  "%",   2, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x57, "Short term secondary O2 trim B2/B4", "%",   2, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x58, "Long term secondary O2 trim B2/B4",  "%",   2, OBD2_FMT_SIGNED_OFFSET_1B, 100.0f/128.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x59, "Fuel rail absolute pressure",         "kPa", 2, OBD2_FMT_LINEAR_2B, 10.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x5A, "Relative accelerator pedal position", "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $5B-$5E: Hybrid + diesel — TELEMETRY ---- */
    { 0x5B, "Hybrid battery pack remaining life",  "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x5C, "Engine oil temperature",              "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f, OBD2_CAT_TELEMETRY },
    { 0x5D, "Fuel injection timing",               "\xC2\xB0", 2, OBD2_FMT_LINEAR_2B, 1.0f/128.0f, -210.0f, OBD2_CAT_TELEMETRY },
    { 0x5E, "Engine fuel rate",                    "L/h", 2, OBD2_FMT_LINEAR_2B, 0.05f, 0, OBD2_CAT_TELEMETRY },

    /* =====================================================================
     * Rozsireni dle Wikipedia OBD-II PIDs (overeno proti normam SAE J1979 +
     * ISO 15031-5 dodatky). Implementovany jen jednoduche skalary; komplexni
     * smisene bit/value formaty (PIDy $66, $69-$76, $7B, $86, $8B, $94...)
     * jsou v tabulce s formatem RAW — frontend zobrazi surove bajty.
     * ===================================================================== */

    /* ---- $61-$63: Kroutici moment motoru ($61, $62 = signed offset 125) — TELEMETRY ---- */
    { 0x61, "Driver demand engine torque (%)",     "%",   1, OBD2_FMT_LINEAR_1B, 1.0f, -125.0f, OBD2_CAT_TELEMETRY },
    { 0x62, "Actual engine torque (%)",            "%",   1, OBD2_FMT_LINEAR_1B, 1.0f, -125.0f, OBD2_CAT_TELEMETRY },
    { 0x63, "Engine reference torque",             "N\xC2\xB7m", 2, OBD2_FMT_LINEAR_2B, 1.0f, 0, OBD2_CAT_TELEMETRY },

    /* ---- $64: Engine percent torque data (5-bod) — RAW ---- */
    { 0x64, "Engine percent torque data (5-point)", "",   5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $65: Auxiliary input/output supported — STATUS (bit pole) ---- */
    { 0x65, "Auxiliary input/output supported",    "",    2, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },

    /* ---- $66-$70: Komplexni diesel PIDy se smisenymi formaty — RAW ---- */
    { 0x66, "Mass air flow sensor (dual)",         "",    5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x67, "Engine coolant temperature (dual)",   "\xC2\xB0""C", 3, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x68, "Intake air temperature sensor (dual)", "\xC2\xB0""C", 3, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x69, "Actual EGR / Commanded EGR / Error",  "",    7, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x6A, "Diesel intake air flow control",      "",    5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x6B, "EGR temperature",                     "\xC2\xB0""C", 5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x6C, "Throttle actuator control",           "",    5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x6D, "Fuel pressure control system",        "",   11, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x6E, "Injection pressure control system",   "",    9, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x6F, "Turbocharger compressor inlet pressure", "kPa", 3, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x70, "Boost pressure control",              "",   10, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $71-$76: Variable geometry turbo & wastegate — RAW ---- */
    { 0x71, "Variable Geometry turbo control",     "",    6, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x72, "Wastegate control",                   "",    5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x73, "Exhaust pressure",                    "Pa",  5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x74, "Turbocharger RPM",                    "rpm", 5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x75, "Turbocharger temperature",            "\xC2\xB0""C", 7, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x76, "Turbocharger temperature (alt)",      "\xC2\xB0""C", 7, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $77: Charge air cooler temperature — TELEMETRY (jednoduchy) ---- */
    { 0x77, "Charge air cooler temperature",       "\xC2\xB0""C", 5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $78-$79: EGT 4-senzor (Bank 1, Bank 2) — TELEMETRY (multi-temp) ---- */
    { 0x78, "Exhaust gas temperature Bank 1 (4 sensors)", "\xC2\xB0""C", 9, OBD2_FMT_TEMP_4S, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x79, "Exhaust gas temperature Bank 2 (4 sensors)", "\xC2\xB0""C", 9, OBD2_FMT_TEMP_4S, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $7A-$7C: DPF — RAW (smisene formaty) ---- */
    { 0x7A, "DPF differential pressure",           "Pa",  7, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x7B, "Diesel particulate filter status",    "",    7, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x7C, "DPF temperature",                     "\xC2\xB0""C", 9, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $7D-$7E: NOx & PM NTE control — STATUS (bit) ---- */
    { 0x7D, "NOx NTE control area status",         "",    1, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },
    { 0x7E, "PM NTE control area status",          "",    1, OBD2_FMT_BIT_ENCODED, 0, 0, OBD2_CAT_STATUS },

    /* ---- $7F: Engine run time celkovy (4-byte LINEAR_4B) — STATUS (citac) ---- */
    { 0x7F, "Engine run time (total)",             "s",  13, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },

    /* ---- $81-$8B: Diesel aftertreatment, run times — RAW (komplexni) ---- */
    { 0x81, "AECD run time #1-#5",                 "s",  41, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x82, "AECD run time #6-#10",                "s",  41, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },

    /* ---- $83: NOx senzor (multi-sensor) — TELEMETRY ---- */
    { 0x83, "NOx sensor concentration",            "ppm", 9, OBD2_FMT_NOX_4S, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $84: Manifold surface temperature — TELEMETRY ---- */
    { 0x84, "Manifold surface temperature",        "\xC2\xB0""C", 1, OBD2_FMT_LINEAR_1B, 1.0f, -40.0f, OBD2_CAT_TELEMETRY },

    /* ---- $85-$94: Komplexni diesel — RAW ---- */
    { 0x85, "NOx reagent system",                  "",   10, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x86, "Particulate matter sensor",           "",    5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x87, "Intake manifold absolute pressure (extended)", "kPa", 5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x88, "SCR induction system",                "",   13, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x89, "AECD run time #11-#15",               "s",  41, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x8A, "AECD run time #16-#20",               "s",  41, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x8B, "Diesel aftertreatment",               "",    7, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x8C, "O2 sensor (wide range)",              "",   17, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $8D-$8E: Throttle G + Friction torque (jednoduche) — TELEMETRY ---- */
    { 0x8D, "Throttle position G",                 "%",   1, OBD2_FMT_LINEAR_1B, 100.0f/255.0f, 0, OBD2_CAT_TELEMETRY },
    { 0x8E, "Engine friction torque (%)",          "%",   1, OBD2_FMT_LINEAR_1B, 1.0f, -125.0f, OBD2_CAT_TELEMETRY },

    /* ---- $8F-$94: Komplexni multi-sensor / WWH-OBD — RAW ---- */
    { 0x8F, "PM sensor Bank 1 & 2",                "",    7, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x90, "WWH-OBD vehicle system info",         "h",   3, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x91, "WWH-OBD vehicle system info (alt)",   "h",   5, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x92, "Fuel system control",                 "",    2, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x93, "WWH-OBD counter support",             "h",   3, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
    { 0x94, "NOx warning/inducement system",       "",   12, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },

    /* ---- $98-$99: EGT alternativni multi-sensor — TELEMETRY ---- */
    { 0x98, "EGT sensor (alt)",                    "\xC2\xB0""C", 9, OBD2_FMT_TEMP_4S, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x99, "EGT sensor (alt 2)",                  "\xC2\xB0""C", 9, OBD2_FMT_TEMP_4S, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $9A-$9F: Hybrid/EV + dalsi diesel — RAW ---- */
    { 0x9A, "Hybrid/EV battery voltage",           "V",   6, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x9B, "Diesel exhaust fluid sensor",         "%",   4, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x9C, "O2 sensor data (extended)",           "",   17, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x9D, "Engine fuel rate (4-byte)",           "g/s", 4, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0x9E, "Engine exhaust flow rate",            "kg/h", 2, OBD2_FMT_LINEAR_2B, 0.2f, 0, OBD2_CAT_TELEMETRY },
    { 0x9F, "Fuel system percentage use",          "%",   9, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },

    /* ---- $A1-$A6: Pokrocile diesel + odometer — TELEMETRY/STATUS ---- */
    { 0xA1, "NOx sensor corrected data",           "ppm", 9, OBD2_FMT_NOX_4S, 0, 0, OBD2_CAT_TELEMETRY },
    { 0xA2, "Cylinder fuel rate",                  "mg/stroke", 2, OBD2_FMT_LINEAR_2B, 0.03125f, 0, OBD2_CAT_TELEMETRY },
    { 0xA3, "Evap system vapor pressure (wide)",   "Pa",  9, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0xA4, "Transmission actual gear",            "",    4, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0xA5, "Diesel exhaust fluid dosing",         "%",   4, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0xA6, "Odometer",                            "km",  4, OBD2_FMT_LINEAR_4B, 0.1f, 0, OBD2_CAT_STATUS },
    { 0xA7, "NOx sensor concentration (sensors 3-4)", "ppm", 4, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0xA8, "NOx corrected concentration (3-4)",   "ppm", 4, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0xA9, "ABS disable switch state",            "",    4, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },

    /* ---- $C3-$C4: Drive condition data — RAW ---- */
    { 0xC3, "Fuel level input A/B",                "%",   2, OBD2_FMT_RAW, 0, 0, OBD2_CAT_TELEMETRY },
    { 0xC4, "Particulate control diagnostic",      "",    8, OBD2_FMT_RAW, 0, 0, OBD2_CAT_STATUS },
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
 *   1) Pokud uz transport bezi, vrati OBD2_OK bez restartu TWAI driveru.
 *   2) Jinak inicializuje ISO-TP vrstvu (ta inicializuje TWAI driver na ESP32).
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

    memset(&_ctx.init_diag, 0, sizeof(_ctx.init_diag));
    _ctx.baudrate = baudrate;
    _ctx.tx_pin = tx_pin;
    _ctx.rx_pin = rx_pin;
    _ctx.init_diag.last_obd_status = OBD2_ERR_NOT_INITIALIZED;

    if (_ctx.initialized) {
        OBD2_LOGI("init: transport already initialized");
        _ctx.init_diag.last_obd_status = OBD2_OK;
        _obd2_diag_refresh_twai(0);
        return OBD2_OK;
    }

    isotp_status_t ist = isotp_init(baudrate, tx_pin, rx_pin);
    _ctx.init_diag.last_isotp_status = ist;
    if (ist != ISOTP_OK) {
        OBD2_LOGE("init: isotp_init failed: %s", isotp_status_str(ist));
        _ctx.init_diag.last_obd_status = OBD2_ERR_ISOTP;
        _obd2_diag_refresh_twai(0);
        return OBD2_ERR_ISOTP;
    }

    _ctx.tx_id       = ISOTP_OBD_PHYS_REQ_BASE;
    _ctx.rx_id       = ISOTP_OBD_PHYS_RESP_BASE;
    _ctx.timeout_ms  = OBD2_DEFAULT_TIMEOUT_MS;
    _ctx.pids_queried = false;
    _ctx.active_ecu_bound = false;
    memset(_ctx.supported_pids, 0, sizeof(_ctx.supported_pids));
    memset(&_ctx.last_nrc, 0, sizeof(_ctx.last_nrc));
    memset(&_ctx.detected_ecus, 0, sizeof(_ctx.detected_ecus));
    _ctx.initialized = true;
    _ctx.init_diag.last_obd_status = OBD2_OK;
    _obd2_diag_refresh_twai(0);

    OBD2_LOGI("init: OK (tx=0x%03X rx=0x%03X timeout=%lums)",
              _ctx.tx_id, _ctx.rx_id, (unsigned long)_ctx.timeout_ms);
    return OBD2_OK;
}

bool obd2_is_transport_initialized(void)
{
    return _ctx.initialized;
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
        _ctx.active_ecu_bound = false;
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
    _ctx.active_ecu_bound = true;
}

obd2_status_t obd2_bind_active_ecu(uint32_t rx_id)
{
    if (rx_id < ISOTP_OBD_PHYS_RESP_BASE ||
        rx_id >= (ISOTP_OBD_PHYS_RESP_BASE + ISOTP_MAX_ECU_RESPONSES)) {
        OBD2_LOGE("bind_active_ecu: invalid rx_id=0x%03X", (unsigned)rx_id);
        return OBD2_ERR_INVALID_ARG;
    }

    if (_ctx.detected_ecus.count > 0 && _obd2_find_detected_ecu(rx_id) == NULL) {
        OBD2_LOGE("bind_active_ecu: rx_id=0x%03X was not discovered", (unsigned)rx_id);
        return OBD2_ERR_INVALID_ARG;
    }

    uint32_t tx_id = rx_id - 8;
    obd2_set_ecu_address(tx_id, rx_id);
    OBD2_LOGI("bind_active_ecu: active ECU tx=0x%03X rx=0x%03X",
              (unsigned)_ctx.tx_id, (unsigned)_ctx.rx_id);
    return OBD2_OK;
}

bool obd2_get_active_ecu(uint32_t *tx_id, uint32_t *rx_id)
{
    if (tx_id) *tx_id = _ctx.tx_id;
    if (rx_id) *rx_id = _ctx.rx_id;
    return _ctx.active_ecu_bound;
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
obd2_status_t _obd2_request_multi(const uint8_t *req, uint8_t req_len,
                                   isotp_result_t *result,
                                   uint32_t timeout_ms)
{
    if (!_ctx.initialized) {
        OBD2_LOGE("request_multi: not initialized");
        return OBD2_ERR_NOT_INITIALIZED;
    }
    if (req == NULL || result == NULL || req_len == 0) {
        OBD2_LOGE("request_multi: invalid argument");
        return OBD2_ERR_INVALID_ARG;
    }

    OBD2_LOGD("request_multi: SID=0x%02X len=%u timeout=%u",
              req[0], req_len, timeout_ms);

    memset(result, 0, sizeof(isotp_result_t));
    _ctx.init_diag.last_tx_id = ISOTP_OBD_FUNC_REQ_ID;
    _ctx.init_diag.last_rx_id = 0;
    isotp_status_t ist = isotp_transaction_broadcast(req, req_len, result, timeout_ms);
    _ctx.init_diag.last_isotp_status = ist;

    if (ist != ISOTP_OK) {
        OBD2_LOGE("request_multi: broadcast isotp error: %s",
                  isotp_status_str(ist));
        _ctx.init_diag.last_obd_status = (ist == ISOTP_ERR_TIMEOUT) ? OBD2_ERR_TIMEOUT
                                                                    : OBD2_ERR_ISOTP;
        _obd2_diag_refresh_twai(0);
        return _ctx.init_diag.last_obd_status;
    }

    if (result->count == 0) {
        OBD2_LOGW("request_multi: got 0 responses");
        _ctx.init_diag.last_obd_status = OBD2_ERR_NO_DATA;
        _obd2_diag_refresh_twai(0);
        return OBD2_ERR_NO_DATA;
    }

    _ctx.init_diag.last_rx_id = result->responses[0].rx_id;
    OBD2_LOGD("request_multi: OK, got %u responses", result->count);
    _ctx.init_diag.last_obd_status = OBD2_OK;
    _obd2_diag_refresh_twai(0);
    return OBD2_OK;
}

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
    _ctx.init_diag.last_tx_id = use_broadcast ? ISOTP_OBD_FUNC_REQ_ID : _ctx.tx_id;
    _ctx.init_diag.last_rx_id = use_broadcast ? 0 : _ctx.rx_id;

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
         * nezabira misto na stacku. _obd2_request_multi() si ji znovu
         * vynuluje memsetem pred pouzitim.
         *
         * Omezeni: funkce neni reentrantni (static buffer je sdileny).
         * To je v poradku -- OBD pozadavky se zpracovavaji sekvencne
         * z jednoho tasku a CAN sbernice je polo-duplexni.
         */
        static isotp_result_t bcast_result;
        obd2_status_t st = _obd2_request_multi(req, req_len, &bcast_result, _ctx.timeout_ms);
        if (st != OBD2_OK) return st;

        /* Pouzijeme prvni validni odpoved, ktera odpovida ocekavanemu SIDu a neni NRC.
         * ECU, ktera na broadcast posle negativni odpoved (napr. NRC 0x11 =
         * serviceNotSupported) nebo chybne echo SIDu, ignoruje se a hledá se
         * dal -- v realne siti casto odpovedi jen nektere jednotky. */
        bool found = false;
        bool nrc_received = false;
        for (uint8_t i = 0; i < bcast_result.count; i++) {
            isotp_response_t *r = &bcast_result.responses[i];
            if (r->valid && r->len >= 1) {
                /* Kontrola NRC (format: [0x7F, req_sid, nrc_code]) */
                if (r->len >= 3 && r->data[0] == OBD2_SID_NEGATIVE_RESPONSE) {
                    OBD2_LOGD("request: ECU 0x%03X sent NRC 0x%02X, skipping",
                              r->rx_id, r->data[2]);
                    _ctx.last_nrc.request_sid = r->data[1];
                    _ctx.last_nrc.nrc = r->data[2];
                    _ctx.init_diag.last_rx_id = r->rx_id;
                    nrc_received = true;
                    continue;
                }
                /* Kontrola SID -- musi byt req[0] + 0x40 */
                if (r->data[0] != expected_resp_sid) {
                    OBD2_LOGD("request: ECU 0x%03X sent unexpected SID 0x%02X, skipping",
                              r->rx_id, r->data[0]);
                    continue;
                }

                /* Kontrola PID (pokud je soucasti pozadavku) */
                if (req_len >= 2 && r->len >= 2 && r->data[1] != req[1]) {
                    OBD2_LOGD("request: ECU 0x%03X sent unexpected PID 0x%02X (expected 0x%02X), skipping",
                              r->rx_id, r->data[1], req[1]);
                    continue;
                }

                uint16_t copy_len = r->len;
                if (copy_len > *resp_len) copy_len = *resp_len;
                memcpy(resp, r->data, copy_len);
                *resp_len = copy_len;
                _ctx.init_diag.last_rx_id = r->rx_id;
                found = true;
                OBD2_LOGD("request: using response from ECU 0x%03X (%u bytes)",
                          r->rx_id, copy_len);
                break;
            }
        }
        if (!found) {
            OBD2_LOGW("request: no valid broadcast response found after SID filter");
            if (nrc_received) {
                return OBD2_ERR_NEGATIVE_RESP;
            }
            return OBD2_ERR_NO_DATA;
        }
    } else {
        /* Fyzicke adresovani: primo na nakonfigurovanou ECU */
        ist = isotp_transaction(_ctx.tx_id, _ctx.rx_id,
                                req, req_len,
                                resp, resp_len,
                                _ctx.timeout_ms);
        _ctx.init_diag.last_isotp_status = ist;
        if (ist != ISOTP_OK) {
            OBD2_LOGE("request: isotp error: %s", isotp_status_str(ist));
            _ctx.init_diag.last_obd_status = (ist == ISOTP_ERR_TIMEOUT) ? OBD2_ERR_TIMEOUT
                                                                        : OBD2_ERR_ISOTP;
            _obd2_diag_refresh_twai(0);
            return _ctx.init_diag.last_obd_status;
        }
    }

    /* Kontrola negativni odpovedi (SID 0x7F = negative response) */
    if (*resp_len >= 3 && resp[0] == OBD2_SID_NEGATIVE_RESPONSE) {
        _ctx.last_nrc.request_sid = resp[1];
        _ctx.last_nrc.nrc = resp[2];
        OBD2_LOGW("request: negative response for SID 0x%02X, NRC=0x%02X (%s)",
                  resp[1], resp[2], obd2_nrc_str(resp[2]));
        _ctx.init_diag.last_obd_status = OBD2_ERR_NEGATIVE_RESP;
        _obd2_diag_refresh_twai(0);
        return OBD2_ERR_NEGATIVE_RESP;
    }

    /* Validace SID odpovedi -- musi byt pozadovany SID + 0x40 */
    if (*resp_len < 1 || resp[0] != expected_resp_sid) {
        OBD2_LOGE("request: unexpected response SID=0x%02X (expected 0x%02X)",
                  resp[0], expected_resp_sid);
        _ctx.init_diag.last_obd_status = OBD2_ERR_RESPONSE_MALFORMED;
        _obd2_diag_refresh_twai(0);
        return OBD2_ERR_RESPONSE_MALFORMED;
    }

    OBD2_LOGD("request: OK, resp_len=%u resp_SID=0x%02X",
              *resp_len, resp[0]);
    _ctx.init_diag.last_obd_status = OBD2_OK;
    _obd2_diag_refresh_twai(0);
    return OBD2_OK;
}

static obd2_status_t _obd2_query_pid00_with_fallback(const uint8_t *req,
                                                      isotp_result_t *result,
                                                      bool allow_physical_fallback)
{
    const uint32_t broadcast_backoff_ms[3] = { 0, 200, 500 };
    obd2_status_t last_st = OBD2_ERR_TIMEOUT;
    bool reinit_done = false;

    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        if (broadcast_backoff_ms[attempt] > 0) {
            OBD2_LOGW("query_supported_pids: PID $00 broadcast retry %u/3 (delay=%lums)",
                      (unsigned)(attempt + 1),
                      (unsigned long)broadcast_backoff_ms[attempt]);
            vTaskDelay(pdMS_TO_TICKS(broadcast_backoff_ms[attempt]));
        }

        _ctx.init_diag.init_attempts++;
        _ctx.init_diag.last_tx_id = ISOTP_OBD_FUNC_REQ_ID;
        _ctx.init_diag.last_rx_id = 0;
        last_st = _obd2_request_multi(req, 2, result, _ctx.timeout_ms);
        _ctx.init_diag.last_obd_status = last_st;

        /* Pokud komunikace prosla, okamzite vyhodnotime a pripadne vratime uspech.
         * TIM ZABRANIME zahozeni uspesne odpovedi kuli transientnim alertum z pozadi! */
        if (last_st == OBD2_OK && result->count > 0) {
            bool positive_pid00 = false;
            for (uint8_t i = 0; i < result->count; i++) {
                const isotp_response_t *r = &result->responses[i];
                if (r->valid && r->len >= 6 &&
                    r->data[0] == (OBD2_SID_CURRENT_DATA + OBD2_SID_RESPONSE_OFFSET) &&
                    r->data[1] == 0x00) {
                    positive_pid00 = true;
                    break;
                }
            }
            if (positive_pid00) {
                _obd2_diag_refresh_twai(0); // Jen vycistime alerty pro cisty stit
                return OBD2_OK;
            }
            
            /* Pokud jsme sice dostali data (napr. NRC), ale nebyl to nas pozitivni PID00 */
            last_st = OBD2_ERR_NO_DATA;
            _ctx.init_diag.last_obd_status = last_st;
        }

        /* Sem se dostaneme jen pokud komunikace opravdu SELHALA (timeout, error), 
         * nebo se auto odmitlo bavit spravnym formatem. Ted ma smysl resit alerty a reinit. */
        _obd2_diag_refresh_twai(0);

        if (!reinit_done && _obd2_diag_needs_reinit()) {
            obd2_status_t rst = _obd2_reinit_transport_for_init();
            reinit_done = true;
            if (rst != OBD2_OK) return rst;
        }
    }

    if (!allow_physical_fallback) {
        return last_st;
    }

    OBD2_LOGW("query_supported_pids: PID $00 broadcast failed, trying physical 0x%03X -> 0x%03X",
              ISOTP_OBD_PHYS_REQ_BASE, ISOTP_OBD_PHYS_RESP_BASE);

    _ctx.tx_id = ISOTP_OBD_PHYS_REQ_BASE;
    _ctx.rx_id = ISOTP_OBD_PHYS_RESP_BASE;
    _ctx.init_diag.used_physical_fallback = true;

    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        if (attempt > 0) {
            vTaskDelay(pdMS_TO_TICKS(250));
        }

        uint8_t resp[ISOTP_MAX_PAYLOAD];
        uint16_t resp_len = sizeof(resp);
        _ctx.init_diag.init_attempts++;
        _ctx.init_diag.last_tx_id = _ctx.tx_id;
        _ctx.init_diag.last_rx_id = _ctx.rx_id;
        last_st = _obd2_request(req, 2, resp, &resp_len, false);
        _ctx.init_diag.last_obd_status = last_st;

        if (last_st == OBD2_OK) {
            _obd2_diag_refresh_twai(0);
            memset(result, 0, sizeof(*result));
            result->status = ISOTP_OK;
            result->count = 1;
            result->responses[0].rx_id = _ctx.rx_id;
            result->responses[0].valid = true;
            result->responses[0].len = resp_len;
            if (result->responses[0].len > ISOTP_MAX_PAYLOAD) {
                result->responses[0].len = ISOTP_MAX_PAYLOAD;
            }
            memcpy(result->responses[0].data, resp, result->responses[0].len);
            return OBD2_OK;
        }

        _obd2_diag_refresh_twai(0);
        if (!reinit_done && _obd2_diag_needs_reinit()) {
            obd2_status_t rst = _obd2_reinit_transport_for_init();
            reinit_done = true;
            if (rst != OBD2_OK) return rst;
        }
    }

    return last_st;
}

obd2_status_t obd2_probe_pid00(isotp_result_t *result)
{
    if (!_ctx.initialized) {
        OBD2_LOGE("probe_pid00: not initialized");
        return OBD2_ERR_NOT_INITIALIZED;
    }
    if (result == NULL) {
        OBD2_LOGE("probe_pid00: NULL result pointer");
        return OBD2_ERR_INVALID_ARG;
    }

    memset(&_ctx.init_diag, 0, sizeof(_ctx.init_diag));
    _obd2_diag_refresh_twai(0);

    uint8_t req[2] = { OBD2_SID_CURRENT_DATA, 0x00 };
    return _obd2_query_pid00_with_fallback(req, result, false);
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
 * UNION pres vice ECU:
 *   Vozidla maji typicky vice ECU (motor, prevodovka, BMS u hybridu...).
 *   Kazda ECU deklaruje svuj VLASTNI seznam podporovanych PIDu — typicky
 *   se prekryvaji jen castecne. Drive jsme brali POUZE prvni odpoved,
 *   coz vedlo k nekonzistenci (napr. Peugeot 3008 HYbrid4 nekdy poslal
 *   prvni odpoved z BMS s pouhymi 2 PIDy ["0x01","0x1C"], jindy z motoru
 *   s plnym seznamem). Reseni: bitwise OR vsech validnich odpovedi —
 *   PID je "podporovan" kdyz ho podporuje alespon JEDNA ECU.
 *
 * RETRY pro PID $00:
 *   Hybridni vozidla maji agresivni sleep mody — engine ECU se po key-on
 *   probouzi se zpozdenim. Prvni broadcast muze prijit zatimco ECU jeste
 *   spi → TIMEOUT. Resime az 3 pokusy s exponencialnim back-off
 *   (0/200/400 ms), nez vratime chybu uzivateli. Dalsi rozsahy ($20+)
 *   retry nemaji — pokud uz $00 prislo, ECU jsou vzhuru.
 *
 * Pravidla iterace:
 *   - Prvni rozsah ($00) MUSI uspet, jinak se vraci chyba (po retry).
 *   - Pokud dalsi rozsah selze, je to normalni (ECU nepodporuje vyssi PIDy).
 *   - Pokud bit D0 v UNION bitmasce neni nastaven, zadna ECU dalsi
 *     rozsah neumi — konec iterace.
 *   - Vysledky se ukladaji do _ctx.supported_pids[0..7].
 *
 * Po uspesnem volani je mozne pouzit obd2_is_pid_supported() pro dotaz
 * na konkretni PID.
 *
 * @return OBD2_OK pri uspechu, chybovy kod pokud PID $00 selze i po retry
 */
obd2_status_t obd2_query_supported_pids(void)
{
    OBD2_LOGI("query_supported_pids: starting");

    if (!_ctx.initialized) {
        OBD2_LOGE("query_supported_pids: not initialized");
        return OBD2_ERR_NOT_INITIALIZED;
    }

    memset(_ctx.supported_pids, 0, sizeof(_ctx.supported_pids));
    memset(&_ctx.detected_ecus, 0, sizeof(_ctx.detected_ecus));
    _ctx.pids_queried = false;
    _ctx.active_ecu_bound = false;
    _ctx.tx_id = ISOTP_OBD_PHYS_REQ_BASE;
    _ctx.rx_id = ISOTP_OBD_PHYS_RESP_BASE;
    memset(&_ctx.init_diag, 0, sizeof(_ctx.init_diag));
    _obd2_diag_refresh_twai(0);

    /*
     * Static buffer pro broadcast vysledky — zabira ~2.1 KB. Stejny duvod
     * jako v _obd2_request: alokace na zasobniku FreeRTOS tasku (typicky
     * 4-8 KB) by zpusobila preteceni. Funkce neni reentrantni, ale to je
     * v poradku — OBD pozadavky se zpracovavaji sekvencne z jednoho tasku.
     */
    static isotp_result_t bcast_result;

    const uint8_t expected_resp_sid = OBD2_SID_CURRENT_DATA + OBD2_SID_RESPONSE_OFFSET;

    for (uint8_t range_pid = 0x00; range_pid <= 0xE0; range_pid += 0x20) {
        uint8_t req[2] = { OBD2_SID_CURRENT_DATA, range_pid };
        obd2_status_t st = OBD2_ERR_TIMEOUT;

        /* Retry mechanizmus — pouze pro PID $00 (uvodni dotaz). Hybridni
         * auta se obcas po key-on probouzeji se zpozdenim, takze prvni
         * broadcast muze trefit uspaneho engine ECU. Pro $20+ uz ECU
         * urcite vzhuru jsou (jinak by neuspelo $00).
         *
         * 5 pokusu s rostoucim backoff — pokryva i pripady kdy CAN bus
         * potrebuje cas na stabilizaci po predchozim selhani/reinit.
         * Pri selhani functional broadcastu nasleduje physical fallback
         * 0x7E0 -> 0x7E8 a pripadne kratky reinit TWAI/ISO-TP. */
        if (range_pid == 0x00) {
            st = _obd2_query_pid00_with_fallback(req, &bcast_result, true);
        } else {
            OBD2_LOGD("query_supported_pids: requesting PID 0x%02X", range_pid);
            st = _obd2_request_multi(req, 2, &bcast_result, _ctx.timeout_ms);
        }

        if (st != OBD2_OK) {
            if (range_pid == 0x00) {
                OBD2_LOGE("query_supported_pids: PID $00 failed after %u attempts: %s",
                          (unsigned)_ctx.init_diag.init_attempts,
                          obd2_status_str(st));
                return st;
            }
            /* Selhani dalsich rozsahu je normalni — ECU je nepodporuji */
            OBD2_LOGD("query_supported_pids: range 0x%02X not available", range_pid);
            break;
        }

        /*
         * UNION pres vsechny validni odpovedi: bitwise OR bitmask.
         * - Ignorujeme NRC odpovedi (data[0] == 0x7F).
         * - Ignorujeme odpovedi se spatnym SID nebo neshodnym echo PID.
         * - Pocitame, kolik ECU prispelo, pro informacni log.
         */
        uint8_t idx = range_pid / 0x20;  /* 0..7 */
        bool any_more_ranges = false;
        uint8_t valid_resp_count = 0;

        for (uint8_t i = 0; i < bcast_result.count; i++) {
            isotp_response_t *r = &bcast_result.responses[i];
            if (!r->valid || r->len < 6) continue;

            /* NRC ignorujeme — ECU rekla, ze tento rozsah nepodporuje */
            if (r->data[0] == OBD2_SID_NEGATIVE_RESPONSE) {
                OBD2_LOGD("query_supported_pids: ECU 0x%03X NRC for range 0x%02X",
                          r->rx_id, range_pid);
                continue;
            }

            /* Validace: [0x41, range_pid, A, B, C, D] */
            if (r->data[0] != expected_resp_sid) continue;
            if (r->data[1] != range_pid) continue;

            uint32_t bitmask = ((uint32_t)r->data[2] << 24) |
                               ((uint32_t)r->data[3] << 16) |
                               ((uint32_t)r->data[4] << 8)  |
                               ((uint32_t)r->data[5]);

            _ctx.supported_pids[idx] |= bitmask;
            valid_resp_count++;

            /* Per-ECU tracking: najdi nebo pridej ECU do seznamu */
            {
                obd2_detected_ecu_list_t *el = &_ctx.detected_ecus;
                uint8_t ei;
                for (ei = 0; ei < el->count; ei++) {
                    if (el->items[ei].rx_id == r->rx_id) break;
                }
                if (ei == el->count && el->count < ISOTP_MAX_ECU_RESPONSES) {
                    /* Nova ECU — inicializace */
                    memset(&el->items[ei], 0, sizeof(obd2_detected_ecu_t));
                    el->items[ei].rx_id = r->rx_id;
                    el->count++;
                }
                if (ei < ISOTP_MAX_ECU_RESPONSES) {
                    el->items[ei].supported_pids[idx] |= bitmask;
                }
            }

            OBD2_LOGD("query_supported_pids: range 0x%02X ECU 0x%03X bitmask=0x%08lX",
                      range_pid, r->rx_id, (unsigned long)bitmask);

            if (bitmask & 0x00000001) any_more_ranges = true;
        }

        if (valid_resp_count == 0) {
            if (range_pid == 0x00) {
                OBD2_LOGE("query_supported_pids: no valid response for PID $00");
                _ctx.init_diag.last_obd_status = OBD2_ERR_NO_DATA;
                return OBD2_ERR_NO_DATA;
            }
            OBD2_LOGD("query_supported_pids: range 0x%02X — no valid response", range_pid);
            break;
        }

        OBD2_LOGI("query_supported_pids: range 0x%02X union from %u ECU(s) = 0x%08lX",
                  range_pid, (unsigned)valid_resp_count,
                  (unsigned long)_ctx.supported_pids[idx]);

        /* Kontrola "next range" bitu na UNION bitmasce — pokracujeme,
         * pokud alespon jedna ECU avizovala dalsi rozsah */
        if (!any_more_ranges) {
            OBD2_LOGD("query_supported_pids: no more ranges after 0x%02X",
                      range_pid);
            break;
        }
    }

    _ctx.pids_queried = true;
    _obd2_bind_best_detected_ecu();

    uint16_t union_count = _obd2_count_pids_in_mask(_ctx.supported_pids);
    uint16_t active_count = union_count;
    const obd2_detected_ecu_t *active = _ctx.active_ecu_bound
                                      ? _obd2_find_detected_ecu(_ctx.rx_id)
                                      : NULL;
    if (active != NULL) {
        active_count = _obd2_count_pids_in_mask(active->supported_pids);
    }
    OBD2_LOGI("query_supported_pids: done, %u active PIDs, %u union PIDs, %u ECU detected",
              (unsigned)active_count, (unsigned)union_count,
              (unsigned)_ctx.detected_ecus.count);

    /* Informacni log per-ECU */
    for (uint8_t e = 0; e < _ctx.detected_ecus.count; e++) {
        obd2_detected_ecu_t *ecu = &_ctx.detected_ecus.items[e];
        uint16_t ecu_pid_count = 0;
        for (uint16_t pid = 0x01; pid <= 0xFF; pid++) {
            uint8_t ri = (pid - 1) / 32;
            uint8_t bp = 31 - ((pid - 1) % 32);
            if (ri < 8 && (ecu->supported_pids[ri] & (1UL << bp))) ecu_pid_count++;
        }
        OBD2_LOGI("  ECU 0x%03X: %u PIDs", (unsigned)ecu->rx_id, (unsigned)ecu_pid_count);
    }

    return OBD2_OK;
}

/**
 * @brief Vraci seznam ECU detekovanych pri poslednim broadcast PID $00.
 */
const obd2_detected_ecu_list_t *obd2_get_detected_ecus(void)
{
    return &_ctx.detected_ecus;
}

const obd2_init_diag_t *obd2_get_init_diag(void)
{
    return &_ctx.init_diag;
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

    if (_ctx.active_ecu_bound) {
        const obd2_detected_ecu_t *ecu = _obd2_find_detected_ecu(_ctx.rx_id);
        if (ecu != NULL) {
            return _obd2_pid_in_mask(ecu->supported_pids, pid);
        }
    }

    return obd2_is_pid_supported_union(pid);
}

bool obd2_is_pid_supported_union(uint8_t pid)
{
    if (!_ctx.pids_queried || pid == 0x00) {
        return false;
    }
    return _obd2_pid_in_mask(_ctx.supported_pids, pid);
}

bool obd2_is_pid_supported_by_ecu(const obd2_detected_ecu_t *ecu, uint8_t pid)
{
    if (ecu == NULL || pid == 0x00) return false;
    return _obd2_pid_in_mask(ecu->supported_pids, pid);
}
