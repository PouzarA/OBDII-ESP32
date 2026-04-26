/**
 * @file ws_handler.cpp
 * @brief Infrastruktura a dispatch WebSocket zprav handleru
 *
 * Tento soubor byl rozdelen na dve casti kvuli prehlednosti a udrzovatelnosti:
 *
 *   ws_handler.cpp   (tento soubor) — infrastruktura, dispatch, pomocne funkce
 *   ws_commands.cpp  — vsechny command handlery (_ws_cmd_* funkce)
 *
 * Tento soubor obsahuje:
 *   1. Interni stav modulu (fronty, OBD inicializace, stream konfigurace)
 *   2. Parsovani prichoziho JSON prikazu (_ws_parse_cmd)
 *   3. Pomocne funkce pro tvorbu JSON odpovedi (_ws_set_error, _ws_serialize)
 *   4. Verejne API: ws_handler_init, ws_handle_incoming, ws_process_obd_command
 *   5. Streaming API: ws_is_streaming, ws_get_stream_interval, ws_process_stream_tick
 *
 * Vsechny _ws_cmd_* funkce jsou definovane v ws_commands.cpp a deklarovane
 * nize jako forward deklarace. Tyto funkce nejsou static, protoze jsou
 * volane z dispatch switche v ws_process_obd_command() v tomto souboru.
 *
 * Pomocne funkce _ws_set_error a _ws_serialize rovnez nejsou static,
 * protoze je pouzivaji command handlery v ws_commands.cpp.
 *
 * @see ws_handler.h pro dokumentaci verejneho API
 * @see ws_commands.cpp pro implementaci jednotlivych OBD prikazu
 */

#include "ws_handler.h"

extern "C" {
    #include "obd2.h"
}

#include <ArduinoJson.h>
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_system.h"

#include "config.h"

static void _ws_format_pid_raw_hex(const obd2_pid_decoded_t &decoded,
                                   char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) return;
    size_t off = 0;
    int written = snprintf(out, out_len, "0x");
    if (written < 0) {
        out[0] = '\0';
        return;
    }
    off = (size_t)written;

    for (uint8_t i = 0; i < decoded.raw_data_len; i++) {
        if (off + 3 > out_len) break;
        written = snprintf(out + off, out_len - off, "%02X", decoded.raw_data[i]);
        if (written < 0) break;
        off += (size_t)written;
    }
}

static const char *_ws_stream_mode_str(ws_stream_mode_t mode)
{
    return (mode == WS_STREAM_MODE_INSPECTOR) ? "inspector" : "dash";
}

static bool _ws_diag_pid_enabled(uint8_t pid, const uint8_t *diag_pids,
                                 uint8_t diag_pid_count)
{
    for (uint8_t i = 0; i < diag_pid_count; i++) {
        if (diag_pids[i] == pid) return true;
    }
    return false;
}

static void _ws_add_stream_pid_diag(JsonObject obj,
                                    const obd2_pid_decoded_t *decoded,
                                    obd2_status_t st)
{
    char tx_hex[8], rx_hex[8];
    const obd2_init_diag_t *diag = obd2_get_init_diag();
    snprintf(tx_hex, sizeof(tx_hex), "0x%03lX", (unsigned long)diag->last_tx_id);
    snprintf(rx_hex, sizeof(rx_hex), "0x%03lX", (unsigned long)diag->last_rx_id);

    obj["obd_status"] = obd2_status_str(st);
    obj["isotp_status"] = isotp_status_str(diag->last_isotp_status);
    obj["tx_id"] = tx_hex;
    obj["rx_id"] = rx_hex;

    if (decoded != NULL) {
        char raw_hex[2 + OBD2_PID_MAX_DATA_BYTES * 2 + 1];
        _ws_format_pid_raw_hex(*decoded, raw_hex, sizeof(raw_hex));
        obj["raw"] = raw_hex;
        obj["raw_len"] = decoded->raw_data_len;
    } else {
        obj["raw"] = "0x";
        obj["raw_len"] = 0;
    }
}

/* ========================================================================= */
/*  Interni stav modulu                                                      */
/* ========================================================================= */

/**
 * Reference na FreeRTOS fronty (predane z ws_handler_init).
 *
 * _req_queue:  smer WebSocket callback (Core 0) → OBD task (Core 1)
 * _resp_queue: smer OBD task (Core 1) → dispatch task (Core 0)
 *
 * Obe fronty jsou thread-safe diky FreeRTOS implementaci.
 * NULL znamena, ze modul jeste nebyl inicializovan volanim ws_handler_init().
 */
static QueueHandle_t _req_queue  = NULL;
static QueueHandle_t _resp_queue = NULL;

/**
 * Stav OBD inicializace.
 *
 * Nastaven na true po uspesnem CMD_INIT (obd2_init + query_supported_pids).
 * Volatile protoze se zapisuje z OBD tasku (Core 1) a cte z WebSocket
 * callbacku (Core 0) — napr. v _ws_cmd_ping pro info o stavu.
 *
 * Pouziva se jako guard ve vsech command handlerech — pokud je false,
 * vrati se chyba OBD2_ERR_NOT_INITIALIZED.
 *
 * Pozn.: Neni static, protoze k nemu pristupuji command handlery
 * v ws_commands.cpp pres extern deklaraci.
 */
volatile bool _obd_initialized = false;

/**
 * Konfigurace streamu — ktere PIDy se cyklicky ctou a jak casto.
 *
 * Pristup z OBD tasku (Core 1) pro cteni, z WebSocket callbacku (Core 0)
 * pro zapis. volatile pro viditelnost mezi jadry. Atomicita zapisu
 * neni kriticka — v nejhorsim se precte castecna konfigurace
 * a dalsi tick uz bude konzistentni.
 *
 * Priklad konfigurace po CMD_START_STREAM s {"pids":[12,13,5], "interval_ms":100}:
 *   active = true, pids = {12, 13, 5, ...}, pid_count = 3, interval_ms = 100
 *
 * Vychozi interval 200ms = 5 Hz obnovovaci frekvence, coz je rozumny
 * kompromis mezi latenci a zatizenim CAN sbernice.
 *
 * Pozn.: Neni static, protoze k nemu pristupuji command handlery
 * v ws_commands.cpp pres extern deklaraci.
 */
/* Typ ws_stream_cfg_t je definovan v ws_handler.h */
volatile ws_stream_cfg_t _stream_cfg = { false, WS_STREAM_MODE_DASH, {0}, {0}, 0, 0, 200 };

/* ========================================================================= */
/*  Pomocne funkce — mapovani cmd retezce na enum                            */
/* ========================================================================= */

/**
 * @brief Mapovani JSON "cmd" retezce na ws_cmd_t enum.
 *
 * Pouziva jednoduche strcmp() porovnani. Pocet prikazu je maly
 * (< 15), takze linearni prohledavani je rychlejsi nez hash tabulka
 * pro tak maly vstup.
 *
 * Priklad: "get_pid" → CMD_GET_PID, "init" → CMD_INIT
 *
 * Hranicni pripady:
 *   - NULL ukazatel → CMD_UNKNOWN (ochrana pred chybejicim "cmd" polem v JSON)
 *   - Prazdny retezec → CMD_UNKNOWN (zadne strcmp neuspeje)
 *   - Neznamy prikaz napr. "reboot" → CMD_UNKNOWN
 *   - Velka/mala pismena: rozlisuje se ("PING" != "ping" → CMD_UNKNOWN)
 *
 * @param cmd_str  Retezec z JSON pole "cmd" (muze byt NULL)
 * @return Odpovidajici ws_cmd_t, nebo CMD_UNKNOWN pri nerozpoznanem prikazu
 */
static ws_cmd_t _ws_parse_cmd(const char *cmd_str)
{
    if (cmd_str == NULL) return CMD_UNKNOWN;

    if (strcmp(cmd_str, "ping") == 0)               return CMD_PING;
    if (strcmp(cmd_str, "transport_init") == 0)     return CMD_TRANSPORT_INIT;
    if (strcmp(cmd_str, "pid00_probe") == 0)        return CMD_PID00_PROBE;
    if (strcmp(cmd_str, "init") == 0)                return CMD_INIT;
    if (strcmp(cmd_str, "get_pid") == 0)             return CMD_GET_PID;
    if (strcmp(cmd_str, "get_pids") == 0)            return CMD_GET_PIDS;
    if (strcmp(cmd_str, "get_supported_pids") == 0)  return CMD_GET_SUPPORTED_PIDS;
    if (strcmp(cmd_str, "get_dtc") == 0)             return CMD_GET_DTC;
    if (strcmp(cmd_str, "get_pending_dtc") == 0)     return CMD_GET_PENDING_DTC;
    if (strcmp(cmd_str, "get_mode06_monitor") == 0)  return CMD_GET_MODE06_MONITOR;
    if (strcmp(cmd_str, "get_permanent_dtc") == 0)   return CMD_GET_PERMANENT_DTC;
    if (strcmp(cmd_str, "clear_dtc") == 0)           return CMD_CLEAR_DTC;
    if (strcmp(cmd_str, "get_vin") == 0)             return CMD_GET_VIN;
    if (strcmp(cmd_str, "get_monitor_status") == 0)  return CMD_GET_MONITOR_STATUS;
    if (strcmp(cmd_str, "get_freeze_frame") == 0)   return CMD_GET_FREEZE_FRAME;
    if (strcmp(cmd_str, "get_ecu_name") == 0)       return CMD_GET_ECU_NAME;
    if (strcmp(cmd_str, "get_cal_id") == 0)         return CMD_GET_CAL_ID;
    if (strcmp(cmd_str, "get_supported_infotypes") == 0) return CMD_GET_SUPPORTED_INFOTYPES;
    if (strcmp(cmd_str, "get_mode09_info") == 0)    return CMD_GET_MODE09_INFO;
    if (strcmp(cmd_str, "get_cvn") == 0)            return CMD_GET_CVN;
    if (strcmp(cmd_str, "get_ipt") == 0)            return CMD_GET_IPT;
    if (strcmp(cmd_str, "get_monitor_status_all") == 0) return CMD_GET_MONITOR_STATUS_ALL;
    if (strcmp(cmd_str, "discover_ecus") == 0)      return CMD_DISCOVER_ECUS;
    if (strcmp(cmd_str, "start_stream") == 0)       return CMD_START_STREAM;
    if (strcmp(cmd_str, "stop_stream") == 0)         return CMD_STOP_STREAM;
    if (strcmp(cmd_str, "manual_query") == 0)        return CMD_MANUAL_QUERY;

    return CMD_UNKNOWN;
}

/* ========================================================================= */
/*  Pomocne funkce — tvorba JSON odpovedi                                    */
/* ========================================================================= */

/**
 * @brief Vlozi do JSON dokumentu spolecne chybove pole.
 *
 * Pouziva se pri OBD chybach (timeout, negative response, atd.).
 * Vysledny format: {"status":"error", "error":"TIMEOUT", "message":"popis"}
 *
 * Pri negativni odpovedi z ECU (OBD2_ERR_NEGATIVE_RESP) se přidá navic
 * NRC detail — numericky kod a jeho textovy popis. Priklad:
 *   {"status":"error", "error":"NEGATIVE_RESP", "nrc_code":0x12,
 *    "nrc_name":"subFunctionNotSupported"}
 *
 * Tato funkce NENI static — pouzivaji ji command handlery v ws_commands.cpp.
 *
 * @param doc      ArduinoJson dokument (musi byt jiz vytvoreny)
 * @param obd_st   OBD2 status kod (pro mapovani na retezec pres obd2_status_str)
 * @param message  Lidsky citelny popis chyby (muze byt NULL — pak se neprida)
 */
void _ws_set_error(JsonDocument &doc, obd2_status_t obd_st,
                          const char *message)
{
    doc["status"]  = "error";
    doc["error"]   = obd2_status_str(obd_st);
    if (message) {
        doc["message"] = message;
    }

    /* Pri negativni odpovedi z ECU se přidá NRC detail (kod + popis) */
    if (obd_st == OBD2_ERR_NEGATIVE_RESP) {
        obd2_nrc_info_t nrc = obd2_get_last_nrc();
        doc["nrc_code"] = nrc.nrc;
        doc["nrc_name"] = obd2_nrc_str(nrc.nrc);
    }
}

/**
 * @brief Serializuje JSON dokument do response zpravy.
 *
 * Kontroluje preteceni bufferu — pokud je JSON delsi nez
 * WS_RESPONSE_JSON_MAX (1024 B), orizne ho a vlozi chybovou zpravu.
 * K preteceni by nemelo dojit pri normalnim provozu, ale muze nastat
 * napr. pri get_supported_pids pokud ECU hlasi neobvykle mnoho PIDu.
 *
 * Tato funkce NENI static — pouzivaji ji command handlery v ws_commands.cpp.
 *
 * @param doc   ArduinoJson dokument k serializaci
 * @param resp  Cilova response zprava (json buffer bude naplnen)
 */
void _ws_serialize(JsonDocument &doc, obd_response_msg_t *resp)
{
    size_t len = serializeJson(doc, resp->json, WS_RESPONSE_JSON_MAX);
    if (len >= WS_RESPONSE_JSON_MAX) {
        /* JSON se nevejde — toto by nemelo nastat pri normalnim provozu */
        snprintf(resp->json, WS_RESPONSE_JSON_MAX,
                 "{\"status\":\"error\",\"error\":\"JSON_OVERFLOW\"}");
    }
}

/* ========================================================================= */
/*  Forward deklarace command handleru (definovane v ws_commands.cpp)         */
/* ========================================================================= */
/*
 * Tyto funkce jsou implementovane v ws_commands.cpp. Nejsou static,
 * protoze jsou volane z dispatch switche v ws_process_obd_command() nize.
 *
 * Kazda funkce zpracovava jeden typ OBD prikazu — prijme request,
 * zavola prislusne obd2_* funkce a naplni response JSON odpovedi.
 *
 * Konvence: vsechny maji stejnou signaturu (req, resp) pro jednoduchy dispatch.
 */

void _ws_cmd_ping(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_transport_init(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_pid00_probe(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_init(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_pid(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_pids(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_supported_pids(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_dtc(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_mode06_monitor(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_clear_dtc(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_vin(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_monitor_status(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_freeze_frame(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_ecu_name(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_cal_id(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_supported_infotypes(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_mode09_info(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_cvn(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_ipt(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_monitor_status_all(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_discover_ecus(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_start_stream(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_stop_stream(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_manual_query(const obd_request_msg_t *req, obd_response_msg_t *resp);

/* ========================================================================= */
/*  Verejne API                                                              */
/* ========================================================================= */

/**
 * @brief Inicializace ws_handler modulu — ulozeni referenci na FreeRTOS fronty.
 *
 * Musi byt volano pred jakymkoliv dalsim volanim ws_handler funkci,
 * typicky v setup() po vytvoreni front pomoci xQueueCreate().
 *
 * Priklad pouziti:
 *   QueueHandle_t req_q  = xQueueCreate(10, sizeof(obd_request_msg_t));
 *   QueueHandle_t resp_q = xQueueCreate(10, sizeof(obd_response_msg_t));
 *   ws_handler_init(req_q, resp_q);
 *
 * Resetuje _obd_initialized na false — dulezite pri sw resetu ESP32,
 * kdy se setup() vola znovu ale globalni promenne mohly zustat.
 */
void ws_handler_init(QueueHandle_t req_queue, QueueHandle_t resp_queue)
{
    _req_queue  = req_queue;
    _resp_queue = resp_queue;
    _obd_initialized = false;
}

/**
 * @brief Vraci stav OBD inicializace.
 *
 * true = CMD_INIT probehlo uspesne, OBD vrstva je pripravena.
 * false = jeste nebylo volano init, nebo init selhalo.
 */
bool ws_is_obd_initialized(void)
{
    return _obd_initialized;
}

/**
 * @brief Zpracovani prichoziho JSON od WebSocket klienta.
 *
 * Volano z WebSocket callbacku na Core 0. Tato funkce:
 *   1. Parsuje JSON payload pomoci ArduinoJson
 *   2. Extrahuje "cmd" pole a namapuje na ws_cmd_t enum
 *   3. Extrahuje parametry specificke pro dany prikaz (pid, pids[], interval_ms)
 *   4. Zaradi request zpravu do fronty pro OBD task (Core 1)
 *
 * Vyjimky zpracovane primo (bez OBD tasku):
 *   - Nevalidni JSON → chybova odpoved primo do resp fronty
 *   - Neznamy prikaz → chybova odpoved primo do resp fronty
 *   - CMD_PING → odpoved primo (nepotrebuje CAN komunikaci)
 *   - Plna request fronta → chybova odpoved primo do resp fronty
 *
 * Hranicni pripady:
 *   - _req_queue nebo _resp_queue je NULL → tichy navrat (modul neinicializovan)
 *   - JSON delsi nez WS_RESPONSE_JSON_MAX → ArduinoJson truncation, ale prikaz
 *     bude zpracovan pokud obsahuje validni "cmd" pole
 *   - Chybejici parametry (napr. "pid" u get_pid) → obd2 funkce vrati chybu
 *
 * Pozn.: static obd_response_msg_t se pouziva misto lokalni promenne
 * protoze async TCP callback ma omezeny stack (~4-8 KB) a struktura
 * ma ~1028 B. Static je bezpecny protoze WS callback bezi na jednom tasku.
 *
 * @param client_id  ID WebSocket klienta (pro smerovani odpovedi zpet)
 * @param payload    JSON string od klienta (null-terminated)
 * @param length     Delka payloadu v bytech (bez null terminatoru)
 */
void ws_handle_incoming(uint32_t client_id, const char *payload, size_t length)
{
    if (_req_queue == NULL || _resp_queue == NULL) {
        return;  /* Modul neni inicializovany */
    }

    /* Parsovani JSON */
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);

    if (err) {
        /*
         * Nevalidni JSON — odesle se chyba primo do response fronty
         * (nemusí se zatezovat OBD task).
         * Používá se static aby se 1028B struktura neukladala na stack
         * async TCP callbacku (omezeny stack ~4-8 KB).
         */
        static obd_response_msg_t resp;
        resp.client_id = client_id;
        snprintf(resp.json, WS_RESPONSE_JSON_MAX,
                 "{\"status\":\"error\",\"error\":\"JSON_PARSE\","
                 "\"message\":\"%s\"}", err.c_str());
        xQueueSend(_resp_queue, &resp, pdMS_TO_TICKS(100));
        return;
    }

    const char *cmd_str = doc["cmd"];
    ws_cmd_t cmd = _ws_parse_cmd(cmd_str);

    if (cmd == CMD_UNKNOWN) {
        static obd_response_msg_t resp;
        resp.client_id = client_id;
        snprintf(resp.json, WS_RESPONSE_JSON_MAX,
                 "{\"status\":\"error\",\"error\":\"UNKNOWN_CMD\","
                 "\"message\":\"Neznamy prikaz: %s\"}",
                 cmd_str ? cmd_str : "(null)");
        xQueueSend(_resp_queue, &resp, pdMS_TO_TICKS(100));
        return;
    }

    /*
     * CMD_PING nepotrebuje OBD task — zpracuje se primo zde (Core 0)
     * a vkládá se do response fronty. Vsechny ostatni prikazy jdou
     * do request fronty pro OBD task (Core 1).
     */
    if (cmd == CMD_PING) {
        obd_request_msg_t req_msg;
        memset(&req_msg, 0, sizeof(req_msg));
        req_msg.cmd = CMD_PING;
        req_msg.client_id = client_id;
        req_msg.hb = doc["hb"] | false; // Pridano: vytazeni hb priznaku pro ping
        
        static obd_response_msg_t resp;
        resp.client_id = client_id;
        _ws_cmd_ping(&req_msg, &resp);
        xQueueSend(_resp_queue, &resp, pdMS_TO_TICKS(100));
        return;
    }

    /* Sestaveni request zpravy pro OBD task */
    obd_request_msg_t req_msg;
    memset(&req_msg, 0, sizeof(req_msg));
    req_msg.cmd = cmd;
    req_msg.client_id = client_id;
    req_msg.hb = doc["hb"] | false;

    /* Extrahovani parametru podle typu prikazu */
    switch (cmd) {
    case CMD_GET_PID:
        req_msg.pid = (uint8_t)doc["pid"].as<int>();
        break;

    case CMD_GET_PIDS: {
        JsonArray arr = doc["pids"].as<JsonArray>();
        req_msg.pid_count = 0;
        for (JsonVariant v : arr) {
            if (req_msg.pid_count >= WS_MAX_PIDS_PER_REQUEST) break;
            req_msg.pids[req_msg.pid_count++] = (uint8_t)v.as<int>();
        }
        break;
    }

    case CMD_GET_FREEZE_FRAME:
        req_msg.pid = (uint8_t)doc["pid"].as<int>();
        break;

    case CMD_GET_MODE06_MONITOR: {
        int mid = doc["mid"] | -1;
        if (mid < 0) mid = doc["pid"] | 0;
        req_msg.pid = (uint8_t)mid;
        break;
    }

    case CMD_GET_MODE09_INFO: {
        int infotype = doc["infotype"] | -1;
        if (infotype < 0) infotype = doc["pid"] | OBD2_INFOTYPE_SUPPORTED;
        req_msg.pid = (uint8_t)infotype;
        break;
    }

    case CMD_GET_IPT: {
        int infotype = doc["infotype"] | -1;
        if (infotype < 0) infotype = doc["pid"] | OBD2_INFOTYPE_IPT;
        req_msg.pid = (uint8_t)infotype;
        if (req_msg.pid != OBD2_INFOTYPE_IPT &&
            req_msg.pid != OBD2_INFOTYPE_IPT_COMPRESSION) {
            req_msg.pid = OBD2_INFOTYPE_IPT;
        }
        break;
    }

    case CMD_START_STREAM: {
        /* Parsovani PIDu a intervalu pro stream */
        const char *mode_str = doc["mode"] | "dash";
        req_msg.stream_mode = (strcmp(mode_str, "inspector") == 0)
                              ? WS_STREAM_MODE_INSPECTOR
                              : WS_STREAM_MODE_DASH;

        uint8_t max_stream_pids = (req_msg.stream_mode == WS_STREAM_MODE_INSPECTOR)
                                ? WS_MAX_DIAG_PIDS
                                : WS_MAX_PIDS_PER_REQUEST;
        JsonArray arr = doc["pids"].as<JsonArray>();
        req_msg.pid_count = 0;
        for (JsonVariant v : arr) {
            if (req_msg.pid_count >= max_stream_pids) break;
            req_msg.pids[req_msg.pid_count++] = (uint8_t)v.as<int>();
        }
        JsonArray diag_arr = doc["diag_pids"].as<JsonArray>();
        req_msg.diag_pid_count = 0;
        for (JsonVariant v : diag_arr) {
            if (req_msg.diag_pid_count >= WS_MAX_DIAG_PIDS) break;
            req_msg.diag_pids[req_msg.diag_pid_count++] = (uint8_t)v.as<int>();
        }
        if (req_msg.stream_mode == WS_STREAM_MODE_INSPECTOR &&
            req_msg.diag_pid_count == 0) {
            for (uint8_t i = 0; i < req_msg.pid_count && i < WS_MAX_DIAG_PIDS; i++) {
                req_msg.diag_pids[req_msg.diag_pid_count++] = req_msg.pids[i];
            }
        }
        req_msg.interval_ms = doc["interval_ms"] | 200;  /* vychozi 200ms */
        break;
    }

    case CMD_CLEAR_DTC: {
        /*
         * Destruktivni prikaz — vyzaduje autentizacni token.
         * Token se kopiruje z JSON do req_msg.token (max. WS_AUTH_TOKEN_MAX-1
         * znaku + '\0'). Vlastni overeni probiha az v handleru
         * _ws_cmd_clear_dtc(), aby byla logika autentizace u prikazu,
         * ne v univerzalnim parseru.
         */
        const char *tok = doc["token"] | "";
        strncpy(req_msg.token, tok, WS_AUTH_TOKEN_MAX - 1);
        req_msg.token[WS_AUTH_TOKEN_MAX - 1] = '\0';
        break;
    }

    case CMD_MANUAL_QUERY: {
        /* Parsovani Service ID a PIDu pro manualni dotaz */
        req_msg.service = (uint8_t)(doc["service"] | 1);
        req_msg.pid = (uint8_t)(doc["pid"] | 0);
        break;
    }

    default:
        /* Ostatni prikazy nemaji parametry */
        break;
    }

    /*
     * Zarazeni do request fronty. Timeout 100ms — pokud je fronta plna
     * (OBD task nestihá), odešle se chybova odpoved klientovi.
     */
    if (xQueueSend(_req_queue, &req_msg, pdMS_TO_TICKS(100)) != pdTRUE) {
        static obd_response_msg_t resp;
        resp.client_id = client_id;
        snprintf(resp.json, WS_RESPONSE_JSON_MAX,
                 "{\"cmd\":\"%s\",\"status\":\"error\","
                 "\"error\":\"QUEUE_FULL\","
                 "\"message\":\"OBD task je zaneprazdnen\"}",
                 cmd_str ? cmd_str : "");
        xQueueSend(_resp_queue, &resp, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Dispatch OBD prikazu na prislusny command handler.
 *
 * Volano v OBD tasku na Core 1. Prijme request zpravu z fronty
 * a zavola odpovidajici _ws_cmd_* funkci (definovanou v ws_commands.cpp).
 *
 * Kazdy handler naplni resp->json serializovanym JSON retezcem.
 * Pri nerozpoznanem prikazu (CMD_UNKNOWN nebo chybejici case) vlozi
 * generickou chybovou zpravu UNHANDLED_CMD.
 *
 * Hranicni pripady:
 *   - req nebo resp je NULL → tichy navrat (ochrana)
 *   - Neocekavany cmd enum (napr. po pridani noveho prikazu bez handleru)
 *     → UNHANDLED_CMD chyba
 *
 * @param req   Vstupni request zprava z fronty (nesmi byt NULL)
 * @param resp  Vystupni response zprava — json buffer bude naplnen (nesmi byt NULL)
 */
void ws_process_obd_command(const obd_request_msg_t *req,
                            obd_response_msg_t *resp)
{
    if (req == NULL || resp == NULL) return;

    memset(resp, 0, sizeof(obd_response_msg_t));
    resp->client_id = req->client_id;

    /*
     * Dispatch na prislusny command handler.
     * Kazdy handler naplni resp->json serializovanym JSONem.
     */
    switch (req->cmd) {
    case CMD_PING:
        _ws_cmd_ping(req, resp);
        break;
    case CMD_TRANSPORT_INIT:
        _ws_cmd_transport_init(req, resp);
        break;
    case CMD_PID00_PROBE:
        _ws_cmd_pid00_probe(req, resp);
        break;
    case CMD_INIT:
        _ws_cmd_init(req, resp);
        break;
    case CMD_GET_PID:
        _ws_cmd_get_pid(req, resp);
        break;
    case CMD_GET_PIDS:
        _ws_cmd_get_pids(req, resp);
        break;
    case CMD_GET_SUPPORTED_PIDS:
        _ws_cmd_get_supported_pids(req, resp);
        break;
    case CMD_GET_DTC:
    case CMD_GET_PENDING_DTC:
    case CMD_GET_PERMANENT_DTC:
        _ws_cmd_get_dtc(req, resp);
        break;
    case CMD_GET_MODE06_MONITOR:
        _ws_cmd_get_mode06_monitor(req, resp);
        break;
    case CMD_CLEAR_DTC:
        _ws_cmd_clear_dtc(req, resp);
        break;
    case CMD_GET_VIN:
        _ws_cmd_get_vin(req, resp);
        break;
    case CMD_GET_MONITOR_STATUS:
        _ws_cmd_get_monitor_status(req, resp);
        break;
    case CMD_GET_FREEZE_FRAME:
        _ws_cmd_get_freeze_frame(req, resp);
        break;
    case CMD_GET_ECU_NAME:
        _ws_cmd_get_ecu_name(req, resp);
        break;
    case CMD_GET_CAL_ID:
        _ws_cmd_get_cal_id(req, resp);
        break;
    case CMD_GET_SUPPORTED_INFOTYPES:
        _ws_cmd_get_supported_infotypes(req, resp);
        break;
    case CMD_GET_MODE09_INFO:
        _ws_cmd_get_mode09_info(req, resp);
        break;
    case CMD_GET_CVN:
        _ws_cmd_get_cvn(req, resp);
        break;
    case CMD_GET_IPT:
        _ws_cmd_get_ipt(req, resp);
        break;
    case CMD_GET_MONITOR_STATUS_ALL:
        _ws_cmd_get_monitor_status_all(req, resp);
        break;
    case CMD_DISCOVER_ECUS:
        _ws_cmd_discover_ecus(req, resp);
        break;
    case CMD_START_STREAM:
        _ws_cmd_start_stream(req, resp);
        break;
    case CMD_STOP_STREAM:
        _ws_cmd_stop_stream(req, resp);
        break;
    case CMD_MANUAL_QUERY:
        _ws_cmd_manual_query(req, resp);
        break;
    default:
        snprintf(resp->json, WS_RESPONSE_JSON_MAX,
                 "{\"status\":\"error\",\"error\":\"UNHANDLED_CMD\"}");
        break;
    }
}

/* ========================================================================= */
/*  Streaming API                                                            */
/* ========================================================================= */

/**
 * @brief Informace zda je aktivni streaming PIDu.
 *
 * Pouziva se v OBD tasku pro rozhodnuti mezi blokujicim
 * cekanim na prikaz (xQueueReceive s portMAX_DELAY) vs.
 * aktivnim ctenim streamu (kratky timeout + ws_process_stream_tick).
 *
 * @return true pokud stream bezi, false jinak
 */
bool ws_is_streaming(void)
{
    return _stream_cfg.active;
}

/**
 * @brief Vrati nastaveny interval streamu v milisekundach.
 *
 * OBD task pouziva tuto hodnotu jako delay mezi stream ticky.
 * Pokud stream neni aktivni, vraci 0.
 *
 * Priklad: pri intervalu 100ms se PIDy ctou 10x za sekundu.
 * Minimalni povoleny interval je 50ms (omezeno v _ws_cmd_start_stream).
 *
 * @return Interval v ms, nebo 0 pokud stream neni aktivni
 */
uint16_t ws_get_stream_interval(void)
{
    return _stream_cfg.active ? _stream_cfg.interval_ms : 0;
}

/**
 * @brief Provede jeden cyklus streamu — precte vsechny nakonfigurovane PIDy.
 *
 * Volano z OBD tasku (Core 1) v aktivnim streaming rezimu.
 * Pro kazdy PID v _stream_cfg.pids vola obd2_get_pid() a vysledek
 * prida do kompaktniho JSON formatu.
 *
 * Kompaktni stream format — klice jsou cisla PIDu jako string,
 * hodnoty jsou floaty. Minimalizuje velikost JSON pro rychly prenos
 * pres WebSocket. Klient si nazvy PIDu namapuje z init/supported_pids odpovedi.
 *
 * Format: {"cmd":"stream","d":{"12":875.25,"13":45,"5":87},"ts":12345,"n":3}
 *
 * Hranicni pripady:
 *   - resp je NULL → tichy navrat
 *   - Stream neni aktivni → resp->json[0] = '\0' (prazdny retezec)
 *   - Vsechny PIDy selzou (napr. ECU neodpovida) → n=0, d={}, ts=...
 *   - Nektery PID selze → preskoci se, klient pozna chybejici klice
 *
 * @param resp  Vystupni response zprava (client_id=0 pro broadcast vsem klientum)
 */
void ws_process_stream_tick(obd_response_msg_t *resp)
{
    if (resp == NULL || !_stream_cfg.active) {
        if (resp) resp->json[0] = '\0';
        return;
    }

    memset(resp, 0, sizeof(obd_response_msg_t));
    resp->client_id = 0;  /* Broadcast vsem klientum */

    /*
     * Kompaktni stream format — klice jsou cisla PIDu jako string,
     * hodnoty jsou floaty. Minimalizuje velikost JSON pro rychly prenos.
     * Klient si nazvy PIDu namapuje z init/supported_pids odpovedi.
     *
     * Format: {"cmd":"stream","d":{"12":875.25,"13":45,"5":87},"ts":12345}
     */
    JsonDocument doc;
    doc["cmd"] = "stream";
    ws_stream_mode_t mode = _stream_cfg.mode;
    uint8_t pid_count = _stream_cfg.pid_count;
    if (pid_count > WS_MAX_PIDS_PER_REQUEST) pid_count = WS_MAX_PIDS_PER_REQUEST;

    uint8_t pids[WS_MAX_PIDS_PER_REQUEST];
    for (uint8_t i = 0; i < pid_count; i++) {
        pids[i] = _stream_cfg.pids[i];
    }

    uint8_t diag_pid_count = _stream_cfg.diag_pid_count;
    if (diag_pid_count > WS_MAX_DIAG_PIDS) diag_pid_count = WS_MAX_DIAG_PIDS;
    uint8_t diag_pids[WS_MAX_DIAG_PIDS];
    for (uint8_t i = 0; i < diag_pid_count; i++) {
        diag_pids[i] = _stream_cfg.diag_pids[i];
    }

    doc["mode"] = _ws_stream_mode_str(mode);
    JsonObject data = doc["d"].to<JsonObject>();
    JsonObject diag_data;
    if (mode == WS_STREAM_MODE_INSPECTOR) {
        diag_data = doc["diag"].to<JsonObject>();
    }

    uint8_t ok_count = 0;
    for (uint8_t i = 0; i < pid_count; i++) {
        uint8_t pid = pids[i];
        bool want_diag = (mode == WS_STREAM_MODE_INSPECTOR) &&
                         _ws_diag_pid_enabled(pid, diag_pids, diag_pid_count);

        /* Klic je PID cislo jako decimalni string (napr. "12" pro 0x0C).
         * Frontend si format prevadi pres pidToInt(). */
        char key[4];
        snprintf(key, sizeof(key), "%u", pid);

        const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(pid);

        /*
         * RAW PIDy (komplexni diesel PIDy bez dekoderu) — cteme pouze raw
         * bajty jednim CAN requestem. Predtim se volalo obd2_get_pid() +
         * obd2_get_pid_raw() = 2 CAN requesty na jeden PID, coz
         * zdvojnasovalo CAN provoz a mohlo zpusobit nekonzistenci dat.
         */
        if (desc && desc->format == OBD2_FMT_RAW) {
            obd2_pid_raw_t raw;
            obd2_status_t st = obd2_get_pid_raw(pid, &raw);
            if (st == OBD2_OK && raw.data_len > 0) {
                ok_count++;
                JsonArray arr = data[key].to<JsonArray>();
                for (uint8_t j = 0; j < raw.data_len; j++) {
                    char hex[6];
                    snprintf(hex, sizeof(hex), "0x%02X", raw.data[j]);
                    arr.add(hex);
                }
            }
            if (want_diag) {
                obd2_pid_decoded_t diag_decoded;
                memset(&diag_decoded, 0, sizeof(diag_decoded));
                if (st == OBD2_OK) {
                    diag_decoded.raw_data_len = raw.data_len;
                    memcpy(diag_decoded.raw_data, raw.data, raw.data_len);
                    _ws_add_stream_pid_diag(diag_data[key].to<JsonObject>(), &diag_decoded, st);
                } else {
                    _ws_add_stream_pid_diag(diag_data[key].to<JsonObject>(), NULL, st);
                }
            }
            continue;
        }

        obd2_pid_decoded_t decoded;
        obd2_status_t st = obd2_get_pid(pid, &decoded);

        if (st != OBD2_OK) {
            if (want_diag) {
                _ws_add_stream_pid_diag(diag_data[key].to<JsonObject>(), NULL, st);
            }
            continue;  /* DASH zachovava puvodni chovani: chybne PIDy se preskoci */
        }

        ok_count++;
        if (want_diag) {
            _ws_add_stream_pid_diag(diag_data[key].to<JsonObject>(), &decoded, st);
        }

        if (!desc) {
            data[key] = (double)decoded.value;
            continue;
        }

        /*
         * Format vystupu podle typu PIDu:
         *
         * 1) Bitove pole / enum / config — hex string ("0xNN", "0xNNNN", "0xNNNNNNNN").
         *    Frontend dekoduje pomoci PID_INFO[pid].vals nebo specialnich funkci
         *    (decodeMonitorStatusVal pro $01).
         *
         * 2) Multi-value (O2 senzory, EGT 4-sensor, NOx) — pole floatu o velikosti
         *    value_count (1-4). Frontend rozezna pole vs skalar pres Array.isArray().
         *
         * 3) Skalarni (RPM, teploty, tlaky...) — primo float.
         */
        switch (desc->format) {
        case OBD2_FMT_BIT_ENCODED:
        case OBD2_FMT_ENUM:
        case OBD2_FMT_CONFIG: {
            char hex[2 + OBD2_PID_MAX_DATA_BYTES * 2 + 1];
            _ws_format_pid_raw_hex(decoded, hex, sizeof(hex));
            data[key] = hex;
            break;
        }

        case OBD2_FMT_SIGNED_OFFSET_1B: {
            if (!isnan(decoded.secondary)) {
                JsonArray arr = data[key].to<JsonArray>();
                arr.add((double)decoded.value);
                arr.add((double)decoded.secondary);
            } else {
                data[key] = (double)decoded.value;
            }
            break;
        }

        case OBD2_FMT_O2_CONV:
        case OBD2_FMT_O2_WIDE_EQ_V:
        case OBD2_FMT_O2_WIDE_EQ_I: {
            /* O2 senzory — dvojhodnota (primary + secondary). Posilame jako pole
             * pro symetrii s multi-sensor PIDy. */
            JsonArray arr = data[key].to<JsonArray>();
            arr.add((double)decoded.value);
            if (!isnan(decoded.secondary)) arr.add((double)decoded.secondary);
            break;
        }

        case OBD2_FMT_TEMP_4S:
        case OBD2_FMT_NOX_4S: {
            /* Multi-sensor: az 4 hodnoty. value_count rika kolik jich je validnich.
             * NaN slot se posila jako null v JSON, ale ArduinoJson pro float NaN
             * dela serializaci jako null automaticky. */
            JsonArray arr = data[key].to<JsonArray>();
            arr.add((double)decoded.value);
            if (decoded.value_count >= 2) {
                if (isnan(decoded.secondary)) arr.add(nullptr);
                else arr.add((double)decoded.secondary);
            }
            if (decoded.value_count >= 3) {
                if (isnan(decoded.extra[0])) arr.add(nullptr);
                else arr.add((double)decoded.extra[0]);
            }
            if (decoded.value_count >= 4) {
                if (isnan(decoded.extra[1])) arr.add(nullptr);
                else arr.add((double)decoded.extra[1]);
            }
            break;
        }

        default:
            /* Skalarni formaty (LINEAR_1B/2B/4B, SIGNED, ...) */
            data[key] = (double)decoded.value;
            break;
        }
    }

    doc["ts"] = (uint32_t)millis();
    doc["n"]  = ok_count;

    _ws_serialize(doc, resp);
}
