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

/* ---- Konfigurace CAN pinu (musi odpovídat hodnotam v .ino) ---- */
/*
 * Vychozi hodnoty TX=12, RX=14 odpovidaji typickemu zapojeni
 * ESP32 s SN65HVD230 CAN transceiverem. Pokud .ino definuje
 * jine piny (napr. pri pouziti jineho breakout boardu),
 * tyto #define se preskoci diky #ifndef.
 *
 * CAN_BAUDRATE 500000 = 500 kbit/s — standardni rychlost
 * pro OBD-II diagnostiku (ISO 15765-4). Nektere vozidla
 * pouzivaji 250 kbit/s, ale 500k je bezne pro osobni auta.
 */
#ifndef CAN_TX_PIN
#define CAN_TX_PIN   12
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN   14
#endif
#ifndef CAN_BAUDRATE
#define CAN_BAUDRATE 500000
#endif

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
volatile ws_stream_cfg_t _stream_cfg = { false, {0}, 0, 200 };

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
    if (strcmp(cmd_str, "init") == 0)                return CMD_INIT;
    if (strcmp(cmd_str, "get_pid") == 0)             return CMD_GET_PID;
    if (strcmp(cmd_str, "get_pids") == 0)            return CMD_GET_PIDS;
    if (strcmp(cmd_str, "get_supported_pids") == 0)  return CMD_GET_SUPPORTED_PIDS;
    if (strcmp(cmd_str, "get_dtc") == 0)             return CMD_GET_DTC;
    if (strcmp(cmd_str, "get_pending_dtc") == 0)     return CMD_GET_PENDING_DTC;
    if (strcmp(cmd_str, "clear_dtc") == 0)           return CMD_CLEAR_DTC;
    if (strcmp(cmd_str, "get_vin") == 0)             return CMD_GET_VIN;
    if (strcmp(cmd_str, "get_monitor_status") == 0)  return CMD_GET_MONITOR_STATUS;
    if (strcmp(cmd_str, "get_freeze_frame") == 0)   return CMD_GET_FREEZE_FRAME;
    if (strcmp(cmd_str, "get_ecu_name") == 0)       return CMD_GET_ECU_NAME;
    if (strcmp(cmd_str, "get_cal_id") == 0)         return CMD_GET_CAL_ID;
    if (strcmp(cmd_str, "start_stream") == 0)       return CMD_START_STREAM;
    if (strcmp(cmd_str, "stop_stream") == 0)         return CMD_STOP_STREAM;

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
 * Pri negativni odpovedi z ECU (OBD2_ERR_NEGATIVE_RESP) pridame navic
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

    /* Pri negativni odpovedi z ECU pridame NRC detail (kod + popis) */
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
void _ws_cmd_init(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_pid(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_pids(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_supported_pids(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_dtc(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_clear_dtc(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_vin(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_monitor_status(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_freeze_frame(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_ecu_name(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_get_cal_id(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_start_stream(const obd_request_msg_t *req, obd_response_msg_t *resp);
void _ws_cmd_stop_stream(const obd_request_msg_t *req, obd_response_msg_t *resp);

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
         * Nevalidni JSON — odesleme chybu primo do response fronty
         * (nemusime zatezovat OBD task).
         * Pouzivame static aby se 1028B struktura neukladala na stack
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
     * CMD_PING nepotrebuje OBD task — zpracujeme primo zde (Core 0)
     * a dame do response fronty. Vsechny ostatni prikazy jdou
     * do request fronty pro OBD task (Core 1).
     */
    if (cmd == CMD_PING) {
        obd_request_msg_t req_msg;
        memset(&req_msg, 0, sizeof(req_msg));
        req_msg.cmd = CMD_PING;
        req_msg.client_id = client_id;
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

    case CMD_START_STREAM: {
        /* Parsovani PIDu a intervalu pro stream */
        JsonArray arr = doc["pids"].as<JsonArray>();
        req_msg.pid_count = 0;
        for (JsonVariant v : arr) {
            if (req_msg.pid_count >= WS_MAX_PIDS_PER_REQUEST) break;
            req_msg.pids[req_msg.pid_count++] = (uint8_t)v.as<int>();
        }
        req_msg.interval_ms = doc["interval_ms"] | 200;  /* vychozi 200ms */
        break;
    }

    default:
        /* Ostatni prikazy nemaji parametry */
        break;
    }

    /*
     * Zarazeni do request fronty. Timeout 100ms — pokud je fronta plna
     * (OBD task nestihá), odesleme chybovou odpoved klientovi.
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
        _ws_cmd_get_dtc(req, resp);
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
    case CMD_START_STREAM:
        _ws_cmd_start_stream(req, resp);
        break;
    case CMD_STOP_STREAM:
        _ws_cmd_stop_stream(req, resp);
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
    JsonObject data = doc["d"].to<JsonObject>();

    uint8_t ok_count = 0;
    for (uint8_t i = 0; i < _stream_cfg.pid_count; i++) {
        uint8_t pid = _stream_cfg.pids[i];
        obd2_pid_decoded_t decoded;
        obd2_status_t st = obd2_get_pid(pid, &decoded);

        if (st == OBD2_OK) {
            /* Klic je PID cislo jako string */
            char key[4];
            snprintf(key, sizeof(key), "%u", pid);
            data[key] = (double)decoded.value;
            ok_count++;
        }
        /* Chybne PIDy preskocime — klient vi ktere chybi */
    }

    doc["ts"] = (uint32_t)millis();
    doc["n"]  = ok_count;

    _ws_serialize(doc, resp);
}
