/**
 * @file ws_commands.cpp
 * @brief Implementace vsech OBD command handleru pro WebSocket rozhrani
 *
 * Tento soubor obsahuje vsechny _ws_cmd_* funkce — kazda zpracovava
 * jeden typ OBD-II diagnostickeho prikazu prijateho pres WebSocket.
 *
 * Architektura:
 *   ws_handler.cpp (dispatch switch) vola _ws_cmd_* funkce z tohoto souboru.
 *   Kazdy handler:
 *     1. Vytvori ArduinoJson dokument
 *     2. Zkontroluje _obd_initialized (guard)
 *     3. Zavola prislusne obd2_* funkce z C vrstvy
 *     4. Naplni JSON dokument vysledky
 *     5. Serializuje dokument do resp->json pomoci _ws_serialize()
 *
 * Funkce _ws_set_error a _ws_serialize jsou definovane v ws_handler.cpp
 * a pristupujeme k nim pres extern deklarace nize.
 *
 * Promenne _obd_initialized a _stream_cfg jsou rovnez v ws_handler.cpp
 * a pristupujeme k nim pres extern.
 *
 * Vsechny funkce v tomto souboru NEJSOU static — jsou volane
 * z dispatch switche ve ws_process_obd_command() v ws_handler.cpp.
 *
 * @see ws_handler.cpp pro dispatch logiku a infrastrukturu
 * @see ws_handler.h pro dokumentaci verejneho API a datove typy
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

/* ========================================================================= */
/*  Extern deklarace — pristup ke sdilenym promennym a funkcim               */
/*  z ws_handler.cpp                                                          */
/* ========================================================================= */

/**
 * Pomocna funkce pro vlozeni chybovych poli do JSON dokumentu.
 * Definovana v ws_handler.cpp. Pouziva se ve vsech handlerech
 * pri chybe obd2_* volani (timeout, negative response, atd.).
 *
 * Priklad pouziti:
 *   _ws_set_error(doc, OBD2_ERR_TIMEOUT, "ECU neodpovida");
 *   → doc bude obsahovat {"status":"error","error":"TIMEOUT","message":"ECU neodpovida"}
 */
extern void _ws_set_error(JsonDocument &doc, obd2_status_t obd_st,
                          const char *message);

/**
 * Pomocna funkce pro serializaci JSON dokumentu do response bufferu.
 * Definovana v ws_handler.cpp. Kontroluje preteceni bufferu.
 *
 * Priklad pouziti:
 *   _ws_serialize(doc, resp);
 *   → resp->json bude obsahovat serializovany JSON retezec
 */
extern void _ws_serialize(JsonDocument &doc, obd_response_msg_t *resp);

/**
 * Stav OBD inicializace — true po uspesnem CMD_INIT.
 * Definovana v ws_handler.cpp. Pouziva se jako guard ve vsech
 * handlerech krome _ws_cmd_ping (ten OBD vrstvu nepotrebuje).
 *
 * Zapisuje se pouze v _ws_cmd_init (pri uspechu → true).
 * Cte se ve vsech ostatnich handlerech pro kontrolu inicializace.
 */
extern volatile bool _obd_initialized;

/**
 * Konfigurace streamu (aktivita, PIDy, interval).
 * Definovana v ws_handler.cpp. Typ ws_stream_cfg_t je v ws_handler.h.
 */
extern volatile ws_stream_cfg_t _stream_cfg;

/* ========================================================================= */
/*  Konfigurace CAN pinu — potrebujeme pro _ws_cmd_init                      */
/* ========================================================================= */
/*
 * Tyto #define musi odpovidat hodnotam v ws_handler.cpp (a .ino).
 * Pokud jsou definovane jinde (napr. v .ino), #ifndef zabrani
 * redefinici a compiler warning.
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
/*  Command handlery — kazdy prikaz ma svoji funkci                          */
/* ========================================================================= */

/**
 * @brief CMD_PING — test zivosti spojeni s ESP32.
 *
 * Nepotrebuje OBD vrstvu ani CAN komunikaci. Odpovida okamzite
 * s uzitecnymi informacemi o stavu systemu:
 *   - free_heap: volna pamet na heapu (pro detekci memory leaku)
 *   - uptime_ms: cas od startu ESP32 (pro detekci resetu)
 *   - obd_init: zda je OBD vrstva inicializovana
 *
 * Priklad odpovedi:
 *   {"cmd":"ping","status":"ok","free_heap":245760,"uptime_ms":12345,"obd_init":true}
 *
 * Hranicni pripady:
 *   - Muze byt volano pred CMD_INIT — obd_init bude false
 *   - Muze byt volano i kdyz je OBD task zaneprazdnen (zpracovava se na Core 0)
 */
void _ws_cmd_ping(const obd_request_msg_t *req,
                         obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"]       = "ping";
    doc["status"]    = "ok";
    doc["free_heap"] = (uint32_t)esp_get_free_heap_size();
    doc["uptime_ms"] = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    doc["obd_init"]  = _obd_initialized;
    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_INIT — inicializace cele OBD-II diagnosticke vrstvy.
 *
 * Sekvence inicializace:
 *   1. obd2_init(baudrate, tx_pin, rx_pin) — inicializuje TWAI driver
 *      a ISO-TP vrstvu. Pokud uz je inicializovano, obd2_init to detekuje
 *      a vrati OK (bezpecne opetovne volani).
 *   2. obd2_set_timeout(2000) — nastavi timeout na 2000ms.
 *      2 sekundy je dostatecne i pro Mode 09 (VIN, ECU name) kde
 *      ECU odpovida multi-frame zpravou pres ISO-TP.
 *   3. obd2_set_log_level + isotp_set_log_level — snizeni logovani
 *      pro produkci (INFO misto TRACE, ktere by zaplavilo serial).
 *   4. obd2_query_supported_pids() — broadcast discovery na 0x7DF,
 *      ECU odpovi s bitmask podporovanych PIDu ktere se ulozi do cache.
 *
 * Pri selhani kterehokoliv kroku vraci chybu a _obd_initialized zustane false.
 *
 * Priklad uspesne odpovedi:
 *   {"cmd":"init","status":"ok","supported_pids":[4,5,12,13,...],"pid_count":28}
 *
 * Priklad chybove odpovedi (napr. CAN sbernice neni pripojena):
 *   {"cmd":"init","status":"error","error":"TIMEOUT","message":"query_supported_pids selhalo"}
 */
void _ws_cmd_init(const obd_request_msg_t *req,
                         obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "init";

    /* Inicializace OBD vrstvy (ta internne vola isotp_init) */
    obd2_status_t st = obd2_init(CAN_BAUDRATE, CAN_TX_PIN, CAN_RX_PIN);
    if (st != OBD2_OK) {
        _ws_set_error(doc, st, "obd2_init selhalo");
        _ws_serialize(doc, resp);
        return;
    }

    /* Nastaveni timeoutu — 2000ms je dostatecne i pro Mode 09 */
    obd2_set_timeout(2000);

    /* Snizeni urovne logovani pro produkci (INFO misto TRACE) */
    obd2_set_log_level(ISOTP_LOG_INFO);
    isotp_set_log_level(ISOTP_LOG_INFO);

    /* Discovery podporovanych PIDu (broadcast na 0x7DF) */
    st = obd2_query_supported_pids();
    if (st != OBD2_OK) {
        _ws_set_error(doc, st, "query_supported_pids selhalo");
        _ws_serialize(doc, resp);
        return;
    }

    _obd_initialized = true;

    /* Sestaveni seznamu podporovanych PIDu */
    doc["status"] = "ok";
    JsonArray pids = doc["supported_pids"].to<JsonArray>();
    for (uint16_t pid = 0x01; pid <= 0xFF; pid++) {
        if (obd2_is_pid_supported((uint8_t)pid)) {
            pids.add(pid);
        }
    }
    doc["pid_count"] = pids.size();

    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_PID — cteni a dekodovani jednoho PIDu (Mode 01).
 *
 * Vola obd2_get_pid() ktery internne:
 *   1. Zkontroluje supported bitmask (jestli ECU PID podporuje)
 *   2. Odesle request na physical ECU adresu (0x7E0)
 *   3. Prijme odpoved a dekoduje ji podle tabulky (SAE J1979 Annex B)
 *
 * Odpoved obsahuje dekodovanou hodnotu, nazev PIDu a jednotku.
 *
 * Typy hodnot v odpovedi:
 *   - Bezne PIDy (teplota, otacky...): "value" jako float
 *     Priklad: {"cmd":"get_pid","pid":12,"status":"ok","name":"Engine RPM","value":875.25,"unit":"rpm"}
 *   - Dual-value PIDy (O2 senzory): "value" + "secondary"
 *     Priklad: {...,"value":0.45,"secondary":3.2,...} (napeti + proud)
 *   - Bit-encoded PIDy ($01 monitor status, $03 fuel system...): "value_raw" jako hex
 *     Priklad: {...,"value_raw":"0x00070007",...}
 *
 * Hranicni pripady:
 *   - OBD neinicializovano → OBD2_ERR_NOT_INITIALIZED
 *   - PID neni podporovan ECU → OBD2_ERR_PID_NOT_SUPPORTED
 *   - ECU neodpovida (odpojeny kabel) → OBD2_ERR_TIMEOUT
 *   - ECU vraci negative response → OBD2_ERR_NEGATIVE_RESP + NRC detail
 */
void _ws_cmd_get_pid(const obd_request_msg_t *req,
                            obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "get_pid";
    doc["pid"] = req->pid;

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Zavolejte nejdrive init");
        _ws_serialize(doc, resp);
        return;
    }

    obd2_pid_decoded_t decoded;
    obd2_status_t st = obd2_get_pid(req->pid, &decoded);

    if (st != OBD2_OK) {
        _ws_set_error(doc, st, NULL);
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"] = "ok";
    doc["name"]   = decoded.name;
    doc["unit"]   = decoded.unit;

    /*
     * Rozliseni typu hodnoty:
     *   - Bit-encoded PIDy (format BIT_ENCODED): hodnota je uint32 pretypovana
     *     na float → posleme jako hex string "0xXXXXXXXX"
     *   - Dual-value PIDy (O2 senzory): primary + secondary
     *   - Ostatni: jednoducha float hodnota
     */
    const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(req->pid);
    if (desc && desc->format == OBD2_FMT_BIT_ENCODED) {
        char hex[12];
        snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)(uint32_t)decoded.value);
        doc["value_raw"] = hex;
    } else {
        doc["value"] = (double)decoded.value;
        /* Secondary hodnota — jen pokud neni NaN (dual-value PID) */
        if (!isnan(decoded.secondary)) {
            doc["secondary"] = (double)decoded.secondary;
        }
    }

    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_PIDS — cteni vice PIDu najednou v jednom prikazu.
 *
 * Iteruje pres pole PIDu z requestu a pro kazdy vola obd2_get_pid().
 * Odpoved obsahuje pole vysledku — kazdy s hodnotou nebo chybou.
 * Casti vysledku mohou byt uspesne i kdyz jine selzou (napr. kdyz
 * jeden PID neni podporovan ale ostatni ano).
 *
 * Maximalne WS_MAX_PIDS_PER_REQUEST PIDu (16) v jednom prikazu.
 * Pri vice PIDech roste cas zpracovani (kazdy PID = 20-50ms CAN komunikace).
 * Pro 16 PIDu je to maximalne ~800ms.
 *
 * Priklad odpovedi:
 *   {"cmd":"get_pids","results":[
 *     {"pid":12,"name":"Engine RPM","value":875.25,"unit":"rpm","status":"ok"},
 *     {"pid":5,"name":"Engine Coolant Temp","value":87,"unit":"°C","status":"ok"},
 *     {"pid":99,"status":"error","error":"PID_NOT_SUPPORTED"}
 *   ],"status":"ok"}
 *
 * Hranicni pripady:
 *   - OBD neinicializovano → cely prikaz selze (ne jednotlive PIDy)
 *   - Prazdne pole PIDu (pid_count=0) → prazdny results array, status ok
 *   - Nektery PID selze → jeho polozka v results ma status error, ostatni ok
 */
void _ws_cmd_get_pids(const obd_request_msg_t *req,
                             obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "get_pids";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Zavolejte nejdrive init");
        _ws_serialize(doc, resp);
        return;
    }

    JsonArray results = doc["results"].to<JsonArray>();

    for (uint8_t i = 0; i < req->pid_count; i++) {
        JsonObject item = results.add<JsonObject>();
        uint8_t pid = req->pids[i];
        item["pid"] = pid;

        obd2_pid_decoded_t decoded;
        obd2_status_t st = obd2_get_pid(pid, &decoded);

        if (st == OBD2_OK) {
            item["status"] = "ok";
            item["name"]   = decoded.name;
            item["unit"]   = decoded.unit;

            const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(pid);
            if (desc && desc->format == OBD2_FMT_BIT_ENCODED) {
                char hex[12];
                snprintf(hex, sizeof(hex), "0x%08lX",
                         (unsigned long)(uint32_t)decoded.value);
                item["value_raw"] = hex;
            } else {
                item["value"] = (double)decoded.value;
                if (!isnan(decoded.secondary)) {
                    item["secondary"] = (double)decoded.secondary;
                }
            }
        } else {
            item["status"] = "error";
            item["error"]  = obd2_status_str(st);
        }
    }

    doc["status"] = "ok";
    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_SUPPORTED_PIDS — seznam podporovanych PIDu z interniho cache.
 *
 * Nevytvari CAN komunikaci — pouze cte z interniho bitmask cache
 * naplneneho pri CMD_INIT (obd2_query_supported_pids).
 * Rychla odpoved (< 1ms), vhodna pro inicializaci klientskeho UI —
 * klient muze zobrazit jen relevantni PIDy ktere ECU podporuje.
 *
 * Priklad odpovedi:
 *   {"cmd":"get_supported_pids","status":"ok","pids":[4,5,6,7,11,12,13,...],"count":28}
 *
 * Hranicni pripady:
 *   - OBD neinicializovano → chyba (cache je prazdny)
 *   - ECU nepodporuje zadne PIDy (nepravdepodobne) → prazdny array, count=0
 */
void _ws_cmd_get_supported_pids(const obd_request_msg_t *req,
                                       obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "get_supported_pids";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Zavolejte nejdrive init");
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"] = "ok";
    JsonArray pids = doc["pids"].to<JsonArray>();
    for (uint16_t pid = 0x01; pid <= 0xFF; pid++) {
        if (obd2_is_pid_supported((uint8_t)pid)) {
            pids.add(pid);
        }
    }
    doc["count"] = pids.size();

    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_DTC / CMD_GET_PENDING_DTC — cteni diagnostickych chybovych kodu.
 *
 * Zpracovava dva typy prikazu podle req->cmd:
 *   - CMD_GET_DTC (Mode 03): potvrzene chybove kody — MIL (check engine) sviti,
 *     chyba se vyskytla opakovane a splnila podminky pro potvrzeni.
 *   - CMD_GET_PENDING_DTC (Mode 07): pending kody — chyba z aktualniho
 *     nebo posledniho jezdniho cyklu, jeste nesplnila podminky pro potvrzeni.
 *     Uzitecne pro diagnostiku intermittentnich problemu.
 *
 * Pouziva broadcast (0x7DF) — vsechny ECU ve vozidle odpovi svymi DTC.
 * DTC kody jsou dekodovane do standardniho formatu:
 *   - P0xxx: Powertrain (motor, prevodovka)
 *   - C0xxx: Chassis (podvozek, ABS, ESP)
 *   - B0xxx: Body (karoserie, airbag, klimatizace)
 *   - U0xxx: Network (komunikacni chyby mezi ECU)
 *
 * Priklad odpovedi:
 *   {"cmd":"get_dtc","status":"ok","count":2,"dtcs":["P0171","P0300"]}
 *   (P0171 = chuda smes, P0300 = nahodne vypadky zapalovani)
 *
 * Hranicni pripady:
 *   - Zadne DTC → count=0, prazdny dtcs array (vozidlo je v poradku)
 *   - ECU neodpovida → TIMEOUT
 *   - Maximalne OBD2_MAX_DTC_COUNT kodu (typicky 32)
 */
void _ws_cmd_get_dtc(const obd_request_msg_t *req,
                            obd_response_msg_t *resp)
{
    JsonDocument doc;
    bool is_pending = (req->cmd == CMD_GET_PENDING_DTC);
    doc["cmd"] = is_pending ? "get_pending_dtc" : "get_dtc";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Zavolejte nejdrive init");
        _ws_serialize(doc, resp);
        return;
    }

    obd2_dtc_t dtcs[OBD2_MAX_DTC_COUNT];
    uint8_t count = 0;
    obd2_status_t st;

    if (is_pending) {
        st = obd2_read_pending_dtc(dtcs, OBD2_MAX_DTC_COUNT, &count);
    } else {
        st = obd2_read_dtc(dtcs, OBD2_MAX_DTC_COUNT, &count);
    }

    if (st != OBD2_OK) {
        _ws_set_error(doc, st, NULL);
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"] = "ok";
    doc["count"]  = count;
    JsonArray arr = doc["dtcs"].to<JsonArray>();
    for (uint8_t i = 0; i < count; i++) {
        arr.add(dtcs[i].code);
    }

    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_CLEAR_DTC — smazani diagnostickych informaci (Mode 04).
 *
 * POZOR: Tento prikaz maze VSECHNO:
 *   - Vsechny potvrzene DTC (Mode 03)
 *   - Vsechny pending DTC (Mode 07)
 *   - Freeze frame data
 *   - Readiness bity (monitory se resetuji na "nekompletni")
 *   - MIL (check engine svetlo) se zhasne
 *
 * Podminky pro uspesne provedeni:
 *   - Zapaleni musi byt ON (klicek v pozici II)
 *   - Motor by mel byt OFF (nektere ECU to vyzaduji)
 *
 * Po smazani je potreba projet jezdni cyklus aby se monitory
 * znovu dokoncily — do te doby vozidlo neprojde emisni kontrolou (STK).
 *
 * Priklad odpovedi:
 *   {"cmd":"clear_dtc","status":"ok","message":"DTC, freeze frame a readiness bity smazany"}
 */
void _ws_cmd_clear_dtc(const obd_request_msg_t *req,
                              obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "clear_dtc";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Zavolejte nejdrive init");
        _ws_serialize(doc, resp);
        return;
    }

    obd2_status_t st = obd2_clear_dtc();
    if (st != OBD2_OK) {
        _ws_set_error(doc, st, NULL);
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"]  = "ok";
    doc["message"] = "DTC, freeze frame a readiness bity smazany";
    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_VIN — cteni identifikacniho cisla vozidla (Mode 09, InfoType $02).
 *
 * VIN je 17-znakovy retezec podle ISO 3779 / ISO 3780.
 * Obsahuje informace o vyrobci, modelu, roku vyroby a seriove cislo.
 * Priklad: "WVWZZZ3CZWE123456"
 *   - WVW = Volkswagen
 *   - ZZZ = vyplnove znaky
 *   - 3C = Passat
 *   - Z = kontrolni cislo
 *   - W = rok 2024
 *   - E = zavod
 *   - 123456 = seriove cislo
 *
 * Nektere ECU (napr. Peugeot, Citroen s EOBD) VIN pres Mode 09
 * nepodporuji — ECU vrati negative response (NRC $12 subFunctionNotSupported).
 * To je normalni chovani a klient zobrazi prislusnou chybu.
 *
 * Priklad odpovedi:
 *   {"cmd":"get_vin","status":"ok","vin":"WVWZZZ3CZWE123456"}
 *
 * Hranicni pripady:
 *   - ECU nepodporuje Mode 09 → NEGATIVE_RESP
 *   - Multi-frame odpoved (VIN ma 17 znaku = vyzaduje ISO-TP segmentaci)
 */
void _ws_cmd_get_vin(const obd_request_msg_t *req,
                            obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "get_vin";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Zavolejte nejdrive init");
        _ws_serialize(doc, resp);
        return;
    }

    char vin[OBD2_VIN_LENGTH + 1];
    obd2_status_t st = obd2_read_vin(vin, sizeof(vin));

    if (st != OBD2_OK) {
        _ws_set_error(doc, st, "VIN neni podporovano touto ECU");
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"] = "ok";
    doc["vin"]    = vin;
    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_MONITOR_STATUS — emisni readiness stav (PID $01, Mode 01).
 *
 * Dekoduje 4-bytovou odpoved PID $01 na strukturovany JSON:
 *   - MIL (Malfunction Indicator Lamp = check engine svetlo): zapnuto/vypnuto
 *   - Pocet ulozenych DTC
 *   - Stav kazdého emisniho monitoru: supported (ECU ho ma) + ready (test dokoncen)
 *
 * Tato informace je klicova pro emisni kontrolu (STK/TUV/MOT):
 *   - Pokud je MIL zapnuta → vozidlo neprojde
 *   - Pokud nektery podporovany monitor neni ready → muze neprojit
 *     (zavisí na pravidlech dane zeme)
 *
 * Monitory se deli na:
 *   Kontinualni (bezi stale pri chodu motoru):
 *     - misfire: vypadky zapalovani
 *     - fuel_system: regulace paliva (lambda)
 *     - components: kontrola komponent (senzory, aktuatory)
 *
 *   Nekontinualni (bezi jen za specifickych podminek):
 *     - catalyst: katalyzator (vyzaduje ustalenou jizdu)
 *     - heated_cat: vyhrivany katalyzator
 *     - evap_system: tesnost paliove soustavy (EVAP)
 *     - secondary_air: sekundarni vzduch
 *     - ac_refrigerant: klimatizacni chladivo
 *     - oxygen_sensor: lambda sonda
 *     - oxygen_heater: ohrev lambda sondy
 *     - egr_system: recirkulace vyfukovych plynu (EGR)
 *
 * Priklad odpovedi:
 *   {"cmd":"get_monitor_status","status":"ok","mil":false,"dtc_count":0,
 *    "monitors":{"misfire":{"sup":true,"rdy":true},"catalyst":{"sup":true,"rdy":false},...}}
 */
void _ws_cmd_get_monitor_status(const obd_request_msg_t *req,
                                       obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "get_monitor_status";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Zavolejte nejdrive init");
        _ws_serialize(doc, resp);
        return;
    }

    obd2_monitor_status_t status;
    obd2_status_t st = obd2_get_monitor_status(NULL, &status);

    if (st != OBD2_OK) {
        _ws_set_error(doc, st, NULL);
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"]    = "ok";
    doc["mil"]       = status.mil_on;
    doc["dtc_count"] = status.dtc_count;

    /*
     * Monitory: pro kazdy uvedeme zda je ECU podporuje (sup)
     * a zda je test dokoncen (rdy). Neni-li podporovan, ready
     * nema smysl zobrazovat.
     */
    JsonObject mon = doc["monitors"].to<JsonObject>();

    /* Kontinualni monitory (bezi stale) */
    JsonObject m;
    m = mon["misfire"].to<JsonObject>();
    m["sup"] = status.misfire_sup;  m["rdy"] = status.misfire_rdy;

    m = mon["fuel_system"].to<JsonObject>();
    m["sup"] = status.fuel_sys_sup; m["rdy"] = status.fuel_sys_rdy;

    m = mon["components"].to<JsonObject>();
    m["sup"] = status.ccm_sup;      m["rdy"] = status.ccm_rdy;

    /* Nekontinualni monitory */
    m = mon["catalyst"].to<JsonObject>();
    m["sup"] = status.cat_sup;      m["rdy"] = status.cat_rdy;

    m = mon["heated_cat"].to<JsonObject>();
    m["sup"] = status.hcat_sup;     m["rdy"] = status.hcat_rdy;

    m = mon["evap_system"].to<JsonObject>();
    m["sup"] = status.evap_sup;     m["rdy"] = status.evap_rdy;

    m = mon["secondary_air"].to<JsonObject>();
    m["sup"] = status.air_sup;      m["rdy"] = status.air_rdy;

    m = mon["ac_refrigerant"].to<JsonObject>();
    m["sup"] = status.acrf_sup;     m["rdy"] = status.acrf_rdy;

    m = mon["oxygen_sensor"].to<JsonObject>();
    m["sup"] = status.o2s_sup;      m["rdy"] = status.o2s_rdy;

    m = mon["oxygen_heater"].to<JsonObject>();
    m["sup"] = status.htr_sup;      m["rdy"] = status.htr_rdy;

    m = mon["egr_system"].to<JsonObject>();
    m["sup"] = status.egr_sup;      m["rdy"] = status.egr_rdy;

    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_FREEZE_FRAME — cteni freeze frame PIDu (Mode 02).
 *
 * Freeze frame uchovava "snimek" hodnot PIDu v okamziku vzniku DTC.
 * Uzitecne pro diagnostiku — napr. jake byly otacky, teplota a zatez
 * motoru kdyz se vyskytla chyba.
 *
 * Request obsahuje PID (ktery chceme z freeze frame precist)
 * a cislo freeze frame (hardcoded 0x00 — prvni/jediny frame).
 * Vetsina ECU podporuje pouze frame 0x00.
 *
 * Odpoved obsahuje:
 *   - Dekodovanou hodnotu (stejne jako u Mode 01)
 *   - Raw data jako hex string pro pokrocilou diagnostiku
 *   - Sekundarni hodnotu pokud existuje (dual-value PIDy)
 *
 * Priklad odpovedi:
 *   {"cmd":"get_freeze_frame","pid":12,"status":"ok","data_len":2,
 *    "value":2500.0,"name":"Engine RPM","unit":"rpm","raw":"09C4"}
 *
 * Hranicni pripady:
 *   - Zadne DTC → freeze frame muze byt prazdny (ECU vrati NEGATIVE_RESP)
 *   - PID neni v freeze frame → ECU vrati chybu
 *   - Dekodovani selze (neznamy PID) → val je NaN, odpoved bude bez "value"
 */
void _ws_cmd_get_freeze_frame(const obd_request_msg_t *req,
                                      obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "get_freeze_frame";
    doc["pid"] = req->pid;

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Call init first");
        _ws_serialize(doc, resp);
        return;
    }

    obd2_pid_raw_t raw;
    obd2_status_t st = obd2_get_freeze_frame_raw(req->pid, 0x00, &raw);

    if (st != OBD2_OK) {
        _ws_set_error(doc, st, NULL);
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"]   = "ok";
    doc["data_len"] = raw.data_len;

    /* Dekodujeme hodnotu stejnym zpusobem jako Mode 01 */
    float val = obd2_decode_pid_value(req->pid, raw.data, raw.data_len);
    if (!isnan(val)) {
        const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(req->pid);
        doc["value"] = (double)val;
        if (desc) {
            doc["name"] = desc->name;
            doc["unit"] = desc->unit;
        }
        float sec = obd2_decode_pid_secondary(req->pid, raw.data, raw.data_len);
        if (!isnan(sec)) {
            doc["secondary"] = (double)sec;
        }
    }

    /* Raw data jako hex string pro pokrocilou diagnostiku */
    char hex[12];
    int off = 0;
    for (uint8_t i = 0; i < raw.data_len && off < 10; i++) {
        off += snprintf(hex + off, sizeof(hex) - off, "%02X", raw.data[i]);
    }
    doc["raw"] = hex;

    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_ECU_NAME — cteni jmena ridici jednotky (Mode 09, InfoType $0A).
 *
 * Vraci textovy identifikator ECU — typicky 4-znakova zkratka
 * nasledovana popisem. Format zavisi na vyrobci.
 * Priklad: "ECM\0-Engine Control Module" nebo "TCM\0-Transmission"
 *
 * Nektere ECU (zejmena starsi nebo levnejsi) tuto funkci nepodporuji
 * a vrati negative response — to je normalni a ocekavane chovani.
 *
 * Priklad odpovedi:
 *   {"cmd":"get_ecu_name","status":"ok","ecu_name":"ECM-Engine Control Module"}
 *
 * Hranicni pripady:
 *   - ECU nepodporuje InfoType $0A → NEGATIVE_RESP
 *   - Jmeno obsahuje non-printable znaky → mohou se zobrazit v JSON
 *   - Maximalni delka: OBD2_ECU_NAME_MAX_LENGTH znaku
 */
void _ws_cmd_get_ecu_name(const obd_request_msg_t *req,
                                  obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "get_ecu_name";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Call init first");
        _ws_serialize(doc, resp);
        return;
    }

    char name_buf[OBD2_ECU_NAME_MAX_LENGTH + 1];
    obd2_status_t st = obd2_read_ecu_name(name_buf, sizeof(name_buf));

    if (st != OBD2_OK) {
        _ws_set_error(doc, st, "ECU name not supported");
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"]   = "ok";
    doc["ecu_name"] = name_buf;
    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_GET_CAL_ID — cteni kalibracnich identifikatoru (Mode 09, InfoType $04).
 *
 * Kazde CalID je 16-znakovy retezec identifikujici verzi softwarove
 * kalibrace ECU. Muze byt vice nez jedno — NODI byte v odpovedi
 * urcuje pocet (typicky 1-4).
 *
 * CalID je dulezite pro emisni kontrolu — overuje se ze ECU
 * pouziva schvalenou verzi software (ne tuning/chip-tuning).
 *
 * Priklad odpovedi:
 *   {"cmd":"get_cal_id","status":"ok","count":2,
 *    "cal_ids":["39101-03CA3   ","39101-03CB1   "]}
 *
 * Hranicni pripady:
 *   - ECU nepodporuje InfoType $04 → NEGATIVE_RESP
 *   - CalID muze obsahovat mezery (padding do 16 znaku)
 *   - Maximalne OBD2_MAX_INFO_ITEMS polozek
 */
void _ws_cmd_get_cal_id(const obd_request_msg_t *req,
                                obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "get_cal_id";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Call init first");
        _ws_serialize(doc, resp);
        return;
    }

    char cal_ids[OBD2_MAX_INFO_ITEMS][OBD2_CAL_ID_LENGTH + 1];
    uint8_t count = 0;
    obd2_status_t st = obd2_read_cal_id(cal_ids, &count);

    if (st != OBD2_OK) {
        _ws_set_error(doc, st, "Calibration ID not supported");
        _ws_serialize(doc, resp);
        return;
    }

    doc["status"] = "ok";
    doc["count"]  = count;
    JsonArray arr = doc["cal_ids"].to<JsonArray>();
    for (uint8_t i = 0; i < count; i++) {
        arr.add(cal_ids[i]);
    }
    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_START_STREAM — spusteni periodickeho cteni PIDu.
 *
 * Klient posle seznam PIDu a volitelny interval (vychozi 200ms).
 * OBD task pak cyklicky cte tyto PIDy a broadcastuje vysledky
 * vsem pripojenym klientum ve kompaktnim formatu.
 *
 * Konfigurace se zapisuje do sdilene volatile struktury _stream_cfg.
 * OBD task ji precte v dalsim cyklu. Poradi zapisu je dulezite:
 * nejdriv pids[], pid_count a interval_ms, az nakonec active=true.
 * Tim se minimalizuje riziko cteni nekonzistentni konfigurace.
 *
 * Pokud uz stream bezi, tento prikaz prepise konfiguraci
 * (zmena PIDu/intervalu za behu bez nutnosti stop_stream).
 *
 * Priklad requestu:
 *   {"cmd":"start_stream","pids":[12,13,5],"interval_ms":100}
 *
 * Priklad odpovedi:
 *   {"cmd":"start_stream","status":"ok","pid_count":3,"interval_ms":100}
 *
 * Hranicni pripady:
 *   - OBD neinicializovano → chyba
 *   - Prazdne pole PIDu (pid_count=0) → chyba NO_PIDS
 *   - Interval < 50ms → automaticky nastaven na 200ms (ochrana pred pretizenim)
 *   - Interval >= 50ms → pouzit tak jak je
 *   - Maximalne WS_MAX_PIDS_PER_REQUEST (16) PIDu
 */
void _ws_cmd_start_stream(const obd_request_msg_t *req,
                                  obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "start_stream";

    if (!_obd_initialized) {
        _ws_set_error(doc, OBD2_ERR_NOT_INITIALIZED, "Zavolejte nejdrive init");
        _ws_serialize(doc, resp);
        return;
    }

    if (req->pid_count == 0) {
        doc["status"]  = "error";
        doc["error"]   = "NO_PIDS";
        doc["message"] = "Zadejte alespon jeden PID ke streamovani";
        _ws_serialize(doc, resp);
        return;
    }

    /* Nastaveni stream konfigurace — OBD task ji precte v dalsim cyklu */
    for (uint8_t i = 0; i < req->pid_count; i++) {
        _stream_cfg.pids[i] = req->pids[i];
    }
    _stream_cfg.pid_count   = req->pid_count;
    _stream_cfg.interval_ms = (req->interval_ms >= 50) ? req->interval_ms : 200;
    _stream_cfg.active      = true;  /* Zapnout az po nastaveni parametru */

    doc["status"]      = "ok";
    doc["pid_count"]   = req->pid_count;
    doc["interval_ms"] = _stream_cfg.interval_ms;
    _ws_serialize(doc, resp);
}

/**
 * @brief CMD_STOP_STREAM — zastaveni periodickeho cteni PIDu.
 *
 * Nastavi _stream_cfg.active na false. OBD task se pak vrati
 * do rezimu blokujiciho cekani na jednotlive prikazy z fronty
 * (xQueueReceive s portMAX_DELAY).
 *
 * Konfigurace (pids, interval) zustava zachovana — dalsi
 * start_stream bez parametru by mohl teoreticky pokracovat
 * (ale aktualne se parametry vzdy prepisuji).
 *
 * Priklad odpovedi:
 *   {"cmd":"stop_stream","status":"ok"}
 *
 * Hranicni pripady:
 *   - Volano kdyz stream nebezi → nevadi, active uz je false, odpoved ok
 *   - Nevyzaduje OBD inicializaci (jen nastavi flag)
 */
void _ws_cmd_stop_stream(const obd_request_msg_t *req,
                                 obd_response_msg_t *resp)
{
    JsonDocument doc;
    doc["cmd"] = "stop_stream";

    _stream_cfg.active = false;

    doc["status"] = "ok";
    _ws_serialize(doc, resp);
}
