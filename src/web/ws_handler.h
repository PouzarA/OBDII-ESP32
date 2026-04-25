/**
 * @file ws_handler.h
 * @brief Obsluzna vrstva WebSocket zprav — parsovani prikazu a sestaveni JSON odpovedi
 *
 * Tento hlavickovy soubor definuje rozhrani prostredni vrstvy (middleware)
 * mezi WebSocket serverem a OBD-II diagnostickou vrstvou. Modul zajistuje
 * tri hlavni ulohy:
 *
 *   1. Parsovani prichoziho JSON od klienta a vytvoreni strukturovane
 *      request zpravy (obd_request_msg_t). Napr. klient posle:
 *        {"cmd":"get_pid","pid":12}
 *      a modul z toho vytvori zpravu s cmd=CMD_GET_PID, pid=12.
 *
 *   2. Zpracovani OBD prikazu volanim prislusnych obd2_* funkci
 *      a nasledne sestaveni JSON odpovedi. Napr. pro PID 0x0C (otacky
 *      motoru) vrati:
 *        {"cmd":"get_pid","pid":12,"value":875.25,"status":"ok"}
 *
 *   3. Bezpecnou mezivlaknovou komunikaci prostrednictvim FreeRTOS front
 *      (queues), coz umoznuje oddelit WebSocket obsluhu od blokovacich
 *      CAN operaci.
 *
 * Architektura — rozdeleni prace mezi dve jadra ESP32:
 *
 *   WebSocket callback (Core 0) --> ws_handle_incoming() --> request fronta
 *   request fronta --> OBD task (Core 1) --> ws_process_obd_command()
 *   ws_process_obd_command() --> response fronta --> dispatch task (Core 0)
 *
 *   Core 0 obsluhuje WiFi stack a WebSocket spojeni — nesmi blokovat,
 *   protoze by doslo k vypadku WiFi watchdogu (typicky timeout 6 s).
 *   Core 1 provadi blokovaci CAN komunikaci (kazdy PID trva 20-50 ms,
 *   DTC muze trvat i 200+ ms pri vice ECU).
 *
 *   Diky rozdeleni na dve jadra muze webovy klient plynule komunikovat,
 *   zatimco CAN sbernice zpracovava pozadavky.
 *
 * JSON protokol — priklady komunikace:
 *
 *   Pozadavek na cteni jednoho PIDu:
 *     {"cmd":"get_pid", "pid":12}
 *     -- pid 12 = 0x0C = otacky motoru (RPM)
 *
 *   Uspesna odpoved:
 *     {"cmd":"get_pid", "pid":12, "value":875.25, "status":"ok"}
 *     -- value je jiz prevedena fyzikalni hodnota (ne surova CAN data)
 *
 *   Chybova odpoved (napr. ECU neodpovedela vcas):
 *     {"cmd":"get_pid", "pid":12, "status":"error", "error":"TIMEOUT"}
 *
 *   Pozadavek na vice PIDu najednou:
 *     {"cmd":"get_pids", "pids":[12,13,5]}
 *     -- 0x0C=RPM, 0x0D=rychlost, 0x05=teplota chladici kapaliny
 *
 *   Streaming (periodicke cteni PIDu):
 *     {"cmd":"start_stream", "pids":[12,13], "interval":200}
 *     -- kazdy 200 ms posle broadcast odpoved vsem klientum
 *
 *   Stream odpoved (kompaktni format pro minimalni latenci):
 *     {"cmd":"stream","d":{"12":875.25,"13":45},"ts":12345}
 *     -- "d" = data (zkraceno kvuli sirce pasma), "ts" = timestamp v ms
 *
 * @author Ales (OBD-II bakalarska prace)
 */

#ifndef WS_HANDLER_H
#define WS_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "secrets.h"   /* WS_AUTH_TOKEN_MAX pro pole token[] */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  Konstanty                                                                */
/* ========================================================================= */

/**
 * Maximalni delka JSON odpovedi v bytech.
 *
 * Pro jednoduche odpovedi (jeden PID, DTC kod) staci zhruba 128-256 B.
 * Nicmene prikaz get_supported_pids muze vracet az 40+ PIDu v jednom
 * JSON poli, napr.:
 *   {"cmd":"get_supported_pids","pids":[0,1,3,4,5,6,7,...],"status":"ok"}
 * coz snadno presahne 512 B.
 *
 * Hodnota 1024 B je zvolena jako bezpecna rezerva, ktera pokryje
 * i nejdelsi moznou odpoved (vsech 96 standardnich PIDu + DTC seznam)
 * a zaroven neplytva RAM — struktura obd_response_msg_t se alokuje
 * na zasobniku (stack) nebo ve fronte, takze prilis velky buffer
 * by zvedl spotrebu RAM na kazdy prvek fronty.
 *
 * Hranicni pripad: pokud by vozidlo podporovalo nestandardne mnoho PIDu
 * a zaroven dlouhe DTC popisy, 1024 B by mohlo byt tesne. V takovem
 * pripade je nutne tuto hodnotu zvysit a zaroven zkontrolovat velikost
 * FreeRTOS fronty (viz obd_response_msg_t).
 */
#define WS_RESPONSE_JSON_MAX    2048

/**
 * Maximalni pocet PIDu v jednom hromadnem pozadavku (get_pids / start_stream).
 *
 * Omezeni existuje ze dvou duvodu:
 *   1. Velikost struktury obd_request_msg_t — pole pids[] ma pevnou
 *      velikost, aby se struktura vesla do FreeRTOS fronty bez
 *      dynamicke alokace pameti (malloc na ESP32 v ISR kontextu
 *      neni bezpecny).
 *   2. Doba zpracovani — kazdy PID vyzaduje jednu CAN transakci
 *      trvajici priblizne 20-50 ms (vysila se CAN request a ceka
 *      se na odpoved). Pri 16 PIDech je to 320-800 ms, coz je
 *      jeste prijatelne pro interaktivni pouziti. Vice PIDu by
 *      znamenalo prilis dlouhe blokovani OBD tasku.
 *
 * Priklad: klient posle {"cmd":"get_pids","pids":[12,13,5,15,17,70]}
 * — to je 6 PIDu, coz je v poradku (< 16). Pokud by poslal pole
 * s 20 PIDy, server vrati chybu "too many PIDs".
 */
#define WS_MAX_PIDS_PER_REQUEST 16

/**
 * Maximalni pocet PIDu, pro ktere stream posila rozsirena diagnosticka
 * metadata (raw bajty, ECU ID, status). Drzime stejny limit jako UI Inspector,
 * aby se stream JSON bezpečně vesel do WS_RESPONSE_JSON_MAX.
 */
#define WS_MAX_DIAG_PIDS        4

/* ========================================================================= */
/*  Typy prikazu (vycet prikazu)                                             */
/* ========================================================================= */

/**
 * Vycet vsech podporovanych WebSocket prikazu.
 *
 * Kazdy prvek odpovida jednomu JSON retezci "cmd" od klienta.
 * Mapovani JSON retezce na hodnotu enumu probiha v ws_handle_incoming()
 * pomoci tabulky retezcu (strcmp porovnani).
 *
 * Rozdeleni prikazu podle OBD rezimu (Mode):
 *   - Mode 01: CMD_GET_PID, CMD_GET_PIDS, CMD_GET_SUPPORTED_PIDS,
 *              CMD_GET_MONITOR_STATUS
 *   - Mode 02: CMD_GET_FREEZE_FRAME
 *   - Mode 03: CMD_GET_DTC
 *   - Mode 04: CMD_CLEAR_DTC
 *   - Mode 07: CMD_GET_PENDING_DTC
 *   - Mode 09: CMD_GET_SUPPORTED_INFOTYPES, CMD_GET_VIN, CMD_GET_ECU_NAME,
 *              CMD_GET_CAL_ID, CMD_GET_CVN, CMD_GET_IPT
 *   - Mode 0A: CMD_GET_PERMANENT_DTC
 *   - Bez OBD: CMD_PING (nevyzaduje CAN komunikaci)
 *   - Ridici:  CMD_INIT, CMD_START_STREAM, CMD_STOP_STREAM
 */
typedef enum {
    CMD_UNKNOWN = 0,        /**< Nerozpoznany prikaz — server vrati chybu
                                 "unknown command". Nastane napr. pri preklepu
                                 v JSON klici "cmd" nebo pri zaslani
                                 nepodporovaneho prikazu. */

    CMD_PING,               /**< Test zivosti spojeni. Klient posle
                                 {"cmd":"ping"}, server okamzite vrati
                                 {"cmd":"ping","status":"pong"}.
                                 Nepouziva OBD vrstvu — odpoved se generuje
                                 primo v ws_handle_incoming() na Core 0. */

    CMD_TRANSPORT_INIT,     /**< Inicializuje pouze CAN/ISO-TP transport.
                                 Neprovadi PID $00 discovery a neodemkne
                                 streaming. JSON: {"cmd":"transport_init"} */

    CMD_PID00_PROBE,        /**< Functional probe Mode 01 PID $00 na 0x7DF.
                                 Vraci vsechny odpovedi ECU vcetne rx_id.
                                 JSON: {"cmd":"pid00_probe"} */

    CMD_INIT,               /**< Inicializace OBD spojeni. Provede
                                 obd2_init() pro nastaveni CAN sbernice
                                 a nasledne dotaz na podporovane PIDy
                                 (PID 0x00, 0x20, 0x40...). Vysledek
                                 se ulozi do interni cache.
                                 JSON: {"cmd":"init"}
                                 Odpoved: {"cmd":"init","status":"ok"}
                                 nebo pri chybe: {"cmd":"init","status":"error",
                                 "error":"CAN_INIT_FAILED"} */

    CMD_GET_PID,            /**< Cteni jednoho PIDu v rezimu Mode 01
                                 (aktualni data). Klient zadava cislo PIDu
                                 (0x00-0xFF) a server vrati prepoctenou
                                 fyzikalni hodnotu.
                                 JSON: {"cmd":"get_pid","pid":12}
                                 Odpoved: {"cmd":"get_pid","pid":12,
                                 "value":875.25,"status":"ok"}
                                 Hranicni pripad: pokud PID neni podporovan
                                 danym vozidlem, vrati "NOT_SUPPORTED". */

    CMD_GET_PIDS,           /**< Hromadne cteni vice PIDu najednou.
                                 Efektivnejsi nez opakovane volani get_pid,
                                 protoze se vytvori pouze jedna request
                                 zprava. Maximalne WS_MAX_PIDS_PER_REQUEST
                                 PIDu v jednom pozadavku.
                                 JSON: {"cmd":"get_pids","pids":[12,13,5]}
                                 Odpoved: {"cmd":"get_pids","values":[
                                 {"pid":12,"value":875.25},
                                 {"pid":13,"value":60},
                                 {"pid":5,"value":85}],"status":"ok"} */

    CMD_GET_SUPPORTED_PIDS, /**< Vrati seznam PIDu podporovanych vozidlem.
                                 Data se ctou z interni cache naplnene
                                 behem CMD_INIT — neni nutna dalsi CAN
                                 komunikace, takze odpoved je okamzita.
                                 JSON: {"cmd":"get_supported_pids"}
                                 Odpoved: {"cmd":"get_supported_pids",
                                 "pids":[0,1,3,4,5,6,7,11,12,13,...],
                                 "status":"ok"} */

    CMD_GET_DTC,            /**< Cteni potvrzenych diagnostickych kodu
                                 zavad (DTC) v rezimu Mode 03. Vraci
                                 retezce ve formatu P0xxx, C0xxx apod.
                                 JSON: {"cmd":"get_dtc"}
                                 Odpoved: {"cmd":"get_dtc","dtcs":
                                 ["P0301","P0420"],"status":"ok"}
                                 Prazdny seznam = zadne potvrzene zavady. */

    CMD_GET_PENDING_DTC,    /**< Cteni cekajicich (pending) DTC v rezimu
                                 Mode 07. Pending DTC jsou zavady detekovane
                                 behem jednoho jezdniho cyklu, ale jeste
                                 nepotvrzene. Pouziva se pro vcasnou
                                 diagnostiku.
                                 JSON: {"cmd":"get_pending_dtc"}
                                 Odpoved: stejny format jako CMD_GET_DTC. */

    CMD_GET_MODE06_MONITOR, /**< Raw cteni Mode 06 OBDMID/TID monitoru.
                                 JSON: {"cmd":"get_mode06_monitor","mid":1} */

    CMD_GET_PERMANENT_DTC,  /**< Cteni permanentnich DTC v rezimu Mode 0A.
                                 JSON: {"cmd":"get_permanent_dtc"}
                                 Odpoved: stejny format jako CMD_GET_DTC. */

    CMD_CLEAR_DTC,          /**< Smazani vsech DTC a resetovani MIL
                                 (kontrolka motoru) v rezimu Mode 04.
                                 POZOR: toto trvale smaze ulozene zavady
                                 a resetuje readiness monitory — pouzivat
                                 s rozvahou.
                                 JSON: {"cmd":"clear_dtc"}
                                 Odpoved: {"cmd":"clear_dtc","status":"ok"} */

    CMD_GET_VIN,            /**< Cteni identifikacniho cisla vozidla (VIN)
                                 v rezimu Mode 09, InfoType 0x02.
                                 VIN je 17znakovy retezec jednoznacne
                                 identifikujici vozidlo.
                                 JSON: {"cmd":"get_vin"}
                                 Odpoved: {"cmd":"get_vin",
                                 "vin":"WVWZZZ3CZWE123456","status":"ok"} */

    CMD_GET_MONITOR_STATUS, /**< Cteni stavu readiness monitoru (PID 0x01).
                                 Indikuje, ktere emisni testy byly
                                 dokonceny od posledniho smazani DTC.
                                 Dulezite pro technicke kontroly (STK).
                                 JSON: {"cmd":"get_monitor_status"}
                                 Odpoved obsahuje bitovou mapu monitoru. */

    CMD_GET_FREEZE_FRAME,   /**< Cteni freeze frame dat v rezimu Mode 02.
                                 Freeze frame je snimek vybranych PIDu
                                 v okamziku, kdy doslo k zavade (DTC).
                                 Umoznuje zpetne analyzovat podminky
                                 pri kterych se zavada projevila.
                                 JSON: {"cmd":"get_freeze_frame","pid":12}
                                 Odpoved: stejny format jako CMD_GET_PID. */

    CMD_GET_ECU_NAME,       /**< Cteni jmena ridici jednotky (ECU) v rezimu
                                 Mode 09, InfoType 0x0A.
                                 JSON: {"cmd":"get_ecu_name"}
                                 Odpoved: {"cmd":"get_ecu_name",
                                 "name":"ECM-EngineControl","status":"ok"}
                                 Nektere ECU tento InfoType nepodporuji. */

    CMD_GET_CAL_ID,         /**< Cteni kalibracniho identifikatoru v rezimu
                                 Mode 09, InfoType 0x04. Kalibracni ID
                                 identifikuje verzi softwaru nahranou
                                 v ridici jednotce.
                                 JSON: {"cmd":"get_cal_id"}
                                 Odpoved: {"cmd":"get_cal_id",
                                 "cal_id":"...","status":"ok"} */

    CMD_GET_SUPPORTED_INFOTYPES, /**< Cteni Mode 09 InfoType $00.
                                 Vraci podporovane InfoType per ECU.
                                 JSON: {"cmd":"get_supported_infotypes"} */

    CMD_GET_MODE09_INFO,    /**< Generic read-only cteni libovolneho Mode 09
                                 InfoType. JSON:
                                 {"cmd":"get_mode09_info","infotype":6} */

    CMD_GET_CVN,            /**< Cteni CVN v rezimu Mode 09, InfoType 0x06.
                                 JSON: {"cmd":"get_cvn"} */

    CMD_GET_IPT,            /**< Cteni IPT v rezimu Mode 09, InfoType 0x08
                                 nebo 0x0B. JSON:
                                 {"cmd":"get_ipt","infotype":8} */

    CMD_GET_MONITOR_STATUS_ALL, /**< Cteni stavu readiness monitoru ze vsech ECU
                                 pres broadcast (Mode 01 PID 0x01).
                                 Vraci per-ECU seznam s identifikaci rx_id.
                                 JSON: {"cmd":"get_monitor_status_all"}
                                 Odpoved: {"cmd":"get_monitor_status_all",
                                 "ecus":[{"id":"0x7E8","mil_on":false, ...}],
                                 "status":"ok"} */

    CMD_DISCOVER_ECUS,      /**< Detekce vsech ECU v siti pres broadcast
                                 Mode 09 InfoType 0x0A (ECU name).
                                 Vraci seznam vsech ECU, ktere odpovedely.
                                 JSON: {"cmd":"discover_ecus"}
                                 Odpoved: {"cmd":"discover_ecus",
                                 "ecus":[{"id":"0x7E8","name":"ECM"}],
                                 "status":"ok"} */

    CMD_START_STREAM,       /**< Spusteni periodickeho cteni vybranych PIDu.
                                 Rezim "dash" krmi hlavni UI/trip/GPS/recording,
                                 rezim "inspector" izolovane krmi PIDs Select
                                 vcetne raw diagnostiky.
                                 JSON: {"cmd":"start_stream",
                                 "mode":"dash","pids":[12,13],
                                 "interval_ms":200}
                                 -- interval je v milisekundach (min. 50 ms)
                                 Odpoved na spusteni: {"cmd":"start_stream",
                                 "status":"ok","mode":"dash"}
                                 Nasledne periodicke odpovedi:
                                 {"cmd":"stream","mode":"dash",
                                 "d":{"12":875.25,"13":45},"ts":12345} */

    CMD_STOP_STREAM,        /**< Zastaveni aktivniho streamu. OBD task
                                 prestane periodicke cteni a vrati se
                                 do rezimu cekanina na jednotlive prikazy.
                                 JSON: {"cmd":"stop_stream"}
                                 Odpoved: {"cmd":"stop_stream",
                                 "status":"ok"} */

    CMD_MANUAL_QUERY        /**< Manualni dotaz pro diagnosticky terminal.
                                 Umoznuje poslat libovolny PID v bezpecnych
                                 sluzbach (01, 09) a videt surova data.
                                 JSON: {"cmd":"manual_query","service":1,"pid":12}
                                 Odpoved obsahuje HEX payload a interpretaci. */
} ws_cmd_t;

/**
 * Rezim periodickeho streamu.
 *
 * DASH rezim krmi hlavni ukazatele, trip/statistiky, recording a GPS OBD
 * snapshoty. INSPECTOR rezim je izolovany diagnosticky stream pro PIDs Select.
 */
typedef enum {
    WS_STREAM_MODE_DASH = 0,
    WS_STREAM_MODE_INSPECTOR = 1
} ws_stream_mode_t;

/* ========================================================================= */
/*  Struktury zprav pro FreeRTOS fronty                                      */
/* ========================================================================= */

/**
 * Request zprava: smer WebSocket handler --> OBD task.
 *
 * Tato struktura predstavuje rozparsovany pozadavek od klienta, ktery
 * se preda prostrednictvim FreeRTOS fronty do OBD tasku na Core 1.
 *
 * Pole client_id jednoznacne identifikuje WebSocket klienta, kteremu
 * se ma odpoved odeslat zpet. Vice klientu muze byt pripojeno soucasne
 * (napr. webovy prohlizec na telefonu + tablet), proto je nutne
 * odpovedi spravne smerovat.
 *
 * Velikost struktury je priblizne 72 B (zavisi na zarovnani):
 *   - ws_cmd_t (4 B) + client_id (4 B) + pid (1 B)
 *   + pids[16] (16 B) + pid_count (1 B) + interval_ms (2 B)
 *   + padding pro zarovnani
 *
 * Tato velikost je dostatecne mala pro prenos pres FreeRTOS frontu
 * (fronta kopiruje data hodnotou, ne odkazem). Fronta s kapacitou 10
 * zabere priblizne 720 B RAM, coz je pro ESP32 zanedbatelne.
 *
 * Poznamka ke stack vs. dynamicke alokaci: struktura se vytvari na
 * zasobniku v ws_handle_incoming() a kopiruje do fronty pres
 * xQueueSend(). Dynamicka alokace (malloc) se zamerne nepouziva,
 * aby se predchazelo fragmentaci haldy a problemum v ISR kontextu.
 */
typedef struct {
    ws_cmd_t  cmd;                          /**< Typ prikazu — urcuje, ktera
                                                 obd2_* funkce se zavola
                                                 v OBD tasku. */
    uint32_t  client_id;                    /**< ID WebSocket klienta pro
                                                 smerovani odpovedi. Prideluje
                                                 se pri navazani spojeni
                                                 (onConnect callback). */
    uint8_t   service;                      /**< Service ID (Mode) pro prikaz
                                                 CMD_MANUAL_QUERY (napr. 0x01). */
    uint8_t   pid;                          /**< Cislo PIDu pro prikazy
                                                 CMD_GET_PID, CMD_GET_FREEZE_FRAME
                                                 a CMD_MANUAL_QUERY.
                                                 Rozsah 0x00-0xFF dle normy
                                                 SAE J1979 / ISO 15031-5. */
    uint8_t   pids[WS_MAX_PIDS_PER_REQUEST]; /**< Pole PIDu pro hromadne
                                                 prikazy CMD_GET_PIDS
                                                 a CMD_START_STREAM.
                                                 Platnych je pouze prvnich
                                                 pid_count prvku. */
    uint8_t   diag_pids[WS_MAX_DIAG_PIDS];  /**< PIDy, pro ktere stream vraci
                                                 rozsirena diagnosticka metadata. */
    uint8_t   pid_count;                    /**< Pocet platnych PIDu v poli
                                                 pids[]. Musi byt v rozsahu
                                                 0 az WS_MAX_PIDS_PER_REQUEST. */
    uint8_t   diag_pid_count;               /**< Pocet platnych PIDu v diag_pids[]. */
    ws_stream_mode_t stream_mode;           /**< Rezim streamu (DASH/INSPECTOR). */
    uint16_t  interval_ms;                  /**< Interval periodickeho cteni
                                                 pro CMD_START_STREAM
                                                 v milisekundach. Minimalni
                                                 hodnota je typicky 50 ms
                                                 (omezeno rychlosti CAN
                                                 sbernice). Ignoruje se
                                                 u ostatnich prikazu. */
    bool      hb;                           /**< Heartbeat priznak. */
    char      token[WS_AUTH_TOKEN_MAX];      /**< Autentizacni token pro
                                                 destruktivni prikazy
                                                 (aktualne pouze CMD_CLEAR_DTC).
                                                 Klient ho posila v JSON
                                                 poli "token". U ostatnich
                                                 prikazu zustava prazdny
                                                 (memset 0 v ws_handle_incoming).
                                                 Kontrola proti WS_AUTH_TOKEN
                                                 probiha v handleru prikazu,
                                                 nikoli pri parsovani, aby
                                                 stejny token mohli pouzivat
                                                 i budouci destruktivni prikazy. */
} obd_request_msg_t;

/**
 * Response zprava: smer OBD task --> WebSocket dispatch.
 *
 * Obsahuje hotovy JSON retezec pripraveny k odeslani klientovi pres
 * WebSocket. JSON se serializuje jiz v OBD tasku (Core 1), aby se
 * minimalizoval cas zpracovani na Core 0 — ten pouze precte retezec
 * z fronty a odesle ho pres WebSocket. Diky tomu Core 0 neblokuje
 * WiFi stack zbytecne dlouho.
 *
 * Pole client_id urcuje ciloveho klienta:
 *   - Nenulova hodnota = odpoved pro konkretniho klienta
 *     (napr. klient s ID 3 poslal get_pid, odpoved jde jen jemu)
 *   - Hodnota 0 = broadcast vsem pripojenym klientum
 *     (pouziva se pro streaming odpovedi, kde vsichni klienti
 *     dostavaji stejna data)
 *
 * Velikost struktury je priblizne 2052 B (4 B client_id + 2048 B buffer):
 *   - Je vyrazne vetsi nez request zprava kvuli JSON bufferu.
 *   - FreeRTOS fronta s kapacitou 10 prvku zabere ~20 KB RAM.
 *   - Na ESP32 s 320 KB SRAM je to podstatna cast, proto se kapacita
 *     fronty nenastavuje zbytecne vysoko.
 *
 * Hranicni pripad: pokud by JSON odpoved presahla WS_RESPONSE_JSON_MAX
 * bytu, dojde k orezani (snprintf). V takovem pripade muze byt JSON
 * nevalidni — klient by mel kontrolovat parsovatelnost odpovedi.
 */
typedef struct {
    uint32_t  client_id;                    /**< Cilovy klient — 0 znamena
                                                 broadcast vsem pripojenym
                                                 klientum (pouzivano
                                                 pro streaming). */
    char      json[WS_RESPONSE_JSON_MAX];   /**< Serializovany JSON retezec
                                                 odpovedi, ukonceny nulovym
                                                 znakem ('\0'). Maximalni
                                                 delka vcetne terminatoru
                                                 je WS_RESPONSE_JSON_MAX. */
} obd_response_msg_t;

/* ========================================================================= */
/*  Verejne API funkce                                                       */
/* ========================================================================= */

/**
 * @brief Inicializace modulu ws_handler — predani odkazu na FreeRTOS fronty.
 *
 * Tato funkce musi byt zavolana pred jakymkoliv dalsim volanim funkci
 * z tohoto modulu. Typicky se vola v hlavni funkci setup() po vytvoreni
 * obou FreeRTOS front pomoci xQueueCreate().
 *
 * @param req_queue   Handle FreeRTOS fronty pro pozadavky (smer: WebSocket
 *                    handler --> OBD task). Prvky fronty jsou typu
 *                    obd_request_msg_t.
 * @param resp_queue  Handle FreeRTOS fronty pro odpovedi (smer: OBD task
 *                    --> WebSocket dispatch). Prvky fronty jsou typu
 *                    obd_response_msg_t.
 *
 * @note Pokud je nektery z parametru NULL, chovani modulu je nedefinovane.
 *       Fronty musi byt vytvoreny s dostatecnou kapacitou — doporuceno
 *       alespon 5-10 prvku pro req_queue a 10 prvku pro resp_queue.
 */
void ws_handler_init(QueueHandle_t req_queue, QueueHandle_t resp_queue);

/**
 * @brief Zpracovani prichoziho JSON pozadavku od WebSocket klienta.
 *
 * Tato funkce je volana z WebSocket callbacku na Core 0. Provede
 * nasledujici kroky:
 *   1. Rozparsuje JSON payload a identifikuje prikaz (pole "cmd").
 *   2. Extrahuje parametry (pid, pids, interval) podle typu prikazu.
 *   3. Vytvori strukturu obd_request_msg_t a zaradi ji do request fronty.
 *
 * Funkce NEBLOKUJE — pouziva xQueueSend s nulovym timeoutem. Pokud
 * je fronta plna (napr. OBD task nestaci zpracovavat pozadavky),
 * funkce odesle chybovou odpoved primo do response fronty:
 *   {"cmd":"...","status":"error","error":"QUEUE_FULL"}
 *
 * Pro prikazy, ktere nepotrebuji OBD vrstvu (napr. CMD_PING), se
 * odpoved generuje primo zde bez zarazeni do request fronty —
 * tim se minimalizuje latence pro jednoduche kontrolni zpravy.
 *
 * @param client_id  Jednoznacne ID WebSocket klienta. Pouziva se pro
 *                   smerovani odpovedi zpet spravnemu klientovi.
 * @param payload    JSON retezec od klienta, ukonceny nulovym znakem.
 *                   Priklad: "{\"cmd\":\"get_pid\",\"pid\":12}"
 *                   Retezec se pouze cte, funkce ho nemodifikuje.
 * @param length     Delka payloadu v bytech (bez terminatoru '\0').
 *                   Pouziva se pro validaci — pokud je prilis dlouhy,
 *                   muze byt odmitnut.
 *
 * @note Tato funkce bezi na Core 0 — nesmi obsahovat blokovaci operace
 *       (zadne cekani na CAN odpoved, zadne vTaskDelay na delsi dobu).
 */
void ws_handle_incoming(uint32_t client_id, const char *payload, size_t length);

/**
 * @brief Zpracovani OBD prikazu — volano v OBD tasku na Core 1.
 *
 * Prijme rozparsovanou request zpravu z fronty, zavola prislusnou
 * obd2_* funkci (napr. obd2_read_pid(), obd2_read_dtc()) a sestavi
 * JSON odpoved do response zpravy.
 *
 * Tato funkce MUZE blokovat — ceka na CAN odpoved od ridici jednotky
 * vozidla, coz typicky trva 20-50 ms pro jeden PID, ale muze trvat
 * az nekolik sekund pri chybe komunikace (timeout). Proto bezi
 * vyhradne na Core 1, kde blokovani neovlivni WiFi stack.
 *
 * @param req   Ukazatel na vstupni request zpravu ziskanou z fronty.
 *              Struktura se pouze cte, funkce ji nemodifikuje.
 *              Nesmi byt NULL.
 * @param resp  Ukazatel na vystupni response zpravu. Funkce naplni
 *              pole json[] serializovanym JSON retezcem a nastavi
 *              client_id podle pozadavku. Nesmi byt NULL.
 *              Volajici je zodpovedny za alokaci struktury (typicky
 *              na zasobniku OBD tasku).
 *
 * @note Po navratu z teto funkce by volajici mel zpravu resp
 *       zaradit do response fronty pomoci xQueueSend().
 */
void ws_process_obd_command(const obd_request_msg_t *req,
                            obd_response_msg_t *resp);

/**
 * @brief Zjisti, zda je OBD vrstva uspesne inicializovana.
 *
 * Vraci true po uspesnem zpracovani prikazu CMD_INIT (tj. po
 * uspesnem volani obd2_init() a nacteni podporovanych PIDu).
 *
 * Pouziva se jako strazni podminka (guard) pred ostatnimi OBD
 * prikazy — pokud neni OBD inicializovano, prikazy jako
 * CMD_GET_PID vraceji chybu "OBD_NOT_INITIALIZED".
 *
 * @return true   OBD vrstva je inicializovana a pripravena.
 * @return false  OBD vrstva jeste nebyla inicializovana nebo
 *                inicializace selhala.
 */
bool ws_is_obd_initialized(void);

/**
 * @brief Zjisti, zda je aktivni streaming PIDu.
 *
 * Pouziva se v hlavni smycce OBD tasku pro rozhodnuti mezi dvema
 * rezimy cinnosti:
 *   - Streaming aktivni: task cyklicky cte PIDy v danem intervalu
 *     a neodbavuje frontu pozadavku (krome CMD_STOP_STREAM).
 *   - Streaming neaktivni: task blokovane ceka na novy pozadavek
 *     z fronty pomoci xQueueReceive() s nekonecnym timeoutem.
 *
 * @return true   Streaming je aktivni (byl prijat CMD_START_STREAM
 *                a jeste nedosel CMD_STOP_STREAM).
 * @return false  Streaming neni aktivni.
 */
bool ws_is_streaming(void);

/**
 * @brief Vrati nastaveny interval streamu v milisekundach.
 *
 * Interval urcuje, jak casto OBD task cte PIDy behem aktivniho
 * streamingu. Hodnota je nastavena prikazem CMD_START_STREAM.
 *
 * @return Interval v ms (napr. 200 = cteni kazdych 200 ms).
 *         Pokud streaming neni aktivni, vraci 0.
 *
 * @note Skutecny interval muze byt delsi nez nastaveny, pokud
 *       samotne cteni vsech PIDu trva dele nez zadany interval
 *       (napr. 8 PIDu * 50 ms = 400 ms > nastaveny interval 200 ms).
 */
uint16_t ws_get_stream_interval(void);

/**
 * @brief Provede jeden cyklus streamu — precte vsechny nakonfigurovane
 *        PIDy a sestavi JSON odpoved.
 *
 * Tato funkce je volana z OBD tasku (Core 1) v aktivnim streaming
 * rezimu. V kazdem cyklu:
 *   1. Precte vsechny PIDy zadane v CMD_START_STREAM.
 *   2. Sestavi kompaktni JSON odpoved pro broadcast vsem klientum.
 *   3. Nastavi client_id na 0 (broadcast).
 *
 * Format JSON odpovedi je zamerene zkraceny pro minimalni objem dat
 * (streaming generuje mnoho zprav za sekundu):
 *   {"cmd":"stream","d":{"12":875.25,"13":45},"ts":12345}
 *   -- "d" = slovnik PID:hodnota (klice jsou cisla PIDu jako retezce)
 *   -- "ts" = casove razitko v milisekundach od startu ESP32
 *              (pouziva se esp_timer_get_time() / 1000)
 *
 * Hranicni pripad: pokud nektory PID selze (timeout), jeho hodnota
 * se ve slovniku "d" vynecha nebo se nastavi na null.
 *
 * @param resp  Ukazatel na vystupni response zpravu. Funkce naplni
 *              json[] a nastavi client_id=0 (broadcast). Nesmi byt NULL.
 *
 * @note Volajici je zodpovedny za odeslani response zpravy do
 *       response fronty po navratu z teto funkce.
 */
void ws_process_stream_tick(obd_response_msg_t *resp);

/* ========================================================================= */
/*  Interni typ pro konfiguraci streamu (sdileny mezi .cpp soubory)          */
/* ========================================================================= */

/**
 * @brief Konfigurace streamovani PIDu.
 *
 * Pouziva se jako volatile promenna sdilena mezi ws_handler.cpp
 * (OBD task na Core 1 cte) a ws_commands.cpp (command handler zapisuje).
 * Typ je v hlavicce aby obe .cpp jednotky videly stejnou definici.
 */
typedef struct {
    bool     active;                            /**< Stream aktivni? */
    ws_stream_mode_t mode;                      /**< Rezim streamu */
    uint8_t  pids[WS_MAX_PIDS_PER_REQUEST];     /**< PIDy ke cteni */
    uint8_t  diag_pids[WS_MAX_DIAG_PIDS];       /**< PIDy s diagnostikou */
    uint8_t  pid_count;                         /**< Pocet PIDu */
    uint8_t  diag_pid_count;                    /**< Pocet diagnostickych PIDu */
    uint16_t interval_ms;                       /**< Interval mezi cykly (ms) */
} ws_stream_cfg_t;

#ifdef __cplusplus
}
#endif

#endif /* WS_HANDLER_H */
