/**
 * @file obdII_wifi.ino
 * @brief Hlavni Arduino sketch pro ESP32 OBD-II WiFi diagnosticky nastroj
 *
 * Tento soubor predstavuje vstupni bod cele aplikace a propojuje vsechny
 * komunikacni a diagnosticke vrstvy dohromady:
 *
 *   - WiFi Access Point (ESP32 funguje jako samostatny hotspot, tj. pristupovy
 *     bod, ke kteremu se pripoji klientske zarizeni — telefon, tablet,
 * notebook)
 *   - ESPAsyncWebServer + AsyncWebSocket pro obousmernou realtime komunikaci
 *     mezi webovym prohlizecem klienta a ESP32 (pres JSON zpravy)
 *   - FreeRTOS OBD task na jadre Core 1, ktery obsluhuje veskerou CAN/OBD-II
 *     komunikaci s ridici jednotkou vozidla (ECU)
 *   - Dve FreeRTOS fronty (queues) pro bezpecny prenos dat mezi tasky —
 *     request fronta (pozadavky od klienta) a response fronta (odpovedi z ECU)
 *
 * Architektura dvou jader ESP32:
 * ──────────────────────────────
 *   Core 0 (PRO_CPU): WiFi stack, TCP/IP, HTTP server, WebSocket handler,
 *                      response dispatch task. Toto jadro je vyhrazeno pro
 *                      sitovou komunikaci a nesmi byt blokovano dlouhymi
 *                      operacemi (jinak dojde k watchdog resetu).
 *
 *   Core 1 (APP_CPU): OBD diagnosticky task, ktery vola obd2_* funkce.
 *                      Tyto funkce blokovane cekaji na CAN odpovedi (az 250
 * ms), proto musi bezet oddelene od WiFi stacku.
 *
 * DULEZITE UPOZORNENI:
 *   WebSocket callback bezi na Core 0 — nikdy primo nevolejte obd2_* funkce
 *   z tohoto callbacku! Vsechny OBD pozadavky se posilaji pres request frontu
 *   a odpovedi se vraceji zpet pres response frontu. Porusteni tohoto pravidla
 *   zpusobi zablokovani WiFi stacku a nasledny watchdog reset (WDT timeout).
 *
 * Tok dat:
 *   Klient (prohlizec) → WebSocket → request fronta → OBD task (Core 1)
 *     → CAN sbernice → ECU → CAN sbernice → OBD task → response fronta
 *     → response dispatch task (Core 0) → WebSocket → Klient
 *
 * Zavislosti (Arduino Library Manager / PlatformIO):
 *   - ESPAsyncWebServer (https://github.com/ESP32Async/ESPAsyncWebServer)
 *     Asynchronni HTTP server, ktery neblokuje hlavni smycku.
 *   - AsyncTCP (https://github.com/ESP32Async/AsyncTCP)
 *     TCP vrstva pro ESPAsyncWebServer, nutna na ESP32.
 *   - ArduinoJson v7.x (https://arduinojson.org/)
 *     Serializace a deserializace JSON zprav pro WebSocket komunikaci.
 *
 * @author Ales Pouzar
 */

#include "dashboard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ws_handler.h"
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

extern "C" {
#include "obd2.h"
}

/* ========================================================================= */
/*  Konfigurace                                                              */
/* ========================================================================= */

/**
 * GPIO piny pro CAN transceiver (SN65HVD230 nebo kompatibilni).
 *
 * ESP32 nema vestaveny CAN fyzicky budic — pouziva se externi transceiver
 * SN65HVD230, ktery prevadi logicke urovne (TX/RX) na diferencni signaly
 * CAN_H a CAN_L na sbernici.
 *
 * Mapovani pinu:
 *   - CAN_TX_PIN (GPIO 12): ESP32 → transceiver → CAN_H/CAN_L
 *   - CAN_RX_PIN (GPIO 14): CAN_H/CAN_L → transceiver → ESP32
 *
 * POZOR: Tyto piny musi odpovidat fyzickemu zapojeni na DPS. Pokud pouzivate
 * jinou desku nez puvodni navrh, overite spravne GPIO cisla. GPIO 12 je take
 * bootstrap pin — pri startu musi byt LOW, jinak ESP32 nenastartuje ze SPI
 * flash. Transceiver SN65HVD230 drzi TX vstup v definovanem stavu, takze to
 * neni problem.
 */
#define CAN_TX_PIN 12
#define CAN_RX_PIN 14

/**
 * Rychlost CAN sbernice v bitech za sekundu.
 *
 * OBD-II standard (ISO 15765-4) vyzaduje 500 kbit/s pro vsechna osobni
 * vozidla vyrobena od roku 2008+ (v EU od 2001 pro benzinova, 2004 pro
 * dieslova). Nektere starsi vozy mohou pouzivat 250 kbit/s — v takovem
 * pripade zmente tuto hodnotu.
 *
 * Priklady:
 *   500000 — standardni OBD-II (vetsina modernich vozidel)
 *   250000 — nektere starsi vozy, uzitková vozidla, nakladni automobily
 *   125000 — zemedelska technika, lodni motory (J1939)
 */
#define CAN_BAUDRATE 500000

/**
 * Nastaveni WiFi Access Pointu (pristupoveho bodu).
 *
 * ESP32 pracuje v rezimu AP (Access Point), coz znamena, ze samo vytvari
 * WiFi sit. Klient (telefon, notebook) se pripoji primo k ESP32 bez
 * nutnosti externi infrastruktury (routeru, internetu).
 *
 * Alternativou by byl rezim STA (Station), kde se ESP32 pripoji k existujici
 * WiFi siti. Rezim AP je vhodnejsi pro diagnostiku v terénu (garaz, servis),
 * kde nemusi byt dostupna zadna WiFi sit.
 *
 * Parametry:
 *   WIFI_SSID        — nazev site, ktery se zobrazi v seznamu WiFi siti
 *   WIFI_PASSWORD     — heslo (WPA2, minimalne 8 znaku)
 *   WIFI_CHANNEL      — WiFi kanal (1-13). Kanal 6 je zvolen jako kompromis:
 *                        kanaly 1, 6, 11 se navzajem neruci (neprekryvaji se).
 *                        Kanal 6 je stredni volba — pokud je v okoli hodne
 *                        siti na kanalu 1 nebo 11, kanal 6 bude mene ruseny.
 *   WIFI_MAX_CLIENTS  — maximalni pocet soucasne pripojenych klientu.
 *                        Omezeno na 2, protoze ESP32 ma limitovane prostredky
 *                        (RAM, TCP sockety). Kazdy klient spotrebuje cca 2-4 KB
 *                        RAM pro TCP buffer + WebSocket frame buffer.
 */
#define WIFI_SSID "OBD2-Diagnostics"
#define WIFI_PASSWORD "obd2pass123"
#define WIFI_CHANNEL 6
#define WIFI_MAX_CLIENTS 2

/**
 * Port HTTP serveru.
 *
 * Standardni port 80 umoznuje pristup bez zadavani portu v URL
 * (staci http://192.168.4.1/ misto http://192.168.4.1:80/).
 */
#define HTTP_PORT 80

/**
 * Velikosti FreeRTOS front pro komunikaci mezi tasky.
 *
 * Fronty slouzi jako mezipameti (buffery) mezi producentem a konzumentem:
 *
 * OBD_REQUEST_QUEUE_SIZE (5):
 *   Fronta pro pozadavky od WebSocket klienta smerem k OBD tasku.
 *   Kapacita 5 polozek je dostatecna, protoze klient typicky posila
 *   prikazy po jednom a ceka na odpoved. I pri rychlem klikani v GUI
 *   nevznikne vic nez 2-3 pozadavky najednou. Kazda polozka zabira
 *   sizeof(obd_request_msg_t) bajtu v RAM.
 *   Priklad: klient posle "init", pak "read_pid 0x0C" — oba se vejdou.
 *
 * OBD_RESPONSE_QUEUE_SIZE (10):
 *   Fronta pro odpovedi z OBD tasku zpet ke klientovi. Vetsi nez request
 *   fronta ze dvou duvodu:
 *   a) Streaming mod generuje odpovedi prubezne (napr. kazdy 100 ms),
 *      a response dispatch task je nestihne okamzite odeslat.
 *   b) Multi-PID prikaz (napr. cteni 5 PIDu najednou) generuje jednu
 *      velkou odpoved, ale zpracovani trva dele nez prijeti pozadavku.
 *   Kazda polozka obsahuje JSON retezec (az 512 B) + metadata.
 *
 * POZOR na pametovy dopad: celkova spotreba = velikost_fronty *
 * sizeof(polozka). Pri OBD_RESPONSE_QUEUE_SIZE=10 a
 * sizeof(obd_response_msg_t)~520 B to je cca 5.2 KB RAM jen pro response
 * frontu.
 */
#define OBD_REQUEST_QUEUE_SIZE 5
#define OBD_RESPONSE_QUEUE_SIZE 10

/**
 * Velikosti zasobniku (stacku) pro FreeRTOS tasky v bajtech.
 *
 * OBD_TASK_STACK_SIZE (8192 B = 8 KB):
 *   OBD task pouziva relativne velke lokalni promenne (napr. resp[256]
 *   pro odpoved, req[3] pro pozadavek) a hlubsi call stack (obd2_*
 *   funkce volaji isotp_* funkce, ktere volaji twai_* funkce). 8 KB
 *   je dostatecna rezerva. Pri nedostatku stacku dojde k stack overflow
 *   a nahodnemu padani (guru meditation error).
 *
 * RESPONSE_TASK_STACK_SIZE (6144 B = 6 KB):
 *   Response dispatch task je jednodussi — pouze cte z fronty a odesila
 *   pres WebSocket. Mensi stack staci, ale musi pojmout ArduinoJson
 *   operace a WebSocket frame alokace.
 */
#define OBD_TASK_STACK_SIZE 8192
#define RESPONSE_TASK_STACK_SIZE 6144

/* ========================================================================= */
/*  Globalni objekty                                                         */
/* ========================================================================= */

/**
 * HTTP server a WebSocket endpoint.
 *
 * AsyncWebServer naslouchá na portu HTTP_PORT (80) a obsluhuje jak
 * klasicke HTTP pozadavky (GET /), tak WebSocket spojeni na ceste /ws.
 * WebSocket je preferovany pro diagnostiku, protoze umoznuje obousmernou
 * komunikaci bez nutnosti neustaleho dotazovani (polling).
 */
AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws("/ws");

/**
 * FreeRTOS fronty pro komunikaci mezi WebSocket handlerem a OBD taskem.
 *
 * obd_request_queue:  smer WebSocket (Core 0) → OBD task (Core 1)
 * obd_response_queue: smer OBD task (Core 1) → response dispatch (Core 0)
 *
 * Inicializuji se na NULL a vytvari se v setup() pomoci xQueueCreate().
 * Pokud se vytvoreni nepodari (nedostatek RAM), program se zastavi.
 */
QueueHandle_t obd_request_queue = NULL;
QueueHandle_t obd_response_queue = NULL;

/**
 * Handle (ukazatele) na FreeRTOS tasky.
 *
 * Pouzivaji se pro pripadnou kontrolu stavu tasku, napr. eTaskGetState(),
 * nebo pro zruseni tasku pomoci vTaskDelete(). V soucasne implementaci
 * se tasky nikdy nerusi, ale handle je uzitecny pro debugging.
 */
TaskHandle_t obd_task_handle = NULL;
TaskHandle_t response_task_handle = NULL;

/**
 * Priznak (flag) indikujici, zda byla OBD vrstva uspesne inicializovana.
 *
 * Nastavi se na true az po uspesnem volani obd2_init() (ktere probehne
 * na prikaz "init" od klienta, NE automaticky pri startu).
 *
 * Deklarovano jako volatile, protoze k nemu pristupuji oba tasky
 * (OBD task na Core 1 ho nastavuje, response dispatch na Core 0 ho cte).
 * Pro jednoduchy bool flag staci volatile bez mutexu.
 */
static volatile bool obd_initialized = false;

/*
 * Dashboard HTML je ulozen v souboru dashboard.h jako PROGMEM pole
 * (dashboard_html[]). PROGMEM zajisti, ze HTML obsah je ulozen ve flash
 * pameti a nezabira vzacnou RAM. Pri pozadavku na "/" se odesle primo
 * z flash pomoci send_P().
 */

/* ========================================================================= */
/*  WiFi Access Point                                                        */
/* ========================================================================= */

/**
 * @brief Inicializace WiFi v rezimu Access Point (pristupovy bod).
 *
 * ESP32 vytvori vlastni bezdrátovou sit s nazvem definovanym v WIFI_SSID.
 * Klientska zarizeni (telefon, notebook, tablet) se pripoji k teto siti
 * a pristoupi na adresu 192.168.4.1 pro webove diagnosticke rozhrani.
 *
 * Rezim AP vs. STA:
 *   - AP (Access Point): ESP32 JE sit — klienti se pripojuji primo.
 *     Vyhoda: funguje vsude, bez zavislosti na externi infrastrukture.
 *     Nevyhoda: neni pristup k internetu.
 *   - STA (Station): ESP32 se pripoji k existujici WiFi siti.
 *     Vyhoda: pristup k internetu, integrace do domaci site.
 *     Nevyhoda: vyzaduje znalost SSID/hesla existujici site.
 *
 * Pro diagnostiku v terenních podminkach (garaz, servis, krajnice)
 * je rezim AP jednoznacne vhodnejsi.
 *
 * Staticka IP konfigurace (192.168.4.1):
 *   - AP IP adresa: 192.168.4.1 — adresa samotneho ESP32
 *   - Gateway: 192.168.4.1 — ESP32 je zaroven branou (neni kam smerovat dal)
 *   - Maska podsite: 255.255.255.0 — podsit /24, az 254 klientu (omezeno
 *     na WIFI_MAX_CLIENTS=2 na urovni WiFi vrstvy)
 *
 * Kanal 6 je zvolen jako stredni neprekryvajici se kanal (1, 6, 11 jsou
 * tri kanaly v pasmu 2.4 GHz, ktere se navzajem neruci).
 * Maximalne 2 klienti — ESP32 ma omezene prostredky pro TCP spojeni.
 * SSID neni skryte (broadcast parametr = 0, tj. false → viditelne).
 */
static void wifi_init_ap(void) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, 0, WIFI_MAX_CLIENTS);
  WiFi.softAPConfig(
      IPAddress(192, 168, 4, 1),  /* IP adresa pristupoveho bodu */
      IPAddress(192, 168, 4, 1),  /* Vychozi brana (gateway) — ESP32 samo */
      IPAddress(255, 255, 255, 0) /* Maska podsite — /24, rozsah .1 az .254 */
  );

  Serial.printf("[WIFI ] AP started: SSID=\"%s\" IP=%s\n", WIFI_SSID,
                WiFi.softAPIP().toString().c_str());
}

/* ========================================================================= */
/*  WebSocket handler                                                        */
/* ========================================================================= */

/**
 * @brief Callback funkce pro vsechny WebSocket udalosti (pripojeni, data,
 * chyby).
 *
 * Tato funkce je volana asynchronne knihovnou ESPAsyncWebServer vzdy, kdyz
 * nastane udalost na WebSocket endpointu /ws. Bezi na Core 0 (WiFi/TCP task),
 * proto NIKDY nesmime primo volat obd2_* funkce — ty blokovane cekaji na
 * CAN odpoved a zablokovaly by WiFi stack.
 *
 * Typy udalosti a jejich zpracovani:
 *
 *   WS_EVT_CONNECT — novy klient navázal WebSocket spojeni.
 *     Logujeme ID klienta a jeho IP adresu. Kazdy klient dostane unikatni
 *     ID (uint32_t), ktere se pouziva pro smerovani odpovedi zpet.
 *     Priklad: "Klient #1 pripojen z 192.168.4.2"
 *
 *   WS_EVT_DISCONNECT — klient ukoncil spojeni (zavreni prohlizece,
 *     ztrata WiFi signalu, timeout). Logujeme pro diagnostiku.
 *     POZOR: pokud klient mel aktivni streaming, je nutne ho zastavit
 *     (resi ws_handler modul).
 *
 *   WS_EVT_DATA — klient poslal data (JSON prikaz).
 *     Prijimame POUZE kompletni textove zpravy (ne binarne, ne fragmentovane).
 *     Fragmentace WebSocket framu:
 *       - info->final  = true pokud je toto posledni (nebo jediny) fragment
 *       - info->index  = offset v ramci zpravy (0 = zacatek prvniho fragmentu)
 *       - info->len    = celkova delka zpravy
 *       - info->opcode = WS_TEXT (0x01) pro textove, WS_BINARY (0x02) pro
 * binární Fragmentovane zpravy ignorujeme — vsechny nase JSON prikazy jsou
 * kratke (typicky < 256 bajtu) a vejdou se do jednoho WebSocket framu. Priklad
 * prichoziho JSON: {"cmd":"read_pid","service":1,"pid":"0x0C"}
 *
 *   WS_EVT_ERROR — chyba WebSocket protokolu (napr. neplatny frame,
 *     preteceni bufferu). Logujeme ID klienta.
 *
 * @param server  Ukazatel na WebSocket objekt (ws)
 * @param client  Ukazatel na konkretniho klienta, ktery udalost vyvolal
 * @param type    Typ udalosti (WS_EVT_CONNECT, WS_EVT_DISCONNECT, atd.)
 * @param arg     Doplnkove informace — pro WS_EVT_DATA obsahuje AwsFrameInfo
 * @param data    Surova data zpravy (pro WS_EVT_DATA)
 * @param len     Delka dat v bajtech
 */
static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {

  case WS_EVT_CONNECT:
    Serial.printf("[WS   ] Klient #%u pripojen z %s\n", client->id(),
                  client->remoteIP().toString().c_str());
    break;

  case WS_EVT_DISCONNECT:
    Serial.printf("[WS   ] Klient #%u odpojen\n", client->id());
    break;

  case WS_EVT_DATA: {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;

    /*
     * Kontrola, ze jde o kompletni textovou zpravu:
     * - info->final == true: je to posledni (nebo jediny) fragment
     * - info->index == 0: jsme na zacatku zpravy (zadny predchozi fragment)
     * - info->len == len: celkova delka odpovida prijatym datum (vse v jednom
     * kusu)
     * - info->opcode == WS_TEXT: textovy frame (ne binarni)
     *
     * Pokud by klient poslal velkou zpravu (napr. > MTU), WebSocket ji
     * rozlozi na vice fragmentu. My takove zpravy ignorujeme, protoze
     * nase JSON prikazy jsou vzdy kratke.
     */
    if (info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT) {
      /* Null-terminate data pro bezpecne parsovani jako C retezec */
      data[len] = '\0';

      Serial.printf("[WS   ] Klient #%u: %s\n", client->id(), (char *)data);

      /*
       * Parsovani JSON zpravy a zarazeni do request fronty.
       * ws_handle_incoming() rozlozi JSON, vytvori obd_request_msg_t
       * strukturu a vlozi ji do obd_request_queue. Pokud je fronta
       * plna, prikaz se zahodi a klientovi se odesle chybova odpoved.
       */
      ws_handle_incoming(client->id(), (const char *)data, len);
    }
    break;
  }

  case WS_EVT_ERROR:
    Serial.printf("[WS   ] Klient #%u chyba\n", client->id());
    break;
  }
}

/* ========================================================================= */
/*  OBD Diagnostic Task (Core 1)                                             */
/* ========================================================================= */

/**
 * @brief Hlavni smycka OBD diagnostickeho tasku bezici na jadre Core 1.
 *
 * Tento task je srdcem cele diagnosticke aplikace. Bezi na Core 1 (APP_CPU),
 * cimz je fyzicky oddelen od WiFi stacku na Core 0 (PRO_CPU).
 *
 * Proc oddeleny task na Core 1:
 *   1. Funkce obd2_* blokovane cekaji na CAN odpoved pomoci twai_receive()
 *      s timeoutem typicky 250 ms. Kdyby bezely na Core 0, zablokovaly by
 *      WiFi/TCP stack a doslo by k watchdog resetu (WDT timeout po 5 s).
 *   2. Core 1 nema WiFi preruseni, takze CAN timing je presnejsi a nedochazi
 *      k nepredvidatelnym zpozděnim zpusobenym WiFi obsluhou.
 *   3. Oddělení umoznuje paralelni beh — zatimco OBD task ceka na CAN
 *      odpoved, WiFi stack na Core 0 muze obsluhovat dalsi klienty.
 *
 * Dva rezimy provozu (dual-mode loop):
 *
 *   1. BEZ STREAMU (normalni rezim):
 *      Task ceka nekonecne (portMAX_DELAY) na prikaz v request fronte.
 *      Behem cekani task "spi" — nespotrebovava CPU cyklus, scheduler
 *      ho probudi az kdyz se objevi polozka ve fronte. To je idealni
 *      pro setréni energie a CPU casu.
 *      Priklad: klient posle "read_pid 0x0C" → task se probudi,
 *      precte otacky motoru a posle odpoved.
 *
 *   2. SE STREAMEM (kontinualni cteni):
 *      Task pouziva kratky timeout (5 ms) pro kontrolu prikazu ve fronte.
 *      Pokud zadny prikaz neprisel, provede jeden "stream tick" — precte
 *      vsechny nastavene PIDy a odesle vysledky. Pak pocka
 * ws_get_stream_interval() milisekund (konfigurovano klientem, typicky 100-500
 * ms). 5 ms timeout je kompromis: dostatecne kratky, aby prikazy (napr. "stop")
 *      byly zpracovany rychle, ale dostatecne dlouhy, aby nedochazelo k
 *      busy-waiting (neustale dotazovani fronty bez uspani).
 *
 * Velikost zasobniku: 8192 B — obd2 funkce pouzivaji staticke buffery,
 * ale lokalni promenne (resp[256], req[3]) a vnoreni volani (obd_task →
 * ws_process_obd_command → obd2_read_pid → isotp_send_receive → twai_transmit)
 * potrebuji dostatek mista na zasobniku.
 *
 * @param param Parametr predany pri vytvoreni tasku (nepouzit, NULL)
 */
static void obd_task(void *param) {
  obd_request_msg_t req;
  obd_response_msg_t resp;

  Serial.println("[OBD  ] Task spusten na Core 1");

  while (1) {
    /*
     * Dynamicky timeout podle aktualniho rezimu:
     *
     * - ws_is_streaming() == false → portMAX_DELAY (nekonecne cekani).
     *   Task spi a nespotrebovava CPU, dokud neprijde prikaz.
     *
     * - ws_is_streaming() == true → 5 ms timeout.
     *   Kratky timeout umoznuje rychle reagovat na prikazy (napr.
     * "stop_stream") i behem aktivniho streamovani. Po vyprseni timeoutu se
     * provede jeden stream cyklus (cteni vsech nakonfigurovanych PIDu). Prikazy
     * od klienta maji prednost pred streamem — pokud je prikaz ve fronte,
     * zpracuje se ihned a stream se pozdrzi o jeden cyklus.
     */
    TickType_t timeout = ws_is_streaming() ? pdMS_TO_TICKS(5) : portMAX_DELAY;

    if (xQueueReceive(obd_request_queue, &req, timeout) == pdTRUE) {
      Serial.printf("[OBD  ] Prikaz: cmd=%d od klienta #%u\n", req.cmd,
                    req.client_id);

      /* Zpracovani prikazu — ws_process_obd_command() zavola prislusnou
       * obd2_* funkci, sestavi JSON odpoved a ulozi ji do resp.json[].
       * Napriklad pro cmd=READ_PID zavola obd2_read_pid() a vysledek
       * serializuje jako {"type":"pid_response","pid":"0x0C","value":3500}. */
      ws_process_obd_command(&req, &resp);

      if (xQueueSend(obd_response_queue, &resp, pdMS_TO_TICKS(500)) != pdTRUE) {
        /* Fronta je plna — odpoved se zahodi. To se muze stat
         * pokud response dispatch task nestíhá odesilat (napr.
         * pri spatnem WiFi signálu). Timeout 500 ms da sanci
         * na uvolneni mista, ale pokud ani to nestaci, logjeme
         * varovani a pokracujeme. */
        Serial.println("[OBD  ] WARN: response queue plna, odpoved zahozena");
      }
    }

    /*
     * Streaming — cyklicke cteni PIDu a broadcast vysledku vsem klientum.
     *
     * ws_process_stream_tick() precte dalsi PID v poradi (round-robin)
     * a ulozi JSON odpoved do resp.json[]. Pokud je resp.json[0] == '\0',
     * znamena to, ze zadny PID nebyl ke cteni (prazdny stream seznam).
     *
     * Prodleva ws_get_stream_interval() mezi cykly je nastavitelna
     * klientem (typicky 100-500 ms). Kratsi interval = casteji aktualizace,
     * ale vetsi zatez CAN sbernice a CPU.
     */
    if (ws_is_streaming()) {
      ws_process_stream_tick(&resp);
      if (resp.json[0] != '\0') {
        if (xQueueSend(obd_response_queue, &resp, pdMS_TO_TICKS(100)) !=
            pdTRUE) {
          /* Stream odpoved zahozena — mene kriticke nez
           * odpoved na primý prikaz, proto kratsi timeout (100 ms). */
          Serial.println("[OBD  ] WARN: stream response zahozena");
        }
      }
      /* Prodleva mezi stream cykly — konfigurovana klientem.
       * vTaskDelay() uvolni CPU pro jine tasky behem cekani. */
      vTaskDelay(pdMS_TO_TICKS(ws_get_stream_interval()));
    }
  }
}

/**
 * @brief Task pro odesilani odpovedi z response fronty klientum pres WebSocket.
 *
 * Tento task bezi na Core 0 (PRO_CPU), coz je stejne jadro jako WiFi stack.
 * To je nutne, protoze pristupuje k WebSocket objektu (ws), ktery NENI
 * thread-safe — volani ws.textAll() nebo client->text() z jineho jadra
 * by mohlo zpusobit poruseni dat (race condition) nebo pad.
 *
 * Princip fungovani:
 *   1. Task periodicky kontroluje response frontu (timeout 50 ms)
 *   2. Pokud je ve fronte odpoved, urcí ciloveho klienta:
 *      - client_id == 0 → broadcast vsem pripojenym klientum (pouzivano
 *        pro stream data, kde vsichni klienti dostavaji stejna data)
 *      - client_id > 0 → odeslani konkretnimu klientovi podle jeho ID
 *        (pouzivano pro odpovedi na jednotlive prikazy)
 *   3. Pred odeslanim overi, ze klient je stale pripojen (WS_CONNECTED).
 *      Pokud se klient mezi tim odpojil, odpoved se zahodi s logem.
 *   4. Po kazdem cyklu (at uz byla odpoved nebo ne) provede cleanup
 *      odpojenych WebSocket klientu.
 *
 * Proc separatni task a ne primo v Arduino loop():
 *   - loop() bezi na Core 1 defaultne v Arduino frameworku pro ESP32.
 *   - Tento task MUSI byt na Core 0, kde bezi WiFi/TCP, protoze
 *     WebSocket objekt neni thread-safe.
 *   - Kratky timeout (50 ms) zajistuje responsivitu (odpoved se odesle
 *     do 50 ms od vlozeni do fronty) bez busy-waitingu (task spi vetsinu
 *     casu a nespotrebovava CPU zbytecne).
 *
 * Priorita 3: vyssi nez loop() task (priorita 1), ale nizsi nez
 * interni WiFi eventy (priorita 19-23). Tim je zajisteno, ze WiFi
 * stack ma vzdy prednost, ale odpovedi se odesilaji driv nez
 * nekriticke operace v loop().
 *
 * @param param Parametr predany pri vytvoreni tasku (nepouzit, NULL)
 */
static void response_dispatch_task(void *param) {
  obd_response_msg_t resp;

  Serial.println("[RESP ] Dispatch task spusten na Core 0");

  while (1) {
    /*
     * Cekame maximalne 50 ms na odpoved ve fronte.
     * Pokud odpoved neprijde (timeout), pokracujeme na cleanup
     * a zacneme cekat znovu. 50 ms je kompromis:
     *   - Kratsi (napr. 10 ms) by znamenal castejsi probouzeni a zbytecne
     *     prepínani kontextu.
     *   - Delsi (napr. 500 ms) by znamenal vetsi latenci odpovedi.
     */
    if (xQueueReceive(obd_response_queue, &resp, pdMS_TO_TICKS(50)) == pdTRUE) {

      if (resp.client_id == 0) {
        /* client_id == 0 znamena broadcast — odpoved se odesle
         * vsem aktualne pripojenym klientum. Pouzivano predevsim
         * pro stream data (napr. otacky, teplota, rychlost),
         * kde vsichni klienti dostavaji stejna data soucasne. */
        ws.textAll(resp.json);
      } else {
        /* Odeslani konkretnimu klientovi podle jeho unikatniho ID.
         * Nejprve overime, ze klient stale existuje a je pripojen.
         * Muze se stat, ze klient se odpojil mezi vlozenim odpovedi
         * do fronty a jejim zpracovanim (napr. zavreni prohlizece). */
        AsyncWebSocketClient *client = ws.client(resp.client_id);
        if (client && client->status() == WS_CONNECTED) {
          client->text(resp.json);
        } else {
          Serial.printf("[RESP ] Klient #%u uz neni pripojen\n",
                        resp.client_id);
        }
      }
    }

    /*
     * Periodicky cleanup (uklid) odpojenych WebSocket klientu.
     *
     * ESPAsyncWebServer doporucuje volat ws.cleanupClients() pravidelne.
     * Bez tohoto volani se hromadi "zombie" spojeni — klient se odpojil
     * na TCP urovni, ale WebSocket objekt stale drzi alokovanou pamet
     * a struktury. Casem by doslo k vycerpani pameti.
     *
     * Volame po kazdem cyklu (kazdych ~50 ms), coz je dostatecne caste.
     */
    ws.cleanupClients();
  }
}

/* ========================================================================= */
/*  Arduino setup() a loop()                                                 */
/* ========================================================================= */

/**
 * @brief Hlavni inicializacni funkce Arduino frameworku — spusti se jednou pri
 * startu.
 *
 * Poradi inicializace je dulezite a nesmi se menit:
 *   1. Stabilizacni prodleva (delay) — cekani na ustáleni napajeni
 *   2. Seriova konzole — pro debugging a diagnosticky vystup
 *   3. WiFi Access Point — vytvoreni bezdrátové site
 *   4. FreeRTOS fronty — komunikacni kanaly mezi tasky
 *   5. WebSocket a HTTP server — webove rozhrani pro klienty
 *   6. FreeRTOS tasky — spusteni OBD a response dispatch tasku
 *
 * POZOR: OBD vrstva (obd2_init) se NEINICIALIZUJE v setup()!
 * Inicializace probiha az na explicitni prikaz "init" od klienta.
 * Duvod: pri startu ESP32 nemusi byt zapaleni zapnute a CAN sbernice
 * nemusi byt aktivni. Volani obd2_init() bez aktivni sbernice by selhalo
 * nebo by zpusobilo stav BUS_OFF (trvale odpojeni od sbernice).
 */
void setup() {
  /*
   * 1. Stabilizacni prodleva
   *
   * Prodleva 1000 ms (1 sekunda) na zacatku je kriticka pri napajeni
   * z OBD-II konektoru vozidla:
   *   - Pin 16 OBD konektoru dodava +12V z baterie vozidla
   *   - DC-DC menic (napr. LM2596) snizuje na 5V/3.3V pro ESP32
   *   - Pri pripojeni konektoru dochazi k napetovemu prechodovemu jevu
   *     (voltage spike/dip), ktery muze trvat az stovky milisekund
   *   - Bez prodlevy muze ESP32 nastartovat s nestabilnim napetim,
   *     coz zpusobi chybu CRC flash pameti ("csum err") a boot-loop
   *     (neustale restartovani)
   *
   * 1 sekunda je konzervativni hodnota — vetsina DC-DC menicu se
   * ustali do 100-200 ms, ale ponechavame rezervu pro extremni pripady
   * (slaba baterie, spatny kontakt v OBD konektoru).
   */
  delay(1000);

  /* 2. Inicializace seriove konzole pro debugging.
   * Rychlost 115200 baud je standardni pro ESP32 (shodna s boot logem). */
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  } /* Cekej max 3 sekundy na pripravenost Serial portu (USB CDC) */
  Serial.println("\n========================================");
  Serial.println("  ESP32 OBD-II Diagnostic Tool v1.0");
  Serial.println("========================================");
  Serial.printf("  Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("  CPU freq:  %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("  Chip:      %s rev %d\n", ESP.getChipModel(),
                ESP.getChipRevision());
  Serial.println("----------------------------------------");

  /* 3. Vytvoreni WiFi pristupoveho bodu */
  wifi_init_ap();

  /*
   * 4. Vytvoreni FreeRTOS front pro komunikaci mezi tasky
   *
   * xQueueCreate() alokuje pamet z heap pro frontu dane velikosti.
   * Kazda polozka ve fronte zabira sizeof(typ) bajtu.
   *
   * Request fronta: WebSocket handler (Core 0) → OBD task (Core 1)
   *   Kapacita 5 polozek — klient posila prikazy po jednom a typicky
   *   ceka na odpoved pred odeslanim dalsiho. I pri rychlem klikani
   *   v GUI nevznikne vic nez par pozadavku najednou. Pokud by fronta
   *   byla plna (5 nevyrizených prikazu), dalsi prikaz se odmitne
   *   a klientovi se odesle chybova zprava.
   *
   * Response fronta: OBD task (Core 1) → response dispatch task (Core 0)
   *   Kapacita 10 polozek — vetsi nez request fronta, protoze:
   *   a) Streaming generuje odpovedi prubezne a dispatch task je
   *      nestihne okamzite odeslat (WiFi propustnost)
   *   b) Vetsi buffer absorbuje kratke spicky (burst) v produkci odpovedi
   *
   * Pametovy dopad:
   *   sizeof(obd_request_msg_t) * 5 + sizeof(obd_response_msg_t) * 10
   *   Typicky cca 0.5 + 5.2 = 5.7 KB RAM celkem pro obe fronty.
   */
  obd_request_queue =
      xQueueCreate(OBD_REQUEST_QUEUE_SIZE, sizeof(obd_request_msg_t));
  obd_response_queue =
      xQueueCreate(OBD_RESPONSE_QUEUE_SIZE, sizeof(obd_response_msg_t));

  if (!obd_request_queue || !obd_response_queue) {
    /* Fatalni chyba — bez front nemuze fungovat komunikace mezi tasky.
     * Pravdepodobna pricina: nedostatek RAM (heap fragmentace).
     * Zastavime program v nekonecne smycce — nema smysl pokracovat. */
    Serial.println("[SETUP] FATAL: Nelze vytvorit fronty!");
    while (1) {
      delay(1000);
    } /* Halt — zastaveni programu */
  }
  Serial.println("[SETUP] Fronty vytvoreny (req=5, resp=10)");

  /* Predani ukazatelu na fronty do ws_handler modulu, ktery je
   * potrebuje pro vkladani pozadavku a cteni odpovedi. */
  ws_handler_init(obd_request_queue, obd_response_queue);

  /*
   * 5. Nastaveni WebSocket a HTTP serveru
   *
   * Poradi kroku je dulezite:
   *   a) Registrace WebSocket event handleru (onWsEvent) — definuje,
   *      jak se zpracuji prichozi spojeni a zpravy.
   *   b) Pridani WebSocket jako handleru do HTTP serveru — HTTP server
   *      bude preposílat WebSocket upgrade pozadavky na /ws endpoint.
   *   c) Registrace HTTP rout (staticke stranky, API endpointy).
   *   d) Spusteni serveru (server.begin()) — az po registraci vsech
   *      handleru, jinak by prichozi pozadavky nemely kam jit.
   */
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  /* Hlavni stranka — diagnosticky dashboard ulozeny v PROGMEM (flash).
   * send_P() odesila data primo z flash pameti bez kopirovani do RAM. */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", dashboard_html);
  });

  /* Systemovy status endpoint — vraci JSON s diagnostickymi informacemi.
   * Pouzivan pro monitoring zdravi systemu (heap, uptime, pocet klientu).
   * Priklad odpovedi: {"free_heap":180000,"min_free_heap":150000,
   *   "uptime_ms":60000,"obd_init":true,"ws_clients":1} */
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["free_heap"] = ESP.getFreeHeap();
    doc["min_free_heap"] = ESP.getMinFreeHeap();
    doc["uptime_ms"] = millis();
    doc["obd_init"] = obd_initialized;
    doc["ws_clients"] = ws.count();
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  server.begin();
  Serial.printf("[SETUP] HTTP server na portu %d\n", HTTP_PORT);

  /*
   * 6. Spusteni FreeRTOS tasku na prislusnych jadrech
   *
   * xTaskCreatePinnedToCore() vytvori task a pripne ho na konkretni jadro.
   * Na rozdil od xTaskCreate() (ktery necha scheduler rozhodnout),
   * toto zajisti deterministicke chovani — OBD task vzdy bezi na Core 1,
   * response dispatch vzdy na Core 0.
   *
   * OBD task → Core 1 (APP_CPU):
   *   - Oddelen od WiFi stacku, ktery bezi na Core 0.
   *   - Priorita 4: vyssi nez vychozi Arduino loop() task (priorita 1),
   *     aby OBD komunikace mela prednost pred loop() operacemi.
   *   - Stack 8192 B: dostatecny pro vnorena volani obd2 → isotp → twai.
   *
   * Response dispatch task → Core 0 (PRO_CPU):
   *   - Na stejnem jadre jako WiFi, protoze pristupuje k WebSocket objektu
   *     (ktery neni thread-safe).
   *   - Priorita 3: vyssi nez loop() (1) ale nizsi nez WiFi eventy (19+).
   *   - Stack 6144 B: jednodussi task, staci mene pameti.
   *
   * POZOR: OBD inicializace (obd2_init) se NEPROVADI zde!
   * Provadi se az na prikaz "init" od klienta. Duvod: pri startu ESP32
   * nemusi byt zapalení zapnute a CAN sbernice neaktivni — obd2_init()
   * by selhalo nebo by CAN kontroler presel do stavu BUS_OFF.
   */
  xTaskCreatePinnedToCore(
      obd_task,            /* Ukazatel na funkci tasku */
      "obd_task",          /* Textove jmeno (pro debugging, max 16 znaku) */
      OBD_TASK_STACK_SIZE, /* Velikost zasobniku: 8192 bajtu */
      NULL,                /* Parametr predany tasku (nepouzit) */
      4,                   /* Priorita (vyssi cislo = vyssi priorita) */
      &obd_task_handle,    /* Handle pro pozdejsi kontrolu stavu */
      1                    /* Jadro: Core 1 (APP_CPU) */
  );

  xTaskCreatePinnedToCore(
      response_dispatch_task, "resp_task",
      RESPONSE_TASK_STACK_SIZE,         /* Velikost zasobniku: 6144 bajtu */
      NULL, 3, &response_task_handle, 0 /* Jadro: Core 0 (PRO_CPU) */
  );

  Serial.println("[SETUP] Tasky spusteny (obd=Core1, resp=Core0)");
  Serial.println("========================================");
  Serial.println("  READY — pripojte se na WiFi:");
  Serial.printf("  SSID: %s\n", WIFI_SSID);
  Serial.printf("  URL:  http://%s/\n", WiFi.softAPIP().toString().c_str());
  Serial.println("========================================\n");
}

/**
 * @brief Hlavni smycka Arduino frameworku — bezi opakovaně po dokonceni
 * setup().
 *
 * V teto aplikaci je loop() vyuzita minimalne — veskera logika bezi
 * v dedikovanych FreeRTOS tascich (obd_task na Core 1, response_dispatch_task
 * na Core 0). Loop() bezi na Core 1 s prioritou 1 (nejnizsi z nasich tasku).
 *
 * Jedina funkce loop():
 *   - Periodicky (kazdych 30 sekund) loguje systemove informace na seriovou
 *     konzoli: volna heap pamet, minimalni historicka heap pamet, pocet
 *     pripojenych WebSocket klientu a dobu behu (uptime).
 *   - Toto slouzi k detekci memory leaku pri dlouhodobem provozu.
 *     Pokud "free" postupne klesa a "min" je vyrazne nizsi nez "free",
 *     existuje pravdepodobne uniklá alokace (memory leak).
 *
 * Priklad vystupu:
 *   [SYS  ] Heap: free=185320 min=162000 clients=1 uptime=120s
 *
 * vTaskDelay(1000 ms) na konci uvolni CPU pro jine tasky — bez nej by
 * loop() bezela v tesnè smycce a zbytecne zatezovala CPU.
 */
void loop() {
  static uint32_t last_heap_log = 0;

  if (millis() - last_heap_log > 30000) {
    last_heap_log = millis();
    Serial.printf("[SYS  ] Heap: free=%d min=%d clients=%d uptime=%ds\n",
                  ESP.getFreeHeap(), ESP.getMinFreeHeap(), ws.count(),
                  (int)(millis() / 1000));
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
}
