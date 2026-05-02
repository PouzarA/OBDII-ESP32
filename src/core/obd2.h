/**
 * @file obd2.h
 * @brief Verejne API diagnostickeho protokolu OBD-II pro ESP32
 *
 * Tento hlavickovy soubor definuje kompletni rozhrani vrstvy OBD-II,
 * ktera implementuje diagnosticke sluzby dle normy ISO 15031-5:2006.
 * Podporovane sluzby (mody):
 *   - $01: Aktualni diagnosticka data hnaci soustavy (otacky, teplota,
 * rychlost...)
 *   - $02: Data zamrzleho snimku (freeze frame) - hodnoty v okamziku vzniku
 * zavady
 *   - $03: Cteni potvrzenych diagnostickych poruchovych kodu (DTC)
 *   - $04: Mazani/reset emisnich diagnostickych informaci
 *   - $07: Cteni cekajicich (pending) DTC z aktualniho/posledniho jezdniho
 * cyklu
 *   - $09: Informace o vozidle (VIN, CalID, CVN, IPT, nazev ECU)
 *   - $0A: Cteni permanentnich DTC
 *
 * Dekodovani PID je tabulkove rizene a pokryva kompletni prilohu B normy
 * (PID $00 az $5F) vcetne vsech vzorcu pro prevod surovych bajtu na
 * fyzikalni veliciny (teplota ve stupnich, otacky v RPM, napeti ve V apod.).
 *
 * Pouziti: Nejprve zavolejte obd2_init(), pak obd2_query_supported_pids()
 * pro zjisteni podporovanych PID, a nasledne ctete data pomoci
 * obd2_get_pid() nebo obd2_get_pid_raw().
 *
 * Reference:
 *   ISO 15031-5:2006 - Emisni diagnosticke sluzby
 *   ISO 15031-5:2006 Priloha A - Podporovane PID/OBDMID/TID/INFOTYPE
 *   ISO 15031-5:2006 Priloha B - Skalovani a definice PID ($01–$5A)
 *   SAE J2012:2007 - Bitove kodovani diagnostickych poruchovych kodu (DTC)
 *
 * @author Ales Pouzar
 */

#ifndef OBD2_H
#define OBD2_H

#include "config.h"

#include "isotp.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  Konfigurace                                                              */
/* ========================================================================= */
/**
 * Delka VIN (Vehicle Identification Number) je vzdy presne 17 znaku ASCII.
 * Tato hodnota je dana normou ISO 3779 a je stejna pro vsechna vozidla
 * na svete. Priklad VIN: "WVWZZZ3CZWE123456".
 * Pozor: buffer pro VIN musi mit alespon 18 bajtu (17 znaku + null terminator).
 */
#define OBD2_VIN_LENGTH 17

/**
 * @brief Struktura pro ulozeni VIN kodu od jedne ECU.
 */
typedef struct {
  uint32_t rx_id;                /**< CAN ID jednotky, ktera VIN poslala */
  char vin[OBD2_VIN_LENGTH + 1]; /**< Null-terminated VIN retezec */
} obd2_vin_item_t;

/**
 * @brief Seznam vsech VIN kodu nalezenych v siti (Multi-ECU).
 */
typedef struct {
  obd2_vin_item_t items[ISOTP_MAX_ECU_RESPONSES]; /**< Pole polozek */
  uint8_t count;                                  /**< Pocet nalezenych VIN */
} obd2_vin_list_t;

/**
 * Maximalni delka nazvu ridici jednotky (ECU name) v bajtech.
 *
 * Format dle normy: 4znakova zkratka + null + '-' + az 15 znaku popisu.
 * Priklad: "ECM\0-Engine Control\0" (ridici jednotka motoru).
 * Celkem az 20 bajtu.
 */
#define OBD2_ECU_NAME_MAX_LENGTH 20

/**
 * @brief Struktura pro nazev jedne ECU.
 */
typedef struct {
  uint32_t rx_id;                          /**< CAN ID jednotky */
  char name[OBD2_ECU_NAME_MAX_LENGTH + 1]; /**< Null-terminated nazev */
} obd2_ecu_name_item_t;

/**
 * @brief Seznam nazvu vsech ECU v siti.
 */
typedef struct {
  obd2_ecu_name_item_t items[ISOTP_MAX_ECU_RESPONSES];
  uint8_t count;
} obd2_ecu_name_list_t;

/**
 * @brief Struktura pro jednu detekovanou ECU z broadcast PID $00.
 *
 * Pri inicializaci (obd2_query_supported_pids) se pro kazdou ECU,
 * ktera odpovi na broadcast, ulozi jeji CAN RX ID, individualni
 * bitmaska podporovanych PIDu a volitelne nazev (z Mode 09 $0A).
 */
typedef struct {
  uint32_t rx_id;                          /**< CAN ID odpovedi (napr. 0x7E8, 0x7E9) */
  uint32_t supported_pids[8];              /**< Per-ECU bitmaska (8 x 32 = 256 PIDu) */
  char     name[OBD2_ECU_NAME_MAX_LENGTH + 1]; /**< Nazev ECU (prazdny pokud nezjisten) */
} obd2_detected_ecu_t;

/**
 * @brief Seznam vsech ECU detekovanych pri broadcast PID $00.
 */
typedef struct {
  obd2_detected_ecu_t items[ISOTP_MAX_ECU_RESPONSES];
  uint8_t count;
} obd2_detected_ecu_list_t;

/**
 * Delka jednoho kalibracniho identifikatoru (CalID).
 *
 * Kazdy CalID ma presne 16 znaku (doplnenych nulami na konci).
 * CalID identifikuje konkretni verzi softwaru/kalibrace ridici jednotky.
 * Priklad: "TNKVWAG011234567" (16 znaku).
 */
#define OBD2_CAL_ID_LENGTH 16

/**
 * Delka jednoho CVN (Calibration Verification Number).
 *
 * CVN je 4bajtove overovaci cislo prirazene ke kalibracnimu ID.
 */
#define OBD2_CVN_LENGTH 4

/**
 * Maximalni pocet polozek CalID nebo CVN (Calibration Verification Number)
 * z jedne ridici jednotky.
 *
 * Jedna ridici jednotka muze mit vice kalibraci (napr. zakladni SW
 * a aplikacni SW), kazda s vlastnim CalID a overovacim cislem CVN.
 * Hodnota 4 pokryva bezne pripady; vetsina ECU ma 1-2 kalibrace.
 */
#define OBD2_MAX_INFO_ITEMS 4

/**
 * @brief Struktura pro CalID od jedne ECU.
 */
typedef struct {
  uint32_t rx_id;
  char cal_ids[OBD2_MAX_INFO_ITEMS][OBD2_CAL_ID_LENGTH + 1];
  uint8_t count;
} obd2_ecu_calid_item_t;

/**
 * @brief Seznam CalID od vsech ECU v siti.
 */
typedef struct {
  obd2_ecu_calid_item_t items[ISOTP_MAX_ECU_RESPONSES];
  uint8_t count;
} obd2_calid_list_t;

/**
 * Maximalni pocet surovych bajtu, ktere verejne API uchova pro jeden Mode 09
 * InfoType. 96 B pokryva standardni VIN, CalID, CVN, ECU name i IPT data a
 * zaroven drzi WebSocket JSON odpovedi v rozumne velikosti.
 */
#define OBD2_INFOTYPE_DATA_MAX 96

/**
 * @brief Surova Mode 09 odpoved od jedne ECU.
 *
 * Pole data[] obsahuje pouze payload za SID/InfoType, u beznych InfoType tedy
 * data za NODI bytem. Pro InfoType $00 obsahuje primo bitmasku podpory.
 */
typedef struct {
  uint32_t rx_id;
  uint8_t infotype;
  uint8_t nodi;
  uint16_t data_len;
  bool truncated;
  uint8_t data[OBD2_INFOTYPE_DATA_MAX];
} obd2_infotype_item_t;

/**
 * @brief Surove Mode 09 odpovedi ze vsech odpovidajicich ECU.
 */
typedef struct {
  obd2_infotype_item_t items[ISOTP_MAX_ECU_RESPONSES];
  uint8_t count;
} obd2_infotype_list_t;

/**
 * @brief CVN hodnoty od jedne ECU.
 */
typedef struct {
  uint32_t rx_id;
  uint32_t cvns[OBD2_MAX_INFO_ITEMS];
  uint8_t count;
} obd2_ecu_cvn_item_t;

/**
 * @brief Seznam CVN hodnot od vsech odpovidajicich ECU.
 */
typedef struct {
  obd2_ecu_cvn_item_t items[ISOTP_MAX_ECU_RESPONSES];
  uint8_t count;
} obd2_cvn_list_t;

/* ========================================================================= */
/*  Identifikatory sluzeb OBD-II (SID - Service ID)                          */
/* ========================================================================= */

/**
 * Sluzba $01 — Pozadavek na aktualni diagnosticka data hnaci soustavy.
 *
 * Dle ISO 15031-5 sekce 7.1: Tato sluzba umoznuje pristup k aktualnim
 * hodnotam parametru hnaci soustavy (napr. otacky motoru PID $0C,
 * teplota chladici kapaliny PID $05, rychlost vozidla PID $0D).
 * Ridici jednotka odpovida surovymi bajty, ktere se dekodujidle
 * vzorcu z Prilohy B.
 */
#define OBD2_SID_CURRENT_DATA 0x01

/**
 * Sluzba $02 — Pozadavek na data zamrzleho snimku (freeze frame).
 *
 * Dle ISO 15031-5 sekce 7.2: Freeze frame obsahuje "snimek" hodnot
 * parametru v okamziku, kdy ridici jednotka zaznamenala poruchu (DTC).
 * Format odpovedi je shodny se sluzbou $01, ale data odpovidaji
 * stavu v okamziku vzniku zavady, ne aktualnimu stavu.
 * Navic je nutne specifikovat cislo snimku (frame number, obvykle $00).
 */
#define OBD2_SID_FREEZE_FRAME 0x02

/**
 * Sluzba $03 — Pozadavek na emisni diagnosticke poruchove kody (DTC).
 *
 * Dle ISO 15031-5 sekce 7.3: Vraci seznam vsech potvrzenych (confirmed)
 * emisnich DTC. Potvrzeny DTC znamena, ze porucha byla detekovana
 * v minimalnim poctu jezdnich cyklu (obvykle 2). Kazdy DTC je
 * kodovan jako 2 bajty dle SAE J2012 a dekoduje se na retezec
 * typu "P0143", "C0100", "B0001", "U0100".
 */
#define OBD2_SID_READ_DTC 0x03

/**
 * Sluzba $04 — Vymazani/reset emisnich diagnostickych informaci.
 *
 * Dle ISO 15031-5 sekce 7.4: Tato sluzba maze:
 *   - Vsechny potvrzene i cekajici DTC
 *   - Data zamrzlych snimku (freeze frame)
 *   - Stav pripravenosti monitoru (I/M readiness)
 *   - Vysledky on-board monitoru
 *   - Pocitadla vzdalenosti a casu od smazani DTC
 *   - Zhasne kontrolku MIL (Malfunction Indicator Lamp)
 *
 * POZOR: Vyzaduje zapnute zapalovani a vypnuty motor.
 * Po smazani je nutne projet ridici cyklus pro obnovu pripravenosti.
 */
#define OBD2_SID_CLEAR_DTC 0x04

/**
 * Sluzba $07 — Pozadavek na cekajici (pending) DTC.
 *
 * Dle ISO 15031-5 sekce 7.7: Vraci DTC detekovane v aktualnim
 * nebo poslednim jezdnim cyklu, ktere jeste nejsou potvrzene.
 * Uzitecne pro diagnostiku: ukazuje problemy, ktere se teprve zacinaji
 * projevovat, ale jeste nevedly k rozsviceni kontrolky MIL.
 * Format odpovedi je shodny se sluzbou $03.
 */
#define OBD2_SID_ONBOARD_MONITOR 0x06

#define OBD2_SID_PENDING_DTC 0x07

/**
 * Sluzba $0A -- Pozadavek na permanentni DTC.
 *
 * Permanentni DTC jsou emisni zavady ulozene podle OBD regulaci tak, aby
 * nezmizely prostym smazanim DTC pres Mode $04. Zmizet maji az po splneni
 * podminek monitoru a potvrzeni opravy v dalsich jizdnich cyklech.
 */
#define OBD2_SID_PERMANENT_DTC 0x0A

/**
 * Sluzba $09 — Pozadavek na informace o vozidle.
 *
 * Dle ISO 15031-5 sekce 7.9: Poskytuje identifikacni udaje vozidla:
 *   - VIN (Vehicle Identification Number) — InfoType $02
 *   - Kalibracni ID (CalID) — InfoType $04
 *   - Overovaci cisla kalibraci (CVN) — InfoType $06
 *   - In-use Performance Tracking (IPT) — InfoType $08
 *   - Nazev ridici jednotky (ECU name) — InfoType $0A
 */
#define OBD2_SID_VEHICLE_INFO 0x09

/**
 * Offset pro kladnou odpoved: SID odpovedi = SID pozadavku + 0x40.
 *
 * Dle ISO 15031-5 sekce 7: Kdyz ridici jednotka uspesne zpracuje
 * pozadavek, odpovida SID zvysenym o 0x40.
 * Priklad: Pozadavek $01 (aktualni data) → Odpoved $41.
 *          Pozadavek $03 (cteni DTC)    → Odpoved $43.
 */
#define OBD2_SID_RESPONSE_OFFSET 0x40

/**
 * Identifikator negativni odpovedi.
 *
 * Kdyz ridici jednotka nemuze zpracovat pozadavek, odpovida ramcem:
 *   [0x7F] [puvodni SID] [NRC kod]
 * Priklad: [7F 01 12] = Sluzba $01, subfunction nepodporovana.
 */
#define OBD2_SID_NEGATIVE_RESPONSE 0x7F

/* ========================================================================= */
/*  Kody negativni odpovedi (NRC — Negative Response Code)                   */
/* ========================================================================= */

/**
 * Obecne odmitnuti — ridici jednotka odmitla pozadavek z neurciteho duvodu.
 * Priklad: Interni chyba ECU, ktera nespadne do zadne konkretni kategorie.
 */
#define OBD2_NRC_GENERAL_REJECT 0x10

/**
 * Sluzba neni podporovana — ridici jednotka nezna pozadovany SID.
 * Priklad: Ridici jednotka prevodovky nepodporuje sluzbu $09 (Vehicle Info).
 */
#define OBD2_NRC_SERVICE_NOT_SUPPORTED 0x11

/**
 * Subfunkce neni podporovana — ridici jednotka zna sluzbu,
 * ale konkretni PID nebo InfoType neni implementovan.
 * Priklad: ECU podporuje sluzbu $01, ale nezna PID $5A.
 */
#define OBD2_NRC_SUB_FUNCTION_NOT_SUPPORTED 0x12

/**
 * Podminky nejsou spravne — pozadavek je platny, ale aktualni
 * stav ridici jednotky neumoznuje jeho provedeni.
 * Priklad: Pokus o smazani DTC ($04) pri bezicim motoru,
 *          nebo cteni freeze frame kdyz zadny neexistuje.
 */
#define OBD2_NRC_CONDITIONS_NOT_CORRECT 0x22

/**
 * Pozadavek mimo rozsah — pozadovany parametr je mimo povoleny rozsah.
 * Priklad: Pozadavek na freeze frame cislo 5, kdyz ECU ulozilo jen frame 0.
 */
#define OBD2_NRC_REQUEST_OUT_OF_RANGE 0x31

/**
 * Odpoved se pripravuje (response pending) — ridici jednotka
 * potrebuje vice casu na zpracovani pozadavku.
 *
 * Toto neni chyba! ECU touto zpravou oznamuje, ze jeste pracuje
 * a aby tester neprerusil spojeni. Po dokonceni odesle skutecnou odpoved.
 * Priklad: Mazani DTC muze trvat dele, ECU posila 7F xx 78 kazdych 50 ms.
 */
#define OBD2_NRC_RESPONSE_PENDING 0x78

/* ========================================================================= */
/*  ID typu informaci pro Mode 09 (ISO 15031-5 sekce 7.9)                    */
/* ========================================================================= */

/**
 * InfoType $00 — Bitmaska podporovanych InfoType.
 * Analogie k PID $00 v Mode 01: kazdy bit urcuje,
 * zda je dany InfoType ($01-$20) podporovan.
 */
#define OBD2_INFOTYPE_SUPPORTED 0x00

/**
 * InfoType $01 — Pocet datovych polozek na zpravy (Message Count / NODI).
 * Urcuje, kolik datovych polozek ocekavat v odpovedi pro dany InfoType.
 * Pouziva se napr. pred ctenim VIN pro zjisteni poctu zprav.
 */
#define OBD2_INFOTYPE_MSG_COUNT 0x01

/**
 * InfoType $02 — Identifikacni cislo vozidla (VIN).
 * VIN ma vzdy 17 znaku ASCII dle ISO 3779. Je jednoznacny
 * identifikator kazdeho vyrobeneho vozidla na svete.
 * Priklad: "WVWZZZ3CZWE123456" (VW Golf, vyrobeny v Nemecku).
 */
#define OBD2_INFOTYPE_VIN 0x02

/**
 * InfoType $04 — Kalibracni identifikator (CalID).
 * Identifikuje verzi softwarove kalibrace ridici jednotky.
 * Kazdy CalID ma 16 znaku. ECU muze mit vice kalibraci
 * (napr. zakladni SW kalibrace a aplikacni kalibrace).
 * Priklad: "TNKVWAG011234567".
 */
#define OBD2_INFOTYPE_CAL_ID 0x04

/**
 * InfoType $06 — Overovaci cislo kalibrace (CVN — Calibration Verification
 * Number). 4bajtovy kontrolni soucet (hash) odpovidajici kalibraci. Sluzi k
 * overeni, ze software ECU nebyl neautorizovane zmenen. Kazdemu CalID odpovida
 * jeden CVN.
 */
#define OBD2_INFOTYPE_CVN 0x06

/**
 * InfoType $08 — Sledovani vykonnosti za provozu (IPT — In-use Performance
 * Tracking). Obsahuje citace dokonceni a podminek pro kazdy emisni monitor
 * (napr. kolikrat byl monitor katalyzatoru dokoncen vs. kolikrat
 * byly splneny podminky pro jeho beh). Pouziva se pri emisnich kontrolach.
 */
#define OBD2_INFOTYPE_IPT 0x08

/**
 * InfoType $0A — Nazev ridici jednotky (ECU name).
 * Format: 4znakova zkratka + null + '-' + textovy popis.
 * Priklad: "ECM\0-Engine Control\0" (ridici jednotka motoru),
 *          "TCM\0-Transmission\0" (ridici jednotka prevodovky).
 */
#define OBD2_INFOTYPE_ECU_NAME 0x0A

/**
 * InfoType $0B -- In-use Performance Tracking pro vznetove / compression
 * ignition aplikace.
 */
#define OBD2_INFOTYPE_IPT_COMPRESSION 0x0B

/* ========================================================================= */
/*  Logovani (zrcadli vzor z isotp.h)                                        */
/* ========================================================================= */

/**
 * Aktualni uroven logovani za behu. Zpravy s urovni vyssi nez tato
 * hodnota nebudou vypisovany. Lze menit pomoci obd2_set_log_level().
 */
extern isotp_log_level_t _obd2_runtime_log_level;

/**
 * Hlavni logovaci makro. Vypisuje zpravu pouze pokud uroven splnuje
 * oba limity: kompilacni (OBD2_LOG_MAX_LEVEL) i runtime
 * (_obd2_runtime_log_level).
 *
 * Format vystupu: "[OBD2  UROVEN] zprava\n"
 * Priklad: "[OBD2  INFO] PID $0C: 3200 rpm"
 *
 * Pouziti: OBD2_LOG(ISOTP_LOG_INFO, "Rychlost: %d km/h", speed);
 */
#define OBD2_LOG(level, fmt, ...)                                              \
  do {                                                                         \
    if ((level) <= OBD2_LOG_MAX_LEVEL && (level) <= _obd2_runtime_log_level) { \
      printf("[OBD2  %s] " fmt "\n", _isotp_log_prefix[(level)],               \
             ##__VA_ARGS__);                                                   \
    }                                                                          \
  } while (0)

/** Logovani chyb — kriticke selhani, ze kterych se nelze zotavit */
#define OBD2_LOGE(fmt, ...) OBD2_LOG(ISOTP_LOG_ERROR, fmt, ##__VA_ARGS__)
/** Logovani varovani — problemy, ktere nebranni dalsi cinnosti */
#define OBD2_LOGW(fmt, ...) OBD2_LOG(ISOTP_LOG_WARN, fmt, ##__VA_ARGS__)
/** Logovani informaci — dulezite udalosti behem normalni cinnosti */
#define OBD2_LOGI(fmt, ...) OBD2_LOG(ISOTP_LOG_INFO, fmt, ##__VA_ARGS__)
/** Logovani ladeni — detailni informace pro vyvojare */
#define OBD2_LOGD(fmt, ...) OBD2_LOG(ISOTP_LOG_DEBUG, fmt, ##__VA_ARGS__)
/** Logovani trasovani — maximalni detail kazde operace */
#define OBD2_LOGT(fmt, ...) OBD2_LOG(ISOTP_LOG_TRACE, fmt, ##__VA_ARGS__)

/* ========================================================================= */
/*  Stavove kody                                                             */
/* ========================================================================= */

/**
 * Navratove stavove kody vsech funkci OBD-II vrstvy.
 *
 * Kazda funkce vraci jeden z techto kodu. Pri chybe je mozne ziskat
 * dalsi detaily pomoci obd2_get_last_nrc() (pro OBD2_ERR_NEGATIVE_RESP)
 * nebo obd2_status_str() pro textovy popis.
 */
typedef enum {
  /**
   * Uspech — operace probehla v poradku.
   * Priklad: obd2_get_pid($0C, &result) vratilo platne otacky.
   */
  OBD2_OK = 0,

  /**
   * Casovy limit vyprsell — ridici jednotka neodpovedela v nastavenem
   * casovem limitu (vychozi 2000 ms).
   * Priklad: CAN sbernice odpojena, ECU neni napajeno, spatna rychlost
   * sbernice. Reseni: Zkontrolujte fyzicke pripojeni a nastavenou rychlost CAN
   * (250k/500k).
   */
  OBD2_ERR_TIMEOUT,

  /**
   * Negativni odpoved — ridici jednotka odmitla pozadavek a poslala
   * ramec [7F SID NRC]. Konkretni NRC kod lze zjistit pomoci
   * obd2_get_last_nrc(). Priklad: Pokus o smazani DTC pri bezicim motoru → NRC
   * 0x22.
   */
  OBD2_ERR_NEGATIVE_RESP,

  /**
   * Zadna data — ridici jednotka odpovedela platne, ale odpoved
   * neobsahuje zadna data (prazdna odpoved).
   * Priklad: Mode 03 (cteni DTC) odpovi s 0 DTC — to neni chyba,
   * ale pokud se ocekavala data, funkce vraci tento kod.
   */
  OBD2_ERR_NO_DATA,

  /**
   * Neplatny argument — funkci byl predan NULL ukazatel, neplatny PID,
   * prilis maly buffer nebo jiny neplatny parametr.
   * Priklad: obd2_get_pid(0x0C, NULL) — chybejici vystupni ukazatel.
   */
  OBD2_ERR_INVALID_ARG,

  /**
   * Vrstva neni inicializovana — funkce byla zavolana pred obd2_init().
   * Priklad: Volani obd2_get_pid() bez predchoziho obd2_init().
   * Reseni: Zavolejte obd2_init() s prislusnymi parametry.
   */
  OBD2_ERR_NOT_INITIALIZED,

  /**
   * Chyba podkladove vrstvy ISO-TP — selhala komunikace na urovni
   * transportniho protokolu (segmentace, flow control apod.).
   * Priklad: ISO-TP timeout pri multi-frame prenosu, chyba CAN ovladace.
   */
  OBD2_ERR_ISOTP,

  /**
   * PID neni podporovan — pozadovany PID neni v bitmapve podporovanych
   * PID, kterou ridici jednotka nahlasila pres PID $00/$20/$40.
   * Priklad: Cteni PID $5A na vozidle, ktere ho nepodporuje.
   * Reseni: Nejprve zavolejte obd2_query_supported_pids()
   *         a zkontrolujte obd2_is_pid_supported().
   */
  OBD2_ERR_UNSUPPORTED_PID,

  /**
   * PID neni v dekodovaci tabulce — PID neni obsazen v interni
   * staticke tabulce deskriptoru (pokryva $00–$5F dle Prilohy B).
   * Priklad: Pokus o dekodovani PID $80 (vyrobce-specificke PID).
   * Reseni: Pouzijte obd2_get_pid_raw() a dekodujte rucne.
   */
  OBD2_ERR_DECODE,

  /**
   * Poskozena odpoved — odpoved od ridici jednotky ma nespravny format:
   * nesouhlasi SID, PID, nebo je odpoved prilis kratka.
   * Priklad: Pozadavek na PID $0C, ale odpoved obsahuje PID $0D.
   *          Nebo odpoved ma 1 bajt dat misto ocekavanychch 2.
   */
  OBD2_ERR_RESPONSE_MALFORMED
} obd2_status_t;

/**
 * @brief Diagnostika posledniho init/discovery pokusu.
 *
 * Struktura je urcena hlavne pro dashboard: pri selhani inicializace vraci
 * stav TWAI radice, posledni alerty a posledni OBD/ISO-TP status, aby chyba
 * nebyla jen obecny TIMEOUT.
 */
typedef struct {
  uint32_t twai_state;          /**< Hodnota twai_state_t, serializovana jako cislo */
  uint32_t tx_error_counter;    /**< TEC z TWAI driveru */
  uint32_t rx_error_counter;    /**< REC z TWAI driveru */
  uint32_t msgs_to_tx;          /**< Pocet ramcu cekajicich ve TX fronte */
  uint32_t msgs_to_rx;          /**< Pocet ramcu cekajicich v RX fronte */
  uint32_t alerts;              /**< OR vsech TWAI alertu zachycenych pri initu */
  uint8_t init_attempts;        /**< Pocet realne odeslanych init dotazu */
  uint32_t last_tx_id;          /**< Posledni CAN ID pozadavku */
  uint32_t last_rx_id;          /**< Ocekavane/posledni CAN ID odpovedi */
  isotp_status_t last_isotp_status; /**< Posledni status ISO-TP vrstvy */
  obd2_status_t last_obd_status;    /**< Posledni status OBD vrstvy */
  bool used_physical_fallback;  /**< true pokud PID $00 zkousel 0x7E0 -> 0x7E8 */
  bool reinit_performed;        /**< true pokud probehl kratky TWAI/ISO-TP reinit */
} obd2_init_diag_t;

/* ========================================================================= */
/*  System dekodovani PID — tabulkove rizeny (ISO 15031-5 Priloha B)         */
/* ========================================================================= */

/**
 * Typy vzorcu pokryvajici vsechny formaty PID z Prilohy B normy ISO 15031-5.
 *
 * Genericka dekodovaci funkce obd2_decode_pid_value() rozhoduje na zaklade
 * tohoto enumu, jaky vzorec pouzit pro prevod surovych bajtu na fyzikalni
 * velicinu (teplotu, otacky, napeti atd.).
 */
typedef enum {
  /**
   * Linearni vzorec s 1 datovym bajtem:
   *   hodnota = A * nasobitel + offset
   *
   * Priklad: PID $05 (teplota chladici kapaliny)
   *   A = 70, nasobitel = 1.0, offset = -40.0
   *   hodnota = 70 * 1.0 + (-40.0) = 30 °C
   *
   * Priklad: PID $04 (zatizeni motoru)
   *   A = 128, nasobitel = 100.0/255.0, offset = 0
   *   hodnota = 128 * (100/255) ≈ 50.2 %
   */
  OBD2_FMT_LINEAR_1B,

  /**
   * Linearni vzorec s 2 datovymi bajty:
   *   hodnota = (256*A + B) * nasobitel + offset
   *
   * Priklad: PID $0C (otacky motoru)
   *   A = 0x0C, B = 0x80, nasobitel = 0.25, offset = 0
   *   hodnota = (256*12 + 128) * 0.25 = 3200/4 = 800 RPM
   *
   * Priklad: PID $0D (rychlost vozidla) pouziva FMT_LINEAR_1B,
   *   ale PID $10 (prutok vzduchu MAF) pouziva tento format:
   *   A = 0x01, B = 0x00, nasobitel = 0.01, offset = 0
   *   hodnota = 256 * 0.01 = 2.56 g/s
   */
  OBD2_FMT_LINEAR_2B,

  /**
   * Znamienkovy vzorec s posunem a 1 datovym bajtem:
   *   hodnota = (A - 128) * nasobitel
   *
   * Pouziva se pro parametry, ktere mohou byt kladne i zaporne,
   * kde 128 predstavuje nulovou hodnotu.
   *
   * Priklad: PID $06 (kratkodoba korekce paliva — STFT, bank 1)
   *   A = 0, nasobitel = 100.0/128.0
   *   hodnota = (0 - 128) * (100/128) = -100% (maximalni ochuzeni smesi)
   *   A = 128, hodnota = 0% (zadna korekce)
   *   A = 255, hodnota = +99.2% (maximalni obohaceni smesi)
   */
  OBD2_FMT_SIGNED_OFFSET_1B,

  /**
   * Znamienkovy vzorec s 2 datovymi bajty (int16_t):
   *   hodnota = (int16_t)(256*A + B) * nasobitel + offset
   *
   * 2 bajty jsou interpretovany jako znamienkove 16bitove cislo
   * (rozsah -32768 az +32767).
   *
   * Priklad: PID $2C (prikaz EGR)
   *   Pouziva se pro parametry, ktere mohou nabyvat zapornych hodnot
   *   v plnem 16bitovem rozsahu.
   */
  OBD2_FMT_SIGNED_2B,

  /**
   * Bitove kodovane PID — data jsou interpretovana po jednotlivych bitech.
   *
   * Pro tyto PID se neprovadi prevod na float; surove bajty se zabali
   * do uint32 a vrati se jako "hodnota" (pretypovana na float).
   * Pro spravnou interpretaci pouzijte specializovane funkce
   * (napr. obd2_get_monitor_status() pro PID $01).
   *
   * PID pouzivajici tento format:
   *   $01 — Stav monitoru od smazani DTC (MIL, pocet DTC, pripravenost)
   *   $03 — Stav palivoveho systemu (otevrena/zavrena smycka)
   *   $12 — Prikazany sekundarni stav vzduchu
   *   $13 — Umisteni kysllikovych senzoru (bank 1/bank 2)
   *   $1C — Standard OBD, kteremu vozidlo vyhovuje
   *   $1D — Umisteni kysllikovych senzoru (4 banky)
   *   $1E — Stav pomocneho vstupu (PTO status)
   *   $41 — Stav monitoru pro aktualni jezdni cyklus
   *   $51 — Typ paliva (benzin, diesel, LPG, CNG, elektro...)
   */
  OBD2_FMT_BIT_ENCODED,

  /**
   * Konvencni kysllilikovy senzor (PIDs $14–$1B, 2 datove bajty).
   *
   * Bajt A = napeti:
   *   napeti = A * 0.005 V (rozsah 0–1.275 V)
   * Bajt B = kratkodoba korekce paliva (STFT):
   *   STFT = (B - 128) * 100/128 % (rozsah -100 az +99.2 %)
   *
   * Primarni hodnota = napeti senzoru.
   * Sekundarni hodnota = STFT (pristupna pres obd2_decode_pid_secondary).
   *
   * Priklad: PID $14 (O2 senzor 1, bank 1)
   *   A = 0x90 (144), B = 0x80 (128)
   *   napeti = 144 * 0.005 = 0.72 V (bohatsi smes)
   *   STFT = (128 - 128) * 100/128 = 0% (zadna korekce)
   */
  OBD2_FMT_O2_CONV,

  /**
   * Sirokopasmovy kysllilikovy senzor: ekvivalencni pomer + napeti
   * (PIDs $24–$2B, 4 datove bajty).
   *
   * Bajty AB = ekvivalencni pomer (lambda):
   *   lambda = (256*A + B) * 2/65536 (rozsah 0–2.0)
   * Bajty CD = napeti:
   *   napeti = (256*C + D) * 8/65536 V (rozsah 0–8.0 V)
   *
   * Primarni hodnota = ekvivalencni pomer (lambda).
   * Sekundarni hodnota = napeti (pristupne pres obd2_decode_pid_secondary).
   *
   * Lambda = 1.0 znamena stechiometrickou smes (14.7:1 pro benzin).
   * Lambda < 1.0 = bohata smes, Lambda > 1.0 = chuda smes.
   */
  OBD2_FMT_O2_WIDE_EQ_V,

  /**
   * Sirokopasmovy kysllilikovy senzor: ekvivalencni pomer + proud
   * (PIDs $34–$3B, 4 datove bajty).
   *
   * Bajty AB = ekvivalencni pomer (lambda):
   *   lambda = (256*A + B) * 2/65536 (rozsah 0–2.0)
   * Bajty CD = proud:
   *   proud = (256*C + D) / 256 - 128 mA
   *   (rozsah -128 az +127.996 mA)
   *
   * Primarni hodnota = ekvivalencni pomer (lambda).
   * Sekundarni hodnota = proud v mA (pristupny pres obd2_decode_pid_secondary).
   *
   * Kladny proud = chuda smes, Zaporny proud = bohata smes.
   */
  OBD2_FMT_O2_WIDE_EQ_I,

  /**
   * Konfiguracni PID ($4F, $50) — konfigurace zkusebniho zarizeni.
   *
   * Tyto PID nejsou urceny pro zobrazeni uzivatelivi; pouzivaji se
   * k uprave skalovani jinych PID (napr. maximalni hodnoty napeti,
   * proudu, ekvivalencniho pomeru pro danou ridici jednotku).
   * V beznem diagnostickem nastroji je neni treba dekodovat.
   */
  OBD2_FMT_CONFIG,

  /**
   * Jednoduchy vyctovy typ (Enum) — 1 datovy bajt.
   * Na rozdil od FMT_BIT_ENCODED se neprovadi zadne bitove posuny
   * a hodnota bajtu A se vrati primo jako float.
   * Pouziva se pro PID $1C (OBD Standard) a $51 (Fuel Type).
   */
  OBD2_FMT_ENUM,

  /**
   * Linearni vzorec se 4 datovymi bajty (32-bit unsigned):
   *   hodnota = ((A<<24) | (B<<16) | (C<<8) | D) * multiplier + offset
   *
   * Pouziti: PID $7F (engine run time, total seconds), PID $A6 (odometer,
   * km × 10), PID $9D (engine fuel rate, alternativni 4-byte forma).
   *
   * Priklad PID $A6 (odometer):
   *   nasobitel = 0.1, offset = 0
   *   raw = 0x000186A0 = 100000 → hodnota = 10000.0 km
   */
  OBD2_FMT_LINEAR_4B,

  /**
   * Multi-sensor teplotni PID — az 4 senzory v jednom dotazu (9 datovych bajtu).
   *
   * Format dat (PIDs $78, $79, $98, $99):
   *   A:    bity A0..A3 = podpora senzoru 1..4 (1 = senzor pritomen)
   *   B,C:  senzor 1 = (256B+C) * 0.1 - 40 °C
   *   D,E:  senzor 2 = (256D+E) * 0.1 - 40 °C
   *   F,G:  senzor 3 = (256F+G) * 0.1 - 40 °C
   *   H,I:  senzor 4 = (256H+I) * 0.1 - 40 °C
   *
   * Dekodovane hodnoty se ulozi do pole values[] v obd2_pid_decoded_t.
   * value_count odrazi pocet PRITOMNYCH (podporovanych) senzoru. Senzory,
   * ktere nejsou v support flagu, vraci NAN na sve pozici.
   */
  OBD2_FMT_TEMP_4S,

  /**
   * Multi-sensor NOx koncentrace — az 4 senzory (9 datovych bajtu).
   *
   * Format dat (PID $83, $A1):
   *   A:    bity A0..A3 = podpora senzoru
   *   B,C:  senzor 1 = (256B+C) ppm  (0xFFFF = neplatne mereni)
   *   D,E:  senzor 2 = (256D+E) ppm
   *   F,G:  senzor 3 = (256F+G) ppm
   *   H,I:  senzor 4 = (256H+I) ppm
   *
   * Senzor s hodnotou 0xFFFF se ulozi jako NAN (neplatny stav).
   */
  OBD2_FMT_NOX_4S,

  /**
   * Raw / nedeklarovany format — pro PIDy z descriptoru bez dekoderu.
   *
   * Hodnota value se nenastavuje (NAN). Klient (frontend) si data prevezme
   * v surove podobe pres obd2_get_pid_raw(). Pouziva se pro PIDy se
   * smisenymi bit/value formaty (napr. $69 EGR multi-info, $6C throttle
   * actuator), kde dekodovani v C vrstve neni rentabilni — frontend
   * ukaze raw bajty pro rucni rozbor.
   */
  OBD2_FMT_RAW,
} obd2_pid_format_t;

/**
 * Kategorie PID pro UI segmentaci.
 *
 * - TELEMETRY: kontinualne se menici skalary vhodne pro DASH (otacky,
 *   teploty, tlaky, prutoky, pomery...). Stream je posila s vysokou frekvenci.
 * - STATUS: bitove pole nebo kumulativni pocitadla, ktere se meni zridka
 *   (DTC count, monitor ready bity, doba behu, vzdalenost s MIL...).
 *   Citaji se on-demand v Diag panelu.
 * - CONFIG: staticke informace o vozidle (OBD standard, typ paliva,
 *   maximalni rozsahy senzoru). Cteno jen 1× po init, zobrazuje se
 *   ve Vehicle Info na HOME.
 * - META: bitmasky podporovanych PIDu ($00, $20, $40, $60, $80, $A0, $C0).
 *   Ne urceny pro zobrazeni — pouze pro detekci podpory.
 */
typedef enum {
  OBD2_CAT_TELEMETRY = 0,
  OBD2_CAT_STATUS,
  OBD2_CAT_CONFIG,
  OBD2_CAT_META,
} obd2_pid_category_t;

/**
 * Deskriptor PID — jeden zaznam pro kazdy standardizovany PID.
 *
 * Interni staticka tabulka v obd2.c obsahuje deskriptory pro PID $00–$5F
 * dle Prilohy B normy ISO 15031-5. Kazdy deskriptor definuje, jak
 * dekodovat surove bajty z ridici jednotky na fyzikalni velicinu.
 *
 * Priklad zaznamu pro PID $0C (otacky motoru):
 *   { .pid=0x0C, .name="Engine RPM", .unit="rpm",
 *     .data_len=2, .format=OBD2_FMT_LINEAR_2B,
 *     .multiplier=0.25, .offset=0.0 }
 */
typedef struct {
  uint8_t pid;      /**< Hodnota PID ($00–$FF) */
  const char *name; /**< Lidsky citelny nazev PID (anglicky, dle normy) */
  const char
      *unit; /**< Jednotka zobrazeni ("rpm", "°C", "%", "kPa", "V", ...) */
  uint8_t
      data_len; /**< Ocekavany pocet datovych bajtu v odpovedi (1, 2, 4, 9...) */
  obd2_pid_format_t format; /**< Typ dekodovaciho vzorce */
  float multiplier;         /**< Nasobitel (skalovaci faktor) ve vzorci */
  float offset;             /**< Offset (posuv) ve vzorci */
  obd2_pid_category_t category; /**< Kategorizace pro UI (TELEMETRY/STATUS/CONFIG/META) */
} obd2_pid_desc_t;

/* ========================================================================= */
/*  Datove typy pro vysledky                                                 */
/* ========================================================================= */

/**
 * Maximalni delka dat jednoho PIDu v bajtech.
 *
 * Standardni PIDy mode 01 maji 1, 2 nebo 4 bajty. Pozdejsi rozsireni
 * (Wikipedia OBD-II PIDs, ISO 15031-5 dodatky) zavadi multi-sensor PIDy
 * s 9 bajty (PID $78/$79 EGT — 4 teplotni senzory) a slozitejsi diesel
 * PIDy do 13+ bajtu (PID $7F engine run time, $86 PM sensor s flagy).
 * Hodnota 64 pokryva i dlouhe J1979-DA PIDy s AECD run-time bloky (41 B).
 */
#define OBD2_PID_MAX_DATA_BYTES 64

/**
 * Maximalni pocet dekodovanych hodnot v jednom PIDu.
 *
 * - 1 hodnota: bezne skalary (RPM, teplota, tlak...)
 * - 2 hodnoty: O2 senzory (napeti+STFT, lambda+napeti, lambda+proud)
 * - 3-4 hodnoty: multi-sensor teploty ($78 EGT 4 senzory) nebo NOx ($83)
 *
 * Hodnota 4 pokryva vsechny soucasne standardni PIDy. Vyssi pocet senzoru
 * by vyzadoval rozsireni teto konstanty.
 */
#define OBD2_PID_MAX_VALUES 4

/**
 * Surova data PID (odpoved Mode 01 / Mode 02).
 *
 * Obsahuje nezpracovane datove bajty presne tak, jak je prijala
 * ridici jednotka. Pro ziskani fyzikalni veliciny je nutne
 * data dekodovat pomoci obd2_decode_pid_value() nebo pouzit
 * primo obd2_get_pid(), ktera dekodovani provede automaticky.
 *
 * Buffer ma OBD2_PID_MAX_DATA_BYTES pro pokryti multi-sensor i dlouhych PIDu.
 * Bezne PIDy pouziji jen prvni 1-4 bajty.
 */
typedef struct {
  uint8_t pid;      /**< Pozadovany PID (echo z odpovedi) */
  uint8_t data[OBD2_PID_MAX_DATA_BYTES]; /**< Datove bajty (A, B, C, D, E... dle normy) */
  uint8_t data_len; /**< Skutecny pocet prijatych datovych bajtu */
} obd2_pid_raw_t;

/**
 * Dekodovana hodnota PID.
 *
 * Primarni hodnota (value) je dekodovany float dle vzorce z Prilohy B.
 * Sekundarni hodnota (secondary) je urcena pro PID s dvojitym vystupem
 * (kysllikove senzory):
 *   - PIDs $14–$1B: primarni = napeti, sekundarni = STFT (%)
 *   - PIDs $24–$2B: primarni = lambda, sekundarni = napeti (V)
 *   - PIDs $34–$3B: primarni = lambda, sekundarni = proud (mA)
 *   - Ostatni 1-2 hodnotove PIDy: sekundarni = NAN (neaplikovatelne)
 *
 * Multi-sensor PIDy (TEMP_4S, NOX_4S) ulozi az 4 hodnoty do extra[]:
 *   - value      = senzor 1
 *   - secondary  = senzor 2
 *   - extra[0]   = senzor 3
 *   - extra[1]   = senzor 4
 * Nedostupny senzor (support bit 0) ma na sve pozici NAN.
 *
 * value_count udava pocet PRITOMNYCH (validnich) hodnot — pro skalary 1,
 * pro O2 obvykle 2, pro multi-sensor 1-4 dle support flagu.
 */
typedef struct {
  uint8_t pid;     /**< Pozadovany PID */
  uint8_t raw_data[OBD2_PID_MAX_DATA_BYTES]; /**< Puvodni datove bajty A..N */
  uint8_t raw_data_len; /**< Pocet puvodnich datovych bajtu */
  uint8_t value_count; /**< Pocet validnich hodnot (1-4); ostatni jsou NAN */
  float value;     /**< Primarni dekodovana hodnota (napr. 800.0 pro 800 RPM) */
  float secondary; /**< Sekundarni hodnota (O2 senzory, 2. teplotni senzor),
                        NAN pokud neaplikovatelne */
  float extra[OBD2_PID_MAX_VALUES - 2]; /**< 3. a 4. hodnota (multi-sensor PIDy) */
  const char *name; /**< Nazev PID z tabulky deskriptoru (napr. "Engine RPM") */
  const char *unit; /**< Jednotka primarni hodnoty (napr. "rpm", "°C") */
} obd2_pid_decoded_t;

/**
 * Diagnosticky poruchovy kod (DTC — Diagnostic Trouble Code).
 *
 * Surove 2 bajty z ridici jednotky jsou dekovany na retezec typu "P0143".
 * Prvni znak urcuje kategorii:
 *   P = Powertrain (hnaci soustava — motor, prevodovka)
 *   C = Chassis (podvozek — ABS, ESP)
 *   B = Body (karoserie — klimatizace, osvetleni)
 *   U = Network (sit — komunikace mezi ridiciimi jednotkami)
 *
 * Priklad: surove bajty [0x01, 0x43] → "P0143" (O2 senzor, bank 1, senzor 3)
 */
typedef struct {
  char code[6];   /**< Textovy DTC, napr. "P0143\0", "C0100\0", "U0401\0" */
  uint8_t raw[2]; /**< Puvodni 2 surove bajty z ridici jednotky */
} obd2_dtc_t;

/**
 * Seznam DTC pro jednu ridici jednotku (ECU).
 */
typedef struct {
  uint16_t ecu_id;     /**< CAN ID ridici jednotky (napr. 0x7E8) */
  uint8_t count;       /**< Pocet DTC v teto jednotce */
  obd2_dtc_t dtcs[32]; /**< Pole DTC kodu (omezeno na 32 pro setreni RAM) */
} obd2_ecu_dtc_list_t;

/**
 * Kompletni vysledek cteni DTC z celeho vozidla (vice ECU).
 */
typedef struct {
  uint8_t ecu_count;           /**< Pocet jednotek, ktere odpovedely */
  obd2_ecu_dtc_list_t ecus[8]; /**< Data z jednotlivych jednotek (max 8) */
} obd2_multi_ecu_dtc_t;

/**
 * Stav MIL a pripravenost monitoru (dekodovany PID $01).
 *
 * PID $01 vraci 4 bajty bitove kodovanych dat obsahujicich:
 *   - Pocet aktivnich emisnich DTC (bity 0-6 bajtu A)
 *   - Stav kontrolky MIL (bit 7 bajtu A)
 *   - Ktere prubezne a neprubezne monitory jsou podporovany (supported)
 *   - Ktere monitory jsou pripraveny (ready) — dokoncily svuj test
 *
 * Monitor "podporovan" (sup) = ridici jednotka tento test implementuje.
 * Monitor "pripraven" (rdy) = test probehl a ma vysledek.
 *
 * Pro emisni kontrolu (STK/ME) musi byt vsechny podporovane monitory
 * ve stavu "pripraven" (ready), jinak kontrola nemuze byt provedena.
 */
typedef struct {
  uint8_t dtc_count; /**< Pocet emisnich DTC (bity 0-6 bajtu A, rozsah 0-127) */
  bool mil_on;       /**< Stav kontrolky MIL: true = sviti (bit 7 bajtu A) */
  bool is_compression; /**< Typ motoru: true = vznětový (Diesel), false = zážehový (Benzín) */

  uint8_t raw[4]; /**< Raw PID $01 bytes A..D for diagnostics. */

  /* Prubezne monitory — bezi nepretrzite behem chodu motoru */
  bool misfire_sup;  /**< Monitor vynechavani zapalovani — podporovan */
  bool misfire_rdy;  /**< Monitor vynechavani zapalovani — pripraven */
  bool fuel_sys_sup; /**< Monitor palivoveho systemu — podporovan */
  bool fuel_sys_rdy; /**< Monitor palivoveho systemu — pripraven */
  bool ccm_sup;      /**< Kompletni monitor komponent (CCM) — podporovan */
  bool ccm_rdy;      /**< Kompletni monitor komponent (CCM) — pripraven */

  /* Neprubezne monitory — bezi jen pri splneni urcitych podminek */
  bool cat_sup;  /**< Monitor katalyzatoru — podporovan */
  bool cat_rdy;  /**< Monitor katalyzatoru — pripraven */
  bool hcat_sup; /**< Monitor vyhrivaneho katalyzatoru — podporovan */
  bool hcat_rdy; /**< Monitor vyhrivaneho katalyzatoru — pripraven */
  bool evap_sup; /**< Monitor systemu odparovani paliva (EVAP) — podporovan */
  bool evap_rdy; /**< Monitor systemu odparovani paliva (EVAP) — pripraven */
  bool air_sup;  /**< Monitor sekundarniho vzduchu — podporovan */
  bool air_rdy;  /**< Monitor sekundarniho vzduchu — pripraven */
  bool acrf_sup; /**< Monitor klimatizace/chladiva — podporovan */
  bool acrf_rdy; /**< Monitor klimatizace/chladiva — pripraven */
  bool o2s_sup;  /**< Monitor kysllikovych senzoru — podporovan */
  bool o2s_rdy;  /**< Monitor kysllikovych senzoru — pripraven */
  bool htr_sup;  /**< Monitor vyhrivani kysllikovych senzoru — podporovan */
  bool htr_rdy;  /**< Monitor vyhrivani kysllikovych senzoru — pripraven */
  bool egr_sup; /**< Monitor recirkulace vyfukovych plynu (EGR) — podporovan */
  bool egr_rdy; /**< Monitor recirkulace vyfukovych plynu (EGR) — pripraven */
} obd2_monitor_status_t;

/**
 * @brief Struktura pro stav monitoru jedne ECU.
 */
typedef struct {
  uint16_t rx_id;               /**< CAN ID jednotky */
  obd2_monitor_status_t status; /**< Stav monitoru */
} obd2_ecu_monitor_status_item_t;

/**
 * @brief Seznam stavu monitoru vsech ECU v siti.
 */
typedef struct {
  obd2_ecu_monitor_status_item_t items[ISOTP_MAX_ECU_RESPONSES];
  uint8_t count;
} obd2_monitor_status_list_t;

/**
 * Informace o negativni odpovedi.
 *
 * Tato struktura je naplnena, kdyz funkce OBD-II vrati OBD2_ERR_NEGATIVE_RESP.
 * Ziskate ji volanim obd2_get_last_nrc().
 *
 * Priklad: Po neuspechu obd2_clear_dtc() s navratem OBD2_ERR_NEGATIVE_RESP:
 *   obd2_nrc_info_t nrc = obd2_get_last_nrc();
 *   // nrc.request_sid = 0x04, nrc.nrc = 0x22 (podminky nejsou spravne)
 */
typedef struct {
  uint8_t request_sid; /**< SID pozadavku, ktery byl odmitnut (napr. 0x04 pro
                          mazani DTC) */
  uint8_t nrc;         /**< Kod negativni odpovedi (viz konstanty OBD2_NRC_*) */
} obd2_nrc_info_t;

/* ========================================================================= */
/*  Verejne API                                                              */
/**
 * @brief Struktura pro ulozeni suroveho diagnostickeho paketu (pro terminál).
 *
 * Tato struktura uchovava kompletni informaci o jednom dotazu a jeho odpovedi
 * v surovem stavu, vcetne CAN ID, SID a vsech datovych bajtu.
 */
typedef struct {
  uint32_t rx_id;                  /**< CAN ID jednotky (odpoved) */
  uint8_t service;                 /**< Service ID pozadavku (např. 0x01) */
  uint8_t pid;                     /**< PID ID pozadavku (např. 0x0C) */
  uint8_t data[ISOTP_MAX_PAYLOAD]; /**< Surova data odpovedi (vcetne SID+40 a
                                      PID) */
  uint16_t data_len; /**< Skutecna delka prijatych dat v poli data */
  bool is_negative;  /**< True pokud slo o negativni odpoved (7F SID NRC) */
  uint8_t nrc_code;  /**< NRC kod pokud is_negative == true */
} obd2_raw_response_t;

/* ========================================================================= */

/* ---- Zivotni cyklus ------------------------------------------------------ */

/**
 * @brief Inicializace vrstvy OBD-II (internne vola isotp_init).
 *
 * Tuto funkci je nutne zavolat jako prvni pred jakymkoliv pouzitim
 * OBD-II rozhrani. Inicializuje CAN ovladac (TWAI), vrstvu ISO-TP
 * a interni stav OBD-II vrstvy.
 *
 * @param baudrate  Rychlost CAN sbernice v bit/s.
 *                  Bezne hodnoty: 500000 (vetsina osobnich vozidel)
 *                  nebo 250000 (nakladni vozidla, starsi vozy).
 * @param tx_pin    Cislo GPIO pinu pro vysilani CAN (TX).
 *                  Priklad: GPIO_NUM_5 na ESP32.
 * @param rx_pin    Cislo GPIO pinu pro prijem CAN (RX).
 *                  Priklad: GPIO_NUM_4 na ESP32.
 * @return OBD2_OK pri uspechu, OBD2_ERR_ISOTP pri selhani inicializace
 *
 * @note Po inicializaci je vychozi adresa ECU nastavena na 0x7E0/0x7E8
 *       (standardni adresa ridici jednotky motoru).
 *
 * Priklad pouziti:
 * @code
 *   obd2_status_t st = obd2_init(500000, GPIO_NUM_5, GPIO_NUM_4);
 *   if (st != OBD2_OK) {
 *       printf("Chyba inicializace: %s\n", obd2_status_str(st));
 *   }
 * @endcode
 */
obd2_status_t obd2_init(uint32_t baudrate, int tx_pin, int rx_pin);

/**
 * @brief Vraci true, pokud je CAN/ISO-TP transport inicializovany.
 *
 * Tento stav znamena pouze to, ze TWAI/ISO-TP vrstva bezi. Neznamena to,
 * ze uz odpovedela ECU nebo ze je vybrana aktivni ECU pro cteni PIDu.
 */
bool obd2_is_transport_initialized(void);

/**
 * @brief Deinicializace vrstvy OBD-II a podkladovych vrstev ISO-TP/TWAI.
 *
 * Uvolni vsechny prostredky alokovane behem inicializace.
 * Po zavolani teto funkce neni mozne pouzivat zadne OBD-II funkce
 * bez opetovneho zavolani obd2_init().
 */
void obd2_deinit(void);

/**
 * @brief Nastaveni urovne logovani za behu pro vrstvu OBD-II.
 *
 * Umoznuje zmenit uroven logovani bez nutnosti rekompilace.
 * Zpravy s urovni vyssi nez nastavena hodnota nebudou vypisovany.
 *
 * @param level  Pozadovana uroven: ISOTP_LOG_NONE (zadne logy)
 *               az ISOTP_LOG_TRACE (maximalni detail).
 *
 * Priklad: obd2_set_log_level(ISOTP_LOG_INFO); // pouze INFO a dulezitejsi
 */
void obd2_set_log_level(isotp_log_level_t level);

/**
 * @brief Nastaveni fyzicke adresy ridici jednotky (vychozi 0x7E0/0x7E8).
 *
 * Zavolejte po inicializaci pro cileni na konkretni ridici jednotku.
 * Ovlivni vsechny fyzicke pozadavky Mode 01/02/09.
 *
 * Standardni adresy dle ISO 15031-5:
 *   0x7E0/0x7E8 — Ridici jednotka motoru (ECM) — vychozi
 *   0x7E1/0x7E9 — Ridici jednotka prevodovky (TCM)
 *   0x7E2–0x7E7 / 0x7EA–0x7EF — Dalsi ridici jednotky
 *
 * @param tx_id  CAN ID pozadavku (0x7E0–0x7E7)
 * @param rx_id  CAN ID odpovedi (0x7E8–0x7EF)
 *
 * @note tx_id a rx_id musi byt sparovany par (rx_id = tx_id + 8).
 */
void obd2_set_ecu_address(uint32_t tx_id, uint32_t rx_id);

/**
 * @brief Vybere aktivni ECU podle jejiho response CAN ID (0x7E8--0x7EF).
 *
 * Po uspesnem bindu budou bezne physical dotazy smerovat na tx_id = rx_id - 8.
 * Pokud uz probehla discovery, funkce vyzaduje, aby ECU byla v seznamu
 * detekovanych jednotek.
 */
obd2_status_t obd2_bind_active_ecu(uint32_t rx_id);

/**
 * @brief Vrati aktualne vybranou physical ECU.
 *
 * @return true pokud je aktivni ECU explicitne vybrana, jinak false.
 */
bool obd2_get_active_ecu(uint32_t *tx_id, uint32_t *rx_id);

/**
 * @brief Nastaveni casoveho limitu pro odpoved (vychozi
 * OBD2_DEFAULT_TIMEOUT_MS).
 *
 * @param timeout_ms  Casovy limit v milisekundach.
 *                    Priklad: 5000 pro pomalou CAN sbernici.
 */
void obd2_set_timeout(uint32_t timeout_ms);

/**
 * @brief Ziskani informaci o posledni negativni odpovedi.
 *
 * Platne pouze po tom, co nektera funkce vratila OBD2_ERR_NEGATIVE_RESP.
 * Umoznuje zjistit, ktera sluzba byla odmitnuta a proc (NRC kod).
 *
 * @return Struktura s SID odmitnute sluzby a NRC kodem.
 *
 * Priklad:
 * @code
 *   if (obd2_clear_dtc() == OBD2_ERR_NEGATIVE_RESP) {
 *       obd2_nrc_info_t nrc = obd2_get_last_nrc();
 *       printf("Sluzba 0x%02X odmitnuta: %s\n",
 *              nrc.request_sid, obd2_nrc_str(nrc.nrc));
 *   }
 * @endcode
 */
obd2_nrc_info_t obd2_get_last_nrc(void);

/* ---- Zjisteni podporovanych PID (Mode 01) -------------------------------- */

/**
 * @brief Dotaz na vsechny podporovane PID z ridici jednotky pres broadcast.
 *
 * Iterativne cte PID $00, $20, $40... dokud bitmaska neindikuje,
 * ze dalsi rozsahy nejsou k dispozici. Vysledky jsou cachovany internne
 * a pouzivaji se funkci obd2_is_pid_supported().
 *
 * Tuto funkci je vhodne zavolat jednou po inicializaci, pred samotnym
 * ctenim diagnostickych dat. Pouziva broadcast adresu (0x7DF),
 * takze odpovi vsechny ridici jednotky na sbernici.
 *
 * @return OBD2_OK pri uspechu (alespon PID $00 odpovezel),
 *         OBD2_ERR_TIMEOUT pokud zadna ECU neodpovedela.
 */
obd2_status_t obd2_query_supported_pids(void);

/**
 * @brief Provede samostatny functional PID $00 probe na 0x7DF.
 *
 * Slouzi pro diagnostiku pred plnym init. Vraci vsechny odpovedi 0x7E8..0x7EF
 * vcetne jejich rx_id a payloadu. Nevybira aktivni ECU a nemeni OBD ready stav.
 */
obd2_status_t obd2_probe_pid00(isotp_result_t *result);

/**
 * @brief Kontrola, zda je dany PID podporovan (z cachovane bitmasky).
 *
 * Vraci false, pokud nebyla zavolana obd2_query_supported_pids().
 *
 * @param pid  PID k overeni ($01–$FF). PID $00/$20/$40 jsou vzdy
 *             "podporovane" (jsou to bitmasky dalsiho rozsahu).
 * @return true pokud je PID podporovan alespon jednou ridici jednotkou
 *
 * Priklad:
 * @code
 *   if (obd2_is_pid_supported(0x0C)) {
 *       // Cteni otacek motoru je mozne
 *   }
 * @endcode
 */
bool obd2_is_pid_supported(uint8_t pid);

/**
 * @brief Kontrola PIDu proti UNION masce vsech detekovanych ECU.
 *
 * Bezne obd2_is_pid_supported() po discovery preferuje aktivni ECU. Tato
 * varianta zustava pro pripady, kdy klient opravdu chce videt union cele site.
 */
bool obd2_is_pid_supported_union(uint8_t pid);

/**
 * @brief Kontrola PIDu proti masce konkretni detekovane ECU.
 */
bool obd2_is_pid_supported_by_ecu(const obd2_detected_ecu_t *ecu, uint8_t pid);

/**
 * @brief Vraci seznam ECU detekovanych pri poslednim obd2_query_supported_pids().
 *
 * Kazda polozka obsahuje CAN RX ID, individualni bitmasku podporovanych
 * PIDu a volitelne nazev (prazdny retezec pokud Mode 09 $0A nepodporovan).
 * Seznam je platny az do dalsiho volani obd2_query_supported_pids().
 *
 * @return Ukazatel na interni seznam (nesmi se uvolnovat). Pokud nebyl
 *         proveden query, count == 0.
 */
const obd2_detected_ecu_list_t *obd2_get_detected_ecus(void);

/**
 * @brief Vraci diagnostiku posledniho init/discovery pokusu.
 *
 * Ukazatel je platny po celou dobu behu programu a meni se pri dalsim
 * obd2_init(), obd2_query_supported_pids() nebo low-level raw dotazu.
 */
const obd2_init_diag_t *obd2_get_init_diag(void);

/* ---- Mode 01: Aktualni data ---------------------------------------------- */

/**
 * @brief Cteni surovych bajtu jednoho PID (Mode 01).
 *
 * Odesle pozadavek na fyzickou adresu ridici jednotky, overi SID
 * a PID v odpovedi, a vraci nezpracovane datove bajty.
 * Pro prevod na fyzikalni velicinu pouzijte obd2_decode_pid_value()
 * nebo rovnou obd2_get_pid().
 *
 * @param pid     PID k precteni ($00–$FF)
 * @param result  Vystup: surove datove bajty. Nesmi byt NULL.
 * @return OBD2_OK pri uspechu,
 *         OBD2_ERR_TIMEOUT pri absenci odpovedi,
 *         OBD2_ERR_INVALID_ARG pri NULL ukazateli,
 *         OBD2_ERR_RESPONSE_MALFORMED pri nespravnem formatu odpovedi.
 */
obd2_status_t obd2_get_pid_raw(uint8_t pid, obd2_pid_raw_t *result);

/**
 * @brief Provede manualni dotaz na auto a vrati surova data (bez dekodovani).
 *
 * @param service Service ID (napr. 0x01, 0x09)
 * @param pid     PID (napr. 0x0C)
 * @param out_res Ukazatel na strukturu, kam se ulozi vysledek
 * @return obd2_status_t Stav operace
 */
obd2_status_t obd2_query_raw(uint8_t service, uint8_t pid,
                             obd2_raw_response_t *out_res);

/**
 * @brief Rozsireny manualni dotaz, ktery umoznuje vynutit broadcast (0x7DF).
 *
 * @param service Service ID (napr. 0x01, 0x09)
 * @param pid     PID (napr. 0x0C)
 * @param out_res Ukazatel na strukturu, kam se ulozi vysledek
 * @param use_broadcast true = poslat na 0x7DF, false = poslat na fyzickou adresu (0x7E0)
 * @return obd2_status_t Stav operace
 */
obd2_status_t obd2_query_raw_ex(uint8_t service, uint8_t pid,
                                obd2_raw_response_t *out_res, bool use_broadcast);

/**
 * @brief Cteni a dekodovani jednoho PID (Mode 01).
 *
 * Kombinuje surove cteni + vyhledani v tabulce deskriptoru + aplikaci
 * dekodovaciho vzorce. Pro bitove kodovane PID ($01, $03, $12...)
 * pole value obsahuje surove uint32 pretypovane na float.
 * Pro PID kysllikovych senzoru pole secondary obsahuje druhou hodnotu.
 *
 * @param pid     PID k precteni
 * @param result  Vystup: dekodovana hodnota s nazvem a jednotkou. Nesmi byt
 * NULL.
 * @return OBD2_OK pri uspechu,
 *         OBD2_ERR_DECODE pokud PID neni v dekodovaci tabulce,
 *         OBD2_ERR_UNSUPPORTED_PID pokud ECU PID nepodporuje.
 *
 * Priklad:
 * @code
 *   obd2_pid_decoded_t val;
 *   if (obd2_get_pid(0x0C, &val) == OBD2_OK) {
 *       printf("%s: %.1f %s\n", val.name, val.value, val.unit);
 *       // Vystup: "Engine RPM: 800.0 rpm"
 *   }
 * @endcode
 */
obd2_status_t obd2_get_pid(uint8_t pid, obd2_pid_decoded_t *result);

/**
 * @brief Dekodovani odpovedi PID $01 do struktury stavu monitoru.
 *
 * Lze zavolat se surovymi daty z obd2_get_pid_raw(), nebo s NULL
 * pro automaticke precteni z ridici jednotky.
 *
 * @param raw    Surova 4bajtova data PID $01 (nebo NULL pro cteni z ECU).
 *               Pri NULL funkce internne zavola obd2_get_pid_raw(0x01, ...).
 * @param status Vystup: dekodovany stav monitoru. Nesmi byt NULL.
 * @return OBD2_OK pri uspechu
 *
 * Priklad:
 * @code
 *   obd2_monitor_status_t ms;
 *   obd2_get_monitor_status(NULL, &ms);
 *   printf("MIL: %s, DTC: %d\n", ms.mil_on ? "SVITI" : "nesviti",
 * ms.dtc_count); if (ms.cat_sup && !ms.cat_rdy) { printf("Monitor katalyzatoru
 * jeste nedokoncil test!\n");
 *   }
 * @endcode
 */
obd2_status_t obd2_get_monitor_status(const uint8_t *raw,
                                      obd2_monitor_status_t *status);

/**
 * @brief Precte stav monitoru ze vsech ECU v siti (broadcast).
 *
 * @param list Vystupni struktura pro seznam stavu
 * @return OBD2_OK pri uspechu, jinak chybovy kod
 */
obd2_status_t obd2_get_monitor_status_all(obd2_monitor_status_list_t *list);

/* ---- Mode 02: Zamrzly snimek (Freeze Frame) ------------------------------ */

/**
 * @brief Cteni surovych bajtu PID ze zamrzleho snimku (Mode 02).
 *
 * Freeze frame obsahuje hodnoty parametru v okamziku vzniku DTC.
 * Umoznuje zpetne zjistit, za jakych podminek k zavade doslo
 * (napr. jake byly otacky, teplota, rychlost v okamziku poruchy).
 *
 * @param pid     PID k precteni (stejne PID jako v Mode 01)
 * @param frame   Cislo zamrzleho snimku ($00 = standardni, vetsina ECU
 *                podporuje pouze frame $00)
 * @param result  Vystup: surove datove bajty. Nesmi byt NULL.
 * @return OBD2_OK pri uspechu,
 *         OBD2_ERR_NO_DATA pokud zadny freeze frame neexistuje.
 */
obd2_status_t obd2_get_freeze_frame_raw(uint8_t pid, uint8_t frame,
                                        obd2_pid_raw_t *result);

/* ---- Mode 03: Potvrzene DTC ---------------------------------------------- */

/**
 * @brief Cteni vsech potvrzenych emisnich DTC (Mode 03).
 *
 * Odesle broadcast pozadavek a zpracuje multi-frame odpovedi.
 * DTC jsou dekodovany na retezce typu "Pxxxx"/"Cxxxx"/"Bxxxx"/"Uxxxx".
 *
 * Potvrzene DTC znamena, ze porucha byla detekovana opakovane
 * (obvykle ve 2 jezdnich cyklech) a je ulozena v pameti ECU.
 * Sviti-li kontrolka MIL, je to zpusobeno potvrzenym DTC.
 *
 * @param dtcs       Vystupni pole (minimalne OBD2_MAX_DTC_COUNT prvku).
 *                   Kazdy prvek obsahuje textovy kod a surove bajty.
 * @param max_count  Maximalni pocet DTC k ulozeni do pole.
 * @param out_count  Vystup: skutecny pocet prijatych DTC. Nesmi byt NULL.
 * @return OBD2_OK (i pri 0 DTC — to je platny vysledek, znamena "bez poruch")
 *
 * Priklad:
 * @code
 *   obd2_dtc_t dtcs[OBD2_MAX_DTC_COUNT];
 *   uint8_t count;
 *   if (obd2_read_dtc(dtcs, OBD2_MAX_DTC_COUNT, &count) == OBD2_OK) {
 *       for (int i = 0; i < count; i++) {
 *           printf("DTC %d: %s\n", i+1, dtcs[i].code);
 *       }
 *   }
 * @endcode
 */
obd2_status_t obd2_read_dtc(obd2_dtc_t *dtcs, uint8_t max_count,
                            uint8_t *out_count);

/**
 * @brief Přečte DTC ze všech jednotek ve vozidle (Multi-ECU).
 *
 * Oproti obd2_read_dtc() (ktera data ze vsech ECU spoji do jednoho pole)
 * tato funkce zachovava oddeleni dtc per ECU vcetne CAN ID.
 *
 * @param sid    SID služby: OBD2_SID_READ_DTC (0x03) nebo OBD2_SID_PENDING_DTC
 * (0x07)
 * @param result Výstupní struktura pro uložení všech DTC od všech ECU
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
obd2_status_t obd2_read_dtc_multi(uint8_t sid, obd2_multi_ecu_dtc_t *result);

/* ---- Mode 04: Mazani DTC ------------------------------------------------- */

/**
 * @brief Vymazani vsech emisnich diagnostickych informaci (Mode 04).
 *
 * Maze: DTC, freeze frame, pripravenost monitoru (I/M readiness),
 * vysledky monitoru, pocitadla vzdalenosti/casu od MIL a od smazani DTC.
 * Po smazani zhasne kontrolka MIL.
 *
 * POZOR: Vyzaduje zapnute zapalovani a vypnuty motor. Pokud je motor
 * v chodu, ridici jednotka odpovi negativni odpovedi (NRC 0x22).
 * Po smazani je nutne projet kompletni jezdni cyklus pro obnovu
 * pripravenosti monitoru (dulezite pred emisni kontrolou STK/ME).
 *
 * @return OBD2_OK pri uspechu,
 *         OBD2_ERR_NEGATIVE_RESP pokud podminky nejsou splneny
 *         (motor bezi, ECU zamitnuto apod.)
 */
obd2_status_t obd2_clear_dtc(void);

/* ---- Mode 07: Cekajici (pending) DTC ------------------------------------- */

/**
 * @brief Cteni cekajicich DTC z aktualniho/posledniho jezdniho cyklu (Mode 07).
 *
 * Cekajici DTC jsou poruchy detekovane v aktualnim nebo poslednim
 * jezdnim cyklu, ktere jeste nebyly potvrzeny (nepredly do Mode 03).
 * Uzitecne pro vcasnou diagnostiku — ukazuji problemy v pocatecni fazi.
 *
 * Format odpovedi je shodny s Mode 03.
 *
 * @param dtcs       Vystupni pole pro DTC.
 * @param max_count  Maximalni pocet DTC k ulozeni.
 * @param out_count  Vystup: skutecny pocet cekajicich DTC.
 * @return OBD2_OK pri uspechu
 */
obd2_status_t obd2_read_pending_dtc(obd2_dtc_t *dtcs, uint8_t max_count,
                                    uint8_t *out_count);

/* ---- Mode 0A: Permanentni DTC ------------------------------------------- */

/**
 * @brief Cteni permanentnich emisnich DTC (Mode 0A).
 *
 * Format odpovedi je pro CAN/ISO-TP shodny s DTC sluzbami $03/$07.
 */
obd2_status_t obd2_read_permanent_dtc(obd2_dtc_t *dtcs, uint8_t max_count,
                                      uint8_t *out_count);

/* ---- Mode 09: Informace o vozidle ---------------------------------------- */

/**
 * @brief Surove cteni libovolneho Mode 09 InfoType ze vsech ECU.
 *
 * Slouzi pro InfoType, ktere dashboard potrebuje zobrazit diagnosticky
 * transparentne (supported InfoTypes, IPT apod.).
 */
obd2_status_t obd2_read_infotype_all(uint8_t infotype,
                                     obd2_infotype_list_t *list);

/**
 * @brief Cteni identifikacniho cisla vozidla — VIN (Mode 09, InfoType $02).
 *
 * VIN ma vzdy presne 17 znaku ASCII dle ISO 3779.
 * Obsahuje informace o vyrobci, modelu, roce vyroby a poradi vyroby.
 * Buffer musi mit alespon 18 bajtu (17 znaku + null terminator).
 *
 * @param vin_buf   Vystupni buffer (zakonceny nulovym znakem).
 *                  Nesmi byt NULL.
 * @param buf_len   Velikost bufferu (minimalne 18 bajtu).
 * @return OBD2_OK pri uspechu,
 *         OBD2_ERR_INVALID_ARG pokud je buffer prilis maly.
 *
 * Priklad:
 * @code
 *   char vin[18];
 *   if (obd2_read_vin(vin, sizeof(vin)) == OBD2_OK) {
 *       printf("VIN: %s\n", vin);  // napr. "WVWZZZ3CZWE123456"
 *   }
 * @endcode
 */
obd2_status_t obd2_read_vin(char *vin_buf, uint8_t buf_len);

/**
 * @brief Precte VIN ze vsech odpovidajicich ECU v siti (Multi-ECU).
 *
 * Vyuziva broadcast adresovani a sesbira vsechny odpovedi. Uzitecne
 * pro identifikaci vsech komponent vozidla, ktere VIN znaji.
 *
 * @param list Vystupni struktura pro seznam nalezenych VIN
 * @return OBD2_OK pri uspechu (alespon jeden VIN nalezen), jinak chybovy kod
 */
obd2_status_t obd2_read_vin_all(obd2_vin_list_t *list);

/**
 * @brief Cteni nazvu/zkratky ridici jednotky (Mode 09, InfoType $0A).
 *
 * Format: "ECM\0-Engine Control\0" (4znakova zkratka + null + '-' + popis).
 * Celkem az 20 bajtu.
 *
 * @param name_buf  Vystupni buffer (zakonceny nulovym znakem). Nesmi byt NULL.
 * @param buf_len   Velikost bufferu (minimalne OBD2_ECU_NAME_MAX_LENGTH + 1 =
 * 21 bajtu).
 * @return OBD2_OK pri uspechu
 *
 * Priklad:
 * @code
 *   char name[OBD2_ECU_NAME_MAX_LENGTH + 1];
 *   if (obd2_read_ecu_name(name, sizeof(name)) == OBD2_OK) {
 *       printf("ECU: %s\n", name);  // napr. "ECM"
 *   }
 * @endcode
 */
obd2_status_t obd2_read_ecu_name(char *name_buf, uint8_t buf_len);

/**
 * @brief Precte nazvy vsech ECU v siti (broadcast).
 *
 * @param list Vystupni struktura pro seznam nazvu
 * @return OBD2_OK pri uspechu, jinak chybovy kod
 */
obd2_status_t obd2_read_ecu_names_all(obd2_ecu_name_list_t *list);

/**
 * @brief Cteni kalibracnich identifikatoru — CalID (Mode 09, InfoType $04).
 *
 * Kazdy CalID ma 16 znaku (doplnenych nulami). Bajt NODI v odpovedi
 * urcuje, kolik CalID ridici jednotka vraci (obvykle 1-2).
 *
 * @param cal_ids    Vystupni 2D pole
 * [OBD2_MAX_INFO_ITEMS][OBD2_CAL_ID_LENGTH+1]. Kazdy radek obsahuje jeden CalID
 * zakonceny nulovym znakem.
 * @param out_count  Vystup: pocet prijatych CalID. Nesmi byt NULL.
 * @return OBD2_OK pri uspechu
 *
 * Priklad:
 * @code
 *   char cal_ids[OBD2_MAX_INFO_ITEMS][OBD2_CAL_ID_LENGTH + 1];
 *   uint8_t count;
 *   if (obd2_read_cal_id(cal_ids, &count) == OBD2_OK) {
 *       for (int i = 0; i < count; i++) {
 *           printf("CalID %d: %s\n", i+1, cal_ids[i]);
 *       }
 *   }
 * @endcode
 */
obd2_status_t obd2_read_cal_id(char cal_ids[][OBD2_CAL_ID_LENGTH + 1],
                               uint8_t *out_count);

/**
 * @brief Precte CalID ze vsech ECU v siti (broadcast).
 *
 * @param list Vystupni struktura pro seznam vsech CalID
 * @return OBD2_OK pri uspechu, jinak chybovy kod
 */
obd2_status_t obd2_read_calids_all(obd2_calid_list_t *list);

/**
 * @brief Precte CVN hodnoty ze vsech ECU v siti (broadcast).
 */
obd2_status_t obd2_read_cvns_all(obd2_cvn_list_t *list);

/* ---- Dekodovaci nastroje (bezstavove, pouzitelne i offline) -------------- */

/**
 * @brief Dekodovani surovych bajtu PID na float dle vzorcu z Prilohy B.
 *
 * Cista funkce — zadne I/O, zadny stav. Lze pouzit i pro data
 * ze zamrzleho snimku (freeze frame) nebo pro offline zpracovani
 * zaznamenanych dat.
 *
 * @param pid       Hodnota PID (urcuje, ktery vzorec se pouzije)
 * @param data      Surove datove bajty (1–4 bajty, v poradi A, B, C, D)
 * @param data_len  Pocet datovych bajtu
 * @return Dekodovana hodnota jako float, nebo NAN pokud PID neni v tabulce.
 *
 * Priklad:
 * @code
 *   uint8_t data[] = {0x0C, 0x80};  // PID $0C, otacky
 *   float rpm = obd2_decode_pid_value(0x0C, data, 2);
 *   // rpm = (256*12 + 128) * 0.25 = 800.0
 * @endcode
 */
float obd2_decode_pid_value(uint8_t pid, const uint8_t *data, uint8_t data_len);

/**
 * @brief Dekodovani sekundarni hodnoty pro PID s dvojitym vystupem (O2
 * senzory).
 *
 * Pro $14–$1B: sekundarni = kratkodoba korekce paliva STFT (%).
 * Pro $24–$2B: sekundarni = napeti senzoru (V).
 * Pro $34–$3B: sekundarni = proud senzorem (mA).
 * Pro vsechny ostatni PID: vraci NAN.
 *
 * @param pid       Hodnota PID
 * @param data      Surove datove bajty
 * @param data_len  Pocet datovych bajtu
 * @return Sekundarni dekodovana hodnota, nebo NAN pokud PID nema sekundarni
 * vystup.
 */
float obd2_decode_pid_secondary(uint8_t pid, const uint8_t *data,
                                uint8_t data_len);

/**
 * @brief Dekoduje 3. a 4. hodnotu pro multi-sensor PIDy.
 *
 * Pouziva se pro PIDy formatu OBD2_FMT_TEMP_4S a OBD2_FMT_NOX_4S, ktere
 * obsahuji az 4 hodnoty (typicky 4 teplotni nebo NOx senzory v jednom dotazu).
 *
 * Senzor 1 vraci obd2_decode_pid_value(), senzor 2 vraci
 * obd2_decode_pid_secondary(), senzory 3 a 4 vraci tato funkce do pole extra[].
 *
 * Pokud neni senzor podporovan (support bit v bajtu A je 0) nebo vraci
 * neplatnou hodnotu (0xFFFF u NOx), prislusna pozice extra[] je NAN.
 *
 * @param pid       Cislo PIDu
 * @param data      Raw data byty (alespon 9 bytu pro multi-sensor formaty)
 * @param data_len  Pocet validnich bytu v data[]
 * @param extra     Pole 2 floatu pro senzory 3 a 4 (musi byt non-NULL,
 *                  velikost OBD2_PID_MAX_VALUES - 2 = 2)
 * @return Pocet validnich hodnot v dekodovanem PID (1-4):
 *         1 = jednoduchy skalar (jen value)
 *         2 = O2 senzor (value + secondary) nebo multi-sensor s 1+2 podporovany
 *         3 = multi-sensor se senzory 1+2+3
 *         4 = vsechny 4 senzory podporovany
 */
uint8_t obd2_decode_pid_extras(uint8_t pid, const uint8_t *data,
                               uint8_t data_len, float *extra);

/**
 * @brief Dekodovani 2 surovych bajtu DTC na lidsky citelny retezec.
 *
 * Horni 2 bity prvniho bajtu urcuji kategorii:
 *   00 = P (Powertrain — hnaci soustava)
 *   01 = C (Chassis — podvozek)
 *   10 = B (Body — karoserie)
 *   11 = U (Network — komunikacni sit)
 * Zbylych 14 bitu tvori 4 hexadecimalni cislice kodu.
 *
 * Priklad: surove bajty [0x01, 0x43] → "P0143"
 *   Prvni bajt: 0x01 = 0000_0001 → horni 2 bity = 00 → "P"
 *   Zbylych 14 bitu: 00_0001_0100_0011 = 0x0143
 *   Vysledek: "P0143"
 *
 * @param raw  2bajtovy DTC z ridici jednotky. Nesmi byt NULL.
 * @param out  Vystupni buffer pro retezec (minimalne 6 bajtu: "P0143\0"). Nesmi
 * byt NULL.
 */
void obd2_decode_dtc_string(const uint8_t *raw, char *out);

/**
 * @brief Vyhledani deskriptoru PID v interni tabulce.
 *
 * Vraci ukazatel na staticke data — nealokuje pamet, ukazatel
 * je platny po celou dobu behu programu.
 *
 * @param pid  Hodnota PID ($00–$5F pro standardni PID z Prilohy B)
 * @return Ukazatel na deskriptor, nebo NULL pokud PID neni v tabulce.
 *
 * Priklad:
 * @code
 *   const obd2_pid_desc_t *desc = obd2_get_pid_descriptor(0x0C);
 *   if (desc) {
 *       printf("PID $%02X: %s [%s], %d bajtu\n",
 *              desc->pid, desc->name, desc->unit, desc->data_len);
 *       // Vystup: "PID $0C: Engine RPM [rpm], 2 bajtu"
 *   }
 * @endcode
 */
const obd2_pid_desc_t *obd2_get_pid_descriptor(uint8_t pid);

/**
 * @brief Vraci lidsky citelny nazev stavoveho kodu.
 *
 * Uzitecne pro logovani a ladeni. Vraci staticky retezec
 * (neni treba uvolnovat pamet).
 *
 * @param status  Stavovy kod (napr. OBD2_OK, OBD2_ERR_TIMEOUT)
 * @return Textovy retezec, napr. "OBD2_OK", "OBD2_ERR_TIMEOUT"
 */
const char *obd2_status_str(obd2_status_t status);

/**
 * @brief Vraci lidsky citelny nazev kodu negativni odpovedi (NRC).
 *
 * Uzitecne pro logovani a ladeni. Vraci staticky retezec.
 *
 * @param nrc  Kod negativni odpovedi (napr. 0x11, 0x22)
 * @return Textovy retezec, napr. "ServiceNotSupported", "ConditionsNotCorrect"
 */
const char *obd2_nrc_str(uint8_t nrc);

#ifdef __cplusplus
}
#endif

#endif /* OBD2_H */
