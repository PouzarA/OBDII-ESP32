/**
 * @file obd2.h
 * @brief Verejne API diagnostickeho protokolu OBD-II pro ESP32
 *
 * Tento hlavickovy soubor definuje kompletni rozhrani vrstvy OBD-II,
 * ktera implementuje diagnosticke sluzby dle normy ISO 15031-5:2006.
 * Podporovane sluzby (mody):
 *   - $01: Aktualni diagnosticka data hnaci soustavy (otacky, teplota, rychlost...)
 *   - $02: Data zamrzleho snimku (freeze frame) - hodnoty v okamziku vzniku zavady
 *   - $03: Cteni potvrzenych diagnostickych poruchovych kodu (DTC)
 *   - $04: Mazani/reset emisnich diagnostickych informaci
 *   - $07: Cteni cekajicich (pending) DTC z aktualniho/posledniho jezdniho cyklu
 *   - $09: Informace o vozidle (VIN, kalibracni ID, nazev ridici jednotky)
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
 * @author Ales (bakalarska prace - OBD-II diagnosticky projekt)
 */

#ifndef OBD2_H
#define OBD2_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "isotp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  Konfigurace                                                              */
/* ========================================================================= */

/**
 * Vychozi casovy limit pro odpoved ridici jednotky (v milisekundach).
 *
 * Dle normy ISO 15031-5 je maximalni doba odezvy P2CAN = 50 ms,
 * ale v praxi nektere ridici jednotky (zejmena starsi nebo vytizene)
 * potrebuji vice casu. Hodnota 2000 ms poskytuje dostatecnou rezervu
 * i pro pomale ridici jednotky, multi-frame odpovedi pres ISO-TP
 * a situace s vysokym zatizenim CAN sbernice.
 *
 * Lze prepsat definici pred vlozenim tohoto hlavickoveho souboru,
 * napr.: #define OBD2_DEFAULT_TIMEOUT_MS 5000
 */
#ifndef OBD2_DEFAULT_TIMEOUT_MS
#define OBD2_DEFAULT_TIMEOUT_MS     2000
#endif

/**
 * Maximalni pocet diagnostickych poruchovych kodu (DTC), ktere lze
 * ulozit pri jednom cteni (Mode 03 nebo Mode 07).
 *
 * Kazdy DTC zabira 2 bajty v odpovedi. Pri 126 DTC je to 252 bajtu,
 * coz se bezpecne vejde do jednoho bajtu pro citac (max 255).
 * V praxi vozidla zridka obsahuji vice nez 20-30 aktivnich DTC,
 * takze 126 je vice nez dostatecna rezerva.
 */
#ifndef OBD2_MAX_DTC_COUNT
#define OBD2_MAX_DTC_COUNT          126
#endif

/**
 * Delka VIN (Vehicle Identification Number) je vzdy presne 17 znaku ASCII.
 * Tato hodnota je dana normou ISO 3779 a je stejna pro vsechna vozidla
 * na svete. Priklad VIN: "WVWZZZ3CZWE123456".
 * Pozor: buffer pro VIN musi mit alespon 18 bajtu (17 znaku + null terminator).
 */
#define OBD2_VIN_LENGTH             17

/**
 * Maximalni delka nazvu ridici jednotky (ECU name) v bajtech.
 *
 * Format dle normy: 4znakova zkratka + null + '-' + az 15 znaku popisu.
 * Priklad: "ECM\0-Engine Control\0" (ridici jednotka motoru).
 * Celkem az 20 bajtu.
 */
#define OBD2_ECU_NAME_MAX_LENGTH    20

/**
 * Delka jednoho kalibracniho identifikatoru (CalID).
 *
 * Kazdy CalID ma presne 16 znaku (doplnenych nulami na konci).
 * CalID identifikuje konkretni verzi softwaru/kalibrace ridici jednotky.
 * Priklad: "TNKVWAG011234567" (16 znaku).
 */
#define OBD2_CAL_ID_LENGTH          16

/**
 * Maximalni pocet polozek CalID nebo CVN (Calibration Verification Number)
 * z jedne ridici jednotky.
 *
 * Jedna ridici jednotka muze mit vice kalibraci (napr. zakladni SW
 * a aplikacni SW), kazda s vlastnim CalID a overovacim cislem CVN.
 * Hodnota 4 pokryva bezne pripady; vetsina ECU ma 1-2 kalibrace.
 */
#define OBD2_MAX_INFO_ITEMS         4

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
#define OBD2_SID_CURRENT_DATA       0x01

/**
 * Sluzba $02 — Pozadavek na data zamrzleho snimku (freeze frame).
 *
 * Dle ISO 15031-5 sekce 7.2: Freeze frame obsahuje "snimek" hodnot
 * parametru v okamziku, kdy ridici jednotka zaznamenala poruchu (DTC).
 * Format odpovedi je shodny se sluzbou $01, ale data odpovidaji
 * stavu v okamziku vzniku zavady, ne aktualnimu stavu.
 * Navic je nutne specifikovat cislo snimku (frame number, obvykle $00).
 */
#define OBD2_SID_FREEZE_FRAME       0x02

/**
 * Sluzba $03 — Pozadavek na emisni diagnosticke poruchove kody (DTC).
 *
 * Dle ISO 15031-5 sekce 7.3: Vraci seznam vsech potvrzenych (confirmed)
 * emisnich DTC. Potvrzeny DTC znamena, ze porucha byla detekovana
 * v minimalnim poctu jezdnich cyklu (obvykle 2). Kazdy DTC je
 * kodovan jako 2 bajty dle SAE J2012 a dekoduje se na retezec
 * typu "P0143", "C0100", "B0001", "U0100".
 */
#define OBD2_SID_READ_DTC           0x03

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
#define OBD2_SID_CLEAR_DTC          0x04

/**
 * Sluzba $07 — Pozadavek na cekajici (pending) DTC.
 *
 * Dle ISO 15031-5 sekce 7.7: Vraci DTC detekovane v aktualnim
 * nebo poslednim jezdnim cyklu, ktere jeste nejsou potvrzene.
 * Uzitecne pro diagnostiku: ukazuje problemy, ktere se teprve zacinaji
 * projevovat, ale jeste nevedly k rozsviceni kontrolky MIL.
 * Format odpovedi je shodny se sluzbou $03.
 */
#define OBD2_SID_PENDING_DTC        0x07

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
#define OBD2_SID_VEHICLE_INFO       0x09

/**
 * Offset pro kladnou odpoved: SID odpovedi = SID pozadavku + 0x40.
 *
 * Dle ISO 15031-5 sekce 7: Kdyz ridici jednotka uspesne zpracuje
 * pozadavek, odpovida SID zvysenym o 0x40.
 * Priklad: Pozadavek $01 (aktualni data) → Odpoved $41.
 *          Pozadavek $03 (cteni DTC)    → Odpoved $43.
 */
#define OBD2_SID_RESPONSE_OFFSET    0x40

/**
 * Identifikator negativni odpovedi.
 *
 * Kdyz ridici jednotka nemuze zpracovat pozadavek, odpovida ramcem:
 *   [0x7F] [puvodni SID] [NRC kod]
 * Priklad: [7F 01 12] = Sluzba $01, subfunction nepodporovana.
 */
#define OBD2_SID_NEGATIVE_RESPONSE  0x7F

/* ========================================================================= */
/*  Kody negativni odpovedi (NRC — Negative Response Code)                   */
/* ========================================================================= */

/**
 * Obecne odmitnuti — ridici jednotka odmitla pozadavek z neurciteho duvodu.
 * Priklad: Interni chyba ECU, ktera nespadne do zadne konkretni kategorie.
 */
#define OBD2_NRC_GENERAL_REJECT             0x10

/**
 * Sluzba neni podporovana — ridici jednotka nezna pozadovany SID.
 * Priklad: Ridici jednotka prevodovky nepodporuje sluzbu $09 (Vehicle Info).
 */
#define OBD2_NRC_SERVICE_NOT_SUPPORTED      0x11

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
#define OBD2_NRC_CONDITIONS_NOT_CORRECT     0x22

/**
 * Pozadavek mimo rozsah — pozadovany parametr je mimo povoleny rozsah.
 * Priklad: Pozadavek na freeze frame cislo 5, kdyz ECU ulozilo jen frame 0.
 */
#define OBD2_NRC_REQUEST_OUT_OF_RANGE       0x31

/**
 * Odpoved se pripravuje (response pending) — ridici jednotka
 * potrebuje vice casu na zpracovani pozadavku.
 *
 * Toto neni chyba! ECU touto zpravou oznamuje, ze jeste pracuje
 * a aby tester neprerusil spojeni. Po dokonceni odesle skutecnou odpoved.
 * Priklad: Mazani DTC muze trvat dele, ECU posila 7F xx 78 kazdych 50 ms.
 */
#define OBD2_NRC_RESPONSE_PENDING           0x78

/* ========================================================================= */
/*  ID typu informaci pro Mode 09 (ISO 15031-5 sekce 7.9)                    */
/* ========================================================================= */

/**
 * InfoType $00 — Bitmaska podporovanych InfoType.
 * Analogie k PID $00 v Mode 01: kazdy bit urcuje,
 * zda je dany InfoType ($01-$20) podporovan.
 */
#define OBD2_INFOTYPE_SUPPORTED     0x00

/**
 * InfoType $01 — Pocet datovych polozek na zpravy (Message Count / NODI).
 * Urcuje, kolik datovych polozek ocekavat v odpovedi pro dany InfoType.
 * Pouziva se napr. pred ctenim VIN pro zjisteni poctu zprav.
 */
#define OBD2_INFOTYPE_MSG_COUNT     0x01

/**
 * InfoType $02 — Identifikacni cislo vozidla (VIN).
 * VIN ma vzdy 17 znaku ASCII dle ISO 3779. Je jednoznacny
 * identifikator kazdeho vyrobeneho vozidla na svete.
 * Priklad: "WVWZZZ3CZWE123456" (VW Golf, vyrobeny v Nemecku).
 */
#define OBD2_INFOTYPE_VIN           0x02

/**
 * InfoType $04 — Kalibracni identifikator (CalID).
 * Identifikuje verzi softwarove kalibrace ridici jednotky.
 * Kazdy CalID ma 16 znaku. ECU muze mit vice kalibraci
 * (napr. zakladni SW kalibrace a aplikacni kalibrace).
 * Priklad: "TNKVWAG011234567".
 */
#define OBD2_INFOTYPE_CAL_ID        0x04

/**
 * InfoType $06 — Overovaci cislo kalibrace (CVN — Calibration Verification Number).
 * 4bajtovy kontrolni soucet (hash) odpovidajici kalibraci.
 * Sluzi k overeni, ze software ECU nebyl neautorizovane zmenen.
 * Kazdemu CalID odpovida jeden CVN.
 */
#define OBD2_INFOTYPE_CVN           0x06

/**
 * InfoType $08 — Sledovani vykonnosti za provozu (IPT — In-use Performance Tracking).
 * Obsahuje citace dokonceni a podminek pro kazdy emisni monitor
 * (napr. kolikrat byl monitor katalyzatoru dokoncen vs. kolikrat
 * byly splneny podminky pro jeho beh). Pouziva se pri emisnich kontrolach.
 */
#define OBD2_INFOTYPE_IPT           0x08

/**
 * InfoType $0A — Nazev ridici jednotky (ECU name).
 * Format: 4znakova zkratka + null + '-' + textovy popis.
 * Priklad: "ECM\0-Engine Control\0" (ridici jednotka motoru),
 *          "TCM\0-Transmission\0" (ridici jednotka prevodovky).
 */
#define OBD2_INFOTYPE_ECU_NAME      0x0A

/* ========================================================================= */
/*  Logovani (zrcadli vzor z isotp.h)                                        */
/* ========================================================================= */

/**
 * Maximalni uroven logovani nastavena v dobe prekladu.
 *
 * Hierarchie urovni (od nejdulezitejsich):
 *   ISOTP_LOG_NONE  (0) — zadne logy
 *   ISOTP_LOG_ERROR (1) — pouze chyby (selhani komunikace, neplatne odpovedi)
 *   ISOTP_LOG_WARN  (2) — varovani (timeout, nepodporovany PID)
 *   ISOTP_LOG_INFO  (3) — informacni zpravy (uspesne operace, VIN, DTC)
 *   ISOTP_LOG_DEBUG (4) — ladici detaily (obsah pozadavku a odpovedi)
 *   ISOTP_LOG_TRACE (5) — maximalni detail (kazdy bajt, casovani)
 *
 * Zpravy s urovni vyssi nez OBD2_LOG_MAX_LEVEL jsou zcela odstraneny
 * prekladacem (nulovy overhead). Vychozi hodnota ISOTP_LOG_TRACE
 * povoli vsechny urovne; skutecna filtrace probiha za behu.
 */
#ifndef OBD2_LOG_MAX_LEVEL
#define OBD2_LOG_MAX_LEVEL          ISOTP_LOG_TRACE
#endif

/**
 * Aktualni uroven logovani za behu. Zpravy s urovni vyssi nez tato
 * hodnota nebudou vypisovany. Lze menit pomoci obd2_set_log_level().
 */
extern isotp_log_level_t _obd2_runtime_log_level;

/**
 * Hlavni logovaci makro. Vypisuje zpravu pouze pokud uroven splnuje
 * oba limity: kompilacni (OBD2_LOG_MAX_LEVEL) i runtime (_obd2_runtime_log_level).
 *
 * Format vystupu: "[OBD2  UROVEN] zprava\n"
 * Priklad: "[OBD2  INFO] PID $0C: 3200 rpm"
 *
 * Pouziti: OBD2_LOG(ISOTP_LOG_INFO, "Rychlost: %d km/h", speed);
 */
#define OBD2_LOG(level, fmt, ...) \
    do { \
        if ((level) <= OBD2_LOG_MAX_LEVEL && \
            (level) <= _obd2_runtime_log_level) { \
            printf("[OBD2  %s] " fmt "\n", \
                   _isotp_log_prefix[(level)], ##__VA_ARGS__); \
        } \
    } while(0)

/** Logovani chyb — kriticke selhani, ze kterych se nelze zotavit */
#define OBD2_LOGE(fmt, ...) OBD2_LOG(ISOTP_LOG_ERROR, fmt, ##__VA_ARGS__)
/** Logovani varovani — problemy, ktere nebranni dalsi cinnosti */
#define OBD2_LOGW(fmt, ...) OBD2_LOG(ISOTP_LOG_WARN,  fmt, ##__VA_ARGS__)
/** Logovani informaci — dulezite udalosti behem normalni cinnosti */
#define OBD2_LOGI(fmt, ...) OBD2_LOG(ISOTP_LOG_INFO,  fmt, ##__VA_ARGS__)
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
     * Priklad: CAN sbernice odpojena, ECU neni napajeno, spatna rychlost sbernice.
     * Reseni: Zkontrolujte fyzicke pripojeni a nastavenou rychlost CAN (250k/500k).
     */
    OBD2_ERR_TIMEOUT,

    /**
     * Negativni odpoved — ridici jednotka odmitla pozadavek a poslala
     * ramec [7F SID NRC]. Konkretni NRC kod lze zjistit pomoci obd2_get_last_nrc().
     * Priklad: Pokus o smazani DTC pri bezicim motoru → NRC 0x22.
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
     *   lambda = (256*A + B) * 2/65535 (rozsah 0–2.0)
     * Bajty CD = napeti:
     *   napeti = (256*C + D) * 8/65535 V (rozsah 0–8.0 V)
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
     *   lambda = (256*A + B) * 2/65535 (rozsah 0–2.0)
     * Bajty CD = proud:
     *   proud = ((int16_t)(256*C + D) - 0x8000) * 128/32768 mA
     *   (rozsah -128 az +128 mA)
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
} obd2_pid_format_t;

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
    uint8_t             pid;        /**< Hodnota PID ($00–$FF) */
    const char         *name;       /**< Lidsky citelny nazev PID (anglicky, dle normy) */
    const char         *unit;       /**< Jednotka zobrazeni ("rpm", "°C", "%", "kPa", "V", ...) */
    uint8_t             data_len;   /**< Ocekavany pocet datovych bajtu v odpovedi (1, 2, nebo 4) */
    obd2_pid_format_t   format;     /**< Typ dekodovaciho vzorce */
    float               multiplier; /**< Nasobitel (skalovaci faktor) ve vzorci */
    float               offset;     /**< Offset (posuv) ve vzorci */
} obd2_pid_desc_t;

/* ========================================================================= */
/*  Datove typy pro vysledky                                                 */
/* ========================================================================= */

/**
 * Surova data PID (odpoved Mode 01 / Mode 02).
 *
 * Obsahuje nezpracovane datove bajty presne tak, jak je prijala
 * ridici jednotka. Pro ziskani fyzikalni veliciny je nutne
 * data dekodovat pomoci obd2_decode_pid_value() nebo pouzit
 * primo obd2_get_pid(), ktera dekodovani provede automaticky.
 */
typedef struct {
    uint8_t     pid;            /**< Pozadovany PID (echo z odpovedi) */
    uint8_t     data[4];        /**< Az 4 datove bajty (oznacovane A, B, C, D dle normy) */
    uint8_t     data_len;       /**< Skutecny pocet prijatych datovych bajtu (1–4) */
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
 *   - Ostatni PID: sekundarni = NAN (neaplikovatelne)
 */
typedef struct {
    uint8_t     pid;            /**< Pozadovany PID */
    float       value;          /**< Primarni dekodovana hodnota (napr. 800.0 pro 800 RPM) */
    float       secondary;      /**< Sekundarni hodnota (O2 senzory), NAN pokud neaplikovatelne */
    const char *name;           /**< Nazev PID z tabulky deskriptoru (napr. "Engine RPM") */
    const char *unit;           /**< Jednotka primarni hodnoty (napr. "rpm", "°C") */
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
    char        code[6];        /**< Textovy DTC, napr. "P0143\0", "C0100\0", "U0401\0" */
    uint8_t     raw[2];         /**< Puvodni 2 surove bajty z ridici jednotky */
} obd2_dtc_t;

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
    uint8_t     request_sid;    /**< SID pozadavku, ktery byl odmitnut (napr. 0x04 pro mazani DTC) */
    uint8_t     nrc;            /**< Kod negativni odpovedi (viz konstanty OBD2_NRC_*) */
} obd2_nrc_info_t;

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
    uint8_t     dtc_count;      /**< Pocet emisnich DTC (bity 0-6 bajtu A, rozsah 0-127) */
    bool        mil_on;         /**< Stav kontrolky MIL: true = sviti (bit 7 bajtu A) */

    /* Prubezne monitory — bezi nepretrzite behem chodu motoru */
    bool        misfire_sup;    /**< Monitor vynechavani zapalovani — podporovan */
    bool        misfire_rdy;    /**< Monitor vynechavani zapalovani — pripraven */
    bool        fuel_sys_sup;   /**< Monitor palivoveho systemu — podporovan */
    bool        fuel_sys_rdy;   /**< Monitor palivoveho systemu — pripraven */
    bool        ccm_sup;        /**< Kompletni monitor komponent (CCM) — podporovan */
    bool        ccm_rdy;        /**< Kompletni monitor komponent (CCM) — pripraven */

    /* Neprubezne monitory — bezi jen pri splneni urcitych podminek */
    bool        cat_sup;        /**< Monitor katalyzatoru — podporovan */
    bool        cat_rdy;        /**< Monitor katalyzatoru — pripraven */
    bool        hcat_sup;       /**< Monitor vyhrivaneho katalyzatoru — podporovan */
    bool        hcat_rdy;       /**< Monitor vyhrivaneho katalyzatoru — pripraven */
    bool        evap_sup;       /**< Monitor systemu odparovani paliva (EVAP) — podporovan */
    bool        evap_rdy;       /**< Monitor systemu odparovani paliva (EVAP) — pripraven */
    bool        air_sup;        /**< Monitor sekundarniho vzduchu — podporovan */
    bool        air_rdy;        /**< Monitor sekundarniho vzduchu — pripraven */
    bool        acrf_sup;       /**< Monitor klimatizace/chladiva — podporovan */
    bool        acrf_rdy;       /**< Monitor klimatizace/chladiva — pripraven */
    bool        o2s_sup;        /**< Monitor kysllikovych senzoru — podporovan */
    bool        o2s_rdy;        /**< Monitor kysllikovych senzoru — pripraven */
    bool        htr_sup;        /**< Monitor vyhrivani kysllikovych senzoru — podporovan */
    bool        htr_rdy;        /**< Monitor vyhrivani kysllikovych senzoru — pripraven */
    bool        egr_sup;        /**< Monitor recirkulace vyfukovych plynu (EGR) — podporovan */
    bool        egr_rdy;        /**< Monitor recirkulace vyfukovych plynu (EGR) — pripraven */
} obd2_monitor_status_t;

/* ========================================================================= */
/*  Verejne API                                                              */
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
 * @brief Nastaveni casoveho limitu pro odpoved (vychozi OBD2_DEFAULT_TIMEOUT_MS).
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
 * @brief Cteni a dekodovani jednoho PID (Mode 01).
 *
 * Kombinuje surove cteni + vyhledani v tabulce deskriptoru + aplikaci
 * dekodovaciho vzorce. Pro bitove kodovane PID ($01, $03, $12...)
 * pole value obsahuje surove uint32 pretypovane na float.
 * Pro PID kysllikovych senzoru pole secondary obsahuje druhou hodnotu.
 *
 * @param pid     PID k precteni
 * @param result  Vystup: dekodovana hodnota s nazvem a jednotkou. Nesmi byt NULL.
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
 *   printf("MIL: %s, DTC: %d\n", ms.mil_on ? "SVITI" : "nesviti", ms.dtc_count);
 *   if (ms.cat_sup && !ms.cat_rdy) {
 *       printf("Monitor katalyzatoru jeste nedokoncil test!\n");
 *   }
 * @endcode
 */
obd2_status_t obd2_get_monitor_status(const uint8_t *raw,
                                       obd2_monitor_status_t *status);

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

/* ---- Mode 09: Informace o vozidle ---------------------------------------- */

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
 * @brief Cteni nazvu/zkratky ridici jednotky (Mode 09, InfoType $0A).
 *
 * Format: "ECM\0-Engine Control\0" (4znakova zkratka + null + '-' + popis).
 * Celkem az 20 bajtu.
 *
 * @param name_buf  Vystupni buffer (zakonceny nulovym znakem). Nesmi byt NULL.
 * @param buf_len   Velikost bufferu (minimalne OBD2_ECU_NAME_MAX_LENGTH + 1 = 21 bajtu).
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
 * @brief Cteni kalibracnich identifikatoru — CalID (Mode 09, InfoType $04).
 *
 * Kazdy CalID ma 16 znaku (doplnenych nulami). Bajt NODI v odpovedi
 * urcuje, kolik CalID ridici jednotka vraci (obvykle 1-2).
 *
 * @param cal_ids    Vystupni 2D pole [OBD2_MAX_INFO_ITEMS][OBD2_CAL_ID_LENGTH+1].
 *                   Kazdy radek obsahuje jeden CalID zakonceny nulovym znakem.
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
 * @brief Dekodovani sekundarni hodnoty pro PID s dvojitym vystupem (O2 senzory).
 *
 * Pro $14–$1B: sekundarni = kratkodoba korekce paliva STFT (%).
 * Pro $24–$2B: sekundarni = napeti senzoru (V).
 * Pro $34–$3B: sekundarni = proud senzorem (mA).
 * Pro vsechny ostatni PID: vraci NAN.
 *
 * @param pid       Hodnota PID
 * @param data      Surove datove bajty
 * @param data_len  Pocet datovych bajtu
 * @return Sekundarni dekodovana hodnota, nebo NAN pokud PID nema sekundarni vystup.
 */
float obd2_decode_pid_secondary(uint8_t pid, const uint8_t *data,
                                 uint8_t data_len);

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
 * @param out  Vystupni buffer pro retezec (minimalne 6 bajtu: "P0143\0"). Nesmi byt NULL.
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
