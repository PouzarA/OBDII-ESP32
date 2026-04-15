/**
 * @file isotp.h
 * @brief Transportní protokol ISO 15765-2 (ISO-TP) nad ESP32 TWAI (CAN).
 *
 * Tato hlavička definuje veřejné API transportní vrstvy ISO-TP pro
 * diagnostickou komunikaci OBD-II. Vrstva sedí mezi datovou vrstvou
 * CAN (TWAI driver v ESP-IDF) a aplikační vrstvou OBD-II (služby
 * Mode 01, Mode 03, Mode 09 atd.).
 *
 * ISO-TP segmentuje diagnostické zprávy delší než 7 bajtů do více
 * CAN rámců a na přijímací straně je opět skládá dohromady. Vrstva
 * rozlišuje čtyři typy rámců (PCI — Protocol Control Information):
 *   - SF (Single Frame)      — kompletní zpráva do 7 B užitečných dat
 *   - FF (First Frame)       — první rámec multi-frame přenosu
 *   - CF (Consecutive Frame) — následující rámce multi-frame přenosu
 *   - FC (Flow Control)      — řízení toku (CTS / WAIT / OVFLW)
 *
 * Odkazy na specifikaci:
 *   - ISO 15765-2:2016 — Transportní protokol a služby síťové vrstvy
 *   - ISO 15765-4:2005 — Požadavky na emisně relevantní systémy (OBD-II)
 *
 * @author Aleš (bakalářský projekt OBD-II)
 */

#ifndef ISOTP_H
#define ISOTP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  Konfigurace — případně přepište pomocí #define před #include             */
/* ========================================================================= */

/**
 * @brief Maximální velikost ISO-TP payloadu v bajtech.
 *
 * Teoretický limit ISO-TP je 4095 B, pro OBD-II ale stačí méně:
 *   - VIN (Mode 09 PID 02) = 20 B
 *   - 126 DTC kódů (Mode 03) = 252 B
 * Hodnota 256 B je bezpečný kompromis mezi pamětí a funkčností.
 */
#ifndef ISOTP_MAX_PAYLOAD
#define ISOTP_MAX_PAYLOAD           256
#endif

/**
 * @brief Výplňový bajt pro nevyužité pozice v CAN rámci.
 *
 * CAN rámec má pevnou délku 8 B; pokud ISO-TP naplní méně, zbylé
 * bajty se vyplňují konstantou. Hodnota 0xCC je běžná konvence
 * (používá ji například ELM327).
 */
#ifndef ISOTP_PADDING_BYTE
#define ISOTP_PADDING_BYTE          0xCC
#endif

/**
 * @brief Velikost TWAI RX fronty (počet CAN rámců).
 *
 * Na reálné CAN sběrnici v autě (500 kbit/s) může být 50+ non-OBD rámců
 * za sekundu (ABS, airbag, klima, BCM...). Když je fronta plná, TWAI
 * driver nové rámce tiše zahazuje — mezi nimi může být i naše OBD
 * odpověď.
 *
 * Každý slot zabírá ~16 bajtů (twai_message_t). Paměťová náročnost:
 *   32 slotů =  ~512 B  (nedostatečné pro vytíženou sběrnici)
 *   64 slotů = ~1024 B  (bezpečné minimum pro OBD-II)
 *  128 slotů = ~2048 B  (pro velmi zatížené sběrnice)
 *
 * Výchozí hodnota 64 je kompromis mezi spolehlivostí a spotřebou RAM
 * na ESP32 (520 KB SRAM).
 */
#ifndef ISOTP_TWAI_RX_QUEUE_LEN
#define ISOTP_TWAI_RX_QUEUE_LEN    64
#endif

/* ---- Časování (ms) — dle ISO 15765-4:2005 Table 6 --------------------- */

/** @brief N_As — maximální doba odeslání jednoho CAN rámce (vysílač). */
#define ISOTP_N_AS_TIMEOUT_MS       25
/** @brief N_Ar — maximální doba přijetí jednoho CAN rámce (přijímač). */
#define ISOTP_N_AR_TIMEOUT_MS       25
/** @brief N_Bs — timeout čekání na Flow Control po odeslání First Frame. */
#define ISOTP_N_BS_TIMEOUT_MS       75
/** @brief N_Cr — timeout čekání na další Consecutive Frame. */
#define ISOTP_N_CR_TIMEOUT_MS       150

/* ---- Parametry Flow Control — dle ISO 15765-4:2005 Table 7 ------------ */

/** @brief BlockSize = 0: posílat všechny CF po jediném FC bez dalšího FC. */
#define ISOTP_FC_BS                 0
/** @brief STmin = 0: posílat CF tak rychle, jak to jde (bez mezery). */
#define ISOTP_FC_STMIN              0

/* ---- Konstanty CAN ID pro OBD-II — dle ISO 15765-4:2005 Table 3 ------- */

/** @brief Funkční broadcast request ID (11-bit) — dotaz na všechny ECU. */
#define ISOTP_OBD_FUNC_REQ_ID      0x7DF
/** @brief Počátek fyzického rozsahu request ID (ECU #1 = 0x7E0). */
#define ISOTP_OBD_PHYS_REQ_BASE    0x7E0
/** @brief Počátek fyzického rozsahu response ID (ECU #1 = 0x7E8). */
#define ISOTP_OBD_PHYS_RESP_BASE   0x7E8
/** @brief Maximální počet ECU odpovídajících na broadcast dle ISO 15765-4. */
#define ISOTP_MAX_ECU_RESPONSES     8

/* ---- Konstanty CAN ---------------------------------------------------- */

/** @brief DLC (Data Length Code) standardního CAN rámce — vždy 8 B. */
#define ISOTP_CAN_DLC               8
/** @brief Maximální počet bajtů užitečných dat v rámci SF (1 B PCI + 7 B data). */
#define ISOTP_CAN_MAX_DATA          7   /* SF payload max */
/** @brief Počet bajtů užitečných dat v rámci FF (2 B PCI + 6 B data). */
#define ISOTP_CAN_FF_DATA           6   /* FF první bajty payloadu */
/** @brief Počet bajtů užitečných dat v každém rámci CF (1 B PCI + 7 B data). */
#define ISOTP_CAN_CF_DATA           7   /* CF bajty payloadu na rámec */

/* ========================================================================= */
/*  Logování                                                                 */
/* ========================================================================= */

/**
 * @brief Úrovně závažnosti logovacích zpráv.
 *
 * Hodnoty jsou řazeny vzestupně podle upovídanosti. NONE vypíná
 * veškerý výstup, TRACE produkuje nejvíce detailů (každý CAN rámec).
 */
typedef enum {
    ISOTP_LOG_NONE  = 0,
    ISOTP_LOG_ERROR = 1,
    ISOTP_LOG_WARN  = 2,
    ISOTP_LOG_INFO  = 3,
    ISOTP_LOG_DEBUG = 4,
    ISOTP_LOG_TRACE = 5
} isotp_log_level_t;

/**
 * @brief Maximální úroveň logování zapečená v době překladu.
 *
 * Zprávy s vyšší úrovní jsou při kompilaci odstraněny (šetří flash).
 * Pro omezení je třeba makro nadefinovat před #include této hlavičky.
 */
#ifndef ISOTP_LOG_MAX_LEVEL
#define ISOTP_LOG_MAX_LEVEL         ISOTP_LOG_TRACE
#endif

/* Prefixy logovacích úrovní (indexovány hodnotou isotp_log_level_t). */
static const char *_isotp_log_prefix[] = {
    "---", "ERR", "WRN", "INF", "DBG", "TRC"
};

/* Runtime úroveň logu — nastavuje se funkcí isotp_set_log_level(). */
extern isotp_log_level_t _isotp_runtime_log_level;

/**
 * @brief Hlavní logovací makro — kontroluje jak překladovou, tak
 *        runtime úroveň logování.
 */
#define ISOTP_LOG(level, fmt, ...) \
    do { \
        if ((level) <= ISOTP_LOG_MAX_LEVEL && \
            (level) <= _isotp_runtime_log_level) { \
            printf("[ISOTP %s] " fmt "\n", \
                   _isotp_log_prefix[(level)], ##__VA_ARGS__); \
        } \
    } while(0)

/** @brief Zkratková makra pro jednotlivé úrovně logování. */
#define ISOTP_LOGE(fmt, ...) ISOTP_LOG(ISOTP_LOG_ERROR, fmt, ##__VA_ARGS__)
#define ISOTP_LOGW(fmt, ...) ISOTP_LOG(ISOTP_LOG_WARN,  fmt, ##__VA_ARGS__)
#define ISOTP_LOGI(fmt, ...) ISOTP_LOG(ISOTP_LOG_INFO,  fmt, ##__VA_ARGS__)
#define ISOTP_LOGD(fmt, ...) ISOTP_LOG(ISOTP_LOG_DEBUG, fmt, ##__VA_ARGS__)
#define ISOTP_LOGT(fmt, ...) ISOTP_LOG(ISOTP_LOG_TRACE, fmt, ##__VA_ARGS__)

/* ========================================================================= */
/*  Datové typy                                                              */
/* ========================================================================= */

/**
 * @brief Typ ISO-TP rámce — horní nibble prvního PCI bajtu.
 *
 * Viz ISO 15765-2 odstavec 9.4. Hodnoty odpovídají přímo bajtu na
 * sběrnici (není třeba posouvat), např. 0x10 = First Frame.
 * Příklady: SF = 0x0n, FF = 0x1n, CF = 0x2n, FC = 0x3n,
 * kde n je pomocné pole (délka / SN / FlowStatus).
 */
typedef enum {
    ISOTP_PCI_SF = 0x00,    /**< Single Frame — samostatná zpráva do 7 B. */
    ISOTP_PCI_FF = 0x10,    /**< First Frame — první rámec multi-frame. */
    ISOTP_PCI_CF = 0x20,    /**< Consecutive Frame — navazující rámec. */
    ISOTP_PCI_FC = 0x30     /**< Flow Control — řízení toku od přijímače. */
} isotp_pci_type_t;

/**
 * @brief Hodnoty pole FlowStatus v Flow Control rámci.
 *
 * Viz ISO 15765-2 odstavec 9.6.5. Přijímač posílá tyto stavy
 * odesílateli po přijetí First Frame nebo po vyčerpání BlockSize.
 */
typedef enum {
    ISOTP_FC_CTS       = 0x00,  /**< Continue To Send — můžeš posílat CF. */
    ISOTP_FC_WAIT      = 0x01,  /**< Wait — počkej na další FC. */
    ISOTP_FC_OVERFLOW  = 0x02   /**< Overflow — zpráva se nevejde, přeruš přenos. */
} isotp_fc_status_t;

/**
 * @brief Návratové kódy ISO-TP operací.
 *
 * ISOTP_OK = 0 (úspěch). Ostatní hodnoty signalizují konkrétní
 * chybu; pro textový popis slouží isotp_status_str().
 */
typedef enum {
    ISOTP_OK             = 0,   /**< Úspěch. */
    ISOTP_ERR_TIMEOUT,          /**< Vypršel timeout N_Bs nebo N_Cr. */
    ISOTP_ERR_OVERFLOW,         /**< FF_DL přesahuje ISOTP_MAX_PAYLOAD. */
    ISOTP_ERR_SEQUENCE,         /**< Chybné sekvenční číslo (SN) u CF. */
    ISOTP_ERR_UNEXPECTED,       /**< Neočekávaný typ PCI rámce. */
    ISOTP_ERR_CAN_TX,           /**< Selhání twai_transmit(). */
    ISOTP_ERR_CAN_RX,           /**< Selhání twai_receive(). */
    ISOTP_ERR_INVALID_ARG,      /**< NULL ukazatel nebo len mimo rozsah. */
    ISOTP_ERR_NOT_INITIALIZED   /**< Nebyl zavolán isotp_init(). */
} isotp_status_t;

/**
 * @brief Odpověď jedné ECU (používá se uvnitř broadcast výsledku).
 *
 * Struktura drží kompletně poskládaný payload (pokud přenos uspěl)
 * spolu s CAN ID zdrojové ECU. Pole `valid` rozlišuje úspěšně
 * přijaté odpovědi od částečných / ztracených.
 *
 * Příklad: pro ECM odpovídající na 0x7E8 bude rx_id = 0x7E8.
 */
typedef struct {
    uint32_t  rx_id;                        /**< Zdrojové CAN ID (0x7E8..0x7EF). */
    uint8_t   data[ISOTP_MAX_PAYLOAD];      /**< Poskládaný payload odpovědi. */
    uint16_t  len;                          /**< Skutečná délka payloadu. */
    bool      valid;                        /**< true = úspěšně přijato. */
} isotp_response_t;

/**
 * @brief Výsledek broadcast transakce — může obsahovat více ECU odpovědí.
 *
 * Protože na funkční dotaz 0x7DF odpovídá každé ECU samostatně
 * (a každá s vlastním response ID z rozsahu 0x7E8–0x7EF), drží
 * struktura pole až ISOTP_MAX_ECU_RESPONSES odpovědí.
 *
 * Pole `count` udává, kolik odpovědí bylo reálně přijato do vypršení
 * časového okna. `status` popisuje celkový výsledek (OK pokud přišla
 * alespoň jedna platná odpověď).
 */
typedef struct {
    isotp_response_t  responses[ISOTP_MAX_ECU_RESPONSES];
    uint8_t           count;                /**< Počet ECU, které odpověděly. */
    isotp_status_t    status;               /**< Celkový výsledek operace. */
} isotp_result_t;

/* ========================================================================= */
/*  Veřejné API                                                              */
/* ========================================================================= */

/**
 * @brief Inicializuje TWAI driver pro komunikaci ISO-TP.
 *
 * Nakonfiguruje GPIO piny pro CAN TX/RX, nastaví rychlost sběrnice
 * a spustí TWAI ovladač v normálním módu. Musí být zavoláno dřív
 * než jakákoli jiná funkce této vrstvy.
 *
 * @param baudrate  Rychlost CAN sběrnice v bit/s (500000 nebo 250000).
 * @param tx_pin    Číslo GPIO pro CAN TX.
 * @param rx_pin    Číslo GPIO pro CAN RX.
 * @return ISOTP_OK při úspěchu, ISOTP_ERR_CAN_TX / ISOTP_ERR_CAN_RX
 *         při chybě konfigurace ovladače.
 *
 * @note Volání opakovaně bez mezikroku isotp_deinit() skončí chybou.
 * @warning Piny TX a RX musí být zapojeny do externího CAN transceiveru
 *          (např. TJA1050, SN65HVD230) — ESP32 neumí CAN fyzickou vrstvu přímo.
 *
 * @code
 *   if (isotp_init(500000, GPIO_NUM_5, GPIO_NUM_4) != ISOTP_OK) {
 *       // zpracovat chybu
 *   }
 * @endcode
 */
isotp_status_t isotp_init(uint32_t baudrate, int tx_pin, int rx_pin);

/**
 * @brief Ukončí TWAI driver.
 *
 * Bezpečné zavolat i pokud nebyl driver inicializován — funkce
 * v tom případě nic neudělá.
 */
void isotp_deinit(void);

/**
 * @brief Nastaví runtime úroveň logování.
 *
 * Běžná úroveň je omezena rovněž překladovou konstantou
 * ISOTP_LOG_MAX_LEVEL — menší z obou úrovní vyhrává.
 *
 * @param level  Požadovaná úroveň (ISOTP_LOG_NONE až ISOTP_LOG_TRACE).
 */
void isotp_set_log_level(isotp_log_level_t level);

/**
 * @brief Transakce se samostatnou ECU: odešle požadavek a přijme odpověď.
 *
 * Pošle požadavek jako Single Frame na `tx_id` (typicky 0x7E0–0x7E7),
 * čeká na kompletní odpověď z `rx_id` (typicky tx_id + 8). Funkce
 * transparentně zvládne jak SF odpovědi, tak víceframové FF+CF
 * (včetně odeslání Flow Control rámce s BS = ISOTP_FC_BS a
 * STmin = ISOTP_FC_STMIN).
 *
 * @param tx_id       Fyzické request CAN ID (např. 0x7E0 pro ECM).
 * @param rx_id       Očekávané response CAN ID (např. 0x7E8).
 * @param request     Payload požadavku (Mode + PID, max 7 B).
 * @param req_len     Délka payloadu požadavku (1..7).
 * @param response    Výstupní buffer (minimálně ISOTP_MAX_PAYLOAD B).
 * @param resp_len    Výstup: skutečná délka přijaté odpovědi.
 * @param timeout_ms  Maximální čekání na první rámec odpovědi.
 * @return ISOTP_OK při úspěchu, jinak konkrétní chybový kód.
 *
 * @note Pro multi-frame odpovědi se celkový čas skládá z `timeout_ms`
 *       (první rámec) plus N_Cr na každý následující CF.
 * @warning Pokud ECU hlásí FF_DL > ISOTP_MAX_PAYLOAD, funkce vrátí
 *          ISOTP_ERR_OVERFLOW a přenos přeruší (bez odeslání FC).
 *
 * @code
 *   uint8_t req[] = { 0x01, 0x0C };  // Mode 01 PID 0C — otáčky
 *   uint8_t rsp[ISOTP_MAX_PAYLOAD];
 *   uint16_t rsp_len = 0;
 *   isotp_status_t s = isotp_transaction(
 *       0x7E0, 0x7E8, req, sizeof(req),
 *       rsp, &rsp_len, 100);
 *   if (s == ISOTP_OK) {
 *       // rsp[0..rsp_len-1] obsahuje odpověď (0x41 0x0C ...)
 *   }
 * @endcode
 */
isotp_status_t isotp_transaction(
    uint32_t tx_id, uint32_t rx_id,
    const uint8_t *request, uint8_t req_len,
    uint8_t *response, uint16_t *resp_len,
    uint32_t timeout_ms
);

/**
 * @brief Broadcast transakce: pošle na 0x7DF a sesbírá odpovědi všech ECU.
 *
 * Odešle požadavek jako Single Frame na funkční ID 0x7DF a poté
 * naslouchá na celém rozsahu 0x7E8–0x7EF dokud nevyprší `timeout_ms`.
 * Umí zpracovat směs SF a FF+CF odpovědí z různých ECU paralelně
 * (každá ECU má vlastní stavový automat ISO-TP).
 *
 * @param request     Payload požadavku (Mode + PID, max 7 B).
 * @param req_len     Délka payloadu požadavku (1..7).
 * @param result      Výstup: struktura se všemi ECU odpověďmi.
 * @param timeout_ms  Celkové okno pro sběr odpovědí.
 * @return ISOTP_OK pokud odpověděla aspoň jedna ECU, jinak chyba.
 *
 * @note Pro každou víceframovou odpověď je nutné poslat samostatný
 *       Flow Control rámec na odpovídající response ID + 0 offset
 *       (cílem je request ID té dané ECU, tj. rx_id - 8).
 * @note Okrajové případy:
 *       - Žádná ECU neodpověděla → ISOTP_ERR_TIMEOUT, count = 0.
 *       - Více než ISOTP_MAX_ECU_RESPONSES ECU → přebývající se ignorují.
 *       - Jedna ECU pošle overflow → zaznamená se valid = false.
 * @warning Při broadcast není možné použít STmin od dané ECU globálně;
 *          vrstva posílá FC s ISOTP_FC_STMIN = 0 ke každé ECU zvlášť.
 *
 * @code
 *   uint8_t req[] = { 0x09, 0x02 };  // Mode 09 PID 02 — VIN
 *   isotp_result_t result;
 *   if (isotp_transaction_broadcast(req, sizeof(req), &result, 500) == ISOTP_OK) {
 *       for (uint8_t i = 0; i < result.count; i++) {
 *           if (result.responses[i].valid) {
 *               // ECU result.responses[i].rx_id odpověděla
 *           }
 *       }
 *   }
 * @endcode
 */
isotp_status_t isotp_transaction_broadcast(
    const uint8_t *request, uint8_t req_len,
    isotp_result_t *result,
    uint32_t timeout_ms
);

/**
 * @brief Vrací textový (human-readable) popis návratového kódu.
 *
 * Užitečné pro logování a diagnostické výpisy.
 *
 * @param status  Návratový kód ISO-TP operace.
 * @return Ukazatel na statický řetězec (není třeba uvolňovat).
 */
const char *isotp_status_str(isotp_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* ISOTP_H */
