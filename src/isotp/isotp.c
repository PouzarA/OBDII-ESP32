/**
 * @file isotp.c
 * @brief Implementace transportniho protokolu ISO 15765-2 (ISO-TP)
 *
 * Tento modul implementuje transportni vrstvu ISO-TP nad sbernici CAN.
 * ISO-TP (ISO 15765-2) umoznuje prenos zprav delsich nez 7 bajtu, ktere
 * se nevejdou do jednoho CAN ramce. Podporovane typy ramcu:
 *   - SF (Single Frame)     — krátké zprávy do 7 bajtů payloadu
 *   - FF (First Frame)      — první ramec víceramcové zprávy
 *   - CF (Consecutive Frame)— pokračovací ramce víceramcové zprávy
 *   - FC (Flow Control)     — řízení toku (CTS / WAIT / OVFLW)
 *
 * Modul využívá ESP-IDF TWAI driver (CAN kontrolér na ESP32).
 *
 * @see isotp.h pro dokumentaci verejneho API
 * @author Ales Pouzar, inspirace z normy ISO-15765 a
 * https://github.com/openxc/isotp-c
 */

#include "isotp.h"

/* ========================================================================= */
/*  Interni stav modulu                                                      */
/* ========================================================================= */

/** Runtime log level — ve vychozim nastaveni TRACE (pro vyvoj a ladeni) */
isotp_log_level_t _isotp_runtime_log_level = ISOTP_LOG_TRACE;

/* Prefixy logovacích úrovní (indexovány hodnotou isotp_log_level_t). */
const char *_isotp_log_prefix[] = {"---", "ERR", "WRN", "INF", "DBG", "TRC"};

/** Priznak, zda je TWAI driver nainstalovan a bezi */
static bool _isotp_initialized = false;

/* Forward deklarace — definice je nize v sekci Inicializace */
static bool _isotp_force_cleanup_twai(void);

/* ========================================================================= */
/*  Prevod stavoveho kodu na textovy retezec                                 */
/* ========================================================================= */

/* Tabulka jmen stavovych kodu — poradi musi odpovidat enumu isotp_status_t */
static const char *_isotp_status_names[] = {
    "OK",     "TIMEOUT", "OVERFLOW",    "SEQUENCE", "UNEXPECTED",
    "CAN_TX", "CAN_RX",  "INVALID_ARG", "NOT_INIT"};

/**
 * @brief Prevede stavovy kod ISO-TP na citelny retezec.
 * @param status Stavovy kod typu isotp_status_t
 * @return Retezec s nazvem stavu, nebo "UNKNOWN" pri neznamem kodu
 */
const char *isotp_status_str(isotp_status_t status) {
  if (status >= 0 && status <= ISOTP_ERR_NOT_INITIALIZED) {
    return _isotp_status_names[status];
  }
  return "UNKNOWN";
}

/* ========================================================================= */
/*  Pomocne funkce pro logovani                                              */
/* ========================================================================= */

/**
 * @brief Nastavi aktualni uroven logovani za behu.
 * @param level Pozadovana uroven (ISOTP_LOG_NONE..ISOTP_LOG_TRACE)
 */
void isotp_set_log_level(isotp_log_level_t level) {
  _isotp_runtime_log_level = level;
  ISOTP_LOGI("Log level set to %d (%s)", level, _isotp_log_prefix[level]);
}

/**
 * @brief Vypise CAN ramec jako hexadecimalni vypis (uroven TRACE).
 *
 * Slouzi pro detailni ladici vypis smeru, CAN ID, DLC a obsahu dat.
 * Pokud je aktualni uroven logovani pod TRACE, funkce nic nedela.
 *
 * @param dir  Textovy popis smeru (napr. "TX >>>" nebo "RX <<<")
 * @param id   CAN identifikator
 * @param data Ukazatel na data ramce
 * @param dlc  Data Length Code (pocet platnych bajtu, 0..8)
 */
static void isotp_log_frame(const char *dir, uint32_t id, const uint8_t *data,
                            uint8_t dlc) {
  if (ISOTP_LOG_TRACE > ISOTP_LOG_MAX_LEVEL ||
      ISOTP_LOG_TRACE > _isotp_runtime_log_level) {
    return;
  }

  char hex[ISOTP_CAN_DLC * 3 + 1];
  for (int i = 0; i < dlc && i < ISOTP_CAN_DLC; i++) {
    sprintf(&hex[i * 3], "%02X ", data[i]);
  }
  hex[dlc * 3] = '\0';

  printf("[ISOTP TRC] %s id=0x%03X dlc=%d [%s]\n", dir, (unsigned)id, dlc, hex);
}

/**
 * @brief Vypise buffer dat jako hex (uroven DEBUG).
 *
 * Pouziva se pro logovani kompletniho payloadu (napr. cele ISO-TP zpravy).
 * Aby nedoslo k zahlceni logu, vypisuje se maximalne 32 bajtu; delsi data
 * jsou zkracena a doplnena o "...".
 *
 * @param label Popis bufferu (napr. "Request", "Response")
 * @param data  Ukazatel na data
 * @param len   Skutecna delka dat v bajtech
 */
static void isotp_log_data(const char *label, const uint8_t *data,
                           uint16_t len) {
  if (ISOTP_LOG_DEBUG > ISOTP_LOG_MAX_LEVEL ||
      ISOTP_LOG_DEBUG > _isotp_runtime_log_level) {
    return;
  }

  /* Omezeni na max. 32 bajtu, aby se log nezahltil */
  uint16_t print_len = (len > 32) ? 32 : len;
  char hex[32 * 3 + 4];
  int pos = 0;
  for (uint16_t i = 0; i < print_len; i++) {
    pos += sprintf(&hex[pos], "%02X ", data[i]);
  }
  if (len > 32) {
    pos += sprintf(&hex[pos], "...");
  }
  hex[pos] = '\0';

  printf("[ISOTP DBG] %s (%d bytes): [%s]\n", label, len, hex);
}

/* ========================================================================= */
/*  Pomocne funkce pro praci s TWAI driverem                                 */
/* ========================================================================= */

/**
 * @brief Vyprazdni vsechny cekajici ramce z RX fronty TWAI driveru.
 *
 * Pouziva se pred zahajenim nove transakce, aby se odstranily stare
 * ramce z predchozi komunikace (napr. opozdene odpovedi, ktere uz nejsou
 * relevantni). Bez teto ochrany by mohlo dojit k chybnemu parovani
 * pozadavek-odpoved.
 *
 * @return Pocet ramcu, ktere byly z fronty odstraneny
 */
static int isotp_flush_rx_queue(void) {
  int count = 0;
  twai_message_t dummy;
  while (twai_receive(&dummy, 0) == ESP_OK) {
    count++;
  }
  return count;
}

/**
 * @brief Odeslani CAN ramce s detekci a recovery BUS_OFF stavu.
 *
 * CAN protokol definuje stav BUS_OFF (ISO 11898-1): kdyz uzel odesle
 * prilis mnoho chybnych ramcu (TEC > 255), odpoji se od sbernice.
 * K tomu dochazi typicky kdyz:
 *   - ESP32 vysila, ale auto je vypnute (nikdo nepotvrdi ACK bit)
 *   - Ruseni nebo spatne ukonceni sbernice
 *
 * TWAI driver v BUS_OFF stavu odmita volani twai_transmit() s chybou
 * ESP_ERR_INVALID_STATE (0x103).
 *
 * Recovery sekvence dle ESP-IDF dokumentace:
 *   1. Detekce: twai_get_status_info() -> state == TWAI_STATE_BUS_OFF
 *   2. Recovery: twai_initiate_recovery() — odesle 128 × 11 recessive bitu
 *   3. Cekani: driver prejde TWAI_STATE_RECOVERING -> TWAI_STATE_STOPPED
 *   4. Restart: twai_start() — obnoveni normalniho provozu
 *
 * Pokud recovery selze (napr. sbernice stale neni dostupna), funkce
 * vrati ISOTP_ERR_CAN_TX bez opakovani — volajici se rozhodne co dal.
 * Zamerne se nepokousime o recovery ve smycce, aby se predeslo
 * nekonecnemu blokovani pri trvale odpojene sbernici.
 *
 * @param id   CAN identifikator (11-bit)
 * @param data Ukazatel na 8 bajtu dat CAN ramce
 * @return ISOTP_OK pri uspechu, ISOTP_ERR_CAN_TX pri selhani
 *
 * @note Ramec ma vzdy DLC=8 a padding se doplnuje volajicim.
 * @warning Pri odpojene sbernici muze funkce blokovat az timeout N_As +
 * recovery.
 */
static isotp_status_t isotp_can_send(uint32_t id, const uint8_t *data) {
  twai_message_t msg = {0};
  msg.identifier = id;
  msg.data_length_code = ISOTP_CAN_DLC;
  msg.extd = 0; /* 11-bitove standardni CAN ID (nikoliv extended 29-bit) */
  memcpy(msg.data, data, ISOTP_CAN_DLC);

  isotp_log_frame("TX >>>", id, data, ISOTP_CAN_DLC);

  /* AUDIT OK: N_As = 25 ms použit správně pro TX timeout vysílače,
   * viz ISO 15765-2:2016 Table 5, řádek 192 */
  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(ISOTP_N_AS_TIMEOUT_MS));

  if (err == ESP_ERR_INVALID_STATE) {
    /*
     * Pravdepodobne BUS_OFF — overime stav TWAI driveru.
     * Pokud je skutecne BUS_OFF, pokusime se o recovery a jeden retry.
     */
    twai_status_info_t status;
    twai_get_status_info(&status);

    if (status.state == TWAI_STATE_BUS_OFF) {
      ISOTP_LOGW("BUS_OFF detekovano (TEC=%d REC=%d), spoustim recovery...",
                 (int)status.tx_error_counter, (int)status.rx_error_counter);

      esp_err_t rec_err = twai_initiate_recovery();
      if (rec_err != ESP_OK) {
        ISOTP_LOGE("twai_initiate_recovery selhalo: 0x%X", rec_err);
        return ISOTP_ERR_CAN_TX;
      }

      /*
       * Cekani na dokonceni recovery — driver prejde ze stavu
       * RECOVERING do STOPPED. Pollujeme s timeoutem 500ms
       * (128 × 11 bitu pri 500 kbit/s < 3ms, ale pri odpojene
       * sbernici muze trvat dele).
       */
      uint32_t rec_start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
      bool recovered = false;

      while ((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - rec_start <
             500) {
        vTaskDelay(pdMS_TO_TICKS(10));
        twai_get_status_info(&status);
        if (status.state == TWAI_STATE_STOPPED) {
          recovered = true;
          break;
        }
      }

      if (!recovered) {
        ISOTP_LOGE("BUS_OFF recovery timeout (stav=%d)", (int)status.state);
        return ISOTP_ERR_CAN_TX;
      }

      /* Restart TWAI driveru po uspesnem recovery */
      err = twai_start();
      if (err != ESP_OK) {
        ISOTP_LOGE("twai_start po recovery selhalo: 0x%X", err);
        return ISOTP_ERR_CAN_TX;
      }

      ISOTP_LOGI("BUS_OFF recovery uspesne, zkousim znovu odeslat");

      /* Jeden pokus o opakovane odeslani po uspesnem recovery */
      /* AUDIT OK: N_As = 25 ms použit správně i pro retry po BUS_OFF recovery */
      err = twai_transmit(&msg, pdMS_TO_TICKS(ISOTP_N_AS_TIMEOUT_MS));
      if (err != ESP_OK) {
        ISOTP_LOGE("twai_transmit po recovery selhalo: 0x%X", err);
        return ISOTP_ERR_CAN_TX;
      }
      return ISOTP_OK;

    } else {
      /* INVALID_STATE, ale neni BUS_OFF — jiny problem (napr. driver neni
       * spusten) */
      ISOTP_LOGE("twai_transmit: ESP_ERR_INVALID_STATE, stav=%d (neni BUS_OFF)",
                 (int)status.state);
      return ISOTP_ERR_CAN_TX;
    }
  }

  if (err != ESP_OK) {
    ISOTP_LOGE("twai_transmit failed: 0x%X (id=0x%03X)", err, (unsigned)id);
    return ISOTP_ERR_CAN_TX;
  }
  return ISOTP_OK;
}

/**
 * @brief Prijme CAN ramec s volitelnou filtrach podle CAN ID.
 *
 * Funkce blokuje az do prijmu ramce se shodnym ID nebo do vyprseni timeoutu.
 * Ramce s neshodnym ID jsou v tichosti zahazovany (ale jejich prijem se
 * zapocitava do casu — cekani NENI restartovano).
 *
 * Rezimy filtrovani:
 *   - filter_id == 0       : akceptuje libovolnou OBD-II odpoved
 *                            (rozsah 0x7E8..0x7EF, broadcast rezim)
 *   - filter_id != 0       : akceptuje pouze presnou shodu s ID
 *
 * @param filter_id   Ocekavane CAN ID, nebo 0 pro vsechny OBD odpovedi
 * @param out_msg     Vystup: prijaty ramec
 * @param timeout_ms  Maximalni doba cekani v milisekundach
 * @return ISOTP_OK pri prijmu, ISOTP_ERR_TIMEOUT pri vyprseni casu,
 *         ISOTP_ERR_CAN_RX pri chybe driveru
 */
static isotp_status_t isotp_can_recv(uint32_t filter_id,
                                     twai_message_t *out_msg,
                                     uint32_t timeout_ms) {
  uint32_t start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  while (1) {
    uint32_t elapsed =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - start;
    if (elapsed >= timeout_ms) {
      ISOTP_LOGD("RX timeout after %dms (filter=0x%03X)", (int)timeout_ms,
                 (unsigned)filter_id);
      return ISOTP_ERR_TIMEOUT;
    }

    uint32_t remaining = timeout_ms - elapsed;
    esp_err_t err = twai_receive(out_msg, pdMS_TO_TICKS(remaining));

    if (err == ESP_ERR_TIMEOUT) {
      return ISOTP_ERR_TIMEOUT;
    }
    if (err != ESP_OK) {
      ISOTP_LOGE("twai_receive error: 0x%X", err);
      return ISOTP_ERR_CAN_RX;
    }

    /* Ramec prijat — overime zda vyhovuje filtru */
    isotp_log_frame("RX <<<", out_msg->identifier, out_msg->data,
                    out_msg->data_length_code);

    if (filter_id == 0) {
      /* Akceptujeme libovolne OBD-II response ID (0x7E8..0x7EF) */
      if (out_msg->identifier >= ISOTP_OBD_PHYS_RESP_BASE &&
          out_msg->identifier <
              ISOTP_OBD_PHYS_RESP_BASE + ISOTP_MAX_ECU_RESPONSES) {
        return ISOTP_OK;
      }
      ISOTP_LOGD("RX ignored non-OBD frame id=0x%03X",
                 (unsigned)out_msg->identifier);
    } else {
      /* Akceptujeme pouze presnou shodu ID */
      if (out_msg->identifier == filter_id) {
        return ISOTP_OK;
      }
      ISOTP_LOGD("RX ignored id=0x%03X (want 0x%03X)",
                 (unsigned)out_msg->identifier, (unsigned)filter_id);
    }
    /* Nespravne ID — pokracujeme ve smycce dokud nevyprsi timeout */
  }
}

/* ========================================================================= */
/*  Sestaveni a odeslani ISO-TP ramcu                                        */
/* ========================================================================= */

/**
 * @brief Sestavi a odesle Single Frame (SF).
 *
 * Single Frame je zakladni typ ramce ISO-TP, ktery prenasi cely payload
 * (1..7 bajtu) v jedinem CAN ramci. Pouziva se pro kratke OBD-II dotazy
 * a odpovedi (napr. zadost o Mode 01 PID 0x0C — RPM).
 *
 * Format SF (ISO 15765-2, kap. 9.6.2.1):
 *   Bajt 0:      [0x0N]  — horni nibble 0 = SF PCI, dolni nibble N = delka
 * (1..7) Bajt 1..N:   data payloadu Bajt N+1..7: padding (obvykle 0xAA nebo
 * 0xCC podle konfigurace)
 *
 * Priklad (zadost o RPM, CAN ID 0x7DF):
 *   02 01 0C AA AA AA AA AA
 *   -- -- --  <-- padding -->
 *   |  |  +-- PID 0x0C (RPM)
 *   |  +----- Mode 01 (current data)
 *   +-------- SF PCI | delka 2
 *
 * @param tx_id CAN ID pro odeslani (napr. 0x7DF pro broadcast)
 * @param data  Ukazatel na payload (1..7 bajtu)
 * @param len   Delka payloadu v bajtech (1..7)
 * @return ISOTP_OK pri uspechu, jinak kod chyby
 *
 * @note Edge case: len == 0 nebo len > 7 vrati ISOTP_ERR_INVALID_ARG.
 */
static isotp_status_t isotp_send_sf(uint32_t tx_id, const uint8_t *data,
                                    uint8_t len) {
  if (data == NULL || len == 0 || len > ISOTP_CAN_MAX_DATA) {
    ISOTP_LOGE("SF send invalid args: data=%p len=%d", data, len);
    return ISOTP_ERR_INVALID_ARG;
  }

  uint8_t frame[ISOTP_CAN_DLC];
  memset(frame, ISOTP_PADDING_BYTE, ISOTP_CAN_DLC);

  /* PCI bajt: horni nibble = 0 (SF), dolni nibble = delka payloadu */
  frame[0] = (uint8_t)(ISOTP_PCI_SF | (len & 0x0F));
  memcpy(&frame[1], data, len);

  ISOTP_LOGD("SF send: id=0x%03X len=%d", (unsigned)tx_id, len);
  return isotp_can_send(tx_id, frame);
}

/*
 * [NEAKTIVNÍ] --> Pro Tx Multiframe zprávy (nebude v této verzi použito)
 *
 * static isotp_status_t isotp_send_ff(uint32_t tx_id,
 *                                     const uint8_t *data,
 *                                     uint16_t total_len) {
 *
 *   // Validace: FF_DL musí být 8..ISOTP_MAX_PAYLOAD
 *   // (< 8 by šlo jako SF, > ISOTP_MAX_PAYLOAD nepodporujeme)
 *   if (data == NULL || total_len < 8 || total_len > ISOTP_MAX_PAYLOAD) {
 *     ISOTP_LOGE("FF send invalid args: data=%p len=%d", data, total_len);
 *     return ISOTP_ERR_INVALID_ARG;
 *   }
 *
 *   uint8_t frame[ISOTP_CAN_DLC];
 *   memset(frame, ISOTP_PADDING_BYTE, ISOTP_CAN_DLC);
 *
 *   // Byte 0: 0x1X — high nibble = FF PCI (0x10),
 *   //                low nibble  = horní 4 bity FF_DL (12-bit hodnota)
 *   // Byte 1: dolních 8 bitů FF_DL
 *   // Norma ISO 15765-2:2016, kap. 9.6.3.2, FF_DL je 12-bitové číslo.
 *   frame[0] = (uint8_t)(ISOTP_PCI_FF | ((total_len >> 8) & 0x0F));
 *   frame[1] = (uint8_t)(total_len & 0xFF);
 *
 *   // Byte 2..7: prvních 6 bajtů payloadu
 *   memcpy(&frame[2], data, ISOTP_CAN_FF_DATA);  // ISOTP_CAN_FF_DATA = 6
 *
 *   ISOTP_LOGD("FF send: id=0x%03X total_len=%d", (unsigned)tx_id, total_len);
 *   return isotp_can_send(tx_id, frame);
 * }
 */

/*
 * [NEAKTIVNÍ] --> Pro Tx Multiframe zprávy (nebude v této verzi použito)
 *
 * static isotp_status_t isotp_send_cf(uint32_t tx_id,
 *                                     const uint8_t *data,
 *                                     uint8_t len,
 *                                     uint8_t sn) {
 *   // len musí být 1..7 (poslední CF může být kratší)
 *   if (data == NULL || len == 0 || len > ISOTP_CAN_CF_DATA) {
 *     return ISOTP_ERR_INVALID_ARG;
 *   }
 *
 *   uint8_t frame[ISOTP_CAN_DLC];
 *   memset(frame, ISOTP_PADDING_BYTE, ISOTP_CAN_DLC);
 *
 *   // Byte 0: 0x2N — high nibble = CF PCI (0x20),
 *   //                low nibble  = Sequence Number (SN mod 16)
 *   // Norma ISO 15765-2:2016, kap. 9.6.4.2
 *   // SN: začíná na 1, inkrementuje, wrappuje se z 0xF → 0x0 → 0x1 ...
 *   frame[0] = (uint8_t)(ISOTP_PCI_CF | (sn & 0x0F));
 *   memcpy(&frame[1], data, len);
 *
 *   ISOTP_LOGD("CF send: id=0x%03X sn=%d len=%d", (unsigned)tx_id, sn, len);
 *   return isotp_can_send(tx_id, frame);
 * }
 */

/*
 * [NEAKTIVNÍ] --> Pro Tx Multiframe zprávy (nebude v této verzi použito)
 *
 * static isotp_status_t isotp_send_multiframe(uint32_t tx_id,
 *                                             uint32_t rx_id,
 *                                             const uint8_t *data,
 *                                             uint16_t total_len) {
 *
 *   // --- 1. Odeslání First Frame ---
 *   isotp_status_t st = isotp_send_ff(tx_id, data, total_len);
 *   if (st != ISOTP_OK) return st;
 *
 *   uint16_t sent = ISOTP_CAN_FF_DATA;  // prvních 6 bajtů odesláno v FF
 *   uint8_t  sn   = 1;                 // SN pro první CF = 1 (dle normy)
 *
 *   // --- 2. Smyčka: čekání na FC + odesílání bloku CF ---
 *   while (sent < total_len) {
 *
 *     // --- 2a. Příjem Flow Control od přijímače ---
 *     // AUDIT OK: Timeout N_Bs = 75 ms dle normy ISO 15765-2:2016, Table 5.
 *     twai_message_t fc_msg;
 *     st = isotp_can_recv(rx_id, &fc_msg, ISOTP_N_BS_TIMEOUT_MS);
 *     if (st == ISOTP_ERR_TIMEOUT) {
 *       ISOTP_LOGE("N_Bs timeout: no FC received from 0x%03X", (unsigned)rx_id);
 *       return ISOTP_ERR_TIMEOUT;
 *     }
 *     if (st != ISOTP_OK) return st;
 *
 *     // Ověření, že přijatý rámec je skutečně FC (PCI = 0x3x)
 *     if (isotp_get_pci_type(fc_msg.data[0]) != ISOTP_PCI_FC) {
 *       ISOTP_LOGE("Expected FC, got PCI=0x%02X", fc_msg.data[0] & 0xF0);
 *       return ISOTP_ERR_UNEXPECTED;
 *     }
 *
 *     // --- 2b. Dekódování FC ---
 *     // Byte 0 low nibble: FlowStatus (0=CTS, 1=WAIT, 2=OVERFLOW)
 *     // Byte 1: BlockSize — počet CF, které smíme odeslat bez dalšího FC
 *     //         0 = žádné omezení (pošli vše)
 *     // Byte 2: STmin — minimální mezera mezi CF v ms (0..127)
 *     //         nebo 0xF1..0xF9 pro 100..900 µs (dle normy ISO 15765-2:2016, kap. 9.6.5.5)
 *     isotp_fc_status_t fc_status =
 *         (isotp_fc_status_t)(fc_msg.data[0] & 0x0F);
 *     uint8_t block_size = fc_msg.data[1];
 *     uint8_t stmin_raw  = fc_msg.data[2];
 *
 *     // Převod STmin na ms pro vTaskDelay():
 *     //   0x00–0x7F : hodnota přímo v ms (0–127 ms)
 *     //   0xF1–0xF9 : 100–900 µs → zaokrouhlíme na 1 ms (vTaskDelay minimum)
 *     //   ostatní   : rezervováno normou, použijeme 0 ms
 *     // AUDIT OK: STmin pauza mezi CF dle FC byte[2], viz ISO 15765-2:2016, kap. 9.6.5.5
 *     uint32_t stmin_ms = 0;
 *     if (stmin_raw <= 0x7F) {
 *       stmin_ms = stmin_raw;
 *     } else if (stmin_raw >= 0xF1 && stmin_raw <= 0xF9) {
 *       stmin_ms = 1;  // 100–900 µs → 1 ms (FreeRTOS tick resolution)
 *     }
 *
 *     ISOTP_LOGD("FC recv: status=%d BS=%d STmin_raw=0x%02X STmin_ms=%d",
 *                fc_status, block_size, stmin_raw, (int)stmin_ms);
 *
 *     // --- 2c. Zpracování FlowStatus ---
 *     if (fc_status == ISOTP_FC_OVERFLOW) {
 *       // Přijímač hlásí přeplnění bufferu — přerušíme přenos.
 *       ISOTP_LOGE("FC Overflow received from 0x%03X", (unsigned)rx_id);
 *       return ISOTP_ERR_OVERFLOW;
 *     }
 *
 *     if (fc_status == ISOTP_FC_WAIT) {
 *       // Přijímač žádá o počkání — vratime se na začátek smyčky a čekáme
 *       // na další FC. Norma povoluje opakovaný WAIT, ale nedefinuje limit;
 *       // TODO: lze přidat čítač max. WAIT rámců pro ochranu před deadlockem.
 *       ISOTP_LOGW("FC Wait received, waiting for next FC...");
 *       continue;
 *     }
 *
 *     // fc_status == ISOTP_FC_CTS → Continue To Send
 *     // --- 2d. Odesílání bloku Consecutive Frames ---
 *     uint8_t cf_in_block = 0;  // počítadlo CF v aktuálním bloku
 *
 *     while (sent < total_len) {
 *       // Při BlockSize > 0 zastavíme po BS rámcích a čekáme na nový FC.
 *       if (block_size != 0 && cf_in_block >= block_size) {
 *         break;  // vrátíme se do vnější smyčky → příjem dalšího FC
 *       }
 *
 *       // Délka aktuálního CF: max 7 bajtů, nebo méně pro poslední rámec.
 *       uint16_t remaining = total_len - sent;
 *       uint8_t  chunk = (remaining > ISOTP_CAN_CF_DATA)
 *                          ? ISOTP_CAN_CF_DATA
 *                          : (uint8_t)remaining;
 *
 *       st = isotp_send_cf(tx_id, &data[sent], chunk, sn);
 *       if (st != ISOTP_OK) return st;
 *
 *       sent += chunk;
 *       sn = (sn + 1) & 0x0F;  // SN wrap: 0xF → 0x0, pak 0x1 atd. (dle normy)
 *       cf_in_block++;
 *
 *       ISOTP_LOGD("CF sent: sn=%d, %d/%d bytes total", sn - 1, sent, total_len);
 *
 *       // STmin: mezera mezi rámci CF dle FC přijímače.
 *       // Norma ISO 15765-2:2016, kap. 9.6.5.5:
 *       //   odesílatel MUSÍ dodržet STmin jako minimální dobu
 *       //   mezi koncem odeslání jednoho CF a začátkem odesílání dalšího CF.
 *       if (stmin_ms > 0 && sent < total_len) {
 *         vTaskDelay(pdMS_TO_TICKS(stmin_ms));
 *       }
 *     }
 *   }
 *
 *   ISOTP_LOGI("Multi-frame TX complete: id=0x%03X, %d bytes", (unsigned)tx_id, total_len);
 *   return ISOTP_OK;
 * }
 */

/**
 * @brief Sestavi a odesle Flow Control (FC) ramec.
 *
 * FC ramec posila prijemce vysilateli pote, co prijme First Frame (FF),
 * aby rekl, jak (a zda vubec) pokracovat s odesilanim Consecutive Frames.
 *
 * Format FC (ISO 15765-2, kap. 9.6.5):
 *   Bajt 0:      [0x3S]  — horni nibble 0x3 = FC PCI, dolni nibble = FlowStatus
 *                          0 = CTS (Clear To Send), 1 = WAIT, 2 = OVFLW
 * (Overflow) Bajt 1:      BlockSize  — pocet CF, ktere smi odesilatel poslat
 * bez dalsiho FC (0 = bez omezeni) Bajt 2:      STmin      — minimalni mezera
 * mezi CF v ms (0..127) nebo us (0xF1..0xF9) Bajt 3..7:   padding
 *
 * @param tx_id  CAN ID pro odeslani FC (obvykle 0x7E0..0x7E7)
 * @param status FlowStatus (ISOTP_FC_CTS / ISOTP_FC_WAIT / ISOTP_FC_OVERFLOW)
 * @return ISOTP_OK pri uspechu, jinak kod chyby
 */
static isotp_status_t isotp_send_fc(uint32_t tx_id, isotp_fc_status_t status) {
  uint8_t frame[ISOTP_CAN_DLC];
  memset(frame, ISOTP_PADDING_BYTE, ISOTP_CAN_DLC);

  /* PCI = 0x30 | FS, FS zabira dolni nibble */
  frame[0] = (uint8_t)(ISOTP_PCI_FC | (status & 0x0F));
  frame[1] = ISOTP_FC_BS;    /* BlockSize z konfigurace */
  frame[2] = ISOTP_FC_STMIN; /* Minimalni separation time z konfigurace */

  ISOTP_LOGD("FC send: id=0x%03X status=%d BS=%d STmin=%d", (unsigned)tx_id,
             status, ISOTP_FC_BS, ISOTP_FC_STMIN);
  /* AUDIT OK: N_Ar (příjemce posílá FC) = 25 ms — isotp_can_send() používá
   * ISOTP_N_AS_TIMEOUT_MS, jehož hodnota je shodná s N_Ar dle normy */
  return isotp_can_send(tx_id, frame);
}

/* ========================================================================= */
/*  Parsovani ISO-TP ramcu                                                   */
/* ========================================================================= */

/**
 * @brief Extrahuje typ PCI z prvniho bajtu CAN ramce.
 *
 * Typ ramce ISO-TP je zakodovan v hornim nibble prvniho datoveho bajtu:
 *   0x00 = SF (Single Frame)
 *   0x10 = FF (First Frame)
 *   0x20 = CF (Consecutive Frame)
 *   0x30 = FC (Flow Control)
 *
 * @param byte0 Prvni bajt datove casti CAN ramce
 * @return Typ PCI (horni nibble byte0)
 */
static isotp_pci_type_t isotp_get_pci_type(uint8_t byte0) {
  return (isotp_pci_type_t)(byte0 & 0xF0);
}

/**
 * @brief Zpracuje Single Frame odpoved a zkopiruje payload do vystupniho
 * bufferu.
 *
 * @param frame_data Ukazatel na 8 bajtu prijateho CAN ramce
 * @param out_buf    Vystupni buffer pro payload
 * @param buf_size   Velikost vystupniho bufferu
 * @return Delka payloadu (1..7), nebo 0 pri chybe (nevalidni SF_DL, mala
 * velikost bufferu)
 *
 * @note Edge case: SF_DL == 0 nebo SF_DL > 7 je povazovano za chybu protokolu.
 */
static uint8_t isotp_parse_sf(const uint8_t *frame_data, uint8_t *out_buf,
                              uint16_t buf_size) {
  /* SF_DL je v dolnim nibble prvniho bajtu */
  uint8_t sf_dl = frame_data[0] & 0x0F;

  if (sf_dl == 0 || sf_dl > ISOTP_CAN_MAX_DATA) {
    ISOTP_LOGE("SF invalid length: SF_DL=%d", sf_dl);
    return 0;
  }
  if (sf_dl > buf_size) {
    ISOTP_LOGE("SF payload %d exceeds buffer %d", sf_dl, buf_size);
    return 0;
  }

  memcpy(out_buf, &frame_data[1], sf_dl);
  ISOTP_LOGD("SF parsed: len=%d", sf_dl);
  return sf_dl;
}

/**
 * @brief Zpracuje prijem viceramcove zpravy (FF + CFs).
 *
 * Funkce je volana, pokud prvni prijaty ramec je First Frame (PCI 0x10).
 * Postup zpracovani:
 *   1. Z FF extrahuje FF_DL (12-bit celkova delka zpravy) a prvnich 6 bajtu
 * dat.
 *   2. Odesle zpet Flow Control s ContinueToSend (CTS).
 *   3. V cyklu prijima Consecutive Frames (CF) a skládá je do vystupniho
 * bufferu.
 *   4. Kontroluje sekvencni cislo (SN) u kazdeho CF; SN zacina na 1 a
 *      obtekne z 0xF na 0x0.
 *
 * Format FF (ISO 15765-2, kap. 9.6.3):
 *   Bajt 0:     [0x1X]  — 0x10 = FF PCI | hornich 4 bity FF_DL
 *   Bajt 1:     [0xYY]  — dolnich 8 bitu FF_DL  (celkem 12-bit, max 4095)
 *   Bajt 2..7:  prvnich 6 bajtu payloadu
 *
 * Format CF (ISO 15765-2, kap. 9.6.4):
 *   Bajt 0:     [0x2N]  — 0x20 = CF PCI | sekvencni cislo (0..F, wrap)
 *   Bajt 1..7:  az 7 bajtu payloadu
 *
 * Priklad FF pro 10 bajtu: 10 0A 49 02 01 31 47 31
 *   10 = FF, 0A = FF_DL=10, nasleduje 6 bajtu payloadu
 *
 * Edge cases:
 *   - FF_DL == 0: neplatny ramec, vraci ISOTP_ERR_UNEXPECTED
 *   - FF_DL > ISOTP_MAX_PAYLOAD: prekroceni kapacity, posila se FC OVFLW
 *   - Timeout N_Cr pri cekani na CF: vraci ISOTP_ERR_TIMEOUT
 *   - Spatne sekvencni cislo: vraci ISOTP_ERR_SEQUENCE
 *
 * @param rx_id   CAN ID, odkud prisel FF (napr. 0x7E8)
 * @param ff_data 8 bajtu prijateho First Frame
 * @param out_buf Vystupni buffer pro slozeny payload
 * @param out_len Vystup: celkova delka payloadu
 * @return ISOTP_OK pri uspesnem slozeni, jinak prislusna chyba
 */
static isotp_status_t isotp_handle_multiframe(uint32_t rx_id,
                                              const uint8_t *ff_data,
                                              uint8_t *out_buf,
                                              uint16_t *out_len) {
  /* ---- Parsovani First Frame ---- */
  /* FF_DL = 12 bitu: dolni nibble byte[0] (hornich 4 bity) + byte[1] (dolnich 8
   * bitu) */
  uint16_t ff_dl = (uint16_t)(((ff_data[0] & 0x0F) << 8) | ff_data[1]);

  ISOTP_LOGD("FF parsed: id=0x%03X FF_DL=%d", (unsigned)rx_id, ff_dl);

  if (ff_dl == 0) {
    ISOTP_LOGE("FF_DL is zero (id=0x%03X)", (unsigned)rx_id);
    return ISOTP_ERR_UNEXPECTED;
  }

  if (ff_dl > ISOTP_MAX_PAYLOAD) {
    ISOTP_LOGE("FF_DL=%d exceeds ISOTP_MAX_PAYLOAD=%d (id=0x%03X)", ff_dl,
               ISOTP_MAX_PAYLOAD, (unsigned)rx_id);
    /* Posleme FC Overflow, abychom rekli ECU at prestane vysilat */
    uint32_t tx_id =
        rx_id - (ISOTP_OBD_PHYS_RESP_BASE - ISOTP_OBD_PHYS_REQ_BASE);
    isotp_send_fc(tx_id, ISOTP_FC_OVERFLOW);
    return ISOTP_ERR_OVERFLOW;
  }

  /* Zkopirujeme prvnich 6 bajtu payloadu primo z FF */
  uint16_t received = 0;
  uint16_t first_chunk =
      (ff_dl < ISOTP_CAN_FF_DATA) ? ff_dl : ISOTP_CAN_FF_DATA;
  memcpy(out_buf, &ff_data[2], first_chunk);
  received = first_chunk;

  ISOTP_LOGD("FF data: %d/%d bytes received", received, ff_dl);

  /* ---- Odeslani Flow Control (ContinueToSend) ---- */
  /* TX ID ziskame odectenim offsetu (0x7E8 - 0x7E0 = 8) od response ID */
  uint32_t tx_id = rx_id - (ISOTP_OBD_PHYS_RESP_BASE - ISOTP_OBD_PHYS_REQ_BASE);
  isotp_status_t st = isotp_send_fc(tx_id, ISOTP_FC_CTS);
  if (st != ISOTP_OK) {
    return st;
  }

  /* ---- Prijem Consecutive Frames ---- */
  uint8_t expected_seq = 1; /* SN zacina u CF na 1, po 0xF obtekne na 0 */

  while (received < ff_dl) {
    twai_message_t cf_msg;
    /* AUDIT OK: N_Cr = 150 ms použit správně pro timeout čekání na CF,
     * viz ISO 15765-2:2016 Table 5 */
    st = isotp_can_recv(rx_id, &cf_msg, ISOTP_N_CR_TIMEOUT_MS);
    if (st != ISOTP_OK) {
      ISOTP_LOGE(
          "CF timeout: expected seq=%d, received %d/%d bytes (id=0x%03X)",
          expected_seq, received, ff_dl, (unsigned)rx_id);
      return ISOTP_ERR_TIMEOUT;
    }

    /* Overime, ze PCI odpovida typu CF (0x20) */
    isotp_pci_type_t pci = isotp_get_pci_type(cf_msg.data[0]);
    if (pci != ISOTP_PCI_CF) {
      ISOTP_LOGE("Expected CF, got PCI=0x%02X (id=0x%03X)",
                 cf_msg.data[0] & 0xF0, (unsigned)rx_id);
      return ISOTP_ERR_UNEXPECTED;
    }

    /* Overime sekvencni cislo (dolni nibble prvniho bajtu) */
    uint8_t actual_seq = cf_msg.data[0] & 0x0F;
    uint8_t expected_sn = expected_seq & 0x0F; /* Obtekani v rozsahu 0..F */

    if (actual_seq != expected_sn) {
      ISOTP_LOGE("CF seq mismatch: expected=0x%X got=0x%X (id=0x%03X)",
                 expected_sn, actual_seq, (unsigned)rx_id);
      return ISOTP_ERR_SEQUENCE;
    }

    /* Zkopirujeme data z CF (az 7 bajtu na ramec) */
    uint16_t remaining = ff_dl - received;
    uint16_t chunk =
        (remaining < ISOTP_CAN_CF_DATA) ? remaining : ISOTP_CAN_CF_DATA;
    memcpy(&out_buf[received], &cf_msg.data[1], chunk);
    received += chunk;

    ISOTP_LOGD("CF seq=%d: %d/%d bytes (id=0x%03X)", actual_seq, received,
               ff_dl, (unsigned)rx_id);

    expected_seq++;
  }

  *out_len = ff_dl;
  ISOTP_LOGI("Multi-frame complete: id=0x%03X, %d bytes", (unsigned)rx_id,
             ff_dl);
  return ISOTP_OK;
}

/* ========================================================================= */
/*  Dispatcher pro prijem zpravy                                             */
/* ========================================================================= */

/**
 * @brief Prijme jednu kompletni ISO-TP zpravu z konkretniho CAN ID.
 *
 * Funkce cekal na prvni ramec s danym rx_id. Podle typu PCI rozhodne,
 * zda jde o Single Frame (hotovo jednim ramcem), nebo First Frame,
 * ktery je nasledovan vyctem Consecutive Frames (vicesramcova zprava).
 *
 * @param rx_id      CAN ID, ze ktereho ocekavame odpoved
 * @param buffer     Vystupni buffer pro payload
 * @param out_len    Vystup: skutecna delka prijate zpravy
 * @param timeout_ms Celkovy timeout pro prijem prvniho ramce
 * @return ISOTP_OK pri uspechu, jinak kod chyby
 *
 * @note Pokud prvni ramec neni ani SF ani FF, vraci ISOTP_ERR_UNEXPECTED.
 */
static isotp_status_t isotp_receive_message(uint32_t rx_id, uint8_t *buffer,
                                            uint16_t *out_len,
                                            uint32_t timeout_ms) {
  twai_message_t msg;
  isotp_status_t st = isotp_can_recv(rx_id, &msg, timeout_ms);
  if (st != ISOTP_OK) {
    return st;
  }

  isotp_pci_type_t pci = isotp_get_pci_type(msg.data[0]);

  switch (pci) {
  case ISOTP_PCI_SF: {
    /* Single Frame — zprava se vejde do jednoho ramce */
    uint8_t len = isotp_parse_sf(msg.data, buffer, ISOTP_MAX_PAYLOAD);
    if (len == 0) {
      return ISOTP_ERR_UNEXPECTED;
    }
    *out_len = len;
    ISOTP_LOGI("SF response: id=0x%03X, %d bytes", (unsigned)rx_id, len);
    return ISOTP_OK;
  }

  case ISOTP_PCI_FF:
    /* First Frame — predame zpracovani funkci pro vicesramcovou zpravu */
    return isotp_handle_multiframe(rx_id, msg.data, buffer, out_len);

  default:
    /* CF nebo FC jako prvni ramec jsou nevalidni — ocekavame SF nebo FF */
    ISOTP_LOGE("Unexpected first PCI=0x%02X from id=0x%03X", pci,
               (unsigned)rx_id);
    return ISOTP_ERR_UNEXPECTED;
  }
}

/* ========================================================================= */
/*  Verejne API: Inicializace a deinicializace                               */
/* ========================================================================= */

/**
 * @brief Inicializuje TWAI (CAN) driver pro ISO-TP komunikaci.
 *
 * Nainstaluje a spusti TWAI driver ESP-IDF s pozadovanou baudrate
 * a GPIO piny pro TX/RX. Podporovane baudrate: 500 kbit/s a 250 kbit/s
 * (standardni OBD-II rychlosti).
 *
 * Filter je nakonfigurovan tak, aby prijimal vsechny ramce; filtrovani
 * podle CAN ID se provadi softwarove v isotp_can_recv().
 *
 * @param baudrate Rychlost sbernice v bit/s (500000 nebo 250000)
 * @param tx_pin   GPIO cislo pro TX pin CAN transceiveru
 * @param rx_pin   GPIO cislo pro RX pin CAN transceiveru
 * @return ISOTP_OK pri uspesne inicializaci, jinak kod chyby
 *
 * @note Pokud je driver jiz inicializovan, provede se nejprve deinit.
 * @warning Nepodporovana baudrate vrati ISOTP_ERR_INVALID_ARG.
 */
isotp_status_t isotp_init(uint32_t baudrate, int tx_pin, int rx_pin) {
  ISOTP_LOGI("Init: baudrate=%d tx=GPIO%d rx=GPIO%d", (int)baudrate, tx_pin,
             rx_pin);

  /*
   * Defenzivni cleanup — VZDY pokusime odinstalovat existujici driver,
   * i kdyz _isotp_initialized je false. Duvod: pokud predchozi deinit
   * selhal (napr. BUS_OFF), _isotp_initialized je false ale driver
   * stale bezi → twai_driver_install by selhalo s ESP_ERR_INVALID_STATE.
   *
   * _isotp_force_cleanup_twai() je bezpecne volani i kdyz driver neni
   * nainstalovany (twai_get_status_info vrati chybu → skip).
   */
  _isotp_force_cleanup_twai();
  _isotp_initialized = false;

  /* Obecna konfigurace — velikost RX fronty dle ISOTP_TWAI_RX_QUEUE_LEN.
   *
   * TWAI alerty: povolime klicove alerty pro diagnostiku stavu sbernice.
   * Bez alertu neni mozne detekovat BUS_OFF, error-passive ci TX failure
   * jinak nez pres navratove kody (ktere casto nemaji dostatecny detail).
   * Alerty se ctou volanim twai_read_alerts() a neprodukuji zadny
   * overhead pokud se nectou. */
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = ISOTP_TWAI_RX_QUEUE_LEN;
  g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS
                          | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED
                          | TWAI_ALERT_RX_QUEUE_FULL;

  /* Casovani sbernice — vybirame podle pozadovane baudrate */
  twai_timing_config_t t_config;
  switch (baudrate) {
  case 500000:
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
    break;
  case 250000:
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
    break;
  default:
    ISOTP_LOGE("Unsupported baudrate: %d (use 500000 or 250000)",
               (int)baudrate);
    return ISOTP_ERR_INVALID_ARG;
  }

  /* Akceptujeme vsechny ramce — softwarove filtrovani dela isotp_can_recv() */
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  /* Instalace TWAI driveru */
  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    ISOTP_LOGE("twai_driver_install failed: 0x%X", err);
    return ISOTP_ERR_CAN_TX;
  }

  /* Spusteni driveru (prechod do TWAI_STATE_RUNNING) */
  err = twai_start();
  if (err != ESP_OK) {
    ISOTP_LOGE("twai_start failed: 0x%X", err);
    twai_driver_uninstall();
    return ISOTP_ERR_CAN_TX;
  }

  /* Verifikace stavu — driver musi byt v RUNNING, error countery na 0 */
  {
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
      ISOTP_LOGI("Init post-check: state=%d TEC=%d REC=%d",
                 (int)status.state,
                 (int)status.tx_error_counter,
                 (int)status.rx_error_counter);
      if (status.state != TWAI_STATE_RUNNING) {
        ISOTP_LOGE("Init: TWAI not in RUNNING state (%d) after start!",
                   (int)status.state);
        twai_stop();
        twai_driver_uninstall();
        return ISOTP_ERR_CAN_TX;
      }
    }
  }

  _isotp_initialized = true;
  ISOTP_LOGI("Init OK: TWAI running at %d bps, RX queue=%d", (int)baudrate,
             ISOTP_TWAI_RX_QUEUE_LEN);
  return ISOTP_OK;
}

/**
 * @brief Interni helper — spolehlivy cleanup TWAI driveru bez ohledu na stav.
 *
 * Resi vsechny mozne stavy TWAI periferie vcetne BUS_OFF a RECOVERING.
 * Volano z isotp_deinit() i z isotp_init() pro defenzivni reinicializaci.
 *
 * Postup:
 *   1. twai_get_status_info — zjisti zda je driver vubec nainstalovany
 *   2. BUS_OFF → recovery → STOPPED
 *   3. RUNNING → stop → STOPPED
 *   4. STOPPED → uninstall
 *
 * @return true pokud driver byl uspesne odinstalovany (nebo nebyl nainstalovany),
 *         false pokud se cleanup nepodaril (toto by nemelo nastat)
 */
static bool _isotp_force_cleanup_twai(void) {
  twai_status_info_t status;
  esp_err_t err = twai_get_status_info(&status);

  if (err != ESP_OK) {
    /* Driver neni nainstalovany — neni co cistit */
    ISOTP_LOGD("force_cleanup: driver not installed (0x%X)", err);
    return true;
  }

  ISOTP_LOGI("force_cleanup: state=%d TEC=%d REC=%d tx_pending=%d rx_pending=%d",
             (int)status.state,
             (int)status.tx_error_counter, (int)status.rx_error_counter,
             (int)status.msgs_to_tx, (int)status.msgs_to_rx);

  /* BUS_OFF — nutna recovery sekvence pred stop */
  if (status.state == TWAI_STATE_BUS_OFF) {
    ISOTP_LOGW("force_cleanup: BUS_OFF, initiating recovery...");
    err = twai_initiate_recovery();
    if (err != ESP_OK) {
      ISOTP_LOGE("force_cleanup: twai_initiate_recovery failed: 0x%X", err);
      /* Zkusime uninstall i tak — driver mohl zustat v nekonzistentnim stavu */
      goto try_uninstall;
    }
    /* Cekani na recovery (max 500ms) */
    for (int i = 0; i < 50; i++) {
      vTaskDelay(pdMS_TO_TICKS(10));
      twai_get_status_info(&status);
      if (status.state == TWAI_STATE_STOPPED) break;
    }
    if (status.state != TWAI_STATE_STOPPED) {
      ISOTP_LOGE("force_cleanup: recovery timeout, state=%d", (int)status.state);
      goto try_uninstall;
    }
    ISOTP_LOGI("force_cleanup: BUS_OFF recovery OK");
  }

  /* RECOVERING — cekame na dokonceni */
  if (status.state == TWAI_STATE_RECOVERING) {
    ISOTP_LOGW("force_cleanup: RECOVERING, waiting...");
    for (int i = 0; i < 50; i++) {
      vTaskDelay(pdMS_TO_TICKS(10));
      twai_get_status_info(&status);
      if (status.state == TWAI_STATE_STOPPED) break;
    }
  }

  /* RUNNING — zastaveni */
  if (status.state == TWAI_STATE_RUNNING) {
    err = twai_stop();
    if (err != ESP_OK) {
      ISOTP_LOGE("force_cleanup: twai_stop failed: 0x%X (state=%d)",
                 err, (int)status.state);
      /* Pokracujeme — mozna se podari uninstall */
    } else {
      ISOTP_LOGD("force_cleanup: twai_stop OK");
    }
  }

try_uninstall:
  err = twai_driver_uninstall();
  if (err != ESP_OK) {
    ISOTP_LOGE("force_cleanup: twai_driver_uninstall failed: 0x%X", err);
    return false;
  }

  ISOTP_LOGI("force_cleanup: TWAI driver uninstalled OK");
  return true;
}

/**
 * @brief Zastavi a odinstaluje TWAI driver, uvolni zdroje.
 *
 * Po volani teto funkce neni mozne komunikovat pres CAN sbernici,
 * dokud neni opet zavolano isotp_init(). Funkce je idempotentni —
 * opakovane volani nema zadny efekt, pokud driver neni inicializovan.
 *
 * Resi vsechny stavy TWAI driveru vcetne BUS_OFF — puvodni verze
 * ignorovala navratove hodnoty twai_stop/twai_driver_uninstall, coz
 * vedlo k "zombie" driveru pri BUS_OFF stavu.
 */
void isotp_deinit(void) {
  if (_isotp_initialized) {
    _isotp_force_cleanup_twai();
    _isotp_initialized = false;
    ISOTP_LOGI("Deinitialized");
  }
}

/* ========================================================================= */
/*  Verejne API: Transakce (request-response)                                */
/* ========================================================================= */

/**
 * @brief Provede jednu ISO-TP transakci: odesle pozadavek, prijme odpoved.
 *
 * Tato funkce je urcena pro fyzicke adresovani konkretni ECU. Odesila
 * pozadavek jako Single Frame (predpoklada se delka <= 7 bajtu) a ceka
 * na kompletni odpoved, ktera muze byt bud SF nebo vicesramcova
 * zprava (FF + CFs).
 *
 * Priklad pouziti (dotaz na RPM od ECU 0x7E0):
 *   uint8_t req[] = { 0x01, 0x0C };
 *   uint8_t resp[16];
 *   uint16_t resp_len;
 *   isotp_transaction(0x7E0, 0x7E8, req, 2, resp, &resp_len, 1000);
 *
 * @param tx_id      CAN ID pro odeslani pozadavku (napr. 0x7E0..0x7E7)
 * @param rx_id      Ocekavane CAN ID odpovedi (napr. 0x7E8..0x7EF)
 * @param request    Ukazatel na data pozadavku
 * @param req_len    Delka pozadavku v bajtech (1..7)
 * @param response   Vystupni buffer pro odpoved
 * @param resp_len   Vystup: skutecna delka odpovedi
 * @param timeout_ms Maximalni cas na prijem odpovedi v ms
 * @return ISOTP_OK pri uspechu, jinak kod chyby
 */
isotp_status_t isotp_transaction(uint32_t tx_id, uint32_t rx_id,
                                 const uint8_t *request, uint8_t req_len,
                                 uint8_t *response, uint16_t *resp_len,
                                 uint32_t timeout_ms) {
  if (!_isotp_initialized) {
    ISOTP_LOGE("Not initialized");
    return ISOTP_ERR_NOT_INITIALIZED;
  }

  if (request == NULL || response == NULL || resp_len == NULL) {
    ISOTP_LOGE("Transaction: NULL argument");
    return ISOTP_ERR_INVALID_ARG;
  }

  ISOTP_LOGI("Transaction: tx=0x%03X rx=0x%03X req_len=%d timeout=%dms",
             (unsigned)tx_id, (unsigned)rx_id, req_len, (int)timeout_ms);
  isotp_log_data("Request", request, req_len);

  /* Vyprazdneni stare komunikace z RX fronty, aby nedoslo k zamene */
  int flushed = isotp_flush_rx_queue();
  if (flushed > 0) {
    ISOTP_LOGW("Flushed %d stale frame(s) from RX queue", flushed);
  }

  /* Odeslani pozadavku jako Single Frame */
  isotp_status_t st = isotp_send_sf(tx_id, request, req_len);
  if (st != ISOTP_OK) {
    ISOTP_LOGE("Transaction failed at SF send: %s", isotp_status_str(st));
    return st;
  }

  /* Prijem kompletni odpovedi (muze byt SF i vicesramcova) */
  st = isotp_receive_message(rx_id, response, resp_len, timeout_ms);
  if (st != ISOTP_OK) {
    ISOTP_LOGE("Transaction failed at receive: %s (rx=0x%03X)",
               isotp_status_str(st), (unsigned)rx_id);
    return st;
  }

  isotp_log_data("Response", response, *resp_len);
  ISOTP_LOGI("Transaction OK: rx=0x%03X, %d bytes", (unsigned)rx_id, *resp_len);
  return ISOTP_OK;
}

/*
 * [NEAKTIVNÍ] --> Pro Tx Multiframe zprávy (nebude v této verzi použito)
 *
 * isotp_status_t isotp_send(uint32_t tx_id, uint32_t rx_id,
 *                           const uint8_t *data, uint16_t len) {
 *   if (!_isotp_initialized) {
 *     ISOTP_LOGE("Not initialized");
 *     return ISOTP_ERR_NOT_INITIALIZED;
 *   }
 *   if (data == NULL || len == 0 || len > ISOTP_MAX_PAYLOAD) {
 *     return ISOTP_ERR_INVALID_ARG;
 *   }
 *
 *   isotp_log_data("Send", data, len);
 *
 *   if (len <= ISOTP_CAN_MAX_DATA) {
 *     // Zpráva se vejde do jednoho rámce — použij SF
 *     return isotp_send_sf(tx_id, data, (uint8_t)len);
 *   } else {
 *     // Zpráva vyžaduje multi-frame sekvenci FF + FC + CF...
 *     return isotp_send_multiframe(tx_id, rx_id, data, len);
 *   }
 * }
 */

/**
 * @brief Odesle broadcast pozadavek a sesbira odpovedi od vsech ECU.
 *
 * Pouziva funkcni broadcast adresu 0x7DF (ISOTP_OBD_FUNC_REQ_ID), na
 * kterou odpovi vsechny ECU, ktere podporuji dany pozadavek. Funkce
 * sesbira vsechny odpovedi dokud nevyprsi timeout nebo nedosahne
 * limitu ISOTP_MAX_ECU_RESPONSES.
 *
 * Odpovedi mohou byt SF i FF+CFs. Kazda odpoved je ulozena do pole
 * result->responses[] spolu s CAN ID ECU, ktera ji poslala.
 *
 * Priklad pouziti: detekce pritomnych ECU dotazem 01 00 (Mode 01 PID 0x00,
 * supported PIDs).
 *
 * Edge cases:
 *   - Zadna odpoved do timeoutu: result->status = ISOTP_ERR_TIMEOUT, count = 0
 *   - Chyba pri vicesramcove odpovedi od jedne ECU nezastavi sber od dalsich
 *   - Prekroceni ISOTP_MAX_ECU_RESPONSES ukonci sber predcasne
 *
 * @param request    Ukazatel na data pozadavku (1..7 bajtu)
 * @param req_len    Delka pozadavku
 * @param result     Vystupni struktura s poli odpovedi a celkovym stavem
 * @param timeout_ms Maximalni cas na sber odpovedi v ms
 * @return ISOTP_OK pokud alespon jedna ECU odpovedela, jinak kod chyby
 */
isotp_status_t isotp_transaction_broadcast(const uint8_t *request,
                                           uint8_t req_len,
                                           isotp_result_t *result,
                                           uint32_t timeout_ms) {
  if (!_isotp_initialized) {
    ISOTP_LOGE("Not initialized");
    return ISOTP_ERR_NOT_INITIALIZED;
  }

  if (request == NULL || result == NULL) {
    ISOTP_LOGE("Broadcast: NULL argument");
    return ISOTP_ERR_INVALID_ARG;
  }

  /* Vynulovani vystupni struktury pred zacatkem sberu */
  memset(result, 0, sizeof(isotp_result_t));

  ISOTP_LOGI("Broadcast: req_len=%d timeout=%dms", req_len, (int)timeout_ms);
  isotp_log_data("Request", request, req_len);

  /* Odstranime stare ramce, ktere by zkreslily sber odpovedi */
  int flushed = isotp_flush_rx_queue();
  if (flushed > 0) {
    ISOTP_LOGW("Flushed %d stale frame(s) from RX queue", flushed);
  }

  /* Odeslani pozadavku na funkcni broadcast adresu 0x7DF */
  isotp_status_t st = isotp_send_sf(ISOTP_OBD_FUNC_REQ_ID, request, req_len);
  if (st != ISOTP_OK) {
    result->status = st;
    ISOTP_LOGE("Broadcast failed at SF send: %s", isotp_status_str(st));
    return st;
  }

  /* Sber odpovedi az do vyprseni timeoutu nebo naplneni pole */
  uint32_t start = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

  /*
   * VOLITELNA OPTIMALIZACE — inter-response timeout (200 ms):
   * -------------------------------------------------------------------
   * Myslenka: jakmile prijde prvni odpoved, neni nutne cekat plnych
   * timeout_ms na zbytek — vsechny OBD ECU na jedne CAN siti obvykle
   * odpovi do 50--100 ms po prvni. Pokud dalsich 200 ms nic neprijde,
   * predpokladame, ze uz zadna dalsi ECU neodpovi a sber ukoncime.
   *
   * Prinos: rychlejsi dokonceni broadcast dotazu (napr. 200 ms misto
   * 1000 ms) u vozidel s mene ECU. Nevyhoda: pri velmi zatizene sbernici
   * muze dojit k predcasnemu ukonceni, pokud se nejaka ECU opozdi.
   *
   * Jak zapnout: odkomentovat `last_resp_time` + blok nize a pouzit
   * `remaining = min(timeout_ms - elapsed, inter_resp_timeout)`.
   */
  // uint32_t last_resp_time = 0;  /* cas posledni uspesne prijate odpovedi */
  // const uint32_t INTER_RESP_TIMEOUT_MS = 200;

  while (result->count < ISOTP_MAX_ECU_RESPONSES) {
    uint32_t elapsed =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - start;
    if (elapsed >= timeout_ms) {
      break;
    }
    uint32_t remaining = timeout_ms - elapsed;

    /* VOLITELNE — inter-response early exit (viz komentar vyse):
     * if (last_resp_time != 0) {
     *     uint32_t since_last = (uint32_t)(xTaskGetTickCount() *
     * portTICK_PERIOD_MS) - last_resp_time; if (since_last >=
     * INTER_RESP_TIMEOUT_MS) { ISOTP_LOGI("Broadcast: inter-response timeout
     * (%ums) reached, stopping early", INTER_RESP_TIMEOUT_MS); break;
     *     }
     *     uint32_t ir_remaining = INTER_RESP_TIMEOUT_MS - since_last;
     *     if (ir_remaining < remaining) remaining = ir_remaining;
     * }
     */

    /* Cekame na libovolny OBD response ramec (filter_id=0 akceptuje
     * 0x7E8..0x7EF) */
    twai_message_t msg;
    st = isotp_can_recv(0, &msg, remaining);
    if (st == ISOTP_ERR_TIMEOUT) {
      break; /* Zadne dalsi odpovedi v timeoutu — normalni ukonceni sberu */
    }
    if (st != ISOTP_OK) {
      ISOTP_LOGW("Broadcast recv error: %s", isotp_status_str(st));
      continue;
    }

    uint32_t resp_id = msg.identifier;
    uint8_t idx = result->count;
    isotp_response_t *resp = &result->responses[idx];
    resp->rx_id = resp_id;

    isotp_pci_type_t pci = isotp_get_pci_type(msg.data[0]);

    switch (pci) {
    case ISOTP_PCI_SF: {
      /* Odpoved se vesla do jednoho ramce */
      uint8_t len = isotp_parse_sf(msg.data, resp->data, ISOTP_MAX_PAYLOAD);
      if (len > 0) {
        resp->len = len;
        resp->valid = true;
        result->count++;
        /* VOLITELNE: last_resp_time = (uint32_t)(xTaskGetTickCount() *
         * portTICK_PERIOD_MS); */
        ISOTP_LOGI("Broadcast: ECU 0x%03X SF, %d bytes [%d/%d]",
                   (unsigned)resp_id, len, result->count,
                   ISOTP_MAX_ECU_RESPONSES);
      }
      break;
    }

    case ISOTP_PCI_FF: {
      /* Vicesramcova odpoved — zpracuje se kompletne (FC + CFs) */
      isotp_status_t mf_st =
          isotp_handle_multiframe(resp_id, msg.data, resp->data, &resp->len);
      if (mf_st == ISOTP_OK) {
        resp->valid = true;
        result->count++;
        ISOTP_LOGI(
            "Broadcast: ECU 0x%03X multi-frame complete, %d bytes [%d/%d]",
            (unsigned)resp_id, resp->len, result->count,
            ISOTP_MAX_ECU_RESPONSES);
      } else {
        resp->valid = false; /* selhání jedné ECU nepřeruší sběr od ostatních */
        ISOTP_LOGW("Broadcast: ECU 0x%03X multi-frame failed: %s",
                   (unsigned)resp_id, isotp_status_str(mf_st));
      }
      break;
    }

    default:
      /* Neocekavany typ PCI (napr. CF/FC jako prvni ramec) — ignoruje se */
      ISOTP_LOGW("Broadcast: unexpected PCI=0x%02X from 0x%03X, ignoring", pci,
                 (unsigned)resp_id);
      break;
    }
  }

  /* Vyhodnoceni celkoveho stavu po ukonceni sberu */
  uint32_t total_elapsed =
      (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - start;

  if (result->count > 0) {
    result->status = ISOTP_OK;
    ISOTP_LOGI("Broadcast done: %d ECU(s) responded in %dms", result->count,
               (int)total_elapsed);
  } else {
    result->status = ISOTP_ERR_TIMEOUT;
    ISOTP_LOGW("Broadcast: no ECU responded within %dms", (int)timeout_ms);
  }

  return result->status;
}
