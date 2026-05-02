/**
 * @file obd2_diag.c
 * @brief Diagnostické služby OBD-II — DTC, VIN, CalID, ECU name
 *
 * Tento soubor je součástí trojice:
 *   obd2.c       — jádro (init, konfigurace, ISO-TP komunikace, PID tabulka)
 *   obd2_pids.c  — čtení a dekódování PID hodnot
 *   obd2_diag.c  — diagnostické služby (TENTO SOUBOR)
 *
 * Implementuje:
 *   - Dekódování DTC kódu z raw bytů (SAE J2012 / ISO 15031-6)
 *   - Mode 03 / Service $03: čtení potvrzených DTC (Confirmed/Stored)
 *   - Mode 07 / Service $07: čtení čekajících DTC (Pending)
 *   - Mode 04 / Service $04: mazání DTC a diagnostických informací
 *   - Mode 09 / Service $09: informace o vozidle (VIN, ECU name, CalID)
 *
 * @author Ales Pouzar, vycházel jsem z ISO 15031-5/-6, SAE J1979, J2012 a
 * dokumentace a wikipedie pro dostupne PIDs
 */

#include "obd2_internal.h"

/* ========================================================================= */
/*  Dekódování DTC                                                           */
/* ========================================================================= */

/**
 * @brief Dekóduje 2 surové byty na textový DTC řetězec (5 znaků + '\0').
 *
 * Kódování dle SAE J2012 / ISO 15031-6:
 *   Dva surové byty jsou převedeny na 5znakový řetězec ve formátu
 *   "Xdddd", kde X je typový prefix a dddd jsou hexadecimální číslice.
 *
 *   Byte 0, bity 7–6: Typ závady (prefix):
 *     00 = 'P' (Powertrain — hnací ústrojí, motor, převodovka)
 *     01 = 'C' (Chassis — podvozek, ABS, ESP, řízení)
 *     10 = 'B' (Body — karoserie, airbagy, klimatizace, osvětlení)
 *     11 = 'U' (Network/Communication — komunikační sběrnice, gateway)
 *
 *   Byte 0, bity 5–4: Druhá číslice (0–3):
 *     0 = SAE generický kód (platný pro všechny výrobce)
 *     1 = kód specifický pro výrobce (manufacturer-specific)
 *     2 = SAE generický (vyhrazeno — používá se jen u P2xxx)
 *     3 = SAE generický a výrobce sdílený
 *
 *   Byte 0, bity 3–0: Třetí číslice (hex 0–F)
 *
 *   Byte 1, bity 7–4: Čtvrtá číslice (hex 0–F)
 *   Byte 1, bity 3–0: Pátá číslice (hex 0–F)
 *
 * Příklady:
 *   [0x01, 0x00] → "P0100" (okruh senzoru hmotnostního průtoku vzduchu MAF)
 *   [0x43, 0x00] → "C0300" (generický kód podvozku — Chassis)
 *   [0xC1, 0x23] → "U0123" (komunikační chyba — Network)
 *   [0x80, 0x00] → "B0000" (karoserie — obecný)
 *   [0x01, 0x31] → "P0131" (lambda sonda — nízké napětí, banka 1, senzor 1)
 *
 * Hraniční případy:
 *   - Pokud je raw == NULL nebo out == NULL, funkce zapíše prázdný řetězec
 *     (pokud out není NULL) a vrátí se bez chyby.
 *   - Výstupní buffer musí mít alespoň 6 bytů (5 znaků + '\0').
 *   - Nulový DTC [0x00, 0x00] → "P0000" — toto je technicky platný kód,
 *     ale v praxi by neměl být vrácen ECU (obvykle znamená padding).
 *
 * @param raw  Ukazatel na pole 2 surových bytů z odpovědi ECU
 * @param out  Výstupní buffer (minimálně 6 bytů) pro null-terminated řetězec
 */
void obd2_decode_dtc_string(const uint8_t *raw, char *out) {
  if (raw == NULL || out == NULL) {
    if (out)
      out[0] = '\0';
    return;
  }

  /*
   * SAE J2012 / ISO 15031-5 sekce 7.3:
   * Byte 0, bity 7–6: typ  00=P, 01=C, 10=B, 11=U
   * Byte 0, bity 5–4: 2. číslice (0–3)
   * Byte 0, bity 3–0: 3. číslice (hex)
   * Byte 1: 4. a 5. číslice (hex)
   */
  static const char type_char[] = {'P', 'C', 'B', 'U'};

  uint8_t type_bits = (raw[0] >> 6) & 0x03;
  uint8_t digit2 = (raw[0] >> 4) & 0x03;
  uint8_t digit3 = raw[0] & 0x0F;
  uint8_t digit45 = raw[1];

  snprintf(out, 6, "%c%u%X%02X", type_char[type_bits], digit2, digit3, digit45);

  OBD2_LOGT("decode_dtc: [%02X %02X] -> %s", raw[0], raw[1], out);
}

/* ========================================================================= */
/*  Mode 03/07: Čtení diagnostických chybových kódů (DTC)                   */
/* ========================================================================= */

/**
 * @brief Interní funkce pro čtení DTC — společná implementace pro Mode 03 i 07.
 *
 * Tato statická funkce realizuje samotnou komunikaci s ECU pro získání
 * seznamu diagnostických chybových kódů. Je volána z veřejných wrapperů
 * obd2_read_dtc() (Mode $03 — potvrzené) a obd2_read_pending_dtc()
 * (Mode $07 — čekající).
 *
 * Komunikační princip:
 *   - Používá broadcast adresu 0x7DF (funkční adresování), protože
 *     dotaz na DTC je relevantní pro VŠECHNY ECU v síti (motor, převodovka,
 *     ABS, airbag, ...). Každá ECU odpoví vlastním seznamem.
 *   - V aktuální implementaci zpracováváme pouze první odpověď (primární ECU).
 *
 * Formát odpovědi (ISO 15031-5):
 *   [SID+0x40, počet_DTC, DTC1_HI, DTC1_LO, DTC2_HI, DTC2_LO, ...]
 *   - SID+0x40: kladná odpověď (0x43 pro Mode 03, 0x47 pro Mode 07)
 *   - počet_DTC: kolik DTC následuje (bajt na pozici [1])
 *   - Páry bytů: každý DTC je zakódován ve 2 bajtech (viz
 * obd2_decode_dtc_string)
 *
 * Příklad odpovědi pro 2 potvrzené DTC:
 *   [43, 02, 01, 00, 01, 31]  →  P0100, P0131
 *
 * Hraniční případy:
 *   - počet_DTC == 0: žádné závady — ECU vrátí [43, 00] a funkce vrátí
 *     out_count=0 s návratovým kódem OBD2_OK. To je dobrá zpráva!
 *   - Více než 3 DTC: data se nevejdou do jednoho CAN rámce (8 B),
 *     ISO-TP vrstva automaticky řeší multi-frame segmentaci (First Frame +
 *     Consecutive Frames), takže to je transparentní pro tuto funkci.
 *   - Odpověď kratší než 2 byty: chybný formát → OBD2_ERR_RESPONSE_MALFORMED.
 *   - max_count omezí počet rozparsovaných DTC (chrání před přetečením
 * bufferu).
 *
 * @param sid        SID služby: OBD2_SID_READ_DTC (0x03) nebo
 * OBD2_SID_PENDING_DTC (0x07)
 * @param dtcs       Výstupní pole struktur obd2_dtc_t pro nalezené DTC
 * @param max_count  Maximální počet DTC, které se vejdou do pole dtcs
 * @param out_count  Výstup: skutečný počet rozparsovaných DTC (0 = žádné)
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
static obd2_status_t _obd2_read_dtc_internal(uint8_t sid, obd2_dtc_t *dtcs,
                                             uint8_t max_count,
                                             uint8_t *out_count) {
  OBD2_LOGI("read_dtc: SID=0x%02X max=%u", sid, max_count);

  if (dtcs == NULL || out_count == NULL) {
    OBD2_LOGE("read_dtc: NULL pointer");
    return OBD2_ERR_INVALID_ARG;
  }
  if (!_ctx.initialized) {
    OBD2_LOGE("read_dtc: not initialized");
    return OBD2_ERR_NOT_INITIALIZED;
  }

  *out_count = 0;

  uint8_t req[1] = {sid};
  uint8_t resp[ISOTP_MAX_PAYLOAD];
  uint16_t resp_len = sizeof(resp);

  /* Použij broadcast — všechny ECU odpovídají na Mode 03/07 */
  obd2_status_t st = _obd2_request(req, 1, resp, &resp_len, true);
  if (st != OBD2_OK)
    return st;

  /*
   * Odpověď: [43/47, počet_DTC, DTC1_HI, DTC1_LO, DTC2_HI, DTC2_LO, ...]
   * Minimálně 2 byty (SID + počet).
   */
  if (resp_len < 2) {
    OBD2_LOGE("read_dtc: response too short (%u bytes)", resp_len);
    return OBD2_ERR_RESPONSE_MALFORMED;
  }

  uint8_t dtc_count_reported = resp[1];
  OBD2_LOGD("read_dtc: ECU reports %u DTCs, response has %u bytes",
            dtc_count_reported, resp_len);

  if (dtc_count_reported == 0) {
    OBD2_LOGI("read_dtc: no DTCs stored");
    return OBD2_OK;
  }

  /* Parsuj páry DTC bytů počínaje offsetem 2 */
  uint16_t offset = 2;
  uint8_t count = 0;

  while (offset + 1 < resp_len && count < dtc_count_reported &&
         count < max_count) {
    dtcs[count].raw[0] = resp[offset];
    dtcs[count].raw[1] = resp[offset + 1];
    obd2_decode_dtc_string(dtcs[count].raw, dtcs[count].code);
    OBD2_LOGD("read_dtc: DTC #%u = %s [%02X %02X]", count + 1, dtcs[count].code,
              dtcs[count].raw[0], dtcs[count].raw[1]);
    count++;
    offset += 2;
  }

  *out_count = count;
  OBD2_LOGI("read_dtc: parsed %u DTCs", count);
  return OBD2_OK;
}

/**
 * @brief Přečte potvrzené (stored/confirmed) DTC — Mode 03 / Service $03.
 *
 * Potvrzené DTC jsou chybové kódy, které se vyskytly opakovaně a byly
 * uloženy do trvalé paměti ECU. Obvykle rozsvítí kontrolku MIL (Check Engine).
 *
 * @param dtcs       Výstupní pole pro DTC struktury
 * @param max_count  Velikost pole dtcs
 * @param out_count  Výstup: počet nalezených DTC
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
obd2_status_t obd2_read_dtc(obd2_dtc_t *dtcs, uint8_t max_count,
                            uint8_t *out_count) {
  return _obd2_read_dtc_internal(OBD2_SID_READ_DTC, dtcs, max_count, out_count);
}

/**
 * @brief Přečte čekající (pending) DTC — Mode 07 / Service $07.
 *
 * Čekající DTC jsou chybové kódy, které se vyskytly, ale dosud nebyly
 * potvrzeny opakovaným výskytem. Po úspěšném dokončení jízdního cyklu
 * bez opakování chyby budou automaticky smazány. Pokud se chyba opakuje,
 * přesunou se do potvrzených (Mode 03).
 *
 * @param dtcs       Výstupní pole pro DTC struktury
 * @param max_count  Velikost pole dtcs
 * @param out_count  Výstup: počet nalezených čekajících DTC
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
obd2_status_t obd2_read_pending_dtc(obd2_dtc_t *dtcs, uint8_t max_count,
                                    uint8_t *out_count) {
  return _obd2_read_dtc_internal(OBD2_SID_PENDING_DTC, dtcs, max_count,
                                 out_count);
}

obd2_status_t obd2_read_permanent_dtc(obd2_dtc_t *dtcs, uint8_t max_count,
                                      uint8_t *out_count) {
  return _obd2_read_dtc_internal(OBD2_SID_PERMANENT_DTC, dtcs, max_count,
                                 out_count);
}

/**
 * @brief Multi-ECU varianta cteni DTC — zachova per-ECU separaci.
 *
 * Na rozdil od obd2_read_dtc() / obd2_read_pending_dtc() (ktere vraci jen
 * prvni nalezeny ECU resp. sloucenou pool DTC) tato funkce ulozi DTC
 * samostatne pro kazdou odpovedajici jednotku (max ISOTP_MAX_ECU_RESPONSES
 * ECU, kazde az 32 DTC — viz obd2_multi_ecu_dtc_t).
 *
 * Pouziva delsi timeout (1000 ms), aby stihly odpovedet vsechny ECU.
 * Filtruje NRC (0x7F) a odpovedi s nespravnym SID.
 *
 * @param sid     OBD2_SID_READ_DTC (0x03) nebo OBD2_SID_PENDING_DTC (0x07)
 * @param result  Vystupni struktura: pole per-ECU seznamu DTC
 * @return OBD2_OK pri uspechu, OBD2_ERR_NO_DATA pokud zadna ECU neodpovedela
 */
obd2_status_t obd2_read_dtc_multi(uint8_t sid, obd2_multi_ecu_dtc_t *result) {
  OBD2_LOGI("read_dtc_multi: SID=0x%02X", sid);

  if (result == NULL) {
    OBD2_LOGE("read_dtc_multi: NULL pointer");
    return OBD2_ERR_INVALID_ARG;
  }
  if (!_ctx.initialized) {
    OBD2_LOGE("read_dtc_multi: not initialized");
    return OBD2_ERR_NOT_INITIALIZED;
  }

  memset(result, 0, sizeof(obd2_multi_ecu_dtc_t));

  uint8_t req[1] = {sid};
  static isotp_result_t bcast_result;

  /* Delsi timeout (1000 ms) — DTC dotaz muze mit vetsi payload a potrebuje
   * vic casu na odpovedi od vsech ECU. */
  obd2_status_t st = _obd2_request_multi(req, 1, &bcast_result, 1000);
  if (st != OBD2_OK)
    return st;

  uint8_t expected_resp_sid = sid + OBD2_SID_RESPONSE_OFFSET;

  for (uint8_t i = 0; i < bcast_result.count && result->ecu_count < 8; i++) {
    isotp_response_t *r = &bcast_result.responses[i];
    if (!r->valid || r->len < 2)
      continue;

    /* Filtr: negativni odpoved (NRC) */
    if (r->data[0] == OBD2_SID_NEGATIVE_RESPONSE) {
      OBD2_LOGD("read_dtc_multi: ECU 0x%03X NRC 0x%02X, skipping", r->rx_id,
                (r->len >= 3) ? r->data[2] : 0);
      continue;
    }
    /* Filtr: nespravny SID */
    if (r->data[0] != expected_resp_sid) {
      OBD2_LOGD("read_dtc_multi: ECU 0x%03X unexpected SID 0x%02X, skipping",
                r->rx_id, r->data[0]);
      continue;
    }

    obd2_ecu_dtc_list_t *ecu_list = &result->ecus[result->ecu_count];
    ecu_list->ecu_id = r->rx_id;

    uint8_t dtc_count_reported = r->data[1];
    uint16_t offset = 2;
    uint8_t count = 0;

    while (offset + 1 < r->len && count < dtc_count_reported && count < 32) {
      ecu_list->dtcs[count].raw[0] = r->data[offset];
      ecu_list->dtcs[count].raw[1] = r->data[offset + 1];
      obd2_decode_dtc_string(ecu_list->dtcs[count].raw,
                             ecu_list->dtcs[count].code);
      OBD2_LOGD("read_dtc_multi: ECU 0x%03X DTC #%u = %s", r->rx_id, count + 1,
                ecu_list->dtcs[count].code);
      count++;
      offset += 2;
    }

    ecu_list->count = count;
    result->ecu_count++;
    OBD2_LOGI("read_dtc_multi: ECU 0x%03X parsed %u DTCs", r->rx_id, count);
  }

  return (result->ecu_count > 0) ? OBD2_OK : OBD2_ERR_NO_DATA;
}

/* ========================================================================= */
/*  Mode 04: Mazání DTC a diagnostických informací                          */
/* ========================================================================= */

/**
 * @brief Smaže všechny DTC a resetuje diagnostické informace — Mode 04 /
 * Service $04.
 *
 * POZOR: Tato operace je DESTRUKTIVNÍ a NEVRATNÁ!
 *
 * Mode 04 provede:
 *   - Smazání všech potvrzených DTC (confirmed/stored)
 *   - Smazání všech čekajících DTC (pending)
 *   - Smazání freeze-frame dat (zmrazená data v okamžiku vzniku chyby)
 *   - Reset readiness monitorů (emisní připravenost) na stav "nehotovo"
 *   - Zhasnutí kontrolky MIL (Check Engine), pokud byla rozsvícena
 *
 * VAROVÁNÍ pro uživatele:
 *   - Po smazání je nutné znovu projet jízdní cyklus, aby se readiness
 *     monitory opět nastavily na "hotovo". To může trvat několik dní jízdy.
 *   - V některých zemích (zejména USA) nesmí vozidlo projít emisní kontrolou
 *     (smog check), pokud nejsou monitory dokončeny.
 *   - Pokud je závada stále přítomna, DTC se znovu objeví po 1–2 jízdních
 * cyklech.
 *
 * Komunikační detaily:
 *   - Používá broadcast (0x7DF), protože VŠECHNY ECU mají smazat své DTC.
 *   - Odpověď je jednoduchá: [0x44] (SID+0x40 = potvrzení úspěchu).
 *     Validaci SID provádí _obd2_request() interně.
 *   - Po úspěšném smazání se invaliduje cache podporovaných PID
 *     (pids_queried = false), protože readiness bity v PID $01 se změní.
 *
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
obd2_status_t obd2_clear_dtc(void) {
  OBD2_LOGI("clear_dtc: sending Mode 04");

  if (!_ctx.initialized) {
    OBD2_LOGE("clear_dtc: not initialized");
    return OBD2_ERR_NOT_INITIALIZED;
  }

  uint8_t req[1] = {OBD2_SID_CLEAR_DTC};
  uint8_t resp[ISOTP_MAX_PAYLOAD];
  uint16_t resp_len = sizeof(resp);

  obd2_status_t st = _obd2_request(req, 1, resp, &resp_len, true);
  if (st != OBD2_OK) {
    OBD2_LOGE("clear_dtc: failed: %s", obd2_status_str(st));
    return st;
  }

  /* Odpověď je pouze [44] — již validováno v _obd2_request */
  OBD2_LOGI("clear_dtc: OK — diagnostic info cleared");

  /* Invalidace cache podporovaných PID (readiness bity se resetovaly) */
  _ctx.pids_queried = false;

  return OBD2_OK;
}

/* ========================================================================= */
/*  Mode 09: Informace o vozidle (VIN, ECU name, CalID)                     */
/* ========================================================================= */

/**
 * @brief Interní funkce pro čtení informací Mode 09 s NODI (Number Of Data
 * Items).
 *
 * Mode 09 (Service $09) slouží k získání identifikačních informací o vozidle
 * a řídicích jednotkách. Každý "InfoType" má specifický formát odpovědi,
 * ale obecná struktura je:
 *
 *   Požadavek: [09, InfoType]
 *   Odpověď:   [49, InfoType, NODI, data_1, data_2, ..., data_N]
 *
 *   - 0x49: kladná odpověď (SID 0x09 + 0x40)
 *   - InfoType: echo dotazovaného typu informace
 *   - NODI (Number Of Data Items): kolik datových položek následuje
 *     Příklady:
 *       VIN (InfoType $02): NODI=1 (jedno VIN, 17 znaků)
 *       CalID (InfoType $04): NODI=1–4 (1 až 4 kalibrační ID, každé 16 B)
 *       ECU name (InfoType $0A): NODI=1 (jeden název, max 20 znaků)
 *
 * Adresování:
 *   - Používá FYZICKOU adresu (ne broadcast), protože informace o vozidle
 *     jsou specifické pro konkrétní ECU. Při broadcast by mohlo odpovědět
 *     více ECU najednou a odpovědi by se promíchaly.
 *
 * Hraniční případy:
 *   - Některé evropské vozy (např. Peugeot 3008, Renault Mégane) nepodporují
 *     VIN přes Mode 09 → ECU vrátí negativní odpověď (NRC), typicky
 *     serviceNotSupported (0x11) nebo subFunctionNotSupported (0x12).
 *   - EOBD (European OBD) NEVYŽADUJE podporu Mode 09 InfoType $02 (VIN).
 *     VIN je v EU typicky dostupné přes UDS službu $22 (ReadDataByIdentifier).
 *   - Odpověď kratší než 3 byty → OBD2_ERR_RESPONSE_MALFORMED.
 *   - Pokud InfoType v odpovědi nesouhlasí s dotazem →
 * OBD2_ERR_RESPONSE_MALFORMED.
 *
 * @param infotype      Kód požadované informace (např. 0x02 = VIN, 0x04 =
 * CalID)
 * @param data_out      Výstupní buffer pro surová data (bez hlavičky
 * SID/InfoType/NODI)
 * @param max_len       Maximální velikost výstupního bufferu v bytech
 * @param nodi_out      Výstup: hodnota NODI z odpovědi (může být NULL, pokud
 * není potřeba)
 * @param data_len_out  Výstup: skutečná délka zkopírovaných dat v bytech
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
static obd2_status_t _obd2_read_infotype(uint8_t infotype, uint8_t *data_out,
                                         uint16_t max_len, uint8_t *nodi_out,
                                         uint16_t *data_len_out) {
  OBD2_LOGI("read_infotype: type=0x%02X", infotype);

  if (data_out == NULL || data_len_out == NULL) {
    OBD2_LOGE("read_infotype: NULL pointer");
    return OBD2_ERR_INVALID_ARG;
  }
  if (!_ctx.initialized) {
    OBD2_LOGE("read_infotype: not initialized");
    return OBD2_ERR_NOT_INITIALIZED;
  }

  uint8_t req[2] = {OBD2_SID_VEHICLE_INFO, infotype};
  uint8_t resp[ISOTP_MAX_PAYLOAD];
  uint16_t resp_len = sizeof(resp);

  obd2_status_t st = _obd2_request(req, 2, resp, &resp_len, false);
  if (st != OBD2_OK)
    return st;

  /* Odpověď: [49, InfoType, NODI, data...] — minimálně 3 byty */
  if (resp_len < 3) {
    OBD2_LOGE("read_infotype: response too short (%u bytes)", resp_len);
    return OBD2_ERR_RESPONSE_MALFORMED;
  }
  if (resp[1] != infotype) {
    OBD2_LOGE("read_infotype: InfoType mismatch: got 0x%02X", resp[1]);
    return OBD2_ERR_RESPONSE_MALFORMED;
  }

  uint8_t nodi = resp[2];
  uint16_t payload_len = resp_len - 3;
  uint16_t copy_len = (payload_len > max_len) ? max_len : payload_len;

  memcpy(data_out, &resp[3], copy_len);
  if (nodi_out)
    *nodi_out = nodi;
  *data_len_out = copy_len;

  OBD2_LOGD("read_infotype: type=0x%02X NODI=%u payload=%u bytes", infotype,
            nodi, payload_len);
  return OBD2_OK;
}

static bool _obd2_infotype_has_nodi(uint8_t infotype) {
  switch (infotype) {
  case OBD2_INFOTYPE_SUPPORTED:
  case OBD2_INFOTYPE_MSG_COUNT:
  case 0x03: /* CalID message count for non-CAN protocols */
  case 0x05: /* CVN message count for non-CAN protocols */
  case 0x07: /* IPT message count for non-CAN protocols */
  case 0x09: /* ECU name message count */
    return false;
  default:
    return true;
  }
}

obd2_status_t obd2_read_infotype_all(uint8_t infotype,
                                     obd2_infotype_list_t *list) {
  OBD2_LOGI("read_infotype_all: type=0x%02X", infotype);

  if (list == NULL)
    return OBD2_ERR_INVALID_ARG;
  if (!_ctx.initialized)
    return OBD2_ERR_NOT_INITIALIZED;

  memset(list, 0, sizeof(obd2_infotype_list_t));

  uint8_t req[2] = {OBD2_SID_VEHICLE_INFO, infotype};
  static isotp_result_t bcast_result;

  obd2_status_t st =
      _obd2_request_multi(req, 2, &bcast_result, _ctx.timeout_ms);
  if (st != OBD2_OK)
    return st;

  const uint8_t expected_sid = OBD2_SID_VEHICLE_INFO + OBD2_SID_RESPONSE_OFFSET;
  const bool has_nodi = _obd2_infotype_has_nodi(infotype);

  for (uint8_t i = 0; i < bcast_result.count; i++) {
    isotp_response_t *r = &bcast_result.responses[i];
    if (!r->valid || r->len < 2)
      continue;
    if (r->data[0] == OBD2_SID_NEGATIVE_RESPONSE)
      continue;
    if (r->data[0] != expected_sid || r->data[1] != infotype)
      continue;
    if (has_nodi && r->len < 3)
      continue;

    obd2_infotype_item_t *item = &list->items[list->count];
    item->rx_id = r->rx_id;
    item->infotype = infotype;

    uint16_t offset = 2;
    if (has_nodi) {
      item->nodi = r->data[2];
      offset = 3;
    }

    uint16_t payload_len = (r->len > offset) ? (uint16_t)(r->len - offset) : 0;
    uint16_t copy_len = (payload_len > OBD2_INFOTYPE_DATA_MAX)
                            ? OBD2_INFOTYPE_DATA_MAX
                            : payload_len;
    if (copy_len > 0) {
      memcpy(item->data, &r->data[offset], copy_len);
    }
    item->data_len = copy_len;
    item->truncated = (payload_len > copy_len);

    list->count++;
    if (list->count >= ISOTP_MAX_ECU_RESPONSES)
      break;
  }

  return (list->count > 0) ? OBD2_OK : OBD2_ERR_NO_DATA;
}

/**
 * @brief Přečte VIN (Vehicle Identification Number) — Mode 09, InfoType $02.
 *
 * VIN je unikátní 17znakový identifikátor vozidla dle ISO 3779.
 * Struktura VIN:
 *   Pozice 1–3:   WMI (World Manufacturer Identifier) — výrobce
 *   Pozice 4–9:   VDS (Vehicle Descriptor Section) — model, motor, výbava
 *   Pozice 10–17: VIS (Vehicle Identifier Section) — rok výroby, sériové číslo
 *
 * Odpověď ECU obsahuje VIN jako ASCII znaky za NODI bytem:
 *   [49, 02, 01, 'W', 'V', 'W', 'Z', 'Z', 'Z', '3', 'C', ...]
 *   NODI=1 znamená jednu datovou položku (jedno VIN).
 *
 * Hraniční případy:
 *   - EOBD (evropské vozidla) NEVYŽADUJE VIN přes OBD-II Mode 09.
 *     Mnoho evropských vozů (Peugeot, Renault, Fiat, ...) tuto funkci
 *     nepodporuje a vrátí negativní odpověď (NRC).
 *   - Buffer musí mít alespoň OBD2_VIN_LENGTH+1 (18) bytů.
 *   - Pokud ECU vrátí méně než 17 znaků → OBD2_ERR_RESPONSE_MALFORMED.
 *
 * @param vin_buf  Výstupní buffer pro null-terminated VIN řetězec (min. 18 B)
 * @param buf_len  Velikost výstupního bufferu v bytech
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
obd2_status_t obd2_read_vin(char *vin_buf, uint8_t buf_len) {
  OBD2_LOGI("read_vin");

  if (vin_buf == NULL || buf_len < OBD2_VIN_LENGTH + 1) {
    OBD2_LOGE("read_vin: invalid args (buf=%p len=%u)", vin_buf, buf_len);
    return OBD2_ERR_INVALID_ARG;
  }

  uint8_t data[OBD2_VIN_LENGTH + 4];
  uint16_t data_len = 0;
  uint8_t nodi = 0;

  obd2_status_t st = _obd2_read_infotype(OBD2_INFOTYPE_VIN, data, sizeof(data),
                                         &nodi, &data_len);
  if (st != OBD2_OK)
    return st;

  if (data_len < OBD2_VIN_LENGTH) {
    OBD2_LOGW("read_vin: short data (%u bytes, expected %u)", data_len,
              OBD2_VIN_LENGTH);
    return OBD2_ERR_RESPONSE_MALFORMED;
  }

  memcpy(vin_buf, data, OBD2_VIN_LENGTH);
  vin_buf[OBD2_VIN_LENGTH] = '\0';

  OBD2_LOGI("read_vin: \"%s\"", vin_buf);
  return OBD2_OK;
}

/**
 * @brief Multi-ECU varianta cteni VIN pres broadcast (funkcni adresa 0x7DF).
 *
 * Na rozdil od obd2_read_vin() (fyzicka adresa) tato funkce posle dotaz na
 * broadcast a nasbira VIN od vsech ECU, ktere podporuji Mode 09 InfoType $02.
 * Typicky odpovi jen jedna jednotka, ale u nekterych vozidel muze odpovedet
 * vice (napr. gateway + ECM). Kazda odpoved je zachovana s vlastnim rx_id.
 *
 * @param list  Vystupni seznam VIN s rx_id (max ISOTP_MAX_ECU_RESPONSES
 * polozek)
 * @return OBD2_OK pri uspechu, OBD2_ERR_NO_DATA pokud zadna ECU neposlala VIN
 */
obd2_status_t obd2_read_vin_all(obd2_vin_list_t *list) {
  OBD2_LOGI("read_vin_all (broadcast)");

  if (list == NULL)
    return OBD2_ERR_INVALID_ARG;
  memset(list, 0, sizeof(obd2_vin_list_t));

  uint8_t req[2] = {OBD2_SID_VEHICLE_INFO, OBD2_INFOTYPE_VIN};
  static isotp_result_t bcast_result;

  obd2_status_t st =
      _obd2_request_multi(req, 2, &bcast_result, _ctx.timeout_ms);
  if (st != OBD2_OK)
    return st;

  for (uint8_t i = 0; i < bcast_result.count; i++) {
    isotp_response_t *r = &bcast_result.responses[i];

    /* Validace: [49, 02, NODI, 17B VIN] — minimalni delka 3 + 17 = 20 B */
    if (!r->valid || r->len < 3 + OBD2_VIN_LENGTH)
      continue;
    if (r->data[0] != (OBD2_SID_VEHICLE_INFO + OBD2_SID_RESPONSE_OFFSET))
      continue;
    if (r->data[1] != OBD2_INFOTYPE_VIN)
      continue;

    obd2_vin_item_t *item = &list->items[list->count];
    item->rx_id = r->rx_id;
    memcpy(item->vin, &r->data[3], OBD2_VIN_LENGTH);
    item->vin[OBD2_VIN_LENGTH] = '\0';

    list->count++;
    if (list->count >= ISOTP_MAX_ECU_RESPONSES)
      break;
  }

  if (list->count == 0) {
    OBD2_LOGW("read_vin_all: no valid VIN in broadcast");
    return OBD2_ERR_NO_DATA;
  }

  OBD2_LOGI("read_vin_all: found %u VIN(s)", list->count);
  return OBD2_OK;
}

/**
 * @brief Přečte název ECU — Mode 09, InfoType $0A.
 *
 * InfoType $0A vrací textový název řídicí jednotky (ECU), maximálně
 * 20 ASCII znaků. Výsledek je null-terminated řetězec.
 *
 * Příklady vrácených názvů:
 *   "ECM-EngineControl" (řídicí jednotka motoru)
 *   "TCM-Transmission"  (řídicí jednotka převodovky)
 *   "ABS/ESP Module"    (modul ABS/ESP)
 *
 * Hraniční případy:
 *   - Některé ECU vrátí kratší název — funkce zkopíruje jen dostupné byty.
 *   - Buffer musí mít alespoň OBD2_ECU_NAME_MAX_LENGTH+1 (21) bytů.
 *   - Pokud ECU nepodporuje InfoType $0A → NRC (negativní odpověď).
 *
 * @param name_buf  Výstupní buffer pro null-terminated název ECU (min. 21 B)
 * @param buf_len   Velikost výstupního bufferu v bytech
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
obd2_status_t obd2_read_ecu_name(char *name_buf, uint8_t buf_len) {
  OBD2_LOGI("read_ecu_name");

  if (name_buf == NULL || buf_len < OBD2_ECU_NAME_MAX_LENGTH + 1) {
    OBD2_LOGE("read_ecu_name: invalid args");
    return OBD2_ERR_INVALID_ARG;
  }

  uint8_t data[OBD2_ECU_NAME_MAX_LENGTH + 4];
  uint16_t data_len = 0;
  uint8_t nodi = 0;

  obd2_status_t st = _obd2_read_infotype(OBD2_INFOTYPE_ECU_NAME, data,
                                         sizeof(data), &nodi, &data_len);
  if (st != OBD2_OK)
    return st;

  uint8_t copy_len = (data_len > OBD2_ECU_NAME_MAX_LENGTH)
                         ? OBD2_ECU_NAME_MAX_LENGTH
                         : (uint8_t)data_len;
  memcpy(name_buf, data, copy_len);
  name_buf[copy_len] = '\0';

  OBD2_LOGI("read_ecu_name: \"%s\"", name_buf);
  return OBD2_OK;
}

/**
 * @brief Multi-ECU varianta cteni nazvu ECU pres broadcast.
 *
 * Posle Mode 09 InfoType $0A na 0x7DF a sesbira nazvy od vsech odpovedajicich
 * ECU. Kazda polozka obsahuje rx_id puvodni jednotky. Pouzite napr. pro
 * discover-ecus funkci (zjistit, ktere ECU jsou na CAN).
 *
 * @param list  Vystupni seznam nazvu s rx_id
 * @return OBD2_OK pri uspechu, OBD2_ERR_NO_DATA pokud zadna ECU neodpovedela
 */
obd2_status_t obd2_read_ecu_names_all(obd2_ecu_name_list_t *list) {
  OBD2_LOGI("read_ecu_names_all (broadcast)");

  if (list == NULL)
    return OBD2_ERR_INVALID_ARG;
  memset(list, 0, sizeof(obd2_ecu_name_list_t));

  uint8_t req[2] = {OBD2_SID_VEHICLE_INFO, OBD2_INFOTYPE_ECU_NAME};
  static isotp_result_t bcast_result;

  obd2_status_t st =
      _obd2_request_multi(req, 2, &bcast_result, _ctx.timeout_ms);
  if (st != OBD2_OK)
    return st;

  for (uint8_t i = 0; i < bcast_result.count; i++) {
    isotp_response_t *r = &bcast_result.responses[i];

    /* Validace: [49, 0A, NODI, data...] */
    if (!r->valid || r->len < 3)
      continue;
    if (r->data[0] != (OBD2_SID_VEHICLE_INFO + OBD2_SID_RESPONSE_OFFSET))
      continue;
    if (r->data[1] != OBD2_INFOTYPE_ECU_NAME)
      continue;

    obd2_ecu_name_item_t *item = &list->items[list->count];
    item->rx_id = r->rx_id;

    uint8_t payload_len = r->len - 3;
    uint8_t copy_len = (payload_len > OBD2_ECU_NAME_MAX_LENGTH)
                           ? OBD2_ECU_NAME_MAX_LENGTH
                           : payload_len;

    memcpy(item->name, &r->data[3], copy_len);
    item->name[copy_len] = '\0';

    list->count++;
    if (list->count >= ISOTP_MAX_ECU_RESPONSES)
      break;
  }

  if (list->count == 0)
    return OBD2_ERR_NO_DATA;
  OBD2_LOGI("read_ecu_names_all: found %u name(s)", list->count);
  return OBD2_OK;
}

/**
 * @brief Přečte kalibrační ID (CalID) — Mode 09, InfoType $04.
 *
 * Kalibrační ID identifikuje verzi softwaru/kalibrace nahrané v ECU.
 * Každé CalID je přesně 16 bytů (dle ISO 15031-5). NODI v odpovědi
 * udává počet kalibračních ID (typicky 1–4).
 *
 * Formát odpovědi:
 *   [49, 04, NODI, CalID_1 (16 B), CalID_2 (16 B), ...]
 *
 * Příklady:
 *   NODI=1: jedna kalibrace, např. "CAL_ID_ENGINE_V2" (doplněno mezerami na 16
 * B) NODI=4: čtyři kalibrace — motor, převodovka, ABS, airbag
 *
 * Význam pro emisní kontrolu:
 *   - EOBD (Euro 5 a novější) VYŽADUJE podporu CalID — je důležité pro
 *     ověření, že software ECU nebyl nelegálně modifikován (chip tuning,
 *     odstranění DPF/EGR/SCR softwarem).
 *   - Emisní stanice může porovnat CalID s databází schválených kalibrací.
 *
 * Hraniční případy:
 *   - Pokud NODI * 16 > délka přijatých dat, funkce sníží počet na
 *     data_len / 16 (ochrana proti chybné odpovědi ECU).
 *   - Maximální počet CalID je omezen konstantou OBD2_MAX_INFO_ITEMS.
 *   - Každý CalID je null-terminated (funkce přidá '\0' za 16. bajt).
 *
 * @param cal_ids    Výstupní 2D pole řetězců [N][OBD2_CAL_ID_LENGTH+1]
 * @param out_count  Výstup: skutečný počet přečtených kalibračních ID
 * @return OBD2_OK při úspěchu, jinak chybový kód
 */
obd2_status_t obd2_read_cal_id(char cal_ids[][OBD2_CAL_ID_LENGTH + 1],
                               uint8_t *out_count) {
  OBD2_LOGI("read_cal_id");

  if (cal_ids == NULL || out_count == NULL) {
    OBD2_LOGE("read_cal_id: NULL pointer");
    return OBD2_ERR_INVALID_ARG;
  }

  *out_count = 0;

  uint8_t data[OBD2_CAL_ID_LENGTH * OBD2_MAX_INFO_ITEMS + 4];
  uint16_t data_len = 0;
  uint8_t nodi = 0;

  obd2_status_t st = _obd2_read_infotype(OBD2_INFOTYPE_CAL_ID, data,
                                         sizeof(data), &nodi, &data_len);
  if (st != OBD2_OK)
    return st;

  /* Každé CalID má 16 bytů, NODI udává jejich počet */
  uint8_t count = nodi;
  if (count > OBD2_MAX_INFO_ITEMS)
    count = OBD2_MAX_INFO_ITEMS;
  if (count * OBD2_CAL_ID_LENGTH > data_len) {
    count = data_len / OBD2_CAL_ID_LENGTH;
  }

  for (uint8_t i = 0; i < count; i++) {
    memcpy(cal_ids[i], &data[i * OBD2_CAL_ID_LENGTH], OBD2_CAL_ID_LENGTH);
    cal_ids[i][OBD2_CAL_ID_LENGTH] = '\0';
    OBD2_LOGD("read_cal_id: #%u = \"%s\"", i + 1, cal_ids[i]);
  }

  *out_count = count;
  OBD2_LOGI("read_cal_id: %u CalIDs", count);
  return OBD2_OK;
}

/**
 * @brief Multi-ECU varianta cteni CalID pres broadcast.
 *
 * Posle Mode 09 InfoType $04 na 0x7DF a sesbira CalID od vsech odpovedajicich
 * ECU. Kazda polozka zachovava rx_id a ma vlastni seznam az OBD2_MAX_INFO_ITEMS
 * kalibracnich ID (kazde 16 B). Pouzitelne napr. pro emisni kontrolu
 * (porovnani CalID napric vsemi jednotkami).
 *
 * @param list  Vystupni seznam CalID per ECU
 * @return OBD2_OK pri uspechu, OBD2_ERR_NO_DATA pokud zadna ECU neodpovedela
 */
obd2_status_t obd2_read_calids_all(obd2_calid_list_t *list) {
  OBD2_LOGI("read_calids_all (broadcast)");

  if (list == NULL)
    return OBD2_ERR_INVALID_ARG;
  memset(list, 0, sizeof(obd2_calid_list_t));

  uint8_t req[2] = {OBD2_SID_VEHICLE_INFO, OBD2_INFOTYPE_CAL_ID};
  static isotp_result_t bcast_result;

  obd2_status_t st =
      _obd2_request_multi(req, 2, &bcast_result, _ctx.timeout_ms);
  if (st != OBD2_OK)
    return st;

  for (uint8_t i = 0; i < bcast_result.count; i++) {
    isotp_response_t *r = &bcast_result.responses[i];

    /* Validace: [49, 04, NODI, data...] */
    if (!r->valid || r->len < 3)
      continue;
    if (r->data[0] != (OBD2_SID_VEHICLE_INFO + OBD2_SID_RESPONSE_OFFSET))
      continue;
    if (r->data[1] != OBD2_INFOTYPE_CAL_ID)
      continue;

    obd2_ecu_calid_item_t *item = &list->items[list->count];
    item->rx_id = r->rx_id;

    uint8_t nodi = r->data[2];
    uint8_t count = nodi;
    if (count > OBD2_MAX_INFO_ITEMS)
      count = OBD2_MAX_INFO_ITEMS;

    uint16_t payload_len = r->len - 3;
    if (count * OBD2_CAL_ID_LENGTH > payload_len) {
      count = payload_len / OBD2_CAL_ID_LENGTH;
    }

    for (uint8_t j = 0; j < count; j++) {
      memcpy(item->cal_ids[j], &r->data[3 + j * OBD2_CAL_ID_LENGTH],
             OBD2_CAL_ID_LENGTH);
      item->cal_ids[j][OBD2_CAL_ID_LENGTH] = '\0';
    }
    item->count = count;

    list->count++;
    if (list->count >= ISOTP_MAX_ECU_RESPONSES)
      break;
  }

  if (list->count == 0)
    return OBD2_ERR_NO_DATA;
  OBD2_LOGI("read_calids_all: found %u ECU(s)", list->count);
  return OBD2_OK;
}

/**
 * @brief Provede manuální dotaz na auto a vrátí surová data (bez dekódování).
 *
 * @param service Service ID (např. 0x01, 0x09)
 * @param pid     PID (např. 0x0C)
 * @param out_res Ukazatel na strukturu, kam se uloží výsledek
 * @return obd2_status_t Stav operace
 */
obd2_status_t obd2_read_cvns_all(obd2_cvn_list_t *list) {
  OBD2_LOGI("read_cvns_all (broadcast)");

  if (list == NULL)
    return OBD2_ERR_INVALID_ARG;
  memset(list, 0, sizeof(obd2_cvn_list_t));

  obd2_infotype_list_t raw_list;
  obd2_status_t st = obd2_read_infotype_all(OBD2_INFOTYPE_CVN, &raw_list);
  if (st != OBD2_OK)
    return st;

  for (uint8_t i = 0;
       i < raw_list.count && list->count < ISOTP_MAX_ECU_RESPONSES; i++) {
    const obd2_infotype_item_t *raw = &raw_list.items[i];
    obd2_ecu_cvn_item_t *item = &list->items[list->count];
    item->rx_id = raw->rx_id;

    uint8_t count = raw->nodi;
    if (count > OBD2_MAX_INFO_ITEMS)
      count = OBD2_MAX_INFO_ITEMS;
    if ((uint16_t)count * OBD2_CVN_LENGTH > raw->data_len) {
      count = raw->data_len / OBD2_CVN_LENGTH;
    }

    for (uint8_t j = 0; j < count; j++) {
      const uint8_t *p = &raw->data[j * OBD2_CVN_LENGTH];
      item->cvns[j] = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                      ((uint32_t)p[2] << 8) | (uint32_t)p[3];
    }
    item->count = count;
    list->count++;
  }

  return (list->count > 0) ? OBD2_OK : OBD2_ERR_NO_DATA;
}

obd2_status_t obd2_query_raw_ex(uint8_t service, uint8_t pid,
                                obd2_raw_response_t *out_res, bool use_broadcast) {
  if (out_res == NULL)
    return OBD2_ERR_INVALID_ARG;
  if (!_ctx.initialized)
    return OBD2_ERR_NOT_INITIALIZED;

  memset(out_res, 0, sizeof(obd2_raw_response_t));
  out_res->service = service;
  out_res->pid = pid;

  /* Příprava požadavku */
  uint8_t req[2] = {service, pid};
  uint8_t req_len = 2;

  /* Pro módy 03, 04, 07 se neposílá PID */
  if (service == OBD2_SID_READ_DTC || service == OBD2_SID_CLEAR_DTC ||
      service == OBD2_SID_PENDING_DTC ||
      service == OBD2_SID_PERMANENT_DTC) {
    req_len = 1;
    out_res->pid = 0;
  }

  uint8_t resp[ISOTP_MAX_PAYLOAD];
  uint16_t resp_len = sizeof(resp);

  obd2_status_t st = _obd2_request(req, req_len, resp, &resp_len, use_broadcast);

  /* Pro terminál je i negativní odpověď "platnou" zprávou pro zobrazení */
  if (st == OBD2_ERR_NEGATIVE_RESP) {
    out_res->is_negative = true;
    out_res->nrc_code = _ctx.last_nrc.nrc;
    out_res->rx_id = use_broadcast && _ctx.init_diag.last_rx_id
                         ? _ctx.init_diag.last_rx_id
                         : _ctx.rx_id;
    out_res->data_len =
        (resp_len > ISOTP_MAX_PAYLOAD) ? ISOTP_MAX_PAYLOAD : resp_len;
    memcpy(out_res->data, resp, out_res->data_len);
    return OBD2_OK;
  }

  if (st != OBD2_OK) {
    return st;
  }

  out_res->rx_id = use_broadcast && _ctx.init_diag.last_rx_id
                       ? _ctx.init_diag.last_rx_id
                       : _ctx.rx_id;
  out_res->data_len =
      (resp_len > ISOTP_MAX_PAYLOAD) ? ISOTP_MAX_PAYLOAD : resp_len;
  memcpy(out_res->data, resp, out_res->data_len);
  out_res->is_negative = false;

  return OBD2_OK;
}

obd2_status_t obd2_query_raw(uint8_t service, uint8_t pid,
                             obd2_raw_response_t *out_res) {
  /* Zpetna kompatibilita: puvodni funkce vzdy vola fyzicke adresovani */
  return obd2_query_raw_ex(service, pid, out_res, false);
}
