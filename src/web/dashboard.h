/**
 * @file dashboard.h
 * @brief Vestavenya jednostrankova diagnosticka aplikace (dashboard) ulozena v PROGMEM
 *
 * Tento soubor obsahuje kompletni HTML/CSS/JS webovou stranku zakodovanou
 * jako PROGMEM retezec pro ESP32. Stranka se servuje pres AsyncWebServer
 * a komunikuje s ESP32 pres WebSocket (JSON protokol).
 *
 * Obsahuje nasledujici funkcni celky:
 *   - SVG ukazatele (gauge) pro otacky motoru (RPM) a rychlost vozidla
 *   - Zalozka "Dash" zobrazujici vsechny podporovane PID hodnoty v bublinach
 *     vcetne sparkline minigrafu historie poslednich 60 vzorku
 *   - DTC panel pro cteni a zobrazeni diagnostickych poruchovych kodu
 *   - Ovladani datoveho proudu (start/stop, vyber PID, nastaveni intervalu)
 *   - Cteni VIN cisla, nazvu ridici jednotky, kalibracniho ID a stavu monitoru
 *   - Komunikacni log s fixni vyskou a vlastnim posuvnikem (max 300 radku)
 *   - Zalozka "Settings" pro parametry vozidla ukladane do localStorage prohlizece
 *   - Zalozka "Stats" s vypoctem okamzite a prumerne spotreby paliva,
 *     odhadem zarazeneho prevodoveho stupne a zrychleni
 *   - Zalozka "Map" s GPS logovanim polohy pomoci Geolocation API,
 *     vizualizaci trasy na canvasu a exportem do GPX formatu
 *
 * Veskeré texty uzivatelskeho rozhrani (tlacitka, popisky, HTML) jsou v anglictine.
 * Komentare v kodu jsou v cestine.
 *
 * @author Ales Pouzar
 */

#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <pgmspace.h>

static const char dashboard_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>OBD-II ISO 15031-5</title>
<style>
/* ---- OEM Minimalist / Mobile-First CSS pro OBD2 Dashboard ---- */
:root {
  --bg: #0b0c10;
  --bg2: #16181d;
  --bg3: #1f2229;
  --fg: #f0f0f0;
  --fg2: #8a8d93;
  --accent: #26a69a;
  --ok: #2e7d32;
  --err: #d32f2f;
  --warn: #f57c00;
  --nav-h: 64px;
}

* { box-sizing: border-box; margin: 0; padding: 0; }

body {
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
  background: var(--bg); 
  color: var(--fg);
  padding: 0 12px calc(var(--nav-h) + 16px) 12px;
  max-width: 600px;
  margin: 0 auto;
  -webkit-user-select: none; user-select: none;
  -webkit-tap-highlight-color: transparent;
}

.hdr {
  display: flex; align-items: center; gap: 10px;
  padding: 18px 18px 16px; margin: 0 -12px 12px -12px;
  position: sticky; top: 0; z-index: 100;
  background: rgba(11, 12, 16, 0.95);
  backdrop-filter: blur(10px); -webkit-backdrop-filter: blur(10px);
  border-bottom: 1px solid rgba(255,255,255,0.05);
}
.hdr h1 { 
  font-size: 1.1em; font-weight: 600; color: var(--fg); flex: 1; letter-spacing: 0.5px; margin: 0;
}
.dot {
  width: 8px; height: 8px; border-radius: 50%;
  background: #444; transition: background 0.3s, box-shadow 0.3s;
}
.dot.on { background: var(--ok); box-shadow: 0 0 8px rgba(46,125,50,0.6); }
.dot.err { background: var(--err); box-shadow: 0 0 8px rgba(211,47,47,0.6); }
.hdr .info { font-size: 0.75em; color: var(--fg2); font-weight: 500; }

.tabs {
  position: fixed; bottom: 0; left: 0; right: 0;
  height: var(--nav-h); background: rgba(22, 24, 29, 0.95);
  backdrop-filter: blur(10px); -webkit-backdrop-filter: blur(10px);
  display: flex; justify-content: space-around; align-items: center;
  border-top: 1px solid rgba(255,255,255,0.05); z-index: 1000;
  padding-bottom: env(safe-area-inset-bottom);
}
.tab {
  flex: 1; text-align: center; font-size: 0.70rem; font-weight: 600;
  padding: 8px 0; color: rgba(138, 141, 147, 0.6); cursor: pointer;
  text-transform: uppercase; letter-spacing: 0.5px;
  transition: all 0.2s;
  height: 100%; display: flex; align-items: center; justify-content: center;
}
.tab.active { color: var(--accent); font-weight: 700; transform: translateY(-2px); }
.tab-content { display: none; padding-top: 4px; }
.tab-content.active { display: block; animation: fadeIn 0.3s ease; }
@keyframes fadeIn { from { opacity: 0; transform: translateY(4px); } to { opacity: 1; transform: translateY(0); } }

.gauges { display: flex; gap: 12px; margin-bottom: 20px; }
.gauge-wrap {
  background: var(--bg2); border-radius: 20px; padding: 20px 10px 12px;
  flex: 1; text-align: center; position: relative;
}
.gauge-wrap svg { width: 100%; height: auto; max-width: 200px; display: block; margin: 0 auto; filter: drop-shadow(0 4px 6px rgba(0,0,0,0.3)); }
.gauge-val {
  font-size: 2.2em; font-weight: 300; color: var(--fg);
  margin: -10px 0 2px; line-height: 1;
}
.gauge-lbl { font-size: 0.75em; color: var(--fg2); font-weight: 500; text-transform: uppercase; letter-spacing: 1px; }

.gauge-arc-bg { fill: none; stroke: var(--bg3); stroke-width: 8; stroke-linecap: round; }
.gauge-arc { fill: none; stroke-width: 8; stroke-linecap: round; transition: stroke-dashoffset 0.4s cubic-bezier(0.4, 0, 0.2, 1), stroke 0.3s; }

.btns { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 16px; }
.btn {
  background: var(--bg3); color: var(--fg); border: none;
  padding: 12px 18px; border-radius: 12px; cursor: pointer;
  font-family: inherit; font-size: 0.85em; font-weight: 600;
  transition: background 0.2s, transform 0.1s;
  flex: 1 1 calc(50% - 8px); min-width: 120px;
}
.btn:hover { background: #2a2e37; }
.btn:active { transform: scale(0.96); background: #2a2e37; }
.btn.on { background: rgba(38, 166, 154, 0.15); color: var(--accent); }
.btn.rec { background: rgba(211, 47, 47, 0.15); color: var(--err); }
.btn:disabled { opacity: 0.3; cursor: not-allowed; }

.rec-indicator { 
  display: inline-block; width: 8px; height: 8px; border-radius: 50%;
  background: var(--err); margin-right: 6px; box-shadow: 0 0 6px var(--err);
  animation: blink 1s infinite;
}
@keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }

.panel {
  background: var(--bg2); border-radius: 20px; padding: 18px;
  margin-bottom: 12px;
}
.panel-title { 
  font-size: 0.75em; font-weight: 600; color: var(--fg2);
  text-transform: uppercase; letter-spacing: 1px; margin-bottom: 14px;
}

.dash-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.pid-bubble {
  background: var(--bg2); border-radius: 16px; padding: 14px 12px 10px;
  text-align: center; border: 1px solid rgba(255,255,255,0.02);
  transition: background 0.3s; display: flex; flex-direction: column; justify-content: center;
}
.pid-bubble.fresh { background: rgba(38, 166, 154, 0.1); }
.pid-bubble .pb-val { font-size: 1.4em; font-weight: 600; color: var(--fg); font-variant-numeric: tabular-nums; }
.pid-bubble .pb-unit { font-size: 0.6em; color: var(--fg2); font-weight: normal; margin-left: 2px; }
.pid-bubble .pb-name { font-size: 0.7em; color: var(--fg2); margin-top: 4px; font-weight: 500; }
.pid-bubble canvas { display: block; width: 100%; height: 36px; margin-top: 8px; opacity: 0.8; }

.dtc-list { color: var(--warn); font-size: 0.9em; line-height: 1.5; }
.dtc-list.empty { color: var(--ok); }
#infoContent { font-size: 0.85em; line-height: 1.5; color: var(--fg); }
#infoContent strong { color: var(--fg2); font-weight: 500; }

.stream-cfg { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; margin-bottom: 12px; }
.stream-cfg label { font-size: 0.8em; color: var(--fg2); }
.stream-cfg input {
  background: var(--bg3); border: none; color: var(--fg);
  padding: 8px 12px; border-radius: 8px; font-family: inherit; font-size: 0.9em; width: 80px; text-align: center;
}
.stream-cfg input:focus { outline: 1px solid var(--accent); }
.pid-checks { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 5px; }
.pid-chk {
  font-size: 0.75em; padding: 8px 12px; background: var(--bg3); color: var(--fg2);
  border-radius: 20px; cursor: pointer; transition: 0.2s; font-weight: 500; border: 1px solid transparent;
}
.pid-chk.sel { background: rgba(38, 166, 154, 0.15); color: var(--accent); border-color: rgba(38, 166, 154, 0.3); }

#logBox {
  background: var(--bg2); border-radius: 16px; padding: 12px; height: 60vh; overflow-y: auto;
  font-family: ui-monospace, SFMono-Regular, Consolas, "Liberation Mono", Menlo, monospace;
  font-size: 0.75em; line-height: 1.6; border: 1px solid rgba(255,255,255,0.02);
}
#logBox::-webkit-scrollbar { width: 4px; }
#logBox::-webkit-scrollbar-thumb { background: var(--bg3); border-radius: 4px; }
.m-in { color: var(--ok); } .m-out { color: var(--warn); } .m-sys { color: var(--fg2); }

.settings-grid { display: flex; flex-direction: column; gap: 10px; }
.set-group { background: var(--bg3); border-radius: 16px; padding: 14px; border: 1px solid rgba(255,255,255,0.02); }
.set-group label { display: block; font-size: 0.75em; font-weight: 500; color: var(--fg2); margin-bottom: 6px; }
.set-group input, .set-group select {
  width: 100%; background: var(--bg); border: none; color: var(--fg);
  padding: 10px 12px; border-radius: 8px; font-family: inherit; font-size: 0.9em;
}
.set-group input:focus, .set-group select:focus { outline: 1px solid var(--accent); }
.set-group .set-hint { font-size: 0.65em; color: rgba(255,255,255,0.3); margin-top: 6px; }

.stats-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 12px; }
.stat-card { background: var(--bg3); border-radius: 16px; padding: 14px; text-align: center; }
.stat-val { font-size: 1.5em; font-weight: 600; color: var(--fg); font-variant-numeric: tabular-nums; }
.stat-unit { font-size: 0.6em; color: var(--fg2); margin-left: 2px; font-weight: normal; }
.stat-lbl { font-size: 0.7em; color: var(--fg2); margin-top: 4px; font-weight: 500; }
.stat-card.wide { grid-column: 1 / -1; }

#mapCanvas { width: 100%; height: 350px; background: var(--bg3); border-radius: 16px; display: block; border: 1px solid rgba(255,255,255,0.02); }
.gps-info { display: flex; flex-direction: column; gap: 6px; margin-top: 12px; font-size: 0.85em; }
.gps-info span { color: var(--fg2); display: flex; justify-content: space-between; padding: 6px 12px; background: var(--bg3); border-radius: 8px; }
.gps-info strong { color: var(--fg); font-weight: 600; }

@media (max-width: 380px) {
  .gauges { flex-direction: column; gap: 12px; }
  .btn { flex: 1 1 100%; }
}
</style>
</head>
<body>

<!-- Hlavicka stranky s nazvem, stavovou teckou a informacemi o pripojeni -->
<div class="hdr">
  <h1>OBD-II &mdash; ISO 15031-5</h1>
  <div class="dot" id="dot"></div>
  <span class="info" id="connTxt">Disconnected</span>
  <span class="info" id="heapTxt"></span>
</div>

<!-- Spodni navigacni lista (Mobile-First) -->
<div class="tabs">
  <div class="tab active" onclick="switchTab('main')">Stream</div>
  <div class="tab" onclick="switchTab('dash')">Dash</div>
  <div class="tab" onclick="switchTab('stats')">Trip</div>
  <div class="tab" onclick="switchTab('map')">Map</div>
  <div class="tab" onclick="switchTab('diag')">Diag</div>
  <div class="tab" onclick="switchTab('settings')">Cfg</div>
  <div class="tab" onclick="switchTab('log')">Log</div>
</div>

<!-- ============================================================ -->
<!-- ZALOZKA: Stream (ukazatele otacek/rychlosti + ovladaci prvky) -->
<!-- ============================================================ -->
<div class="tab-content active" id="tab_main">

  <!-- SVG ukazatele: otacky motoru (RPM) a rychlost vozidla (km/h) -->
  <div class="gauges">
    <div class="gauge-wrap">
      <svg viewBox="0 0 200 115">
        <path d="M 20 105 A 80 80 0 0 1 180 105" class="gauge-arc-bg"/>
        <path d="M 20 105 A 80 80 0 0 1 180 105" class="gauge-arc" id="arc_rpm"
          stroke="#00e5ff" stroke-dasharray="251" stroke-dashoffset="251"/>
      </svg>
      <div class="gauge-val" id="val_rpm">&mdash;</div>
      <div class="gauge-lbl">RPM</div>
    </div>
    <div class="gauge-wrap">
      <svg viewBox="0 0 200 115">
        <path d="M 20 105 A 80 80 0 0 1 180 105" class="gauge-arc-bg"/>
        <path d="M 20 105 A 80 80 0 0 1 180 105" class="gauge-arc" id="arc_spd"
          stroke="#00e5ff" stroke-dasharray="251" stroke-dashoffset="251"/>
      </svg>
      <div class="gauge-val" id="val_spd">&mdash;</div>
      <div class="gauge-lbl">km/h</div>
    </div>
  </div>

  <!-- Ovladaci tlacitka: inicializace OBD, spusteni/zastaveni streamu,
       nahravani dat a export do CSV -->
  <div class="btns">
    <button class="btn" id="btnInit" onclick="cmd({cmd:'init'})">Init OBD</button>
    <button class="btn" id="btnStream" onclick="toggleStream()" disabled>Start Stream</button>
    <button class="btn" id="btnRec" onclick="toggleRec()" disabled>Record</button>
    <button class="btn" id="btnExport" onclick="exportCSV()" disabled>Export CSV</button>
    <button class="btn" onclick="cmd({cmd:'ping'})">Ping</button>
  </div>

  <!-- Panel konfigurace datoveho proudu — interval a vyber PID -->
  <div class="panel" id="streamPanel" style="display:none">
    <div class="panel-title">Stream Configuration</div>
    <div class="stream-cfg">
      <label>Interval (ms):</label>
      <input type="number" id="streamInt" value="200" min="50" max="5000" step="50" style="width:70px">
    </div>
    <div class="pid-checks" id="pidChecks"></div>
  </div>
</div>

<!-- ============================================================ -->
<!-- ZALOZKA: Dash (vsechny podporovane PID jako interaktivni bubliny) -->
<!-- ============================================================ -->
<div class="tab-content" id="tab_dash">
  <div class="panel">
    <div class="panel-title">Live PID Values</div>
    <div class="dash-grid" id="dashGrid">
      <div style="color:var(--fg2);font-size:0.82em;padding:20px;text-align:center;width:100%">
        Press <strong>Init OBD</strong> and <strong>Start Stream</strong> to see live data.
      </div>
    </div>
  </div>
</div>

<!-- ============================================================ -->
<!-- ZALOZKA: Stats (vypocet spotreby paliva + statistiky jizdy)   -->
<!-- ============================================================ -->
<div class="tab-content" id="tab_stats">
  <div class="panel">
    <div class="panel-title">Trip Statistics</div>
    <div class="btns" style="margin-bottom:6px">
      <button class="btn" id="btnTrip" onclick="toggleTrip()">Start Trip</button>
      <button class="btn" onclick="resetTrip()">Reset Trip</button>
    </div>
    <div class="stats-grid">
      <div class="stat-card">
        <div class="stat-val" id="st_fc_inst">&mdash;<span class="stat-unit">L/h</span></div>
        <div class="stat-lbl">Fuel Flow (instant)</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_fc_100">&mdash;<span class="stat-unit">L/100km</span></div>
        <div class="stat-lbl">Consumption</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_fc_avg">&mdash;<span class="stat-unit">L/100km</span></div>
        <div class="stat-lbl">Average Consumption</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_fuel_used">&mdash;<span class="stat-unit">L</span></div>
        <div class="stat-lbl">Fuel Used</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_dist">&mdash;<span class="stat-unit">km</span></div>
        <div class="stat-lbl">Trip Distance</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_cost">&mdash;<span class="stat-unit"></span></div>
        <div class="stat-lbl">Fuel Cost</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_time">&mdash;</div>
        <div class="stat-lbl">Trip Time</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_avg_spd">&mdash;<span class="stat-unit">km/h</span></div>
        <div class="stat-lbl">Average Speed</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_gear">&mdash;</div>
        <div class="stat-lbl">Estimated Gear</div>
      </div>
      <div class="stat-card">
        <div class="stat-val" id="st_accel">&mdash;<span class="stat-unit">m/s&sup2;</span></div>
        <div class="stat-lbl">Acceleration</div>
      </div>
    </div>
  </div>
</div>

<!-- ============================================================ -->
<!-- ZALOZKA: Map (GPS zaznamenavani trasy + vizualizace na canvasu) -->
<!-- ============================================================ -->
<div class="tab-content" id="tab_map">
  <div class="panel">
    <div class="panel-title">GPS Track</div>
    <div class="btns" style="margin-bottom:6px">
      <button class="btn" id="btnGps" onclick="toggleGps()">Start GPS</button>
      <button class="btn" onclick="exportGpx()" id="btnGpxExport" disabled>Export GPX</button>
      <button class="btn" onclick="clearTrack()">Clear Track</button>
    </div>
    <canvas id="mapCanvas"></canvas>
    <div class="gps-info" id="gpsInfo">
      <span>Status: <strong id="gpsStatus">Inactive</strong></span>
      <span>Points: <strong id="gpsPoints">0</strong></span>
      <span>Lat: <strong id="gpsLat">&mdash;</strong></span>
      <span>Lon: <strong id="gpsLon">&mdash;</strong></span>
      <span>GPS Speed: <strong id="gpsSpd">&mdash;</strong></span>
      <span>Distance: <strong id="gpsDist">&mdash;</strong></span>
    </div>
  </div>
</div>

<!-- ============================================================ -->
<!-- ZALOZKA: Diagnostika (DTC kody, VIN, stav monitoru emisi)     -->
<!-- ============================================================ -->
<div class="tab-content" id="tab_diag">
  <div class="btns">
    <button class="btn" onclick="cmd({cmd:'get_supported_pids'})" disabled id="btnSup">Supported PIDs</button>
    <button class="btn" onclick="cmd({cmd:'get_dtc'})" disabled id="btnDtc">Read DTC</button>
    <button class="btn" onclick="cmd({cmd:'get_pending_dtc'})" disabled id="btnPdtc">Pending DTC</button>
    <button class="btn" onclick="cmd({cmd:'get_vin'})" disabled id="btnVin">Read VIN</button>
    <button class="btn" onclick="cmd({cmd:'get_ecu_name'})" disabled id="btnEcu">ECU Name</button>
    <button class="btn" onclick="cmd({cmd:'get_cal_id'})" disabled id="btnCal">Calibration ID</button>
    <button class="btn" onclick="cmd({cmd:'get_monitor_status'})" disabled id="btnMon">Monitor Status</button>
  </div>

  <!-- Panel diagnostickych poruchovych kodu (DTC) -->
  <div class="panel" id="dtcPanel" style="display:none">
    <div class="panel-title">Diagnostic Trouble Codes (DTC)</div>
    <div id="dtcContent" class="dtc-list empty">No DTCs</div>
  </div>

  <!-- Informacni panel (VIN, stav monitoru, nazev ECU apod.) -->
  <div class="panel" id="infoPanel" style="display:none">
    <div class="panel-title">Vehicle Information</div>
    <div id="infoContent"></div>
  </div>
</div>

<!-- ============================================================ -->
<!-- ZALOZKA: Nastaveni (parametry vozidla pro vypocty spotreby)   -->
<!-- ============================================================ -->
<div class="tab-content" id="tab_settings">
  <div class="panel">
    <div class="panel-title">Vehicle Parameters</div>
    <p style="font-size:0.72em;color:var(--fg2);margin-bottom:10px">
      These values are used for fuel consumption and statistics calculations.
      Saved automatically to browser storage.
    </p>
    <div class="settings-grid">
      <div class="set-group">
        <label>Fuel Type</label>
        <select id="set_fuel_type" onchange="saveSetting(this)">
          <option value="diesel">Diesel</option>
          <option value="gasoline">Gasoline</option>
          <option value="lpg">LPG</option>
          <option value="cng">CNG</option>
        </select>
      </div>
      <div class="set-group">
        <label>Engine Displacement (L)</label>
        <input type="number" id="set_displacement" value="2.0" min="0.5" max="8.0" step="0.1" onchange="saveSetting(this)">
        <div class="set-hint">e.g. 2.0 for 2000cc</div>
      </div>
      <div class="set-group">
        <label>Fuel Density (g/L)</label>
        <input type="number" id="set_fuel_density" value="832" min="500" max="1000" step="1" onchange="saveSetting(this)">
        <div class="set-hint">Diesel ~832, Gasoline ~745, LPG ~550</div>
      </div>
      <div class="set-group">
        <label>Stoichiometric AFR</label>
        <input type="number" id="set_afr" value="14.6" min="6" max="20" step="0.1" onchange="saveSetting(this)">
        <div class="set-hint">Diesel 14.6, Gasoline 14.7, LPG 15.7</div>
      </div>
      <div class="set-group">
        <label>Fuel Price per Liter</label>
        <input type="number" id="set_fuel_price" value="1.50" min="0" max="10" step="0.01" onchange="saveSetting(this)">
        <div class="set-hint">In your local currency</div>
      </div>
      <div class="set-group">
        <label>Currency Symbol</label>
        <input type="text" id="set_currency" value="CZK" maxlength="5" onchange="saveSetting(this)">
      </div>
    </div>
  </div>

</div>

<!-- ============================================================ -->
<!-- ZALOZKA: Log (komunikacni log s fixni vyskou a posuvnikem)    -->
<!-- ============================================================ -->
<div class="tab-content" id="tab_log">
  <div id="logBox"></div>
</div>

<script>
/* ================================================================ */
/*  Globalni stav aplikace                                          */
/*  ws         — instance WebSocket pripojeni k ESP32               */
/*  streaming  — priznak, zda probiha kontinualni cteni PID         */
/*  obdReady   — priznak, zda probehla uspesna inicializace OBD     */
/*  supportedPids — pole PID cisel, ktere ECU vozidla podporuje     */
/*  streamPids — aktualne vybrane PID pro streamovani (vychozi:     */
/*               RPM=12, rychlost=13, teplota chladici kap.=5)     */
/*  pidNames   — cache pro nazvy PID (pouziva se pri zobrazeni)     */
/* ================================================================ */
var ws = null, streaming = false, obdReady = false;
var supportedPids = [], streamPids = [12, 13, 5];
var pidNames = {};

/* Vyhledavaci tabulka informaci o beznych PID v rezimu 01 (Mode 01).
   Kazdy zaznam obsahuje zkraceny nazev (n) a jednotku (u).
   Pouziva se pro pojmenovani bublin, CSV hlavicek a PID stitku. */
var PID_INFO = {
  4:{n:'Engine Load',u:'%'}, 5:{n:'Coolant Temp',u:'\u00B0C'},
  6:{n:'Short Fuel Trim B1',u:'%'}, 7:{n:'Long Fuel Trim B1',u:'%'},
  11:{n:'Intake MAP',u:'kPa'}, 12:{n:'Engine RPM',u:'rpm'},
  13:{n:'Vehicle Speed',u:'km/h'}, 14:{n:'Timing Advance',u:'\u00B0'},
  15:{n:'Intake Air Temp',u:'\u00B0C'}, 16:{n:'MAF Rate',u:'g/s'},
  17:{n:'Throttle Pos',u:'%'}, 28:{n:'OBD Standard',u:''},
  29:{n:'O2 Sensors',u:''}, 31:{n:'Run Time',u:'s'},
  33:{n:'Dist w/ MIL',u:'km'}, 35:{n:'Fuel Rail Press',u:'kPa'},
  36:{n:'O2 B1S1 EQ Ratio',u:''}, 51:{n:'Fuel Type',u:''},
  70:{n:'ECT 2',u:'\u00B0C'}, 92:{n:'Engine Oil Temp',u:'\u00B0C'}
};

/* Buffer historie hodnot pro sparkline minigrafy.
   Pro kazdy PID se uchovava poslednich HIST_LEN (60) vzorku.
   Hodnoty se pridavaji pri kazdem prijetom stream paketu
   a nejstarsi se zahazuji, kdyz buffer prekroci limit. */
var HIST_LEN = 60;
var pidHistory = {};  /* pid -> [hodnota, hodnota, ...] */

/* Prida novou hodnotu do kruhoveho bufferu historie daneho PID.
   Pokud buffer prekroci HIST_LEN, nejstarsi vzorek se odebere. */
function pushHistory(pid, val) {
  if (!pidHistory[pid]) pidHistory[pid] = [];
  var h = pidHistory[pid];
  h.push(val);
  if (h.length > HIST_LEN) h.shift();
}

/* Vykresli sparkline minigraf na zadany canvas prvek.
   Vstupem je pole ciselnych hodnot. Funkce najde minimum a maximum
   pro normalizaci, pak vykresli vyplneny gradient pod carou
   a samotnou caru s azurovou barvou. Sparkline umoznuje uzivately
   videt trend hodnoty PID v case primo v bubline. */
function drawSparkline(canvas, data) {
  if (!canvas || data.length < 2) return;
  var ctx = canvas.getContext('2d');
  var w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  var min = data[0], max = data[0];
  for (var i = 1; i < data.length; i++) {
    if (data[i] < min) min = data[i];
    if (data[i] > max) max = data[i];
  }
  var range = max - min || 1;
  var pad = 2;

  /* Vytvoreni linearniho gradientu pod carou grafu —
     pruhledna azurova nahoze, uplne pruhledna dole */
  var grad = ctx.createLinearGradient(0, 0, 0, h);
  grad.addColorStop(0, 'rgba(38,166,154,0.18)');
  grad.addColorStop(1, 'rgba(38,166,154,0)');

  ctx.beginPath();
  for (var i = 0; i < data.length; i++) {
    var x = (i / (data.length - 1)) * w;
    var y = h - pad - ((data[i] - min) / range) * (h - pad * 2);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  /* Uzavreni oblasti pro vyplneni gradientem */
  ctx.lineTo(w, h); ctx.lineTo(0, h); ctx.closePath();
  ctx.fillStyle = grad; ctx.fill();

  /* Vykresleni samotne cary sparkline grafu */
  ctx.beginPath();
  for (var i = 0; i < data.length; i++) {
    var x = (i / (data.length - 1)) * w;
    var y = h - pad - ((data[i] - min) / range) * (h - pad * 2);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.strokeStyle = '#26a69a';
  ctx.lineWidth = 1.5;
  ctx.stroke();
}

/* ================================================================ */
/*  Nahravani dat — zaznamenavani stream vzorku do pameti            */
/*  a jejich nasledny export jako CSV soubor                        */
/*                                                                  */
/*  recData je pole objektu {ts: casove razitko, d: {pid: hodnota}} */
/*  MAX_REC_ROWS omezuje pocet vzorku na ~18000 (cca 1 hodina       */
/*  pri frekvenci 5 Hz, zabira priblizne 1 MB v pameti prohlizece). */
/*  Po dosazeni limitu se nahravani automaticky zastavi.             */
/* ================================================================ */
var recording = false;
var recData = [];     /* [{ts:milisekundy, d:{pid:hodnota,...}}, ...] */
var recStart = 0;
var MAX_REC_ROWS = 18000; /* ~1h pri 5Hz, ~1MB v pameti */

/* Prepinac nahravani — spusti nebo zastavi zaznamenavani vzorku.
   Pred spustenim je nutne mit aktivni stream. Pri zastaveni
   se odemkne tlacitko pro export CSV. */
function toggleRec() {
  if (!streaming) { syslog('Start stream first!'); return; }
  recording = !recording;
  var btn = document.getElementById('btnRec');
  if (recording) {
    recData = []; recStart = Date.now();
    btn.innerHTML = '<span class="rec-indicator"></span>Stop Rec';
    btn.classList.add('rec');
    document.getElementById('btnExport').disabled = true;
    syslog('Recording started');
  } else {
    btn.textContent = 'Record';
    btn.classList.remove('rec');
    document.getElementById('btnExport').disabled = (recData.length === 0);
    syslog('Recording stopped: ' + recData.length + ' samples');
  }
}

/* Ulozi jeden vzorek do nahravaci pameti.
   Vola se pri kazdem prijatem stream paketu.
   Pokud pocet vzorku dosahne MAX_REC_ROWS, nahravani se automaticky zastavi. */
function recordSample(d, ts) {
  if (!recording) return;
  if (recData.length >= MAX_REC_ROWS) {
    recording = false;
    document.getElementById('btnRec').textContent = 'Record';
    document.getElementById('btnRec').classList.remove('rec');
    document.getElementById('btnExport').disabled = false;
    syslog('Recording auto-stopped: max ' + MAX_REC_ROWS + ' samples');
    return;
  }
  recData.push({ts: ts || Date.now(), d: Object.assign({}, d)});
}

/* Export nahranich dat do CSV souboru.
   Postup:
   1. Sebere vsechny unikatni PID klice ze vsech vzorku
   2. Vytvori hlavicku s casovymi sloupci a pojmenovanymi PID sloupci
   3. Sestavi radky s hodnotami (prazdne kde PID nema hodnotu)
   4. Vytvori Blob s MIME typem text/csv a spusti stahovani
      pomoci docasneho <a> elementu s vygenerovanym nazvem souboru */
function exportCSV() {
  if (recData.length === 0) { syslog('No recorded data'); return; }

  /* Sber vsech unikatnich PID klicu napric vsemi vzorky */
  var keys = {};
  recData.forEach(function(r) { for (var k in r.d) keys[k] = true; });
  var pidKeys = Object.keys(keys).sort(function(a,b){ return parseInt(a)-parseInt(b); });

  /* Hlavicka CSV: casove razitko v ms, cas v sekundach + sloupce PID s nazvy */
  var header = 'time_ms,time_s';
  pidKeys.forEach(function(pid) {
    var info = PID_INFO[pid] || {n:'PID_'+pid};
    header += ',' + info.n.replace(/,/g, ' ') + ' (0x' + parseInt(pid).toString(16).toUpperCase() + ')';
  });

  /* Sestaveni datovych radku — cas relativni k prvnimu vzorku */
  var lines = [header];
  recData.forEach(function(r) {
    var elapsed = r.ts - recData[0].ts;
    var row = elapsed + ',' + (elapsed/1000).toFixed(2);
    pidKeys.forEach(function(pid) {
      row += ',' + (r.d[pid] !== undefined ? r.d[pid] : '');
    });
    lines.push(row);
  });

  /* Vytvoreni Blob objektu a spusteni stahovani souboru */
  var csv = lines.join('\n');
  var blob = new Blob([csv], {type: 'text/csv'});
  var url = URL.createObjectURL(blob);
  var a = document.createElement('a');
  var now = new Date();
  a.href = url;
  a.download = 'obd2_log_' + now.toISOString().slice(0,19).replace(/[:-]/g,'') + '.csv';
  a.click();
  URL.revokeObjectURL(url);
  syslog('Exported ' + recData.length + ' rows as CSV');
}

/* ================================================================ */
/*  Nastaveni — parametry vozidla ukladane do localStorage          */
/*                                                                  */
/*  Pristup k persistenci:                                          */
/*  - Vsechna nastaveni se ukladaji jako jeden JSON objekt pod      */
/*    klicem 'obd2_settings' v localStorage prohlizece              */
/*  - Pri nacteni stranky se hodnoty obnovi z localStorage          */
/*    a aplikuji do vstupnich poli formulare                        */
/*  - Kazda zmena pole okamzite serializuje cely objekt zpet        */
/*  - Presets automaticky vyplni hustotu a AFR pri zmene typu paliva*/
/* ================================================================ */
var SETTINGS_KEY = 'obd2_settings';
var settings = {
  fuel_type: 'diesel', displacement: 2.0, fuel_density: 832,
  afr: 14.6, fuel_price: 1.50, currency: 'CZK'
};

/* Nacteni ulozemich nastaveni z localStorage.
   Pokud existuji, aplikuji se na vychozi objekt settings.
   Hodnoty se pote propisou do odpovidajicich vstupnich poli na strance. */
function loadSettings() {
  try {
    var saved = localStorage.getItem(SETTINGS_KEY);
    if (saved) {
      var parsed = JSON.parse(saved);
      for (var k in parsed) {
        if (settings.hasOwnProperty(k)) settings[k] = parsed[k];
      }
    }
  } catch(e) {}
  /* Aplikovani hodnot do vstupnich poli formulare */
  for (var k in settings) {
    var el = document.getElementById('set_' + k);
    if (el) el.value = settings[k];
  }
}

/* Ulozi zmenenou hodnotu jednoho nastaveni.
   Cisla se parsuje jako float, ostatni jako retezec.
   Cely objekt nastaveni se serializuje do localStorage. */
function saveSetting(el) {
  var key = el.id.replace('set_', '');
  if (el.type === 'number') settings[key] = parseFloat(el.value) || 0;
  else settings[key] = el.value;
  try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings)); } catch(e) {}
}

/* Prednastavene hodnoty hustoty paliva a stechiometrickeho pomeru
   vzduch/palivo (AFR) pro ruzne typy paliv */
var FUEL_PRESETS = {
  diesel:   {density: 832, afr: 14.6},
  gasoline: {density: 745, afr: 14.7},
  lpg:      {density: 550, afr: 15.7},
  cng:      {density: 720, afr: 17.2}
};

/* Automaticke predvyplneni hustoty paliva a AFR pri zmene typu paliva.
   Listener se napoji pri nacteni stranky pomoci IIFE (okamzite volane funkce). */
(function() {
  var sel = document.getElementById('set_fuel_type');
  if (sel) sel.addEventListener('change', function() {
    var preset = FUEL_PRESETS[this.value];
    if (preset) {
      document.getElementById('set_fuel_density').value = preset.density;
      document.getElementById('set_afr').value = preset.afr;
      settings.fuel_density = preset.density;
      settings.afr = preset.afr;
      saveSetting(this);
    }
  });
})();

/* ================================================================ */
/*  Statistiky — vypocet spotreby paliva a statistik jizdy          */
/*                                                                  */
/*  Hlavni vzorec spotreby paliva (zalozeny na MAF senzoru):        */
/*    prutok_paliva [L/h] = (MAF [g/s] * 3.6) / (AFR * hustota)   */
/*  kde MAF je hmotnostni prutok nasavaneho vzduchu,                */
/*  AFR je stechiometricky pomer vzduch/palivo a hustota paliva     */
/*  se prepocitava z g/L na kg/L delenim 1000.                     */
/*                                                                  */
/*  Zarazeny prevodovy stupen se odhaduje z pomeru otacky/rychlost: */
/*    rpk = RPM / rychlost [km/h]                                  */
/*  Vyssi pomer = nizsi prevodovy stupen. Prahove hodnoty:          */
/*    >120: 1. stupen, >70: 2., >45: 3., >33: 4., >25: 5., jinak 6.*/
/*                                                                  */
/*  Zrychleni se pocita jako zmena rychlosti v m/s za cas:          */
/*    a = (v2 - v1) / 3.6 / dt   [m/s^2]                          */
/* ================================================================ */
var tripActive = false;
var tripData = {
  startTime: 0, totalFuel: 0, totalDist: 0,
  lastSpeed: 0, lastTime: 0, speedSum: 0, speedCount: 0,
  lastSpeedForAccel: -1, lastAccelTime: 0
};

/* Prepinac sledovani jizdy — spusti nebo zastavi kumulaci
   statistickych dat (spotreba, vzdalenost, prumerna rychlost) */
function toggleTrip() {
  tripActive = !tripActive;
  var btn = document.getElementById('btnTrip');
  if (tripActive) {
    tripData.startTime = Date.now();
    tripData.lastTime = Date.now();
    btn.textContent = 'Stop Trip';
    btn.classList.add('on');
    syslog('Trip tracking started');
  } else {
    btn.textContent = 'Start Trip';
    btn.classList.remove('on');
    syslog('Trip tracking stopped');
  }
}

/* Resetuje vsechna data jizdy do vychoziho stavu
   a vymaze zobrazene hodnoty na kartach statistik */
function resetTrip() {
  tripActive = false;
  tripData = {startTime:0, totalFuel:0, totalDist:0,
    lastSpeed:0, lastTime:0, speedSum:0, speedCount:0,
    lastSpeedForAccel:-1, lastAccelTime:0};
  document.getElementById('btnTrip').textContent = 'Start Trip';
  document.getElementById('btnTrip').classList.remove('on');
  var ids = ['st_fc_inst','st_fc_100','st_fc_avg','st_fuel_used',
             'st_dist','st_cost','st_time','st_avg_spd','st_gear','st_accel'];
  ids.forEach(function(id) {
    var el = document.getElementById(id);
    var unit = el.querySelector('.stat-unit');
    el.innerHTML = '&mdash;' + (unit ? unit.outerHTML : '');
  });
  syslog('Trip data reset');
}

/* Hlavni funkce aktualizace statistik — vola se pri kazdem stream paketu.
   Provadi vypocet okamzite spotreby, spotreby na 100 km, kumulaci
   celkoveho paliva a vzdalenosti, odhad prevodoveho stupne a zrychleni. */
function updateStats(d) {
  var now = Date.now();
  var dt = (now - tripData.lastTime) / 1000; /* sekundy od posledni aktualizace */
  if (dt <= 0 || dt > 5) { tripData.lastTime = now; return; }
  tripData.lastTime = now;

  var maf = d['16'];   /* MAF prutok vzduchu g/s — PID 0x10 */
  var speed = d['13'];  /* Rychlost vozidla km/h — PID 0x0D */
  var rpm = d['12'];    /* Otacky motoru RPM — PID 0x0C */
  var load = d['4'];    /* Zatizeni motoru % — PID 0x04 */

  /* ---- Okamzity prutok paliva (L/h) ---- */
  /* Primarni vzorec vyuziva MAF senzor (hmotnostni prutok vzduchu):
     prutok_paliva [L/h] = (MAF [g/s] * 3600) / (lambda * AFR * hustota [g/L])
     Lambda je predpokladana 1.0 (stechiometricka smes). U dieselu
     motor bezi typicky chude (lambda 1.3-2.0), ale MAF uz reflektuje
     skutecny prutok vzduchu, takze lambda=1 dava stechiometrickou
     hmotnost paliva. ECU ridi vstrikovani tak, aby odpovidal MAF. */
  var fuelFlow = null;
  if (maf !== undefined && maf > 0) {
    fuelFlow = (maf * 3.6) / (settings.afr * (settings.fuel_density / 1000));
  } else if (load !== undefined && rpm !== undefined && rpm > 0) {
    /* Zaloha: odhad ze zatizeni motoru + objemu valcu.
       Prutok vzduchu se aproximuje z objemove ucinnosti motoru,
       otacek a hustoty vzduchu (1.225 kg/m3). Tato metoda je
       mene presna nez MAF, ale funguje bez MAF senzoru. */
    var airFlow = (load / 100) * (settings.displacement / 1000) * (rpm / 120) * 1.225;
    fuelFlow = (airFlow * 3.6) / (settings.afr * (settings.fuel_density / 1000));
  }

  /* ---- Zobrazeni okamziteho prutoku paliva ---- */
  if (fuelFlow !== null) {
    setStatVal('st_fc_inst', fuelFlow.toFixed(2), 'L/h');
  }

  /* ---- Spotreba v L/100km ---- */
  /* Prepocet z L/h na L/100km: (prutok * 100) / rychlost.
     Pri rychlosti pod 3 km/h se nezobrazuje (deleni nulou, nesmyslna hodnota). */
  if (fuelFlow !== null && speed !== undefined && speed > 3) {
    var lPer100 = (fuelFlow * 100) / speed;
    if (lPer100 > 99) lPer100 = 99;
    setStatVal('st_fc_100', lPer100.toFixed(1), 'L/100km');
  } else if (speed !== undefined && speed <= 3 && fuelFlow !== null) {
    setStatVal('st_fc_100', '---', 'L/100km');
  }

  /* ---- Kumulace dat jizdy (trip) ---- */
  if (tripActive) {
    /* Spotrebovane palivo — integrace prutoku v case (L/h -> L/s * dt) */
    if (fuelFlow !== null) {
      tripData.totalFuel += (fuelFlow / 3600) * dt;
    }

    /* Ujeta vzdalenost — integrace rychlosti v case (km/h -> m/s * dt = metry) */
    if (speed !== undefined) {
      tripData.totalDist += (speed / 3.6) * dt;
      tripData.speedSum += speed;
      tripData.speedCount++;
    }

    /* Zobrazeni hodnot jizdy na kartach statistik */
    var distKm = tripData.totalDist / 1000;
    setStatVal('st_fuel_used', tripData.totalFuel.toFixed(3), 'L');
    setStatVal('st_dist', distKm.toFixed(2), 'km');

    /* Prumerna spotreba — celkove palivo / vzdalenost * 100 */
    if (distKm > 0.05) {
      var avgCons = (tripData.totalFuel / distKm) * 100;
      setStatVal('st_fc_avg', avgCons.toFixed(1), 'L/100km');
    }

    /* Naklady na palivo — celkove spotrebovane litry * cena za litr */
    var cost = tripData.totalFuel * settings.fuel_price;
    setStatVal('st_cost', cost.toFixed(2), ' ' + settings.currency);

    /* Cas jizdy — prepocet sekund na format HH:MM:SS */
    var elapsed = (now - tripData.startTime) / 1000;
    var mm = Math.floor(elapsed / 60);
    var ss = Math.floor(elapsed % 60);
    var hh = Math.floor(mm / 60);
    mm = mm % 60;
    setStatVal('st_time', (hh ? hh + 'h ' : '') + mm + 'm ' + ss + 's', '');

    /* Prumerna rychlost — soucet vsech rychlosti / pocet mereni */
    if (tripData.speedCount > 0) {
      setStatVal('st_avg_spd', (tripData.speedSum / tripData.speedCount).toFixed(1), 'km/h');
    }
  }

  /* ---- Odhad zarazeneho prevodoveho stupne ---- */
  /* Algorimus pouziva pomer otacky/rychlost (rpk = RPM / km/h).
     Vyssi hodnota rpk znamena nizsi prevodovy stupen.
     Prahove hodnoty jsou priblizne a mohou se lisit podle vozidla.
     Odhad funguje pouze pri otackach > 500 a rychlosti > 5 km/h. */
  if (rpm !== undefined && speed !== undefined && rpm > 500 && speed > 5) {
    var rpk = rpm / speed;
    var gear;
    if      (rpk > 120) gear = 1;
    else if (rpk > 70)  gear = 2;
    else if (rpk > 45)  gear = 3;
    else if (rpk > 33)  gear = 4;
    else if (rpk > 25)  gear = 5;
    else                 gear = 6;
    setStatVal('st_gear', gear.toString(), '');
  }

  /* ---- Vypocet zrychleni ---- */
  /* Zrychleni se pocita jako zmena rychlosti (prevedena z km/h na m/s)
     delena casovym intervalem: a = ((v2 - v1) / 3.6) / dt [m/s^2].
     Interval musi byt mezi 0.1 a 3 sekundami pro vylouceni
     chybnych hodnot pri prestce nebo dlouhem vpadku dat. */
  if (speed !== undefined) {
    if (tripData.lastSpeedForAccel >= 0) {
      var aDt = (now - tripData.lastAccelTime) / 1000;
      if (aDt > 0.1 && aDt < 3) {
        var accel = ((speed - tripData.lastSpeedForAccel) / 3.6) / aDt;
        setStatVal('st_accel', accel.toFixed(2), 'm/s\u00B2');
      }
    }
    tripData.lastSpeedForAccel = speed;
    tripData.lastAccelTime = now;
  }
}

/* Pomocna funkce pro nastaveni hodnoty a jednotky na statisticke karte */
function setStatVal(id, val, unit) {
  var el = document.getElementById(id);
  if (!el) return;
  el.innerHTML = val + (unit ? '<span class="stat-unit">' + unit + '</span>' : '');
}

/* ================================================================ */
/*  Mapa — GPS zaznamenavani trasy a vizualizace na canvasu         */
/*                                                                  */
/*  Pouziva Geolocation API prohlizece (navigator.geolocation)      */
/*  s watchPosition pro kontinualni sledovani polohy.               */
/*  Body trasy se ukladaji do pole gpsTrack s casovym razitkem,     */
/*  rychlosti a nadmorskou vyskou.                                  */
/*  Vzdalenost mezi body se pocita Haversinovym vzorcem.            */
/*  Vizualizace na canvasu pouziva ekvirektangularni projekci       */
/*  s korekci pomeru stran podle zemepisne sirky (cosinus).         */
/*  Segmenty trasy jsou barevne rozliseny podle rychlosti:          */
/*    zelena (pomala) -> azurova (stredni) -> cervena (rychla).     */
/*  Export trasy do standardniho GPX 1.1 formatu s casem,           */
/*  nadmorskou vyskou a rychlosti u kazdeho trackpointu.            */
/* ================================================================ */
var gpsActive = false;
var gpsWatchId = null;
var gpsTrack = [];      /* [{lat, lon, ts, speed, alt}, ...] */
var gpsTotalDist = 0;   /* celkova vzdalenost v metrech */

/* Prepinac GPS sledovani — spusti nebo zastavi watchPosition.
   Pouziva vysokou presnost (enableHighAccuracy: true) s timeoutem 10s. */
function toggleGps() {
  if (!navigator.geolocation) {
    syslog('Geolocation not supported by browser');
    return;
  }
  gpsActive = !gpsActive;
  var btn = document.getElementById('btnGps');
  if (gpsActive) {
    btn.textContent = 'Stop GPS';
    btn.classList.add('on');
    document.getElementById('gpsStatus').textContent = 'Acquiring...';
    gpsWatchId = navigator.geolocation.watchPosition(
      onGpsPosition,
      onGpsError,
      {enableHighAccuracy: false, maximumAge: 2000, timeout: 10000}
    );
    syslog('GPS tracking started (Low Power Mode)');
  } else {
    btn.textContent = 'Start GPS';
    btn.classList.remove('on');
    if (gpsWatchId !== null) {
      navigator.geolocation.clearWatch(gpsWatchId);
      gpsWatchId = null;
    }
    document.getElementById('gpsStatus').textContent = 'Stopped';
    syslog('GPS tracking stopped (' + gpsTrack.length + ' points)');
  }
}

/* Callback volany pri kazde nove GPS pozici.
   Vytvori bod trasy, spocita vzdalenost od predchoziho bodu
   pomoci Haversinova vzorce a aktualizuje UI informace.
   Body s posunem mene nez 1 metr se ignoruji (GPS jitter/sum). */
function onGpsPosition(pos) {
  var pt = {
    lat: pos.coords.latitude,
    lon: pos.coords.longitude,
    ts: pos.timestamp || Date.now(),
    speed: pos.coords.speed,
    alt: pos.coords.altitude
  };

  /* Vypocet vzdalenosti od predchoziho bodu Haversinovym vzorcem */
  if (gpsTrack.length > 0) {
    if (pos.coords.accuracy && pos.coords.accuracy > 40) return; /* Ignorovat zcela nepresne body */
    var prev = gpsTrack[gpsTrack.length - 1];
    var d = haversine(prev.lat, prev.lon, pt.lat, pt.lon);
    if (d < 3) return; /* Preskocit pokud posun < 3m (Automobilovy filtr sumu) */
    gpsTotalDist += d;
  }

  gpsTrack.push(pt);
  document.getElementById('gpsStatus').textContent = 'Active';
  document.getElementById('gpsPoints').textContent = gpsTrack.length;
  document.getElementById('gpsLat').textContent = pt.lat.toFixed(6);
  document.getElementById('gpsLon').textContent = pt.lon.toFixed(6);
  document.getElementById('gpsDist').textContent =
    gpsTotalDist > 1000 ? (gpsTotalDist/1000).toFixed(2) + ' km'
                        : Math.round(gpsTotalDist) + ' m';
  if (pt.speed !== null && pt.speed !== undefined) {
    document.getElementById('gpsSpd').textContent = (pt.speed * 3.6).toFixed(1) + ' km/h';
  }

  document.getElementById('btnGpxExport').disabled = (gpsTrack.length < 2);
  drawMap();
}

/* Callback volany pri chybe geolokace (odmitnuti opravneni, timeout apod.) */
function onGpsError(err) {
  document.getElementById('gpsStatus').textContent = 'Error: ' + err.message;
  syslog('GPS error: ' + err.message);
}

/* Haversinuv vzorec — vypocet vzdalenosti dvou GPS bodu v metrech.
   Vzorec bere v uvahu zakriveni Zeme pomoci polomerem R = 6371000 m.
   Vstup: zemepisna sirka a delka obou bodu ve stupnich.
   Vystup: vzdalenost v metrech. */
function haversine(lat1, lon1, lat2, lon2) {
  var R = 6371000;
  var dLat = (lat2 - lat1) * Math.PI / 180;
  var dLon = (lon2 - lon1) * Math.PI / 180;
  var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
          Math.cos(lat1 * Math.PI/180) * Math.cos(lat2 * Math.PI/180) *
          Math.sin(dLon/2) * Math.sin(dLon/2);
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
}

/* Vykresleni GPS trasy na canvas — ekvirektangularni projekce s barvami rychlosti.
   Postup:
   1. Zjisti hranice (min/max lat/lon) vsech bodu trasy
   2. Prida odsazeni (padding) kolem hranic
   3. Koriguje pomer stran podle cosinu stredni zemepisne sirky
      (na vysokych sirkach je stupen delky kratsi nez stupen sirky)
   4. Vypocte meritko tak, aby se trasa vesla do canvasu
   5. Vykresli segmenty s barevnym prechodem podle rychlosti:
      zelena (pomala) -> azurova (stredni) -> cervena (rychla)
   6. Nakresli zeleny bod na zacatek a azurovy bod na konec trasy
   7. Zobrazi meritkovou listu a legendu barev rychlosti */
function drawMap() {
  var canvas = document.getElementById('mapCanvas');
  if (!canvas || gpsTrack.length < 2) return;
  var ctx = canvas.getContext('2d');

  /* Nastaveni rozliseni canvasu podle velikosti zobrazeni a DPI displeje */
  var rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * (window.devicePixelRatio || 1);
  canvas.height = rect.height * (window.devicePixelRatio || 1);
  ctx.scale(window.devicePixelRatio || 1, window.devicePixelRatio || 1);
  var w = rect.width, h = rect.height;

  ctx.clearRect(0, 0, w, h);

  /* Zjisteni geografickych hranic trasy (bounding box) */
  var minLat = gpsTrack[0].lat, maxLat = gpsTrack[0].lat;
  var minLon = gpsTrack[0].lon, maxLon = gpsTrack[0].lon;
  for (var i = 1; i < gpsTrack.length; i++) {
    if (gpsTrack[i].lat < minLat) minLat = gpsTrack[i].lat;
    if (gpsTrack[i].lat > maxLat) maxLat = gpsTrack[i].lat;
    if (gpsTrack[i].lon < minLon) minLon = gpsTrack[i].lon;
    if (gpsTrack[i].lon > maxLon) maxLon = gpsTrack[i].lon;
  }

  /* Pridani odsazeni (15%) kolem hranic pro lepsi vizualni vysledek */
  var latRange = maxLat - minLat || 0.001;
  var lonRange = maxLon - minLon || 0.001;
  var pad = 0.15;
  minLat -= latRange * pad; maxLat += latRange * pad;
  minLon -= lonRange * pad; maxLon += lonRange * pad;
  latRange = maxLat - minLat;
  lonRange = maxLon - minLon;

  /* Korekce pomeru stran zavisle na zemepisne sirce —
     na vyssich sirkach je stupen delky kratsi (cos efekt) */
  var midLat = (minLat + maxLat) / 2;
  var cosLat = Math.cos(midLat * Math.PI / 180);
  var scaledLonRange = lonRange * cosLat;

  /* Vypocet meritka tak, aby se trasa vesla do canvasu se zachovanim pomeru stran */
  var scaleX = w / scaledLonRange;
  var scaleY = h / latRange;
  var scale = Math.min(scaleX, scaleY);
  var offX = (w - scaledLonRange * scale) / 2;
  var offY = (h - latRange * scale) / 2;

  function toX(lon) { return offX + (lon - minLon) * cosLat * scale; }
  function toY(lat) { return h - offY - (lat - minLat) * scale; }

  /* Zjisteni maximalni rychlosti pro normalizaci barevneho mapovani */
  var maxSpd = 1;
  for (var i = 0; i < gpsTrack.length; i++) {
    var s = gpsTrack[i].speed;
    if (s !== null && s !== undefined && s > maxSpd) maxSpd = s;
  }

  /* Vykresleni segmentu trasy s barvou podle rychlosti */
  ctx.lineWidth = 3;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  for (var i = 1; i < gpsTrack.length; i++) {
    var spd = gpsTrack[i].speed || 0;
    var ratio = Math.min(1, spd / maxSpd);
    /* Barevny prechod: zelena (pomala) -> azurova (stredni) -> cervena (rychla) */
    var r, g, b;
    if (ratio < 0.5) {
      r = 0; g = Math.round(230 * (1 - ratio * 2) + 229 * ratio * 2);
      b = Math.round(229 * ratio * 2);
    } else {
      var t = (ratio - 0.5) * 2;
      r = Math.round(244 * t); g = Math.round(229 * (1 - t));
      b = Math.round(255 * (1 - t));
    }
    ctx.strokeStyle = 'rgb(' + r + ',' + g + ',' + b + ')';
    ctx.beginPath();
    ctx.moveTo(toX(gpsTrack[i-1].lon), toY(gpsTrack[i-1].lat));
    ctx.lineTo(toX(gpsTrack[i].lon), toY(gpsTrack[i].lat));
    ctx.stroke();
  }

  /* Znacka zacatku trasy (zeleny kruh) */
  ctx.beginPath();
  ctx.arc(toX(gpsTrack[0].lon), toY(gpsTrack[0].lat), 5, 0, Math.PI * 2);
  ctx.fillStyle = '#00e676';
  ctx.fill();

  /* Znacka konce trasy (azurovy kruh) */
  var last = gpsTrack[gpsTrack.length - 1];
  ctx.beginPath();
  ctx.arc(toX(last.lon), toY(last.lat), 5, 0, Math.PI * 2);
  ctx.fillStyle = '#26a69a';
  ctx.fill();

  /* Meritkova lista v levem dolnim rohu — zobrazuje vzdalenost v m nebo km */
  var scaleBarM = calcScaleBar(latRange, h, scale);
  if (scaleBarM) {
    var barPx = scaleBarM.meters / (latRange / h * 111320);
    var barX = 10, barY = h - 15;
    ctx.strokeStyle = '#666';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(barX, barY); ctx.lineTo(barX + barPx, barY);
    ctx.moveTo(barX, barY - 4); ctx.lineTo(barX, barY + 4);
    ctx.moveTo(barX + barPx, barY - 4); ctx.lineTo(barX + barPx, barY + 4);
    ctx.stroke();
    ctx.fillStyle = '#888';
    ctx.font = '11px sans-serif';
    ctx.fillText(scaleBarM.label, barX + barPx / 2 - 10, barY - 6);
  }

  /* Legenda barev rychlosti v pravem hornim rohu */
  ctx.font = '10px sans-serif';
  ctx.fillStyle = '#00e676'; ctx.fillText('slow', w - 80, 15);
  ctx.fillStyle = '#26a69a'; ctx.fillText('mid', w - 52, 15);
  ctx.fillStyle = '#f44336'; ctx.fillText('fast', w - 26, 15);
}

/* Vypocet vhodne delky meritkove listy.
   Z rozsahu zemepisne sirky a vysky canvasu urcime metry na pixel,
   pak vybereme nejblizsi "peknou" hodnotu (10, 20, 50, 100, ... 10000 m)
   tak, aby lista merila priblizne 80 pixelu. */
function calcScaleBar(latRange, canvasH, scale) {
  var metersPerPx = (latRange * 111320) / canvasH;
  var targetPx = 80;
  var targetM = metersPerPx * targetPx;
  var nice = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000];
  for (var i = 0; i < nice.length; i++) {
    if (nice[i] >= targetM * 0.5) {
      var label = nice[i] >= 1000 ? (nice[i]/1000) + ' km' : nice[i] + ' m';
      return {meters: nice[i], label: label};
    }
  }
  return null;
}

/* Smazani cele GPS trasy — vyprazdni pole bodu, vynuluje vzdalenost
   a procisti canvas */
function clearTrack() {
  gpsTrack = [];
  gpsTotalDist = 0;
  document.getElementById('gpsPoints').textContent = '0';
  document.getElementById('gpsDist').textContent = '\u2014';
  document.getElementById('btnGpxExport').disabled = true;
  var canvas = document.getElementById('mapCanvas');
  if (canvas) {
    var ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
  }
  syslog('GPS track cleared');
}

/* Export GPS trasy do GPX 1.1 formatu (standard pro vymenu GPS dat).
   GPX soubor obsahuje jeden element <trk> s jednim segmentem <trkseg>.
   Kazdy trackpoint <trkpt> ma atributy lat/lon a volitelne elementy:
   <ele> pro nadmorskou vysku, <time> pro casove razitko ISO 8601
   a <speed> pro rychlost v m/s.
   Soubor se stahne pomoci docasneho Blob URL a <a> elementu. */
function exportGpx() {
  if (gpsTrack.length < 2) { syslog('No GPS data to export'); return; }

  var gpx = '<?xml version="1.0" encoding="UTF-8"?>\n';
  gpx += '<gpx version="1.1" creator="OBD2-ESP32-Dashboard">\n';
  gpx += '  <trk><name>OBD2 Drive ' + new Date().toISOString().slice(0,10) + '</name>\n';
  gpx += '    <trkseg>\n';
  gpsTrack.forEach(function(pt) {
    gpx += '      <trkpt lat="' + pt.lat.toFixed(7) + '" lon="' + pt.lon.toFixed(7) + '">';
    if (pt.alt !== null && pt.alt !== undefined) gpx += '<ele>' + pt.alt.toFixed(1) + '</ele>';
    gpx += '<time>' + new Date(pt.ts).toISOString() + '</time>';
    if (pt.speed !== null && pt.speed !== undefined) gpx += '<speed>' + pt.speed.toFixed(2) + '</speed>';
    gpx += '</trkpt>\n';
  });
  gpx += '    </trkseg>\n  </trk>\n</gpx>';

  var blob = new Blob([gpx], {type: 'application/gpx+xml'});
  var url = URL.createObjectURL(blob);
  var a = document.createElement('a');
  a.href = url;
  a.download = 'obd2_track_' + new Date().toISOString().slice(0,19).replace(/[:-]/g,'') + '.gpx';
  a.click();
  URL.revokeObjectURL(url);
  syslog('Exported ' + gpsTrack.length + ' GPS points as GPX');
}

/* ================================================================ */
/*  System zalozek (tabu)                                           */
/*  Prepinani mezi sedmi zalozkami: Stream, Dash, Stats, Map,       */
/*  Diagnostics, Settings a Log. Aktivni zalozka se zvyrazni        */
/*  a jeji obsah se zobrazi, ostatni se skryji.                     */
/*  Pri prepnuti na zalozku Map se s kratkym zpozdenim prekresli    */
/*  canvas (kvuli spravnemu vypoctu rozmeru po zmene zobrazeni).    */
/* ================================================================ */
var TAB_IDS = ['main','dash','stats','map','diag','settings','log'];

/* Prepne aktivni zalozku — nastavi CSS tridu .active na zvolenou zalozku
   i jeji obsah a skryje ostatni */
function switchTab(name) {
  document.querySelectorAll('.tab').forEach(function(t,i) {
    t.classList.toggle('active', TAB_IDS[i] === name);
  });
  document.querySelectorAll('.tab-content').forEach(function(c) {
    c.classList.toggle('active', c.id === 'tab_' + name);
  });
  /* Prekresleni mapy pri prepnuti na zalozku Map (kvuli layoutu) */
  if (name === 'map' && gpsTrack.length >= 2) {
    setTimeout(drawMap, 50);
  }
}

/* ================================================================ */
/*  WebSocket pripojeni k ESP32                                     */
/*  Pripojuje se na ws://<hostname>/ws (adresa ESP32 AP).           */
/*  Logika opetovneho pripojeni:                                    */
/*  - Pri otevreni se nastavi zeleny indikator a stav "Connected"   */
/*  - Prichozi zpravy se parsuje jako JSON a predaji handleru       */
/*  - Pri uzavreni spojeni se automaticky pokusi o znovupripojeni   */
/*    po 2 sekundach (setTimeout) a nastavi stav "Reconnecting"     */
/*  - Pri chybe se nastavi cerveny indikator                        */
/*  Funkce cmd() odesila JSON prikazy na server, pokud je spojeni   */
/*  otevrene (readyState === 1 = OPEN).                             */
/* ================================================================ */
function connect() {
  var url = 'ws://' + location.hostname + '/ws';
  syslog('Connecting to ' + url + '...');
  ws = new WebSocket(url);

  ws.onopen = function() {
    document.getElementById('dot').className = 'dot on';
    document.getElementById('connTxt').textContent = 'Connected';
    syslog('WebSocket OPEN');
  };
  ws.onmessage = function(e) {
    logMsg('in', e.data);
    try { handleResponse(JSON.parse(e.data)); } catch(err) {}
  };
  ws.onclose = function() {
    document.getElementById('dot').className = 'dot';
    document.getElementById('connTxt').textContent = 'Reconnecting...';
    syslog('WebSocket CLOSED');
    streaming = false; updateStreamBtn();
    setTimeout(connect, 2000);
  };
  ws.onerror = function() {
    document.getElementById('dot').className = 'dot err';
    syslog('WebSocket ERROR');
  };
}

/* Odesle JSON prikaz na ESP32 pres WebSocket.
   Pred odeslanim overi, ze je spojeni aktivni. */
function cmd(obj) {
  if (!ws || ws.readyState !== 1) { syslog('Not connected!'); return; }
  var json = JSON.stringify(obj);
  logMsg('out', json);
  ws.send(json);
}

/* ================================================================ */
/*  Zpracovani odpovedi ze serveru (dispatcher)                     */
/*  Hlavni funkce handleResponse rozdeluje odpovedi podle pole      */
/*  r.cmd a vola prislusne funkce pro aktualizaci UI:               */
/*  - 'init': inicializace OBD probehla, odemknuti tlacitek        */
/*  - 'stream': aktualizace ukazatelu, bublin, statistik, nahravani*/
/*  - 'start/stop_stream': zmena stavu streamovani                 */
/*  - 'get_pid/get_pids': zobrazeni vysledku cteni PID             */
/*  - 'get_supported_pids': naplneni seznamu podporovanych PID     */
/*  - 'get_dtc/get_pending_dtc': zobrazeni diagnostickych kodu     */
/*  - 'get_vin/get_ecu_name/get_cal_id': info o vozidle            */
/*  - 'get_freeze_frame': zmrazeny snimek dat pri zavade            */
/*  - 'get_monitor_status': stav emisnich monitoru a MIL           */
/*  Pole r.free_heap se zobrazuje v hlavicce jako volna pamet ESP32.*/
/* ================================================================ */
function handleResponse(r) {
  if (r.free_heap) {
    document.getElementById('heapTxt').textContent =
      'Heap: ' + (r.free_heap/1024).toFixed(0) + 'KB';
  }

  switch (r.cmd) {
  case 'init':
    if (r.status === 'ok') {
      obdReady = true;
      supportedPids = r.supported_pids || r.pids || [];
      
      /* Automaticke zarazeni vsech podporovanych datovych PIDu do streamu */
      streamPids = [];
      supportedPids.forEach(function(p) {
        /* Preskoceni prilis pomalych nebo ciste bit-encoded PIDu */
        var bit_encoded = [1, 2, 3, 18, 19, 28, 29, 30, 47, 65, 81];
        if (p % 32 !== 0 && bit_encoded.indexOf(p) === -1) {
          streamPids.push(p);
        }
      });

      enableButtons(true);
      buildPidChecks();
      syslog('OBD init OK \u2014 ' + (r.pid_count||supportedPids.length) + ' PIDs found');
    } else {
      syslog('Init error: ' + (r.error||'') + ' ' + (r.message||''));
    }
    break;

  case 'stream':
    updateGauges(r.d);
    updateDashBubbles(r.d);
    updateStats(r.d);
    recordSample(r.d, r.ts);
    break;

  case 'start_stream':
    if (r.status === 'ok') {
      streaming = true; updateStreamBtn();
      document.getElementById('btnRec').disabled = false;
      syslog('Stream started: ' + r.pid_count + ' PIDs, ' + r.interval_ms + 'ms');
    } else { syslog('Stream error: ' + (r.error||'')); }
    break;

  case 'stop_stream':
    streaming = false; updateStreamBtn();
    if (recording) toggleRec(); /* automaticke zastaveni nahravani */
    document.getElementById('btnRec').disabled = true;
    syslog('Stream stopped');
    break;

  case 'get_pid':
    if (r.status === 'ok') showPidResult(r);
    else syslog('PID ' + r.pid + ' error: ' + r.error);
    break;

  case 'get_pids':
    if (r.results) r.results.forEach(function(p) {
      if (p.status==='ok') showPidResult(p);
    });
    break;

  case 'get_supported_pids':
    if (r.status === 'ok') {
      supportedPids = r.pids || [];
      buildPidChecks();
      showSupportedPids(supportedPids);
      syslog('Supported PIDs: ' + supportedPids.length);
    }
    break;

  case 'get_dtc':
  case 'get_pending_dtc':
    showDtc(r);
    break;

  case 'get_vin':
    showInfo('VIN', r.status === 'ok' ? r.vin : ('Error: ' + fmtErr(r)));
    break;

  case 'get_ecu_name':
    showInfo('ECU Name', r.status === 'ok' ? r.ecu_name : ('Error: ' + fmtErr(r)));
    break;

  case 'get_cal_id':
    if (r.status === 'ok') {
      showInfo('Calibration ID', (r.cal_ids||[]).join(', ') + ' (' + r.count + ' item(s))');
    } else { showInfo('Calibration ID', 'Error: ' + fmtErr(r)); }
    break;

  case 'get_freeze_frame':
    if (r.status === 'ok') {
      var ff = 'PID 0x' + r.pid.toString(16).toUpperCase() + ': ';
      ff += r.value !== undefined ? r.value.toFixed(2) : ('raw=' + (r.raw||'?'));
      if (r.name) ff += ' (' + r.name + ' ' + (r.unit||'') + ')';
      showInfo('Freeze Frame', ff);
    } else { showInfo('Freeze Frame', 'Error: ' + fmtErr(r)); }
    break;

  case 'get_monitor_status':
    if (r.status === 'ok') showMonitorStatus(r);
    else syslog('Monitor status error: ' + (r.error||''));
    break;

  case 'ping':
    break;
  }
}

/* Formatovani chybove zpravy s volitelnym detailem NRC (Negative Response Code).
   Obsahuje cislo NRC v hexadecimalnim formatu a jeho nazev, pokud jsou k dispozici. */
function fmtErr(r) {
  var s = r.error || 'unknown';
  if (r.nrc_name) s += ' (NRC 0x' + (r.nrc_code||0).toString(16).toUpperCase() + ': ' + r.nrc_name + ')';
  if (r.message) s += ' \u2014 ' + r.message;
  return s;
}

/* ================================================================ */
/*  SVG ukazatele (gauges) — RPM a rychlost                        */
/*  Kazdy ukazatel pouziva SVG polokruhovy oblouk (path element).  */
/*  Animace se provadi zmenou atributu stroke-dashoffset:           */
/*    plny oblouk ma delku 251 (stroke-dasharray), offset 251 =    */
/*    prazdny, offset 0 = plne vyplneny. Hodnota se linearne       */
/*    interpoluje mezi min a max.                                   */
/*  Barva oblouku se meni pri vysokych hodnotach:                   */
/*    azurova (normalni) -> oranzova (>80% varovani) -> cervena     */
/*    (prekroceni varovne hranice warnHigh).                        */
/* ================================================================ */
function setGauge(arcId, valId, value, min, max, warnHigh) {
  var pct = Math.max(0, Math.min(1, (value - min) / (max - min)));
  var arc = document.getElementById(arcId);
  arc.style.strokeDashoffset = 251 * (1 - pct);
  if (warnHigh && value > warnHigh) arc.style.stroke = '#f44336';
  else if (warnHigh && value > warnHigh * 0.8) arc.style.stroke = '#ff9800';
  else arc.style.stroke = '#26a69a';
  document.getElementById(valId).textContent = Math.round(value);
}

/* Aktualizace obou ukazatelu z prijatych stream dat.
   PID 12 = otacky motoru (rozsah 0-7000, varovani na 6000).
   PID 13 = rychlost vozidla (rozsah 0-240, bez varovani). */
function updateGauges(d) {
  if (d['12'] !== undefined) setGauge('arc_rpm','val_rpm', d['12'], 0, 7000, 6000);
  if (d['13'] !== undefined) setGauge('arc_spd','val_spd', d['13'], 0, 240, null);
}

/* ================================================================ */
/*  Zalozka Dash — dynamicke PID bubliny                            */
/*  Pro kazdy PID prijaty ve stream datech se vytvori bublina       */
/*  (div.pid-bubble) obsahujici:                                    */
/*    - Aktualni ciselnou hodnotu s jednotkou                       */
/*    - Nazev PID z tabulky PID_INFO                                */
/*    - Canvas se sparkline minigrafem posledních 60 vzorku         */
/*  Bubliny se vytvareji dynamicky pri prvnim prijeti daneho PID    */
/*  a nasledne se jen aktualizuji. Puvodni zastupny text            */
/*  ("Press Init OBD...") se odstrani pri prvnich datech.           */
/* ================================================================ */
function updateDashBubbles(d) {
  var grid = document.getElementById('dashGrid');
  /* Odebrani zastupneho textu pri prvnich prijatrch datech */
  if (grid.dataset.init !== '1') { grid.innerHTML = ''; grid.dataset.init = '1'; }

  for (var pid in d) {
    var numVal = d[pid];
    if (typeof numVal !== 'number') continue;
    pushHistory(pid, numVal);

    var info = PID_INFO[pid] || {n:'PID 0x'+parseInt(pid).toString(16).toUpperCase(), u:''};
    var val = numVal.toFixed(1);
    var id = 'pb_' + pid;
    var el = document.getElementById(id);

    if (!el) {
      /* Vytvoreni nove bubliny pro PID, ktery jeste nema DOM element */
      el = document.createElement('div');
      el.className = 'pid-bubble'; el.id = id;
      el.innerHTML = '<div class="pb-val">' + val + ' <span class="pb-unit">' + info.u + '</span></div>' +
                     '<div class="pb-name">' + info.n + '</div>' +
                     '<canvas id="sp_' + pid + '" width="180" height="32"></canvas>';
      grid.appendChild(el);
    } else {
      /* Aktualizace hodnoty v existujici bubline */
      el.querySelector('.pb-val').innerHTML = val + ' <span class="pb-unit">' + info.u + '</span>';
    }

    /* Vykresleni sparkline minigrafu z historie hodnot */
    var canvas = document.getElementById('sp_' + pid);
    if (pidHistory[pid]) drawSparkline(canvas, pidHistory[pid]);
  }
}

/* Zobrazeni vysledku cteni jednoho PID v systemovem logu */
function showPidResult(r) {
  var v = r.value !== undefined ? r.value.toFixed(1) : (r.value_raw || '?');
  syslog('PID ' + r.pid + ' (' + (r.name||'') + '): ' + v + ' ' + (r.unit||''));
}

/* ================================================================ */
/*  Diagnostika — zobrazeni DTC kodu, informaci o vozidle           */
/*  a stavu emisnich monitoru                                       */
/* ================================================================ */

/* Zobrazeni diagnostickych poruchovych kodu (DTC).
   Rozlisuje stavy: chyba cteni, zadne kody (vse v poradku),
   nebo seznam nalezenych kodu s poctem. */
function showDtc(r) {
  var p = document.getElementById('dtcPanel');
  var c = document.getElementById('dtcContent');
  p.style.display = 'block';
  if (r.status !== 'ok') {
    c.className = 'dtc-list'; c.textContent = 'Error: ' + (r.error||'');
    return;
  }
  if (!r.count || r.count === 0) {
    c.className = 'dtc-list empty';
    c.textContent = (r.cmd==='get_pending_dtc' ? 'No pending DTCs' : 'No DTCs') + ' \u2014 all clear';
    return;
  }
  c.className = 'dtc-list';
  c.innerHTML = '<strong>' + r.count + ' fault(s):</strong> ' + (r.dtcs||[]).join(', ');
}

/* Zobrazeni obecne informace o vozidle v informacnim panelu */
function showInfo(title, text) {
  document.getElementById('infoPanel').style.display = 'block';
  document.getElementById('infoContent').innerHTML = '<strong>' + title + ':</strong> ' + text;
}

/* Zobrazeni stavu emisnich monitoru v tabulkovem formatu.
   Zahrnuje stav MIL kontrolky (zapnuta/vypnuta), pocet ulozenych
   DTC a tabulku vsech monitoru s informaci, zda jsou podporovany
   a zda jsou ve stavu Ready (pripraven) nebo Not Ready. */
function showMonitorStatus(r) {
  var html = '<div style="display:flex;gap:16px;align-items:center;margin-bottom:8px">' +
    '<div><strong>MIL (Check Engine):</strong> ' +
    (r.mil ? '<span style="color:#f44336;font-weight:700">ON</span>'
           : '<span style="color:#00e676;font-weight:700">OFF</span>') + '</div>' +
    '<div><strong>Stored DTCs:</strong> ' + r.dtc_count + '</div></div>';

  if (r.monitors) {
    html += '<table style="width:100%;font-size:0.82em;border-collapse:collapse">';
    html += '<tr style="color:var(--fg2);text-align:left"><th style="padding:3px 6px">Monitor</th>' +
            '<th style="padding:3px 6px">Supported</th><th style="padding:3px 6px">Status</th></tr>';
    for (var name in r.monitors) {
      var m = r.monitors[name];
      var label = name.replace(/_/g, ' ');
      label = label.charAt(0).toUpperCase() + label.slice(1);
      var sup = m.sup ? '<span style="color:#00e676">Yes</span>' : '<span style="color:#555">No</span>';
      var rdy = !m.sup ? '<span style="color:#555">\u2014</span>'
                : (m.rdy ? '<span style="color:#00e676">Ready</span>'
                         : '<span style="color:#ff9800">Not Ready</span>');
      html += '<tr style="border-top:1px solid #222"><td style="padding:3px 6px">' + label + '</td>' +
              '<td style="padding:3px 6px">' + sup + '</td>' +
              '<td style="padding:3px 6px">' + rdy + '</td></tr>';
    }
    html += '</table>';
  }
  document.getElementById('infoPanel').style.display = 'block';
  document.getElementById('infoContent').innerHTML = html;
}

/* Zobrazeni seznamu podporovanych PID jako barevne stitky s hexadecimalnim
   cislem a nazvem. PID ktere jsou nasobky 32 se preskakovuji,
   protoze slouzi pouze jako bitmaskove indikatory podpory dalsich PID. */
function showSupportedPids(pids) {
  var html = '<strong>Supported PIDs (' + pids.length + '):</strong><div style="display:flex;flex-wrap:wrap;gap:4px;margin-top:6px">';
  pids.forEach(function(pid) {
    if (pid % 32 === 0) return; /* preskocit bitmaskove PID (0x00, 0x20, 0x40...) */
    var info = PID_INFO[pid] || {n:'PID 0x'+pid.toString(16).toUpperCase()};
    html += '<span style="background:var(--bg3);padding:3px 8px;border-radius:4px;font-size:0.78em">' +
            '<span style="color:var(--accent)">0x' + pid.toString(16).toUpperCase().padStart(2,'0') + '</span> ' +
            info.n + '</span>';
  });
  html += '</div>';
  document.getElementById('infoPanel').style.display = 'block';
  document.getElementById('infoContent').innerHTML = html;
}

/* ================================================================ */
/*  Ovladani datoveho proudu (stream)                               */
/*  Umoznuje spustit a zastavit kontinualni cteni vybranych PID     */
/*  se zadanym intervalem. PID se vybiraji pomoci toglovatelnych    */
/*  stitku v konfiguracnim panelu.                                  */
/* ================================================================ */

/* Prepinac streamu — odesle prikaz start_stream nebo stop_stream.
   Pri startu sebere vybrane PID z checkboxu a interval z inputu. */
function toggleStream() {
  if (streaming) {
    cmd({cmd: 'stop_stream'});
  } else {
    var checks = document.querySelectorAll('.pid-chk.sel');
    var pids = [];
    checks.forEach(function(el) { pids.push(parseInt(el.dataset.pid)); });
    if (pids.length === 0) { syslog('Select at least one PID!'); return; }
    var interval = parseInt(document.getElementById('streamInt').value) || 200;
    cmd({cmd: 'start_stream', pids: pids, interval_ms: interval});
  }
}

/* Aktualizace textu a stylu tlacitka streamu podle aktualniho stavu */
function updateStreamBtn() {
  var btn = document.getElementById('btnStream');
  if (streaming) {
    btn.textContent = 'Stop Stream'; btn.classList.add('on');
  } else {
    btn.textContent = 'Start Stream'; btn.classList.remove('on');
  }
}

/* Odemknuti/zamknuti diagnostickych a streamovacich tlacitek.
   Vola se po uspesne inicializaci OBD (init OK). */
function enableButtons(en) {
  ['btnStream','btnSup','btnDtc','btnPdtc','btnVin','btnEcu','btnCal','btnMon'].forEach(function(id) {
    document.getElementById(id).disabled = !en;
  });
  document.getElementById('streamPanel').style.display = en ? 'block' : 'none';
  if (en) document.getElementById('btnInit').textContent = 'Re-Init';
}

/* Sestaveni toglovatelnych PID stitku v konfiguracnim panelu streamu.
   Pro kazdy podporovany PID (krome bitmaskovych) se vytvori element,
   ktery lze kliknutim vybrat nebo zrusit pro zarazeni do streamu. */
function buildPidChecks() {
  var wrap = document.getElementById('pidChecks');
  wrap.innerHTML = '';
  supportedPids.forEach(function(pid) {
    if (pid % 32 === 0) return; /* preskocit bitmaskove PID */
    var info = PID_INFO[pid] || {n:'0x'+pid.toString(16).toUpperCase()};
    var el = document.createElement('span');
    el.className = 'pid-chk' + (streamPids.indexOf(pid) >= 0 ? ' sel' : '');
    el.dataset.pid = pid;
    el.textContent = info.n || ('PID ' + pid);
    el.onclick = function() {
      this.classList.toggle('sel');
      var idx = streamPids.indexOf(pid);
      if (idx >= 0) streamPids.splice(idx, 1);
      else streamPids.push(pid);
    };
    wrap.appendChild(el);
  });
}

/* ================================================================ */
/*  Log (komunikacni log s fixni vyskou)                            */
/*  Zobrazuje vsechny zpravy (prichozi, odchozi, systemove)        */
/*  s casovym razitkem. Stream data se filtruje (prilis caste).     */
/*  Buffer je omezen na 300 radku — nejstarsi se automaticky mazi.  */
/* ================================================================ */

/* Zapise systemovou zpravu do logu */
function syslog(t) { logMsg('sys', t); }

/* Zapise zpravu do komunikacniho logu s casovym razitkem a typem.
   Typy: 'in' = prichozi (zelena), 'out' = odchozi (oranzova),
   'sys' = systemova (seda). Zpravy obsahujici "stream" se v
   prichozim smeru preskakovuji kvuli nadmernemu poctu. */
function logMsg(type, t) {
  var box = document.getElementById('logBox');
  /* Vynechani stream dat v logu — prilis velky objem zprav */
  if (type === 'in' && t.indexOf('"stream"') > -1) return;
  /* Omezeni na 300 radku — nejstarsi radek se odstrani */
  if (box.childElementCount > 300) box.removeChild(box.firstChild);
  var d = document.createElement('div');
  d.className = 'm-' + type;
  var ts = new Date().toLocaleTimeString('en-GB',{hour:'2-digit',minute:'2-digit',second:'2-digit'});
  d.textContent = '[' + ts + '] ' + (type==='in'?'<< ':type==='out'?'>> ':'') + t;
  box.appendChild(d);
  box.scrollTop = box.scrollHeight;
}

/* ================================================================ */
/*  Spusteni aplikace                                               */
/*  Pri nacteni stranky se nejprve obnovi nastaveni z localStorage  */
/*  a pote se navaze WebSocket pripojeni k ESP32 serveru.           */
/* ================================================================ */
loadSettings();
connect();
</script>
</body>
</html>
)rawliteral";

#endif /* DASHBOARD_H */
