/*
 * ece4781_sweat_sensor.ino
 * ----------------------------------------------------------------------------
 * ECE-4781 Team 21 - Dual-Sensor Skin Conductance Front-End
 * Target : ESP32-C6-DevKitC-1, Arduino-ESP32 core 3.x (Boards Manager "esp32")
 * Libs   : none beyond the core (Wire, WiFi, WebServer, ESPmDNS, Preferences)
 *
 * Signal chain (two-electrode, concentric probes after Hamed 2012 [ref 3]):
 *   GPIO4 LEDC PWM (150 kHz carrier, 2 kHz sine LUT)
 *     -> Sallen-Key LPF (R3=R4=10k, C1=C2=10nF, fc=1.59 kHz, Q=0.5)
 *     -> C3 1uF AC coupling, R5 100k bias to V_MID (no DC through skin)
 *     -> R6 100k safety resistor -> CD74HC4052 1Y -> probe outer ring
 *     -> skin -> probe inner disc -> CD74HC4052 2Y -> TIA_IN
 *   TIA (R7 100k, +IN = V_MID 1.65 V) -> precision peak detector
 *     (D1, C4 1uF || R8 100k to V_MID, tau 100 ms) -> DC_OUT -> ADS1115 A0
 *
 *   Envelope above V_MID:  E = R7 * Vexc / (Rs + Rx)  (Rs ~= R6)
 *   =>  Rx = k / E - Rs,  k = R7 * Vexc  (mV*Ohm),  G = 1/Rx
 *   k and Rs start at nominal circuit values and can be replaced by a
 *   two-point calibration with the 100k / 470k test resistors.
 *
 * Safety: peak skin current is set by hardware (Vexc / R6). With the values
 * below it is ~6 uA peak even with the electrodes shorted. The firmware
 * refuses to start the excitation if its own model predicts >= 10 uA.
 * Only connect to skin when running from the 3-AA pack with USB unplugged.
 *
 * Serial (115200): CSV rows only; every diagnostic line starts with '#'.
 *   Time_ms,Cov_mV,Ref_mV,Cov_uS,Ref_uS,Diff_uS,TempCov_C,TempRef_C,Press_mV
 *   Serial commands: help, status, header, tare, cal, calreset, clear
 *
 * Web: http://sensors.local/ (or the IP printed at boot)
 *   /            live dashboard (self-contained, no CDN)
 *   /api/data    JSON, ?since=<seq> returns newer rows
 *   /export.csv  full RAM buffer (up to 1800 rows, ~15 min) as a download
 * ----------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <esp_random.h>
#include <math.h>

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || (ESP_ARDUINO_VERSION_MAJOR < 3)
#error "This sketch needs Arduino-ESP32 core 3.x (ledcAttach / timerBegin(freq) API). Update the esp32 package in Boards Manager."
#endif

// ============================================================================
// USER CONFIGURATION
// ============================================================================

// Wi-Fi network to join. If it cannot be joined within WIFI_CONNECT_TIMEOUT_MS
// (or the SSID is left as "YOUR_WIFI_SSID"), the board starts its own access
// point instead, so the dashboard works untethered on a phone or laptop.
const char *WIFI_SSID     = "YOUR_WIFI_SSID";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char *MDNS_HOSTNAME        = "sensors";          // http://sensors.local
const char *FALLBACK_AP_SSID     = "ECE4781-Sensors";
const char *FALLBACK_AP_PASSWORD = "team21sweat";      // >= 8 characters
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;

// ---- Pins ------------------------------------------------------------------
const int PIN_I2C_SDA  = 21;
const int PIN_I2C_SCL  = 22;
const int PIN_DAC_SINE = 4;    // -> R3, Sallen-Key input
#if defined(CONFIG_IDF_TARGET_ESP32C6)
// GPIO 27 does not exist on the ESP32-C6-DevKitC-1 headers (GPIO 24-30 are the
// module's internal SPI-flash lines; driving one can crash the chip).
// Wire CD74HC4052 pin 10 (S0) to GPIO 23 instead (J3 header, next to GPIO 22).
const int PIN_MUX_SEL  = 23;
#else
const int PIN_MUX_SEL  = 27;   // classic ESP32-DevKitC
#endif

// ---- I2C addresses ---------------------------------------------------------
// The ADS1115 and TMP117 #1 cannot both use 0x48 (ADDR->GND) on one bus.
// Move the ADS1115 ADDR pin from GND to SCL -> 0x4B. TMP117s stay as wired.
constexpr uint8_t ADS1115_ADDR    = 0x4B;  // ADDR -> SCL
constexpr uint8_t TMP117_COV_ADDR = 0x48;  // ADDR -> GND   (covered site)
constexpr uint8_t TMP117_REF_ADDR = 0x49;  // ADDR -> +3.3V (reference site)

// ---- Analog front-end constants --------------------------------------------
const float VCC_MV          = 3300.0f;
const float V_MID_MV        = 1650.0f;    // R1/R2 divider buffered by U2A
const float R6_SAFETY_OHMS  = 100000.0f;
const float R7_TIA_OHMS     = 100000.0f;
const float R9_FSR_OHMS     = 10000.0f;
const float SK_FC_HZ        = 1591.55f;   // 1 / (2*pi*10k*10nF)
const float SK_Q            = 0.5f;       // equal-R, equal-C unity-gain Sallen-Key
const float ENVELOPE_TAU_MS = 100.0f;     // C4 * R8

// Set true only if you add a jumper from the V_MID row to ADS1115 A2; the live
// V_MID is then measured every cycle instead of assuming 1650 mV.
const bool MEASURE_VMID_ON_A2 = false;

// Known test resistors used by the two-point calibration
const float CAL_R_COV_OHMS = 100000.0f;   // on channel 0 (1Y0/2Y0)
const float CAL_R_REF_OHMS = 470000.0f;   // on channel 1 (1Y1/2Y1)

// ---- Excitation generator --------------------------------------------------
const uint32_t EXC_FREQ_HZ      = 2000;
const uint32_t PWM_CARRIER_HZ   = 150000;
const uint8_t  PWM_RES_BITS     = 8;           // 150 kHz * 256 = 38.4 MHz LEDC clock
const uint32_t PWM_MID_COUNTS   = 1u << (PWM_RES_BITS - 1);
const uint32_t SINE_AMPL_COUNTS = 120;         // duty swings 8..248 of 256
constexpr int  SINE_POINTS      = 16;          // LUT updates at 32 kHz
const uint32_t SINE_TIMER_HZ    = 4000000;     // timer tick; alarm every 125 ticks
const float    SKIN_CURRENT_LIMIT_UA = 10.0f;

// ---- Sampling --------------------------------------------------------------
const uint32_t MUX_SETTLE_MS     = 250;   // envelope detector settling after a switch
const uint8_t  ADS_DCOUT_AVG     = 4;     // 4 x 860 SPS conversions per channel (~6 ms)
const uint8_t  ADS_AUX_AVG       = 2;
const float    OPEN_CIRCUIT_ENV_MV = 2.0f; // envelope below this = no contact (0 uS)
const float    MIN_RX_OHMS       = 1000.0f;
const float    SATURATION_MV     = VCC_MV - 60.0f;
const uint32_t DEVICE_RETRY_MS   = 5000;

constexpr uint32_t BUFFER_CAPACITY = 1800;  // ~15 min of rows
const uint32_t MAX_ROWS_PER_RESPONSE = 300;
const int CAL_AVG_ROWS = 10;
const int CAL_MIN_ROWS = 5;

// ---- Compile-time sanity checks --------------------------------------------
static_assert(ADS1115_ADDR != TMP117_COV_ADDR && ADS1115_ADDR != TMP117_REF_ADDR &&
              TMP117_COV_ADDR != TMP117_REF_ADDR,
              "I2C address conflict: every device on the shared bus needs its own address");
static_assert((SINE_POINTS & (SINE_POINTS - 1)) == 0, "SINE_POINTS must be a power of two");
#if defined(CONFIG_IDF_TARGET_ESP32C6)
static_assert(PIN_MUX_SEL >= 0 && PIN_MUX_SEL <= 23 && PIN_MUX_SEL != 12 && PIN_MUX_SEL != 13,
              "ESP32-C6: MUX_SEL must be a header GPIO 0-23 that is not USB D-/D+ (12/13)");
#endif

// ============================================================================
// Device register maps
// ============================================================================
const uint8_t  ADS_REG_CONV     = 0x00;
const uint8_t  ADS_REG_CONFIG   = 0x01;
const uint16_t ADS_OS_SINGLE    = 0x8000;
const uint16_t ADS_MUX_AIN0     = 0x4000;
const uint16_t ADS_MUX_AIN1     = 0x5000;
const uint16_t ADS_MUX_AIN2     = 0x6000;
const uint16_t ADS_PGA_4_096    = 0x0200;   // GAIN_ONE, +/-4.096 V
const uint16_t ADS_MODE_SINGLE  = 0x0100;
const uint16_t ADS_DR_860SPS    = 0x00E0;
const uint16_t ADS_COMP_DISABLE = 0x0003;
const float    ADS_LSB_MV       = 0.125f;   // 4.096 V / 32768

const uint8_t  TMP_REG_TEMP      = 0x00;
const uint8_t  TMP_REG_CONFIG    = 0x01;
const uint8_t  TMP_REG_DEVICE_ID = 0x0F;
const uint16_t TMP_DEVICE_ID     = 0x0117;
const uint16_t TMP_CONFIG_250MS  = 0x0120;  // continuous, CONV=010, AVG=8 -> 250 ms cycle
const float    TMP117_LSB_C      = 0.0078125f;

// ============================================================================
// Types and global state
// ============================================================================
enum MuxChannel : uint8_t { CH_COVERED = 0, CH_REFERENCE = 1 };

const uint8_t FLAG_COV_SETTLE = 0x01;  // reading may be the previous channel's decay tail
const uint8_t FLAG_REF_SETTLE = 0x02;
const uint8_t FLAG_COV_SAT    = 0x04;  // TIA / detector at the rail
const uint8_t FLAG_REF_SAT    = 0x08;

struct Sample {
  uint32_t t_ms;
  float cov_mV, ref_mV, cov_uS, ref_uS, diff_uS, tempCov_C, tempRef_C, press_mV;
  uint8_t flags;
};

static Sample   ring[BUFFER_CAPACITY];
static uint32_t totalRows   = 0;   // sequence number of the next row
static uint32_t firstSeq    = 0;   // first sequence number after the last clear
static uint32_t bufferEpoch = 0;
static uint32_t bootId      = 0;

WebServer   server(80);
Preferences prefs;

bool adsOk = false, tmpCovOk = false, tmpRefOk = false, excitationOk = false;
bool wifiIsAP = false;

float vmid_mV = V_MID_MV;
float excPeak_mV = 0.0f, skinPeak_uA = 0.0f, kNominal = 0.0f;
float calK = 0.0f, calRs = R6_SAFETY_OHMS;
bool  calUser = false;
float fsrTare_mV = 0.0f;

MuxChannel activeChannel = CH_COVERED;
uint32_t channelSwitchMs = 0;
float lastEnvCov = NAN, lastEnvRef = NAN;
Sample pending;
uint32_t lastRowMs = 0;
float rowPeriodMs = 2.0f * (MUX_SETTLE_MS + 6);
uint32_t lastRetryMs = 0;
String serialLine;

static uint32_t sineLut[SINE_POINTS];
static volatile uint32_t sineIdx = 0;
hw_timer_t *sineTimer = nullptr;

static const char CSV_HEADER[] =
    "Time_ms,Cov_mV,Ref_mV,Cov_uS,Ref_uS,Diff_uS,TempCov_C,TempRef_C,Press_mV";

// ============================================================================
// Embedded dashboard
// ============================================================================
static const char DASHBOARD_HTML[] = R"rawliteral(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Sweat Sensor Dashboard</title>
<style>
:root{--bg:#f5f6f8;--card:#fff;--ink:#1b1f24;--muted:#5c6670;--line:#e1e4e8;--grid:#eceff2;
--cov:#1f6fd1;--ref:#d06a12;--diff:#7a4cc2;--press:#0f8a7a;--bad:#c62828;--ok:#2e7d32;--warn:#a85f00}
@media (prefers-color-scheme:dark){:root{--bg:#101317;--card:#191d22;--ink:#e8eaed;--muted:#9aa4ae;
--line:#2a3038;--grid:#232930;--cov:#5aa2ff;--ref:#ff9d4d;--diff:#b18cff;--press:#3cc7b3;--bad:#ff6b6b;--ok:#6bd47a;--warn:#ffb74d}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{padding:14px 16px;border-bottom:1px solid var(--line);display:flex;flex-wrap:wrap;gap:8px 16px;align-items:center;justify-content:space-between}
h1{font-size:17px;margin:0}
.sub{color:var(--muted);font-size:12px}
main{padding:16px;max-width:1240px;margin:0 auto}
.pills{display:flex;flex-wrap:wrap;gap:6px}
.pill{font-size:12px;padding:3px 9px;border-radius:999px;border:1px solid var(--line);background:var(--card)}
.pill.ok{color:var(--ok)}.pill.bad{color:var(--bad);border-color:var(--bad);font-weight:600}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(180px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;min-width:0}
.label{color:var(--muted);font-size:12px;font-weight:600;letter-spacing:.02em}
.big{font-size:25px;font-weight:600;font-variant-numeric:tabular-nums;margin-top:2px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.small{color:var(--muted);font-size:13px;font-variant-numeric:tabular-nums}
.dc{color:var(--bad)!important;font-size:18px}
.k-cov{border-top:3px solid var(--cov)}.k-ref{border-top:3px solid var(--ref)}.k-diff{border-top:3px solid var(--diff)}.k-press{border-top:3px solid var(--press)}
.charts{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(min(100%,480px),1fr));margin-top:12px}
canvas{width:100%;height:220px;display:block}
.legend{display:flex;gap:14px;font-size:12px;color:var(--muted);margin:2px 0 6px}
.sw{display:inline-block;width:12px;height:3px;border-radius:2px;vertical-align:middle;margin-right:5px}
.bar{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-top:12px}
.btn{appearance:none;border:1px solid var(--line);background:var(--card);color:var(--ink);padding:8px 14px;border-radius:8px;font:inherit;cursor:pointer;text-decoration:none}
.btn.primary{background:var(--cov);border-color:var(--cov);color:var(--bg);font-weight:600}
.btn:hover{filter:brightness(1.07)}
select{font:inherit;padding:7px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink)}
#msg{font-size:13px;color:var(--muted)}
.warn{color:var(--warn);font-size:13px;min-height:18px;margin-top:8px}
details{margin-top:14px}summary{cursor:pointer;color:var(--muted)}
footer{color:var(--muted);font-size:12px;padding:4px 16px 28px;max-width:1240px;margin:0 auto}
</style></head><body>
<header><div><h1>ECE-4781 Dual-Site Skin Conductance</h1><div class="sub" id="sub">Connecting&hellip;</div></div>
<div class="pills" id="pills"></div></header>
<main>
<div class="grid">
 <div class="card k-cov"><div class="label">Covered conductance</div><div class="big" id="covuS">&mdash;</div><div class="small" id="covmV">&mdash;</div></div>
 <div class="card k-ref"><div class="label">Reference conductance</div><div class="big" id="refuS">&mdash;</div><div class="small" id="refmV">&mdash;</div></div>
 <div class="card k-diff"><div class="label">Differential (Cov &minus; Ref)</div><div class="big" id="diffuS">&mdash;</div><div class="small">sweat signal under liner</div></div>
 <div class="card k-cov"><div class="label">Covered skin temp</div><div class="big" id="tcov">&mdash;</div><div class="small" id="tdiff">&mdash;</div></div>
 <div class="card k-ref"><div class="label">Reference skin temp</div><div class="big" id="tref">&mdash;</div><div class="small">TMP117 #2</div></div>
 <div class="card k-press"><div class="label">Contact pressure (FSR)</div><div class="big" id="ppct">&mdash;</div><div class="small" id="pmV">&mdash;</div></div>
</div>
<div class="warn" id="warn"></div>
<div class="bar">
 <a class="btn primary" href="/export.csv" download>Download CSV</a>
 <button class="btn" id="tare">Tare pressure</button>
 <button class="btn" id="clear">Clear buffer</button>
 <label class="small">Window <select id="win"><option value="60000">1 min</option><option value="300000" selected>5 min</option><option value="900000">15 min</option></select></label>
 <span id="msg"></span>
</div>
<div class="charts">
 <div class="card"><div class="label">Skin conductance (&micro;S)</div><div class="legend"><span><i class="sw" style="background:var(--cov)"></i>Covered</span><span><i class="sw" style="background:var(--ref)"></i>Reference</span></div><canvas id="cG"></canvas></div>
 <div class="card"><div class="label">Differential conductance (&micro;S)</div><div class="legend"><span><i class="sw" style="background:var(--diff)"></i>Covered &minus; Reference</span></div><canvas id="cD"></canvas></div>
 <div class="card"><div class="label">Skin temperature (&deg;C)</div><div class="legend"><span><i class="sw" style="background:var(--cov)"></i>Covered</span><span><i class="sw" style="background:var(--ref)"></i>Reference</span></div><canvas id="cT"></canvas></div>
 <div class="card"><div class="label">Contact pressure (% of divider span, tared)</div><div class="legend"><span><i class="sw" style="background:var(--press)"></i>FSR</span></div><canvas id="cP"></canvas></div>
</div>
<details><summary>Calibration &amp; front-end details</summary><div class="card" style="margin-top:8px">
 <div class="small" id="calinfo">&mdash;</div>
 <p class="small">Two-point calibration: put the 100&nbsp;k&Omega; test resistor on the Covered channel (1Y0/2Y0) and the 470&nbsp;k&Omega; resistor on the Reference channel (1Y1/2Y1), wait about 10&nbsp;s, then press Calibrate. The device averages the last 10 rows, solves the excitation gain and series resistance, and stores them in flash.</p>
 <div class="bar"><button class="btn" id="cal">Calibrate (100 k / 470 k)</button><button class="btn" id="calr">Reset to nominal</button></div>
</div></details>
</main>
<footer id="foot"></footer>
<script>
const $=id=>document.getElementById(id);
const S={since:-1,boot:null,epoch:null,rows:[],st:null,win:300000,online:null};
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const ok=v=>v!==null&&v!==undefined&&isFinite(v);
const f=(v,d)=>ok(v)?Number(v).toFixed(d):'—';
function pct(mV){const st=S.st;if(!ok(mV)||!st)return null;const t=st.tare,span=st.vcc-t;return span>0?(mV-t)/span*100:null;}
async function post(url,ask){
 if(ask&&!confirm(ask))return;
 $('msg').textContent='Working…';
 try{const r=await fetch(url,{method:'POST'});const j=await r.json();$('msg').textContent=j.msg;}
 catch(e){$('msg').textContent='Request failed: '+e;}
}
$('tare').onclick=()=>post('/api/tare');
$('clear').onclick=()=>post('/api/clear','Clear every buffered reading on the device? Download the CSV first if you need it.');
$('cal').onclick=()=>post('/api/calibrate','The Covered channel must have the 100 kΩ test resistor and the Reference channel the 470 kΩ test resistor, with readings stable for about 10 s. Calibrate now?');
$('calr').onclick=()=>post('/api/cal_reset','Discard the stored calibration and go back to nominal circuit values?');
$('win').onchange=e=>{S.win=+e.target.value;render();};
window.addEventListener('resize',()=>render());

async function poll(){
 try{
  for(let i=0;i<12;i++){
   const r=await fetch('/api/data?since='+S.since,{cache:'no-store'});
   if(!r.ok)throw new Error('HTTP '+r.status);
   const j=await r.json();
   if(j.boot!==S.boot||j.epoch!==S.epoch){
    const fromStart=(S.since===-1);
    S.boot=j.boot;S.epoch=j.epoch;S.rows=[];S.since=-1;
    if(!fromStart)continue;
   }
   S.st=j;
   for(const row of j.rows)S.rows.push(row);
   if(j.rows.length)S.since=j.rows[j.rows.length-1][0];
   if(S.rows.length>j.cap)S.rows.splice(0,S.rows.length-j.cap);
   if(!j.more)break;
  }
  S.online=true;
 }catch(e){S.online=false;}
 render();
 setTimeout(poll,500);
}

function setv(id,txt,bad){const e=$(id);e.textContent=txt;e.classList.toggle('dc',!!bad);}
function pill(name,good,okTxt,badTxt){return '<span class="pill '+(good?'ok':'bad')+'">'+name+': '+(good?okTxt:badTxt)+'</span>';}

function render(){
 const st=S.st,last=S.rows.length?S.rows[S.rows.length-1]:null;
 if(!st){$('sub').textContent=S.online===false?'Device unreachable — retrying…':'Connecting…';return;}
 $('pills').innerHTML=pill('Link',S.online,'live','OFFLINE')+pill('ADS1115',st.ads,'OK','DISCONNECTED')+
  pill('TMP117 cov',st.tmpCov,'OK','DISCONNECTED')+pill('TMP117 ref',st.tmpRef,'OK','DISCONNECTED')+
  pill('Excitation',st.exc,'2 kHz','OFF');
 const mins=(st.count*st.rowMs/60000).toFixed(1);
 $('sub').textContent=(S.online?'':'OFFLINE (showing last data) · ')+st.wifi+' '+st.ip+' · '+st.count+'/'+st.cap+' rows ('+mins+' min) · '+(1000/st.rowMs).toFixed(2)+' Hz';
 const condOk=st.ads&&st.exc, condMsg=st.ads?'EXCITATION OFF':'DISCONNECTED';
 if(!condOk){setv('covuS',condMsg,1);setv('refuS',condMsg,1);setv('diffuS',condMsg,1);setv('covmV','—');setv('refmV','—');}
 else if(last){
  setv('covuS',f(last[4],3)+' µS');setv('covmV','DC_OUT '+f(last[2],1)+' mV');
  setv('refuS',f(last[5],3)+' µS');setv('refmV','DC_OUT '+f(last[3],1)+' mV');
  setv('diffuS',(ok(last[6])&&last[6]>0?'+':'')+f(last[6],3)+' µS');
 }
 if(!st.tmpCov)setv('tcov','DISCONNECTED',1);else if(last)setv('tcov',f(last[7],2)+' °C');
 if(!st.tmpRef)setv('tref','DISCONNECTED',1);else if(last)setv('tref',f(last[8],2)+' °C');
 setv('tdiff',last&&ok(last[7])&&ok(last[8])?'Cov − Ref '+f(last[7]-last[8],2)+' °C':'TMP117 #1');
 if(!st.ads){setv('ppct','DISCONNECTED',1);setv('pmV','—');}
 else if(last){
  const p=pct(last[9]);setv('ppct',f(p,1)+' %');
  const r=ok(last[9])&&last[9]>1?(st.r9*(st.vcc-last[9])/last[9]/1000):null;
  setv('pmV',f(last[9],1)+' mV · FSR '+(r===null?'open':f(r,1)+' kΩ')+' · tare '+f(st.tare,1)+' mV');
 }
 const w=[];
 if(last&&condOk){
  if(last[10]&1)w.push('Covered reading may be settling-limited (Reference envelope was >12× larger).');
  if(last[10]&2)w.push('Reference reading may be settling-limited (Covered envelope was >12× larger).');
  if(last[10]&4)w.push('Covered channel saturated at the rail.');
  if(last[10]&8)w.push('Reference channel saturated at the rail.');
 }
 if(!st.calUser)w.push('Using nominal circuit constants — run the 100 k / 470 k calibration for accurate µS.');
 $('warn').textContent=w.join('  ');
 $('calinfo').textContent='k = '+st.calK.toExponential(4)+' mV·Ω (R7 × excitation), series R = '+(st.calRs/1000).toFixed(2)+' kΩ — '+(st.calUser?'user calibration (stored in flash)':'nominal values (not calibrated)')+'. Nominal k = '+st.kNom.toExponential(4)+'.';
 $('foot').textContent='Excitation '+f(st.excPk,0)+' mV peak at R6 → max skin current '+f(st.iPk,2)+' µA (limit 10 µA) · V_MID '+f(st.vmid,1)+' mV · mux settle '+st.settle+' ms · uptime '+(st.uptime/60000).toFixed(1)+' min'+(st.wifi==='STA'?' · RSSI '+st.rssi+' dBm':'');
 chart('cG',[{get:r=>r[4],c:'--cov'},{get:r=>r[5],c:'--ref'}],{minSpan:0.1});
 chart('cD',[{get:r=>r[6],c:'--diff'}],{zero:true,minSpan:0.1});
 chart('cT',[{get:r=>r[7],c:'--cov'},{get:r=>r[8],c:'--ref'}],{minSpan:0.5});
 chart('cP',[{get:r=>pct(r[9]),c:'--press'}],{minSpan:2});
}

function ticks(lo,hi,n){
 const raw=(hi-lo)/n,mag=Math.pow(10,Math.floor(Math.log10(raw))),e=raw/mag;
 const step=(e>=7.5?10:e>=3.5?5:e>=1.5?2:1)*mag;
 const a=Math.floor(lo/step)*step,b=Math.ceil(hi/step)*step,list=[];
 for(let k=0;a+k*step<=b+step*1e-6;k++){const v=a+k*step;list.push(Math.abs(v)<step*1e-9?0:v);}
 return {lo:a,hi:b,list,dec:Math.max(0,-Math.floor(Math.log10(step)))};
}

function chart(id,series,opt){
 const cv=$(id),dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
 if(!w||!h)return;
 if(cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr);}
 const g=cv.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,w,h);
 g.font='11px system-ui,sans-serif';
 const muted=css('--muted'),grid=css('--grid'),R=S.rows,L=52,RT=16,T=8,B=20,pw=w-L-RT,ph=h-T-B;
 if(!R.length){g.fillStyle=muted;g.fillText('Waiting for data…',L,T+20);return;}
 const tEnd=R[R.length-1][1],t0=tEnd-S.win;
 let i0=R.length-1;while(i0>0&&R[i0-1][1]>=t0)i0--;
 let lo=Infinity,hi=-Infinity;
 for(let i=i0;i<R.length;i++)for(const s of series){const v=s.get(R[i]);if(ok(v)){if(v<lo)lo=v;if(v>hi)hi=v;}}
 if(lo===Infinity){g.fillStyle=css('--bad');g.fillText('No valid data — sensor DISCONNECTED',L,T+20);return;}
 if(opt.zero){lo=Math.min(lo,0);hi=Math.max(hi,0);}
 if(hi-lo<opt.minSpan){const m=(hi+lo)/2;lo=m-opt.minSpan/2;hi=m+opt.minSpan/2;}
 const tk=ticks(lo,hi,4);lo=tk.lo;hi=tk.hi;
 const X=t=>L+(t-t0)/S.win*pw,Y=v=>T+(hi-v)/(hi-lo)*ph;
 g.strokeStyle=grid;g.lineWidth=1;g.fillStyle=muted;g.textAlign='right';g.textBaseline='middle';
 for(const v of tk.list){const y=Math.round(Y(v))+0.5;g.beginPath();g.moveTo(L,y);g.lineTo(L+pw,y);g.stroke();g.fillText(v.toFixed(tk.dec),L-6,y);}
 const step=S.win<=60000?10000:S.win<=300000?60000:180000;
 g.textBaseline='top';
 for(let a=0;a<=S.win;a+=step){
  const x=Math.round(X(tEnd-a))+0.5;g.beginPath();g.moveTo(x,T);g.lineTo(x,T+ph);g.stroke();
  g.textAlign=a===0?'right':(a===S.win?'left':'center');
  g.fillText(a===0?'now':(step<60000?'−'+a/1000+' s':'−'+a/60000+' min'),x,T+ph+4);
 }
 g.save();g.beginPath();g.rect(L,T,pw,ph);g.clip();g.lineWidth=2;g.lineJoin='round';
 for(const s of series){
  g.strokeStyle=css(s.c);g.beginPath();let pt=null;
  for(let i=i0;i<R.length;i++){
   const r=R[i],v=s.get(r);
   if(!ok(v)){pt=null;continue;}
   const x=X(r[1]),y=Y(v);
   if(pt===null||r[1]-pt>3000)g.moveTo(x,y);else g.lineTo(x,y);
   pt=r[1];
  }
  g.stroke();
 }
 g.restore();
}
poll();
</script></body></html>
)rawliteral";

// ============================================================================
// Excitation: 2 kHz sine synthesised with LEDC PWM + timer ISR
// ============================================================================
void ARDUINO_ISR_ATTR onSineTimer() {
  ledcWrite(PIN_DAC_SINE, sineLut[sineIdx]);
  sineIdx = (sineIdx + 1) & (SINE_POINTS - 1);
}

void computeExcitationModel() {
  const float x = (float)EXC_FREQ_HZ / SK_FC_HZ;
  const float hFilter = 1.0f / sqrtf((1.0f - x * x) * (1.0f - x * x) + (x / SK_Q) * (x / SK_Q));
  const float zohArg = (float)M_PI / SINE_POINTS;
  const float hZoh = sinf(zohArg) / zohArg;  // staircase fundamental gain
  const float pwmPeak_mV = (float)SINE_AMPL_COUNTS / (float)(1u << PWM_RES_BITS) * VCC_MV;
  excPeak_mV  = pwmPeak_mV * hZoh * hFilter;      // at NODE_AC / R6 input
  skinPeak_uA = excPeak_mV / R6_SAFETY_OHMS * 1000.0f;  // electrodes shorted
  kNominal    = R7_TIA_OHMS * excPeak_mV;
}

bool startExcitation() {
  if (skinPeak_uA >= SKIN_CURRENT_LIMIT_UA) {
    Serial.printf("# ERROR: excitation model predicts %.2f uA peak (limit %.1f uA). Excitation NOT started.\n",
                  skinPeak_uA, SKIN_CURRENT_LIMIT_UA);
    return false;
  }
  for (int i = 0; i < SINE_POINTS; i++) {
    const float s = sinf(2.0f * (float)M_PI * i / SINE_POINTS);
    sineLut[i] = (uint32_t)lroundf((float)PWM_MID_COUNTS + (float)SINE_AMPL_COUNTS * s);
  }
  if (!ledcAttach(PIN_DAC_SINE, PWM_CARRIER_HZ, PWM_RES_BITS)) {
    Serial.printf("# ERROR: ledcAttach(GPIO%d, %lu Hz, %u bit) failed - no excitation.\n",
                  PIN_DAC_SINE, (unsigned long)PWM_CARRIER_HZ, PWM_RES_BITS);
    return false;
  }
  ledcWrite(PIN_DAC_SINE, PWM_MID_COUNTS);
  sineTimer = timerBegin(SINE_TIMER_HZ);
  if (sineTimer == nullptr) {
    Serial.println("# ERROR: timerBegin failed - no excitation.");
    return false;
  }
  timerAttachInterrupt(sineTimer, &onSineTimer);
  timerAlarm(sineTimer, SINE_TIMER_HZ / (EXC_FREQ_HZ * SINE_POINTS), true, 0);
  Serial.printf("# Excitation: GPIO%d PWM %lu Hz x %u bit, %d-point sine at %lu Hz\n",
                PIN_DAC_SINE, (unsigned long)ledcReadFreq(PIN_DAC_SINE), PWM_RES_BITS, SINE_POINTS,
                (unsigned long)EXC_FREQ_HZ);
  Serial.printf("#   %.0f mV peak after Sallen-Key -> %.2f uA peak max skin current via R6 (limit %.0f uA)\n",
                excPeak_mV, skinPeak_uA, SKIN_CURRENT_LIMIT_UA);
  return true;
}

// ============================================================================
// I2C helpers and drivers
// ============================================================================
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWrite16(uint8_t addr, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

bool i2cRead16(uint8_t addr, uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  const uint8_t hi = Wire.read();
  const uint8_t lo = Wire.read();
  value = ((uint16_t)hi << 8) | lo;
  return true;
}

void setDeviceState(bool &state, bool nowOk, const char *name, uint8_t addr) {
  if (state && !nowOk) {
    Serial.printf("# ERROR: %s (0x%02X) stopped responding - marked DISCONNECTED, retrying every %lu s\n",
                  name, addr, (unsigned long)(DEVICE_RETRY_MS / 1000));
  } else if (!state && nowOk) {
    Serial.printf("# INFO: %s (0x%02X) connected\n", name, addr);
  }
  state = nowOk;
}

// ADS1115 is identified by a config-register round trip (OS=0, so no conversion starts).
bool adsProbe() {
  if (!i2cPing(ADS1115_ADDR)) return false;
  const uint16_t testCfg = ADS_MUX_AIN0 | ADS_PGA_4_096 | ADS_MODE_SINGLE | ADS_DR_860SPS | ADS_COMP_DISABLE;
  uint16_t rb = 0;
  if (!i2cWrite16(ADS1115_ADDR, ADS_REG_CONFIG, testCfg)) return false;
  if (!i2cRead16(ADS1115_ADDR, ADS_REG_CONFIG, rb)) return false;
  return (rb & 0x7FFF) == testCfg;
}

// Single-ended reading in mV, averaged over nAvg single-shot conversions.
bool adsReadMv(uint16_t mux, uint8_t nAvg, float &mv) {
  int32_t sum = 0;
  for (uint8_t i = 0; i < nAvg; i++) {
    const uint16_t cfg = ADS_OS_SINGLE | mux | ADS_PGA_4_096 | ADS_MODE_SINGLE | ADS_DR_860SPS | ADS_COMP_DISABLE;
    if (!i2cWrite16(ADS1115_ADDR, ADS_REG_CONFIG, cfg)) return false;
    delayMicroseconds(1100);
    const uint32_t t0 = micros();
    uint16_t status = 0;
    while (true) {
      if (!i2cRead16(ADS1115_ADDR, ADS_REG_CONFIG, status)) return false;
      if (status & ADS_OS_SINGLE) break;
      if (micros() - t0 > 10000) return false;
      delayMicroseconds(100);
    }
    uint16_t raw = 0;
    if (!i2cRead16(ADS1115_ADDR, ADS_REG_CONV, raw)) return false;
    sum += (int16_t)raw;
  }
  mv = ((float)sum / nAvg) * ADS_LSB_MV;
  return true;
}

// TMP117 is identified by DEVICE_ID, then set to a 250 ms conversion cycle.
bool tmpProbe(uint8_t addr, uint16_t &idOut) {
  idOut = 0;
  if (!i2cPing(addr)) return false;
  uint16_t id = 0;
  if (!i2cRead16(addr, TMP_REG_DEVICE_ID, id)) return false;
  idOut = id;
  if ((id & 0x0FFF) != TMP_DEVICE_ID) return false;
  return i2cWrite16(addr, TMP_REG_CONFIG, TMP_CONFIG_250MS);
}

float readTmp(uint8_t addr, bool &state, const char *name) {
  if (!state) return NAN;
  uint16_t raw = 0;
  if (!i2cRead16(addr, TMP_REG_TEMP, raw)) {
    setDeviceState(state, false, name, addr);
    return NAN;
  }
  if (raw == 0x8000) return NAN;  // power-on value, first conversion not finished yet
  return (int16_t)raw * TMP117_LSB_C;
}

void scanI2C() {
  Serial.print("# I2C scan (SDA=GPIO");
  Serial.print(PIN_I2C_SDA);
  Serial.print(", SCL=GPIO");
  Serial.print(PIN_I2C_SCL);
  Serial.print("):");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (i2cPing(a)) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  Serial.println(found ? "" : " no devices - check SDA/SCL wiring, pull-ups and 3.3 V");
}

void initDevices() {
  adsOk = adsProbe();
  if (adsOk) {
    Serial.printf("# OK: ADS1115 at 0x%02X (GAIN_ONE +/-4.096 V, 860 SPS single-shot)\n", ADS1115_ADDR);
  } else {
    Serial.printf("# ERROR: ADS1115 not found at 0x%02X. Conductance and pressure will show DISCONNECTED.\n",
                  ADS1115_ADDR);
    Serial.println("#        Check VDD/GND/SDA/SCL and that ADS1115 ADDR is tied to SCL (0x4B).");
    Serial.println("#        With ADDR->GND it sits at 0x48 and collides with TMP117 #1 on the same bus.");
  }

  uint16_t id = 0;
  tmpCovOk = tmpProbe(TMP117_COV_ADDR, id);
  if (tmpCovOk) {
    Serial.printf("# OK: TMP117 #1 (covered) at 0x%02X\n", TMP117_COV_ADDR);
  } else if (id != 0) {
    Serial.printf("# ERROR: device at 0x%02X returned DEVICE_ID 0x%04X, expected 0x0117. Another chip shares this\n"
                  "#        address (ADS1115 with ADDR->GND?). Covered temperature will show DISCONNECTED.\n",
                  TMP117_COV_ADDR, id);
  } else {
    Serial.printf("# ERROR: TMP117 #1 (covered) not found at 0x%02X (ADDR->GND). Will show DISCONNECTED.\n",
                  TMP117_COV_ADDR);
  }

  tmpRefOk = tmpProbe(TMP117_REF_ADDR, id);
  if (tmpRefOk) {
    Serial.printf("# OK: TMP117 #2 (reference) at 0x%02X\n", TMP117_REF_ADDR);
  } else if (id != 0) {
    Serial.printf("# ERROR: device at 0x%02X returned DEVICE_ID 0x%04X, expected 0x0117. Will show DISCONNECTED.\n",
                  TMP117_REF_ADDR, id);
  } else {
    Serial.printf("# ERROR: TMP117 #2 (reference) not found at 0x%02X (ADDR->+3.3V). Will show DISCONNECTED.\n",
                  TMP117_REF_ADDR);
  }
}

void retryDisconnectedDevices() {
  if (millis() - lastRetryMs < DEVICE_RETRY_MS) return;
  lastRetryMs = millis();
  uint16_t id = 0;
  if (!adsOk) setDeviceState(adsOk, adsProbe(), "ADS1115", ADS1115_ADDR);
  if (!tmpCovOk) setDeviceState(tmpCovOk, tmpProbe(TMP117_COV_ADDR, id), "TMP117 #1 (covered)", TMP117_COV_ADDR);
  if (!tmpRefOk) setDeviceState(tmpRefOk, tmpProbe(TMP117_REF_ADDR, id), "TMP117 #2 (reference)", TMP117_REF_ADDR);
}

// ============================================================================
// Ring buffer
// ============================================================================
uint32_t oldestSeq() {
  const uint32_t lo = (totalRows > BUFFER_CAPACITY) ? totalRows - BUFFER_CAPACITY : 0;
  return lo > firstSeq ? lo : firstSeq;
}

uint32_t bufferedCount() { return totalRows - oldestSeq(); }

const Sample &sampleAt(uint32_t seq) { return ring[seq % BUFFER_CAPACITY]; }

void pushSample(const Sample &s) {
  ring[totalRows % BUFFER_CAPACITY] = s;
  totalRows++;
}

// ============================================================================
// Signal conversion
// ============================================================================
float dcOutToMicroSiemens(float dcOut_mV) {
  if (!excitationOk || isnan(dcOut_mV)) return NAN;
  const float env = dcOut_mV - vmid_mV;
  if (env < OPEN_CIRCUIT_ENV_MV) return 0.0f;  // no current flowing: open electrodes
  float rx = calK / env - calRs;
  if (rx < MIN_RX_OHMS) rx = MIN_RX_OHMS;
  return 1.0e6f / rx;
}

// After switching, the peak detector decays from the previous channel's
// envelope toward V_MID with tau = C4*R8. The new reading is trustworthy only
// if it sits above that decay tail.
uint8_t settleFlag(float envNow, float envOther, uint32_t elapsedMs, uint8_t flag) {
  if (isnan(envNow) || isnan(envOther)) return 0;
  const float residual = envOther * expf(-(float)elapsedMs / ENVELOPE_TAU_MS);
  return (envNow <= residual * 1.10f) ? flag : 0;
}

// ============================================================================
// Sampler (non-blocking state machine)
// ============================================================================
void selectChannel(MuxChannel ch) {
  digitalWrite(PIN_MUX_SEL, ch == CH_REFERENCE ? HIGH : LOW);
  activeChannel = ch;
  channelSwitchMs = millis();
}

float readDcOutMv() {
  if (!adsOk) return NAN;
  float mv = NAN;
  if (!adsReadMv(ADS_MUX_AIN0, ADS_DCOUT_AVG, mv)) {
    setDeviceState(adsOk, false, "ADS1115", ADS1115_ADDR);
    return NAN;
  }
  return mv;
}

void readAuxSensors(Sample &s) {
  if (adsOk) {
    float mv = NAN;
    if (adsReadMv(ADS_MUX_AIN1, ADS_AUX_AVG, mv)) {
      s.press_mV = mv;
    } else {
      setDeviceState(adsOk, false, "ADS1115", ADS1115_ADDR);
    }
    if (MEASURE_VMID_ON_A2 && adsOk) {
      if (adsReadMv(ADS_MUX_AIN2, ADS_AUX_AVG, mv)) {
        vmid_mV = mv;
      } else {
        setDeviceState(adsOk, false, "ADS1115", ADS1115_ADDR);
      }
    }
  }
  s.tempCov_C = readTmp(TMP117_COV_ADDR, tmpCovOk, "TMP117 #1 (covered)");
  s.tempRef_C = readTmp(TMP117_REF_ADDR, tmpRefOk, "TMP117 #2 (reference)");
}

void resetPending() {
  pending.t_ms = 0;
  pending.cov_mV = pending.ref_mV = NAN;
  pending.cov_uS = pending.ref_uS = pending.diff_uS = NAN;
  pending.tempCov_C = pending.tempRef_C = pending.press_mV = NAN;
  pending.flags = 0;
}

int fmtVal(char *dst, size_t n, float v, int decimals, const char *nanText) {
  if (isnan(v) || isinf(v)) return snprintf(dst, n, "%s", nanText);
  return snprintf(dst, n, "%.*f", decimals, (double)v);
}

size_t formatCsvRow(const Sample &s, char *out, size_t n) {
  int len = snprintf(out, n, "%lu", (unsigned long)s.t_ms);
  const float vals[8] = {s.cov_mV, s.ref_mV, s.cov_uS, s.ref_uS, s.diff_uS, s.tempCov_C, s.tempRef_C, s.press_mV};
  const int dec[8] = {3, 3, 4, 4, 4, 4, 4, 2};
  for (int i = 0; i < 8 && len < (int)n - 1; i++) {
    out[len++] = ',';
    out[len] = '\0';
    len += fmtVal(out + len, n - len, vals[i], dec[i], "NaN");
  }
  return (size_t)len;
}

void finalizeRow(Sample &s) {
  const uint32_t now = millis();
  s.t_ms = now;
  s.cov_uS = dcOutToMicroSiemens(s.cov_mV);
  s.ref_uS = dcOutToMicroSiemens(s.ref_mV);
  s.diff_uS = (isnan(s.cov_uS) || isnan(s.ref_uS)) ? NAN : s.cov_uS - s.ref_uS;
  if (!isnan(s.cov_mV) && s.cov_mV >= SATURATION_MV) s.flags |= FLAG_COV_SAT;
  if (!isnan(s.ref_mV) && s.ref_mV >= SATURATION_MV) s.flags |= FLAG_REF_SAT;
  pushSample(s);

  char line[160];
  formatCsvRow(s, line, sizeof(line));
  Serial.println(line);

  if (lastRowMs != 0) rowPeriodMs = 0.9f * rowPeriodMs + 0.1f * (float)(now - lastRowMs);
  lastRowMs = now;
}

// Cycle: covered settle 250 ms -> read A0 -> switch to reference (read FSR and
// temperatures during that settle) -> read A0 -> switch back -> emit row.
void serviceSampler() {
  const uint32_t elapsed = millis() - channelSwitchMs;
  if (elapsed < MUX_SETTLE_MS) return;

  const float dc = readDcOutMv();
  const float env = isnan(dc) ? NAN : dc - vmid_mV;

  if (activeChannel == CH_COVERED) {
    resetPending();
    pending.cov_mV = dc;
    pending.flags |= settleFlag(env, lastEnvRef, elapsed, FLAG_COV_SETTLE);
    lastEnvCov = env;
    selectChannel(CH_REFERENCE);
    readAuxSensors(pending);
  } else {
    pending.ref_mV = dc;
    pending.flags |= settleFlag(env, lastEnvCov, elapsed, FLAG_REF_SETTLE);
    lastEnvRef = env;
    selectChannel(CH_COVERED);
    finalizeRow(pending);
    retryDisconnectedDevices();
  }
}

// ============================================================================
// Actions shared by web and serial
// ============================================================================
void saveCalibration() {
  prefs.putFloat("calK", calK);
  prefs.putFloat("calRs", calRs);
  prefs.putBool("calUser", calUser);
}

bool runCalibration(String &msg) {
  if (!adsOk) { msg = "ADS1115 is DISCONNECTED - cannot calibrate."; return false; }
  if (!excitationOk) { msg = "Excitation is off - cannot calibrate."; return false; }
  double sumCov = 0, sumRef = 0;
  int n = 0;
  const uint32_t lo = oldestSeq();
  for (uint32_t s = totalRows; s > lo && n < CAL_AVG_ROWS;) {
    --s;
    const Sample &x = sampleAt(s);
    if (!isnan(x.cov_mV) && !isnan(x.ref_mV)) {
      sumCov += x.cov_mV;
      sumRef += x.ref_mV;
      n++;
    }
  }
  if (n < CAL_MIN_ROWS) { msg = "Need at least 5 valid rows - wait a few seconds and retry."; return false; }
  const float e1 = (float)(sumCov / n) - vmid_mV;  // 100k channel
  const float e2 = (float)(sumRef / n) - vmid_mV;  // 470k channel
  if (e1 < 5.0f || e2 < 5.0f) {
    msg = String("Envelope too small (Cov ") + String(e1, 1) + " mV, Ref " + String(e2, 1) +
          " mV). Check the test resistors and the excitation path.";
    return false;
  }
  if (e1 <= 1.2f * e2) {
    msg = String("Covered envelope (") + String(e1, 1) + " mV) should be ~2.9x the Reference (" + String(e2, 1) +
          " mV). Is 100k on channel 0 and 470k on channel 1?";
    return false;
  }
  const float k = (CAL_R_REF_OHMS - CAL_R_COV_OHMS) / (1.0f / e2 - 1.0f / e1);
  const float rs = k / e1 - CAL_R_COV_OHMS;
  if (!(k > 0.5f * kNominal && k < 2.0f * kNominal) || !(rs > 50000.0f && rs < 200000.0f)) {
    msg = String("Result out of range (k=") + String(k, 0) + " mV*Ohm, Rs=" + String(rs / 1000.0f, 1) +
          " kOhm; nominal k=" + String(kNominal, 0) + ", Rs=100 kOhm). Not saved.";
    return false;
  }
  calK = k;
  calRs = rs;
  calUser = true;
  saveCalibration();
  msg = String("Calibrated from ") + n + " rows: k=" + String(k, 0) + " mV*Ohm, Rs=" + String(rs / 1000.0f, 2) +
        " kOhm. Saved to flash.";
  return true;
}

void resetCalibration(String &msg) {
  calK = kNominal;
  calRs = R6_SAFETY_OHMS;
  calUser = false;
  saveCalibration();
  msg = String("Calibration reset to nominal: k=") + String(calK, 0) + " mV*Ohm, Rs=100 kOhm.";
}

bool runTare(String &msg) {
  if (!adsOk) { msg = "ADS1115 is DISCONNECTED - cannot tare."; return false; }
  double sum = 0;
  int n = 0;
  const uint32_t lo = oldestSeq();
  for (uint32_t s = totalRows; s > lo && n < 4;) {
    --s;
    const Sample &x = sampleAt(s);
    if (!isnan(x.press_mV)) { sum += x.press_mV; n++; }
  }
  if (n == 0) { msg = "No valid FSR reading yet - wait a second and retry."; return false; }
  fsrTare_mV = (float)(sum / n);
  prefs.putFloat("tare", fsrTare_mV);
  msg = String("Pressure tared at ") + String(fsrTare_mV, 1) + " mV.";
  return true;
}

void clearBuffer(String &msg) {
  firstSeq = totalRows;
  bufferEpoch++;
  msg = "Buffer cleared.";
}

// ============================================================================
// Web server
// ============================================================================
class ChunkWriter {
 public:
  void begin(const char *contentType) {
    len_ = 0;
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, contentType, "");
  }
  void add(const char *s) {
    const size_t n = strlen(s);
    if (len_ + n >= sizeof(buf_)) flush();
    if (n >= sizeof(buf_)) {
      server.sendContent(s, n);
      return;
    }
    memcpy(buf_ + len_, s, n);
    len_ += n;
  }
  void flush() {
    if (len_) server.sendContent(buf_, len_);
    len_ = 0;
  }
  void finish() {
    flush();
    server.sendContent("");
  }

 private:
  char buf_[1400];
  size_t len_ = 0;
};
static ChunkWriter chunk;

void sendActionResult(bool ok, const String &msg) {
  String safe = msg;
  safe.replace("\\", "\\\\");
  safe.replace("\"", "\\\"");
  server.sendHeader("Cache-Control", "no-store");
  server.send(ok ? 200 : 409, "application/json", String("{\"ok\":") + (ok ? "true" : "false") + ",\"msg\":\"" + safe + "\"}");
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", DASHBOARD_HTML);
}

void handleApiData() {
  long since = server.hasArg("since") ? server.arg("since").toInt() : -1;
  const uint32_t total = totalRows;
  const uint32_t oldest = oldestSeq();
  uint32_t start = (since < 0) ? oldest : (uint32_t)since + 1;
  if (start < oldest) start = oldest;
  if (start > total) start = total;
  uint32_t end = start + MAX_ROWS_PER_RESPONSE;
  if (end > total) end = total;

  char tmp[200], a[24], b[24], c[24], d[24], e[24];
  server.sendHeader("Cache-Control", "no-store");
  chunk.begin("application/json");
  fmtVal(a, sizeof(a), vmid_mV, 1, "null");
  snprintf(tmp, sizeof(tmp),
           "{\"boot\":%lu,\"epoch\":%lu,\"seq\":%lu,\"count\":%lu,\"cap\":%lu,\"more\":%s,\"uptime\":%lu,\"rowMs\":%.1f,",
           (unsigned long)bootId, (unsigned long)bufferEpoch, (unsigned long)total, (unsigned long)(total - oldest),
           (unsigned long)BUFFER_CAPACITY, end < total ? "true" : "false", (unsigned long)millis(), (double)rowPeriodMs);
  chunk.add(tmp);
  snprintf(tmp, sizeof(tmp), "\"ads\":%s,\"tmpCov\":%s,\"tmpRef\":%s,\"exc\":%s,\"wifi\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,",
           adsOk ? "true" : "false", tmpCovOk ? "true" : "false", tmpRefOk ? "true" : "false",
           excitationOk ? "true" : "false", wifiIsAP ? "AP" : "STA",
           (wifiIsAP ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(), wifiIsAP ? 0 : (int)WiFi.RSSI());
  chunk.add(tmp);
  snprintf(tmp, sizeof(tmp),
           "\"vmid\":%s,\"vcc\":%.1f,\"r9\":%.1f,\"tare\":%.2f,\"excPk\":%.1f,\"iPk\":%.3f,\"settle\":%lu,"
           "\"calK\":%.1f,\"calRs\":%.1f,\"calUser\":%s,\"kNom\":%.1f,\"rows\":[",
           a, (double)VCC_MV, (double)R9_FSR_OHMS, (double)fsrTare_mV, (double)excPeak_mV, (double)skinPeak_uA,
           (unsigned long)MUX_SETTLE_MS, (double)calK, (double)calRs, calUser ? "true" : "false", (double)kNominal);
  chunk.add(tmp);

  for (uint32_t s = start; s < end; s++) {
    const Sample &x = sampleAt(s);
    char f2[24], f3[24], f4[24];
    fmtVal(a, sizeof(a), x.cov_mV, 3, "null");
    fmtVal(b, sizeof(b), x.ref_mV, 3, "null");
    fmtVal(c, sizeof(c), x.cov_uS, 4, "null");
    fmtVal(d, sizeof(d), x.ref_uS, 4, "null");
    fmtVal(e, sizeof(e), x.diff_uS, 4, "null");
    fmtVal(f2, sizeof(f2), x.tempCov_C, 4, "null");
    fmtVal(f3, sizeof(f3), x.tempRef_C, 4, "null");
    fmtVal(f4, sizeof(f4), x.press_mV, 2, "null");
    snprintf(tmp, sizeof(tmp), "%s[%lu,%lu,%s,%s,%s,%s,%s,%s,%s,%s,%u]", s == start ? "" : ",", (unsigned long)s,
             (unsigned long)x.t_ms, a, b, c, d, e, f2, f3, f4, (unsigned)x.flags);
    chunk.add(tmp);
  }
  chunk.add("]}");
  chunk.finish();
}

void handleExportCsv() {
  const uint32_t end = totalRows;  // snapshot: rows added during the download are not included
  const uint32_t start = oldestSeq();
  char disp[96];
  snprintf(disp, sizeof(disp), "attachment; filename=\"ece4781_sweat_%08lX_%lu.csv\"", (unsigned long)bootId,
           (unsigned long)(millis() / 1000));
  server.sendHeader("Content-Disposition", disp);
  server.sendHeader("Cache-Control", "no-store");
  chunk.begin("text/csv");
  chunk.add(CSV_HEADER);
  chunk.add("\n");
  char line[164];
  for (uint32_t s = start; s < end; s++) {
    const size_t n = formatCsvRow(sampleAt(s), line, sizeof(line) - 2);
    line[n] = '\n';
    line[n + 1] = '\0';
    chunk.add(line);
  }
  chunk.finish();
  Serial.printf("# INFO: exported %lu rows to /export.csv\n", (unsigned long)(end - start));
}

void handleTare() { String m; bool ok = runTare(m); sendActionResult(ok, m); }
void handleClear() { String m; clearBuffer(m); sendActionResult(true, m); }
void handleCalibrate() { String m; bool ok = runCalibration(m); Serial.println("# CAL: " + m); sendActionResult(ok, m); }
void handleCalReset() { String m; resetCalibration(m); Serial.println("# CAL: " + m); sendActionResult(true, m); }
void handleNotFound() { server.send(404, "text/plain", "Not found"); }

void startWiFi() {
  WiFi.persistent(false);
  const bool haveCreds = strlen(WIFI_SSID) > 0 && strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0;
  bool connected = false;
  if (haveCreds) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(MDNS_HOSTNAME);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("# Wi-Fi: joining \"%s\"", WIFI_SSID);
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    connected = (WiFi.status() == WL_CONNECTED);
    if (!connected) Serial.println("# Wi-Fi: could not join network - starting fallback access point");
  } else {
    Serial.println("# Wi-Fi: WIFI_SSID not set - starting access point");
  }

  if (connected) {
    wifiIsAP = false;
    WiFi.setSleep(false);
    Serial.printf("# Wi-Fi: connected, IP address %s (RSSI %d dBm)\n", WiFi.localIP().toString().c_str(),
                  (int)WiFi.RSSI());
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASSWORD);
    wifiIsAP = true;
    Serial.printf("# Wi-Fi: access point \"%s\" (password \"%s\"), IP address %s\n", FALLBACK_AP_SSID,
                  FALLBACK_AP_PASSWORD, WiFi.softAPIP().toString().c_str());
  }

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("# Dashboard: http://%s.local/  or  http://%s/\n", MDNS_HOSTNAME,
                  (wifiIsAP ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str());
  } else {
    Serial.printf("# ERROR: mDNS failed to start - use http://%s/\n",
                  (wifiIsAP ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str());
  }
}

void startWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleApiData);
  server.on("/export.csv", HTTP_GET, handleExportCsv);
  server.on("/api/tare", HTTP_POST, handleTare);
  server.on("/api/clear", HTTP_POST, handleClear);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/cal_reset", HTTP_POST, handleCalReset);
  server.onNotFound(handleNotFound);
  server.begin();
}

// ============================================================================
// Serial console
// ============================================================================
void printStatus() {
  Serial.printf("# STATUS: ADS1115 %s | TMP117 #1 %s | TMP117 #2 %s | excitation %s\n",
                adsOk ? "OK" : "DISCONNECTED", tmpCovOk ? "OK" : "DISCONNECTED", tmpRefOk ? "OK" : "DISCONNECTED",
                excitationOk ? "ON" : "OFF");
  Serial.printf("# STATUS: k=%.0f mV*Ohm, Rs=%.2f kOhm (%s), tare=%.1f mV, V_MID=%.1f mV\n", calK, calRs / 1000.0f,
                calUser ? "calibrated" : "nominal", fsrTare_mV, vmid_mV);
  Serial.printf("# STATUS: %lu rows buffered of %lu, row period %.0f ms, IP %s (%s)\n",
                (unsigned long)bufferedCount(), (unsigned long)BUFFER_CAPACITY, rowPeriodMs,
                (wifiIsAP ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(), wifiIsAP ? "AP" : "STA");
}

void handleSerialCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;
  String msg;
  if (cmd == "help") {
    Serial.println("# Commands: status, header, tare, cal, calreset, clear");
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "header") {
    Serial.println(CSV_HEADER);
  } else if (cmd == "tare") {
    runTare(msg);
    Serial.println("# " + msg);
  } else if (cmd == "cal") {
    runCalibration(msg);
    Serial.println("# CAL: " + msg);
  } else if (cmd == "calreset") {
    resetCalibration(msg);
    Serial.println("# CAL: " + msg);
  } else if (cmd == "clear") {
    clearBuffer(msg);
    Serial.println("# " + msg);
  } else {
    Serial.println("# Unknown command '" + cmd + "' - type help");
  }
}

void serviceSerial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialLine.length()) handleSerialCommand(serialLine);
      serialLine = "";
    } else if (serialLine.length() < 32) {
      serialLine += ch;
    }
  }
}

// ============================================================================
// Arduino entry points
// ============================================================================
void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);
  bootId = esp_random();

  Serial.println();
  Serial.println("# ==== ECE-4781 Team 21 - Dual-Sensor Skin Conductance Front-End ====");
  Serial.printf("# Pins: SDA=GPIO%d SCL=GPIO%d DAC_SINE=GPIO%d MUX_SEL=GPIO%d\n", PIN_I2C_SDA, PIN_I2C_SCL,
                PIN_DAC_SINE, PIN_MUX_SEL);

  pinMode(PIN_MUX_SEL, OUTPUT);
  selectChannel(CH_COVERED);

  computeExcitationModel();
  prefs.begin("ece4781", false);
  calUser = prefs.getBool("calUser", false);
  calK = calUser ? prefs.getFloat("calK", kNominal) : kNominal;
  calRs = calUser ? prefs.getFloat("calRs", R6_SAFETY_OHMS) : R6_SAFETY_OHMS;
  fsrTare_mV = prefs.getFloat("tare", 0.0f);

  excitationOk = startExcitation();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(25);
  scanI2C();
  initDevices();
  lastRetryMs = millis();

  Serial.printf("# Conversion: k=%.0f mV*Ohm, Rs=%.2f kOhm (%s). Expected DC_OUT with test resistors: "
                "100k -> %.0f mV, 470k -> %.0f mV\n",
                calK, calRs / 1000.0f, calUser ? "stored calibration" : "nominal", V_MID_MV + calK / (calRs + 100000.0f),
                V_MID_MV + calK / (calRs + 470000.0f));

  startWiFi();
  startWebServer();

  Serial.println("# Type 'help' for commands. CSV follows:");
  Serial.println(CSV_HEADER);
  resetPending();
  selectChannel(CH_COVERED);  // restart the settle timer after the Wi-Fi delay
}

void loop() {
  server.handleClient();
  serviceSampler();
  serviceSerial();
}
