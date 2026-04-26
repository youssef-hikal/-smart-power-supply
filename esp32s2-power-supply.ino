/*
 * ============================================================
 *  Advanced Programmable Power Supply
 *  IEEE Nile University — by eng Youssef Yaser and youssef hikal
 *  Target: ESP32-S2  |  Arduino core 2.x
 * ============================================================
 *
 *  Libraries (install via Library Manager):
 *   INA226_WE        – by Wolfgang Ewald
 *   Adafruit_GFX     – Adafruit GFX Library
 *   Adafruit_SSD1306 – Adafruit SSD1306
 *
 *  No FastLED, no touch_pad driver needed.
 *
 * ============================================================
 */

// ─── Pins ────────────────────────────────────────────────────
#define SDA_PIN                3
#define SCL_PIN                7
#define PULSE_LED_PIN         15
#define ENC_CLK               17
#define ENC_DT                21
#define ENC_SW                34   // encoder push-button
#define BTN1_PIN              11   // was TOUCH_PAD_NUM11 -> plain GPIO
#define BTN2_PIN              12   // was TOUCH_PAD_NUM12 -> plain GPIO
#define PROTECTION_MOSFET_PIN  1
#define DC_CONTROL_PIN        40
#define NTC_ADC_PIN            2
#define LED_UI_PIN            39   // WS2812 data line (GPIO >= 32)
#define NUM_LEDS               2

// ─── INA226 ──────────────────────────────────────────────────
#define INA226_I2C_ADDRESS    0x40
#define SHUNT_RESISTANCE_OHMS 0.0053f
#define SHUNT_MAX_CURRENT_A   3.2f

// ─── NTC thermistor ──────────────────────────────────────────
#define NTC_NOMINAL_RES      10000.0f
#define NTC_BETA_COEFF       3470.0f
#define NTC_SERIES_RESISTOR  3300.0f
#define NTC_NOMINAL_TEMP_C   25.0f

// ─── Protection thresholds ───────────────────────────────────
#define TEMP_MAX_C     70.0f
#define CURRENT_MAX_A   3.0f

// ─── Includes ────────────────────────────────────────────────
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <INA226_WE.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "esp_rom_sys.h"
#include "soc/gpio_struct.h"

// ═══════════════════════════════════════════════════════════════
//  WS2812 bit-bang driver (no FastLED)
//  LED_UI_PIN = 39 (>= 32) uses the high GPIO bank: out1_w1ts/tc
// ═══════════════════════════════════════════════════════════════
#define WS_MASK  (1u << (LED_UI_PIN - 32))

struct RGB { uint8_t r, g, b; };
static RGB ws_leds[NUM_LEDS];

static inline void IRAM_ATTR ws_byte(uint8_t v) {
  for (int b = 7; b >= 0; b--) {
    if ((v >> b) & 1) {
      GPIO.out1_w1ts.val = WS_MASK;
      esp_rom_delay_us(1);
      GPIO.out1_w1tc.val = WS_MASK;
    } else {
      GPIO.out1_w1ts.val = WS_MASK;
      GPIO.out1_w1tc.val = WS_MASK;
      esp_rom_delay_us(1);
    }
  }
}

static void ws_show() {
  portDISABLE_INTERRUPTS();
  for (int i = 0; i < NUM_LEDS; i++) {
    ws_byte(ws_leds[i].g);  // WS2812 order: G R B
    ws_byte(ws_leds[i].r);
    ws_byte(ws_leds[i].b);
  }
  portENABLE_INTERRUPTS();
  esp_rom_delay_us(80);
}

static void ws_set(int idx, uint8_t r, uint8_t g, uint8_t b,
                   uint8_t bright = 60) {
  if (idx < 0 || idx >= NUM_LEDS) return;
  ws_leds[idx] = {
    (uint8_t)((uint16_t)r * bright / 255),
    (uint8_t)((uint16_t)g * bright / 255),
    (uint8_t)((uint16_t)b * bright / 255)
  };
}

// ═══════════════════════════════════════════════════════════════
//  Objects
// ═══════════════════════════════════════════════════════════════
#define OLED_W 128
#define OLED_H  64
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

INA226_WE ina226(INA226_I2C_ADDRESS);

WebServer server(80);

// ─── WiFi ────────────────────────────────────────────────────
const char* AP_SSID = "PowerSupply";
const char* AP_PASS = "";           // open network

// ─── PWM ─────────────────────────────────────────────────────
#define PWM_CH    0
#define PWM_FREQ  5000
#define PWM_BITS  8
uint8_t pwmDuty = 0;

// ─── Measurements ────────────────────────────────────────────
struct Meas {
  float    voltage  = 0;
  float    current  = 0;
  float    power    = 0;
  float    shunt    = 0;
  float    tempC    = 0;
  float    energyWh = 0;
  uint32_t uptime   = 0;
  bool     protect  = false;
} M;

// ─── History ring-buffer (60 s) ──────────────────────────────
#define HIST 60
float    hV[HIST] = {}, hI[HIST] = {};
uint8_t  hHead = 0;
uint32_t hLast = 0;

// ─── Encoder ─────────────────────────────────────────────────
volatile int encPos = 128;
int lastCLK = HIGH;

void IRAM_ATTR encISR() {
  int clk = digitalRead(ENC_CLK);
  if (clk != lastCLK && clk == LOW) {
    if (digitalRead(ENC_DT) != clk) { if (encPos < 255) encPos++; }
    else                             { if (encPos > 0)   encPos--; }
  }
  lastCLK = clk;
}

// ─── UI state ────────────────────────────────────────────────
enum Page { PAGE_MAIN, PAGE_GRAPH, PAGE_SET, PAGE_CNT };
Page curPage  = PAGE_MAIN;
bool outputOn = true;

bool encBtnLast = HIGH, btn1Last = HIGH, btn2Last = HIGH;

// ─── Protection ──────────────────────────────────────────────
void setProtect(bool on) {
  M.protect = on;
  digitalWrite(PROTECTION_MOSFET_PIN, on ? LOW : HIGH);
  ws_set(0, on ? 255 : 0, 0, 0);
  ws_set(1, on ? 255 : 0, 0, 0);
  ws_show();
}

// ─── NTC temperature ─────────────────────────────────────────
float readTemp() {
  int raw = analogRead(NTC_ADC_PIN);
  if (raw <= 0 || raw >= 4095) return 25.0f;
  float ratio = (float)raw / 4095.0f;
  float rNTC  = NTC_SERIES_RESISTOR * ratio / (1.0f - ratio);
  float inv   = logf(rNTC / NTC_NOMINAL_RES) / NTC_BETA_COEFF
              + 1.0f / (NTC_NOMINAL_TEMP_C + 273.15f);
  return (1.0f / inv) - 273.15f;
}

// ─── OLED bar helper ─────────────────────────────────────────
void drawBar(int x, int y, int w, int h, float pct) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int fill = constrain((int)(pct * (w - 2)), 0, w - 2);
  if (fill > 0)
    display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
}

// ─── Splash screen ───────────────────────────────────────────
void drawSplash() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(0, 0, 128, 3, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(18, 7);  display.print(F("IEEE NILE UNIVERSITY"));
  display.drawLine(0, 17, 127, 17, SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(4, 21);  display.print(F("PowerSupply"));
  display.setTextSize(1);
  display.setCursor(25, 42); display.print(F("by Youssef Yaser"));
  display.drawLine(0, 54, 127, 54, SSD1306_WHITE);
  display.setCursor(20, 57); display.print(F("Initialising..."));
  display.fillRect(0, 61, 128, 3, SSD1306_WHITE);
  display.display();
  delay(2500);
}

// ─── OLED: Main ──────────────────────────────────────────────
void drawMain() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);  display.print(F("PSU"));
  char b[24];
  display.setCursor(30, 2);
  snprintf(b, sizeof(b), "%s %3.0f%%", outputOn ? "ON " : "OFF", pwmDuty / 2.55f);
  display.print(b);
  display.setCursor(98, 2);
  display.print(M.protect ? "!OVR" : "    ");
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 13);
  snprintf(b, sizeof(b), "%.2fV", M.voltage);  display.print(b);
  display.setCursor(0, 30);
  snprintf(b, sizeof(b), "%.3fA", M.current);  display.print(b);
  display.setTextSize(1);
  display.setCursor(80, 13);
  snprintf(b, sizeof(b), "%.2fW", M.power);    display.print(b);
  display.setCursor(80, 24);
  snprintf(b, sizeof(b), "%.1fC", M.tempC);    display.print(b);
  display.setCursor(0, 48);  display.print(F("I:"));
  drawBar(12, 48, 68, 6, M.current / SHUNT_MAX_CURRENT_A);
  display.setCursor(84, 48); display.print(F("T:"));
  drawBar(95, 48, 30, 6, M.tempC / TEMP_MAX_C);
  display.setCursor(0, 57);  display.print(F("PWM:"));
  drawBar(26, 57, 100, 6, pwmDuty / 255.0f);
  display.display();
}

// ─── OLED: Graph ─────────────────────────────────────────────
void drawGraph() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(22, 1); display.print(F("V / I HISTORY"));
  display.setTextColor(SSD1306_WHITE);
  char b[10];
  float vMax = 0.01f;
  for (int i = 0; i < HIST; i++) if (hV[i] > vMax) vMax = hV[i];
  for (int i = 0; i < HIST; i++) {
    int idx = (hHead + i) % HIST;
    int h   = (int)(hV[idx] / vMax * 24);
    int x   = i * 128 / HIST;
    display.drawLine(x, 37 - h, x, 37, SSD1306_WHITE);
  }
  snprintf(b, sizeof(b), "%.1fV", vMax);
  display.setCursor(0, 11); display.print(b);
  display.drawLine(0, 39, 127, 39, SSD1306_WHITE);
  float iMax = 0.01f;
  for (int i = 0; i < HIST; i++) if (hI[i] > iMax) iMax = hI[i];
  for (int i = 0; i < HIST; i++) {
    int idx = (hHead + i) % HIST;
    int h   = (int)(hI[idx] / iMax * 22);
    int x   = i * 128 / HIST;
    display.drawLine(x, 63 - h, x, 63, SSD1306_WHITE);
  }
  snprintf(b, sizeof(b), "%.2fA", iMax);
  display.setCursor(0, 41); display.print(b);
  display.display();
}

// ─── OLED: Settings ──────────────────────────────────────────
void drawSettings() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(28, 1); display.print(F("SETTINGS"));
  display.setTextColor(SSD1306_WHITE);
  char b[32];
  display.setCursor(0, 12);
  snprintf(b, sizeof(b), "PWM    : %3d (%.0f%%)", pwmDuty, pwmDuty / 2.55f);
  display.print(b);
  display.setCursor(0, 22);
  snprintf(b, sizeof(b), "OUTPUT : %s", outputOn ? "ENABLED" : "DISABLED");
  display.print(b);
  display.setCursor(0, 32);
  snprintf(b, sizeof(b), "MAX-T  : %.0f C", TEMP_MAX_C);
  display.print(b);
  display.setCursor(0, 42);
  snprintf(b, sizeof(b), "MAX-I  : %.1f A", CURRENT_MAX_A);
  display.print(b);
  display.setCursor(0, 52);
  snprintf(b, sizeof(b), "Energy : %.3f Wh", M.energyWh);
  display.print(b);
  display.display();
}

// ═══════════════════════════════════════════════════════════════
//  Web dashboard (stored in flash)
// ═══════════════════════════════════════════════════════════════
const char HTML[] PROGMEM = R"RAW(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>PowerSupply - IEEE Nile University</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Rajdhani:wght@400;600;700&family=Share+Tech+Mono&display=swap');
:root{--bg:#04080f;--card:#0d1424;--border:#1a2d55;--ac:#0a84ff;--ac2:#00d4ff;--warn:#ff6b35;--ok:#00e5a0;--txt:#c8d8f0;--dim:#4a6080}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Rajdhani',sans-serif;min-height:100vh}
body::before{content:'';position:fixed;inset:0;background-image:linear-gradient(rgba(10,132,255,.04)1px,transparent 1px),linear-gradient(90deg,rgba(10,132,255,.04)1px,transparent 1px);background-size:40px 40px;pointer-events:none;z-index:0}
.wrap{position:relative;z-index:1;max-width:1100px;margin:0 auto;padding:20px 14px}
header{display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid var(--border);padding-bottom:14px;margin-bottom:20px;flex-wrap:wrap;gap:10px}
.brand{font-size:2rem;font-weight:700;letter-spacing:.1em;background:linear-gradient(90deg,var(--ac2),var(--ac));-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{font-size:.75rem;color:var(--dim);letter-spacing:.18em;margin-top:2px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--ok);box-shadow:0 0 8px var(--ok);display:inline-block;margin-right:6px;animation:p 2s infinite}
@keyframes p{0%,100%{opacity:1}50%{opacity:.3}}
.live{font-size:.75rem;color:var(--ok);display:flex;align-items:center}
.upt{font-family:'Share Tech Mono',monospace;font-size:.82rem;color:var(--dim)}
.g3{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin-bottom:16px}
.g2{display:grid;grid-template-columns:2fr 1fr;gap:14px;margin-bottom:16px}
@media(max-width:640px){.g3,.g2{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:18px;position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;background:linear-gradient(90deg,transparent,var(--ac),transparent);opacity:.5}
.lbl{font-size:.68rem;letter-spacing:.2em;text-transform:uppercase;color:var(--dim);margin-bottom:6px}
.val{font-family:'Share Tech Mono',monospace;font-size:2.3rem;line-height:1}
.unit{font-size:.95rem;color:var(--dim);margin-left:3px}
.sub2{font-size:.78rem;color:var(--dim);margin-top:5px;font-family:'Share Tech Mono',monospace}
.track{height:5px;background:rgba(255,255,255,.07);border-radius:3px;overflow:hidden;margin-top:12px}
.fill{height:100%;border-radius:3px;background:linear-gradient(90deg,var(--ac),var(--ac2));transition:width .5s}
.fill.hot{background:linear-gradient(90deg,var(--warn),#ff2d55)}
.gl{display:flex;justify-content:space-between;font-size:.62rem;color:var(--dim);margin-top:3px}
canvas{width:100%;height:170px;display:block;margin-top:10px}
.badge{display:inline-flex;align-items:center;gap:5px;padding:4px 12px;border-radius:20px;font-size:.73rem;letter-spacing:.1em;background:rgba(0,229,160,.1);border:1px solid var(--ok);color:var(--ok)}
.badge.alert{background:rgba(255,107,53,.12);border-color:var(--warn);color:var(--warn)}
.sl{-webkit-appearance:none;width:100%;height:5px;border-radius:3px;background:rgba(255,255,255,.1);outline:none;cursor:pointer;margin-top:6px}
.sl::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;border-radius:50%;background:var(--ac);box-shadow:0 0 10px var(--ac);cursor:pointer}
.plbl{display:flex;justify-content:space-between;font-size:.73rem;color:var(--dim);margin-top:12px}
.pval{font-family:'Share Tech Mono',monospace;color:var(--ac2);font-size:1rem}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
.btn{padding:7px 16px;border-radius:7px;border:1px solid var(--border);background:rgba(10,132,255,.1);color:var(--txt);cursor:pointer;font-family:'Rajdhani',sans-serif;font-size:.82rem;letter-spacing:.07em;transition:all .2s}
.btn:hover{background:rgba(10,132,255,.25);border-color:var(--ac)}
.btn.d{border-color:var(--warn);background:rgba(255,107,53,.1);color:var(--warn)}
.btn.d:hover{background:rgba(255,107,53,.25)}
.btn.ok{border-color:var(--ok);background:rgba(0,229,160,.1);color:var(--ok)}
.btn.ok:hover{background:rgba(0,229,160,.25)}
table{width:100%;border-collapse:collapse;font-size:.8rem;margin-top:12px}
td{padding:5px 7px;border-bottom:1px solid rgba(26,45,85,.5)}
td:first-child{color:var(--dim)}
td:last-child{font-family:'Share Tech Mono',monospace;color:var(--ac2)}
.log{background:#020509;border:1px solid var(--border);border-radius:7px;padding:10px;height:140px;overflow-y:auto;font-family:'Share Tech Mono',monospace;font-size:.7rem;color:#4a9;line-height:1.6;margin-top:10px}
.log .ts{color:var(--dim)}.log .er{color:var(--warn)}
footer{text-align:center;font-size:.68rem;color:var(--dim);border-top:1px solid var(--border);padding-top:14px;margin-top:20px;letter-spacing:.1em}
</style></head><body>
<div class="wrap">
<header>
  <div><div class="brand">POWER SUPPLY</div>
  <div class="sub">IEEE Nile University &nbsp;·&nbsp; by Youssef Yaser</div></div>
  <div style="display:flex;flex-direction:column;align-items:flex-end;gap:5px">
    <div class="live"><span class="dot" id="dot"></span>LIVE</div>
    <div class="upt" id="upt">UP: 00:00:00</div>
  </div>
</header>
<div class="g3">
  <div class="card"><div class="lbl">Bus Voltage</div>
    <div class="val"><span id="voltage">--</span><span class="unit">V</span></div>
    <div class="sub2" id="shunt">Shunt: --mV</div>
    <div class="track"><div class="fill" id="vbar" style="width:0%"></div></div>
    <div class="gl"><span>0V</span><span>30V</span></div></div>
  <div class="card"><div class="lbl">Current</div>
    <div class="val"><span id="current">--</span><span class="unit">A</span></div>
    <div class="sub2" id="power">Power: --W</div>
    <div class="track"><div class="fill" id="ibar" style="width:0%"></div></div>
    <div class="gl"><span>0A</span><span>3.2A</span></div></div>
  <div class="card"><div class="lbl">Temperature</div>
    <div class="val"><span id="temp">--</span><span class="unit">C</span></div>
    <div class="sub2" id="energy">Energy: --Wh</div>
    <div class="track"><div class="fill" id="tbar" style="width:0%"></div></div>
    <div class="gl"><span>0C</span><span>70C</span></div></div>
</div>
<div class="g2">
  <div class="card"><div class="lbl">Live Chart - Voltage &amp; Current</div>
    <canvas id="cv"></canvas></div>
  <div class="card"><div class="lbl">Output Control</div>
    <div style="margin-bottom:10px"><span class="badge" id="badge">&#x25CF; NORMAL</span></div>
    <div class="plbl"><span>PWM Duty</span><span class="pval"><span id="pct">0</span>%</span></div>
    <input type="range" class="sl" id="sl" min="0" max="255" value="0" oninput="setPWM(this.value)">
    <div class="btns">
      <button class="btn ok" onclick="cmd('output_on')">Output ON</button>
      <button class="btn d"  onclick="cmd('output_off')">Output OFF</button>
      <button class="btn d"  onclick="cmd('clear_protection')">Clear Prot.</button>
    </div>
    <table>
      <tr><td>PWM</td><td><span id="pwmraw">--</span>/255</td></tr>
      <tr><td>Output</td><td id="outst">--</td></tr>
      <tr><td>Protection</td><td id="protst">--</td></tr>
      <tr><td>IP</td><td>192.168.4.1</td></tr>
    </table></div>
</div>
<div class="card" style="margin-bottom:16px">
  <div class="lbl" style="display:flex;justify-content:space-between">
    <span>Debug Log</span>
    <button class="btn" style="padding:2px 9px;font-size:.68rem" onclick="logBox.innerHTML=''">Clear</button>
  </div>
  <div class="log" id="logBox"></div>
</div>
<footer>POWER SUPPLY CONTROLLER &nbsp;|&nbsp; IEEE NILE UNIVERSITY &nbsp;|&nbsp; YOUSSEF YASER &nbsp;|&nbsp; ESP32-S2</footer>
</div>
<script>
const HIST=60,cv=document.getElementById('cv'),ctx=cv.getContext('2d'),logBox=document.getElementById('logBox');
let vD=new Array(HIST).fill(0),iD=new Array(HIST).fill(0);
function rsz(){cv.width=cv.offsetWidth;cv.height=cv.offsetHeight;}
window.addEventListener('resize',rsz);rsz();
function drawChart(){
  const W=cv.width,H=cv.height;
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle='rgba(26,45,85,.5)';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){ctx.beginPath();ctx.moveTo(0,H/4*i);ctx.lineTo(W,H/4*i);ctx.stroke();}
  for(let i=0;i<=5;i++){ctx.beginPath();ctx.moveTo(W/5*i,0);ctx.lineTo(W/5*i,H);ctx.stroke();}
  function line(d,col){
    let mx=Math.max(...d,0.01);
    ctx.beginPath();ctx.strokeStyle=col;ctx.lineWidth=2;ctx.shadowColor=col;ctx.shadowBlur=5;
    for(let i=0;i<d.length;i++){let x=i/(d.length-1)*W,y=H-(d[i]/mx)*H*.85-H*.05;i?ctx.lineTo(x,y):ctx.moveTo(x,y);}
    ctx.stroke();ctx.shadowBlur=0;
  }
  line(vD,'#00d4ff');line(iD,'#00e5a0');
  ctx.font='11px Share Tech Mono,monospace';
  ctx.fillStyle='#00d4ff';ctx.fillText('V',6,14);
  ctx.fillStyle='#00e5a0';ctx.fillText('I',6,27);
}
function lg(m,cls){
  const t=new Date(),ts=String(t.getHours()).padStart(2,'0')+':'+String(t.getMinutes()).padStart(2,'0')+':'+String(t.getSeconds()).padStart(2,'0');
  const l=document.createElement('div');l.innerHTML='<span class="ts">['+ts+']</span> '+m;
  if(cls)l.classList.add(cls);logBox.appendChild(l);logBox.scrollTop=logBox.scrollHeight;
}
lg('Dashboard ready');
async function poll(){
  try{
    const d=await(await fetch('/api/data')).json();
    document.getElementById('voltage').textContent=d.voltage.toFixed(3);
    document.getElementById('current').textContent=d.current.toFixed(4);
    document.getElementById('power').textContent='Power: '+d.power.toFixed(3)+'W';
    document.getElementById('shunt').textContent='Shunt: '+(d.shunt*1000).toFixed(2)+'mV';
    document.getElementById('temp').textContent=d.temp.toFixed(1);
    document.getElementById('energy').textContent='Energy: '+d.energy.toFixed(3)+'Wh';
    document.getElementById('vbar').style.width=Math.min(d.voltage/30*100,100)+'%';
    document.getElementById('ibar').style.width=Math.min(d.current/3.2*100,100)+'%';
    const tp=Math.min(d.temp/70*100,100),tb=document.getElementById('tbar');
    tb.style.width=tp+'%';tb.className='fill'+(tp>80?' hot':'');
    const bd=document.getElementById('badge');
    bd.textContent=d.protected?'! PROTECTED':'● NORMAL';
    bd.className='badge'+(d.protected?' alert':'');
    document.getElementById('pwmraw').textContent=d.pwm;
    document.getElementById('pct').textContent=(d.pwm/2.55).toFixed(0);
    document.getElementById('sl').value=d.pwm;
    document.getElementById('outst').textContent=d.output?'ENABLED':'DISABLED';
    document.getElementById('protst').textContent=d.protected?'TRIPPED':'OK';
    const u=d.uptime;
    document.getElementById('upt').textContent='UP: '+String(Math.floor(u/3600)).padStart(2,'0')+':'+String(Math.floor(u/60)%60).padStart(2,'0')+':'+String(u%60).padStart(2,'0');
    document.getElementById('dot').style.background=d.protected?'#ff6b35':'#00e5a0';
    vD.push(d.voltage);vD.shift();iD.push(d.current);iD.shift();drawChart();
  }catch(e){lg('Fetch error: '+e.message,'er');document.getElementById('dot').style.background='#ff2d55';}
}
async function cmd(c){try{const r=await fetch('/api/cmd?c='+c);lg('CMD '+c+' -> '+(await r.text()));}catch(e){lg('CMD error: '+e.message,'er');}}
async function setPWM(v){document.getElementById('pct').textContent=(v/2.55).toFixed(0);try{await fetch('/api/pwm?v='+v);lg('PWM='+v);}catch(e){lg('PWM err','er');}}
poll();setInterval(poll,750);
</script></body></html>)RAW";

// ─── Web handlers ─────────────────────────────────────────────
void handleRoot() { server.send_P(200, "text/html", HTML); }

void handleData() {
  char j[384];
  snprintf(j, sizeof(j),
    "{\"voltage\":%.4f,\"current\":%.5f,\"power\":%.4f,"
    "\"shunt\":%.6f,\"temp\":%.2f,\"energy\":%.4f,"
    "\"pwm\":%d,\"output\":%s,\"protected\":%s,\"uptime\":%lu}",
    M.voltage, M.current, M.power,
    M.shunt,   M.tempC,   M.energyWh,
    pwmDuty,
    outputOn  ? "true" : "false",
    M.protect ? "true" : "false",
    M.uptime);
  server.send(200, "application/json", j);
}

void handleCmd() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "missing c"); return; }
  String c = server.arg("c");
  if      (c == "output_on")         { outputOn = true;  digitalWrite(PROTECTION_MOSFET_PIN, HIGH); }
  else if (c == "output_off")        { outputOn = false; digitalWrite(PROTECTION_MOSFET_PIN, LOW);  }
  else if (c == "clear_protection")  { setProtect(false); }
  else { server.send(400, "text/plain", "unknown"); return; }
  server.send(200, "text/plain", "OK");
}

void handlePwm() {
  if (!server.hasArg("v")) { server.send(400, "text/plain", "missing v"); return; }
  pwmDuty = (uint8_t)constrain(server.arg("v").toInt(), 0, 255);
  ledcWrite(PWM_CH, pwmDuty);
  server.send(200, "text/plain", "OK");
}

// ═══════════════════════════════════════════════════════════════
//  setup
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // output & indicator pins
  pinMode(PROTECTION_MOSFET_PIN, OUTPUT);
  digitalWrite(PROTECTION_MOSFET_PIN, HIGH);
  pinMode(PULSE_LED_PIN, OUTPUT);

  // encoder
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT,  INPUT_PULLUP);
  pinMode(ENC_SW,  INPUT_PULLUP);

  // buttons (simple digital, replaces touch pads)
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // WS2812 startup colour: blue
  pinMode(LED_UI_PIN, OUTPUT);
  digitalWrite(LED_UI_PIN, LOW);
  ws_set(0, 0, 0, 255);
  ws_set(1, 0, 0, 255);
  ws_show();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED init fail"));
  }
  drawSplash();

  // INA226
  if (!ina226.init()) {
    Serial.println(F("INA226 init fail"));
  } else {
    ina226.setResistorRange(SHUNT_RESISTANCE_OHMS, SHUNT_MAX_CURRENT_A);
    ina226.setAverage(INA226_AVERAGE_4);
    ina226.setConversionTime(INA226_CONV_TIME_1100, INA226_CONV_TIME_1100);
    ina226.setMeasureMode(INA226_CONTINUOUS);
  }

  // PWM output
  ledcSetup(PWM_CH, PWM_FREQ, PWM_BITS);
  ledcAttachPin(DC_CONTROL_PIN, PWM_CH);
  ledcWrite(PWM_CH, 0);

  // encoder interrupt
  lastCLK = digitalRead(ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encISR, CHANGE);

  // WiFi soft AP
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("AP IP: ")); Serial.println(WiFi.softAPIP());

  // HTTP routes
  server.on("/",         HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/cmd",  HTTP_GET, handleCmd);
  server.on("/api/pwm",  HTTP_GET, handlePwm);
  server.begin();
  Serial.println(F("HTTP server started"));

  // ready: green
  ws_set(0, 0, 255, 0);
  ws_set(1, 0, 255, 0);
  ws_show();
}

// ═══════════════════════════════════════════════════════════════
//  loop
// ═══════════════════════════════════════════════════════════════
uint32_t tMeas = 0, tOled = 0, tLed = 0;
uint32_t tStart = millis();
uint8_t  ledPh  = 0;

void loop() {
  server.handleClient();
  uint32_t now = millis();

  // encoder button → cycle OLED page
  bool encNow = (digitalRead(ENC_SW) == LOW);
  if (encNow && !encBtnLast) {
    curPage = (Page)((curPage + 1) % PAGE_CNT);
    delay(40);
  }
  encBtnLast = encNow;

  // encoder position → PWM duty
  pwmDuty = (uint8_t)encPos;
  ledcWrite(PWM_CH, outputOn ? pwmDuty : 0);

  // BTN1 → toggle output on/off
  bool b1 = (digitalRead(BTN1_PIN) == LOW);
  if (b1 && !btn1Last) {
    outputOn = !outputOn;
    digitalWrite(PROTECTION_MOSFET_PIN, outputOn ? HIGH : LOW);
    delay(40);
  }
  btn1Last = b1;

  // BTN2 → clear protection latch
  bool b2 = (digitalRead(BTN2_PIN) == LOW);
  if (b2 && !btn2Last) {
    if (M.protect) setProtect(false);
    delay(40);
  }
  btn2Last = b2;

  // measurements every 200 ms
  if (now - tMeas >= 200) {
    tMeas    = now;
    M.uptime = (now - tStart) / 1000;
    M.voltage = ina226.getBusVoltage_V();
    M.current = ina226.getCurrent_mA() / 1000.0f;
    M.power   = ina226.getBusPower()   / 1000.0f;
    M.shunt   = ina226.getShuntVoltage_mV() / 1000.0f;
    M.tempC   = readTemp();
    if (M.current < 0) M.current = 0;
    if (M.power   < 0) M.power   = 0;
    M.energyWh += M.power * (0.2f / 3600.0f);

    // protection check
    if (!M.protect && (M.tempC >= TEMP_MAX_C || M.current >= CURRENT_MAX_A)) {
      setProtect(true);
      outputOn = false;
    }

    // history sample every 1 s
    if (now - hLast >= 1000) {
      hLast = now;
      hV[hHead] = M.voltage;
      hI[hHead] = M.current;
      hHead = (hHead + 1) % HIST;
    }

    // heartbeat LED
    static bool hl = false; hl = !hl;
    digitalWrite(PULSE_LED_PIN, hl);

    Serial.printf("[%lus] V=%.3f I=%.4f P=%.3f T=%.1f PWM=%d PROT=%d\n",
      M.uptime, M.voltage, M.current, M.power, M.tempC, pwmDuty, (int)M.protect);
  }

  // OLED refresh every 120 ms
  if (now - tOled >= 120) {
    tOled = now;
    switch (curPage) {
      case PAGE_MAIN:  drawMain();     break;
      case PAGE_GRAPH: drawGraph();    break;
      case PAGE_SET:   drawSettings(); break;
      default: break;
    }
  }

  // WS2812 breathing every 500 ms (when not in protection)
  if (now - tLed >= 500 && !M.protect) {
    tLed = now;
    uint8_t br = (uint8_t)(128 + 127.0f * sinf(ledPh++ * 0.12f));
    ws_set(0, 0, br / 4, br);
    ws_set(1, 0, br / 8, br / 2);
    ws_show();
  }
}
