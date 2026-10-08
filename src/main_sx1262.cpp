// ============================================================================
//  ODBIORNIK STACJI POGODOWEJ 868 MHz NA SX1262 (SPI) - ESP32-S2 Mini
// ============================================================================
//  Po co: CC1101 ma kwarc, ktory dryfuje (~100 kHz na dobe) i to byla glowna
//  przyczyna przerw w odbiorze. SX1262 z TCXO ma stabilnosc ~0,5 ppm, wiec
//  problem dryfu znika, a czulosc w FSK jest o ~10 dB lepsza.
//
//  PARAMETRY RADIOWE - ZMIERZONE z dzialajacego odbiornika CC1101 (2026-10-08):
//    modulacja     : 2-FSK
//    bitrate       : 11,11 kbaud   (CC1101 MDMCFG4=0x68, MDMCFG3=0xC0)
//    dewiacja      : +-38 kHz      (CC1101 DEVIATN=0x44)
//    filtr CC1101  : 271 kHz (celowo szeroki na dryf kwarcu)
//    sync word     : CA54 (w CC1101 wykrywany programowo -> tu sprzetowo)
//    pakiet        : 28 bajtow, STALA dlugosc, BEZ CRC, BEZ whiteningu, bez FEC
//    naglowek      : AA 00 F8 82, bajty 4-5 = 10 02, ostatnie 6 bajtow = nr seryjny
//
//  Dlaczego bitrate jest pewny: przy zlej predkosci bit przesunalby sie w ciagu
//  240 bitow ramki i dekoder nie zlozylby poprawnych 28 bajtow. Skoro sklada je
//  co do bitu, prawdziwa predkosc czujnika jest w granicach +-0,5% od 11,11 kbaud.
//
//  DEKODER jest ten sam plik co w wersji CC1101 (src/vevor.cpp) - nic sie nie
//  zmienia w interpretacji danych, wiec dane do stacji glownej sa identyczne.
//
//  Budowa:  pio run -e esp32-s2-sx1262
//  Wgranie: pio run -e esp32-s2-sx1262 -t upload
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <RadioLib.h>
#include "decoders.h"

// ==================== PINY (ESP32-S2 Mini) ====================
// Piny 7-13 sa bezpieczne na S2 Mini. NIE uzywac: GPIO0 (BOOT), GPIO19/20 (USB),
// GPIO26-32 (flash), GPIO33-37 (PSRAM), GPIO15 (dioda LED na wiekszosci plytek).
#define PIN_SCK    12
#define PIN_MISO   13
#define PIN_MOSI   11
#define PIN_NSS    10
#define PIN_RST     9
#define PIN_BUSY    8
#define PIN_DIO1    7

// ==================== PARAMETRY RADIA ====================
#define RF_FREQ_MHZ_DEF   868.350f   // srodek pasma (AFC w CC1101 zbiegal do 868,349-868,354)
#define RF_BITRATE        11.11f     // kbaud
#define RF_DEVIATION      38.0f      // kHz
#define RF_RXBW           125.0f     // kHz - 2*(dewiacja + bitrate/2) = ~87 kHz + margines
#define RF_SYNC           0xCA54     // 16 bitow synchronizacji
#define RF_PACKET_LEN     28         // STALA dlugosc - protokol VEVOR nie ma pola dlugosci
#define RF_PREAMBLE       16         // bity preambuly (RX nie wymaga dokladnej dlugosci)
#define RF_POWER_DBM      0          // tylko odbior, ale RadioLib wymaga wartosci

// Napięcie TCXO na DIO3. 0 = modul ze zwyklym kwarcem, 1.8 lub 3.3 = z TCXO.
// UWAGA do DX-LR30-900M22S: w specyfikacji jest "Crystal Oscillator Frequency: 32MHz",
// czyli ZWYKLY KWARC -> startujemy od 0. Gdyby radio nie odbieralo, sprobuj 1.8, potem 3.3
// (w /setup). Zla wartosc = radio nie wystartuje albo nie uslyszy nic.
// Dlatego (wlasnie z powodu kwarcu) firmware ma AFC na getFrequencyError().
#define RF_TCXO_V_DEF     0.0f

// ==================== AFC (korekta czestotliwosci) ====================
// Kwarc 32 MHz ma +-10 ppm, czyli +-8,7 kHz przy 868 MHz, a do tego dryfuje termicznie.
// SX1262 podaje BLAD CZESTOTLIWOSCI odebranej ramki w Hz (rozdzielczosc ~1 Hz, nie 1587 Hz
// jak FREQEST w CC1101), wiec srodek pasma mozna korygowac plynnie i dokladnie.
// Zasady przeniesione z doswiadczen z CC1101 (patrz dokumentacja sniffera):
//   - mediana z 5 probek chroni przed pojedynczym szumem,
//   - bramka MAD (mediana odchylen od mediany) - odrzuca okna, w ktorych pomiary sie nie zgadzaja,
//     bo rozrzut max-min zabil automat przy CC1101 (jeden odpad trzymal "rozrzut" 125 kHz),
//   - bezpiecznik: po AFC_SKIP_FAILOPEN odrzutach Z RZEDU korygujemy mimo wszystko,
//   - krok max 3 kHz i tlumienie 0,5 - dryf jest powolny, nie ma co skakac.
const int   AFC_HIST_N        = 5;
const float AFC_DAMPING       = 0.5f;
const float AFC_STEP_MAX_HZ   = 3000.0f;
const int   AFC_MAD_MAX_HZ    = 8000;
const int   AFC_SKIP_FAILOPEN = 10;

int      afcHist[AFC_HIST_N] = {0, 0, 0, 0, 0};
int      afcHistN  = 0;
int      afcHistIdx = 0;
int      afcMadHz  = 0;
int      afcSkipRow = 0;
uint32_t afcUpdates = 0;
uint32_t afcSkips   = 0;
unsigned long afcLastSaveMs = 0;

#define WDT_TIMEOUT_S     20
#define STALL_WARN_MS     360000UL   // 6 min bez ramki -> ponow odbior
#define STALL_REINIT_MS   900000UL   // 15 min bez ramki -> reinicjalizacja radia

// Domyślny numer seryjny NASZEGO czujnika (ostatnie 6 bajtow ramki).
// Zmieniasz w /setup, jesli wymienisz czujnik.
#define VEV_ID_DEF   "d86b2565d196"

// ==================== STAN ====================
SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY,
                          SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));

WebServer server(80);
Preferences prefs;

struct Cfg {
  String staSsid;
  String staPass;
  String ip, gw, mask;
  bool   dhcp = true;
  float  freqMhz = RF_FREQ_MHZ_DEF;
  float  tcxoV   = RF_TCXO_V_DEF;
  String vevId   = VEV_ID_DEF;   // 12 znakow hex
} cfg;

uint8_t  vevId[6]     = { 0xd8, 0x6b, 0x25, 0x65, 0xd1, 0x96 };
bool     vevIdSet     = false;

uint32_t framesTotal  = 0;
uint32_t framesOurs   = 0;
uint32_t framesOther  = 0;
uint32_t framesBroken = 0;
uint32_t radioInits   = 0;
uint32_t rxRestarts   = 0;

int      lastRssi     = 0;
int      lastFreqErr  = 0;      // Hz - odchyłka częstotliwości zmierzona przez SX1262
int      lastSnr      = 0;
unsigned long lastFrameMs = 0;
unsigned long startTime   = 0;
String   lastHex      = "";

WeatherData lastWeather;
bool        lastWeatherValid = false;

// Ostatnie surowe ramki (do diagnostyki i dostrajania) - bez String w petli.
#define RAW_KEEP 8
struct RawEntry { uint8_t data[RF_PACKET_LEN]; int rssi; int freqErr; bool ours; unsigned long t; };
RawEntry rawLog[RAW_KEEP];
int      rawIdx = 0;

// ==================== POMOCNICZE ====================
String bytesToHex(const uint8_t* b, int n) {
  static const char* h = "0123456789abcdef";
  String s;
  s.reserve(n * 2);
  for (int i = 0; i < n; i++) { s += h[b[i] >> 4]; s += h[b[i] & 0x0F]; }
  return s;
}

int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

void parseVevId(const String& s) {
  if (s.length() != 12) return;
  uint8_t tmp[6];
  for (int i = 0; i < 6; i++) {
    int hi = hexVal(s[i * 2]), lo = hexVal(s[i * 2 + 1]);
    if (hi < 0 || lo < 0) return;
    tmp[i] = (uint8_t)((hi << 4) | lo);
  }
  memcpy(vevId, tmp, 6);
  vevIdSet = true;
}

// ==================== USTAWIENIA (NVS) ====================
void loadCfg() {
  prefs.begin("sxcfg", true);
  cfg.staSsid = prefs.getString("ssid", "");
  cfg.staPass = prefs.getString("pass", "");
  cfg.dhcp    = prefs.getBool("dhcp", true);
  cfg.ip      = prefs.getString("ip", "");
  cfg.gw      = prefs.getString("gw", "");
  cfg.mask    = prefs.getString("mask", "255.255.255.0");
  cfg.freqMhz = prefs.getFloat("freq", RF_FREQ_MHZ_DEF);
  cfg.tcxoV   = prefs.getFloat("tcxo", RF_TCXO_V_DEF);
  cfg.vevId   = prefs.getString("vevid", VEV_ID_DEF);
  prefs.end();
  if (cfg.freqMhz < 850.0f || cfg.freqMhz > 950.0f) cfg.freqMhz = RF_FREQ_MHZ_DEF;
  parseVevId(cfg.vevId);
}

void saveCfg() {
  prefs.begin("sxcfg", false);
  prefs.putString("ssid", cfg.staSsid);
  prefs.putString("pass", cfg.staPass);
  prefs.putBool("dhcp", cfg.dhcp);
  prefs.putString("ip", cfg.ip);
  prefs.putString("gw", cfg.gw);
  prefs.putString("mask", cfg.mask);
  prefs.putFloat("freq", cfg.freqMhz);
  prefs.putFloat("tcxo", cfg.tcxoV);
  prefs.putString("vevid", cfg.vevId);
  prefs.end();
}

// ==================== RADIO ====================
bool radioInit() {
  Serial.print("[radio] beginFSK ");
  Serial.print(cfg.freqMhz, 4);
  Serial.print(" MHz, ");
  Serial.print(RF_BITRATE, 2);
  Serial.print(" kbaud, dev ");
  Serial.print(RF_DEVIATION, 1);
  Serial.print(" kHz, RXBW ");
  Serial.print(RF_RXBW, 1);
  Serial.print(" kHz, TCXO ");
  Serial.println(cfg.tcxoV, 2);

  int16_t st = radio.beginFSK(cfg.freqMhz, RF_BITRATE, RF_DEVIATION, RF_RXBW,
                              RF_POWER_DBM, RF_PREAMBLE, cfg.tcxoV, false);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("[radio] BLAD beginFSK: ");
    Serial.println(st);
    return false;
  }

  // Sync word CA54 - RadioLib przyjmuje tablice bajtow (dokladnie jak leci w powietrzu).
  uint8_t syncBytes[2] = { (uint8_t)(RF_SYNC >> 8), (uint8_t)(RF_SYNC & 0xFF) };
  radio.setSyncWord(syncBytes, 2);
  radio.setWhitening(false);
  radio.setCRC(0);
  radio.fixedPacketLengthMode(RF_PACKET_LEN);
  radio.setDio2AsRfSwitch(true);
  radio.setCurrentLimit(140.0);

  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("[radio] BLAD startReceive: ");
    Serial.println(st);
    return false;
  }
  radioInits++;
  Serial.println("[radio] nasluch FSK aktywny.");
  return true;
}

// ==================== AFC ====================
static void afcSort(int* a, int n) {
  for (int i = 0; i < n - 1; i++)
    for (int j = 0; j < n - 1 - i; j++)
      if (a[j] > a[j + 1]) { int t = a[j]; a[j] = a[j + 1]; a[j + 1] = t; }
}

void afcOnFrame(int errHz) {
  if (errHz < -200000 || errHz > 200000) return;      // wartosc bez sensu - ignoruj

  afcHist[afcHistIdx] = errHz;
  afcHistIdx = (afcHistIdx + 1) % AFC_HIST_N;
  if (afcHistN < AFC_HIST_N) { afcHistN++; afcSkips++; afcSkipRow++; return; }

  int tmp[AFC_HIST_N];
  for (int i = 0; i < AFC_HIST_N; i++) tmp[i] = afcHist[i];
  afcSort(tmp, AFC_HIST_N);
  int med = tmp[AFC_HIST_N / 2];

  int dev[AFC_HIST_N];
  for (int i = 0; i < AFC_HIST_N; i++) { int d = tmp[i] - med; dev[i] = (d < 0) ? -d : d; }
  afcSort(dev, AFC_HIST_N);
  afcMadHz = dev[AFC_HIST_N / 2];

  if (afcMadHz > AFC_MAD_MAX_HZ && afcSkipRow < AFC_SKIP_FAILOPEN) {
    afcSkips++; afcSkipRow++;
    return;                                            // pomiary niestabilne - stoimy
  }
  afcSkipRow = 0;

  float step = (float)med * AFC_DAMPING;
  if (step >  AFC_STEP_MAX_HZ) step =  AFC_STEP_MAX_HZ;
  if (step < -AFC_STEP_MAX_HZ) step = -AFC_STEP_MAX_HZ;
  if (step < 200.0f && step > -200.0f) return;          // < 200 Hz - nie warto ruszac

  float nf = cfg.freqMhz + step / 1e6f;
  if (nf < 850.0f || nf > 950.0f) return;

  cfg.freqMhz = nf;
  radio.setFrequency(cfg.freqMhz);
  radio.startReceive();
  afcUpdates++;

  if (millis() - afcLastSaveMs > 60000UL) {             // NVS nie czesciej niz raz na minute
    afcLastSaveMs = millis();
    saveCfg();
  }
}

// ==================== RAMKA ====================
void handleFrame(const uint8_t* f, int rssi, int freqErr, int snr) {
  framesTotal++;

  bool hdrOk = (f[0] == 0xAA && f[1] == 0x00 && f[2] == 0xF8 && f[3] == 0x82);
  bool ours  = hdrOk && (memcmp(f + 22, vevId, 6) == 0) && f[4] == 0x10 && f[5] == 0x02;

  RawEntry& e = rawLog[rawIdx];
  memcpy(e.data, f, RF_PACKET_LEN);
  e.rssi = rssi; e.freqErr = freqErr; e.ours = ours;
  e.t = millis();
  rawIdx = (rawIdx + 1) % RAW_KEEP;

  lastRssi = rssi; lastFreqErr = freqErr; lastSnr = snr;
  lastHex = bytesToHex(f, RF_PACKET_LEN);

  if (!hdrOk) { framesBroken++; return; }
  if (!ours)  { framesOther++;  return; }

  WeatherData w;
  if (!decodeVevor7in1(f, RF_PACKET_LEN, w)) { framesBroken++; return; }

  // Kontrola sensownosci - odrzucamy tylko to, co fizycznie niemozliwe.
  if (w.haveWind && (w.windAvgMs < 0.0f || w.windAvgMs > 60.0f)) { framesBroken++; return; }
  if (w.haveGust && (w.windMaxMs < 0.0f || w.windMaxMs > 80.0f)) { framesBroken++; return; }
  if (w.haveHum  && (w.humidity < 0 || w.humidity > 100))        { framesBroken++; return; }

  w.rssi = rssi;
  lastWeather = w;
  lastWeatherValid = true;
  lastFrameMs = millis();
  framesOurs++;

  afcOnFrame(freqErr);      // dostrojenie srodka pasma do czujnika (kwarc dryfuje)

  Serial.print("POGODA ");
  Serial.print(w.model);
  Serial.print(" id=");    Serial.print(w.id);
  Serial.print(" T=");     Serial.print(w.tempC, 1);
  Serial.print("C RH=");   Serial.print(w.humidity);
  Serial.print("% wiatr=");Serial.print(w.windAvgMs, 2);
  Serial.print(" kier=");  Serial.print(w.windDirDeg);
  Serial.print(" deszcz=");Serial.print(w.rainMm, 2);
  Serial.print(" lux=");   Serial.print(w.lightLux, 0);
  Serial.print(" rssi=");  Serial.print(rssi);
  Serial.print("dBm err=");Serial.print(freqErr);
  Serial.print("Hz snr="); Serial.print(snr);
  Serial.print("dB hex="); Serial.println(lastHex);
}

// ==================== WWW ====================
String pageHtml() {
  String h = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>SX1262 868 MHz</title><style>"
    "body{font-family:Consolas,monospace;background:#0d1117;color:#c9d1d9;margin:0;padding:14px}"
    "h1{color:#58a6ff;font-size:1.15em}table{border-collapse:collapse;margin:8px 0}"
    "td,th{border:1px solid #30363d;padding:5px 9px;text-align:left}"
    "a{color:#58a6ff;margin-right:12px}pre{background:#010409;border:1px solid #30363d;"
    "padding:10px;overflow-x:auto}.ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}"
    "</style></head><body>");
  h += F("<h1>SX1262 868 MHz - odbiornik stacji</h1>");
  h += F("<div><a href='/json'>JSON</a><a href='/status'>Status</a>"
         "<a href='/setup'>Ustawienia</a><a href='/reboot'>Restart</a></div>");

  h += F("<h3>Dane pogodowe</h3><table>");
  if (lastWeatherValid) {
    const WeatherData& w = lastWeather;
    h += "<tr><th>Model</th><td>" + w.model + " (id " + String(w.id) + ")</td></tr>";
    h += "<tr><th>Temperatura</th><td>" + String(w.tempC, 1) + " &deg;C</td></tr>";
    h += "<tr><th>Wilgotnosc</th><td>" + String(w.humidity) + " %</td></tr>";
    h += "<tr><th>Wiatr</th><td>" + String(w.windAvgMs, 2) + " m/s, poryw " +
         String(w.windMaxMs, 2) + " m/s, " + String(w.windDirDeg) + "&deg;</td></tr>";
    h += "<tr><th>Deszcz</th><td>" + String(w.rainMm, 2) + " mm</td></tr>";
    h += "<tr><th>UV / swiatlo</th><td>" + String(w.uv) + " / " + String(w.lightLux, 0) + " lux</td></tr>";
    h += "<tr><th>RSSI</th><td>" + String(w.rssi) + " dBm</td></tr>";
  } else {
    h += F("<tr><td>brak ramek - czekam na czujnik</td></tr>");
  }
  h += F("</table>");

  h += F("<h3>Radio</h3><table>");
  h += "<tr><th>Czestotliwosc</th><td>" + String(cfg.freqMhz, 4) + " MHz</td></tr>";
  h += "<tr><th>Bitrate / dewiacja</th><td>" + String(RF_BITRATE, 2) + " kbaud / &plusmn;" +
       String(RF_DEVIATION, 1) + " kHz</td></tr>";
  h += "<tr><th>Filtr / sync / pakiet</th><td>" + String(RF_RXBW, 0) + " kHz / 0x" +
       String(RF_SYNC, HEX) + " / " + String(RF_PACKET_LEN) + " B</td></tr>";
  h += "<tr><th>TCXO</th><td>" + String(cfg.tcxoV, 2) + " V</td></tr>";
  h += "<tr><th>Blad czestotliwosci</th><td>" + String(lastFreqErr) + " Hz</td></tr>";
  h += "<tr><th>AFC</th><td>" + String(afcUpdates) + " korekt, MAD " + String(afcMadHz) +
       " Hz, pominiec " + String(afcSkips) + "</td></tr>";
  h += "<tr><th>RSSI / SNR</th><td>" + String(lastRssi) + " dBm / " + String(lastSnr) + " dB</td></tr>";
  h += "</table>";

  h += F("<h3>Liczniki</h3><table>");
  h += "<tr><th>Wszystkie ramki</th><td>" + String(framesTotal) + "</td></tr>";
  h += "<tr><th>Nasze / obce</th><td><span class='ok'>" + String(framesOurs) +
       "</span> / <span class='warn'>" + String(framesOther) + "</span></td></tr>";
  h += "<tr><th>Uszkodzone (naglowek/dekod)</th><td>" + String(framesBroken) + "</td></tr>";
  h += "<tr><th>Inicjalizacje / resety odbioru</th><td>" + String(radioInits) + " / " +
       String(rxRestarts) + "</td></tr>";
  h += "<tr><th>Czas pracy</th><td>" + String((millis() - startTime) / 1000) + " s</td></tr>";
  h += "</table>";

  h += F("<h3>Ostatnie ramki (surowe)</h3><pre>");
  for (int i = 0; i < RAW_KEEP; i++) {
    int k = (rawIdx - 1 - i + RAW_KEEP * 2) % RAW_KEEP;
    if (!rawLog[k].t) continue;
    h += String(i == 0 ? ">" : " ");
    h += String((millis() - rawLog[k].t) / 1000) + "s ";
    h += String(rawLog[k].rssi) + "dBm err=" + String(rawLog[k].freqErr) + "Hz ";
    h += rawLog[k].ours ? "[NASZA] " : "[obca]  ";
    h += bytesToHex(rawLog[k].data, RF_PACKET_LEN) + "\n";
  }
  h += F("</pre></body></html>");
  return h;
}

void handleRoot()   { server.send(200, "text/html; charset=utf-8", pageHtml()); }

void handleStatus() {
  String j = "{";
  j += "\"uptime\":" + String((millis() - startTime) / 1000) + ",";
  j += "\"freqMhz\":" + String(cfg.freqMhz, 4) + ",";
  j += "\"bitrate\":" + String(RF_BITRATE, 2) + ",";
  j += "\"deviationKhz\":" + String(RF_DEVIATION, 1) + ",";
  j += "\"rxBwKhz\":" + String(RF_RXBW, 1) + ",";
  j += "\"syncWord\":\"0x" + String(RF_SYNC, HEX) + "\",";
  j += "\"packetLen\":" + String(RF_PACKET_LEN) + ",";
  j += "\"tcxoV\":" + String(cfg.tcxoV, 2) + ",";
  j += "\"rssi\":" + String(lastRssi) + ",";
  j += "\"snr\":" + String(lastSnr) + ",";
  j += "\"freqErrHz\":" + String(lastFreqErr) + ",";
  j += "\"afcUpdates\":" + String(afcUpdates) + ",";
  j += "\"afcMadHz\":" + String(afcMadHz) + ",";
  j += "\"afcSkips\":" + String(afcSkips) + ",";
  j += "\"frames\":" + String(framesTotal) + ",";
  j += "\"ours\":" + String(framesOurs) + ",";
  j += "\"other\":" + String(framesOther) + ",";
  j += "\"broken\":" + String(framesBroken) + ",";
  j += "\"radioInits\":" + String(radioInits) + ",";
  j += "\"rxRestarts\":" + String(rxRestarts) + ",";
  j += "\"lastFrameAgeSec\":" + String(lastFrameMs ? (millis() - lastFrameMs) / 1000 : -1) + ",";
  j += "\"wifiRssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) + ",";
  j += "\"wifiIp\":\"" + WiFi.localIP().toString() + "\",";
  j += "\"freeHeap\":" + String(ESP.getFreeHeap());
  j += "}";
  server.send(200, "application/json", j);
}

// To samo, co podawal sniffer na CC1101 - stacja glowna odpytuje ten adres.
void handleJson() {
  String j = "{\"freq\":\"" + String(cfg.freqMhz, 4) + "\",";
  j += "\"freqMhz\":" + String(cfg.freqMhz, 4) + ",";
  j += "\"afcOffsetHz\":" + String(lastFreqErr) + ",";
  j += "\"prof\":\"FSK_11k\",\"mode\":\"vevor\",";
  j += "\"totalFrames\":" + String(framesTotal) + ",";
  j += "\"acceptedFrames\":" + String(framesOurs) + ",";
  j += "\"rejectedFrames\":" + String(framesOther + framesBroken) + ",";
  j += "\"obceLacznie\":" + String(framesOther) + ",";
  j += "\"vevIdFilter\":true,";
  j += "\"uptime\":" + String((millis() - startTime) / 1000) + ",";
  if (lastWeatherValid) {
    const WeatherData& w = lastWeather;
    j += "\"weather\":{";
    j += "\"model\":\"" + w.model + "\",";
    j += "\"id\":" + String(w.id) + ",";
    j += "\"battery_ok\":" + String(w.batteryOk ? "true" : "false") + ",";
    j += "\"temperature_C\":" + String(w.tempC, 1) + ",";
    j += "\"humidity\":" + String(w.humidity) + ",";
    j += "\"wind_dir_deg\":" + String(w.windDirDeg) + ",";
    j += "\"wind_avg_m_s\":" + String(w.windAvgMs, 2) + ",";
    j += "\"wind_max_m_s\":" + String(w.windMaxMs, 2) + ",";
    j += "\"rain_mm\":" + String(w.rainMm, 2) + ",";
    j += "\"uv\":" + String(w.uv) + ",\"uvi\":" + String(w.uvi) + ",";
    j += "\"light_lux\":" + String(w.lightLux, 0) + ",";
    j += "\"rssi\":" + String(w.rssi) + ",";
    j += "\"hex\":\"" + lastHex + "\"";
    j += "},";
  }
  j += "\"lastRawRssi\":" + String(lastRssi) + ",";
  j += "\"lastRawHex\":\"" + lastHex + "\"";
  j += "}";
  server.send(200, "application/json", j);
}

void handleSetupGet() {
  String h = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Ustawienia</title><style>body{font-family:Consolas,monospace;background:#0d1117;"
    "color:#c9d1d9;padding:14px}input{background:#0d1117;color:#c9d1d9;border:1px solid #30363d;"
    "padding:5px;margin:3px 0;width:260px}button{background:#21262d;color:#c9d1d9;"
    "border:1px solid #30363d;padding:7px 14px}</style></head><body>"
    "<h1>Ustawienia</h1><form method='POST' action='/setup'>");
  h += "WiFi SSID<br><input name='ssid' value='" + cfg.staSsid + "'><br>";
  h += "WiFi haslo<br><input name='pass' type='password' value='" + cfg.staPass + "'><br>";
  h += "DHCP (1=tak)<br><input name='dhcp' value='" + String(cfg.dhcp ? 1 : 0) + "'><br>";
  h += "IP<br><input name='ip' value='" + cfg.ip + "'><br>";
  h += "Brama<br><input name='gw' value='" + cfg.gw + "'><br>";
  h += "Maska<br><input name='mask' value='" + cfg.mask + "'><br>";
  h += "Czestotliwosc [MHz]<br><input name='freq' value='" + String(cfg.freqMhz, 4) + "'><br>";
  h += "TCXO [V] (0 = kwarc)<br><input name='tcxo' value='" + String(cfg.tcxoV, 2) + "'><br>";
  h += "Nr seryjny czujnika (12 hex)<br><input name='vevid' value='" + cfg.vevId + "'><br>";
  h += F("<button type='submit'>Zapisz i zrestartuj</button></form>"
         "<p>Po zapisie urzadzenie restartuje sie i laczy z nowa siecia.</p>"
         "</body></html>");
  server.send(200, "text/html; charset=utf-8", h);
}

void handleSetupPost() {
  if (server.hasArg("ssid")) cfg.staSsid = server.arg("ssid");
  if (server.hasArg("pass")) cfg.staPass = server.arg("pass");
  if (server.hasArg("dhcp")) cfg.dhcp    = server.arg("dhcp").toInt() != 0;
  if (server.hasArg("ip"))   cfg.ip      = server.arg("ip");
  if (server.hasArg("gw"))   cfg.gw      = server.arg("gw");
  if (server.hasArg("mask")) cfg.mask    = server.arg("mask");
  if (server.hasArg("freq")) cfg.freqMhz = server.arg("freq").toFloat();
  if (server.hasArg("tcxo")) cfg.tcxoV   = server.arg("tcxo").toFloat();
  if (server.hasArg("vevid")) cfg.vevId  = server.arg("vevid");
  if (cfg.freqMhz < 850.0f || cfg.freqMhz > 950.0f) cfg.freqMhz = RF_FREQ_MHZ_DEF;
  saveCfg();
  server.send(200, "text/plain; charset=utf-8", "Zapisane. Restart...");
  delay(400);
  ESP.restart();
}

void handleReboot() {
  server.send(200, "text/plain; charset=utf-8", "Restart...");
  delay(300);
  ESP.restart();
}

// ==================== WIFI ====================
void wifiConnect() {
  if (cfg.staSsid.length() == 0) {
    Serial.println("[wifi] brak SSID - tryb AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("WeatherSniffer-SX", "sniffer123");
    Serial.print("[wifi] AP IP: ");
    Serial.println(WiFi.softAPIP());
    return;
  }
  if (!cfg.dhcp) {
    IPAddress ip, gw, mask;
    if (ip.fromString(cfg.ip) && gw.fromString(cfg.gw) && mask.fromString(cfg.mask))
      WiFi.config(ip, gw, mask);
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // uspienie modemu gubi pakiety i opoznia HTTP
  WiFi.begin(cfg.staSsid.c_str(), cfg.staPass.c_str());
  Serial.print("[wifi] lacze z ");
  Serial.println(cfg.staSsid);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[wifi] OK, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[wifi] nie udalo sie - tryb AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("WeatherSniffer-SX", "sniffer123");
  }
}

// ==================== SETUP / LOOP ====================
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("=== Odbiornik 868 MHz na SX1262 (ESP32-S2 Mini) ===");

  startTime = millis();
  loadCfg();

  // Radio: wlasna konfiguracja pinow SPI (ESP32-S2 pozwala routowac SPI na dowolne GPIO)
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  wifiConnect();

  if (!radioInit()) {
    Serial.println("[radio] nie wystartowal - sprawdz TCXO (0 / 1.8 / 3.3 V) w /setup");
  }

  // Watchdog zadania - tak samo jak w wersji na CC1101 (rozne API dla core 2.x/3.x)
  esp_err_t werr = ESP_FAIL;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
  esp_task_wdt_config_t wcfg = {};
  wcfg.timeout_ms     = WDT_TIMEOUT_S * 1000;
  wcfg.idle_core_mask = 0;
  wcfg.trigger_panic  = true;
  werr = esp_task_wdt_reconfigure(&wcfg);
  if (werr != ESP_OK) werr = esp_task_wdt_init(&wcfg);
#else
  werr = esp_task_wdt_init(WDT_TIMEOUT_S, true);
  if (werr == ESP_ERR_INVALID_STATE) werr = ESP_OK;
#endif
  if (werr == ESP_OK) esp_task_wdt_add(NULL);

  server.on("/",        handleRoot);
  server.on("/json",    handleJson);
  server.on("/status",  handleStatus);
  server.on("/setup",   HTTP_GET,  handleSetupGet);
  server.on("/setup",   HTTP_POST, handleSetupPost);
  server.on("/reboot",  handleReboot);
  server.begin();
  Serial.println("[www] serwer na porcie 80");
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();

  if (radio.available()) {
    uint8_t buf[RF_PACKET_LEN];
    size_t len = radio.getPacketLength();
    int st = radio.readData(buf, RF_PACKET_LEN);
    if (st == RADIOLIB_ERR_NONE && len == RF_PACKET_LEN) {
      handleFrame(buf, (int)radio.getRSSI(), (int)radio.getFrequencyError(), (int)radio.getSNR());
    } else {
      framesBroken++;
      Serial.print("[radio] odczyt ramki, kod=");
      Serial.println(st);
    }
    radio.startReceive();   // wroc do nasluchu
  }

  // Prosty nadzor odbioru: gdy dlugo nie ma ZADNEJ ramki, ponow nasluch.
  static unsigned long lastWarn = 0, lastReinit = 0;
  unsigned long age = lastFrameMs ? (millis() - lastFrameMs) : (millis() - startTime);
  if (age > STALL_WARN_MS && millis() - lastWarn > STALL_WARN_MS) {
    lastWarn = millis();
    rxRestarts++;
    radio.startReceive();
    Serial.print("[radio] brak ramek ");
    Serial.print(age / 1000);
    Serial.println(" s - ponawiam nasluch");
  }
  if (age > STALL_REINIT_MS && millis() - lastReinit > STALL_REINIT_MS) {
    lastReinit = millis();
    Serial.println("[radio] dlugi brak ramek - reinicjalizacja radia");
    radioInit();
  }

  delay(2);
}
