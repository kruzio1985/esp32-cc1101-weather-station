/*
  868 MHz Weather Station Raw Sniffer - ESP32-S3 + CC1101

  Uniwersalny odbiornik surowych ramek RF do rozpracowania protokołu
  stacji pogodowej (YT60309 / Vevor, 868 MHz).

  Skanuje kilka częstotliwości pasma 868 MHz w wielu konfiguracjach
  modemu (2-FSK, GFSK, ASK/OOK; różne przepływności). Zapisuje KAŻDĄ
  odebraną ramkę (niezależnie od tego, czy da się ją sparsować) do
  bufora kołowego i pokazuje na stronie WWW oraz na porcie szeregowym.

  To narzędzie służy do przechwycenia surowego strumienia bitów, żeby
  później rozpracować strukturę ramek stacji pogodowej.
*/

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "decoders.h"

// ==================== KONFIGURACJA ====================
// Domyślne parametry punktu dostępowego (AP) urządzenia. Gdy w pamięci NVS
// nie ma jeszcze zapisanych ustawień, sniffer startuje jako AP o tych danych,
// więc ZAWSZE można się do niego podłączyć i skonfigurować sieć.
const char* AP_SSID_DEFAULT = "WeatherSniffer";
const char* AP_PASS_DEFAULT = "sniffer123";

// Domyślne ustawienia sieci klienckiej (STA). Puste SSID = praca tylko w AP.
// UWAGA: hasło/SSID do Twojej sieci ustawiasz przez panel WWW (zapis w NVS),
// nie wpisuj ich tutaj — ten plik może trafić na publiczne repozytorium.
const char* DEF_STA_SSID = "";
const char* DEF_STA_PASS = "";
const bool  DEF_STA_DHCP = false;
const char* DEF_STA_IP   = "192.168.1.130";
const char* DEF_STA_GW   = "192.168.1.1";
const char* DEF_STA_MASK = "255.255.255.0";
const char* DEF_STA_DNS  = "192.168.1.1";

// Bieżące ustawienia sieciowe (ładowane/zapisywane w NVS)
struct WifiCfg {
  String apSsid = AP_SSID_DEFAULT;
  String apPass = AP_PASS_DEFAULT;
  String staSsid = DEF_STA_SSID;
  String staPass = DEF_STA_PASS;
  bool   useDhcp = DEF_STA_DHCP;
  String ip   = DEF_STA_IP;
  String gw   = DEF_STA_GW;
  String mask = DEF_STA_MASK;
  String dns  = DEF_STA_DNS;
};
WifiCfg wifiCfg;

// Filtr ramek: radio odbiera nadal WSZYSTKO, ale gdy filtr jest włączony,
// do bufora (strona WWW / JSON / później RS485) trafiają tylko ramki pasujące
// do wzorca (np. identyfikator urządzenia stacji pogody). Reszta jest tylko
// zliczana i pokazywana na porcie szeregowym.
struct FilterCfg {
  bool   enabled = false;
  String pattern = "";   // hex bez spacji, np. "a5b1c2"
  int    offset  = -1;   // -1 = szukaj gdziekolwiek; >=0 = od tego bajtu
  int    minLen  = 0;    // 0 = bez limitu
  int    maxLen  = 0;    // 0 = bez limitu
};
FilterCfg filterCfg;

// Tryb pracy odbiornika:
//  RAW_SCAN      - skanowanie 6 częst. x 8 profili, zapis surowych ramek
//                  (do rozpracowywania nieznanych protokołów).
//  FINE_OFFSET   - nasłuch na 868.30 MHz / 2-FSK 17.24k / sync 0x2DD4
//                  i dekodowanie stacji Fine Offset WH65 (VEVOR YT60309).
//  VEVOR_7IN1    - nasłuch na 868.35 MHz / 2-FSK 11.11k / sync 0xCA54
//                  i dekodowanie stacji VEVOR/Youtong 7-in-1 (YT60231/234).
//  VEVOR_YT60309 - nasłuch na 868.35 MHz / 2-FSK 11.11k / sync 0xC0AA C0AA
//                  i dekodowanie stacji VEVOR YT60309 (CMT2119A, 32 bajty).
//  BRESSER       - nasłuch na 868.30 MHz / 2-FSK 8.21k / sync 0xAA2D
//                  i dekodowanie stacji Bresser 5-in-1 / 6-in-1 / 7-in-1.
enum RxMode { MODE_RAW_SCAN = 0, MODE_FINE_OFFSET = 1, MODE_VEVOR_7IN1 = 2, MODE_BRESSER = 3, MODE_VEVOR_YT60309 = 4, MODE_WEATHER_AUTO = 5 };
RxMode rxMode = MODE_RAW_SCAN;

// Tryb sondy (diagnostyka): blokuje radio na jednej częstotliwości i mierzy
// RSSI w pętli. -1 = wyłączony, 0..5 = indeks do FREQ_TABLE, 7 = wartość z probeMhz.
int probeFreqIdx = -1;
double probeMhz = 0.0;
bool probeWide = false;   // true = szerokie pasmo 812 kHz (przemiatanie calego zakresu)

// Ostatnie zdekodowane dane pogodowe (pokazywane na stronie / JSON).
WeatherData lastWeather;       // surowe wartości z dekodera
WeatherData lastWeatherCal;    // po kalibracji (do wyświetlania i wysyłki)
bool lastWeatherValid = false;
String lastDecodedHex = "";
float rainBaseline = -1.0f;    // punkt odniesienia opadu (pierwsza wartość po starcie)

// ==================== KALIBRACJA CZUJNIKÓW ====================
// Wartości surowe z dekodera są przeliczane: y = x*factor + offset.
struct CalCfg {
  float tempOffset   = 0.0f;   // dodawane do temperatury [°C]
  float humOffset    = 0.0f;   // dodawane do wilgotności [%]
  float windFactor   = 1.0f;   // mnożnik prędkości wiatru
  float gustFactor   = 1.0f;   // mnożnik porywów
  float rainFactor   = 1.0f;   // mnożnik opadu (rozmiar przetwornika)
  float lightFactor  = 1.0f;   // mnożnik natężenia światła
  int   windDirOffset = 0;     // przesunięcie kierunku wiatru [°] (0..359)
};
CalCfg calCfg;

// ==================== WYSYŁANIE (WiFi / RS485) ====================
struct SendCfg {
  bool   wifiEnabled = false;
  String targetUrl   = "";     // np. http://192.168.1.50:8080/weather
  bool   rs485Enabled = false;
  int    rs485TxPin   = 17;    // UART1 TX
  int    rs485RxPin   = 18;    // UART1 RX
  int    rs485Baud    = 9600;
  int    rs485DePin   = -1;    // pin DE/RE transceivera MAX485 (opcjonalnie)
};
SendCfg sendCfg;

// ==================== MQTT (Home Assistant) ====================
struct MqttCfg {
  bool   enabled = false;
  String broker  = "";         // np. 192.168.1.10
  int    port    = 1883;
  String user    = "";
  String pass    = "";
  String topicPrefix = "weathersniffer";
  bool   haDiscovery = false;  // publikuj konfigurację auto-odkrycia HA
};
MqttCfg mqttCfg;

WiFiClient mqttWifi;
PubSubClient mqtt(mqttWifi);
String mqttClientId = "";

// Deklaracja wprzód - definicja niżej (potrzebna w loopFineOffset()).
String bytesToHex(uint8_t* data, int len);
int calculateRSSI(uint8_t raw);
WeatherData calibrateWeather(const WeatherData& in);
void publishWeather();

// ==================== PINY CC1101 ====================
#define PIN_CS   10
#define PIN_SCK  12
#define PIN_MISO 13
#define PIN_MOSI 11
#define PIN_GDO0 5
#define PIN_GDO2 6

// ==================== PARAMETRY SKANOWANIA ====================
const unsigned long COMBO_DWELL_MS = 1000;  // czas na kombinację (freq x profil)
const int RSSI_THRESHOLD   = -95;            // poniżej = szum, nie zapisuj
const int MIN_FRAME_LEN    = 2;              // minimalna liczba bajtów ramki
const int MAX_FRAMES       = 128;            // rozmiar bufora kołowego
const int MAX_SHOW_FRAMES  = 20;             // ile ostatnich ramek pokazać na WWW
const int MAX_DEVICES      = 20;             // ile różnych urządzeń śledzić na WWW
const int DEVICE_SIG_BYTES = 6;              // sygnatura urządzenia = pierwsze bajty ramki
const int MAX_HEX_BYTES    = 64;             // max bajtów hex w ramce

// ==================== CZĘSTOTLIWOŚCI (pasmo 868 MHz) ====================
// FREQ word = round(f_hz * 2^16 / 26 MHz)
static const char* FREQ_NAMES[] = {
  "868.95", "868.30", "868.35", "868.25", "868.50", "868.00"
};
static const uint8_t FREQ_TABLE[][3] = {
  {0x21, 0x6B, 0xD1},  // 868.95 MHz
  {0x21, 0x65, 0x6A},  // 868.30 MHz
  {0x21, 0x65, 0xE8},  // 868.35 MHz
  {0x21, 0x64, 0xEC},  // 868.25 MHz
  {0x21, 0x67, 0x62},  // 868.50 MHz
  {0x21, 0x62, 0x76},  // 868.00 MHz
};
const int NUM_FREQS = sizeof(FREQ_TABLE) / sizeof(FREQ_TABLE[0]);

// ==================== PROFILE MODEMU ====================
// Kolumny: MDMCFG4, MDMCFG3, MDMCFG2 (MOD_FORMAT, SYNC_MODE=0), DEVIATN
static const char* PROF_NAMES[] = {
  "FSK_100k", "FSK_50k", "FSK_17k", "FSK_9k6",
  "GFSK_50k", "GFSK_17k", "OOK_9k6", "OOK_4k8"
};
static const uint8_t PROFILES[][4] = {
  {0x5B, 0xF8, 0x00, 0x50},  // FSK_100k  rate=99.98k dev=50.8k BW=325k
  {0x7A, 0xF8, 0x00, 0x50},  // FSK_50k   rate=49.99k dev=50.8k BW=232k
  {0xB9, 0x5C, 0x00, 0x40},  // FSK_17k   rate=17.26k dev=25.4k BW=116k
  {0xF8, 0x83, 0x00, 0x34},  // FSK_9k6   rate=9.60k  dev=19.0k BW=58k
  {0x7A, 0xF8, 0x10, 0x40},  // GFSK_50k  rate=49.99k dev=25.4k BW=232k
  {0xB9, 0x5C, 0x10, 0x40},  // GFSK_17k  rate=17.26k dev=25.4k BW=116k
  {0x08, 0x83, 0x30, 0x00},  // OOK_9k6   rate=9.60k  BW=812k
  {0x37, 0x83, 0x30, 0x00},  // OOK_4k8   rate=4.80k  BW=464k
};
const int NUM_PROFS = sizeof(PROFILES) / sizeof(PROFILES[0]);

// ==================== STAN SKANERA ====================
int currentFreq = 0;
int currentProf = 0;
unsigned long lastComboChange = 0;

// Domyślna szyna SPI (jak w oryginalnym firmware RadioControl).
// CS (PIN_CS) jest sterowany ręcznie - nie przekazujemy go do SPI.begin(),
// bo sterownik wtedy sam przełącza ten pin i dochodzi do konfliktu.
SPIClass& radioSPI = SPI;
WebServer server(80);

// ==================== BUFOR RAMKOWY ====================
struct RawFrame {
  unsigned long t;    // millis() odbioru
  uint8_t freq;       // indeks częstotliwości
  uint8_t prof;       // indeks profilu
  int rssi;           // dBm
  int lqi;            // jakość łącza
  uint8_t len;        // długość ramki
  String hex;         // pełny hex
};

RawFrame frames[MAX_FRAMES];
int frameHead = 0;    // indeks następnego zapisu
int frameCount = 0;   // ile ramek w buforze

// ==================== DEDUPLIKACJA URZĄDZEŃ (krótki widok WWW) ====================
// Zamiast setek ramek od tych samych liczników, grupujemy po sygnaturze
// (pierwsze bajty ramki) i pokazujemy tylko MAX_DEVICES różnych nadajników.
struct DeviceEntry {
  String sig;          // hex sygnatury (pierwsze DEVICE_SIG_BYTES bajtów)
  unsigned long lastSeen;  // millis() ostatniego odbioru
  int count;               // ile razy odebrano
  int lastRssi;            // ostatni RSSI [dBm]
  uint8_t freq;            // indeks częstotliwości
  uint8_t prof;            // indeks profilu
  uint8_t len;             // długość ostatniej ramki
  String lastHex;          // ostatnia ramka (hex)
};
DeviceEntry devices[MAX_DEVICES];
int deviceCount = 0;

// Diagnostyka dekodera: ostatnie pakiety z dopasowanym sync word (także te
// odrzucone przez sumę kontrolną). Pokazywane na stronie w trybach dekodera,
// żeby było widać, czy cokolwiek dociera z eteru.
struct DiagFrame {
  unsigned long t;
  int rssi;
  uint8_t len;
  bool ok;
  String hex;
};
const int MAX_DIAG = 8;
DiagFrame diagFrames[MAX_DIAG];
int diagHead = 0;
int diagCount = 0;
unsigned long diagTotal = 0;   // pakiety z poprawnym sync word w trybie dekodera
String lastRawHex = "";        // ostatni pakiet z sync (nawet odrzucony)
int lastRawRssi = 0;

void diagPush(const uint8_t* data, int len, int rssi, bool ok) {
  DiagFrame& d = diagFrames[diagHead];
  d.t = millis();
  d.rssi = rssi;
  d.len = (uint8_t)len;
  d.ok = ok;
  d.hex = bytesToHex((uint8_t*)data, len);
  diagHead = (diagHead + 1) % MAX_DIAG;
  if (diagCount < MAX_DIAG) diagCount++;
  diagTotal++;
  lastRawHex = d.hex;
  lastRawRssi = rssi;
}

unsigned long totalFrames = 0;    // wszystkie ramki powyżej progu RSSI
unsigned long acceptedFrames = 0; // zapisane do bufora (po filtrze)
unsigned long rejectedFrames = 0; // odrzucone przez filtr
unsigned long startTime = 0;
int comboHits[6][8];  // licznik trafień per (freq, prof)

// ==================== CC1101 FUNKCJE ====================
// Prędkość SPI dobrana pod ten moduł (oryginalny firmware RadioControl
// używał domyślnego, wolnego SPI ~1 MHz; 4 MHz powodowało niestabilne odczyty).
#define SPI_HZ 1000000

inline void csLow()  { digitalWrite(PIN_CS, LOW); }
inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

uint8_t ccXfer(uint8_t b) {
  return radioSPI.transfer(b);
}

void ccWrite(uint8_t reg, uint8_t value) {
  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(reg);
  ccXfer(value);
  radioSPI.endTransaction();
  csHigh();
}

// Nadpisanie częstotliwości dla trybu AUTO (przemiatanie pasma 868 MHz).
bool freqOverride = false;
uint8_t freqOvr[3] = {0, 0, 0};

void ccWriteFreq(uint8_t d, uint8_t e, uint8_t f) {
  if (freqOverride) { d = freqOvr[0]; e = freqOvr[1]; f = freqOvr[2]; }
  ccWrite(0x0D, d);  // FREQ2
  ccWrite(0x0E, e);  // FREQ1
  ccWrite(0x0F, f);  // FREQ0
}

uint8_t ccRead(uint8_t reg) {
  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(reg | 0x80);
  uint8_t v = ccXfer(0);
  radioSPI.endTransaction();
  csHigh();
  return v;
}

uint8_t ccStatusRead(uint8_t reg) {
  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(reg | 0xC0);   // rejestry statusu: odczyt z bitem burst
  uint8_t v = ccXfer(0);
  radioSPI.endTransaction();
  csHigh();
  return v;
}

void ccStrobe(uint8_t cmd) {
  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(cmd);
  radioSPI.endTransaction();
  csHigh();
}

void ccReset() {
  csHigh();
  delayMicroseconds(5);
  csLow();
  delayMicroseconds(10);
  csHigh();
  delayMicroseconds(45);

  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0x30); // SRES
  radioSPI.endTransaction();
  csHigh();
  delay(10);
}

// ==================== INIT CC1101 (część wspólna) ====================
void ccInitBase() {
  ccReset();

  // GDO0 = sygnał synchronizacji pakietu (jak w oryginalnym firmware RadioControl)
  ccWrite(0x00, 0x06);  // IOCFG2
  ccWrite(0x02, 0x06);  // IOCFG0

  // Pętle kalibracyjne / AGC / front-end - wartości z działającego firmware
  // RadioControl dla tego konkretnego modułu (CC1101 + ESP32-S3).
  ccWrite(0x18, 0x18);  // MCSM0
  ccWrite(0x19, 0x17);  // FOCCFG
  ccWrite(0x1A, 0x6C);  // BSCFG
  ccWrite(0x1B, 0x03);  // AGCCTRL2: maksymalne wzmocnienie LNA (najlepsza czułość)
  ccWrite(0x1C, 0x40);  // AGCCTRL1
  ccWrite(0x1D, 0x91);  // AGCCTRL0
  ccWrite(0x21, 0x56);  // FREND1 (RX front-end)
  ccWrite(0x22, 0x10);  // FREND0
  ccWrite(0x23, 0xE9);  // FSCAL3
  ccWrite(0x24, 0x2A);  // FSCAL2
  ccWrite(0x25, 0x00);  // FSCAL1
  ccWrite(0x26, 0x1F);  // FSCAL0

  // MCSM1: po odebraniu pakietu zostań w RX (RXOFF_MODE=11). Bez tego radio
  // przechodzi w IDLE i przestaje odbierać kolejne transmisje.
  ccWrite(0x17, 0x3C);  // MCSM1

  // Konfiguracja pakietu: stała długość, bez CRC, bez whiteningu
  ccWrite(0x06, 0xFF);  // PKTLEN = 255
  ccWrite(0x07, 0x00);  // PKTCTRL1 (bez APPEND_STATUS)
  ccWrite(0x08, 0x00);  // PKTCTRL0 (fixed, normal, bez CRC/whitening)
}

// ==================== USTAWIENIE KOMBINACJI (freq + profil) ====================
void applyCombo(int f, int p) {
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX (flush RX)

  ccWrite(0x0D, FREQ_TABLE[f][0]);  // FREQ2
  ccWrite(0x0E, FREQ_TABLE[f][1]);  // FREQ1
  ccWrite(0x0F, FREQ_TABLE[f][2]);  // FREQ0

  ccWrite(0x10, PROFILES[p][0]);  // MDMCFG4 (BW + DRATE_E)
  ccWrite(0x11, PROFILES[p][1]);  // MDMCFG3 (DRATE_M)
  ccWrite(0x12, PROFILES[p][2] & 0xF0);  // MDMCFG2 (tylko MOD_FORMAT, SYNC_MODE=0)
  ccWrite(0x13, 0x22);  // MDMCFG1 (FEC off, 4 bajty preambuły)
  ccWrite(0x14, 0xF8);  // MDMCFG0 (channel spacing - nieużywane przy RX)
  ccWrite(0x15, PROFILES[p][3]);  // DEVIATN
  ccWrite(0x06, 0xFF);  // PKTLEN = 255 (raw capture, bez limitu długości)

  delay(3);
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX
}

// ==================== TRYB FINE OFFSET (VEVOR / WH65) ====================
// Lock na 868.30 MHz, 2-FSK, 17.24 kbit/s, hardware sync word 0x2D 0xD4,
// stała długość pakietu 17 bajtów (payload po sync).
void applyFineOffsetMode() {
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX

  ccWriteFreq(0x21, 0x65, 0x6A);  // 868.30 MHz (domyślnie)
  ccWrite(0x10, 0xB9);  // MDMCFG4: rate 17.26k, BW 116 kHz
  ccWrite(0x11, 0x5C);  // MDMCFG3
  ccWrite(0x12, 0x02);  // MDMCFG2: 2-FSK, SYNC_MODE=2 (16-bit sync)
  ccWrite(0x13, 0x22);  // MDMCFG1: FEC off, 4 bajty preambuły
  ccWrite(0x14, 0xF8);  // MDMCFG0
  ccWrite(0x15, 0x40);  // DEVIATN ~25.4 kHz (do zweryfikowania; tolerancja OK)
  ccWrite(0x04, 0x2D);  // SYNC1 = 0x2D
  ccWrite(0x05, 0xD4);  // SYNC0 = 0xD4

  ccWrite(0x06, 0x11);  // PKTLEN = 17 (fixed length)
  ccWrite(0x07, 0x00);  // PKTCTRL1 (bez adresu, bez append status)
  ccWrite(0x08, 0x00);  // PKTCTRL0 (fixed, bez CRC/whitening)

  delay(3);
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX
}

// ==================== TRYB VEVOR / YOUTONG 7-in-1 ====================
// 868.30 MHz, 2-FSK, ~11.11 kbit/s. Odbior BEZ hardware sync: hardware sync
// na tym sygnale nie lapie (stacja ma nietypowa preambule), wiec dekoder
// sam szuka wzorca "CA 54" w strumieniu surowych bajtow (metoda potwierdzona
// snifferem).
void applyVevorMode() {
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX

  ccWriteFreq(0x21, 0x65, 0x6A);  // 868.30 MHz (Twoja stacja)

  ccWrite(0x10, 0x98);  // MDMCFG4: rate ~11.11k, BW 162.5 kHz
  ccWrite(0x11, 0xC0);  // MDMCFG3: DRATE_M=192 -> ~11.11 kbaud
  ccWrite(0x12, 0x00);  // MDMCFG2: 2-FSK, bez sync (software sync)
  ccWrite(0x13, 0x22);  // MDMCFG1
  ccWrite(0x14, 0xF8);  // MDMCFG0
  ccWrite(0x15, 0x44);  // DEVIATN ~38 kHz (jak w snifferze, ktory dekodowal)
  ccWrite(0x06, 0xFF);  // PKTLEN = 255
  ccWrite(0x07, 0x00);  // PKTCTRL1
  ccWrite(0x08, 0x02);  // PKTCTRL0: nieskonczona dlugosc

  delay(3);
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX
}

// ==================== TRYB VEVOR YT60309 ====================
// Lock na 868.35 MHz, 2-FSK, ~11.11 kbit/s, sync 0xC0AA C0AA (32 bity),
// stała długość pakietu 32 bajty. Nadajnik: CMT2119A.
// Parametry wg github.com/FPR36/Vevor-Meteo-station-rf-protocol.
void applyVevorYT60309Mode() {
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX

  ccWriteFreq(0x21, 0x65, 0xE8);  // 868.35 MHz (domyślnie)

  ccWrite(0x10, 0x98);  // MDMCFG4: rate ~11.11k (DRATE_E=8), BW 162.5 kHz
  ccWrite(0x11, 0xC0);  // MDMCFG3: DRATE_M=192 -> ~11.11 kbaud
  ccWrite(0x12, 0x03);  // MDMCFG2: 2-FSK, SYNC_MODE=3 (30/32-bit sync)
  ccWrite(0x13, 0x02);  // MDMCFG1: FEC off, NUM_PREAMBLE=2 bajty (lagodniej dla slabego sygnalu)
  ccWrite(0x14, 0xF8);  // MDMCFG0
  ccWrite(0x15, 0x44);  // DEVIATN ~38 kHz (E=4, M=4)
  ccWrite(0x04, 0xC0);  // SYNC1 = 0xC0 (sync C0AA C0AA -> 32 bity)
  ccWrite(0x05, 0xAA);  // SYNC0 = 0xAA

  ccWrite(0x06, 0x20);  // PKTLEN = 32 (fixed length)
  ccWrite(0x07, 0x00);  // PKTCTRL1 (bez adresu, bez append status)
  ccWrite(0x08, 0x00);  // PKTCTRL0 (fixed, bez CRC/whitening)

  delay(3);
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX
}

// ==================== TRYB BRESSER 5/6/7-in-1 ====================
// 868.30 MHz, 2-FSK, ~8.21 kbit/s, dev ~57 kHz, BW ~203 kHz,
// hardware sync word 0xAA 0x2D, stała długość pakietu 27 bajtów.
// Pierwszy bajt payloadu (0xD4) to ostatni bajt sync/preambuły; dekodery
// dostają dane PO nim (5-in-1: 26 B, 6-in-1: 18 B, 7-in-1: 25 B).
// Parametry RF wg matthias-bs/BresserWeatherSensorReceiver (CC1101).
void applyBresserMode() {
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX

  ccWriteFreq(0x21, 0x65, 0x6A);  // 868.30 MHz (domyślnie)

  ccWrite(0x10, 0x88);  // MDMCFG4: rate 8.21k, BW 203 kHz
  ccWrite(0x11, 0x4B);  // MDMCFG3: DRATE_M=75
  ccWrite(0x12, 0x02);  // MDMCFG2: 2-FSK, SYNC_MODE=2 (16-bit sync)
  ccWrite(0x13, 0x22);  // MDMCFG1: FEC off, 4 bajty preambuły
  ccWrite(0x14, 0xF8);  // MDMCFG0
  ccWrite(0x15, 0x51);  // DEVIATN ~57.1 kHz (E=5, M=1)
  ccWrite(0x04, 0xAA);  // SYNC1 = 0xAA
  ccWrite(0x05, 0x2D);  // SYNC0 = 0x2D

  ccWrite(0x06, 0x1B);  // PKTLEN = 27 (fixed length)
  ccWrite(0x07, 0x00);  // PKTCTRL1 (bez adresu, bez append status)
  ccWrite(0x08, 0x00);  // PKTCTRL0 (fixed, bez CRC/whitening)

  delay(3);
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX
}

void nextCombo() {
  currentProf++;
  if (currentProf >= NUM_PROFS) {
    currentProf = 0;
    currentFreq++;
    if (currentFreq >= NUM_FREQS) {
      currentFreq = 0;
    }
  }
  applyCombo(currentFreq, currentProf);
  lastComboChange = millis();
  Serial.print(">>> ");
  Serial.print(FREQ_NAMES[currentFreq]);
  Serial.print(" MHz / ");
  Serial.println(PROF_NAMES[currentProf]);
}

// ==================== PĘTLA TRYBU FINE OFFSET ====================
void loopFineOffset() {
  uint8_t rxBytes = ccStatusRead(0x3B) & 0x7F;  // RXBYTES
  // Czekaj na kompletny pakiet (17 bajtów po sync word). Czytanie częściowe
  // mogłoby urwać ramkę w trakcie odbioru i zgubić dekodowanie.
  if (rxBytes < 17) {
    delay(1);
    return;
  }

  uint8_t frame[17];
  memset(frame, 0, sizeof(frame));

  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0xFF);  // burst read FIFO
  for (int i = 0; i < 17; i++) {
    frame[i] = ccXfer(0);
  }
  radioSPI.endTransaction();
  csHigh();

  int rssiRaw = ccStatusRead(0x34);  // RSSI
  int rssi = calculateRSSI(rssiRaw);

  // Zresetuj RX po odczycie
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX

  totalFrames++;  // każdy pakiet z poprawnym sync word

  WeatherData w;
  bool ok = decodeFineOffset(frame, 17, w);
  diagPush(frame, 17, rssi, ok);
  if (ok) {
    w.rssi = rssi;
    w.t = millis();
    lastWeather = w;
    lastWeatherCal = calibrateWeather(w);
    lastWeatherValid = true;
    lastDecodedHex = bytesToHex(frame, 17);
    acceptedFrames++;
    publishWeather();

    Serial.print("POGODA ");
    Serial.print(w.model);
    Serial.print(" id="); Serial.print(w.id);
    if (w.haveTemp) { Serial.print(" T="); Serial.print(w.tempC, 1); Serial.print("C"); }
    if (w.haveHum)  { Serial.print(" RH="); Serial.print(w.humidity); Serial.print("%"); }
    if (w.haveWind) { Serial.print(" wiatr="); Serial.print(w.windAvgMs, 1); Serial.print("m/s"); }
    if (w.haveWindDir) { Serial.print(" dir="); Serial.print(w.windDirDeg); Serial.print("("); Serial.print(windDirText(w.windDirDeg)); Serial.print(")"); }
    if (w.haveRain) { Serial.print(" deszcz="); Serial.print(w.rainMm, 1); Serial.print("mm"); }
    if (w.haveUv)   { Serial.print(" UV="); Serial.print(w.uv); Serial.print("/"); Serial.print(w.uvi); }
    if (w.haveLight){ Serial.print(" lux="); Serial.print(w.lightLux, 0); }
    Serial.print(" RSSI="); Serial.print(rssi);
    Serial.print(" hex="); Serial.println(lastDecodedHex);
  } else {
    rejectedFrames++;
    Serial.print("FO ? [");
    Serial.print(17);
    Serial.print("] RSSI:");
    Serial.print(rssi);
    Serial.print(" : ");
    Serial.println(bytesToHex(frame, 17));
  }
}

// ==================== PĘTLA TRYBU VEVOR / YOUTONG (software sync) ====================
// Radio dziala bez hardware sync (nieskonczona dlugosc). Dekoder utrzymuje
// bufor kołowy surowych bajtów i szuka wzorca "CA 54" (sync) - po nim nastepuje
// 28-bajtowy payload zaczynajacy sie od 0xAA 0x00. Metoda potwierdzona snifferem.
void loopVevor() {
  static uint8_t rbuf[1024];
  static int rhead = 0, rcount = 0;

  // Radio musiało wpaść w overflow (nieskończona długość) - wróć do RX.
  if ((ccStatusRead(0x35) & 0x1F) != 0x0D) {
    ccStrobe(0x3A);  // SFRX
    ccStrobe(0x34);  // SRX
  }

  uint8_t rxBytes = ccStatusRead(0x3B) & 0x7F;  // RXBYTES
  if (rxBytes == 0) {
    delay(1);
    return;
  }

  int n = rxBytes < 64 ? rxBytes : 64;
  uint8_t tmp[64];

  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0xFF);  // burst read FIFO
  for (int i = 0; i < n; i++) tmp[i] = (uint8_t)ccXfer(0);
  radioSPI.endTransaction();
  csHigh();

  int rssi = calculateRSSI(ccStatusRead(0x34));

  // Dodaj do bufora kołowego.
  for (int i = 0; i < n; i++) {
    rbuf[rhead] = tmp[i];
    rhead = (rhead + 1) % 1024;
    if (rcount < 1024) rcount++;
  }

  // Szukaj "CA 54" w buforze (sync + payload 28 B = 30 B min).
  for (int off = 0; off + 30 <= rcount; off++) {
    int idx = (rhead - rcount + off + 1024 * 4) % 1024;
    if (rbuf[idx] == 0xCA && rbuf[(idx + 1) % 1024] == 0x54) {
      uint8_t frame[28];
      for (int k = 0; k < 28; k++) frame[k] = rbuf[(idx + 2 + k) % 1024];
      WeatherData w;
      if (decodeVevor7in1(frame, 28, w)) {
        totalFrames++;
        w.rssi = rssi;
        w.t = millis();
        lastWeather = w;
        lastWeatherCal = calibrateWeather(w);
        lastWeatherValid = true;
        lastDecodedHex = bytesToHex(frame, 28);
        acceptedFrames++;
        diagPush(frame, 28, rssi, true);
        publishWeather();

        Serial.print("POGODA ");
        Serial.print(w.model);
        Serial.print(" id="); Serial.print(w.id);
        if (w.haveTemp) { Serial.print(" T="); Serial.print(w.tempC, 1); Serial.print("C"); }
        if (w.haveHum)  { Serial.print(" RH="); Serial.print(w.humidity); Serial.print("%"); }
        if (w.haveWind) { Serial.print(" wiatr="); Serial.print(w.windAvgMs, 1); Serial.print("m/s"); }
        if (w.haveGust) { Serial.print(" poryw="); Serial.print(w.windMaxMs, 1); Serial.print("m/s"); }
        if (w.haveWindDir) { Serial.print(" dir="); Serial.print(w.windDirDeg); Serial.print("("); Serial.print(windDirText(w.windDirDeg)); Serial.print(")"); }
        if (w.haveRain) { Serial.print(" deszcz="); Serial.print(w.rainMm, 1); Serial.print("mm"); }
        if (w.haveUv)   { Serial.print(" UV="); Serial.print(w.uv); Serial.print("/"); Serial.print(w.uvi); }
        if (w.haveLight){ Serial.print(" lux="); Serial.print(w.lightLux, 0); }
        Serial.print(" RSSI="); Serial.print(rssi);
        Serial.print(" hex="); Serial.println(lastDecodedHex);
        // Wyczyść bufor, żeby nie dekodować tej samej paczki wielokrotnie.
        rcount = 0;
        return;
      }
    }
  }
}

// ==================== PĘTLA TRYBU VEVOR YT60309 ====================
void loopVevorYT60309() {
  uint8_t rxBytes = ccStatusRead(0x3B) & 0x7F;  // RXBYTES
  if (rxBytes < 32) {   // czekaj na kompletny pakiet 32 bajtów
    delay(1);
    return;
  }

  uint8_t frame[32];
  memset(frame, 0, sizeof(frame));

  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0xFF);  // burst read FIFO
  for (int i = 0; i < 32; i++) {
    frame[i] = ccXfer(0);
  }
  radioSPI.endTransaction();
  csHigh();

  int rssiRaw = ccStatusRead(0x34);  // RSSI
  int rssi = calculateRSSI(rssiRaw);

  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX

  totalFrames++;  // każdy pakiet z poprawnym sync word

  WeatherData w;
  bool ok = decodeVevorYT60309(frame, 32, w);
  diagPush(frame, 32, rssi, ok);
  if (ok) {
    w.rssi = rssi;
    w.t = millis();
    lastWeather = w;
    lastWeatherCal = calibrateWeather(w);
    lastWeatherValid = true;
    lastDecodedHex = bytesToHex(frame, 32);
    acceptedFrames++;
    publishWeather();

    Serial.print("POGODA ");
    Serial.print(w.model);
    Serial.print(" id="); Serial.print(w.id);
    if (w.haveTemp) { Serial.print(" T="); Serial.print(w.tempC, 1); Serial.print("C"); }
    if (w.haveHum)  { Serial.print(" RH="); Serial.print(w.humidity); Serial.print("%"); }
    if (w.haveWind) { Serial.print(" wiatr="); Serial.print(w.windAvgMs, 1); Serial.print("m/s"); }
    if (w.haveGust) { Serial.print(" poryw="); Serial.print(w.windMaxMs, 1); Serial.print("m/s"); }
    if (w.haveWindDir) { Serial.print(" dir="); Serial.print(w.windDirDeg); Serial.print("("); Serial.print(windDirText(w.windDirDeg)); Serial.print(")"); }
    if (w.haveRain) { Serial.print(" deszcz="); Serial.print(w.rainMm, 1); Serial.print("mm"); }
    if (w.haveUv)   { Serial.print(" UV="); Serial.print(w.uvi); }
    if (w.haveLight){ Serial.print(" sol="); Serial.print(w.lightLux, 0); Serial.print("W/m2"); }
    Serial.print(" RSSI="); Serial.print(rssi);
    Serial.print(" hex="); Serial.println(lastDecodedHex);
  } else {
    rejectedFrames++;
    Serial.print("YT60309 ? [");
    Serial.print(32);
    Serial.print("] RSSI:");
    Serial.print(rssi);
    Serial.print(" : ");
    Serial.println(bytesToHex(frame, 32));
  }
}

// ==================== PĘTLA TRYBU BRESSER ====================
// Jeden tryb 27-bajtowych pakietów obsługuje 5-in-1, 6-in-1 i 7-in-1:
// po bajcie 0xD4 dekodery próbujemy po kolei (5 -> 6 -> 7), każdy ma
// własną, ścisłą sumę kontrolną, więc błędne dopasowania są odrzucane.
void loopBresser() {
  uint8_t rxBytes = ccStatusRead(0x3B) & 0x7F;  // RXBYTES
  if (rxBytes < 27) {   // czekaj na kompletny pakiet 27 bajtów
    delay(1);
    return;
  }

  uint8_t frame[27];
  memset(frame, 0, sizeof(frame));

  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0xFF);  // burst read FIFO
  for (int i = 0; i < 27; i++) {
    frame[i] = ccXfer(0);
  }
  radioSPI.endTransaction();
  csHigh();

  int rssiRaw = ccStatusRead(0x34);  // RSSI
  int rssi = calculateRSSI(rssiRaw);

  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX

  totalFrames++;  // każdy pakiet z poprawnym sync word

  if (frame[0] != 0xD4) {
    rejectedFrames++;
    Serial.print("BRESSER !sync [");
    Serial.print(bytesToHex(frame, 27));
    Serial.print("] RSSI:");
    Serial.println(rssi);
    return;
  }

  WeatherData w;
  bool ok = false;
  if (decodeBresser5in1(frame + 1, 26, w)) {
    ok = true;
  } else if (decodeBresser6in1(frame + 1, 18, w)) {
    ok = true;
  } else if (decodeBresser7in1(frame + 1, 25, w)) {
    ok = true;
  }
  diagPush(frame, 27, rssi, ok);

  if (ok) {
    w.rssi = rssi;
    w.t = millis();
    lastWeather = w;
    lastWeatherCal = calibrateWeather(w);
    lastWeatherValid = true;
    lastDecodedHex = bytesToHex(frame, 27);
    acceptedFrames++;
    publishWeather();

    Serial.print("POGODA ");
    Serial.print(w.model);
    Serial.print(" id="); Serial.print(w.id);
    if (w.haveTemp) { Serial.print(" T="); Serial.print(w.tempC, 1); Serial.print("C"); }
    if (w.haveHum)  { Serial.print(" RH="); Serial.print(w.humidity); Serial.print("%"); }
    if (w.haveWind) { Serial.print(" wiatr="); Serial.print(w.windAvgMs, 1); Serial.print("m/s"); }
    if (w.haveGust) { Serial.print(" poryw="); Serial.print(w.windMaxMs, 1); Serial.print("m/s"); }
    if (w.haveWindDir) { Serial.print(" dir="); Serial.print(w.windDirDeg); Serial.print("("); Serial.print(windDirText(w.windDirDeg)); Serial.print(")"); }
    if (w.haveRain) { Serial.print(" deszcz="); Serial.print(w.rainMm, 1); Serial.print("mm"); }
    if (w.haveUv)   { Serial.print(" UV="); Serial.print(w.uv); Serial.print("/"); Serial.print(w.uvi); }
    if (w.haveLight){ Serial.print(" lux="); Serial.print(w.lightLux, 0); }
    Serial.print(" RSSI="); Serial.print(rssi);
    Serial.print(" hex="); Serial.println(lastDecodedHex);
  } else {
    rejectedFrames++;
    Serial.print("BRESSER ? [");
    Serial.print(27);
    Serial.print("] RSSI:");
    Serial.print(rssi);
    Serial.print(" : ");
    Serial.println(bytesToHex(frame, 27));
  }
}

// ==================== TRYB SONDY RSSI (diagnostyka) ====================
// Blokuje radio na jednej częstotliwości (FREQ_TABLE[probeFreqIdx]) w trybie
// surowym (bez sync word, 2-FSK 100 kbaud) i próbkuje RSSI w pętli.
// Wypisuje co 1 s minimalny i maksymalny RSSI - pozwala stwierdzić, czy na
// danej częstotliwości coś nadaje (max wyraźnie wyższy od szumu ~-95 dBm).
void applyProbe() {
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX

  if (probeFreqIdx == 6) {
    // 433.92 MHz (test: sprawdzenie czy RSSI -19 dotyczy tylko pasma 868 MHz)
    ccWrite(0x0D, 0x10);  // FREQ2
    ccWrite(0x0E, 0xB0);  // FREQ1
    ccWrite(0x0F, 0xA5);  // FREQ0
  } else if (probeFreqIdx == 7) {
    // Dowolna częstotliwość podana w MHz (do precyzyjnego przemiatania pasma).
    uint32_t w = (uint32_t)((double)probeMhz * 1e6 * 65536.0 / 26e6 + 0.5);
    ccWrite(0x0D, (w >> 16) & 0xFF);
    ccWrite(0x0E, (w >> 8) & 0xFF);
    ccWrite(0x0F, w & 0xFF);
  } else {
    ccWrite(0x0D, FREQ_TABLE[probeFreqIdx][0]);  // FREQ2
    ccWrite(0x0E, FREQ_TABLE[probeFreqIdx][1]);  // FREQ1
    ccWrite(0x0F, FREQ_TABLE[probeFreqIdx][2]);  // FREQ0
  }

  ccWrite(0x10, probeWide ? 0x43 : 0x98);  // MDMCFG4: szerokie 812 kHz albo 162.5 kHz
  ccWrite(0x11, probeWide ? 0x83 : 0xC0);  // MDMCFG3
  ccWrite(0x12, 0x00);  // MDMCFG2: 2-FSK, bez sync word
  ccWrite(0x13, 0x22);  // MDMCFG1
  ccWrite(0x14, 0xF8);  // MDMCFG0
  ccWrite(0x15, 0x44);  // DEVIATN ~38 kHz
  ccWrite(0x1B, 0xFB);  // AGCCTRL2: obniżone wzmocnienie LNA (-14 dB), MAGN_TARGET 30 dB
  ccWrite(0x06, 0xFF);  // PKTLEN = 255 (raw)
  ccWrite(0x07, 0x00);
  ccWrite(0x08, 0x00);

  delay(3);
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX
}

void loopProbe() {
  static int minR = 0, maxR = -200;
  static unsigned long lastPrint = 0;

  // Jeśli FIFO się przepełniło, CC1101 sam przeszedł w IDLE i RSSI przestaje
  // się aktualizować. Wracamy do RX tylko wtedy, gdy naprawdę trzeba - częste
  // restarty zerują AGC i RSSI zamarza na stałej wartości.
  uint8_t st = ccStatusRead(0x35) & 0x1F;   // MARCSTATE (0x0D = RX)
  if (st != 0x0D) {
    ccStrobe(0x36);  // SIDLE
    ccStrobe(0x3A);  // SFRX
    ccStrobe(0x34);  // SRX
    delay(1);
  }

  int rssi = calculateRSSI(ccStatusRead(0x34));   // RSSI (bez zakłócania RX)
  if (rssi > maxR) maxR = rssi;
  if (rssi < minR) minR = rssi;

  // Opróżnij FIFO samym odczytem (bez restartu radia).
  uint8_t rxBytes = ccStatusRead(0x3B) & 0x7F;
  if (rxBytes > 0) {
    int n = rxBytes < 64 ? rxBytes : 64;
    csLow();
    radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    ccXfer(0xFF);
    for (int i = 0; i < n; i++) ccXfer(0);
    radioSPI.endTransaction();
    csHigh();
  }

  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.print("PROBE[");
    Serial.print(probeFreqIdx == 6 ? "433.92" : (probeFreqIdx == 7 ? String(probeMhz, 2) : String(FREQ_NAMES[probeFreqIdx])));
    Serial.print("] RSSI min=");
    Serial.print(minR);
    Serial.print(" max=");
    Serial.println(maxR);
    minR = 0;
    maxR = -200;
  }
}

// ==================== TRYB ZGRYWANIA SUROWEGO STRUMIENIA ====================
// Nasłuch 868.35 MHz, 2-FSK 11.11 kbaud, BEZ sync word: FIFO zbiera
// zdemodulowane bajty. Wypisujemy je tylko, gdy RSSI jest wysokie (burst),
// żeby znaleźć w strumieniu prawdziwy nagłówek stacji (C0AA C0AA).
// 433.92 MHz (test: sprawdzenie czy RSSI -19 dotyczy tylko pasma 868 MHz)
bool captureMode = false;

void applyCapture() {
  ccStrobe(0x36);
  ccStrobe(0x3A);

  ccWrite(0x0D, 0x21);  // 868.35 MHz
  ccWrite(0x0E, 0x65);
  ccWrite(0x0F, 0xE8);

  ccWrite(0x10, 0x98);  // 11.11k, BW 162.5 kHz
  ccWrite(0x11, 0xC0);
  ccWrite(0x12, 0x00);  // 2-FSK, bez sync word
  ccWrite(0x13, 0x22);
  ccWrite(0x14, 0xF8);
  ccWrite(0x15, 0x44);  // ~38 kHz
  ccWrite(0x06, 0xFF);  // PKTLEN 255
  ccWrite(0x07, 0x00);
  ccWrite(0x08, 0x00);

  delay(3);
  ccStrobe(0x36);
  ccStrobe(0x3A);
  ccStrobe(0x34);  // SRX
}

void loopCapture() {
  uint8_t rxBytes = ccStatusRead(0x3B) & 0x7F;
  if (rxBytes == 0) {
    delay(1);
    return;
  }

  uint8_t buf[64];
  int n = rxBytes < 64 ? rxBytes : 64;

  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0xFF);
  for (int i = 0; i < n; i++) buf[i] = ccXfer(0);
  radioSPI.endTransaction();
  csHigh();

  int rssiRaw = ccStatusRead(0x34);
  int rssi = calculateRSSI(rssiRaw);

  ccStrobe(0x36);
  ccStrobe(0x3A);
  ccStrobe(0x34);

  if (rssi > -75) {
    Serial.print("CAP ");
    Serial.print(rssi);
    Serial.print(" : ");
    Serial.println(bytesToHex(buf, n));
  }
}

// ==================== TRYB AUTO (przemiatanie pasma 868 MHz) ====================
// Nie wiadomo z góry, na jakiej dokładnie częstotliwości i w jakim protokole
// nadaje dana stacja. Ten tryb przełącza radio po kolei przez całe pasmo
// 868.0-868.9 MHz (krok 0.1 MHz) oraz przez pozostałe znane protokoły.
// Każdy krok trwa AUTO_DWELL_MS, czyli dłużej niż typowy okres nadawania
// stacji (~16-20 s), więc prędzej czy później trafimy na jej transmisję.
struct AutoStep {
  const char* name;
  uint8_t f[3];
  uint8_t proto;   // 0=FineOffset, 1=YT60309, 2=Youtong, 3=Bresser
  uint8_t len;
};

const AutoStep AUTO_STEPS_TBL[] = {
  { "868.30 YT60309",   {0x21, 0x65, 0x6A}, 1, 32 },
  { "868.35 YT60309",   {0x21, 0x65, 0xE8}, 1, 32 },
  { "868.30 Youtong",   {0x21, 0x65, 0x6A}, 2, 28 },
  { "868.35 Youtong",   {0x21, 0x65, 0xE8}, 2, 28 },
  { "868.30 FineOffset",{0x21, 0x65, 0x6A}, 0, 17 },
  { "868.35 FineOffset",{0x21, 0x65, 0xE8}, 0, 17 },
  { "868.30 Bresser",   {0x21, 0x65, 0x6A}, 3, 27 },
  { "868.35 Bresser",   {0x21, 0x65, 0xE8}, 3, 27 },
  { "868.40 YT60309",   {0x21, 0x66, 0x66}, 1, 32 },
  { "868.20 YT60309",   {0x21, 0x64, 0x6E}, 1, 32 },
  { "433.92 FineOffset",{0x10, 0xB0, 0x71}, 0, 17 },
  { "433.92 YT60309",   {0x10, 0xB0, 0x71}, 1, 32 },
  // 868.95 MHz - pasmo wM-Bus, ale sprawdzamy tez wszystkie protokoly stacji,
  // bo odbiornik stoi przy czujniku i tu pojawial sie silny sygnal.
  { "868.95 YT60309",   {0x21, 0x6B, 0xD1}, 1, 32 },
  { "868.95 FineOffset",{0x21, 0x6B, 0xD1}, 0, 17 },
  { "868.95 Youtong",   {0x21, 0x6B, 0xD1}, 2, 28 },
  { "868.95 Bresser",   {0x21, 0x6B, 0xD1}, 3, 27 },
};

const int AUTO_STEPS = sizeof(AUTO_STEPS_TBL) / sizeof(AUTO_STEPS_TBL[0]);
const unsigned long AUTO_DWELL_MS = 20000;

int autoIdx = 0;
unsigned long autoChange = 0;
unsigned long autoSync[sizeof(AUTO_STEPS_TBL) / sizeof(AUTO_STEPS_TBL[0])] = {0};
unsigned long autoDecoded[sizeof(AUTO_STEPS_TBL) / sizeof(AUTO_STEPS_TBL[0])] = {0};

void applyAutoStep(int i) {
  freqOvr[0] = AUTO_STEPS_TBL[i].f[0];
  freqOvr[1] = AUTO_STEPS_TBL[i].f[1];
  freqOvr[2] = AUTO_STEPS_TBL[i].f[2];
  freqOverride = true;

  switch (AUTO_STEPS_TBL[i].proto) {
    case 0:  applyFineOffsetMode();   break;
    case 1:  applyVevorYT60309Mode(); break;
    case 2:  applyVevorMode();        break;
    default: applyBresserMode();      break;
  }

  freqOverride = false;
}

void loopAuto() {
  if (autoChange == 0) {
    autoChange = millis();
    applyAutoStep(autoIdx);
  }
  if (millis() - autoChange >= AUTO_DWELL_MS) {
    autoIdx = (autoIdx + 1) % AUTO_STEPS;
    applyAutoStep(autoIdx);
    autoChange = millis();
    Serial.print("AUTO -> ");
    Serial.println(AUTO_STEPS_TBL[autoIdx].name);
  }

  int need = AUTO_STEPS_TBL[autoIdx].len;
  uint8_t rxBytes = ccStatusRead(0x3B) & 0x7F;
  if (rxBytes < need) {
    delay(1);
    return;
  }

  uint8_t frame[32];
  memset(frame, 0, sizeof(frame));

  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0xFF);
  for (int i = 0; i < need; i++) frame[i] = ccXfer(0);
  radioSPI.endTransaction();
  csHigh();

  int rssi = calculateRSSI(ccStatusRead(0x34));

  ccStrobe(0x36);
  ccStrobe(0x3A);
  ccStrobe(0x34);

  autoSync[autoIdx]++;

  WeatherData w;
  bool ok = false;
  switch (AUTO_STEPS_TBL[autoIdx].proto) {
    case 0:
      ok = decodeFineOffset(frame, need, w);
      break;
    case 1:
      ok = decodeVevorYT60309(frame, need, w);
      break;
    case 2:
      ok = decodeVevor7in1(frame, need, w);
      break;
    default:
      if (frame[0] == 0xD4) {
        ok = decodeBresser5in1(frame + 1, 26, w) ||
             decodeBresser6in1(frame + 1, 18, w) ||
             decodeBresser7in1(frame + 1, 25, w);
      }
      break;
  }

  diagPush(frame, need, rssi, ok);

  // Podgląd na porcie szeregowym (max ~2 linie/s, żeby nie zalać logu).
  static unsigned long lastAutoPrint = 0;
  if (!ok && millis() - lastAutoPrint >= 500) {
    lastAutoPrint = millis();
    Serial.print("AUTO ");
    Serial.print(AUTO_STEPS_TBL[autoIdx].name);
    Serial.print(" RSSI=");
    Serial.print(rssi);
    Serial.print(" : ");
    Serial.println(bytesToHex(frame, need));
  }

  if (ok) {
    autoDecoded[autoIdx]++;
    w.rssi = rssi;
    w.t = millis();
    lastWeather = w;
    lastWeatherCal = calibrateWeather(w);
    lastWeatherValid = true;
    lastDecodedHex = bytesToHex(frame, need);
    acceptedFrames++;
    publishWeather();

    Serial.print("POGODA ");
    Serial.print(w.model);
    Serial.print(" @ ");
    Serial.print(AUTO_STEPS_TBL[autoIdx].name);
    Serial.print(" id="); Serial.print(w.id);
    if (w.haveTemp) { Serial.print(" T="); Serial.print(w.tempC, 1); Serial.print("C"); }
    if (w.haveHum)  { Serial.print(" RH="); Serial.print(w.humidity); Serial.print("%"); }
    if (w.haveWind) { Serial.print(" wiatr="); Serial.print(w.windAvgMs, 1); Serial.print("m/s"); }
    if (w.haveWindDir) { Serial.print(" dir="); Serial.print(w.windDirDeg); }
    if (w.haveRain) { Serial.print(" deszcz="); Serial.print(w.rainMm, 1); Serial.print("mm"); }
    Serial.println();
  } else {
    rejectedFrames++;
  }
}

// ==================== POMOCNICZE ====================
String bytesToHex(uint8_t* data, int len) {
  String hex;
  hex.reserve(len * 2);
  int n = len < MAX_HEX_BYTES ? len : MAX_HEX_BYTES;
  for (int i = 0; i < n; i++) {
    if (data[i] < 16) hex += "0";
    hex += String(data[i], HEX);
  }
  return hex;
}

int calculateRSSI(uint8_t raw) {
  int rssi;
  if (raw >= 128) {
    rssi = (raw - 256) / 2 - 74;
  } else {
    rssi = raw / 2 - 74;
  }
  return rssi;
}

// ==================== KALIBRACJA / WYSYŁANIE / MQTT ====================
WeatherData calibrateWeather(const WeatherData& in) {
  WeatherData w = in;
  if (w.haveTemp)     w.tempC      += calCfg.tempOffset;
  if (w.haveHum)      w.humidity    = (int)roundf(w.humidity + calCfg.humOffset);
  if (w.haveWind)     w.windAvgMs  *= calCfg.windFactor;
  if (w.haveGust)     w.windMaxMs  *= calCfg.gustFactor;
  // Opad: czujnik trzyma licznik skumulowany (może być naliczony dawniej,
  // zanim konsola została zresetowana). Odejmujemy punkt odniesienia
  // (pierwsza wartość po starcie), żeby WWW pokazywało "opad od włączenia"
  // i zgadzało się z konsolą (0, gdy nie pada).
  if (w.haveRain) {
    if (rainBaseline < 0.0f) rainBaseline = w.rainMm;
    w.rainMm = (w.rainMm - rainBaseline) * calCfg.rainFactor;
    if (w.rainMm < 0.0f) w.rainMm = 0.0f;
  }
  if (w.haveLight)    w.lightLux   *= calCfg.lightFactor;
  if (w.haveWindDir)  w.windDirDeg  = ((w.windDirDeg + calCfg.windDirOffset) % 360 + 360) % 360;
  return w;
}

// JSON (bez zewnętrznych nawiasów) - wspólny dla WWW, HTTP i RS485.
String weatherToJson(const WeatherData& w) {
  String j = "\"model\":\"" + w.model + "\",\"id\":" + String(w.id);
  j += ",\"battery_ok\":" + String(w.batteryOk ? "true" : "false");
  if (w.haveTemp)     j += ",\"temperature_C\":" + String(w.tempC, 1);
  if (w.haveHum)      j += ",\"humidity\":" + String(w.humidity);
  if (w.haveWindDir)  j += ",\"wind_dir_deg\":" + String(w.windDirDeg);
  if (w.haveWind)     j += ",\"wind_avg_m_s\":" + String(w.windAvgMs, 2);
  if (w.haveGust)     j += ",\"wind_max_m_s\":" + String(w.windMaxMs, 2);
  if (w.haveRain)     j += ",\"rain_mm\":" + String(w.rainMm, 2);
  if (w.haveUv)       j += ",\"uv\":" + String(w.uv) + ",\"uvi\":" + String(w.uvi);
  if (w.haveLight)    j += ",\"light_lux\":" + String(w.lightLux, 0);
  j += ",\"rssi\":" + String(w.rssi);
  return j;
}

// ---------- MQTT ----------
void mqttPubDiscovery(const char* key, const char* name, const char* unit, const char* devClass, const char* stateTopic) {
  String topic = "homeassistant/sensor/" + mqttCfg.topicPrefix + "/" + key + "/config";
  String payload = "{\"name\":\"" + String(name) + "\"";
  payload += ",\"state_topic\":\"" + String(stateTopic) + "\"";
  payload += ",\"unique_id\":\"ws_" + mqttCfg.topicPrefix + "_" + key + "\"";
  if (unit && strlen(unit) > 0)         payload += ",\"unit_of_measurement\":\"" + String(unit) + "\"";
  if (devClass && strlen(devClass) > 0) payload += ",\"device_class\":\"" + String(devClass) + "\"";
  payload += ",\"device\":{\"identifiers\":[\"" + mqttCfg.topicPrefix + "\"],\"name\":\"WeatherSniffer 868\",\"model\":\"ESP32-S3 + CC1101\"}";
  payload += "}";
  mqtt.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPubDiscoveryAll() {
  String p = mqttCfg.topicPrefix;
  mqttPubDiscovery("temperature",    "Temperatura",        "°C",   "temperature",     (p + "/temperature").c_str());
  mqttPubDiscovery("humidity",       "Wilgotnosc",         "%",    "humidity",        (p + "/humidity").c_str());
  mqttPubDiscovery("wind_speed",     "Wiatr",              "m/s",  "wind_speed",      (p + "/wind_speed").c_str());
  mqttPubDiscovery("wind_gust",      "Wiatr (poryw)",      "m/s",  "wind_speed",      (p + "/wind_gust").c_str());
  mqttPubDiscovery("wind_direction", "Kierunek wiatru",    "°",    "",                (p + "/wind_direction").c_str());
  mqttPubDiscovery("rain",           "Opad",               "mm",   "",                (p + "/rain").c_str());
  mqttPubDiscovery("uv_index",       "Indeks UV",          "UVI",  "",                (p + "/uv_index").c_str());
  mqttPubDiscovery("light",          "Swiatlo",            "lx",   "illuminance",     (p + "/light").c_str());
  mqttPubDiscovery("rssi",           "RSSI",               "dBm",  "signal_strength", (p + "/rssi").c_str());
}

void publishMqttWeather(const WeatherData& w) {
  if (!mqtt.connected()) return;
  String p = mqttCfg.topicPrefix;
  if (w.haveTemp)     mqtt.publish((p + "/temperature").c_str(),    String(w.tempC, 1).c_str());
  if (w.haveHum)      mqtt.publish((p + "/humidity").c_str(),       String(w.humidity).c_str());
  if (w.haveWind)     mqtt.publish((p + "/wind_speed").c_str(),     String(w.windAvgMs, 2).c_str());
  if (w.haveGust)     mqtt.publish((p + "/wind_gust").c_str(),      String(w.windMaxMs, 2).c_str());
  if (w.haveWindDir)  mqtt.publish((p + "/wind_direction").c_str(), String(w.windDirDeg).c_str());
  if (w.haveRain)     mqtt.publish((p + "/rain").c_str(),           String(w.rainMm, 2).c_str());
  if (w.haveUv)       mqtt.publish((p + "/uv_index").c_str(),       String(w.uvi).c_str());
  if (w.haveLight)    mqtt.publish((p + "/light").c_str(),          String(w.lightLux, 0).c_str());
  mqtt.publish((p + "/rssi").c_str(), String(w.rssi).c_str());
}

void mqttEnsureConnected() {
  if (!mqttCfg.enabled || mqttCfg.broker.length() == 0) return;
  if (mqtt.connected()) return;
  static unsigned long lastTry = 0;
  if (millis() - lastTry < 10000) return;
  lastTry = millis();

  if (mqttClientId.length() == 0) {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char buf[32];
    snprintf(buf, sizeof(buf), "WS-%02X%02X%02X", mac[3], mac[4], mac[5]);
    mqttClientId = String(buf);
  }

  bool ok;
  if (mqttCfg.user.length() > 0)
    ok = mqtt.connect(mqttClientId.c_str(), mqttCfg.user.c_str(), mqttCfg.pass.c_str());
  else
    ok = mqtt.connect(mqttClientId.c_str());

  if (ok) {
    Serial.println("MQTT polaczony: " + mqttCfg.broker);
    if (mqttCfg.haDiscovery) mqttPubDiscoveryAll();
  } else {
    Serial.print("MQTT blad (rc=");
    Serial.print(mqtt.state());
    Serial.println(")");
  }
}

void applyMqtt() {
  if (mqttCfg.enabled && mqttCfg.broker.length() > 0) {
    mqtt.setServer(mqttCfg.broker.c_str(), mqttCfg.port);
    mqttClientId = "";   // wymuś nowe client-id po zmianie konfiguracji
  } else {
    mqtt.disconnect();
  }
}

void applySend() {
  Serial1.end();
  if (sendCfg.rs485Enabled) {
    Serial1.begin(sendCfg.rs485Baud, SERIAL_8N1, sendCfg.rs485RxPin, sendCfg.rs485TxPin);
    if (sendCfg.rs485DePin >= 0) {
      pinMode(sendCfg.rs485DePin, OUTPUT);
      digitalWrite(sendCfg.rs485DePin, LOW);
    }
    Serial.println("RS485: TX=" + String(sendCfg.rs485TxPin) + " RX=" + String(sendCfg.rs485RxPin) +
                   " baud=" + String(sendCfg.rs485Baud) + " DE=" + String(sendCfg.rs485DePin));
  }
}

// Wysyła ostatnie (skalibrowane) dane wszystkimi włączonymi kanałami.
void publishWeather() {
  if (!lastWeatherValid) return;
  const WeatherData& w = lastWeatherCal;
  String json = "{" + weatherToJson(w) + "}";

  // 1) HTTP POST do innego ESP / dowolnego odbiorcy JSON
  if (sendCfg.wifiEnabled && sendCfg.targetUrl.length() > 0 && WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.setTimeout(3000);
    if (http.begin(sendCfg.targetUrl)) {
      http.addHeader("Content-Type", "application/json");
      int code = http.POST(json);
      if (code < 0) {
        Serial.print("HTTP wysylka blad: ");
        Serial.println(http.errorToString(code));
      }
      http.end();
    } else {
      Serial.println("HTTP: nie mozna utworzyc polaczenia z " + sendCfg.targetUrl);
    }
  }

  // 2) RS485 (UART1) - jedna linia JSON zakończona \n
  if (sendCfg.rs485Enabled) {
    if (sendCfg.rs485DePin >= 0) digitalWrite(sendCfg.rs485DePin, HIGH);
    Serial1.print(json);
    Serial1.print('\n');
    Serial1.flush();
    if (sendCfg.rs485DePin >= 0) digitalWrite(sendCfg.rs485DePin, LOW);
  }

  // 3) MQTT
  if (mqttCfg.enabled && mqtt.connected()) {
    publishMqttWeather(w);
  }
}

// ---------- NVS: kalibracja / wysyłanie / MQTT ----------
Preferences calPrefs, sendPrefs, mqttPrefs;

void loadCalCfg() {
  calPrefs.begin("cal", false);
  if (!calPrefs.getBool("init", false)) {
    calPrefs.putFloat("tempOffset", 0.0f);
    calPrefs.putFloat("humOffset", 0.0f);
    calPrefs.putFloat("windFactor", 1.0f);
    calPrefs.putFloat("gustFactor", 1.0f);
    calPrefs.putFloat("rainFactor", 1.0f);
    calPrefs.putFloat("lightFactor", 1.0f);
    calPrefs.putInt("windDirOffset", 0);
    calPrefs.putBool("init", true);
  }
  calCfg.tempOffset    = calPrefs.getFloat("tempOffset", 0.0f);
  calCfg.humOffset     = calPrefs.getFloat("humOffset", 0.0f);
  calCfg.windFactor    = calPrefs.getFloat("windFactor", 1.0f);
  calCfg.gustFactor    = calPrefs.getFloat("gustFactor", 1.0f);
  calCfg.rainFactor    = calPrefs.getFloat("rainFactor", 1.0f);
  calCfg.lightFactor   = calPrefs.getFloat("lightFactor", 1.0f);
  calCfg.windDirOffset = calPrefs.getInt("windDirOffset", 0);
  calPrefs.end();
}

void saveCalCfg() {
  calPrefs.begin("cal", false);
  calPrefs.putFloat("tempOffset", calCfg.tempOffset);
  calPrefs.putFloat("humOffset", calCfg.humOffset);
  calPrefs.putFloat("windFactor", calCfg.windFactor);
  calPrefs.putFloat("gustFactor", calCfg.gustFactor);
  calPrefs.putFloat("rainFactor", calCfg.rainFactor);
  calPrefs.putFloat("lightFactor", calCfg.lightFactor);
  calPrefs.putInt("windDirOffset", calCfg.windDirOffset);
  calPrefs.end();
}

void loadSendCfg() {
  sendPrefs.begin("send", false);
  if (!sendPrefs.getBool("init", false)) {
    sendPrefs.putBool("wifiEnabled", false);
    sendPrefs.putString("targetUrl", "");
    sendPrefs.putBool("rs485Enabled", false);
    sendPrefs.putInt("rs485TxPin", 17);
    sendPrefs.putInt("rs485RxPin", 18);
    sendPrefs.putInt("rs485Baud", 9600);
    sendPrefs.putInt("rs485DePin", -1);
    sendPrefs.putBool("init", true);
  }
  sendCfg.wifiEnabled   = sendPrefs.getBool("wifiEnabled", false);
  sendCfg.targetUrl     = sendPrefs.getString("targetUrl", "");
  sendCfg.rs485Enabled  = sendPrefs.getBool("rs485Enabled", false);
  sendCfg.rs485TxPin    = sendPrefs.getInt("rs485TxPin", 17);
  sendCfg.rs485RxPin    = sendPrefs.getInt("rs485RxPin", 18);
  sendCfg.rs485Baud     = sendPrefs.getInt("rs485Baud", 9600);
  sendCfg.rs485DePin    = sendPrefs.getInt("rs485DePin", -1);
  sendPrefs.end();
}

void saveSendCfg() {
  sendPrefs.begin("send", false);
  sendPrefs.putBool("wifiEnabled", sendCfg.wifiEnabled);
  sendPrefs.putString("targetUrl", sendCfg.targetUrl);
  sendPrefs.putBool("rs485Enabled", sendCfg.rs485Enabled);
  sendPrefs.putInt("rs485TxPin", sendCfg.rs485TxPin);
  sendPrefs.putInt("rs485RxPin", sendCfg.rs485RxPin);
  sendPrefs.putInt("rs485Baud", sendCfg.rs485Baud);
  sendPrefs.putInt("rs485DePin", sendCfg.rs485DePin);
  sendPrefs.end();
}

void loadMqttCfg() {
  mqttPrefs.begin("mqtt", false);
  if (!mqttPrefs.getBool("init", false)) {
    mqttPrefs.putBool("enabled", false);
    mqttPrefs.putString("broker", "");
    mqttPrefs.putInt("port", 1883);
    mqttPrefs.putString("user", "");
    mqttPrefs.putString("pass", "");
    mqttPrefs.putString("topicPrefix", "weathersniffer");
    mqttPrefs.putBool("haDiscovery", false);
    mqttPrefs.putBool("init", true);
  }
  mqttCfg.enabled     = mqttPrefs.getBool("enabled", false);
  mqttCfg.broker      = mqttPrefs.getString("broker", "");
  mqttCfg.port        = mqttPrefs.getInt("port", 1883);
  mqttCfg.user        = mqttPrefs.getString("user", "");
  mqttCfg.pass        = mqttPrefs.getString("pass", "");
  mqttCfg.topicPrefix = mqttPrefs.getString("topicPrefix", "weathersniffer");
  mqttCfg.haDiscovery = mqttPrefs.getBool("haDiscovery", false);
  mqttPrefs.end();
}

void saveMqttCfg() {
  mqttPrefs.begin("mqtt", false);
  mqttPrefs.putBool("enabled", mqttCfg.enabled);
  mqttPrefs.putString("broker", mqttCfg.broker);
  mqttPrefs.putInt("port", mqttCfg.port);
  mqttPrefs.putString("user", mqttCfg.user);
  mqttPrefs.putString("pass", mqttCfg.pass);
  mqttPrefs.putString("topicPrefix", mqttCfg.topicPrefix);
  mqttPrefs.putBool("haDiscovery", mqttCfg.haDiscovery);
  mqttPrefs.end();
}

// ---------- FILTR RAMKOWY ----------
Preferences filterPrefs;

void loadFilterCfg() {
  filterPrefs.begin("filter", false);
  if (!filterPrefs.getBool("init", false)) {
    filterPrefs.putBool("enabled", false);
    filterPrefs.putString("pattern", "");
    filterPrefs.putInt("offset", -1);
    filterPrefs.putInt("minLen", 0);
    filterPrefs.putInt("maxLen", 0);
    filterPrefs.putBool("init", true);
  }
  filterCfg.enabled = filterPrefs.getBool("enabled", false);
  filterCfg.pattern = filterPrefs.getString("pattern", "");
  filterCfg.offset  = filterPrefs.getInt("offset", -1);
  filterCfg.minLen  = filterPrefs.getInt("minLen", 0);
  filterCfg.maxLen  = filterPrefs.getInt("maxLen", 0);
  filterPrefs.end();
}

void saveFilterCfg() {
  filterPrefs.begin("filter", false);
  filterPrefs.putBool("enabled", filterCfg.enabled);
  filterPrefs.putString("pattern", filterCfg.pattern);
  filterPrefs.putInt("offset", filterCfg.offset);
  filterPrefs.putInt("minLen", filterCfg.minLen);
  filterPrefs.putInt("maxLen", filterCfg.maxLen);
  filterPrefs.end();
}

// ---------- TRYB ODBIORU (NVS) ----------
Preferences modePrefs;

void loadRxMode() {
  modePrefs.begin("mode", false);
  if (!modePrefs.getBool("init", false)) {
    modePrefs.putInt("rxMode", MODE_RAW_SCAN);
    modePrefs.putBool("init", true);
  }
  rxMode = (RxMode)modePrefs.getInt("rxMode", MODE_RAW_SCAN);
  if (rxMode != MODE_RAW_SCAN && rxMode != MODE_FINE_OFFSET && rxMode != MODE_VEVOR_7IN1 && rxMode != MODE_BRESSER && rxMode != MODE_VEVOR_YT60309 && rxMode != MODE_WEATHER_AUTO) rxMode = MODE_RAW_SCAN;
  modePrefs.end();
}

void saveRxMode() {
  modePrefs.begin("mode", false);
  modePrefs.putInt("rxMode", (int)rxMode);
  modePrefs.end();
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Hex -> bajty. Zwraca liczbę bajtów (-1 = błąd). "a5b1" -> {0xA5,0xB1}
static int hexToBytes(const String& hex, uint8_t* out, int maxOut) {
  String h = hex;
  h.toLowerCase();
  h.replace(" ", "");
  h.replace("-", "");
  if (h.length() % 2) return -1;
  int n = h.length() / 2;
  if (n > maxOut) return -1;
  for (int i = 0; i < n; i++) {
    int hi = hexVal(h[2 * i]);
    int lo = hexVal(h[2 * i + 1]);
    if (hi < 0 || lo < 0) return -1;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return n;
}

// Czy ramka przechodzi przez filtr (przy wyłączonym filtrze -> zawsze tak).
bool frameMatchesFilter(const uint8_t* data, int len) {
  if (!filterCfg.enabled) return true;
  if (filterCfg.minLen > 0 && len < filterCfg.minLen) return false;
  if (filterCfg.maxLen > 0 && len > filterCfg.maxLen) return false;
  if (filterCfg.pattern.length() == 0) return true;  // tylko filtr długości

  uint8_t pat[64];
  int plen = hexToBytes(filterCfg.pattern, pat, 64);
  if (plen <= 0) return false;

  if (filterCfg.offset >= 0) {
    if (filterCfg.offset + plen > len) return false;
    for (int i = 0; i < plen; i++)
      if (data[filterCfg.offset + i] != pat[i]) return false;
    return true;
  }

  // Szukaj wzorca gdziekolwiek w ramce
  for (int s = 0; s + plen <= len; s++) {
    bool ok = true;
    for (int i = 0; i < plen; i++)
      if (data[s + i] != pat[i]) { ok = false; break; }
    if (ok) return true;
  }
  return false;
}

// Rejestruje ramkę jako "urządzenie" w krótkim widoku WWW. Ramki o tej samej
// sygnaturze (pierwsze bajty) są scalane, więc widok nie zalewa setkami wpisów.
void trackDevice(uint8_t* data, int len, int rssi, uint8_t freq, uint8_t prof) {
  String sig = bytesToHex(data, (len < DEVICE_SIG_BYTES) ? len : DEVICE_SIG_BYTES);

  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].sig == sig) {
      devices[i].count++;
      devices[i].lastSeen = millis();
      devices[i].lastRssi = rssi;
      devices[i].freq = freq;
      devices[i].prof = prof;
      devices[i].len = len;
      devices[i].lastHex = bytesToHex(data, len);
      return;
    }
  }

  // Nowe urządzenie. Jak brak miejsca -> zastąp najdawniej widziane (LRU).
  int slot = -1;
  if (deviceCount < MAX_DEVICES) {
    slot = deviceCount++;
  } else {
    unsigned long oldest = devices[0].lastSeen;
    slot = 0;
    for (int i = 1; i < MAX_DEVICES; i++) {
      if (devices[i].lastSeen < oldest) {
        oldest = devices[i].lastSeen;
        slot = i;
      }
    }
  }

  devices[slot].sig = sig;
  devices[slot].count = 1;
  devices[slot].lastSeen = millis();
  devices[slot].lastRssi = rssi;
  devices[slot].freq = freq;
  devices[slot].prof = prof;
  devices[slot].len = len;
  devices[slot].lastHex = bytesToHex(data, len);
}

void recordFrame(uint8_t* data, int len, int rssi, int lqi) {
  totalFrames++;  // każda ramka powyżej progu RSSI

  bool pass = frameMatchesFilter(data, len);
  if (pass) {
    RawFrame& f = frames[frameHead];
    f.t = millis();
    f.freq = currentFreq;
    f.prof = currentProf;
    f.rssi = rssi;
    f.lqi = lqi;
    f.len = len;
    f.hex = bytesToHex(data, len);

    frameHead = (frameHead + 1) % MAX_FRAMES;
    if (frameCount < MAX_FRAMES) frameCount++;
    acceptedFrames++;
    trackDevice(data, len, rssi, currentFreq, currentProf);
  } else {
    rejectedFrames++;
  }

  comboHits[currentFreq][currentProf]++;

  // Pełny hex na serial (do analizy offline) - również ramki odrzucone przez filtr.
  Serial.print(pass ? "RX[" : "RX-ODRZ[");
  Serial.print(len);
  Serial.print("] ");
  Serial.print(FREQ_NAMES[currentFreq]);
  Serial.print("/");
  Serial.print(PROF_NAMES[currentProf]);
  Serial.print(" RSSI:");
  Serial.print(rssi);
  Serial.print(" LQI:");
  Serial.print(lqi);
  Serial.print(" : ");
  Serial.println(bytesToHex(data, len));
}

// Stan łączności z CC1101 (wyświetlany na stronie, żeby od razu widzieć,
// czy moduł radiowy odpowiada po SPI).
int radioVersion = -1;
int radioPartnum = -1;
int radioVersionHits = 0;     // ile z prób dało wartość dominującą
int radioVersionSamples = 0;  // liczba prób

void radioHealthCheck() {
  // Pojedynczy odczyt po SPI może czasem zgubić bit, więc czytamy kilka razy
  // i wybieramy wartość dominującą (najczęstszą).
  int count[256] = {0};
  int n = 5;
  for (int i = 0; i < n; i++) {
    count[ccStatusRead(0x31)]++;
    delayMicroseconds(200);
  }
  int best = -1, bestCnt = 0;
  for (int v = 0; v < 256; v++) {
    if (count[v] > bestCnt) { bestCnt = count[v]; best = v; }
  }
  radioVersion = best;
  radioVersionHits = bestCnt;
  radioVersionSamples = n;
  radioPartnum = ccStatusRead(0x30);
}

// ==================== STRONA WWW ====================
void handleRoot() {
  radioHealthCheck();
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="3">
  <title>868 MHz Weather Sniffer</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
           background: #1a1a2e; color: #eee; padding: 16px; }
    h1 { color: #00d9ff; margin-bottom: 10px; font-size: 1.3em; }
    .stats { background: #16213e; padding: 14px; border-radius: 8px;
             margin-bottom: 16px; display: flex; gap: 24px; flex-wrap: wrap; }
    .stat { text-align: center; }
    .stat-value { font-size: 1.6em; color: #00d9ff; font-weight: bold; }
    .stat-label { color: #888; font-size: 0.85em; }
    h2 { color: #00d9ff; margin: 16px 0 8px; font-size: 1.1em; }
    table { width: 100%; border-collapse: collapse; background: #16213e;
            border-radius: 8px; overflow: hidden; margin-bottom: 12px; }
    th { background: #0f3460; color: #00d9ff; padding: 8px; text-align: left; font-size: 0.8em; }
    td { padding: 7px 8px; border-bottom: 1px solid #0f3460; font-size: 0.8em; }
    tr:hover { background: #1f4068; }
    .hex { font-family: monospace; font-size: 0.7em; color: #9ad0ff;
           word-break: break-all; max-width: 420px; }
    .rssi { color: #6bcb77; }
    .hit { color: #00ff88; font-weight: bold; }
    .zero { color: #444; }
    .mono { font-family: monospace; }
  </style>
</head>
<body>
  <h1>📡 868 MHz — odbiornik stacji pogodowych</h1>
  <div style="margin-bottom:12px">
    <a href="/" style="color:#00d9ff;margin-right:14px;text-decoration:none">📡 Odczyt</a>
    <a href="/setup" style="color:#00d9ff;margin-right:14px;text-decoration:none">⚙️ Ustawienia</a>
    <a href="/cal" style="color:#00d9ff;margin-right:14px;text-decoration:none">📐 Kalibracja</a>
    <a href="/send" style="color:#00d9ff;margin-right:14px;text-decoration:none">📤 Wysyłanie</a>
    <a href="/mqtt" style="color:#00d9ff;margin-right:14px;text-decoration:none">🔌 MQTT</a>
    <a href="/json" style="color:#00d9ff;margin-right:14px;text-decoration:none">JSON</a>
    <a href="/clear" style="color:#ff6b6b;text-decoration:none">Wyczyść</a>
  </div>
  <div class="stats">
    <div class="stat">
      <div class="stat-value mono" style="color:#ff6b6b">)";
  html += (rxMode == MODE_FINE_OFFSET)
            ? "868.30 / FSK_17k"
            : (rxMode == MODE_VEVOR_7IN1)
              ? "868.30 / FSK_11k"
              : (rxMode == MODE_VEVOR_YT60309)
                ? "868.35 / FSK_11k"
                : (rxMode == MODE_WEATHER_AUTO)
                  ? "AUTO"
                  : (rxMode == MODE_BRESSER)
                    ? "868.30 / FSK_8k"
                    : String(FREQ_NAMES[currentFreq]) + " / " + String(PROF_NAMES[currentProf]);
  html += R"(</div>
      <div class="stat-label">Aktualna kombinacja</div>
    </div>
    <div class="stat">
      <div class="stat-value" style="color:#c084fc">)";
  html += (rxMode == MODE_FINE_OFFSET) ? "Fine Offset (VEVOR)"
        : (rxMode == MODE_VEVOR_7IN1) ? "VEVOR / Youtong"
        : (rxMode == MODE_VEVOR_YT60309) ? "VEVOR YT60309"
        : (rxMode == MODE_WEATHER_AUTO) ? "AUTO (wszystkie)"
        : (rxMode == MODE_BRESSER) ? "Bresser 5/6/7-in-1"
        : "Skan raw";
  html += R"(</div>
      <div class="stat-label">Tryb odbioru</div>
    </div>
    <div class="stat">
      <div class="stat-value">)";
  html += String(totalFrames);
  html += R"(</div>
      <div class="stat-label">Ramek (wszystkie)</div>
    </div>
    <div class="stat">
      <div class="stat-value" style="color:#6bcb77">)";
  html += String(acceptedFrames);
  html += R"(</div>
      <div class="stat-label">Zapisane</div>
    </div>
    <div class="stat">
      <div class="stat-value" style="color:#ffb84d">)";
  html += String(rejectedFrames);
  html += R"(</div>
      <div class="stat-label">Odrzucone (filtr)</div>
    </div>
    <div class="stat">
      <div class="stat-value">)";
  html += String((millis() - startTime) / 1000);
  html += R"(s</div>
      <div class="stat-label">Uptime</div>
    </div>
    <div class="stat">
      <div class="stat-value">)";
  html += WiFi.localIP().toString();
  html += R"(</div>
      <div class="stat-label">IP</div>
    </div>
    <div class="stat">
      <div class="stat-value" style=")";
  bool radioOk = (radioVersion == 0x14);
  html += (radioOk && radioVersionHits >= 4) ? "color:#6bcb77" : (radioOk ? "color:#ffb84d" : "color:#ff6b6b");
  html += R"(">)";
  html += radioOk ? ("OK " + String(radioVersionHits) + "/" + String(radioVersionSamples)) : "BRAK";
  html += R"(</div>
      <div class="stat-label">Radio CC1101</div>
    </div>
  </div>
)";

  if (radioVersion != 0x14) {
    html += R"(<div style="background:#3a1a1a;border:1px solid #ff6b6b;padding:14px;border-radius:8px;color:#ffb3b3;margin-bottom:14px">
      <b>Moduł radiowy CC1101 nie odpowiada po SPI</b> (VERSION=0x)";
    html += String(radioVersion, HEX);
    html += R"( zamiast 0x14). Sprawdź połączenia: zasilanie 3.3 V, GND, SCK=GPIO12,
      MISO=GPIO13, MOSI=GPIO11, CS=GPIO10. Bez poprawnego połączenia nic nie zostanie
      odebrane, niezależnie od ustawień.
    </div>)";
  } else if (radioVersionHits < radioVersionSamples) {
    html += R"(<div style="background:#3a2f1a;border:1px solid #ffb84d;padding:14px;border-radius:8px;color:#ffe0a3;margin-bottom:14px">
      <b>Uwaga: odczyty SPI są niestabilne</b> — radio odpowiada poprawnie tylko w )";
    html += String(radioVersionHits);
    html += R"( z )";
    html += String(radioVersionSamples);
    html += R"( prób. Radio działa, ale pojedyncze odczyty gubią bity.
      Warto sprawdzić jakość połączeń (krótkie przewody, dobre styki) oraz zasilanie 3.3 V.
    </div>)";
  }

  // --- Sekcja danych pogodowych (dekoder) ---
  html += R"(<h2>Dane pogodowe (dekoder)</h2>)";

  if (rxMode == MODE_RAW_SCAN) {
    html += R"(<div style="background:#16213e;padding:14px;border-radius:8px;color:#888">
      Tryb dekodera wylaczony. Wlacz „VEVOR YT60309”, „Fine Offset (VEVOR)”,
      „VEVOR / Youtong 7-in-1” lub „Bresser 5/6/7-in-1” w Ustawieniach,
      aby nasluchiwac stacji pogodowej i dekodowac dane.
    </div>)";
  } else if (rxMode == MODE_WEATHER_AUTO && !lastWeatherValid) {
    html += R"(<table>
      <tr><th>Konfiguracja (częstotliwość / protokół)</th><th>Pakiety z sync</th><th>Zdekodowane</th></tr>)";
    for (int i = 0; i < AUTO_STEPS; i++) {
      html += "<tr><td>";
      html += (i == autoIdx) ? "<b style='color:#00d9ff'>" + String(AUTO_STEPS_TBL[i].name) + " ← teraz</b>" : String(AUTO_STEPS_TBL[i].name);
      html += "</td><td>" + String(autoSync[i]) + "</td><td class='" + (autoDecoded[i] > 0 ? "hit" : "zero") + "'>" + String(autoDecoded[i]) + "</td></tr>";
    }
    html += R"(</table>
    <div style="background:#16213e;padding:14px;border-radius:8px;color:#ffb84d">
      Tryb AUTO sprawdza po kolei wszystkie protokoly (po )";
    html += String(AUTO_DWELL_MS / 1000);
    html += R"( s kazdy). Czekam na pakiet stacji — kolumna „Zdekodowane” pokaze,
      ktory protokol pasuje. Potrzeba kilku minut (stacja nadaje co ~16–20 s).
    </div>)";
  } else if (!lastWeatherValid) {
    html += R"(<div style="background:#16213e;padding:14px;border-radius:8px;color:#ffb84d">
      Nasluchuje )";
    html += (rxMode == MODE_VEVOR_7IN1) ? "868.30 MHz (VEVOR / Youtong)"
          : (rxMode == MODE_VEVOR_YT60309) ? "868.35 MHz (VEVOR YT60309)"
          : (rxMode == MODE_WEATHER_AUTO) ? "wszystkie protokoly (AUTO)"
          : (rxMode == MODE_BRESSER) ? "868.30 MHz (Bresser)"
          : "868.30 MHz (Fine Offset / VEVOR)";
    html += R"(... Czekam na pakiet
      (stacja wysyla co ~16–20 s). Nie odebrano jeszcze poprawnej ramki.
    </div>)";
  } else {
    const WeatherData& w = lastWeatherCal;   // wartości po kalibracji
    html += R"(<table>
      <tr><th>Parametr</th><th>Wartość</th></tr>)";
    html += "<tr><td>Model / rodzina</td><td class='mono'>" + w.model + "</td></tr>";
    html += "<tr><td>ID nadajnika</td><td class='mono'>" + String(w.id) + "</td></tr>";
    html += "<tr><td>Bateria</td><td>" + String(w.batteryOk ? "OK" : "SLABA") + "</td></tr>";
    if (w.haveTemp)
      html += "<tr><td>Temperatura</td><td>" + String(w.tempC, 1) + " &deg;C</td></tr>";
    if (w.haveHum)
      html += "<tr><td>Wilgotność</td><td>" + String(w.humidity) + " %</td></tr>";
    if (w.haveWind)
      html += "<tr><td>Wiatr (średni)</td><td>" + String(w.windAvgMs, 1) + " m/s (" + String(w.windAvgMs * 3.6f, 1) + " km/h)</td></tr>";
    if (w.haveGust)
      html += "<tr><td>Wiatr (poryw)</td><td>" + String(w.windMaxMs, 1) + " m/s (" + String(w.windMaxMs * 3.6f, 1) + " km/h)</td></tr>";
    if (w.haveWindDir)
      html += "<tr><td>Kierunek wiatru</td><td>" + String(w.windDirDeg) + "&deg; (" + String(windDirText(w.windDirDeg)) + ")</td></tr>";
    if (w.haveRain)
      html += "<tr><td>Opad (od włączenia)</td><td>" + String(w.rainMm, 1) + " mm</td></tr>";
    if (w.haveUv)
      html += "<tr><td>UV / indeks UV</td><td>" + String(w.uv) + " / " + String(w.uvi) + "</td></tr>";
    if (w.haveLight)
      html += "<tr><td>Światło</td><td>" + String(w.lightLux, 0) + (w.model == "Vevor-YT60309" ? " W/m²" : " lux") + "</td></tr>";
    html += "<tr><td>RSSI</td><td>" + String(w.rssi) + " dBm</td></tr>";
    html += "<tr><td>Ostatni pakiet</td><td class='hex'>" + lastDecodedHex + "</td></tr>";
    html += R"(</table>)";
  }

  // --- Diagnostyka odbioru (tylko w trybach dekodera) ---
  if (rxMode != MODE_RAW_SCAN) {
    html += R"(<h2>Diagnostyka odbioru</h2>
    <div style="background:#16213e;padding:14px;border-radius:8px;margin-bottom:12px;color:#bbb">
      Pakiety z poprawnym sync word: <b style="color:#00d9ff">)";
    html += String(diagTotal);
    html += R"(</b> &nbsp;|&nbsp; Zdekodowane poprawnie: <b style="color:#6bcb77">)";
    html += String(acceptedFrames);
    html += R"(</b> &nbsp;|&nbsp; Odrzucone (zła suma kontrolna): <b style="color:#ffb84d">)";
    html += String(rejectedFrames);
    html += R"(</b> &nbsp;|&nbsp; Ostatni RSSI: <b class="rssi">)";
    html += String(lastRawRssi);
    html += R"( dBm</b>
    </div>)";

    if (diagCount == 0) {
      html += R"(<div style="background:#16213e;padding:14px;border-radius:8px;color:#ffb84d">
        Radio nasłuchuje, ale nie dotarł jeszcze ŻADEN pakiet z poprawnym sync word.
        Znaczy to, że sygnał stacji nie dociera do odbiornika (za daleko, za słaby
        albo zagłuszony przez inne nadajniki 868 MHz w pobliżu).
      </div>)";
    } else {
      html += R"(<table>
        <tr><th>Czas</th><th>RSSI</th><th>Długość</th><th>Status</th><th>Hex</th></tr>)";
      int showD = diagCount < MAX_DIAG ? diagCount : MAX_DIAG;
      for (int i = 0; i < showD; i++) {
        int idx = (diagHead - showD + i + MAX_DIAG) % MAX_DIAG;
        DiagFrame& d = diagFrames[idx];
        html += "<tr>";
        html += "<td class='mono'>" + String((d.t - startTime) / 1000) + "s</td>";
        html += "<td class='rssi'>" + String(d.rssi) + "</td>";
        html += "<td>" + String(d.len) + "</td>";
        html += d.ok ? "<td style='color:#6bcb77'>OK</td>" : "<td style='color:#ffb84d'>zła suma</td>";
        html += "<td class='hex'>" + d.hex + "</td>";
        html += "</tr>";
      }
      html += R"(</table>)";
    }
  }

  // --- Sekcje skanera surowego (tylko w trybie skanu raw) ---
  if (rxMode == MODE_RAW_SCAN) {
  html += R"(
  <h2>Mapa trafień (częstotliwość × profil)</h2>
  <table>
    <tr><th>MHz</th>)";
  for (int p = 0; p < NUM_PROFS; p++) {
    html += "<th>";
    html += PROF_NAMES[p];
    html += "</th>";
  }
  html += "</tr>";
  for (int f = 0; f < NUM_FREQS; f++) {
    html += "<tr><td class='mono'>";
    html += FREQ_NAMES[f];
    html += "</td>";
    for (int p = 0; p < NUM_PROFS; p++) {
      int h = comboHits[f][p];
      html += (h > 0) ? "<td class='hit'>" : "<td class='zero'>";
      html += String(h);
      html += "</td>";
    }
    html += "</tr>";
  }
  html += R"(
  </table>

  <h2>Urządzenia (deduplikowane, maks. )";
  html += String(MAX_DEVICES);
  html += R"( )</h2>)";

  if (deviceCount == 0) {
    html += R"(<div style="background:#16213e;padding:14px;border-radius:8px;color:#888">
      Brak odebranych urządzeń. Radio nasłuchuje - jeżeli w okolicy nadaje
      licznik wMBUS / inny czujnik 868 MHz, pojawi się tu skrócony wpis.
    </div>)";
  } else {
    html += R"(<table>
      <tr><th>Sygnatura</th><th>Ile</th><th>RSSI</th><th>MHz</th><th>Profil</th><th>Len</th><th>Ostatnia ramka</th></tr>)";
    for (int i = 0; i < deviceCount; i++) {
      DeviceEntry& d = devices[i];
      html += "<tr>";
      html += "<td class='mono'>" + d.sig + "</td>";
      html += "<td>" + String(d.count) + "</td>";
      html += "<td class='rssi'>" + String(d.lastRssi) + "</td>";
      html += "<td class='mono'>" + String(FREQ_NAMES[d.freq]) + "</td>";
      html += "<td class='mono'>" + String(PROF_NAMES[d.prof]) + "</td>";
      html += "<td>" + String(d.len) + "</td>";
      html += "<td class='hex'>" + d.lastHex + "</td>";
      html += "</tr>";
    }
    html += R"(</table>)";
  }

  html += R"(<h2>Ostatnie ramki (maks. )";
  html += String(MAX_SHOW_FRAMES);
  html += R"( )</h2>
  <table>
    <tr><th>Czas</th><th>MHz</th><th>Profil</th><th>RSSI</th><th>Len</th><th>Hex</th></tr>)";

  int show = frameCount < MAX_SHOW_FRAMES ? frameCount : MAX_SHOW_FRAMES;
  for (int i = 0; i < show; i++) {
    int idx = (frameHead - show + i + MAX_FRAMES) % MAX_FRAMES;
    RawFrame& f = frames[idx];
    html += "<tr>";
    html += "<td class='mono'>" + String((f.t - startTime) / 1000) + "s</td>";
    html += "<td class='mono'>" + String(FREQ_NAMES[f.freq]) + "</td>";
    html += "<td class='mono'>" + String(PROF_NAMES[f.prof]) + "</td>";
    html += "<td class='rssi'>" + String(f.rssi) + "</td>";
    html += "<td>" + String(f.len) + "</td>";
    html += "<td class='hex'>" + f.hex + "</td>";
    html += "</tr>";
  }

  html += R"(
  </table>)";
  }  // koniec sekcji skanera surowego

  html += R"(<p style="margin-top:12px; color:#6bcb77; font-size:0.9em;">Filtr ramek: )";
  html += filterCfg.enabled ? "WŁĄCZONY" : "WYŁĄCZONY";
  if (filterCfg.enabled) {
    html += " — wzorzec: ";
    html += filterCfg.pattern.length() ? filterCfg.pattern : "(brak, tylko długość)";
    html += ", offset: ";
    html += String(filterCfg.offset);
    html += ", długość: ";
    html += String(filterCfg.minLen);
    html += "..";
    html += String(filterCfg.maxLen);
  }
  html += R"(</p>)";

  if (rxMode == MODE_RAW_SCAN) {
    html += R"(<p style="margin-top:12px; color:#666; font-size:0.85em;">
    Skan: 6 częstotliwości × 8 profili, )";
    html += String(COMBO_DWELL_MS);
    html += R"( ms na kombinację. Zapisuje każdą ramkę z RSSI powyżej )";
    html += String(RSSI_THRESHOLD);
    html += R"( dBm.
  </p>)";
  }

  html += R"(
</body>
</html>)";

  server.send(200, "text/html", html);
}

void handleJson() {
  radioHealthCheck();
  String json = "{\"freq\":\"";
  json += (rxMode == MODE_FINE_OFFSET) ? "868.30" : (rxMode == MODE_VEVOR_7IN1) ? "868.30" : (rxMode == MODE_VEVOR_YT60309) ? "868.35" : (rxMode == MODE_BRESSER) ? "868.30" : String(FREQ_NAMES[currentFreq]);
  json += "\",\"prof\":\"";
  json += (rxMode == MODE_FINE_OFFSET) ? "FSK_17k" : (rxMode == MODE_VEVOR_7IN1 || rxMode == MODE_VEVOR_YT60309) ? "FSK_11k" : (rxMode == MODE_BRESSER) ? "FSK_8k" : String(PROF_NAMES[currentProf]);
  json += "\",\"mode\":\"" + String(rxMode == MODE_FINE_OFFSET ? "fine_offset" : (rxMode == MODE_VEVOR_7IN1 ? "vevor" : (rxMode == MODE_VEVOR_YT60309 ? "vevor_yt60309" : (rxMode == MODE_WEATHER_AUTO ? "auto" : (rxMode == MODE_BRESSER ? "bresser" : "raw_scan"))))) + "\",";
  json += "\"totalFrames\":" + String(totalFrames) + ",";
  json += "\"acceptedFrames\":" + String(acceptedFrames) + ",";
  json += "\"rejectedFrames\":" + String(rejectedFrames) + ",";
  json += "\"filterEnabled\":" + String(filterCfg.enabled ? "true" : "false") + ",";
  json += "\"filterPattern\":\"" + filterCfg.pattern + "\",";
  json += "\"uptime\":" + String((millis() - startTime) / 1000) + ",";

  // Zdekodowane dane pogodowe (po kalibracji — takie jak na stronie WWW)
  if (lastWeatherValid) {
    const WeatherData& w = lastWeatherCal;
    json += "\"weather\":{";
    json += "\"model\":\"" + w.model + "\",";
    json += "\"id\":" + String(w.id) + ",";
    json += "\"battery_ok\":" + String(w.batteryOk ? "true" : "false") + ",";
    if (w.haveTemp) json += "\"temperature_C\":" + String(w.tempC, 1) + ",";
    if (w.haveHum)  json += "\"humidity\":" + String(w.humidity) + ",";
    if (w.haveWindDir) json += "\"wind_dir_deg\":" + String(w.windDirDeg) + ",";
    if (w.haveWind) json += "\"wind_avg_m_s\":" + String(w.windAvgMs, 2) + ",";
    if (w.haveGust) json += "\"wind_max_m_s\":" + String(w.windMaxMs, 2) + ",";
    if (w.haveRain) json += "\"rain_mm\":" + String(w.rainMm, 2) + ",";
    if (w.haveUv)   json += "\"uv\":" + String(w.uv) + ",\"uvi\":" + String(w.uvi) + ",";
    if (w.haveLight) json += "\"light_lux\":" + String(w.lightLux, 0) + ",";
    json += "\"rssi\":" + String(w.rssi) + ",";
    json += "\"hex\":\"" + lastDecodedHex + "\"";
    json += "},";
  }

  json += "\"diagTotal\":" + String(diagTotal) + ",";
  json += "\"radioVersion\":" + String(radioVersion) + ",";
  json += "\"radioOk\":" + String(radioVersion == 0x14 ? "true" : "false") + ",";
  json += "\"lastRawRssi\":" + String(lastRawRssi) + ",";
  json += "\"lastRawHex\":\"" + lastRawHex + "\",";
  json += "\"autoIdx\":" + String(autoIdx) + ",";
  json += "\"autoNow\":\"" + String(AUTO_STEPS_TBL[autoIdx].name) + "\",";
  json += "\"autoSync\":[";
  for (int i = 0; i < AUTO_STEPS; i++) { if (i) json += ","; json += String(autoSync[i]); }
  json += "],\"autoDecoded\":[";
  for (int i = 0; i < AUTO_STEPS; i++) { if (i) json += ","; json += String(autoDecoded[i]); }
  json += "],";

  json += "\"devices\":[";

  for (int i = 0; i < deviceCount; i++) {
    DeviceEntry& d = devices[i];
    if (i > 0) json += ",";
    json += "{\"sig\":\"" + d.sig + "\",";
    json += "\"count\":" + String(d.count) + ",";
    json += "\"rssi\":" + String(d.lastRssi) + ",";
    json += "\"freq\":\"" + String(FREQ_NAMES[d.freq]) + "\",";
    json += "\"prof\":\"" + String(PROF_NAMES[d.prof]) + "\",";
    json += "\"len\":" + String(d.len) + ",";
    json += "\"hex\":\"" + d.lastHex + "\"}";
  }
  json += "],";

  json += "\"frames\":[";


  int n = frameCount;
  for (int i = 0; i < n; i++) {
    int idx = (frameHead - n + i + MAX_FRAMES) % MAX_FRAMES;
    RawFrame& f = frames[idx];
    if (i > 0) json += ",";
    json += "{\"t\":" + String((f.t - startTime) / 1000) + ",";
    json += "\"freq\":\"" + String(FREQ_NAMES[f.freq]) + "\",";
    json += "\"prof\":\"" + String(PROF_NAMES[f.prof]) + "\",";
    json += "\"rssi\":" + String(f.rssi) + ",";
    json += "\"lqi\":" + String(f.lqi) + ",";
    json += "\"len\":" + String(f.len) + ",";
    json += "\"hex\":\"" + f.hex + "\"}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleClear() {
  frameHead = 0;
  frameCount = 0;
  totalFrames = 0;
  acceptedFrames = 0;
  rejectedFrames = 0;
  memset(comboHits, 0, sizeof(comboHits));
  lastWeatherValid = false;
  lastDecodedHex = "";
  rainBaseline = -1.0f;
  deviceCount = 0;
  diagHead = 0;
  diagCount = 0;
  diagTotal = 0;
  lastRawHex = "";
  lastRawRssi = 0;
  startTime = millis();
  server.send(200, "text/plain", "Cleared");
}

// ==================== USTAWIENIA WIFI (NVS) ====================
Preferences wifiPrefs;

void loadWifiCfg() {
  wifiPrefs.begin("wifi", false);

  // Pierwsze uruchomienie - zapisz domyślne wartości, żeby uniknąć błędów NVS.
  if (!wifiPrefs.getBool("init", false)) {
    wifiPrefs.putString("apSsid", AP_SSID_DEFAULT);
    wifiPrefs.putString("apPass", AP_PASS_DEFAULT);
    wifiPrefs.putString("ssid", DEF_STA_SSID);
    wifiPrefs.putString("pass", DEF_STA_PASS);
    wifiPrefs.putBool("dhcp", DEF_STA_DHCP);
    wifiPrefs.putString("ip", DEF_STA_IP);
    wifiPrefs.putString("gw", DEF_STA_GW);
    wifiPrefs.putString("mask", DEF_STA_MASK);
    wifiPrefs.putString("dns", DEF_STA_DNS);
    wifiPrefs.putBool("init", true);
  }

  // Migracja: nadpisz klucze sieciowe starszych wersji domyślnymi wartościami.
  if (!wifiPrefs.getBool("init2", false)) {
    wifiPrefs.putString("ssid", DEF_STA_SSID);
    wifiPrefs.putString("pass", DEF_STA_PASS);
    wifiPrefs.putBool("dhcp", DEF_STA_DHCP);
    wifiPrefs.putBool("init2", true);
  }

  wifiCfg.apSsid  = wifiPrefs.getString("apSsid", AP_SSID_DEFAULT);
  wifiCfg.apPass  = wifiPrefs.getString("apPass", AP_PASS_DEFAULT);
  wifiCfg.staSsid = wifiPrefs.getString("ssid", DEF_STA_SSID);
  wifiCfg.staPass = wifiPrefs.getString("pass", DEF_STA_PASS);
  wifiCfg.useDhcp = wifiPrefs.getBool("dhcp", DEF_STA_DHCP);
  wifiCfg.ip   = wifiPrefs.getString("ip", DEF_STA_IP);
  wifiCfg.gw   = wifiPrefs.getString("gw", DEF_STA_GW);
  wifiCfg.mask = wifiPrefs.getString("mask", DEF_STA_MASK);
  wifiCfg.dns  = wifiPrefs.getString("dns", DEF_STA_DNS);
  wifiPrefs.end();
}

void saveWifiCfg() {
  wifiPrefs.begin("wifi", false);
  wifiPrefs.putString("apSsid", wifiCfg.apSsid);
  wifiPrefs.putString("apPass", wifiCfg.apPass);
  wifiPrefs.putString("ssid", wifiCfg.staSsid);
  wifiPrefs.putString("pass", wifiCfg.staPass);
  wifiPrefs.putBool("dhcp", wifiCfg.useDhcp);
  wifiPrefs.putString("ip", wifiCfg.ip);
  wifiPrefs.putString("gw", wifiCfg.gw);
  wifiPrefs.putString("mask", wifiCfg.mask);
  wifiPrefs.putString("dns", wifiCfg.dns);
  wifiPrefs.end();
}

void applyWifi() {
  // AP zawsze włączony + równoległa próba połączenia z siecią kliencką.
  WiFi.mode(WIFI_AP_STA);

  // Obniż moc nadawczą WiFi. Pełna moc ESP32 (~20 dBm) wstrzykuje krótkie
  // skoki szumu do CC1101 (-74 dBm) na każdej częstotliwości i maskuje
  // słabsze sygnały stacji pogodowej. Niska moc nadal starcza na domowy
  // zasięg, a radia przestaje "głuszyć".
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  if (wifiCfg.apPass.length() >= 8) {
    WiFi.softAP(wifiCfg.apSsid.c_str(), wifiCfg.apPass.c_str());
  } else {
    WiFi.softAP(wifiCfg.apSsid.c_str());  // AP otwarty (hasło za krótkie)
  }

  if (wifiCfg.staSsid.length() == 0) {
    Serial.println("Brak skonfigurowanego SSID STA - praca tylko w trybie AP.");
    return;
  }

  if (wifiCfg.useDhcp) {
    WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));  // przywróć DHCP
  } else {
    IPAddress ip, gw, mask, dns;
    if (ip.fromString(wifiCfg.ip) && gw.fromString(wifiCfg.gw) && mask.fromString(wifiCfg.mask)) {
      if (dns.fromString(wifiCfg.dns)) {
        WiFi.config(ip, gw, mask, dns);
      } else {
        WiFi.config(ip, gw, mask);
      }
    } else {
      Serial.println("Blad adresu IP w ustawieniach - uzywam DHCP.");
      WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
    }
  }

  WiFi.begin(wifiCfg.staSsid.c_str(), wifiCfg.staPass.c_str());
}

String htmlEscape(const String& s) {
  String r;
  r.reserve(s.length() + 16);
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': r += "&amp;"; break;
      case '<': r += "&lt;"; break;
      case '>': r += "&gt;"; break;
      case '"': r += "&quot;"; break;
      case '\'': r += "&#39;"; break;
      default: r += c; break;
    }
  }
  return r;
}

// ==================== STRONA USTAWIEŃ ====================
void handleSetup() {
  String h = R"(<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Ustawienia - Weather Sniffer</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
           background: #1a1a2e; color: #eee; padding: 16px; }
    h1 { color: #00d9ff; margin-bottom: 10px; font-size: 1.3em; }
    .card { background: #16213e; padding: 16px; border-radius: 8px; margin-bottom: 16px; }
    h2 { color: #00d9ff; margin-bottom: 12px; font-size: 1.05em; }
    label { display: block; margin: 10px 0 4px; color: #bbb; font-size: 0.9em; }
    input { width: 100%; max-width: 420px; padding: 8px; border-radius: 6px;
            border: 1px solid #0f3460; background: #0f1b3d; color: #eee; font-size: 0.95em; }
    input[type=checkbox] { width: auto; }
    .row { display: flex; align-items: center; gap: 8px; margin-top: 10px; }
    .row label { margin: 0; }
    button { margin-top: 16px; background: #00d9ff; color: #0f1b3d; border: 0;
             padding: 10px 22px; border-radius: 6px; font-size: 1em; font-weight: bold; cursor: pointer; }
    a { color: #00d9ff; text-decoration: none; margin-right: 16px; }
    .hint { color: #888; font-size: 0.8em; margin-top: 4px; }
    .grid { display: flex; flex-wrap: wrap; gap: 12px; margin-top: 6px; }
    .grid div { width: 160px; }
    .grid input { width: 160px; }
  </style>
</head>
<body>
  <h1>Konfiguracja sieci</h1>
  <div style="margin-bottom:14px">
    <a href="/">📡 Odczyt</a>
    <a href="/setup">⚙️ Ustawienia</a>
    <a href="/cal">📐 Kalibracja</a>
    <a href="/send">📤 Wysyłanie</a>
    <a href="/mqtt">🔌 MQTT</a>
    <a href="/json">JSON</a>
    <a href="/clear" style="color:#ff6b6b">Wyczyść</a>
  </div>
  <form method="post" action="/save">
    <div class="card">
      <h2>Punkt dostepowy (AP) - zawsze aktywny</h2>
      <label>Nazwa sieci AP (SSID)</label>
      <input type="text" name="apSsid" value=")";
  h += htmlEscape(wifiCfg.apSsid);
  h += R"(">
      <label>Haslo AP (min. 8 znakow)</label>
      <input type="text" name="apPass" value=")";
  h += htmlEscape(wifiCfg.apPass);
  h += R"(">
    </div>
    <div class="card">
      <h2>Sieć kliencka (STA)</h2>
      <label>SSID sieci Wi-Fi</label>
      <input type="text" name="ssid" value=")";
  h += htmlEscape(wifiCfg.staSsid);
  h += R"(">
      <label>Haslo sieci Wi-Fi</label>
      <input type="password" name="pass" value=")";
  h += htmlEscape(wifiCfg.staPass);
  h += R"(">
      <div class="row">
        <input type="checkbox" name="staticip" id="staticip" )";
  h += wifiCfg.useDhcp ? "" : "checked";
  h += R"(>
        <label for="staticip">Uzyj stalego adresu IP</label>
      </div>
      <div class="grid">
        <div><label>Adres IP</label><input type="text" name="ip" value=")";
  h += htmlEscape(wifiCfg.ip);
  h += R"("></div>
        <div><label>Maska</label><input type="text" name="mask" value=")";
  h += htmlEscape(wifiCfg.mask);
  h += R"("></div>
        <div><label>Brama</label><input type="text" name="gw" value=")";
  h += htmlEscape(wifiCfg.gw);
  h += R"("></div>
        <div><label>DNS</label><input type="text" name="dns" value=")";
  h += htmlEscape(wifiCfg.dns);
  h += R"("></div>
      </div>
      <div class="hint">Staly adres IP dziala tylko w sieci klienckiej (STA). Gdy opcja jest odznaczona, uzywane jest DHCP.</div>
    </div>
    <div class="card">
      <h2>Tryb odbioru</h2>
      <label>Tryb pracy radia</label>
      <select name="rxMode" style="width:100%;max-width:420px;padding:8px;border-radius:6px;border:1px solid #0f3460;background:#0f1b3d;color:#eee;font-size:0.95em">
        <option value="raw" )";
  h += (rxMode == MODE_RAW_SCAN) ? "selected" : "";
  h += R"(>Skan raw (wiele profili, mapa trafień)</option>
        <option value="auto" )";
  h += (rxMode == MODE_WEATHER_AUTO) ? "selected" : "";
  h += R"(>AUTO — próbuj wszystkie protokoły stacji (zalecane)</option>
        <option value="vevor_yt60309" )";
  h += (rxMode == MODE_VEVOR_YT60309) ? "selected" : "";
  h += R"(>VEVOR YT60309 (868.35 MHz, Twoja stacja)</option>
        <option value="fo" )";
  h += (rxMode == MODE_FINE_OFFSET) ? "selected" : "";
  h += R"(>Fine Offset / WH65 (868.30 MHz, dekoder pogody)</option>
        <option value="vevor" )";
  h += (rxMode == MODE_VEVOR_7IN1) ? "selected" : "";
  h += R"(>VEVOR / Youtong 7-in-1 (868.30 MHz, protokół 263)</option>
        <option value="bresser" )";
  h += (rxMode == MODE_BRESSER) ? "selected" : "";
  h += R"(>Bresser 5/6/7-in-1 (868.30 MHz, dekoder pogody)</option>
      </select>
      <div class="hint">„AUTO” po kolei próbuje wszystkich protokołów stacji pogodowych (po 20 s każdy) — zalecane na start, pokaże który protokół pasuje. „VEVOR / Youtong 7-in-1” nasluchuje 868.30 MHz (sync CA 54, protokół 263) — to pasuje do Twojej stacji. „VEVOR YT60309” nasluchuje 868.35 MHz (sync C0AA C0AA). „Fine Offset / WH65” nasluchuje 868.30 MHz. „Bresser” nasluchuje 868.30 MHz (FSK ~8.2 kbaud). „Skan raw” przeszukuje czestotliwosci i profile.</div>
    </div>
    <div class="card">
      <h2>Filtr ramek (ID urzadzenia)</h2>
      <div class="row">
        <input type="checkbox" name="filtEn" id="filtEn" )";
  h += filterCfg.enabled ? "checked" : "";
  h += R"(>
        <label for="filtEn">Wlacz filtr (zapisuj tylko pasujace ramki)</label>
      </div>
      <label>Wzorzec hex (np. ID urzadzenia)</label>
      <input type="text" name="filtPat" value=")";
  h += htmlEscape(filterCfg.pattern);
  h += R"(">
      <div class="hint">Pusty wzorzec = filtruj tylko po dlugosci ramki. Hex, spacje myslniki dozwolone.</div>
      <div class="grid">
        <div><label>Offset (bajt startu)</label><input type="number" name="filtOff" value=")";
  h += String(filterCfg.offset);
  h += R"("></div>
        <div><label>Min. dlugosc</label><input type="number" name="filtMin" value=")";
  h += String(filterCfg.minLen);
  h += R"("></div>
        <div><label>Max. dlugosc (0=bez)</label><input type="number" name="filtMax" value=")";
  h += String(filterCfg.maxLen);
  h += R"("></div>
      </div>
      <div class="hint">Offset -1 = szukaj wzorca w calej ramce. Offset >= 0 = wzorzec musi byc dokladnie na tym bajcie (liczac od 0).</div>
    </div>
    <button type="submit">Zapisz i zastosuj</button>
  </form>
  <p style="margin-top:14px; color:#666; font-size:0.85em">
    Po zapisaniu urzadzenie przełączy sieć. Punkt dostepowy dziala zawsze - domyslnie pod adresem 192.168.4.1.
  </p>
</body>
</html>)";
  server.send(200, "text/html", h);
}

void handleSave() {
  if (server.hasArg("apSsid")) wifiCfg.apSsid = server.arg("apSsid");
  if (server.hasArg("apPass")) wifiCfg.apPass = server.arg("apPass");
  if (server.hasArg("ssid"))   wifiCfg.staSsid = server.arg("ssid");
  if (server.hasArg("pass"))   wifiCfg.staPass = server.arg("pass");
  // Zaznaczony checkbox "staticip" => stały adres IP
  wifiCfg.useDhcp = !server.hasArg("staticip");
  if (server.hasArg("ip"))   wifiCfg.ip   = server.arg("ip");
  if (server.hasArg("gw"))   wifiCfg.gw   = server.arg("gw");
  if (server.hasArg("mask")) wifiCfg.mask = server.arg("mask");
  if (server.hasArg("dns"))  wifiCfg.dns  = server.arg("dns");

  // Filtr ramek
  filterCfg.enabled = server.hasArg("filtEn");
  if (server.hasArg("filtPat")) filterCfg.pattern = server.arg("filtPat");
  filterCfg.offset = server.hasArg("filtOff") ? server.arg("filtOff").toInt() : -1;
  filterCfg.minLen = server.hasArg("filtMin") ? server.arg("filtMin").toInt() : 0;
  filterCfg.maxLen = server.hasArg("filtMax") ? server.arg("filtMax").toInt() : 0;
  saveFilterCfg();

  // Tryb odbioru
  RxMode newMode = MODE_RAW_SCAN;
  if (server.arg("rxMode") == "fo") newMode = MODE_FINE_OFFSET;
  else if (server.arg("rxMode") == "vevor") newMode = MODE_VEVOR_7IN1;
  else if (server.arg("rxMode") == "vevor_yt60309") newMode = MODE_VEVOR_YT60309;
  else if (server.arg("rxMode") == "bresser") newMode = MODE_BRESSER;
  else if (server.arg("rxMode") == "auto") newMode = MODE_WEATHER_AUTO;
  bool modeChanged = (newMode != rxMode);
  rxMode = newMode;
  saveRxMode();

  // Reset stanu dekodera
  lastWeatherValid = false;
  lastDecodedHex = "";
  rainBaseline = -1.0f;

  // Po zmianie filtra/trybu zaczynamy liczenie od nowa
  frameHead = 0;
  frameCount = 0;
  totalFrames = 0;
  acceptedFrames = 0;
  rejectedFrames = 0;
  deviceCount = 0;
  diagHead = 0;
  diagCount = 0;
  diagTotal = 0;
  lastRawHex = "";
  lastRawRssi = 0;
  startTime = millis();

  if (modeChanged) {
    if (rxMode == MODE_FINE_OFFSET) {
      applyFineOffsetMode();
    } else if (rxMode == MODE_VEVOR_7IN1) {
      applyVevorMode();
    } else if (rxMode == MODE_VEVOR_YT60309) {
      applyVevorYT60309Mode();
    } else if (rxMode == MODE_WEATHER_AUTO) {
      autoIdx = 0;
      autoChange = 0;
      applyAutoStep(0);
    } else if (rxMode == MODE_BRESSER) {
      applyBresserMode();
    } else {
      applyCombo(0, 0);
    }
  }

  saveWifiCfg();
  applyWifi();

  String msg = R"(<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta http-equiv="refresh" content="3;url=/">
  <title>Zapisano</title>
</head>
<body style="background:#1a1a2e;color:#eee;font-family:sans-serif;padding:24px">
  <h2 style="color:#00d9ff">Ustawienia zapisane</h2>
  <p>Przełączam sieć Wi-Fi... Za chwilę nastąpi powrót na stronę główną.</p>
</body>
</html>)";
  server.send(200, "text/html", msg);
}

// ==================== STRONY KONFIGURACYJNE (Kalibracja / Wysyłanie / MQTT) ====================
String cfgPageStart(const String& title, const String& h1) {
  String h = R"(<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>)";
  h += title;
  h += R"(</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
           background: #1a1a2e; color: #eee; padding: 16px; }
    h1 { color: #00d9ff; margin-bottom: 10px; font-size: 1.3em; }
    .card { background: #16213e; padding: 16px; border-radius: 8px; margin-bottom: 16px; }
    h2 { color: #00d9ff; margin-bottom: 12px; font-size: 1.05em; }
    label { display: block; margin: 10px 0 4px; color: #bbb; font-size: 0.9em; }
    input, select { width: 100%; max-width: 420px; padding: 8px; border-radius: 6px;
            border: 1px solid #0f3460; background: #0f1b3d; color: #eee; font-size: 0.95em; }
    input[type=checkbox] { width: auto; }
    .row { display: flex; align-items: center; gap: 8px; margin-top: 10px; }
    .row label { margin: 0; }
    button { margin-top: 16px; background: #00d9ff; color: #0f1b3d; border: 0;
             padding: 10px 22px; border-radius: 6px; font-size: 1em; font-weight: bold; cursor: pointer; }
    a { color: #00d9ff; text-decoration: none; margin-right: 14px; }
    .hint { color: #888; font-size: 0.8em; margin-top: 4px; }
    .grid { display: flex; flex-wrap: wrap; gap: 12px; margin-top: 6px; }
    .grid div { width: 160px; }
    .grid input { width: 160px; }
    .msg { background: #16213e; padding: 14px; border-radius: 8px; color: #6bcb77; margin-bottom: 16px; }
  </style>
</head>
<body>
  <h1>)";
  h += h1;
  h += R"(</h1>
  <div style="margin-bottom:12px">
    <a href="/">📡 Odczyt</a>
    <a href="/setup">⚙️ Ustawienia</a>
    <a href="/cal">📐 Kalibracja</a>
    <a href="/send">📤 Wysyłanie</a>
    <a href="/mqtt">🔌 MQTT</a>
    <a href="/json">JSON</a>
    <a href="/clear" style="color:#ff6b6b">Wyczyść</a>
  </div>)";
  return h;
}

String cfgPageEnd() {
  return R"(</body>
</html>)";
}

void handleCal() {
  String h = cfgPageStart("Kalibracja - Weather Sniffer", "📐 Kalibracja czujników");
  h += R"(<form method="post" action="/savecal">
    <div class="card">
      <h2>Korekta wskazań (stosowana po dekodowaniu)</h2>
      <div class="hint">Wzór: y = x · factor + offset. Factor 1.0 i offset 0 = bez zmian.</div>
      <div class="grid">
        <div><label>Temp. offset [°C]</label><input type="number" step="0.1" name="tempOffset" value=")";
  h += String(calCfg.tempOffset, 1);
  h += R"("></div>
        <div><label>Wilgotność offset [%]</label><input type="number" step="0.1" name="humOffset" value=")";
  h += String(calCfg.humOffset, 1);
  h += R"("></div>
        <div><label>Wiatr × factor</label><input type="number" step="0.01" name="windFactor" value=")";
  h += String(calCfg.windFactor, 3);
  h += R"("></div>
        <div><label>Porywy × factor</label><input type="number" step="0.01" name="gustFactor" value=")";
  h += String(calCfg.gustFactor, 3);
  h += R"("></div>
        <div><label>Deszcz × factor</label><input type="number" step="0.01" name="rainFactor" value=")";
  h += String(calCfg.rainFactor, 3);
  h += R"("></div>
        <div><label>Światło × factor</label><input type="number" step="0.01" name="lightFactor" value=")";
  h += String(calCfg.lightFactor, 3);
  h += R"("></div>
        <div><label>Kierunek wiatru offset [°]</label><input type="number" name="windDirOffset" value=")";
  h += String(calCfg.windDirOffset);
  h += R"("></div>
      </div>
    </div>
    <button type="submit">Zapisz kalibrację</button>
  </form>)";
  h += cfgPageEnd();
  server.send(200, "text/html", h);
}

void handleSend() {
  String h = cfgPageStart("Wysyłanie - Weather Sniffer", "📤 Wysyłanie danych");
  h += R"(<form method="post" action="/savesend">
    <div class="card">
      <h2>HTTP (POST JSON do innego urządzenia)</h2>
      <div class="row">
        <input type="checkbox" name="wifiEnabled" id="wifiEnabled" )";
  h += sendCfg.wifiEnabled ? "checked" : "";
  h += R"(>
        <label for="wifiEnabled">Włącz wysyłkę HTTP po każdym odbiorze</label>
      </div>
      <label>Adres URL odbiorcy</label>
      <input type="text" name="targetUrl" value=")";
  h += htmlEscape(sendCfg.targetUrl);
  h += R"(">
      <div class="hint">Np. http://192.168.1.50:8080/weather — dane wysyłane jako JSON (POST).</div>
    </div>
    <div class="card">
      <h2>RS485 (UART1)</h2>
      <div class="row">
        <input type="checkbox" name="rs485Enabled" id="rs485Enabled" )";
  h += sendCfg.rs485Enabled ? "checked" : "";
  h += R"(>
        <label for="rs485Enabled">Włącz wysyłkę RS485 (linia JSON zakończona znakiem nowej linii)</label>
      </div>
      <div class="grid">
        <div><label>TX pin</label><input type="number" name="rs485TxPin" value=")";
  h += String(sendCfg.rs485TxPin);
  h += R"("></div>
        <div><label>RX pin</label><input type="number" name="rs485RxPin" value=")";
  h += String(sendCfg.rs485RxPin);
  h += R"("></div>
        <div><label>Baud</label><input type="number" name="rs485Baud" value=")";
  h += String(sendCfg.rs485Baud);
  h += R"("></div>
        <div><label>DE/RE pin (-1 = brak)</label><input type="number" name="rs485DePin" value=")";
  h += String(sendCfg.rs485DePin);
  h += R"("></div>
      </div>
    </div>
    <button type="submit">Zapisz wysyłanie</button>
  </form>
  <form method="post" action="/sendtest" style="margin-top:8px">
    <button type="submit">Wyślij testowy pakiet (ostatnie dane)</button>
  </form>)";
  h += cfgPageEnd();
  server.send(200, "text/html", h);
}

void handleMqtt() {
  String h = cfgPageStart("MQTT - Weather Sniffer", "🔌 MQTT (Home Assistant)");
  h += R"(<form method="post" action="/savemqtt">
    <div class="card">
      <h2>Broker MQTT</h2>
      <div class="row">
        <input type="checkbox" name="enabled" id="enabled" )";
  h += mqttCfg.enabled ? "checked" : "";
  h += R"(>
        <label for="enabled">Włącz MQTT</label>
      </div>
      <label>Adres brokera</label>
      <input type="text" name="broker" value=")";
  h += htmlEscape(mqttCfg.broker);
  h += R"(">
      <label>Port</label>
      <input type="number" name="port" value=")";
  h += String(mqttCfg.port);
  h += R"(">
      <label>Użytkownik (opcjonalnie)</label>
      <input type="text" name="user" value=")";
  h += htmlEscape(mqttCfg.user);
  h += R"(">
      <label>Hasło (opcjonalnie)</label>
      <input type="password" name="pass" value=")";
  h += htmlEscape(mqttCfg.pass);
  h += R"(">
      <label>Prefiks tematu</label>
      <input type="text" name="topicPrefix" value=")";
  h += htmlEscape(mqttCfg.topicPrefix);
  h += R"(">
      <div class="row">
        <input type="checkbox" name="haDiscovery" id="haDiscovery" )";
  h += mqttCfg.haDiscovery ? "checked" : "";
  h += R"(>
        <label for="haDiscovery">Publikuj auto-odkrycie Home Assistant</label>
      </div>
    </div>
    <button type="submit">Zapisz MQTT</button>
  </form>)";
  h += cfgPageEnd();
  server.send(200, "text/html", h);
}

void handleSaveCal() {
  calCfg.tempOffset    = server.hasArg("tempOffset")    ? server.arg("tempOffset").toFloat() : 0.0f;
  calCfg.humOffset     = server.hasArg("humOffset")     ? server.arg("humOffset").toFloat() : 0.0f;
  calCfg.windFactor    = server.hasArg("windFactor")    ? server.arg("windFactor").toFloat() : 1.0f;
  calCfg.gustFactor    = server.hasArg("gustFactor")    ? server.arg("gustFactor").toFloat() : 1.0f;
  calCfg.rainFactor    = server.hasArg("rainFactor")    ? server.arg("rainFactor").toFloat() : 1.0f;
  calCfg.lightFactor   = server.hasArg("lightFactor")   ? server.arg("lightFactor").toFloat() : 1.0f;
  calCfg.windDirOffset = server.hasArg("windDirOffset") ? server.arg("windDirOffset").toInt() : 0;
  saveCalCfg();
  if (lastWeatherValid) lastWeatherCal = calibrateWeather(lastWeather);

  String msg = cfgPageStart("Zapisano", "✅ Kalibracja zapisana");
  msg += R"(<div class="msg">Zapisano. <a href="/cal">Wróć do kalibracji</a> · <a href="/">Odczyt</a></div>)";
  msg += cfgPageEnd();
  server.send(200, "text/html", msg);
}

void handleSaveSend() {
  sendCfg.wifiEnabled  = server.hasArg("wifiEnabled");
  if (server.hasArg("targetUrl")) sendCfg.targetUrl = server.arg("targetUrl");
  sendCfg.rs485Enabled = server.hasArg("rs485Enabled");
  sendCfg.rs485TxPin   = server.hasArg("rs485TxPin") ? server.arg("rs485TxPin").toInt() : 17;
  sendCfg.rs485RxPin   = server.hasArg("rs485RxPin") ? server.arg("rs485RxPin").toInt() : 18;
  sendCfg.rs485Baud    = server.hasArg("rs485Baud")  ? server.arg("rs485Baud").toInt() : 9600;
  sendCfg.rs485DePin   = server.hasArg("rs485DePin") ? server.arg("rs485DePin").toInt() : -1;
  saveSendCfg();
  applySend();

  String msg = cfgPageStart("Zapisano", "✅ Wysyłanie zapisane");
  msg += R"(<div class="msg">Zapisano. <a href="/send">Wróć do wysyłania</a> · <a href="/">Odczyt</a></div>)";
  msg += cfgPageEnd();
  server.send(200, "text/html", msg);
}

void handleSaveMqtt() {
  mqttCfg.enabled     = server.hasArg("enabled");
  if (server.hasArg("broker"))      mqttCfg.broker = server.arg("broker");
  mqttCfg.port        = server.hasArg("port") ? server.arg("port").toInt() : 1883;
  if (server.hasArg("user"))        mqttCfg.user = server.arg("user");
  if (server.hasArg("pass"))        mqttCfg.pass = server.arg("pass");
  if (server.hasArg("topicPrefix")) mqttCfg.topicPrefix = server.arg("topicPrefix");
  mqttCfg.haDiscovery = server.hasArg("haDiscovery");
  saveMqttCfg();
  applyMqtt();

  String msg = cfgPageStart("Zapisano", "✅ MQTT zapisane");
  msg += R"(<div class="msg">Zapisano. <a href="/mqtt">Wróć do MQTT</a> · <a href="/">Odczyt</a></div>)";
  msg += cfgPageEnd();
  server.send(200, "text/html", msg);
}

void handleSendTest() {
  String msg = cfgPageStart("Test wysyłki", "📤 Test wysyłki");
  if (!lastWeatherValid) {
    msg += R"(<div class="msg" style="color:#ffb84d">Brak danych pogodowych do wysłania.
      Poczekaj na odebranie ramki (stacja nadaje co ~16–20 s), potem spróbuj ponownie.</div>)";
  } else {
    publishWeather();
    msg += R"(<div class="msg">Wysłano ostatnie dane (HTTP / RS485 / MQTT — według włączonych kanałów).
      Sprawdź odbiorcę lub log szeregowy (Serial).</div>)";
  }
  msg += R"(<div><a href="/send">Wróć do wysyłania</a> · <a href="/">Odczyt</a></div>)";
  msg += cfgPageEnd();
  server.send(200, "text/html", msg);
}

// ==================== DETEKTOR PACZEK (diagnostyka, wysoka rozdzielczosc) ====================
// Sonda RSSI probkuje raz na sekunde: paczka 85 ms daje ~8% szans na trafienie,
// wiec brak wyniku nic nie dowodzi. Ten tryb probkuje RSSI w ciasnej petli
// (tysiace razy na sekunde) i mierzy realna dlugosc paczki oraz odstepy miedzy
// nimi. Nadajnik stacji pogodowej = krotkie paczki (kilkadziesiat ms) powtarzane
// cyklicznie (co ~16-20 s). Zaklócenie = pojedyncze, nieregularne skoki.
// Odczyt FIFO odbiornika. dst == NULL -> tylko oproznij (trzyma RSSI zywym).
static int ccReadFifo(uint8_t* dst, int maxBytes) {
  uint8_t rb = ccStatusRead(0x3B) & 0x7F;
  if (!rb) return 0;
  int n = rb < maxBytes ? rb : maxBytes;
  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0xFF);  // burst read RXFIFO
  for (int i = 0; i < n; i++) {
    uint8_t v = (uint8_t)ccXfer(0);
    if (dst) dst[i] = v;
  }
  radioSPI.endTransaction();
  csHigh();
  return n;
}

String burstScan(float mhz, int secs, bool wide, int agc) {
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX

  uint32_t w = (uint32_t)((double)mhz * 1e6 * 65536.0 / 26e6 + 0.5);
  ccWrite(0x0D, (w >> 16) & 0xFF);
  ccWrite(0x0E, (w >> 8) & 0xFF);
  ccWrite(0x0F, w & 0xFF);

  ccWrite(0x10, wide ? 0x43 : 0x98);  // MDMCFG4: 812 kHz albo 162.5 kHz
  ccWrite(0x11, wide ? 0x83 : 0xC0);
  ccWrite(0x12, 0x00);  // MDMCFG2: 2-FSK, bez sync word
  ccWrite(0x13, 0x22);
  ccWrite(0x14, 0xF8);
  ccWrite(0x15, 0x44);  // DEVIATN ~38 kHz
  ccWrite(0x1B, (uint8_t)agc);  // AGCCTRL2 - domyslnie 0x03 (maks. czulosc)
  ccWrite(0x00, 0x0E);  // IOCFG2 = carrier sense (sprzetowy detektor nosnej)
  ccWrite(0x06, 0xFF);
  ccWrite(0x07, 0x00);
  // PKTCTRL0 = 0x02: dlugosc nieskonczona. Dzieki temu radio nie konczy paczki
  // co 255 bajtow i nie przelacza sie IDLE - inaczej pomiar mialby sztuczna
  // okresowosc ~165 ms.
  ccWrite(0x08, 0x02);

  delay(3);
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x33);  // SCAL - kalibracja syntezera/front-endu przed nasluchem
  delay(2);
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX
  delay(5);

  // Faza 1: mediana z 256 probek = prawdziwy poziom szumu. Wlasnie mediana,
  // a nie maksimum: wlasne WiFi ESP32 co jakis czas wstrzykuje krotkie skoki
  // RSSI az do -74 dBm na KAZDEJ czestotliwosci i przy kazdym AGC. Progi
  // liczone z maksimum powodowaly, ze te skoki byly brane za sygnal stacji.
  int base[256];
  for (int i = 0; i < 256; i++) {
    base[i] = calculateRSSI(ccStatusRead(0x34));
    ccReadFifo(NULL, 64);
  }
  for (int i = 0; i < 255; i++)
    for (int j = 0; j < 255 - i; j++)
      if (base[j] > base[j + 1]) { int t = base[j]; base[j] = base[j + 1]; base[j + 1] = t; }
  const int noiseMed = base[128];
  const int noiseMin = base[0];

  const int thrHi = noiseMed + 10;
  const int thrLo = noiseMed + 6;

  // Faza 2: nasluch z histereza + kontrola zywotnosci pomiaru.
  // rssiZmian > 0 dowodzi, ze rejestr RSSI naprawde sie aktualizuje (nie zamarzl).
  // cs to sprzetowy carrier sense z GDO2 - niezalezny od rejestru RSSI.
  // FIFO oprozniamy na biezaco: przy dlugosci nieskonczonej FIFO sie przelewa,
  // radio wchodzi w MARCSTATE=RXFIFO_OVERFLOW i RSSI przestaje sie zmieniac
  // (to wlasnie falszowalo wczesniejsze pomiary).
  int maxR = -200, minR = 200, prevR = -999;
  long samples = 0, rssiChanged = 0, csPulses = 0, csHighN = 0;
  long rxChecks = 0, rxOk = 0, drained = 0;
  int csPrev = digitalRead(PIN_GDO2);
  int bursts = 0;
  bool inBurst = false;
  unsigned long burstStart = 0, lastEnd = 0;
  String durs = "", gaps = "";

  unsigned long endUs = micros() + (unsigned long)secs * 1000000UL;
  while (true) {
    int r = calculateRSSI(ccStatusRead(0x34));
    samples++;
    if (r > maxR) maxR = r;
    if (r < minR) minR = r;
    if (r != prevR) { rssiChanged++; prevR = r; }

    int cs = digitalRead(PIN_GDO2);
    if (cs != csPrev) { csPulses++; csPrev = cs; }
    if (cs) csHighN++;

    if (inBurst) {
      if (r < thrLo) {
        inBurst = false;
        unsigned long stop = micros();
        if (bursts <= 10) durs += String((stop - burstStart) / 1000UL) + "ms ";
        lastEnd = stop;
      }
    } else if (r > thrHi) {
      inBurst = true;
      bursts++;
      burstStart = micros();
      if (lastEnd && bursts <= 10) gaps += String((burstStart - lastEnd) / 1000UL) + "ms ";
    }

    if ((samples & 0x7F) == 0) {
      rxChecks++;
      if ((ccStatusRead(0x35) & 0x1F) == 0x0D) {
        rxOk++;
      } else {
        ccStrobe(0x3A);  // SFRX
        ccStrobe(0x34);  // SRX - wroc do odbioru
      }
      uint8_t rb = ccStatusRead(0x3B) & 0x7F;
      if (rb) {
        int n2 = rb < 64 ? rb : 64;
        csLow();
        radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
        ccXfer(0xFF);  // burst read RXFIFO
        for (int i = 0; i < n2; i++) ccXfer(0);
        radioSPI.endTransaction();
        csHigh();
        drained += n2;
      }
      if (micros() >= endUs) break;
    }
  }
  if (inBurst && bursts <= 10) durs += String((micros() - burstStart) / 1000UL) + "ms ";

  uint8_t marc = ccStatusRead(0x35) & 0x1F;

  String s = "BURST " + String(mhz, 2) + "MHz ";
  s += wide ? "BW812k " : "BW162k ";
  s += "agc=0x" + String(agc, HEX) + " ";
  s += "szumMed=" + String(noiseMed) + " szumMin=" + String(noiseMin) + " prog=" + String(thrHi) + " max=" + String(maxR) + " min=" + String(minR) + "dBm ";
  s += "rssiZmian=" + String(rssiChanged) + " ";
  s += "marc=0x" + String(marc, HEX) + " ";
  s += "RX=" + String(rxChecks ? (100UL * rxOk / rxChecks) : 0) + "% ";
  s += "bajtow=" + String(drained) + " ";
  s += "CS=" + String(csPulses) + "imp/" + String(samples ? (100UL * csHighN / samples) : 0) + "% ";
  s += "probek=" + String(samples) + " ";
  s += "paczek=" + String(bursts);
  if (durs.length()) s += " dlugosc=[" + durs + "]";
  if (gaps.length()) s += " odstep=[" + gaps + "]";
  Serial.println(s);
  return s;
}

void handleBurst() {
  float mhz = server.hasArg("mhz") ? server.arg("mhz").toFloat() : 868.40;
  if (mhz < 300.0 || mhz > 1000.0) {
    server.send(400, "text/plain", "Zakres 300..1000 MHz");
    return;
  }
  int secs = server.hasArg("sec") ? server.arg("sec").toInt() : 30;
  if (secs < 5) secs = 5;
  if (secs > 300) secs = 300;

  // Opcjonalnie wylacz WiFi na czas pomiaru: wlasne WiFi ESP32 wstrzykuje
  // krotkie skoki RSSI (~ -74 dBm) na kazdej czestotliwosci i maskuje
  // prawdziwe sygnaly. Przy wylaczonym WiFi pomiar jest czysty.
  bool wifiWasOn = (WiFi.getMode() != WIFI_OFF);
  bool wifiOff = server.hasArg("wifi") && server.arg("wifi") == "0";
  if (wifiOff && wifiWasOn) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
  }

  probeFreqIdx = -1;
  int agc = server.hasArg("agc") ? (int)strtol(server.arg("agc").c_str(), NULL, 0) : 0x03;
  String res = burstScan(mhz, secs, server.hasArg("wide"), agc);

  // Przywroc normalny tryb odbioru.
  ccInitBase();
  ccWrite(0x00, 0x06);  // IOCFG2 z powrotem: GDO2 = sync pakietu
  if (rxMode == MODE_FINE_OFFSET) applyFineOffsetMode();
  else if (rxMode == MODE_VEVOR_7IN1) applyVevorMode();
  else if (rxMode == MODE_VEVOR_YT60309) applyVevorYT60309Mode();
  else if (rxMode == MODE_BRESSER) applyBresserMode();
  else applyCombo(currentFreq, currentProf);

  if (wifiOff && wifiWasOn) {
    WiFi.mode(WIFI_STA);
    applyWifi();
  }

  server.send(200, "text/plain", res);
}

// ==================== SNIFFER: surowe bajty z prawdziwej paczki ====================
// Lapie burst po RSSI (jak /burst), a w momencie burstu odrzuca smieci z FIFO
// (SFRX) i zrzuca surowe zdemodulowane bajty. Pozwala odczytac strukture
// paczki - dluga preambula, sync word, payload - bez zgadywania protokolu.
static uint8_t sniffBuf[420];


String sniffRun(float mhz, float kbaud, bool ook, int agc, int secs) {
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX

  uint32_t w = (uint32_t)((double)mhz * 1e6 * 65536.0 / 26e6 + 0.5);
  ccWrite(0x0D, (w >> 16) & 0xFF);
  ccWrite(0x0E, (w >> 8) & 0xFF);
  ccWrite(0x0F, w & 0xFF);

  uint8_t m4, m3;
  if (kbaud < 5.0f)       { m4 = 0x43; m3 = 0x83; }  // ~4.8k, BW 812 kHz
  else if (kbaud < 9.5f)  { m4 = 0x88; m3 = 0x4B; }  // ~8.21k, BW 203 kHz
  else if (kbaud < 14.0f) { m4 = 0x98; m3 = 0xC0; }  // ~11.11k, BW 162.5 kHz
  else                    { m4 = 0xB9; m3 = 0x5C; }  // ~17.24k, BW 116 kHz

  ccWrite(0x10, m4);
  ccWrite(0x11, m3);
  ccWrite(0x12, ook ? 0x30 : 0x00);  // MOD_FORMAT: 3=ASK/OOK, 0=2-FSK; SYNC_MODE=0
  ccWrite(0x13, 0x22);
  ccWrite(0x14, 0xF8);
  ccWrite(0x15, 0x44);
  ccWrite(0x1B, (uint8_t)agc);
  ccWrite(0x06, 0xFF);
  ccWrite(0x07, 0x00);
  ccWrite(0x08, 0x02);  // nieskonczona dlugosc - FIFO nie konczy "paczki"

  delay(3);
  ccStrobe(0x36);
  ccStrobe(0x33);  // SCAL
  delay(2);
  ccStrobe(0x3A);
  ccStrobe(0x34);

  // Mediana szumu (odporna na skoki od wlasnego WiFi ESP32).
  int base[256];
  for (int i = 0; i < 256; i++) {
    base[i] = calculateRSSI(ccStatusRead(0x34));
    ccReadFifo(NULL, 64);
  }
  for (int i = 0; i < 255; i++)
    for (int j = 0; j < 255 - i; j++)
      if (base[j] > base[j + 1]) { int t = base[j]; base[j] = base[j + 1]; base[j + 1] = t; }
  const int noiseMed = base[128];
  const int thrHi = noiseMed + 10;
  const int thrLo = noiseMed + 6;

  String out = "SNIFF " + String(mhz, 2) + "MHz " + String(kbaud, 2) + "k ";
  out += ook ? "OOK " : "FSK ";
  out += "agc=0x" + String(agc, HEX) + " szumMed=" + String(noiseMed) + " prog=" + String(thrHi) + "dBm";

  unsigned long tEnd = micros() + (unsigned long)secs * 1000000UL;
  int found = 0;
  long guard = 0;
  while (found < 3 && micros() < tEnd) {
    int r = calculateRSSI(ccStatusRead(0x34));
    // Bez ciaglego oprozniania FIFO radio wchodzi w RXFIFO_OVERFLOW i RSSI
    // zamarza - wtedy zaden burst nie zostanie wykryty.
    ccReadFifo(NULL, 64);
    if ((++guard & 0xFF) == 0 && (ccStatusRead(0x35) & 0x1F) != 0x0D) {
      ccStrobe(0x3A);  // SFRX
      ccStrobe(0x34);  // SRX
    }
    if (r < thrHi) continue;

    found++;
    unsigned long tb = micros();
    // UWAGA: nie robimy tu SFRX. Strobe SFRX wywala radio ze stanu RX i
    // demodulator przestaje produkowac bajty (dlatego poprzednia wersja
    // lapala 0 bajtow). Zostawiamy kilka bajtow smieci sprzed paczki.
    int blen = 0;
    int marcMin = 0x0D;
    while (micros() - tb < 200000UL && blen < (int)sizeof(sniffBuf)) {
      int got = ccReadFifo(sniffBuf + blen, (int)sizeof(sniffBuf) - blen);
      blen += got;
      if (!got) {
        uint8_t st = ccStatusRead(0x35) & 0x1F;
        if (st != 0x0D) {
          if (st < marcMin) marcMin = st;
          ccStrobe(0x34);  // SRX - wroc do odbioru
        }
      }
    }

    out += "\n  #" + String(found) + " t=" + String((tb - (tEnd - (unsigned long)secs * 1000000UL)) / 1000UL) + "ms bajtow=" + String(blen) + " marc=0x" + String(marcMin, HEX) + " : ";
    for (int i = 0; i < blen; i++) {
      if (sniffBuf[i] < 0x10) out += '0';
      out += String(sniffBuf[i], HEX);
      out += ' ';
    }

    // Czekaj, az burst sie skonczy - inaczej zlapiemy ten sam trzy razy.
    unsigned long tw = micros();
    while (micros() - tw < 1000000UL) {
      if (calculateRSSI(ccStatusRead(0x34)) <= thrLo) break;
      ccReadFifo(NULL, 64);
    }
  }

  if (found == 0) out += "\n  brak paczek w " + String(secs) + " s";
  Serial.println(out);
  return out;
}

void handleSniff() {
  float mhz = server.hasArg("mhz") ? server.arg("mhz").toFloat() : 868.30;
  float kbaud = server.hasArg("kbaud") ? server.arg("kbaud").toFloat() : 11.11;
  int secs = server.hasArg("sec") ? server.arg("sec").toInt() : 45;
  if (secs < 10) secs = 10;
  if (secs > 300) secs = 300;
  int agc = server.hasArg("agc") ? (int)strtol(server.arg("agc").c_str(), NULL, 0) : 0x03;

  bool wifiWasOn = (WiFi.getMode() != WIFI_OFF);
  bool wifiOff = server.hasArg("wifi") && server.arg("wifi") == "0";
  if (wifiOff && wifiWasOn) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
  }

  probeFreqIdx = -1;
  String res = sniffRun(mhz, kbaud, server.hasArg("ook"), agc, secs);

  ccInitBase();
  ccWrite(0x00, 0x06);
  if (rxMode == MODE_FINE_OFFSET) applyFineOffsetMode();
  else if (rxMode == MODE_VEVOR_7IN1) applyVevorMode();
  else if (rxMode == MODE_VEVOR_YT60309) applyVevorYT60309Mode();
  else if (rxMode == MODE_BRESSER) applyBresserMode();
  else applyCombo(currentFreq, currentProf);

  if (wifiOff && wifiWasOn) {
    WiFi.mode(WIFI_STA);
    applyWifi();
  }

  server.send(200, "text/plain", res);
}

void handleProbe() {
  if (server.hasArg("mhz")) {
    probeMhz = server.arg("mhz").toFloat();
    if (probeMhz < 300.0 || probeMhz > 1000.0) {
      server.send(400, "text/plain", "Zakres 300..1000 MHz");
      return;
    }
    probeWide = server.hasArg("wide");
    probeFreqIdx = 7;
    applyProbe();
    server.send(200, "text/plain", "Probe ON: " + String(probeMhz, 2) + " MHz" + (probeWide ? " (szerokie 812 kHz)" : ""));
    return;
  }

  probeWide = server.hasArg("wide");
  int f = server.hasArg("f") ? server.arg("f").toInt() : -1;
  if (f < 0 || f > 6) {
    server.send(400, "text/plain", "Zly indeks (0..5 = 868 MHz, 6 = 433.92 MHz) lub uzyj ?mhz=");
    return;
  }
  probeFreqIdx = f;
  applyProbe();
  Serial.print("PROBE start na ");
  Serial.println(f == 6 ? "433.92" : FREQ_NAMES[f]);
  server.send(200, "text/plain", "Probe ON: " + String(FREQ_NAMES[f]) + " MHz (sprawdz Serial). /probeoff aby wylaczyc.");
}

void handleProbeOff() {
  probeFreqIdx = -1;
  if (rxMode == MODE_FINE_OFFSET) applyFineOffsetMode();
  else if (rxMode == MODE_VEVOR_7IN1) applyVevorMode();
  else if (rxMode == MODE_VEVOR_YT60309) applyVevorYT60309Mode();
  else if (rxMode == MODE_BRESSER) applyBresserMode();
  else applyCombo(currentFreq, currentProf);
  server.send(200, "text/plain", "Probe OFF, przywrocono tryb normalny.");
}

// Test diagnostyczny: RSSI w stanie IDLE (bez odbioru) - sprawdza, czy
// odczyt -19 dBm to realny silny sygnał, czy artefakt nasyconego AGC.
void handleRssiIdle() {
  uint8_t partnum = ccStatusRead(0x30);  // PARTNUM (CC1101 = 0x00)
  uint8_t version = ccStatusRead(0x31);  // VERSION (CC1101 = 0x14)
  uint8_t st1 = ccStatusRead(0x35);      // MARCSTATE (0x0D = RX)
  delay(2);
  uint8_t rawOk = ccStatusRead(0x34);    // RSSI (rejestry statusu: odczyt z bitem burst)
  String s = "PARTNUM=0x" + String(partnum, HEX) + " VERSION=0x" + String(version, HEX) +
             " MARCSTATE=0x" + String(st1 & 0x1F, HEX) +
             " RSSI=" + String(rawOk) + "->" + String(calculateRSSI(rawOk)) + " dBm";
  Serial.println(s);
  server.send(200, "text/plain", s);
}

void handleCaptureStart() {
  probeFreqIdx = -1;
  captureMode = true;
  applyCapture();
  Serial.println("CAPTURE start (868.35 MHz, FSK 11.11k, bez sync)");
  server.send(200, "text/plain", "Capture ON - 868.35 MHz FSK 11.11k bez sync. Sprawdz Serial (linie CAP). /capstop aby wylaczyc.");
}

void handleCaptureStop() {
  captureMode = false;
  if (rxMode == MODE_FINE_OFFSET) applyFineOffsetMode();
  else if (rxMode == MODE_VEVOR_7IN1) applyVevorMode();
  else if (rxMode == MODE_VEVOR_YT60309) applyVevorYT60309Mode();
  else if (rxMode == MODE_BRESSER) applyBresserMode();
  else applyCombo(currentFreq, currentProf);
  Serial.println("CAPTURE stop");
  server.send(200, "text/plain", "Capture OFF.");
}

// ==================== AUTOTEST DEKODERA ====================
// Na wejściu dostajemy 32 bajty realnie przechwycone z VEVOR YT60309
// (przykłady z github.com/FPR36/Vevor-Meteo-station-rf-protocol). Jeśli
// dekoder zwraca sensowne wartości, to znaczy, że logika jest poprawna.
static int parseCsvHex(const char* s, uint8_t* out, int maxOut) {
  int n = 0;
  const char* p = s;
  while (*p && n < maxOut) {
    while (*p == ',' || *p == ' ') p++;
    if (!*p) break;
    char* end;
    long v = strtol(p, &end, 16);
    if (end == p) break;
    out[n++] = (uint8_t)v;
    p = end;
  }
  return n;
}

static String selfTestLine(const char* csv) {
  uint8_t b[32];
  int n = parseCsvHex(csv, b, 32);
  WeatherData w;
  String s = "len=" + String(n) + " -> ";
  if (!decodeVevorYT60309(b, n, w)) {
    return s + "ODRZUCONY (suma kontrolna / naglowek)";
  }
  s += w.model;
  if (w.haveTemp) { s += "  T="; s += String(w.tempC, 1); s += "C"; }
  if (w.haveHum)  { s += "  RH="; s += String(w.humidity); s += "%"; }
  if (w.haveWind) { s += "  wiatr="; s += String(w.windAvgMs, 2); s += "m/s"; }
  if (w.haveGust) { s += "  poryw="; s += String(w.windMaxMs, 2); s += "m/s"; }
  if (w.haveWindDir) { s += "  kier="; s += String(w.windDirDeg); s += "st"; }
  if (w.haveRain) { s += "  deszcz="; s += String(w.rainMm, 1); s += "mm"; }
  if (w.haveUv)   { s += "  UV="; s += String(w.uvi); }
  if (w.haveLight){ s += "  slonce="; s += String(w.lightLux, 0); s += "W/m2"; }
  return s;
}

void handleSelfTest() {
  const char* v1 = "14,AA,00,24,0D,1E,02,DA,40,01,06,01,01,57,00,1A,06,98,88,F4,CA,F5,C1,B2,65,9A,74,39,C7,38,C9,1E";
  const char* v2 = "14,AA,00,24,0D,1E,02,DA,41,01,0E,02,01,8D,01,1A,06,98,61,42,11,43,C1,B2,65,9A,74,39,C7,38,C9,1E";
  const char* v3 = "14,AA,00,24,0D,1E,02,CD,43,01,2E,0B,01,B5,01,1A,05,95,6E,53,71,54,C1,B2,65,9A,74,39,C7,38,C9,1E";
  const char* v4 = "14,AA,00,24,0D,1E,02,D6,3F,01,0B,01,01,BC,01,1A,06,99,3A,16,E4,17,C1,B2,65,9A,74,39,C7,38,C9,1E";

  String out = "AUTOTEST dekoderow na prawdziwych pakietach:\n\n";

  out += "--- VEVOR YT60309 (FPR36) ---\n";
  out += "P1: " + selfTestLine(v1) + "\n";
  out += "P2: " + selfTestLine(v2) + "\n";
  out += "P3: " + selfTestLine(v3) + "\n";
  out += "P4: " + selfTestLine(v4) + "\n";

  // Fine Offset WH24/WH65 - prawdziwy pakiet z rtl_433 (tests/fineoffset).
  // Oczekiwane: id=191, temp=11.8C, wilg=78%, kierunek=266 st
  // (wiatr/deszcz liczone wspolczynnikiem WH24: 1.12 m/s i 0.3 mm).
  uint8_t fo[17] = { 0x24,0xbf,0x0a,0xe2,0x06,0x4e,0x08,0x02,
                     0x00,0x4a,0x00,0x01,0x00,0x00,0x00,0x8f,0x07 };
  out += "\n--- Fine Offset WH24/WH65 ---\n";
  WeatherData wf;
  if (decodeFineOffset(fo, 17, wf)) {
    out += "OK  id=" + String(wf.id);
    if (wf.haveTemp) out += "  T=" + String(wf.tempC, 1) + "C";
    if (wf.haveHum)  out += "  RH=" + String(wf.humidity) + "%";
    if (wf.haveWindDir) out += "  kier=" + String(wf.windDirDeg) + "st";
    if (wf.haveWind) out += "  wiatr=" + String(wf.windAvgMs, 2) + "m/s";
    out += "\n(oczekiwane: id=191, T=11.8C, RH=78%, kier=266st)\n";
  } else {
    out += "ODRZUCONY! (blad sumy kontrolnej lub CRC w dekoderze FineOffset)\n";
  }

  // VEVOR/Youtong 7-in-1 (protokół 263) - prawdziwa paczka z Twojej stacji
  // (868.30 MHz, przechwycona snifferem 2026-09-25). Suma kontrolna = 0x85.
  uint8_t vv[28] = { 0xaa,0x00,0xf8,0x82,0x10,0x02,0x62,0x5b,0x01,0x35,
                     0x0a,0x02,0x0b,0x01,0x05,0x01,0x01,0x01,0x3c,0x85,
                     0x3d,0x14,0xd8,0x6b,0x25,0x65,0xd1,0x96 };
  out += "\n--- VEVOR/Youtong 263 (prawdziwa paczka ze stacji) ---\n";
  WeatherData wv;
  if (decodeVevor7in1(vv, 28, wv)) {
    out += "OK  id=" + String(wv.id);
    if (wv.haveTemp) out += "  T=" + String(wv.tempC, 1) + "C";
    if (wv.haveHum)  out += "  RH=" + String(wv.humidity) + "%";
    if (wv.haveWind) out += "  wiatr=" + String(wv.windAvgMs, 2) + "m/s";
    if (wv.haveGust) out += "  poryw=" + String(wv.windMaxMs, 2) + "m/s";
    if (wv.haveWindDir) out += "  kier=" + String(wv.windDirDeg) + "st";
    if (wv.haveRain) out += "  deszcz=" + String(wv.rainMm, 1) + "mm";
    if (wv.haveUv)   out += "  UV=" + String(wv.uvi);
    out += "\n(oczekiwane: T~11C, RH~91%, kier~266st (W))\n";
  } else {
    out += "ODRZUCONY! (dekoder VEVOR/Youtong nie przyjal wlasnej paczki)\n";
  }

  Serial.println(out);
  server.send(200, "text/plain; charset=utf-8", out);
}

// Twardy test stabilnosci SPI z radiem: wielokrotny odczyt VERSION (powinno
// byc stale 0x14 dla CC1101) dwoma sposobami dostepu do rejestrow statusu.
void handleSpiTest() {
  String s = "1) VERSION przed resetem: ";
  s += "0x" + String(ccStatusRead(0x31), HEX) + "\n";

  // Twardy reset układu (SRES) + czas na start rezonatora
  ccStrobe(0x30);   // SRES
  delay(100);
  s += "2) VERSION po SRES: ";
  s += "0x" + String(ccStatusRead(0x31), HEX) + "\n";

  // Wybudzenie przez CS (w SLEEP chip budzi się zboczem CS)
  csHigh(); delayMicroseconds(5);
  csLow();  delayMicroseconds(10);
  csHigh(); delay(60);
  s += "3) VERSION po wybudzeniu CS: ";
  s += "0x" + String(ccStatusRead(0x31), HEX) + "\n";

  s += "4) PARTNUM: 0x" + String(ccStatusRead(0x30), HEX) + "\n";
  s += "5) MARCSTATE: 0x" + String(ccStatusRead(0x35) & 0x1F, HEX) + "\n";
  s += "6) VERSION x8: ";
  for (int i = 0; i < 8; i++) { s += "0x" + String(ccStatusRead(0x31), HEX) + " "; delay(3); }

  Serial.println(s);
  server.send(200, "text/plain; charset=utf-8", s);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== 868 MHz Weather Sniffer (raw) ===");

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_GDO2, INPUT);

  radioSPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  memset(comboHits, 0, sizeof(comboHits));

  ccInitBase();
  Serial.print("CC1101 VERSION=0x");
  Serial.print(ccStatusRead(0x31), HEX);
  Serial.print(" PARTNUM=0x");
  Serial.println(ccStatusRead(0x30), HEX);
  loadRxMode();
  if (rxMode == MODE_FINE_OFFSET) {
    applyFineOffsetMode();
    Serial.println("CC1101 gotowy. Tryb: Fine Offset WH65 (VEVOR YT60309) - nasluch 868.30 MHz.");
  } else if (rxMode == MODE_VEVOR_7IN1) {
    applyVevorMode();
    Serial.println("CC1101 gotowy. Tryb: VEVOR / Youtong 7-in-1 - nasluch 868.30 MHz.");
  } else if (rxMode == MODE_VEVOR_YT60309) {
    applyVevorYT60309Mode();
    Serial.println("CC1101 gotowy. Tryb: VEVOR YT60309 - nasluch 868.35 MHz (sync C0AA).");
  } else if (rxMode == MODE_WEATHER_AUTO) {
    autoIdx = 0;
    autoChange = 0;
    applyAutoStep(0);
    Serial.println("CC1101 gotowy. Tryb: AUTO - probuje wszystkich protokolow stacji.");
  } else if (rxMode == MODE_BRESSER) {
    applyBresserMode();
    Serial.println("CC1101 gotowy. Tryb: Bresser 5/6/7-in-1 - nasluch 868.30 MHz.");
  } else {
    applyCombo(0, 0);
    Serial.print("CC1101 gotowy. Skanuje ");
    Serial.print(NUM_FREQS);
    Serial.print(" czestotliwosci x ");
    Serial.print(NUM_PROFS);
    Serial.println(" profili (raw, bez sync word).");
  }

  lastComboChange = millis();

  // WiFi
  loadWifiCfg();
  applyWifi();

  // Filtr ramek
  loadFilterCfg();
  Serial.print("Filtr ramek: ");
  Serial.println(filterCfg.enabled ? "WLACZONY" : "WYLACZONY");
  if (filterCfg.enabled) {
    Serial.println("  wzorzec: " + filterCfg.pattern);
    Serial.print("  offset: ");
    Serial.println(filterCfg.offset);
    Serial.print("  dlugosc: ");
    Serial.print(filterCfg.minLen);
    Serial.print("..");
    Serial.println(filterCfg.maxLen);
  }
  // Kalibracja / wysyłanie / MQTT
  loadCalCfg();
  loadSendCfg();
  applySend();
  loadMqttCfg();
  applyMqtt();

  Serial.println("AP SSID: " + wifiCfg.apSsid);
  Serial.println("AP IP:   " + WiFi.softAPIP().toString());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(250);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi STA OK, IP: " + WiFi.localIP().toString());
  } else if (wifiCfg.staSsid.length() > 0) {
    Serial.println("Nie polaczono z STA (SSID: " + wifiCfg.staSsid + ")");
  }

  server.on("/", handleRoot);
  server.on("/json", handleJson);
  server.on("/clear", handleClear);
  server.on("/setup", handleSetup);
  server.on("/save", handleSave);
  server.on("/cal", handleCal);
  server.on("/send", handleSend);
  server.on("/mqtt", handleMqtt);
  server.on("/savecal", handleSaveCal);
  server.on("/savesend", handleSaveSend);
  server.on("/savemqtt", handleSaveMqtt);
  server.on("/sendtest", handleSendTest);
  server.on("/probe", handleProbe);
  server.on("/burst", handleBurst);
  server.on("/sniff", handleSniff);
  server.on("/probeoff", handleProbeOff);
  server.on("/rssiidle", handleRssiIdle);
  server.on("/spitest", handleSpiTest);
  server.on("/capstart", handleCaptureStart);
  server.on("/capstop", handleCaptureStop);
  server.on("/selftest", handleSelfTest);
  server.begin();
  Serial.println("Web server na porcie 80");

  startTime = millis();
  Serial.println("Nasluch...\n");
}

// ==================== LOOP ====================
void loop() {
  server.handleClient();

  // Tryb sondy RSSI ma priorytet
  if (probeFreqIdx >= 0) {
    loopProbe();
    return;
  }
  if (captureMode) {
    loopCapture();
    return;
  }

  // Obsługa MQTT (niezależnie od trybu pracy radia)
  mqttEnsureConnected();
  mqtt.loop();

  // Tryb dekodera stacji pogodowej (Fine Offset / VEVOR / Bresser)
  if (rxMode == MODE_FINE_OFFSET) {
    loopFineOffset();
    return;
  }
  if (rxMode == MODE_VEVOR_7IN1) {
    loopVevor();
    return;
  }
  if (rxMode == MODE_VEVOR_YT60309) {
    loopVevorYT60309();
    return;
  }
  if (rxMode == MODE_WEATHER_AUTO) {
    loopAuto();
    return;
  }
  if (rxMode == MODE_BRESSER) {
    loopBresser();
    return;
  }

  // Przełącz kombinację po czasie
  if (millis() - lastComboChange >= COMBO_DWELL_MS) {
    nextCombo();
  }

  // Ile bajtów czeka w FIFO
  uint8_t rxBytes = ccStatusRead(0x3B) & 0x7F;  // RXBYTES

  if (rxBytes == 0) {
    delay(1);
    return;
  }

  // Prawie pełne FIFO -> zresetuj, żeby nie zgubić danych
  if (rxBytes >= 200) {
    ccStrobe(0x36);  // SIDLE
    ccStrobe(0x3A);  // SFRX
    ccStrobe(0x34);  // SRX
    return;
  }

  uint8_t frame[256];
  memset(frame, 0, sizeof(frame));

  csLow();
  radioSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  ccXfer(0xFF);  // burst read FIFO
  for (int i = 0; i < rxBytes; i++) {
    frame[i] = ccXfer(0);
  }
  radioSPI.endTransaction();
  csHigh();

  int rssiRaw = ccStatusRead(0x34);  // RSSI
  int lqi = ccRead(0x33) & 0x7F;  // LQI
  int rssi = calculateRSSI(rssiRaw);

  // Zresetuj RX po odczycie
  ccStrobe(0x36);  // SIDLE
  ccStrobe(0x3A);  // SFRX
  ccStrobe(0x34);  // SRX

  // Filtruj szum
  if (rssi < RSSI_THRESHOLD || rxBytes < MIN_FRAME_LEN) {
    return;
  }

  recordFrame(frame, rxBytes, rssi, lqi);
}
