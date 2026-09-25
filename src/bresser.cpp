/*
  bresser.cpp - dekodery stacji pogodowych Bresser 5-in-1 / 6-in-1 / 7-in-1.

  Źródła (port z C):
    - rtl_433: src/devices/bresser_5in1.c, bresser_6in1.c, bresser_7in1.c
    - lfsr_digest16 z rtl_433: src/bit_util.c
    - konfiguracja RF potwierdzona w projekcie matthias-bs/BresserWeatherSensorReceiver

  Wspólna konfiguracja RF (wszystkie 3 warianty nadają tak samo):
    868.30 MHz, 2-FSK, ~8.21 kbaud, dewiacja ~57 kHz, sync word 0xAA 0x2D.
    CC1101 w trybie stałej długości pakietu (27 bajtów); pierwszy bajt payloadu
    to 0xD4 (ostatni bajt preambuły/sync, który ląduje w FIFO). Dekoder dostaje
    dane PO tym bajcie.

  Bresser 5-in-1: 26 bajtów po 0xD4.
  Bresser 6-in-1: 18 bajtów po 0xD4 (reszta bufora 27 B to "ogon" ramki).
  Bresser 7-in-1: 25 bajtów po 0xD4.
*/

#include "decoders.h"

// LFSR-16 digest (dokładnie jak w rtl_433 src/bit_util.c).
// gen = 0x8810 (z MSB, bo LFSR "toczy się" w prawo).
static uint16_t lfsrDigest16(const uint8_t* msg, int bytes, uint16_t gen, uint16_t key) {
  uint16_t sum = 0;
  for (int k = 0; k < bytes; k++) {
    uint8_t data = msg[k];
    for (int i = 7; i >= 0; i--) {
      if ((data >> i) & 1) {
        sum ^= key;
      }
      if (key & 1) {
        key = (key >> 1) ^ gen;
      } else {
        key = (key >> 1);
      }
    }
  }
  return sum;
}

// ---- Bresser 5-in-1 ----
bool decodeBresser5in1(const uint8_t* b, int len, WeatherData& w) {
  w.reset();
  if (len < 26) return false;

  // Pierwsze 13 bajtów musi być negacją ostatnich 13 bajtów.
  for (int i = 0; i < 13; i++) {
    if ((b[i] ^ b[i + 13]) != 0xFF) return false;
  }

  int sensor_type = b[15] & 0x7F;

  // --- dekodowanie pól (uwaga: BCD "nibble-swapped") ---
  int id = b[14];

  // Temperatura: BCD, b[20] + setki z b[21] (niskie nibble).
  bool temp_ok = (b[20] & 0x0F) <= 9;
  int temp_raw = (b[20] & 0x0F) + ((b[20] >> 4) & 0x0F) * 10 + (b[21] & 0x0F) * 100;
  if (b[25] & 0x0F) temp_raw = -temp_raw;   // znak minus (dowolny niezerowy nibble)

  // Wilgotność: BCD, b[22].
  bool hum_ok = (b[22] & 0x0F) <= 9;
  int humidity = (b[22] & 0x0F) + ((b[22] >> 4) & 0x0F) * 10;

  // Kierunek: górny nibble b[17] -> 16 kierunków x 22.5°.
  float wind_dir = ((b[17] >> 4) & 0x0F) * 22.5f;

  // Poryw: normalny binarnie, MSB "poza kolejnością".
  int gust_raw = ((b[17] & 0x0F) << 8) + b[16];

  // Wiatr średni: BCD, b[18] + setki z b[19].
  int wind_raw = (b[18] & 0x0F) + ((b[18] >> 4) & 0x0F) * 10 + (b[19] & 0x0F) * 100;

  // Deszcz: BCD, b[23] + b[24] (4 cyfry).
  int rain_raw = (b[23] & 0x0F) + ((b[23] >> 4) & 0x0F) * 10
               + (b[24] & 0x0F) * 100 + ((b[24] >> 4) & 0x0F) * 1000;

  bool battery_low = (b[25] & 0x80) != 0;

  float rain = rain_raw * 0.1f;
  // Bresser Professional Rain Gauge (typ 0x39..0x3B) - inna kalibracja kropli.
  bool is_pro_rain = (sensor_type >= 0x39 && sensor_type <= 0x3B);
  if (is_pro_rain) rain *= 2.5f;

  w.valid = true;
  w.model = is_pro_rain ? "Bresser-ProRainGauge" : "Bresser-5in1";
  w.id = id;
  w.batteryOk = !battery_low;

  if (temp_ok) {
    w.haveTemp = true;
    w.tempC = temp_raw * 0.1f;
  }
  if (hum_ok) {
    w.haveHum = true;
    w.humidity = humidity;
  }
  w.haveWindDir = true;
  w.windDirDeg = (int)wind_dir;
  w.haveWind = true;
  w.windAvgMs = wind_raw * 0.1f;
  w.haveGust = true;
  w.windMaxMs = gust_raw * 0.1f;
  w.haveRain = true;
  w.rainMm = rain;

  return true;
}

// ---- Bresser 6-in-1 ----
bool decodeBresser6in1(const uint8_t* b, int len, WeatherData& w) {
  w.reset();
  if (len < 18) return false;

  // LFSR-16 digest, generator 0x8810, klucz 0x5412, po bajtach 2..16 (15 B).
  uint16_t chkdgst = (uint16_t)((b[0] << 8) | b[1]);
  uint16_t digest = lfsrDigest16(&b[2], 15, 0x8810, 0x5412);
  if (chkdgst != digest) return false;

  // Suma kontrolna: suma bajtów 2..17 (mod 256) == 0xFF.
  uint16_t sum = 0;
  for (int i = 2; i < 18; i++) sum += b[i];
  if ((sum & 0xFF) != 0xFF) return false;

  uint32_t id = ((uint32_t)b[2] << 24) | ((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5];
  int s_type  = b[6] >> 4;          // 1=stacja, 2=termo/higro, 3=basen, 4=gleba
  int chan    = b[6] & 0x07;
  bool battery_good = (b[13] >> 1) & 1;

  // Temperatura: BCD b[12] + b[13] (górny nibble = jednostki).
  bool temp_ok = (b[12] <= 0x99) && ((b[13] & 0xF0) <= 0x90);
  int temp_raw = (b[12] >> 4) * 100 + (b[12] & 0x0F) * 10 + (b[13] >> 4);
  bool temp_sign = (b[13] >> 3) & 1;
  float temp_c = temp_raw * 0.1f;
  if (temp_sign) temp_c = (temp_raw - 1000) * 0.1f;
  if (temp_c < -50.0f) temp_c = -temp_raw * 0.1f;

  // Wilgotność: BCD b[14].
  int humidity = (b[14] >> 4) * 10 + (b[14] & 0x0F);

  // UV: zanegowane 2 bajty BCD.
  bool uv_ok = ((b[16] & 0x0F) == 0) && ((~b[15] & 0xFF) <= 0x99) && ((~b[16] & 0xF0) <= 0x90);
  int uv_raw = ((~b[15] & 0xF0) >> 4) * 100 + (~b[15] & 0x0F) * 10 + ((~b[16] & 0xF0) >> 4);

  // Wiatr: 3 zanegowane bajty BCD.
  uint8_t w0 = b[7] ^ 0xFF, w1 = b[8] ^ 0xFF, w2 = b[9] ^ 0xFF;
  bool wind_ok = (w0 <= 0x99) && (w1 <= 0x99) && (w2 <= 0x99);
  int gust_raw = (w0 >> 4) * 100 + (w0 & 0x0F) * 10 + (w1 >> 4);
  int wavg_raw = (w2 >> 4) * 100 + (w2 & 0x0F) * 10 + (w1 & 0x0F);
  int wind_dir = (b[10] >> 4) * 100 + (b[10] & 0x0F) * 10 + (b[11] >> 4);

  // Deszcz: 3 zanegowane bajty BCD.
  uint8_t r0 = b[12] ^ 0xFF, r1 = b[13] ^ 0xFF, r2 = b[14] ^ 0xFF;
  bool rain_ok = (b[16] & 1) != 0;
  int rain_raw = (r0 >> 4) * 100000 + (r0 & 0x0F) * 10000
               + (r1 >> 4) * 1000 + (r1 & 0x0F) * 100
               + (r2 >> 4) * 10 + (r2 & 0x0F);

  // Termo/higro (2) i gleba (4) nie niosą wiatru/UV.
  if (s_type == 2 || s_type == 4) {
    wind_ok = false;
    uv_ok = false;
  }

  w.valid = true;
  w.model = "Bresser-6in1";
  w.id = (int)id;
  w.batteryOk = battery_good;

  if (temp_ok) {
    w.haveTemp = true;
    w.tempC = temp_c;
  }
  if (temp_ok) {
    w.haveHum = true;
    w.humidity = humidity;
  }
  if (wind_ok) {
    w.haveWindDir = true;
    w.windDirDeg = wind_dir;
    w.haveWind = true;
    w.windAvgMs = wavg_raw * 0.1f;
    w.haveGust = true;
    w.windMaxMs = gust_raw * 0.1f;
  }
  if (rain_ok) {
    w.haveRain = true;
    w.rainMm = rain_raw * 0.1f;
  }
  if (uv_ok) {
    w.haveUv = true;
    w.uv = uv_raw;            // BCD = uvi*10 (np. 32 => uvi 3.2)
    w.uvi = uv_raw / 10;      // indeks UV 0..13 (ucięty)
  }

  return true;
}

// ---- Bresser 7-in-1 ----
bool decodeBresser7in1(const uint8_t* b, int len, WeatherData& w) {
  w.reset();
  if (len < 25) return false;
  if (b[21] == 0x00) return false;   // pusta ramka

  int s_type = b[6] >> 4;            // 1=pogoda, 12=3-in-1, 13=8-in-1
  int chan   = b[6] & 0x07;

  // Whitening 0xAA - kopia, żeby nie psuć bufora ramki.
  uint8_t m[25];
  for (int i = 0; i < 25; i++) m[i] = b[i] ^ 0xAA;

  // LFSR-16 digest, generator 0x8810, klucz 0xBA95, finalny XOR 0x6DF1.
  uint16_t chk = (uint16_t)((m[0] << 8) | m[1]);
  uint16_t digest = lfsrDigest16(&m[2], 23, 0x8810, 0xBA95);
  if ((chk ^ digest) != 0x6DF1) return false;

  // Obsługujemy tylko warianty "pogodowe" (inne to PM/CO2/HCHO - poza zakresem).
  if (s_type != 1 && s_type != 12 && s_type != 13) return false;

  int id = (m[2] << 8) | m[3];

  int flags = m[15] & 0x0F;
  bool battery_low = (flags & 0x06) == 0x06;

  int wdir = (m[4] >> 4) * 100 + (m[4] & 0x0F) * 10 + (m[5] >> 4);
  int wgst_raw = (m[7] >> 4) * 100 + (m[7] & 0x0F) * 10 + (m[8] >> 4);
  int wavg_raw = (m[8] & 0x0F) * 100 + (m[9] >> 4) * 10 + (m[9] & 0x0F);
  int rain_raw = (m[10] >> 4) * 100000 + (m[10] & 0x0F) * 10000 + (m[11] >> 4) * 1000
               + (m[11] & 0x0F) * 100 + (m[12] >> 4) * 10 + (m[12] & 0x0F);
  int temp_raw = (m[14] >> 4) * 100 + (m[14] & 0x0F) * 10 + (m[15] >> 4);
  float temp_c = temp_raw * 0.1f;
  if (temp_raw > 600) temp_c = (temp_raw - 1000) * 0.1f;
  int humidity = (m[16] >> 4) * 10 + (m[16] & 0x0F);
  int lght_raw = (m[17] >> 4) * 100000 + (m[17] & 0x0F) * 10000 + (m[18] >> 4) * 1000
               + (m[18] & 0x0F) * 100 + (m[19] >> 4) * 10 + (m[19] & 0x0F);
  int uv_raw = (m[20] >> 4) * 100 + (m[20] & 0x0F) * 10 + (m[21] >> 4);

  // 3-in-1 nie ma wiatru ani światła.
  bool wind_light_ok = (s_type != 12);

  w.valid = true;
  w.model = "Bresser-7in1";
  w.id = id;
  w.batteryOk = !battery_low;

  w.haveTemp = true;
  w.tempC = temp_c;
  w.haveHum = true;
  w.humidity = humidity;
  w.haveRain = true;
  w.rainMm = rain_raw * 0.1f;
  if (wind_light_ok) {
    w.haveWindDir = true;
    w.windDirDeg = wdir;
    w.haveWind = true;
    w.windAvgMs = wavg_raw * 0.1f;
    w.haveGust = true;
    w.windMaxMs = wgst_raw * 0.1f;
    w.haveLight = true;
    w.lightLux = (float)lght_raw;
    w.haveUv = true;
    w.uv = uv_raw;            // BCD = uvi*10 (np. 32 => uvi 3.2)
    w.uvi = uv_raw / 10;      // indeks UV 0..13 (ucięty)
  }

  return true;
}
