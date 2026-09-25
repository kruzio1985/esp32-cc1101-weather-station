/*
  fineoffset.cpp - dekoder Fine Offset WH24 / WH65 (17-bajtowy payload).

  Źródło: rtl_433, src/devices/fineoffset.c (fineoffset_WH25_callback).
  Rodzina 0x24 obejmuje m.in.:
    - Fine Offset WH24, WH65 (7-in-1)
    - VEVOR YT60309 / YT60233 / YT60234 (rebrand WH65)
    - Ecowitt WS65, HP1000, Misol WS2320 (rodzina WH25/WH65)
    - WS69 (wersja 25-bajtowa z ciśnieniem - obsługa w przyszłości)

  Pakiet (po sync word 0x2D 0xD4): 17 bajtów.
    b[0]  = 0x24 (family code)
    b[1]  = id (zmienia się przy wymianie baterii)
    b[2..4]  = kierunek wiatru, bateria, temperatura
    b[5]  = wilgotność [%]
    b[6]  = prędkość wiatru (skalowane /8)
    b[7]  = poryw wiatru
    b[8..9]  = licznik opadu (tips)
    b[10..11] = UV
    b[12..14] = światło
    b[16] = suma kontrolna
*/

#include "decoders.h"

bool decodeFineOffset(const uint8_t* b, int len, WeatherData& w) {
  w.reset();
  if (len < 17) return false;
  if (b[0] != 0x24) return false;   // kod rodziny Fine Offset

  // Suma kontrolna (jak w rtl_433): CRC-8 Dallas po b[0..15] musi dać 0,
  // a dodatkowo suma bajtów b[0..15] (mod 256) musi równać się b[16].
  uint8_t crc = crc8_dallas(b, 16);
  uint16_t sum = 0;
  for (int i = 0; i < 16; i++) sum += b[i];
  if (crc != 0 || (sum & 0xFF) != b[16]) return false;

  // ---- Dekodowanie pól ----
  int id          = b[1];
  int wind_dir    = b[2] | ((b[3] & 0x80) << 1);   // 0..359, 0x1ff = brak
  int low_battery = (b[3] & 0x08) >> 3;
  int temp_raw    = ((b[3] & 0x07) << 8) | b[4];   // 0x7ff = brak
  int humidity    = b[5];                          // 0xff = brak
  int wind_raw    = b[6] | ((b[3] & 0x10) << 4);   // 0x1ff = brak
  int gust_raw    = b[7];                          // 0xff = brak
  int rain_raw    = (b[8] << 8) | b[9];            // licznik tips
  int uv_raw      = (b[10] << 8) | b[11];          // 0xffff = brak
  int light_raw   = (b[12] << 16) | (b[13] << 8) | b[14]; // 0xffffff = brak

  // WH65 (i VEVOR YT60309): wiatr *0.51 m/s, kropla deszczu = 0.254 mm.
  // WH24 różni się (wiatr *1.12, kropla 0.3 mm) - nie do odróżnienia z pakietu.
  const float wind_factor   = 0.51f;
  const float rain_cup_mm   = 0.254f;

  w.valid = true;
  w.model = "FineOffset-WH65";
  w.id = id;
  w.batteryOk = !low_battery;

  if (temp_raw != 0x7ff) {
    w.haveTemp = true;
    w.tempC = (temp_raw - 400) * 0.1f;   // zakres -40.0..60.0 C
  }
  if (humidity != 0xff) {
    w.haveHum = true;
    w.humidity = humidity;
  }
  if (wind_dir != 0x1ff) {
    w.haveWindDir = true;
    w.windDirDeg = wind_dir;
  }
  if (wind_raw != 0x1ff) {
    w.haveWind = true;
    w.windAvgMs = wind_raw * 0.125f * wind_factor;
  }
  if (gust_raw != 0xff) {
    w.haveGust = true;
    w.windMaxMs = gust_raw * wind_factor;
  }
  if (rain_raw != 0xffff) {
    w.haveRain = true;
    w.rainMm = rain_raw * rain_cup_mm;
  }
  if (uv_raw != 0xffff) {
    w.haveUv = true;
    w.uv = uv_raw;
    // Przeliczenie surowego UV na indeks (tabela z rtl_433).
    static const int uvi_upper[] = {
      432, 851, 1210, 1570, 2017, 2450, 2761, 3100, 3512, 3918, 4277, 4650, 5029
    };
    int idx = 0;
    while (idx < 13 && uvi_upper[idx] < uv_raw) idx++;
    w.uvi = idx;
  }
  if (light_raw != 0xffffff) {
    w.haveLight = true;
    w.lightLux = light_raw * 0.1f;   // zakres 0..300000 lux
  }

  return true;
}
