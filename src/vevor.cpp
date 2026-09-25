/*
  vevor.cpp - dekoder VEVOR / Youtong 7-in-1 (protokół 263 z rtl_433).

  Źródło: rtl_433, src/devices/vevor_7in1.c (Copyright (C) 2024 Bruno OCTAU).

  Producent: Fujian Youtong Industries Co., Ltd., rebrand VEVOR.
  Modele: YT60231 (868 MHz EU), YT60234 (915 MHz US), YT60238, YT60240, LWS234.

  Parametry RF (EU 868 MHz, na podstawie ESPHome parrel/esphome-vevor-7in1):
    - częstotliwość: 868.35 MHz
    - 2-FSK, symbol rate ~11111 Baud, deviation ~70 kHz, BW ~100 kHz
    - burst ~85 ms co 20 s
    - preambuła/sync: AA AA AA CA CA 54
    - sync word do CC1101: CA 54 (ostatnie 2 bajty preambuły)

  Po sync word następuje 28-bajtowa wiadomość:
    b[0]  = 0xAA (stała)
    b[1]  = kind(4b) | channel(4b), zwykle 0x00
    b[2..3]  = ID nadajnika (16-bit)
    b[4]  = flaga baterii (bit7: 1 = słaba)
    b[5..6]  = temperatura, offset 500, skala 0.1 °C
    b[7]  = wilgotność [%]
    b[8..9]  = prędkość wiatru (offset 257, /8.333 → km/h)
    b[10] = poryw wiatru (/1.25 → km/h)
    b[11..12] = kierunek wiatru (offset 257)
    b[13..14] = suma opadu (offset 257, *0.233 mm/tip)
    b[15] = indeks UV (5 bitów, offset 1)
    b[16..17] = światło (offset 257, bit15 = mnożnik ×10)
    b[18] = licznik inkrementowany co nadanie
    b[19] = suma kontrolna (suma b[0..18] mod 256)
    b[20] = licznik inkrementowany (b[18]+1)
    b[21..27] = stałe / nieznane
*/

#include "decoders.h"

bool decodeVevor7in1(const uint8_t* b, int len, WeatherData& w) {
  w.reset();
  if (len < 21) return false;
  if (b[0] != 0xAA) return false;
  if (b[1] != 0x00) return false;   // kind=0, channel=0

  // Suma kontrolna: suma b[0..18] mod 256 == b[19]
  uint16_t sum = 0;
  for (int i = 0; i < 19; i++) sum += b[i];
  if ((sum & 0xFF) != b[19]) return false;

  int id          = (b[2] << 8) | b[3];
  bool battery_ok = !((b[4] & 0x80) >> 7);

  int temp_raw    = (b[5] << 8) | b[6];
  float temp_c    = (temp_raw - 500) * 0.1f;
  int humidity    = b[7];

  int wind_raw    = ((b[8] << 8) | b[9]) - 257;  // -1 na każdy bajt = -257
  float wind_kmh  = wind_raw / 10.0f;            // stacja EU 868 MHz: km/h × 10
  float wind_ms   = wind_kmh / 3.6f;

  int gust_raw    = b[10];
  float gust_kmh  = gust_raw * 0.75f;            // skala EU 868 MHz (konsola ~0.75x skali US)
  float gust_ms   = gust_kmh / 3.6f;

  int direction   = (((b[11] & 0x0f) << 8) | b[12]) - 257;

  int rain_raw    = ((b[13] << 8) | b[14]) - 257;
  float rain_mm   = rain_raw * 0.233f;           // wg rtl_433

  int uv_index    = (b[15] & 0x1f) - 1;

  // Światło: offset "1 na bajt" (jak w rtl_433) — od każdego bajtu odejmujemy 1,
  // z zawinięciem uint8 (0x00 -> 0xff), potem bit15 = mnożnik ×10.
  uint8_t b16 = (uint8_t)(b[16] - 1);
  uint8_t b17 = (uint8_t)(b[17] - 1);
  int light_raw   = (b16 << 8) | b17;
  int lux_multi   = (light_raw & 0x8000) >> 15;
  int light_lux   = light_raw & 0x7fff;
  if (lux_multi == 1) light_lux *= 10;

  w.valid = true;
  w.model = "Vevor-7in1";
  w.id = id;
  w.batteryOk = battery_ok;

  // Walidacja zakresów (ochrona przed śmieciowymi danymi po fałszywym sync).
  if (temp_raw != 0 && temp_c > -50.0f && temp_c < 80.0f) {
    w.haveTemp = true;
    w.tempC = temp_c;
  }
  if (humidity <= 100) {
    w.haveHum = true;
    w.humidity = humidity;
  }
  if (wind_raw >= 0 && wind_raw < 5000 && wind_ms >= 0.0f && wind_ms < 80.0f) {
    w.haveWind = true;
    w.windAvgMs = wind_ms;
  }
  if (gust_raw >= 0 && gust_raw < 255 && gust_ms >= 0.0f && gust_ms < 80.0f) {
    w.haveGust = true;
    w.windMaxMs = gust_ms;
  }
  if (direction >= 0 && direction <= 359) {
    w.haveWindDir = true;
    w.windDirDeg = direction;
  }
  if (rain_raw >= 0 && rain_mm >= 0.0f && rain_mm < 5000.0f) {
    w.haveRain = true;
    w.rainMm = rain_mm;
  }
  if (uv_index >= 0 && uv_index <= 16) {
    w.haveUv = true;
    w.uv = 0;                 // protokół 263 nie przesyła surowego UV — tylko indeks
    w.uvi = uv_index;
  }
  if (light_raw >= 0 && light_lux >= 0 && light_lux < 500000) {
    w.haveLight = true;
    w.lightLux = light_lux;
  }

  return true;
}

/*
  decodeVevorYT60309 - dekoder VEVOR YT60309 (7-in-1, osobny protokół!).

  Źródło: github.com/FPR36/Vevor-Meteo-station-rf-protocol (main.c, CheckData()).

  Nadajnik: CMOSTEC CMT2119A, 868.35 MHz FSK, bit ~90 us (11.11 kbaud),
  preambuła 39 bitów + sync C0AA C0AA (32 bity), potem 32 bajty danych.

  Układ 32 bajtów (indeks 0..31):
    b[0..5]  = nagłówek produktu/bezpieczeństwa: 14 AA 00 24 0D 1E
    b[6..7]  = temperatura (HI, LO), offset 500, skala 0.1 °C
    b[8]     = wilgotność [%] (0..100)
    b[9..10] = prędkość wiatru (b[10] = km/h × 10; b[9] górne bity niepewne)
    b[11]    = poryw wiatru, km/h * 100/65 (autor: *65/100)
    b[12]    = bity kierunku/baterii (bit 1 = 9. bit kierunku)
    b[13]    = kierunek wiatru 0..255 (8 z 9 bitów, 0..360°)
    b[14..15]= licznik opadu (HI, LO), 1 krok = 0.2 mm
    b[16]    = indeks UV (0..15)
    b[17..19]= promieniowanie słoneczne 24 bity (b17=HI), /36000 = W/m²
    b[20]    = suma kontrolna: suma b[1..19] (mod 256)
    b[21]    = b[19] + 1 (nieużywane)
    b[22..31]= stałe (sygnatura firmware)
*/
bool decodeVevorYT60309(const uint8_t* b, int len, WeatherData& w) {
  w.reset();
  if (len < 32) return false;

  // Nagłówek (autor sprawdza dokładnie te dwa bajty)
  if (b[1] != 0xAA || b[2] != 0x00) return false;

  // Suma kontrolna: suma b[1..19] mod 256 == b[20]
  uint16_t sum = 0;
  for (int i = 1; i < 20; i++) sum += b[i];
  if ((sum & 0xFF) != b[20]) return false;

  int temp_tenths = ((b[6] << 8) | b[7]) - 500;   // 0.1 °C
  float temp_c = temp_tenths * 0.1f;
  int humidity  = b[8];

  int wind_raw  = b[10];                          // dziesiątki km/h (autor: /10)
  float wind_kmh = wind_raw / 10.0f;
  float wind_ms  = wind_kmh / 3.6f;

  int gust_raw  = b[11];
  float gust_kmh = gust_raw * 65.0f / 100.0f;     // autor: *65/100
  float gust_ms  = gust_kmh / 3.6f;

  int dir_raw   = b[13] + ((b[12] & 0x02) ? 256 : 0);  // 0..511
  int wind_dir  = (int)((dir_raw * 360L + 256) / 512); // -> 0..359
  if (wind_dir > 359) wind_dir = 359;

  int rain_raw  = (b[14] << 8) | b[15];
  float rain_mm = rain_raw * 0.2f;

  int uv_index  = b[16];                          // 0..15

  int solar_raw = (b[17] << 16) | (b[18] << 8) | b[19];  // b17 = HI
  float solar_wm2 = solar_raw / 36000.0f;

  w.valid = true;
  w.model = "Vevor-YT60309";
  w.id = ((b[3] << 8) | b[4]) & 0xFFFF;          // część nagłówka (sygnatura)
  w.batteryOk = true;                             // bit baterii w b[12] - niepewny

  if (temp_tenths > -500 && temp_tenths < 600) {
    w.haveTemp = true;
    w.tempC = temp_c;
  }
  if (humidity <= 100) {
    w.haveHum = true;
    w.humidity = humidity;
  }
  if (wind_kmh >= 0.0f && wind_kmh < 200.0f) {
    w.haveWind = true;
    w.windAvgMs = wind_ms;
  }
  if (gust_kmh >= 0.0f && gust_kmh < 200.0f) {
    w.haveGust = true;
    w.windMaxMs = gust_ms;
  }
  if (dir_raw >= 0 && dir_raw <= 511) {
    w.haveWindDir = true;
    w.windDirDeg = wind_dir;
  }
  if (rain_mm >= 0.0f && rain_mm < 10000.0f) {
    w.haveRain = true;
    w.rainMm = rain_mm;
  }
  if (uv_index >= 0 && uv_index <= 16) {
    w.haveUv = true;
    w.uv = uv_index;
    w.uvi = uv_index;
  }
  if (solar_raw >= 0 && solar_wm2 < 2000.0f) {
    w.haveLight = true;
    w.lightLux = solar_wm2;   // UWAGA: to W/m² (nie lux) dla tego protokołu
  }

  return true;
}
