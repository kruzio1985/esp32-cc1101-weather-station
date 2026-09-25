/*
  decoders.h - wspólny szkielet dekoderów stacji pogodowych 868 MHz.

  Każdy dekoder wypełnia strukturę WeatherData. Dzięki temu strona WWW,
  JSON i (w przyszłości) RS485/MQTT pokazują dane jednakowo, niezależnie
  od tego, czy pakiet pochodzi z Fine Offset / VEVOR, Bresser, TFA itd.
*/

#pragma once
#include <Arduino.h>

struct WeatherData {
  bool   valid = false;
  String model = "";       // nazwa modelu / rodziny (np. "FineOffset-WH65")

  int    id = -1;          // identyfikator nadajnika
  bool   batteryOk = true;

  bool   haveTemp = false;
  float  tempC = 0.0f;

  bool   haveHum = false;
  int    humidity = 0;     // %

  bool   haveWindDir = false;
  int    windDirDeg = 0;   // 0..359

  bool   haveWind = false;
  float  windAvgMs = 0.0f; // prędkość średnia [m/s]

  bool   haveGust = false;
  float  windMaxMs = 0.0f; // poryw [m/s]

  bool   haveRain = false;
  float  rainMm = 0.0f;    // suma opadu [mm]

  bool   haveUv = false;
  int    uv = 0;           // surowa wartość UV (0..20000)
  int    uvi = 0;          // indeks UV (0..13)

  bool   haveLight = false;
  float  lightLux = 0.0f;  // natężenie światła [lux]

  int    rssi = 0;         // dBm
  unsigned long t = 0;     // millis() odbioru

  void reset() {
    valid = false;
    model = "";
    id = -1;
    batteryOk = true;
    haveTemp = haveHum = haveWindDir = haveWind = haveGust = false;
    haveRain = haveUv = haveLight = false;
  }
};

// CRC-8 Dallas/Maxim (poly 0x31, init 0x00) - używany przez Fine Offset
// (i wiele innych czujników 1-wire'owych).
static inline uint8_t crc8_dallas(const uint8_t* b, int n) {
  uint8_t crc = 0;
  for (int i = 0; i < n; i++) {
    crc ^= b[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Kierunek wiatru w stopniach -> tekst (16 kierunków).
static inline const char* windDirText(int deg) {
  static const char* dirs[] = {
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
  };
  int i = (int)((deg + 11) / 22.5f) % 16;
  if (i < 0) i = 0;
  return dirs[i];
}

// Dekoder Fine Offset WH24 / WH65 / WS69 (rodzina 0x24).
// Zwraca true, gdy pakiet poprawnie zdekodowany (CRC + suma kontrolna OK).
bool decodeFineOffset(const uint8_t* b, int len, WeatherData& w);

// Dekoder VEVOR / Youtong 7-in-1 (protokół 263).
// Zwraca true, gdy pakiet poprawnie zdekodowany (suma kontrolna OK).
bool decodeVevor7in1(const uint8_t* b, int len, WeatherData& w);

// Dekoder VEVOR YT60309 (CMT2119A, 868.35 MHz, sync C0AA C0AA, 32 bajty).
// Źródło: github.com/FPR36/Vevor-Meteo-station-rf-protocol (main.c).
bool decodeVevorYT60309(const uint8_t* b, int len, WeatherData& w);

// Dekodery Bresser (5-in-1 / 6-in-1 / 7-in-1). Dane przekazywane PO bajcie 0xD4.
bool decodeBresser5in1(const uint8_t* b, int len, WeatherData& w);
bool decodeBresser6in1(const uint8_t* b, int len, WeatherData& w);
bool decodeBresser7in1(const uint8_t* b, int len, WeatherData& w);
