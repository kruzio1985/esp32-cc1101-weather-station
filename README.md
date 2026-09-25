# ESP32-S3 + CC1101 — Odbiornik stacji pogodowej 868 MHz
# ESP32-S3 + CC1101 — 868 MHz Weather Station Receiver

[Polski](#polski) · [English](#english)

Firmware dla **ESP32-S3** z modułem radiowym **CC1101** (868 MHz). Odbiera i dekoduje
bezprzewodowe stacje pogodowe (głównie **VEVOR / Youtong 7-w-1, protokół 263**) i udostępnia
dane przez stronę WWW, JSON, HTTP oraz MQTT.

Firmware for an **ESP32-S3** with a **CC1101** (868 MHz) radio module. It receives and decodes
wireless weather stations (mainly **VEVOR / Youtong 7-in-1, protocol 263**) and exposes the data
via a web page, JSON, HTTP and MQTT.

---

<a name="polski"></a>
## Polski

### Co potrafi

- Odbiera i dekoduje stację **VEVOR / Youtong 7-w-1** (868 MHz, protokół 263) — temperatura,
  wilgotność, prędkość i kierunek wiatru, poryw wiatru, suma opadu, indeks UV, natężenie światła,
  stan baterii i ID nadajnika.
- Obsługuje też (eksperymentalnie): **Bresser 5/6/7-w-1**, **Fine Offset WH24/WH65**,
  **VEVOR YT60309** (868.35 MHz).
- Strona WWW z zakładkami: **Odczyt**, **Kalibracja**, **Wysyłanie**, **MQTT**.
- API JSON (`/json`) do integracji z Home Assistant i innymi systemami.
- Wysyłka danych przez HTTP (POST JSON), MQTT i RS485.
- Narzędzia diagnostyczne: skan surowych ramek, pomiar RSSI, detektor burstów, autotest
  dekoderów i test SPI.

### WAŻNE — jaka stacja pogodowa jest potrzebna

Ten firmware powstał i został przetestowany pod stację:

- **VEVOR / Youtong 7-w-1**, model **YT60231** — wersja europejska **868 MHz**
  (sprzedawana m.in. pod marką VEVOR; to stacja z protokołem „263", który dekoduje rtl_433
  jako `vevor_7in1`).

Parametry radiowe, które **musi** spełniać nadajnik w czujniku zewnętrznym:

| Parametr | Wartość |
|---|---|
| Częstotliwość | **868.30 MHz** |
| Modulacja | 2-FSK, ~11.11 kbaud, dewiacja ~70 kHz |
| Okres nadawania | co **~20 s** (burst ~85 ms) |
| Preambuła / sync | `AA AA AA AA AA CA CA 54` (sync word `CA 54`) |
| Długość ramki | 28 bajtów po sync word |
| Suma kontrolna | `suma b[0..18] mod 256 == b[19]` |

Dane, które stacja wysyła w ramce: temperatura, wilgotność, prędkość wiatru (średnia),
poryw wiatru, kierunek wiatru (16 kierunków), suma opadu, indeks UV, natężenie światła (lux),
flaga baterii oraz ID nadajnika.

> **Ciśnienie atmosferyczne NIE jest przesyłane przez radio.** Barometr znajduje się w konsoli
> (wyświetlaczu), a nie w czujniku zewnętrznym — dlatego odbiornik nie może go odebrać.
> Konsola pokazuje „baro relative" (przeliczone na poziom morza) i „baro absolute" (rzeczywiste
> w miejscu konsoli) — oba pochodzą z jej wewnętrznego barometru.

> Wersja **915 MHz (USA, YT60234)** używa tego samego protokołu, ale innego skalowania wiatru
> i opadu. Ten firmware jest skalibrowany pod wersję **868 MHz EU**.

### Sprzęt i podłączenie

- **ESP32-S3** (DevKitC-1, 8 MB flash)
- Moduł **CC1101 868 MHz**

Podłączenie (nie zmieniaj!):

| CC1101 | ESP32-S3 |
|---|---|
| CSN   | GPIO10 |
| SCK   | GPIO12 |
| MISO/SO | GPIO13 |
| MOSI/SI | GPIO11 |
| GDO0  | GPIO5  |
| GDO2  | GPIO6  |
| VCC   | 3.3 V  |
| GND   | GND    |

SPI: 1 MHz, tryb 0, magistrala VSPI. Antena CC1101 powinna być ustawiona pionowo,
z dala od ESP32 (ESP32 z WiFi generuje szumy) i z dala od metalowych elementów.
Przy dłuższych przewodach zasilających dodaj kondensator 100 nF (i opcjonalnie 47–100 µF)
tuż przy pinach VCC–GND modułu.

### Budowanie i wgrywanie

Wymagania: [PlatformIO](https://platformio.org/) (projekt skonfigurowany dla płyty
`esp32-s3-devkitc-1`, framework Arduino).

```powershell
# budowanie
pio run

# wgranie (port COM12 — zmień w platformio.ini jeśli trzeba)
pio run -t upload
```

### Konfiguracja sieci

- **AP (punkt dostępowy)**: SSID `WeatherSniffer`, hasło `sniffer123`, adres `192.168.4.1`.
  AP działa zawsze — można się do niego podłączyć i skonfigurować sieć.
- **STA (Twoje WiFi)**: konfiguruje się przez panel WWW (zakładka ustawień). Dane są zapisywane
  w pamięci NVS urządzenia — **nie wpisuj ich w kodzie źródłowym** (ten plik może trafić na
  publiczne repozytorium).

### Kalibracja

W zakładce **Kalibracja** można ustawić poprawki temperatury, wilgotności, mnożnik wiatru
i opadu itd. Opad na stronie WWW pokazuje **przyrost od włączenia odbiornika** (jak „total"
na konsoli stacji), a nie pełną sumę licznika stacji.

### Licencja

**GPL-2.0-or-later** (zobacz plik [LICENSE](LICENSE)).

Dekodery zostały przeniesione/oparte na:

- [rtl_433](https://github.com/merbanan/rtl_433) — GPL-2.0-or-later
  (`vevor_7in1.c`, `bresser_5in1.c`, `bresser_6in1.c`, `bresser_7in1.c`, `fineoffset.c`, `bit_util.c`)
- [BresserWeatherSensorLW](https://github.com/matthias-bs/BresserWeatherSensorLW) — MIT
  (odniesienie konfiguracji RF)
- [FPR36/Vevor-Meteo-station-rf-protocol](https://github.com/FPR36/Vevor-Meteo-station-rf-protocol)
  (odniesienie protokołu YT60309)

---

<a name="english"></a>
## English

### Features

- Receives and decodes a **VEVOR / Youtong 7-in-1** station (868 MHz, protocol 263) — temperature,
  humidity, wind speed and direction, gust, rain total, UV index, light level, battery flag and
  transmitter ID.
- Also supports (experimental): **Bresser 5/6/7-in-1**, **Fine Offset WH24/WH65**,
  **VEVOR YT60309** (868.35 MHz).
- Web UI with tabs: **Reading**, **Calibration**, **Sending**, **MQTT**.
- JSON API (`/json`) for Home Assistant and other integrations.
- Data forwarding via HTTP (JSON POST), MQTT and RS485.
- Diagnostic tools: raw frame scan, RSSI measurement, burst detector, decoder self-test and SPI test.

### IMPORTANT — which weather station is required

This firmware was built and tested against:

- **VEVOR / Youtong 7-in-1**, model **YT60231** — the European **868 MHz** version
  (sold under the VEVOR brand; the station that speaks "protocol 263", decoded by rtl_433
  as `vevor_7in1`).

The outdoor sensor transmitter must match these RF parameters:

| Parameter | Value |
|---|---|
| Frequency | **868.30 MHz** |
| Modulation | 2-FSK, ~11.11 kbaud, ~70 kHz deviation |
| TX interval | every **~20 s** (burst ~85 ms) |
| Preamble / sync | `AA AA AA AA AA CA CA 54` (sync word `CA 54`) |
| Frame length | 28 bytes after the sync word |
| Checksum | `sum of b[0..18] mod 256 == b[19]` |

The station transmits: temperature, humidity, average wind speed, wind gust, wind direction
(16 directions), rain total, UV index, light (lux), battery flag and transmitter ID.

> **Atmospheric pressure is NOT transmitted over the radio.** The barometer is inside the
> console (display), not in the outdoor sensor — so the receiver cannot pick it up. The console's
> "baro relative" (sea-level adjusted) and "baro absolute" (actual at the console location) both
> come from its internal barometer.

> The **915 MHz (US, YT60234)** version uses the same protocol but different wind/rain scaling.
> This firmware is calibrated for the **868 MHz EU** version.

### Hardware & wiring

- **ESP32-S3** (DevKitC-1, 8 MB flash)
- **CC1101 868 MHz** module

Wiring (do not change!):

| CC1101 | ESP32-S3 |
|---|---|
| CSN   | GPIO10 |
| SCK   | GPIO12 |
| MISO/SO | GPIO13 |
| MOSI/SI | GPIO11 |
| GDO0  | GPIO5  |
| GDO2  | GPIO6  |
| VCC   | 3.3 V  |
| GND   | GND    |

SPI: 1 MHz, mode 0, VSPI bus. Keep the CC1101 antenna vertical, away from the ESP32
(the ESP32 with WiFi produces noise) and away from metal. For longer power wires, add a 100 nF
capacitor (and optionally 47–100 µF) close to the module's VCC–GND pins.

### Build & flash

Requires [PlatformIO](https://platformio.org/) (project configured for `esp32-s3-devkitc-1`,
Arduino framework).

```powershell
# build
pio run

# flash (port COM12 — change in platformio.ini if needed)
pio run -t upload
```

### Network configuration

- **AP (access point)**: SSID `WeatherSniffer`, password `sniffer123`, address `192.168.4.1`.
  The AP is always on, so you can connect and configure the network.
- **STA (your WiFi)**: configured via the web panel (settings tab). Credentials are stored in the
  device's NVS memory — **do not put them in the source code** (this file may end up in a public
  repository).

### Calibration

The **Calibration** tab lets you set temperature/humidity offsets, wind and rain multipliers, etc.
The web page shows rain as the **increment since the receiver was powered on** (like the console's
"total"), not the station's full lifetime rain counter.

### License

**GPL-2.0-or-later** (see [LICENSE](LICENSE)).

Decoders were ported/based on:

- [rtl_433](https://github.com/merbanan/rtl_433) — GPL-2.0-or-later
  (`vevor_7in1.c`, `bresser_5in1.c`, `bresser_6in1.c`, `bresser_7in1.c`, `fineoffset.c`, `bit_util.c`)
- [BresserWeatherSensorLW](https://github.com/matthias-bs/BresserWeatherSensorLW) — MIT
  (RF configuration reference)
- [FPR36/Vevor-Meteo-station-rf-protocol](https://github.com/FPR36/Vevor-Meteo-station-rf-protocol)
  (YT60309 protocol reference)
