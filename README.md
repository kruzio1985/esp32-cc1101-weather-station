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

Obsługiwane dwie płytki: **ESP32-S3** (DevKitC-1, 8 MB flash) oraz klasyczny
**ESP32 WROOM-32** (4 MB flash, bez PSRAM). Firmware działa na obu — bez PSRAM,
bo zużywa tylko ~55 kB RAM (wewnętrzna pamięć wystarcza z dużym zapasem).

Moduł **CC1101 868 MHz**.

#### ESP32-S3 (DevKitC-1)

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

#### ESP32 WROOM-32 (DevKitC v4)

> **Uwaga 1:** na klasycznym ESP32 piny GPIO 6–11 są zajęte przez wewnętrzny flash,
> więc podłączenie jest INNE niż na S3.
>
> **Uwaga 2 (ważne):** CSN **musi** być na GPIO27, a **nie** na GPIO5.
> Na klasycznym ESP32 GPIO5 to sprzętowy **CS0 kontrolera VSPI** — tego samego,
> którego używamy (GPIO18/19/23 to piny IOMUX VSPI). Sprzętowy CS0 przejmuje
> wtedy linię i przełącza ją między bajtami transakcji. Objaw jest bardzo
> mylący: odczyty rejestrów CC1101 działają, a **zapisy nigdy nie docierają**
> (rejestry zostają na wartościach fabrycznych, radio siedzi na 800 MHz i nie
> wchodzi w RX). Na S3 ten problem nie występuje, bo używany tam GPIO10 nie
> jest CS0 tej magistrali.

| CC1101 | ESP32 WROOM | (odpowiednik S3) |
|---|---|---|
| CSN   | GPIO27 | 10 → 27 |
| SCK   | GPIO18 | 12 → 18 |
| MISO/SO | GPIO19 | 13 → 19 |
| MOSI/SI | GPIO23 | 11 → 23 |
| GDO0  | GPIO4  | 5 → 4   |
| GDO2  | GPIO16 | 6 → 16  |
| VCC   | 3.3 V  | —       |
| GND   | GND    | —       |

SPI: prędkość dobierana automatycznie (100 kHz…4 MHz), tryb 0, magistrala VSPI.
Firmware sam sprawdza zapis/odczyt i wybiera najszybszą działającą prędkość,
a test powtarza co 30 s — dzięki temu działa też na dłuższych kablach.
Antena CC1101 powinna być ustawiona pionowo,
z dala od ESP32 (ESP32 z WiFi generuje szumy) i z dala od metalowych elementów.
Przy dłuższych przewodach zasilających dodaj kondensator 100 nF (i opcjonalnie 47–100 µF)
tuż przy pinach VCC–GND modułu.

### Budowanie i wgrywanie

Wymagania: [PlatformIO](https://platformio.org/) (framework Arduino).

```powershell
# ESP32-S3 (domyślne)
pio run
pio run -t upload

# ESP32 WROOM-32 (4 MB, bez PSRAM)
pio run -e esp32-wroom
pio run -e esp32-wroom -t upload
```

Port USB ustaw w `platformio.ini` (`upload_port` / `monitor_port`), jeśli urządzenie
dostaje inny port niż `COM12`.

> **Wgrywanie na płytkę WROOM:** ta płytka często nie robi autoresetu niezawodnie.
> Jeśli `pio run -e esp32-wroom -t upload` zgłasza `Wrong boot mode detected`,
> wymuś tryb download ręcznie: **przytrzymaj BOOT**, naciśnij i puść **EN/RST**,
> nadal trzymaj BOOT aż wgrywanie ruszy (`Wrote ... bytes`). W `platformio.ini`
> dla tego środowiska ustawione jest `upload_flags = --before no-reset` — dzięki
> temu esptool nie „walczy" z przyciskiem.
>
> Jeśli płytka zgłasza `Invalid head of packet` — to zakłócenie synchronizacji;
> po prostu powtórz wgrywanie.

### Aktualizacja firmware przez OTA (bez USB i przycisku)

Firmware ma wbudowaną aktualizację przez WWW — wystarczy przeglądarka:

1. Wejdź na **http://192.168.1.130/update** (albo `http://192.168.4.1/update`, gdy urządzenie jest w trybie AP).
2. Wybierz plik **`firmware.bin`** (sama aplikacja, ~1,2 MB — **NIE** `firmware.factory.bin`).
3. Kliknij „Wgraj i zrestartuj". Urządzenie się zrestartuje z nowym firmware.

Dzięki temu kolejne aktualizacje **nie wymagają USB ani przycisku BOOT**.
Plik `firmware.bin` powstaje po `pio run -e esp32-wroom` w katalogu
`.pio/build/esp32-wroom/firmware.bin`.

> **Uwaga:** OTA wymaga tablicy partycji z dwoma partycjami aplikacji
> (`min_spiffs.csv` — ustawiona domyślnie dla WROOM). Jeśli na płytce jest stara
> tablica `huge_app.csv`, wgraj raz przez USB (BOOT+EN), a potem już zawsze OTA.

### Konfiguracja sieci

- **AP (punkt dostępowy)**: SSID `WeatherSniffer`, hasło `sniffer123`, adres `192.168.4.1`.
  AP jest włączony tylko wtedy, gdy **nie** masz skonfigurowanego WiFi STA. Po połączeniu
  z domowym WiFi AP się wyłącza — to celowe: beacony AP zagłuszały odbiornik 868 MHz.
- **STA (Twoje WiFi)**: konfiguruje się przez panel WWW (zakładka ustawień). Dane są zapisywane
  w pamięci NVS urządzenia — **nie wpisuj ich w kodzie źródłowym** (ten plik może trafić na
  publiczne repozytorium).

> **Moc WiFi:** firmware celowo obniża moc nadawczą WiFi do 2 dBm. Pełna moc ESP32
> (~20 dBm) wstrzykiwała szum do CC1101 i zagłuszała słabe ramki stacji pogodowej
> (RSSI spadało o ~12 dB). 2 dBm w zupełności wystarcza na kilka metrów do routera.

> **Uwaga:** wgranie scalonego obrazu `firmware.factory.bin` od adresu `0x0`
> (np. `esptool write-flash 0x0 firmware.factory.bin`) **kasuje NVS** — tracisz
> zapisane WiFi, tryb pracy i kalibrację. Normalne `pio run -t upload` wgrywa
> tylko aplikację i NVS zostaje. Po takim wgraniu urządzenie startuje w trybie AP
> i trzeba WiFi skonfigurować ponownie.

### Diagnostyka radia

Firmware sam sprawdza tor SPI i wypisuje wynik na porcie szeregowym (115200):

- `SPI autotune: 4000k=A5 ... -> OK, wybrano X kHz` — zapis do CC1101 dociera,
  wybrano najszybszą działającą prędkość SPI. `A5` to wartość, którą zapisano
  i odczytano z powrotem.
- `CC1101 VERSION=0x14 PARTNUM=0x0` — układ odpowiada prawidłowo.
- `CC1101: RX aktywny (MARCSTATE=0x0D)` — radio weszło w odbiór.
- `-> BLAD: zapis do CC1101 nie dociera...` — sygnał problemu z okablowaniem.
  Na płytce WROOM najczęstsza przyczyna to **CSN na GPIO5** (patrz tabela
  podłączenia wyżej).

Test powtarzany jest co 30 s, więc jeśli połączenie się pogorszy, firmware sam
przejdzie na wolniejszą prędkość SPI.

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

Two boards are supported: **ESP32-S3** (DevKitC-1, 8 MB flash) and the classic
**ESP32 WROOM-32** (4 MB flash, no PSRAM). The firmware works on both without PSRAM —
it only uses ~55 kB of RAM (internal memory is plenty).

**CC1101 868 MHz** module.

#### ESP32-S3 (DevKitC-1)

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

#### ESP32 WROOM-32 (DevKitC v4)

> **Note 1:** on the classic ESP32, GPIO 6–11 are used by the internal flash,
> so the wiring is DIFFERENT from the S3.
>
> **Note 2 (important):** CSN **must** be on GPIO27, **not** on GPIO5.
> On the classic ESP32 GPIO5 is the hardware **CS0 of the VSPI controller** —
> the very controller used here (GPIO18/19/23 are the VSPI IOMUX pins). The
> hardware CS0 then takes over the line and toggles it between the bytes of a
> transaction. The symptom is very misleading: CC1101 register *reads* work
> while *writes* never arrive (registers stay at factory defaults, the radio
> stays on 800 MHz and never enters RX). This does not happen on the S3 because
> the GPIO10 used there is not the CS0 of that bus.

| CC1101 | ESP32 WROOM | (S3 equivalent) |
|---|---|---|
| CSN   | GPIO27 | 10 → 27 |
| SCK   | GPIO18 | 12 → 18 |
| MISO/SO | GPIO19 | 13 → 19 |
| MOSI/SI | GPIO23 | 11 → 23 |
| GDO0  | GPIO4  | 5 → 4   |
| GDO2  | GPIO16 | 6 → 16  |
| VCC   | 3.3 V  | —       |
| GND   | GND    | —       |

SPI: speed selected automatically (100 kHz…4 MHz), mode 0, VSPI bus.
The firmware verifies a write/read-back and picks the fastest working speed,
re-testing every 30 s — so it also works on longer wires.
Keep the CC1101 antenna vertical, away from the ESP32
(the ESP32 with WiFi produces noise) and away from metal. For longer power wires, add a 100 nF
capacitor (and optionally 47–100 µF) close to the module's VCC–GND pins.

### Build & flash

Requires [PlatformIO](https://platformio.org/) (Arduino framework).

```powershell
# ESP32-S3 (default)
pio run
pio run -t upload

# ESP32 WROOM-32 (4 MB, no PSRAM)
pio run -e esp32-wroom
pio run -e esp32-wroom -t upload
```

Set the USB port in `platformio.ini` (`upload_port` / `monitor_port`) if the device
gets a different port than `COM12`.

> **Flashing the WROOM board:** auto-reset is often unreliable on this board. If
> `pio run -e esp32-wroom -t upload` reports `Wrong boot mode detected`, enter
> download mode manually: **hold BOOT**, press and release **EN/RST**, keep
> holding BOOT until flashing starts (`Wrote ... bytes`). `platformio.ini` sets
> `upload_flags = --before no-reset` for this environment so that esptool does
> not fight the button.
>
> An `Invalid head of packet` error is a sync glitch — just retry the upload.

### Over-the-air firmware update (no USB, no button)

The firmware has a built-in web update — just use a browser:

1. Open **http://192.168.1.130/update** (or `http://192.168.4.1/update` in AP mode).
2. Choose the **`firmware.bin`** file (the application only, ~1.2 MB — **NOT** `firmware.factory.bin`).
3. Click "Wgraj i zrestartuj" (upload and restart). The device reboots with the new firmware.

This means future updates **need no USB and no BOOT button**. `firmware.bin` is produced
by `pio run -e esp32-wroom` at `.pio/build/esp32-wroom/firmware.bin`.

> **Note:** OTA requires a partition table with two app partitions
> (`min_spiffs.csv` — the default for WROOM). If the board still has the old
> `huge_app.csv`, flash it once over USB (BOOT+EN), then use OTA from then on.

### Network configuration

- **AP (access point)**: SSID `WeatherSniffer`, password `sniffer123`, address `192.168.4.1`.
  The AP is enabled only when **no** STA WiFi is configured. Once connected to your home
  WiFi the AP turns off — this is intentional: the AP beacons were desensitizing the 868 MHz
  receiver.
- **STA (your WiFi)**: configured via the web panel (settings tab). Credentials are stored in the
  device's NVS memory — **do not put them in the source code** (this file may end up in a public
  repository).

> **WiFi power:** the firmware deliberately lowers the WiFi TX power to 2 dBm. Full ESP32
> power (~20 dBm) injected noise into the CC1101 and masked the station's weak frames
> (RSSI dropped by ~12 dB). 2 dBm is plenty for a router a few meters away.

> **Note:** flashing a merged `firmware.factory.bin` from address `0x0`
> (e.g. `esptool write-flash 0x0 firmware.factory.bin`) **erases NVS** — you lose
> the saved WiFi, operating mode and calibration. A normal `pio run -t upload`
> writes only the application, so NVS survives. After such a flash the device
> boots in AP mode and WiFi must be configured again.

### Radio diagnostics

The firmware validates the SPI path by itself and prints the result on the serial
port (115200):

- `SPI autotune: 4000k=A5 ... -> OK, wybrano X kHz` — writes reach the CC1101 and
  the fastest working SPI speed was selected. `A5` is the value written to a
  register and read back.
- `CC1101 VERSION=0x14 PARTNUM=0x0` — the chip responds correctly.
- `CC1101: RX aktywny (MARCSTATE=0x0D)` — the radio entered RX.
- `-> BLAD: zapis do CC1101 nie dociera...` — a wiring problem. On the WROOM board
  the most common cause is **CSN on GPIO5** (see the wiring table above).

The test repeats every 30 s, so if the connection degrades the firmware
automatically falls back to a slower SPI speed.

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
