# ESP32 (WROOM-32 / S3) + CC1101 — Odbiornik stacji pogodowej 868 MHz
# ESP32 (WROOM-32 / S3) + CC1101 — 868 MHz Weather Station Receiver

[Polski](#polski) · [English](#english)

Firmware dla **ESP32** — klasyczny **WROOM-32** albo **ESP32-S3** — z modułem radiowym
**CC1101** (868 MHz). Odbiera i dekoduje
bezprzewodowe stacje pogodowe (głównie **VEVOR / Youtong 7-w-1, protokół 263**) i udostępnia
dane przez stronę WWW, JSON, HTTP oraz MQTT.

Firmware for an **ESP32** (classic **WROOM-32** or **ESP32-S3**) with a **CC1101** (868 MHz)
radio module. It receives and decodes
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

### Obsługiwane stacje pogodowe

Firmware ma kilka dekoderów (wzorowanych na bibliotece **rtl_433**) i tryb **AUTO**,
który sam sprawdza po kolei wszystkie protokoły. Poniżej pełna lista:

| Tryb / dekoder | Stacja | Częstotliwość | Protokół |
|---|---|---|---|
| `vevor` ⭐ | **VEVOR / Youtong 7-w-1 — YT60231** (EU) | 868.30 MHz | 2-FSK 11.11k, sync `CA 54`, 263 (`vevor_7in1` w rtl_433) |
| `vevor` | VEVOR / Youtong 7-w-1 — **YT60238, YT60240, LWS234** | 868.30 MHz | ten sam protokół 263 |
| `vevor` | **VEVOR YT60234** (USA) | 915 MHz | ten sam protokół, inne skalowanie wiatru i opadu |
| `vevor_yt60309` | **VEVOR / Youtong YT60309** (CMT2119A) | 868.30 / **868.35** MHz | 2-FSK 11.11k, sync `C0AA C0AA`, 32 bajty |
| `fine_offset` | **Fine Offset WH24 / WH65 / WS69** (rodzina 0x24) | 868.30 MHz | 2-FSK 17.24k, sync `2D D4`, 17 bajtów, CRC-8 |
| `bresser` | **Bresser 5-w-1 / 6-w-1 / 7-w-1** (np. 7002510, 7002520, 7002580) | 868.30 MHz | 2-FSK ~8.21k, sync `AA 2D`, 27 bajtów |
| `auto` | wszystkie powyższe po kolei | 868.20 / 868.30 / 868.35 / 868.40 / 868.95 / 433.92 MHz | po 20 s na kombinację, wybiera pierwszy działający |
| `raw_scan` | dowolny nadajnik (tryb diagnostyczny) | 6 częstotliwości × 8 profili | surowe ramki, bez dekodowania |

⭐ = tryb domyślny i **przetestowany na prawdziwym sprzęcie**.

> Stacje z tej samej rodziny (np. rebrandy VEVOR, Youtong, LWS) używają **tego samego
> protokołu**, więc zwykle działają od razu. Jeśli Twoja stacja nadaje inaczej, użyj
> trybu **AUTO** albo trybu **raw_scan** i prześlij zebrane ramki — wtedy dopiszę dekoder.

Dane dekodowane z każdej stacji: temperatura, wilgotność, prędkość wiatru (średnia),
poryw wiatru, kierunek wiatru (16 kierunków), suma opadu, indeks UV, natężenie światła (lux),
flaga baterii oraz ID nadajnika (zależnie od tego, co dana stacja wysyła).

> **Ciśnienie atmosferyczne NIE jest przesyłane przez radio.** Barometr znajduje się w konsoli
> (wyświetlaczu), a nie w czujniku zewnętrznym — dlatego odbiornik nie może go odebrać. To
> dotyczy wszystkich stacji z tej listy.

### WAŻNE — jakiego modułu radiowego użyć (nie klona!)

Do tego odbiornika potrzebny jest **prawdziwy moduł CC1101** na **868 MHz**, najlepiej
sprawdzony odpowiednik modułów **RADIOCONTROLLI** / **ELECHOUSE** (np. ten sam, co w projekcie
`WMBUS_RadioControl_866MHz`). Ten firmware był pisany i testowany na takim module.

**Nie używaj tanich klonów.** Klony CC1101 (głównie z aukcji „CC1101 868MHz" bez marki) bardzo
często mają:
- **zły rezonator/kwarc** (np. 27 MHz zamiast **26 MHz**) — wtedy odbiornik działa na złej
  częstotliwości i łapie tylko szum; objaw: `VERSION=0x14` jest OK, ale stacja nie jest
  odbierana,
- brak ekranowania i słabe filtry — duży szum własny (wysoki „podłogowy" RSSI),
- brak kondensatorów odsprzęgających przy układzie — radio niestabilnie startuje.

Zły moduł można rozpoznać po tym, że **ta sama stacja i ten sam firmware** na oryginalnym
module działa, a na klonie nie — mimo poprawnych połączeń SPI.

**Jak sprawdzić, czy moduł jest dobry** (wbudowana diagnostyka):
- `CC1101 VERSION=0x14 PARTNUM=0x0` — układ odpowiada prawidłowo,
- `SPI autotune: ... -> OK` — zapisy do rejestrów docierają,
- `CC1101: RX aktywny (MARCSTATE=0x0D)` — radio weszło w odbiór,
- `RSSI` przy braku transmisji powinien być niski (np. -100…-110 dBm). Jeśli „na pusto"
  pokazuje -70 dBm i więcej, moduł ma duży szum własny (typowe dla klonów) albo jest
  zagłuszany.

#### Gdzie kupić **oryginalny** moduł

Poniżej tylko **oryginały** (producent i duży dystrybutor). Celowo **nie ma tu linków do
tanich klonów** z aukcji — właśnie one są najczęstszą przyczyną „radio nie odbiera".

| Moduł | Sklep | Uwagi |
|---|---|---|
| **[RADIOCONTROLLI RC-CC1101-SPI-868 (THT)](https://www.tme.eu/pl/details/rc-cc1101-spi-868/moduly-rf/radiocontrolli/)** | **TME** | ⭐ polecany — oryginalny moduł RADIOCONTROLLI, **868 MHz**, SPI, czułość -110 dBm, 1,8–3,6 V, 10 dBm, 21,5×15,6 mm, montaż przewlekany |
| **[RADIOCONTROLLI RC-CC1101-SPI-SMT-868 (SMD)](https://www.tme.eu/pl/details/rc-cc1101-smt-868/moduly-rf/radiocontrolli/rc-cc1101-spi-smt-868/)** | **TME** | ta sama rodzina, montaż powierzchniowy 15×18 mm |
| **[RADIOCONTROLLI RC-CC1101-SPI-434 (THT)](https://www.tme.eu/pl/details/rc-cc1101-spi-434/moduly-rf/radiocontrolli/)** — pasmo **433 MHz** | **TME** | oryginał na 433 MHz (gdy odbiornik ma pracować na 433,92 MHz) |
| **[Moduł CC1101 868 MHz — sklep producenta](https://shop.radiocontrolli.com/en/rf-modules-434868mhz/48-rc-cc1101-spi-868.html)** | RADIOCONTROLLI | bezpośrednio od producenta (Włochy) — oryginał |
| **[Moduł CC1101 433 MHz — sklep producenta](https://shop.radiocontrolli.com/en/rf-modules-434868mhz/47-rc-cc1101-spi-434.html)** | RADIOCONTROLLI | bezpośrednio od producenta — wersja 433 MHz |

**Zalecenie:** kup oryginalny moduł **RADIOCONTROLLI** (TME albo sklep producenta). Ma
poprawny rezonator 26 MHz, ekranowanie i powtarzalne parametry — czyli dokładnie to, czego
ten odbiornik potrzebuje do słabych ramek.

Tanie klony (aukcje typu „CC1101 868 MHz z anteną", ~15 zł) potrafią działać, ale nie ma
gwarancji, że mają rezonator 26 MHz — a bez niego **nie odbiorą 868 MHz**. Jeśli mimo to
używasz takiego modułu, dołóż kondensator 100 nF + 10 µF przy VCC i sprawdź go diagnostyką
opisaną wyżej.

> Uwaga: nie każdy „CC1101 868 MHz" z aukcji ma wlutowany rezonator **26 MHz**. Zdarzają się
> płytki z rezonatorem 27 MHz (od wersji 433 MHz) — taki moduł **nie odbierze 868 MHz**.

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

Wymagania: [PlatformIO](https://platformio.org/) (framework Arduino). Nic więcej —
**wszystko, czego trzeba, PlatformIO pobierze samo**.

```powershell
# ESP32-S3 (domyślne)
pio run
pio run -t upload

# ESP32 WROOM-32 (4 MB, bez PSRAM)
pio run -e esp32-wroom
pio run -e esp32-wroom -t upload
```

Port USB jest **wykrywany automatycznie** — w `platformio.ini` nie ma żadnych ustawień
specyficznych dla konkretnego komputera, więc projekt buduje się tak samo na każdym PC.
Jeśli automat wybierze zły port, podaj go ręcznie:

```powershell
pio run -e esp32-wroom -t upload --upload-port COM5
```

#### Prościej: gotowy skrypt `build.cmd`

W katalogu projektu jest skrypt, który **sam powtarza build**, gdy PlatformIO przerwie go
przejściowym błędem pakietu (zamiast tego trzeba wtedy pamiętać o `pio pkg install`):

```powershell
build                          # build esp32-wroom
build -Env esp32s3             # build ESP32-S3
build -Env all                 # oba warianty
build -Upload                  # build + wgranie po USB
build -Upload -Port COM8       # ... ze wskazaniem portu
build -Clean                   # czyszczenie przed buildem
```

`build.cmd` uruchamia `build.ps1` z `-ExecutionPolicy Bypass`, więc działa nawet gdy
PowerShell blokuje skrypty. Skrypt pokazuje zajętość Flash/RAM, a przy błędzie wgrywania
podpowiada procedurę BOOT + EN/RST (albo że port jest zajęty).

#### Ważne: wymagany rdzeń Arduino 3.x

Firmware używa API, które istnieje dopiero w **Arduino-ESP32 core 3.x**. Oficjalna
platforma `espressif32` z rejestru PlatformIO (wersja 6.x) ma jeszcze rdzeń **2.x** i
kompilacja się nie uda (`esp_task_wdt_reconfigure was not declared`). Dlatego w
`platformio.ini` platforma jest przypięta do wydania **pioarduino 55.03.312**
(rdzeń Arduino 3.3.12) w formie adresu URL:

```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.312/platform-espressif32.zip
```

PlatformIO pobierze je automatycznie przy pierwszym budowaniu (kilkadziesiąt MB, chwilę
to trwa).

> **Pierwszy build może się nie udać** — PlatformIO instaluje wtedy platformę i narzędzia
> i potrafi przerwać z błędem typu `TypeError: ... not 'NoneType'` albo brakiem nagłówka
> (`esp32-hal-ldo.h: No such file or directory`, `pins_arduino.h: No such file or directory`).
> Wtedy brakuje pakietu frameworku. Napraw to tak:
>
> ```bash
> pio pkg install -e esp32-wroom
> pio run -e esp32-wroom
> ```
>
> Samo powtórzenie `pio run` może nie wystarczyć. Ten projekt był sprawdzony pod kątem
> czystego builda (świeży katalog bez `.pio`) na obu środowiskach:
> `esp32-wroom` → OK (Flash 65,5 %, RAM 20,9 %), `esp32s3` → OK (Flash 38,0 %, RAM 20,2 %).
>
> **Nie buduj tego projektu platformą `espressif32` z rejestru PlatformIO** (to Arduino
> core 2.x). Instaluje ona framework pod tą samą nazwą i nadpisuje właściwą wersję 3.3.12,
> przez co kolejne buildy padają.

> **Wgrywanie na płytkę WROOM:** niektóre płytki WROOM-32 (szczególnie z konwerterem
> CH340) nie robią autoresetu niezawodnie. Jeśli `pio run -e esp32-wroom -t upload`
> zgłasza `Wrong boot mode detected`, wymuś tryb download ręcznie:
>
> 1. naciśnij i **przytrzymaj BOOT**,
> 2. nie puszczając BOOT naciśnij i puść **EN/RST**,
> 3. trzymaj BOOT aż wgrywanie ruszy (zobaczysz `Wrote ... bytes`).
>
> Jeśli płytka zgłasza `Invalid head of packet` — to zakłócenie synchronizacji przy
> dużej prędkości; po prostu powtórz wgrywanie albo zejdź na wolniejszą prędkość:
>
> ```powershell
> pio run -e esp32-wroom -t upload --upload-port COM5 --upload-speed 115200
> ```
>
> **Najprościej:** po pierwszym wgraniu przez USB aktualizuj firmware przez **OTA**
> (patrz niżej) — bez kabla, bez przycisków i bez zmiany czegokolwiek w konfiguracji.

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

> **Moc i uśpienie WiFi:** firmware nadaje z mocą **15 dBm** i **wyłącza uśpienie modemu WiFi**
> (`WiFi.setSleep(false)`). Historia dojścia do tego: **2 dBm** okazało się za słabe (nie
> przechodziło OTA), **8,5 dBm** dawało serie timeoutów połączenia ze stacją główną, a pełna moc
> ESP32 (~20 dBm) wstrzykiwała szum do CC1101 i zagłuszała słabe ramki stacji pogodowej
> (RSSI spadało o ~12 dB). **15 dBm** to kompromis: łącze ma zapas, a zakłócenie 868 MHz
> pozostaje wyraźnie niższe niż przy pełnej mocy (na czas OTA moc chwilowo rośnie do 19,5 dBm).
> **Uśpienie modemu wyłączono 2026-10-04** — dodawało ~100 ms opóźnień (DTIM 102,4 ms) i gubiło
> pakiety, przez co stacja główna odnotowywała timeouts (1,5 s) i wpadała w 5-minutowe przerwy
> w danych. Jeśli po tej zmianie pojawią się brownouty, firmware **sam włączy uśpienie z powrotem**
> (ochrona zasilania). Aktualny stan widać w `/status` jako pole `wifiSleep`.

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

### Zdalna diagnostyka, log błędów i watchdog

Cała strona WWW jest **dwujęzyczna (polski / angielski)** — etykiety mają obie
wersje, np. „Temperatura / Temperature".

| Adres | Co pokazuje |
|---|---|
| `/` | odczyt na żywo (odświeżanie przez JavaScript, **bez przeładowania strony**) |
| `/log` | log zdarzeń + pełny stan radia i systemu (widok konsoli, PL/EN) |
| `/log?raw=1` | to samo jako czysty tekst (do skryptów) |
| `/errors` | plik błędów z przyciskiem kasowania i testu |
| `/errors?raw=1` | plik błędów jako tekst |
| `/errors?clear=1` | skasowanie pliku błędów |
| `/status` | stan w JSON (do skryptów / Home Assistant) |
| `/update` | aktualizacja firmware przez przeglądarkę (OTA) |
| `/reboot` | zdalny restart |

**Watchdog:** firmware pilnuje się sprzętowym watchdogiem (30 s). Jeśli pętla
główna się zawiesi, ESP restartuje się sam i po restarcie **zapisuje przyczynę**
(`watchdog`, `brownout`, `crash`) — widać ją na stronie i w `/log`.

**Log zdarzeń w RAM:** bufor kołowy (40 wpisów). Nowe zdarzenia nadpisują
najstarsze, więc pamięć **nigdy się nie zapełni** — nie trzeba nic kasować.

**Plik błędów:** do pliku na flashu (`/bledy.log`) trafiają **tylko błędy**
(zwykłe zdarzenia zostają w RAM). Plik ma wbudowaną rotację, żeby miejsce nigdy
się nie skończyło:
- maksymalny rozmiar 16 kB — po przekroczeniu zostają najnowsze wpisy (8 kB),
- wpisy **starsze niż 7 dni** są kasowane automatycznie (wymaga czasu z NTP),
- ręczne kasowanie: `/errors?clear=1`.

**Ochrona pamięci:** firmware sprawdza co minutę ilość wolnej pamięci RAM.
Jeśli spadnie poniżej 12 kB, restartuje się *zanim* zabraknie pamięci (a zdarzenie
zapisuje do pliku błędów). Dzięki temu urządzenie może pracować tygodniami.

### Diagnostyka radia (English)

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

### Supported weather stations

The firmware contains several decoders (modelled on **rtl_433**) plus an **AUTO** mode that
tries every protocol in turn. Full list:

| Mode / decoder | Station | Frequency | Protocol |
|---|---|---|---|
| `vevor` ⭐ | **VEVOR / Youtong 7-in-1 — YT60231** (EU) | 868.30 MHz | 2-FSK 11.11k, sync `CA 54`, protocol 263 (`vevor_7in1` in rtl_433) |
| `vevor` | VEVOR / Youtong 7-in-1 — **YT60238, YT60240, LWS234** | 868.30 MHz | same protocol 263 |
| `vevor` | **VEVOR YT60234** (US) | 915 MHz | same protocol, different wind/rain scaling |
| `vevor_yt60309` | **VEVOR / Youtong YT60309** (CMT2119A) | 868.30 / **868.35** MHz | 2-FSK 11.11k, sync `C0AA C0AA`, 32 bytes |
| `fine_offset` | **Fine Offset WH24 / WH65 / WS69** (family 0x24) | 868.30 MHz | 2-FSK 17.24k, sync `2D D4`, 17 bytes, CRC-8 |
| `bresser` | **Bresser 5-in-1 / 6-in-1 / 7-in-1** (e.g. 7002510, 7002520, 7002580) | 868.30 MHz | 2-FSK ~8.21k, sync `AA 2D`, 27 bytes |
| `auto` | all of the above, one after another | 868.20 / 868.30 / 868.35 / 868.40 / 868.95 / 433.92 MHz | 20 s per combination, picks the first that works |
| `raw_scan` | any transmitter (diagnostic mode) | 6 frequencies × 8 profiles | raw frames, no decoding |

⭐ = default mode, **tested on real hardware**.

> Stations from the same family (VEVOR/Youtong/LWS rebrands) use the **same protocol**, so they
> normally work out of the box. If your station transmits differently, use **AUTO** or
> **raw_scan** and send me the captured frames — I will add a decoder.

Decoded from each station: temperature, humidity, average wind speed, wind gust, wind direction
(16 directions), rain total, UV index, light (lux), battery flag and transmitter ID
(depending on what that station transmits).

> **Atmospheric pressure is NOT transmitted over the radio.** The barometer is inside the
> console (display), not in the outdoor sensor — so the receiver cannot pick it up. This applies
> to every station in the list above.

### IMPORTANT — which radio module to use (not a clone!)

You need a **genuine CC1101** module for **868 MHz** — ideally a proven **RADIOCONTROLLI** /
**ELECHOUSE**-class part (the same kind used in the `WMBUS_RadioControl_866MHz` project).
This firmware was written and tested on such a module.

**Do not use cheap clones.** Unbranded "CC1101 868MHz" boards very often have:
- the **wrong crystal/resonator** (e.g. 27 MHz instead of **26 MHz**) — the receiver then works
  on the wrong frequency and only picks up noise; symptom: `VERSION=0x14` is fine but the
  station is never received,
- no shielding and poor filters — high self-noise (a high "noise floor" RSSI),
- no decoupling capacitors at the chip — the radio starts unreliably.

A bad module is recognisable by this: **the same station and the same firmware** works on a
genuine module but not on the clone, despite correct SPI wiring.

**How to check that your module is good** (built-in diagnostics):
- `CC1101 VERSION=0x14 PARTNUM=0x0` — the chip responds correctly,
- `SPI autotune: ... -> OK` — register writes arrive,
- `CC1101: RX aktywny (MARCSTATE=0x0D)` — the radio entered RX,
- the idle `RSSI` should be low (e.g. -100…-110 dBm). If it reads -70 dBm or higher with no
  transmission, the module has high self-noise (typical for clones) or is being jammed.

#### Where to buy a **genuine** module

Only **originals** below (the manufacturer and a large distributor). There are deliberately
**no links to cheap clones** from marketplace listings — those are the most common cause of
"the radio does not receive anything".

| Module | Shop | Notes |
|---|---|---|
| **[RADIOCONTROLLI RC-CC1101-SPI-868 (THT)](https://www.tme.eu/pl/details/rc-cc1101-spi-868/moduly-rf/radiocontrolli/)** | **TME** | ⭐ recommended — genuine RADIOCONTROLLI module, **868 MHz**, SPI, -110 dBm sensitivity, 1.8–3.6 V, 10 dBm, 21.5×15.6 mm, through-hole |
| **[RADIOCONTROLLI RC-CC1101-SPI-SMT-868 (SMD)](https://www.tme.eu/pl/details/rc-cc1101-smt-868/moduly-rf/radiocontrolli/rc-cc1101-spi-smt-868/)** | **TME** | same family, surface-mount version 15×18 mm |
| **[RADIOCONTROLLI RC-CC1101-SPI-434 (THT)](https://www.tme.eu/pl/details/rc-cc1101-spi-434/moduly-rf/radiocontrolli/)** — **433 MHz** band | **TME** | genuine 433 MHz part (if the receiver is to work on 433.92 MHz) |
| **[CC1101 868 MHz module — manufacturer shop](https://shop.radiocontrolli.com/en/rf-modules-434868mhz/48-rc-cc1101-spi-868.html)** | RADIOCONTROLLI | straight from the manufacturer (Italy) — genuine |
| **[CC1101 433 MHz module — manufacturer shop](https://shop.radiocontrolli.com/en/rf-modules-434868mhz/47-rc-cc1101-spi-434.html)** | RADIOCONTROLLI | straight from the manufacturer — 433 MHz version |

**Recommendation:** buy the genuine **RADIOCONTROLLI** module (TME or the manufacturer shop).
It has the correct 26 MHz resonator, shielding and repeatable parameters — exactly what this
receiver needs for weak frames.

Cheap clones ("CC1101 868 MHz with antenna" listings, ~15 PLN) can work, but there is no
guarantee they carry a 26 MHz resonator — and without it they **will not receive 868 MHz**.
If you use one anyway, add a 100 nF + 10 µF capacitor at VCC and verify it with the
diagnostics described above.

> Note: not every "CC1101 868 MHz" auction listing has a **26 MHz** resonator fitted. Boards
> with a 27 MHz resonator (from the 433 MHz version) exist — such a module **will not receive
> 868 MHz**.

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

Requires [PlatformIO](https://platformio.org/) (Arduino framework). Nothing else —
**PlatformIO downloads everything needed automatically**.

```powershell
# ESP32-S3 (default)
pio run
pio run -t upload

# ESP32 WROOM-32 (4 MB, no PSRAM)
pio run -e esp32-wroom
pio run -e esp32-wroom -t upload
```

The USB port is **auto-detected** — `platformio.ini` contains no machine-specific
settings, so the project builds identically on any PC. If auto-detection picks the wrong
port, pass it explicitly:

```powershell
pio run -e esp32-wroom -t upload --upload-port COM5
```

#### Simpler: the included `build.cmd` script

The project ships a script that **retries the build automatically** when PlatformIO aborts
with a transient package error (otherwise you have to remember to run `pio pkg install`):

```powershell
build                          # build esp32-wroom
build -Env esp32s3             # build ESP32-S3
build -Env all                 # both variants
build -Upload                  # build + flash over USB
build -Upload -Port COM8       # ... specifying the port
build -Clean                   # clean before building
```

`build.cmd` invokes `build.ps1` with `-ExecutionPolicy Bypass`, so it works even when
PowerShell blocks scripts. The script prints Flash/RAM usage, and on a failed upload it
suggests the BOOT + EN/RST procedure (or tells you the port is busy).

#### Important: Arduino core 3.x is required

The firmware uses an API that only exists in **Arduino-ESP32 core 3.x**. The official
`espressif32` platform from the PlatformIO registry (version 6.x) still ships core **2.x**
and the build fails with `esp_task_wdt_reconfigure was not declared`. That is why
`platformio.ini` pins the platform to the **pioarduino 55.03.312** release (Arduino core
3.3.12) via a URL:

```ini
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.312/platform-espressif32.zip
```

PlatformIO downloads it on the first build (tens of MB, takes a moment).

> **The first build may fail** — PlatformIO is installing the platform and tools and can
> abort with an error such as `TypeError: ... not 'NoneType'`, or with a missing header
> (`esp32-hal-ldo.h: No such file or directory`, `pins_arduino.h: No such file or directory`).
> In that case the framework package is missing. Fix it with:
>
> ```bash
> pio pkg install -e esp32-wroom
> pio run -e esp32-wroom
> ```
>
> Simply re-running `pio run` may not help. This project was verified with a clean build
> (fresh directory, no `.pio`) on both environments:
> `esp32-wroom` → OK (Flash 65.5 %, RAM 20.9 %), `esp32s3` → OK (Flash 38.0 %, RAM 20.2 %).
>
> **Do not build this project with the registry `espressif32` platform** (that is Arduino
> core 2.x). It installs the framework under the same name and overwrites the correct
> 3.3.12 version, which makes subsequent builds fail.

> **Flashing the WROOM board:** some WROOM-32 boards (especially those with a CH340
> converter) do not auto-reset reliably. If `pio run -e esp32-wroom -t upload` reports
> `Wrong boot mode detected`, enter download mode manually:
>
> 1. press and **hold BOOT**,
> 2. without releasing BOOT, press and release **EN/RST**,
> 3. keep holding BOOT until flashing starts (you will see `Wrote ... bytes`).
>
> An `Invalid head of packet` error is a sync glitch at high speed — just retry, or drop
> to a slower speed:
>
> ```powershell
> pio run -e esp32-wroom -t upload --upload-port COM5 --upload-speed 115200
> ```
>
> **Easiest:** after the first USB flash, update the firmware over **OTA** (see below) —
> no cable, no buttons and no configuration changes.

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

> **WiFi power and sleep:** the firmware transmits at **15 dBm** and **disables WiFi modem sleep**
> (`WiFi.setSleep(false)`). How we got here: **2 dBm** was too weak (OTA failed),
> **8.5 dBm** caused bursts of connection timeouts towards the main station, and full ESP32
> power (~20 dBm) injected noise into the CC1101 and masked the station's weak frames
> (RSSI dropped by ~12 dB). **15 dBm** is the compromise: the link has margin while the
> 868 MHz interference stays well below full-power levels (during OTA the power temporarily
> rises to 19.5 dBm). **Modem sleep was disabled on 2026-10-04** — it added ~100 ms latency
> (DTIM 102.4 ms) and dropped packets, which made the main station log timeouts (1.5 s) and
> enter 5-minute data gaps. If brownouts appear after this change, the firmware **re-enables
> sleep automatically** (power protection). Current state is shown in `/status` as `wifiSleep`.

> **Note:** flashing a merged `firmware.factory.bin` from address `0x0`
> (e.g. `esptool write-flash 0x0 firmware.factory.bin`) **erases NVS** — you lose
> the saved WiFi, operating mode and calibration. A normal `pio run -t upload`
> writes only the application, so NVS survives. After such a flash the device
> boots in AP mode and WiFi must be configured again.

### Remote diagnostics, error log and watchdog

The whole web UI is **bilingual (Polish / English)** — labels carry both versions,
e.g. "Temperatura / Temperature".

| URL | What it shows |
|---|---|
| `/` | live reading (refreshed by JavaScript, **no page reload**) |
| `/log` | event log + full radio and system status (console view, PL/EN) |
| `/log?raw=1` | the same as plain text (for scripts) |
| `/errors` | error file with clear and test buttons |
| `/errors?raw=1` | error file as plain text |
| `/errors?clear=1` | delete the error file |
| `/status` | status as JSON (for scripts / Home Assistant) |
| `/update` | firmware update from the browser (OTA) |
| `/reboot` | remote reboot |

**Watchdog:** the firmware guards itself with a hardware watchdog (30 s). If the
main loop hangs, the ESP reboots on its own and **records the reason** after
restart (`watchdog`, `brownout`, `crash`) — visible on the page and in `/log`.

**RAM event log:** a ring buffer (40 entries). New events overwrite the oldest,
so memory **never fills up** — nothing to clear manually.

**Error file:** only **errors** go to the flash file (`/bledy.log`) — regular
events stay in RAM. The file rotates so space never runs out:
- maximum size 16 kB — beyond that only the newest entries (8 kB) are kept,
- entries **older than 7 days** are deleted automatically (needs NTP time),
- manual clear: `/errors?clear=1`.

**Memory guard:** every minute the firmware checks free RAM. If it drops below
12 kB it restarts *before* memory runs out (and writes the event to the error
file). This lets the device run for weeks.

### Radio diagnostics (original section)

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
