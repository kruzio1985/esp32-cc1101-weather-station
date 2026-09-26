<#
  build.ps1 - build i (opcjonalnie) wgranie firmware odbiornika stacji 868 MHz.

  Po co ten skrypt: PlatformIO potrafi przerwac build bledem
      TypeError: argument should be a str or an os.PathLike object ... not 'NoneType'
  albo brakiem naglowka (esp32-hal-ldo.h / pins_arduino.h). Dzieje sie tak, gdy build
  trafi w moment, w ktorym PlatformIO reinstaluje pakiet frameworku Arduino. Skrypt
  rozpoznaje ten przejsciowy blad, sam robi "pio pkg install" i powtarza build.

  UZYCIE (w katalogu projektu):
      .\build.ps1                          # build esp32-wroom (domyslne)
      .\build.ps1 -Env esp32s3             # build ESP32-S3
      .\build.ps1 -Env all                 # build obu wariantow
      .\build.ps1 -Upload                  # build + wgranie po USB
      .\build.ps1 -Upload -Port COM8       # ... ze wskazaniem portu
      .\build.ps1 -Clean                   # czyszczenie przed buildem

  Jesli PowerShell blokuje skrypty ("running scripts is disabled"), uruchom tak:
      powershell -ExecutionPolicy Bypass -File .\build.ps1
#>

[CmdletBinding()]
param(
    [ValidateSet('esp32-wroom', 'esp32s3', 'all')]
    [string]$Env = 'esp32-wroom',

    [switch]$Upload,

    [string]$Port = '',

    [switch]$Clean,

    [int]$MaxAttempts = 3,

    [string]$PioPath = ''
)

$ErrorActionPreference = 'Continue'

# Wzorce bledow, ktore sa przejsciowe - wynikaja z reinstalacji pakietu frameworku.
$script:TransientPatterns = @(
    "not 'NoneType'",
    'esp32-hal-ldo\.h: No such file or directory',
    'esp32-hal-periman\.h: No such file or directory',
    'pins_arduino\.h: No such file or directory'
)

function Resolve-Pio {
    param([string]$Explicit)

    if ($Explicit) {
        if (Test-Path $Explicit) { return $Explicit }
        Write-Host "Blad: nie ma pliku pio pod podana sciezka: $Explicit" -ForegroundColor Red
        exit 1
    }

    $onPath = Get-Command pio -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }

    $candidates = @(
        (Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe')
    )
    foreach ($c in $candidates) {
        if ($c -and (Test-Path $c)) { return $c }
    }

    $pyRoot = Join-Path $env:LOCALAPPDATA 'Programs\Python'
    if (Test-Path $pyRoot) {
        $found = Get-ChildItem $pyRoot -Directory -ErrorAction SilentlyContinue |
                 ForEach-Object { Join-Path $_.FullName 'Scripts\pio.exe' } |
                 Where-Object { Test-Path $_ } |
                 Select-Object -First 1
        if ($found) { return $found }
    }

    Write-Host 'Blad: nie znalazlem pio.exe.' -ForegroundColor Red
    Write-Host 'Podaj sciezke recznie, np.:' -ForegroundColor Yellow
    Write-Host '  .\build.ps1 -PioPath "C:\Users\...\Scripts\pio.exe"'
    exit 1
}

function Invoke-Pio {
    param([string[]]$PioArgs)

    $out = & $script:Pio @PioArgs 2>&1
    return @{ Code = $LASTEXITCODE; Text = ($out | Out-String) }
}

function Test-TransientFailure {
    param([string]$Text)

    foreach ($p in $script:TransientPatterns) {
        if ($Text -match $p) { return $true }
    }
    return $false
}

function Get-SizeSummary {
    param([string]$Text)

    $ram   = [regex]::Matches($Text, '(?m)^RAM:\s+\[[^\]]*\]\s+([0-9.]+)%')
    $flash = [regex]::Matches($Text, '(?m)^Flash:\s+\[[^\]]*\]\s+([0-9.]+)%')
    if ($ram.Count -gt 0 -and $flash.Count -gt 0) {
        return "RAM $($ram[$ram.Count-1].Groups[1].Value)%   Flash $($flash[$flash.Count-1].Groups[1].Value)%"
    }
    return ''
}

function Build-Target {
    param([string]$Target)

    $logFile = Join-Path $env:TEMP "weathersniffer_build_$Target.log"
    $attempt = 0
    $ok = $false
    $result = $null

    # Czyszczenie musi byc OSOBNYM wywolaniem: "pio run -t clean" tylko czysci
    # i nie kompiluje, wiec dolaczenie go do builda nic by nie zbudowalo.
    if ($Clean) {
        Write-Host "[$Target] czyszczenie..." -ForegroundColor DarkGray
        [void](Invoke-Pio -PioArgs @('run', '-e', $Target, '-t', 'clean'))
    }

    while ($attempt -lt $MaxAttempts) {
        $attempt++
        Write-Host ("[{0}] build, proba {1}/{2}..." -f $Target, $attempt, $MaxAttempts) -ForegroundColor Cyan

        $result = Invoke-Pio -PioArgs @('run', '-e', $Target)
        $result.Text | Set-Content -Path $logFile -Encoding UTF8

        if ($result.Code -eq 0) { $ok = $true; break }

        if ((Test-TransientFailure -Text $result.Text) -and ($attempt -lt $MaxAttempts)) {
            Write-Host '  Przejsciowy blad pakietu frameworku - naprawiam i powtarzam.' -ForegroundColor Yellow
            Write-Host "  pio pkg install -e $Target"
            [void](Invoke-Pio -PioArgs @('pkg', 'install', '-e', $Target))
            continue
        }
        break
    }

    $summary = Get-SizeSummary -Text $result.Text

    if ($ok) {
        Write-Host ("[{0}] OK   {1}" -f $Target, $summary) -ForegroundColor Green
    } else {
        Write-Host ("[{0}] BLAD (proby: {1})" -f $Target, $attempt) -ForegroundColor Red
        if (Test-TransientFailure -Text $result.Text) {
            Write-Host '  To nadal blad pakietu. Sprobuj recznie:' -ForegroundColor Yellow
            Write-Host "    pio pkg install -e $Target"
        } else {
            Write-Host "  Pelny log: $logFile" -ForegroundColor Yellow
            $errLines = ($result.Text -split "`r?`n") | Where-Object { $_ -match 'error:|Error' } | Select-Object -First 8
            foreach ($l in $errLines) { Write-Host "  $l" -ForegroundColor DarkYellow }
        }
    }

    return @{ Ok = $ok; Log = $logFile; Text = $result.Text }
}

function Upload-Target {
    param([string]$Target)

    Write-Host "[$Target] wgrywanie po USB..." -ForegroundColor Cyan
    $pioArgs = @('run', '-e', $Target, '-t', 'upload')
    if ($Port) { $pioArgs += @('--upload-port', $Port) }

    $result = Invoke-Pio -PioArgs $pioArgs
    $logFile = Join-Path $env:TEMP "weathersniffer_upload_$Target.log"
    $result.Text | Set-Content -Path $logFile -Encoding UTF8

    if ($result.Code -eq 0) {
        Write-Host "[$Target] wgrane OK" -ForegroundColor Green
        return $true
    }

    if ($result.Text -match 'Wrong boot mode|Failed to connect|No serial data received') {
        Write-Host '  Plytka nie weszla w tryb wgrywania. Procedura reczna:' -ForegroundColor Yellow
        Write-Host '    1. przytrzymaj BOOT'
        Write-Host '    2. nacisnij i pusc EN (albo RST)'
        Write-Host '    3. nadal trzymajac BOOT czekaj na "Wrote ... bytes"'
        Write-Host '    4. pusc BOOT'
        Write-Host '  Czasem trzeba kilku prob - to normalne dla WROOM z CH340.'
    } elseif ($result.Text -match 'Could not open COM|port is busy|FileNotFoundError') {
        Write-Host '  Nie moge otworzyc portu - jest zajety albo go nie ma.' -ForegroundColor Yellow
        Write-Host '  Sprawdz dostepne porty:'
        Write-Host '    pio device list'
        Write-Host '  Najczestsza przyczyna: otwarty monitor szeregowy trzyma port.'
        Write-Host '  Zamknij go i podaj port recznie, np.:'
        Write-Host '    build -Upload -Port COM8'
    } elseif ($result.Text -match 'Please specify|multiple|More than one') {
        Write-Host '  Wykryto wiecej niz jedno urzadzenie. Wskaz port:' -ForegroundColor Yellow
        Write-Host '    build -Upload -Port COM8'
    } elseif ($result.Text -match 'Invalid head of packet') {
        Write-Host '  Zaklocenie transmisji. Powtorz, ewentualnie zwolnij tempo:' -ForegroundColor Yellow
        Write-Host '    pio run -e esp32-wroom -t upload --upload-port COM8 --upload-speed 115200'
    } else {
        Write-Host "[$Target] wgrywanie nie udalo sie. Log: $logFile" -ForegroundColor Red
    }
    return $false
}

# ------------------------------- glowna czesc -------------------------------

if (-not (Test-Path 'platformio.ini')) {
    Write-Host 'Blad: nie ma tu platformio.ini.' -ForegroundColor Red
    Write-Host 'Uruchom skrypt w katalogu projektu, np.:' -ForegroundColor Yellow
    Write-Host '  cd C:\Projects\OfflineWorkspace\HISTORIA\PROJEKTY\HOME_ASSISTANT\WMBUS\WMBUS_WeatherSniffer'
    exit 1
}

$script:Pio = Resolve-Pio -Explicit $PioPath
Write-Host "PlatformIO: $script:Pio" -ForegroundColor DarkGray
Write-Host "Katalog   : $(Get-Location)" -ForegroundColor DarkGray
Write-Host ''

$targets = if ($Env -eq 'all') { @('esp32-wroom', 'esp32s3') } else { @($Env) }
$buildFailed = $false
$uploadFailed = $false

foreach ($t in $targets) {
    $b = Build-Target -Target $t
    if (-not $b.Ok) { $buildFailed = $true; continue }
    if ($Upload) {
        if (-not (Upload-Target -Target $t)) { $uploadFailed = $true }
    }
    Write-Host ''
}

if ($buildFailed) {
    Write-Host 'WYNIK: build nie udal sie.' -ForegroundColor Red
    exit 1
}
if ($uploadFailed) {
    Write-Host 'WYNIK: build OK, ale wgrywanie nie udalo sie.' -ForegroundColor Yellow
    exit 2
}

Write-Host 'WYNIK: gotowe.' -ForegroundColor Green
exit 0
