@echo off
REM ---------------------------------------------------------------------------
REM  build.cmd - launcher do build.ps1
REM
REM  Po co: Windows domyslnie blokuje uruchamianie skryptow .ps1
REM  ("running scripts is disabled on this system"). Ten plik uruchamia
REM  build.ps1 z -ExecutionPolicy Bypass, wiec dziala bez zmiany ustawien systemu.
REM
REM  UZYCIE (w katalogu projektu):
REM      build                        build esp32-wroom
REM      build -Env esp32s3           build ESP32-S3
REM      build -Env all               build obu wariantow
REM      build -Upload                build + wgranie po USB
REM      build -Upload -Port COM8     ... ze wskazaniem portu
REM ---------------------------------------------------------------------------
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" %*
exit /b %ERRORLEVEL%
