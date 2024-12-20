:: felt: a command-line utility for Windows for building and testing
:: Flutter web engine.
:: FELT stands for Flutter Engine Local Tester.

@ECHO OFF
SETLOCAL

:: Starting from this script's path, walk up to engine source directory.
SET SCRIPT_DIR=%~dp0
FOR %%a IN ("%SCRIPT_DIR:~0,-1%") DO SET TMP=%%~dpa
FOR %%a IN ("%TMP:~0,-1%") DO SET TMP=%%~dpa
FOR %%a IN ("%TMP:~0,-1%") DO SET TMP=%%~dpa
FOR %%a IN ("%TMP:~0,-1%") DO SET ENGINE_SRC_DIR=%%~dpa

SET ENGINE_SRC_DIR=%ENGINE_SRC_DIR:~0,-1%
SET FLUTTER_DIR=%ENGINE_SRC_DIR%\flutter
SET WEB_UI_DIR=%FLUTTER_DIR%\lib\web_ui
SET SDK_PREBUILTS_DIR=%FLUTTER_DIR%\prebuilts
SET PREBUILT_TARGET=windows-x64
IF NOT DEFINED DART_SDK_DIR (
  SET DART_SDK_DIR=%SDK_PREBUILTS_DIR%\%PREBUILT_TARGET%\dart-sdk
)
SET DART_BIN=%DART_SDK_DIR%\bin\dart

cd %WEB_UI_DIR%

:: We need to invoke pub get here before we actually invoke felt.
%DART_BIN% pub get
%DART_BIN% run dev/felt.dart %*

EXIT /B %ERRORLEVEL%
