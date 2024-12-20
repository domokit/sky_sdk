@ECHO off
REM Copyright 2013 The Flutter Authors. All rights reserved.
REM Use of this source code is governed by a BSD-style license that can be
REM found in the LICENSE file.

REM ---------------------------------- NOTE ----------------------------------
REM
REM Please keep the logic in this file consistent with the logic in the
REM `format.sh` script in the same directory to ensure that it continues to
REM work across all platforms!
REM
REM --------------------------------------------------------------------------

SETLOCAL ENABLEDELAYEDEXPANSION

FOR %%i IN ("%~dp0..\..") DO SET SRC_DIR=%%~fi

REM Test if Git is available on the Host
where /q git || ECHO Error: Unable to find git in your PATH. && EXIT /B 1

SET repo_dir=%SRC_DIR%\flutter
SET ci_dir=%repo_dir%\ci

REM Determine which platform we are on and use the right prebuilt Dart SDK
IF "%PROCESSOR_ARCHITECTURE%"=="AMD64" (
    SET dart_sdk_path=%SRC_DIR%\flutter\prebuilts\windows-x64\dart-sdk
) ELSE IF "%PROCESSOR_ARCHITECTURE%"=="ARM64" (
    SET dart_sdk_path=%SRC_DIR%\flutter\prebuilts\windows-arm64\dart-sdk
) ELSE (
    ECHO "Windows x86 (32-bit) is not supported" && EXIT /B 1
)

SET dart=%dart_sdk_path%\bin\dart.exe

cd "%ci_dir%"

REM Do not use the CALL command in the next line to execute Dart. CALL causes
REM Windows to re-read the line from disk after the CALL command has finished
REM regardless of the ampersand chain.
"%dart%" bin\format.dart %* & exit /B !ERRORLEVEL!
