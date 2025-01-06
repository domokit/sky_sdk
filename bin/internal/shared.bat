@ECHO off
REM Copyright 2014 The Flutter Authors. All rights reserved.
REM Use of this source code is governed by a BSD-style license that can be
REM found in the LICENSE file.

REM ---------------------------------- NOTE ----------------------------------
REM
REM Please keep the logic in this file consistent with the logic in the
REM `shared.sh` script in the same directory to ensure that Flutter & Dart continue to
REM work across all platforms!
REM
REM --------------------------------------------------------------------------

SETLOCAL

SET flutter_tools_dir=%FLUTTER_ROOT%\packages\flutter_tools
SET cache_dir=%FLUTTER_ROOT%\bin\cache
SET snapshot_path=%cache_dir%\flutter_tools.snapshot
SET snapshot_path_old=%cache_dir%\flutter_tools.snapshot.old
SET stamp_path=%cache_dir%\flutter_tools.stamp
SET script_path=%flutter_tools_dir%\bin\flutter_tools.dart
SET dart_sdk_path=%cache_dir%\dart-sdk
SET engine_stamp=%cache_dir%\engine-dart-sdk.stamp
SET engine_version_path=%FLUTTER_ROOT%\bin\internal\engine.version
SET dart=%dart_sdk_path%\bin\dart.exe

REM Ensure that bin/cache exists.
IF NOT EXIST "%cache_dir%" MKDIR "%cache_dir%"

REM If the cache still doesn't exist, fail with an error that we probably don't have permissions.
IF NOT EXIST "%cache_dir%" (
  ECHO Error: Unable to create cache directory at                                 1>&2
  ECHO            %cache_dir%                                                     1>&2
  ECHO.                                                                           1>&2
  ECHO        This may be because flutter doesn't have write permissions for      1>&2
  ECHO        this path. Try moving the flutter directory to a writable location, 1>&2
  ECHO        such as within your home directory.                                 1>&2
  EXIT 1
)

:acquire_lock
2>NUL (
  REM "3" is now stderr because of "2>NUL".
  CALL :subroutine %* 2>&3 9> "%cache_dir%\flutter.bat.lock" || GOTO acquire_lock
)
GOTO :after_subroutine

:subroutine
  REM If present, run the bootstrap script first
  SET bootstrap_path=%FLUTTER_ROOT%\bin\internal\bootstrap.bat
  IF EXIST "%bootstrap_path%" (
    CALL "%bootstrap_path%"
  )

  REM Check that git exists and get the revision.
  WHERE git >NUL 2>&1
  IF "%ERRORLEVEL%" NEQ "0" (
    REM Could not find git. Exit without /B to avoid retrying.
    ECHO Error: Unable to find git in your PATH. && EXIT 1
  )
  2>NUL (
    REM 'FOR /f' spawns a new terminal instance to run the command. If an
    REM 'AutoRun' command is defined in the user's registry, that command could
    REM change the working directory, and then we wouldn't be in the directory
    REM we expect to be in. To prevent this, we need to 'PUSHD %FLUTTER_ROOT%'
    REM before getting the git revision.
    REM
    REM See https://github.com/flutter/flutter/issues/159018
    FOR /f %%r IN ('PUSHD %FLUTTER_ROOT% ^& $git rev-parse HEAD') DO (
      SET revision=%%r
    )
  )
  SET compilekey="%revision%:%FLUTTER_TOOL_ARGS%"

  REM Invalidate cache if:
  REM  * SNAPSHOT_PATH is not a file, or
  REM  * STAMP_PATH is not a file, or
  REM  * STAMP_PATH is an empty file, or
  REM  * Contents of STAMP_PATH is not what we are going to compile, or
  REM  * pubspec.yaml last modified after pubspec.lock

  REM The following IF conditions are all linked with a logical OR. However,
  REM there is no OR operator in batch and a GOTO construct is used as replacement.

  CALL :do_ensure_engine_version
  IF NOT EXIST "%engine_stamp%" GOTO do_sdk_update_and_snapshot
  SET /P dart_required_version=<"%engine_version_path%"
  SET /P dart_installed_version=<"%engine_stamp%"
  IF %dart_required_version% NEQ %dart_installed_version% GOTO do_sdk_update_and_snapshot
  IF NOT EXIST "%snapshot_path%" GOTO do_snapshot
  IF NOT EXIST "%stamp_path%" GOTO do_snapshot
  SET /P stamp_value=<"%stamp_path%"
  IF %stamp_value% NEQ %compilekey% GOTO do_snapshot
  SET pubspec_yaml_path=%flutter_tools_dir%\pubspec.yaml
  SET pubspec_lock_path=%flutter_tools_dir%\pubspec.lock
  FOR /F %%i IN ('DIR /B /O:D "%pubspec_yaml_path%" "%pubspec_lock_path%"') DO SET newer_file=%%i
  FOR %%i IN (%pubspec_yaml_path%) DO SET pubspec_yaml_timestamp=%%~ti
  FOR %%i IN (%pubspec_lock_path%) DO SET pubspec_lock_timestamp=%%~ti
  IF "%pubspec_yaml_timestamp%" == "%pubspec_lock_timestamp%" SET newer_file=""
  IF "%newer_file%" EQU "pubspec.yaml" GOTO do_snapshot

  REM Everything is up-to-date - exit subroutine
  EXIT /B

  :do_ensure_engine_version
    REM Detect which PowerShell executable is available on the Host
    REM PowerShell version <= 5: PowerShell.exe
    REM PowerShell version >= 6: pwsh.exe
    WHERE /Q pwsh.exe && (
        SET powershell_executable=pwsh.exe
    ) || WHERE /Q PowerShell.exe && (
        SET powershell_executable=PowerShell.exe
    ) || (
        ECHO Error: PowerShell executable not found.                        1>&2
        ECHO        Either pwsh.exe or PowerShell.exe must be in your PATH. 1>&2
        EXIT 1
    )
    SET update_engine_bin=%FLUTTER_ROOT%\bin\internal\update_engine_version.ps1
    REM Escape apostrophes from the executable path
    SET "update_engine_bin=%update_engine_bin:'=''%"
    REM PowerShell command must have exit code set in order to prevent all non-zero exit codes from being translated
    REM into 1. The exit code 2 is used to detect the case where the major version is incorrect and there should be
    REM no subsequent retries.
    %powershell_executable% -ExecutionPolicy Bypass -NoProfile -Command "Unblock-File -Path '%update_engine_bin%'; & '%update_engine_bin%'; exit $LASTEXITCODE;"
    IF "%ERRORLEVEL%" EQU "2" (
      EXIT 1
    )
    IF "%ERRORLEVEL%" NEQ "0" (
      ECHO Error: Unable to determine engine version... 1>&2
      EXIT 1
    )
    REM Do not fall through - return from subroutine
    EXIT /B

  :do_sdk_update_and_snapshot
    REM Detect which PowerShell executable is available on the Host
    REM PowerShell version <= 5: PowerShell.exe
    REM PowerShell version >= 6: pwsh.exe
    WHERE /Q pwsh.exe && (
        SET powershell_executable=pwsh.exe
    ) || WHERE /Q PowerShell.exe && (
        SET powershell_executable=PowerShell.exe
    ) || (
        ECHO Error: PowerShell executable not found.                        1>&2
        ECHO        Either pwsh.exe or PowerShell.exe must be in your PATH. 1>&2
        EXIT 1
    )
    ECHO Checking Dart SDK version... 1>&2
    SET update_dart_bin=%FLUTTER_ROOT%\bin\internal\update_dart_sdk.ps1
    REM Escape apostrophes from the executable path
    SET "update_dart_bin=%update_dart_bin:'=''%"
    REM PowerShell command must have exit code set in order to prevent all non-zero exit codes from being translated
    REM into 1. The exit code 2 is used to detect the case where the major version is incorrect and there should be
    REM no subsequent retries.
    ECHO Downloading Dart SDK from Flutter engine %dart_required_version%... 1>&2
    %powershell_executable% -ExecutionPolicy Bypass -NoProfile -Command "Unblock-File -Path '%update_dart_bin%'; & '%update_dart_bin%'; exit $LASTEXITCODE;"
    IF "%ERRORLEVEL%" EQU "2" (
      EXIT 1
    )
    IF "%ERRORLEVEL%" NEQ "0" (
      ECHO Error: Unable to update Dart SDK. Retrying... 1>&2
      timeout /t 5 /nobreak
      GOTO :do_sdk_update_and_snapshot
    )

  :do_snapshot
    IF EXIST "%FLUTTER_ROOT%\version" DEL "%FLUTTER_ROOT%\version"
    IF EXIST "%FLUTTER_ROOT%\bin\cache\flutter.version.json" DEL "%FLUTTER_ROOT%\bin\cache\flutter.version.json"
    ECHO: > "%cache_dir%\.dartignore"

    ECHO Building flutter tool... 1>&2
    PUSHD "%flutter_tools_dir%"

    REM Makes changes to PUB_ENVIRONMENT only visible to commands within SETLOCAL/ENDLOCAL
    SETLOCAL
      IF "%CI%" == "true" GOTO on_bot
      IF "%BOT%" == "true" GOTO on_bot
      IF "%CONTINUOUS_INTEGRATION%" == "true" GOTO on_bot
      IF "%CHROME_HEADLESS%" == "1" GOTO on_bot
      GOTO not_on_bot
      :on_bot
        SET PUB_ENVIRONMENT=%PUB_ENVIRONMENT%:flutter_bot
      :not_on_bot
      SET PUB_SUMMARY_ONLY=1
      SET PUB_ENVIRONMENT=%PUB_ENVIRONMENT%:flutter_install
      IF "%PUB_CACHE%" == "" (
        IF EXIST "%pub_cache_path%" SET PUB_CACHE=%pub_cache_path%
      )

      SET /A total_tries=10
      SET /A remaining_tries=%total_tries%-1
      :retry_pub_upgrade
        ECHO Running pub upgrade... 1>&2
        "%dart%" pub upgrade --suppress-analytics
        IF "%ERRORLEVEL%" EQU "0" goto :upgrade_succeeded
        ECHO Error (%ERRORLEVEL%): Unable to 'pub upgrade' flutter tool. Retrying in five seconds... (%remaining_tries% tries left) 1>&2
        timeout /t 5 /nobreak 2>NUL
        SET /A remaining_tries-=1
        IF "%remaining_tries%" EQU "0" GOTO upgrade_retries_exhausted
        GOTO :retry_pub_upgrade
      :upgrade_retries_exhausted
        SET exit_code=%ERRORLEVEL%
        ECHO Error: 'pub upgrade' still failing after %total_tries% tries. 1>&2
        GOTO final_exit
      :upgrade_succeeded
    ENDLOCAL

    POPD

    REM Move the old snapshot - we can't just overwrite it as the VM might currently have it
    REM memory mapped (e.g. on flutter upgrade), and deleting it might not work if the file
    REM is in use. For downloading a new dart sdk the folder is moved, so we take the same
    REM approach of moving the file here.
    SET /A snapshot_path_suffix=1
    :move_old_snapshot
      IF EXIST "%snapshot_path_old%%snapshot_path_suffix%" (
        SET /A snapshot_path_suffix+=1
        GOTO move_old_snapshot
      ) ELSE (
        IF EXIST "%snapshot_path%" (
          MOVE "%snapshot_path%" "%snapshot_path_old%%snapshot_path_suffix%" 2> NUL > NUL
        )
      )

    IF "%FLUTTER_TOOL_ARGS%" == "" (
      "%dart%" --verbosity=error --snapshot="%snapshot_path%" --snapshot-kind="app-jit" --packages="%flutter_tools_dir%\.dart_tool\package_config.json" --no-enable-mirrors "%script_path%" > NUL
    ) else (
      "%dart%" "%FLUTTER_TOOL_ARGS%" --verbosity=error --snapshot="%snapshot_path%" --snapshot-kind="app-jit" --packages="%flutter_tools_dir%\.dart_tool\package_config.json" "%script_path%" > NUL
    )
    IF "%ERRORLEVEL%" NEQ "0" (
      ECHO Error: Unable to create dart snapshot for flutter tool. 1>&2
      SET exit_code=%ERRORLEVEL%
      GOTO :final_exit
    )
    >"%stamp_path%" ECHO %compilekey%

    REM Try to delete any old snapshots now. Swallow any errors though.
    DEL "%snapshot_path%.old*" 2> NUL > NUL

  REM Exit Subroutine
  EXIT /B

:after_subroutine

:final_exit
  EXIT /B %exit_code%
