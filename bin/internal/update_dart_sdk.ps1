# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.


# ---------------------------------- NOTE ---------------------------------- #
#
# Please keep the logic in this file consistent with the logic in the
# `update_dart_sdk.sh` script in the same directory to ensure that Flutter
# continues to work across all platforms!
#
# -------------------------------------------------------------------------- #

$ErrorActionPreference = "Stop"

$progName = Split-Path -parent $MyInvocation.MyCommand.Definition
$flutterRoot = (Get-Item $progName).parent.parent.FullName

$cachePath = "$flutterRoot\bin\cache"
$dartSdkPath = "$cachePath\dart-sdk"
$dartSdkLicense = "$cachePath\LICENSE.dart_sdk_archive.md"
$engineStamp = "$cachePath\engine-dart-sdk.stamp"
$engineRealm = ""

# Test for fusion repository
if ((Test-Path "$flutterRoot\DEPS" -PathType Leaf) -and (Test-Path "$flutterRoot\engine\src\.gn" -PathType Leaf)) {
    # Calculate the engine hash from tracked git files.
    $branch = (git -C "$flutterRoot" rev-parse --abbrev-ref HEAD)
    if ($null -eq $Env:LUCI_CONTEXT) {
        $ErrorActionPreference = "Continue"
        git -C "$flutterRoot" remote get-url upstream *> $null
        $exitCode = $?
        $ErrorActionPreference = "Stop"
        if ($exitCode) {
            $engineVersion = (git -C "$flutterRoot"  merge-base HEAD upstream/master)
        } else {
            $engineVersion = (git -C "$flutterRoot"  merge-base HEAD origin/master)
        }
    }
    else {
        $engineVersion = (git -C "$flutterRoot" rev-parse HEAD)
    }

    if (($branch -ne "stable" -and $branch -ne "beta")) {
        # Write the engine version out so downstream tools know what to look for.
        $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
        [System.IO.File]::WriteAllText("$flutterRoot\bin\internal\engine.version", $engineVersion, $utf8NoBom)

        # The realm on CI is passed in.
        if ($Env:FLUTTER_REALM) {
            [System.IO.File]::WriteAllText("$flutterRoot\bin\internal\engine.realm", $Env:FLUTTER_REALM, $utf8NoBom)
        }
    }
}
$engineVersion = (Get-Content "$flutterRoot\bin\internal\engine.version")
$engineRealm = (Get-Content "$flutterRoot\bin\internal\engine.realm")

$oldDartSdkPrefix = "dart-sdk.old"

# Make sure that PowerShell has expected version.
$psMajorVersionRequired = 5
$psMajorVersionLocal = $PSVersionTable.PSVersion.Major
if ($psMajorVersionLocal -lt $psMajorVersionRequired) {
    Write-Host "Flutter requires PowerShell $psMajorVersionRequired.0 or newer."
    Write-Host "See https://flutter.dev/docs/get-started/install/windows for more."
    Write-Host "Current version is $psMajorVersionLocal."
    # Use exit code 2 to signal that shared.bat should exit immediately instead of retrying.
    exit 2
}

if ((Test-Path $engineStamp) -and ($engineVersion -eq (Get-Content $engineStamp))) {
    return
}

$dartSdkBaseUrl = $Env:FLUTTER_STORAGE_BASE_URL
if (-not $dartSdkBaseUrl) {
    $dartSdkBaseUrl = "https://storage.googleapis.com"
}
if ($engineRealm) {
    $dartSdkBaseUrl = "$dartSdkBaseUrl/$engineRealm"
}

# It's important to use the native Dart SDK as the default target architecture
# for Flutter Windows builds depend on the Dart executable's architecture.
$dartZipNameX64 = "dart-sdk-windows-x64.zip"
$dartZipNameArm64 = "dart-sdk-windows-arm64.zip"
$dartZipName = $dartZipNameX64
if ($env:PROCESSOR_ARCHITECTURE -eq "ARM64") {
    $dartSdkArm64Url = "$dartSdkBaseUrl/flutter_infra_release/flutter/$engineVersion/$dartZipNameArm64"
    Try {
        Invoke-WebRequest -Uri $dartSdkArm64Url -UseBasicParsing -Method Head | Out-Null
        $dartZipName = $dartZipNameArm64
    }
    Catch {
        Write-Host "The current channel's Dart SDK does not support Windows Arm64, falling back to Windows x64..."
    }
}
$dartSdkUrl = "$dartSdkBaseUrl/flutter_infra_release/flutter/$engineVersion/$dartZipName"

if ((Test-Path $dartSdkPath) -or (Test-Path $dartSdkLicense)) {
    # Move old SDK to a new location instead of deleting it in case it is still in use (e.g. by IntelliJ).
    $oldDartSdkSuffix = 1
    while (Test-Path "$cachePath\$oldDartSdkPrefix$oldDartSdkSuffix") { $oldDartSdkSuffix++ }

    if (Test-Path $dartSdkPath) {
        Rename-Item $dartSdkPath "$oldDartSdkPrefix$oldDartSdkSuffix"
    }

    if (Test-Path $dartSdkLicense) {
        Rename-Item $dartSdkLicense "$oldDartSdkPrefix$oldDartSdkSuffix.LICENSE.md"
    }
}
New-Item $dartSdkPath -force -type directory | Out-Null
$dartSdkZip = "$cachePath\$dartZipName"

Try {
    Import-Module BitsTransfer
    $ProgressPreference = 'SilentlyContinue'
    Start-BitsTransfer -Source $dartSdkUrl -Destination $dartSdkZip -ErrorAction Stop
}
Catch {
    Write-Host "Downloading the Dart SDK using the BITS service failed, retrying with WebRequest..."
    # Invoke-WebRequest is very slow when the progress bar is visible - a 28
    # second download can become a 33 minute download. Disable it with
    # $ProgressPreference and then restore the original value afterwards.
    # https://github.com/flutter/flutter/issues/37789
    $OriginalProgressPreference = $ProgressPreference
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri $dartSdkUrl -OutFile $dartSdkZip
    $ProgressPreference = $OriginalProgressPreference
}

If (Get-Command 7z -errorAction SilentlyContinue) {
    Write-Host "Expanding downloaded archive with 7z..."
    # The built-in unzippers are painfully slow. Use 7-Zip, if available.
    & 7z x $dartSdkZip "-o$cachePath" -bd | Out-Null
} ElseIf (Get-Command 7za -errorAction SilentlyContinue) {
    Write-Host "Expanding downloaded archive with 7za..."
    # Use 7-Zip's standalone version 7za.exe, if available.
    & 7za x $dartSdkZip "-o$cachePath" -bd | Out-Null
} ElseIf (Get-Command Microsoft.PowerShell.Archive\Expand-Archive -errorAction SilentlyContinue) {
    Write-Host "Expanding downloaded archive with PowerShell..."
    # Use PowerShell's built-in unzipper, if available (requires PowerShell 5+).
    $global:ProgressPreference='SilentlyContinue'
    Microsoft.PowerShell.Archive\Expand-Archive $dartSdkZip -DestinationPath $cachePath
} Else {
    Write-Host "Expanding downloaded archive with Windows..."
    # As last resort: fall back to the Windows GUI.
    $shell = New-Object -com shell.application
    $zip = $shell.NameSpace($dartSdkZip)
    foreach($item in $zip.items()) {
        $shell.Namespace($cachePath).copyhere($item)
    }
}

Remove-Item $dartSdkZip
$engineVersion | Out-File $engineStamp -Encoding ASCII

# Try to delete all old SDKs and license files.
Get-ChildItem -Path $cachePath | Where {$_.BaseName.StartsWith($oldDartSdkPrefix)} | Remove-Item -Recurse -ErrorAction SilentlyContinue
