# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.


# ---------------------------------- NOTE ---------------------------------- #
#
# Please keep the logic in this file consistent with the logic in the
# `update_engine_version.sh` script in the same directory to ensure that Flutter
# continues to work across all platforms!
#
# -------------------------------------------------------------------------- #

$ErrorActionPreference = "Stop"

$progName = Split-Path -parent $MyInvocation.MyCommand.Definition
$flutterRoot = (Get-Item $progName).parent.parent.FullName

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