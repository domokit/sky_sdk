@ECHO off
REM Copyright 2013 The Flutter Authors. All rights reserved.
REM Use of this source code is governed by a BSD-style license that can be
REM found in the LICENSE file.

REM ---------------------------------- NOTE ----------------------------------
REM
REM Please keep the logic in this file consistent with the logic in the
REM `yapf.sh` script in the same directory to ensure that it continues to
REM work across all platforms!
REM
REM --------------------------------------------------------------------------

SET yapf_path=%~dp0\..\..\flutter\third_party\yapf

cmd /V /C "SET PYTHONPATH=%yapf_path%&& vpython3 %yapf_path%\yapf %*"
