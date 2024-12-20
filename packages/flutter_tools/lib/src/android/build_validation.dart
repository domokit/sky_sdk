// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/common.dart';
import '../build_info.dart';

const String kGooglePlayVersioning = 'https://developer.android.com/studio/publish/versioning.html';
const String kSupportedAbis = 'https://flutter.dev/to/android-supported-architectures';

/// Validates that the build mode and build number are valid for a given build.
void validateBuild(AndroidBuildInfo androidBuildInfo) {
  final BuildInfo buildInfo = androidBuildInfo.buildInfo;
  if (buildInfo.codeSizeDirectory != null && androidBuildInfo.targetArchs.length > 1) {
    throwToolExit(
      'Cannot perform code size analysis when building for multiple ABIs. '
      'Specify one of android-arm, android-arm64, or android-x64 in the '
      '--target-platform flag.',
    );
  }
  if (buildInfo.mode.isPrecompiled && androidBuildInfo.targetArchs.contains(AndroidArch.x86)) {
    throwToolExit(
      'Cannot build ${androidBuildInfo.buildInfo.mode.cliName} mode for x86 ABI.\n'
      'For more information see $kSupportedAbis .',
    );
  }
  if (buildInfo.buildNumber != null) {
    final int? result = int.tryParse(buildInfo.buildNumber!);
    if (result == null) {
      throwToolExit(
        'buildNumber: ${buildInfo.buildNumber} was not a valid integer value.\n'
        'For more information see $kGooglePlayVersioning .',
      );
    }
    if (result < 0) {
      throwToolExit(
        'buildNumber: ${buildInfo.buildNumber} must be a positive integer value.\n'
        'For more information see $kGooglePlayVersioning .',
      );
    }
    if (result > 2100000000) {
      throwToolExit(
        'buildNumber: ${buildInfo.buildNumber} is greater than the maximum '
        'allowed value of 2100000000.\n'
        'For more information see $kGooglePlayVersioning .',
      );
    }
  }
}
