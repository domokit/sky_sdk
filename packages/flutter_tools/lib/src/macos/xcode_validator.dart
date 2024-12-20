// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/user_messages.dart';
import '../base/version.dart';
import '../build_info.dart';
import '../doctor_validator.dart';
import '../ios/simulators.dart';
import 'xcode.dart';

String _iOSSimulatorMissing(String version) => '''
iOS $version Simulator not installed; this may be necessary for iOS and macOS development.
To download and install the platform, open Xcode, select Xcode > Settings > Components,
and click the GET button for the required platform.

For more information, please visit:
  https://developer.apple.com/documentation/xcode/installing-additional-simulator-runtimes''';

class XcodeValidator extends DoctorValidator {
  XcodeValidator({
    required Xcode xcode,
    required IOSSimulatorUtils iosSimulatorUtils,
    required UserMessages userMessages,
  }) : _xcode = xcode,
       _iosSimulatorUtils = iosSimulatorUtils,
       _userMessages = userMessages,
       super('Xcode - develop for iOS and macOS');

  final Xcode _xcode;
  final IOSSimulatorUtils _iosSimulatorUtils;
  final UserMessages _userMessages;

  @override
  Future<ValidationResult> validate() async {
    final List<ValidationMessage> messages = <ValidationMessage>[];
    ValidationType xcodeStatus = ValidationType.missing;
    String? xcodeVersionInfo;

    final String? xcodeSelectPath = _xcode.xcodeSelectPath;

    if (_xcode.isInstalled) {
      xcodeStatus = ValidationType.success;
      if (xcodeSelectPath != null) {
        messages.add(ValidationMessage(_userMessages.xcodeLocation(xcodeSelectPath)));
      }
      final String? versionText = _xcode.versionText;
      if (versionText != null) {
        xcodeVersionInfo = versionText;
        if (xcodeVersionInfo.contains(',')) {
          xcodeVersionInfo = xcodeVersionInfo.substring(0, xcodeVersionInfo.indexOf(','));
        }
      }
      if (_xcode.buildVersion != null) {
        messages.add(ValidationMessage('Build ${_xcode.buildVersion}'));
      }
      if (!_xcode.isInstalledAndMeetsVersionCheck) {
        xcodeStatus = ValidationType.partial;
        messages.add(
          ValidationMessage.error(_userMessages.xcodeOutdated(xcodeRequiredVersion.toString())),
        );
      } else if (!_xcode.isRecommendedVersionSatisfactory) {
        xcodeStatus = ValidationType.partial;
        messages.add(
          ValidationMessage.hint(
            _userMessages.xcodeRecommended(xcodeRecommendedVersion.toString()),
          ),
        );
      }

      if (!_xcode.eulaSigned) {
        xcodeStatus = ValidationType.partial;
        messages.add(ValidationMessage.error(_userMessages.xcodeEula));
      }
      if (!_xcode.isSimctlInstalled) {
        xcodeStatus = ValidationType.partial;
        messages.add(ValidationMessage.error(_userMessages.xcodeMissingSimct));
      }

      final ValidationMessage? missingSimulatorMessage = await _validateSimulatorRuntimeInstalled();
      if (missingSimulatorMessage != null) {
        xcodeStatus = ValidationType.partial;
        messages.add(missingSimulatorMessage);
      }
    } else {
      xcodeStatus = ValidationType.missing;
      if (xcodeSelectPath == null || xcodeSelectPath.isEmpty) {
        messages.add(ValidationMessage.error(_userMessages.xcodeMissing));
      } else {
        messages.add(ValidationMessage.error(_userMessages.xcodeIncomplete));
      }
    }

    return ValidationResult(xcodeStatus, messages, statusInfo: xcodeVersionInfo);
  }

  /// Validate the Xcode-installed iOS simulator SDK has a corresponding iOS
  /// simulator runtime installed.
  ///
  /// Starting with Xcode 15, the iOS simulator runtime is no longer downloaded
  /// with Xcode and must be downloaded and installed separately.
  /// iOS applications cannot be run without it.
  Future<ValidationMessage?> _validateSimulatorRuntimeInstalled() async {
    // Skip this validation if Xcode is not installed, Xcode is a version less
    // than 15, simctl is not installed, or if the EULA is not signed.
    if (!_xcode.isInstalled ||
        _xcode.currentVersion == null ||
        _xcode.currentVersion!.major < 15 ||
        !_xcode.isSimctlInstalled ||
        !_xcode.eulaSigned) {
      return null;
    }

    final Version? platformSDKVersion = await _xcode.sdkPlatformVersion(EnvironmentType.simulator);
    if (platformSDKVersion == null) {
      return const ValidationMessage.error('Unable to find the iPhone Simulator SDK.');
    }

    final List<IOSSimulatorRuntime> runtimes = await _iosSimulatorUtils.getAvailableIOSRuntimes();
    if (runtimes.isEmpty) {
      return const ValidationMessage.error('Unable to get list of installed Simulator runtimes.');
    }

    // Verify there is a simulator runtime installed matching the
    // iphonesimulator SDK major version.
    try {
      runtimes.firstWhere(
        (IOSSimulatorRuntime runtime) => runtime.version?.major == platformSDKVersion.major,
      );
    } on StateError {
      return ValidationMessage.hint(_iOSSimulatorMissing(platformSDKVersion.toString()));
    }

    return null;
  }
}
