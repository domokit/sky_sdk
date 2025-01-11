// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/platform.dart';
import '../doctor_validator.dart';
import 'chrome.dart';

/// A validator for Chromium-based browsers.
abstract class ChromiumValidator extends DoctorValidator {
  ChromiumValidator(super.title);

  Platform get _platform;
  ChromiumLauncher get _chromiumLauncher;
  String get _name;

  @override
  Future<ValidationResult> validateImpl() async {
    final bool canRunChromium = _chromiumLauncher.canFindExecutable();
    final String chromiumSearchLocation = _chromiumLauncher.findExecutable();
    final List<ValidationMessage> messages = <ValidationMessage>[
      if (_platform.environment.containsKey(kChromeEnvironment))
        if (!canRunChromium)
          ValidationMessage.hint('$chromiumSearchLocation is not executable.')
        else
          ValidationMessage('$kChromeEnvironment = $chromiumSearchLocation')
      else if (!canRunChromium)
        ValidationMessage.hint(
          'Cannot find $_name. Try setting '
          '$kChromeEnvironment to a $_name executable.',
        )
      else
        ValidationMessage('$_name at $chromiumSearchLocation'),
    ];
    if (!canRunChromium) {
      return ValidationResult(
        ValidationType.missing,
        messages,
        statusInfo: 'Cannot find $_name executable at $chromiumSearchLocation',
      );
    }
    return ValidationResult(ValidationType.success, messages);
  }
}

/// A validator that checks whether Chrome is installed and can run.
class ChromeValidator extends ChromiumValidator {
  ChromeValidator({
    required Platform platform,
    required ChromiumLauncher chromiumLauncher,
  })  : _platform = platform,
        _chromiumLauncher = chromiumLauncher,
        super('Chrome - develop for the web');

  @override
  final Platform _platform;

  @override
  final ChromiumLauncher _chromiumLauncher;

  @override
  String get _name => 'Chrome';
}

/// A validator that checks whether Edge is installed and can run.
class EdgeValidator extends ChromiumValidator {
  EdgeValidator({
    required Platform platform,
    required ChromiumLauncher chromiumLauncher,
  })  : _platform = platform,
        _chromiumLauncher = chromiumLauncher,
        super('Edge - develop for the web');

  @override
  final Platform _platform;

  @override
  final ChromiumLauncher _chromiumLauncher;

  @override
  String get _name => 'Edge';
}
