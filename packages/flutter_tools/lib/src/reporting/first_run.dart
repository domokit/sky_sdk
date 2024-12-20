// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:convert/convert.dart';
import 'package:crypto/crypto.dart';

import '../convert.dart';
import '../persistent_tool_state.dart';

/// This message is displayed on the first run of the Flutter tool, or anytime
/// that the contents of this string change.
const String _kFlutterFirstRunMessage = '''
  ╔════════════════════════════════════════════════════════════════════════════╗
  ║                 Welcome to Flutter! - https://flutter.dev                  ║
  ║                                                                            ║
  ║ The Flutter tool uses Google Analytics to anonymously report feature usage ║
  ║ statistics and basic crash reports. This data is used to help improve      ║
  ║ Flutter tools over time.                                                   ║
  ║                                                                            ║
  ║ Flutter tool analytics are not sent on the very first run. To disable      ║
  ║ reporting, type 'flutter config --no-analytics'. To display the current    ║
  ║ setting, type 'flutter config'. If you opt out of analytics, an opt-out    ║
  ║ event will be sent, and then no further information will be sent by the    ║
  ║ Flutter tool.                                                              ║
  ║                                                                            ║
  ║ By downloading the Flutter SDK, you agree to the Google Terms of Service.  ║
  ║ The Google Privacy Policy describes how data is handled in this service.   ║
  ║                                                                            ║
  ║ Moreover, Flutter includes the Dart SDK, which may send usage metrics and  ║
  ║ crash reports to Google.                                                   ║
  ║                                                                            ║
  ║ Read about data we send with crash reports:                                ║
  ║ https://flutter.dev/to/crash-reporting                                     ║
  ║                                                                            ║
  ║ See Google's privacy policy:                                               ║
  ║ https://policies.google.com/privacy                                        ║
  ║                                                                            ║
  ║ To disable animations in this tool, use                                    ║
  ║ 'flutter config --no-cli-animations'.                                      ║
  ╚════════════════════════════════════════════════════════════════════════════╝
''';

/// The first run messenger determines whether the first run license terms
/// need to be displayed.
class FirstRunMessenger {
  FirstRunMessenger({required PersistentToolState persistentToolState})
    : _persistentToolState = persistentToolState;

  final PersistentToolState _persistentToolState;

  /// Whether the license terms should be displayed.
  ///
  /// This is implemented by caching a hash of the previous license terms. This
  /// does not update the cache hash value.
  ///
  /// The persistent tool state setting [PersistentToolState.redisplayWelcomeMessage]
  /// can also be used to make this return false. This is primarily used to ensure
  /// that the license terms are not printed during a `flutter upgrade`, until the
  /// user manually runs the tool.
  bool shouldDisplayLicenseTerms() {
    if (_persistentToolState.shouldRedisplayWelcomeMessage == false) {
      return false;
    }
    final String? oldHash = _persistentToolState.lastActiveLicenseTermsHash;
    return oldHash != _currentHash;
  }

  /// Update the cached license terms hash once the new terms have been displayed.
  void confirmLicenseTermsDisplayed() {
    _persistentToolState.setLastActiveLicenseTermsHash(_currentHash);
  }

  /// The hash of the current license representation.
  String get _currentHash => hex.encode(md5.convert(utf8.encode(licenseTerms)).bytes);

  /// The current license terms.
  String get licenseTerms => _kFlutterFirstRunMessage;
}
