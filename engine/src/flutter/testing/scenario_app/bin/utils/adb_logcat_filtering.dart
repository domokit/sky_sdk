// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Some notes about filtering `adb logcat` output, especially as a result of
/// running `adb shell` to instrument the app and test scripts, as it's
/// non-trivial and error-prone.
///
/// 1. It's probably worth keeping `ActivityManager` lines unconditionally.
///    They are the most important ones, and they are not too verbose (for
///    example, they don't typically contain stack traces).
///
/// 2. `ActivityManager` starts with the application name and process ID:
///
/// ```txt
/// [stdout] 02-15 10:20:36.914  1735  1752 I ActivityManager: Start proc 6840:dev.flutter.scenarios/u0a98 for added application dev.flutter.scenarios
/// ```
///
/// The "application" comes from the file `android/app/build.gradle` under
/// `android > defaultConfig > applicationId`.
///
/// 3. Once we have the process ID, we can filter the logcat output further:
///
/// ```txt
/// [stdout] 02-15 10:20:37.430  6840  6840 E GeneratedPluginsRegister: Tried to automatically register plugins with FlutterEngine (io.flutter.embedding.engine.FlutterEngine@144d737) but could not find or invoke the GeneratedPluginRegistrant.
/// ```
///
/// A sample output of `adb logcat` command lives in `./sample_adb_logcat.txt`.
///
/// See also: <https://developer.android.com/tools/logcat>.
library;

import 'package:meta/meta.dart';

import 'logs.dart';

/// Represents a line of `adb logcat` output parsed into a structured form.
///
/// For example the line:
/// ```txt
/// 02-22 13:54:39.839   549  3683 I ActivityManager: Force stopping dev.flutter.scenarios appid=10226 user=0: start instr
/// ```
///
/// ## Implementation notes
///
/// The reason this is an extension type and not a class is partially to use the
/// language feature, and partially because extension types work really well
/// with lazy parsing.
extension type const AdbLogLine._(Match _match) {
  // RegEx that parses into the following groups:
  // 1. The time of the log message, such as `02-22 13:54:39.839`.
  // 2. The process ID.
  // 3. The thread ID.
  // 4. The character representing the severity of the log message, such as `I`.
  // 5. The tag, such as `ActivityManager`.
  // 6. The actual log message.
  //
  // This regex is simple versus being more precise. Feel free to improve it.
  static final RegExp _pattern = RegExp(
    r'(\d+-\d+\s[\d|:]+\.\d+)\s+(\d+)\s+(\d+)\s(\w)\s(\S+)\s*:\s*(.*)',
  );

  /// Parses the given [adbLogCatLine] into a structured form.
  ///
  /// Returns `null` if the line does not match the expected format.
  static AdbLogLine? tryParse(String adbLogCatLine) {
    final Match? match = _pattern.firstMatch(adbLogCatLine);
    return match == null ? null : AdbLogLine._(match);
  }

  /// Tries to parse the process that was started, if the log line is about it.
  String? tryParseProcess() {
    if (name == activityManagerTag && message.startsWith('Start proc')) {
      // ActivityManager: Start proc 4475:dev.flutter.scenarios/u0a190 for added application ...
      final RegExpMatch? match = RegExp(
        'Start proc (\\d+):$flutterProcessName',
      ).firstMatch(message);
      return match?.group(1);
    }
    return null;
  }

  @visibleForTesting
  static const String activityManagerTag = 'ActivityManager';

  @visibleForTesting
  static const String flutterProcessName = 'dev.flutter.scenarios';

  @visibleForTesting
  static const Set<String> knownNoiseTags = <String>{
    'CCodec',
    'CCodecBufferChannel',
    'CCodecConfig',
    'Codec2Client',
    'ColorUtils',
    'DMABUFHEAPS',
    'Gralloc4',
    'MediaCodec',
    'MonitoringInstr',
    'ResourceExtractor',
    'UsageTrackerFacilitator',
    'hw-BpHwBinder',
    'ziparchive',
  };

  @visibleForTesting
  static const Set<String> knownUsefulTags = <String>{activityManagerTag};

  @visibleForTesting
  static const Set<String> knownUsefulErrorTags = <String>{'androidemu', 'THREAD_STATE'};

  /// Returns `true` if the log line is verbose.
  bool isVerbose({String? filterProcessId}) => !_isRelevant(filterProcessId: filterProcessId);
  bool _isRelevant({String? filterProcessId}) {
    // Fatal errors are always useful.
    if (severity == 'F') {
      return true;
    }

    // Verbose and debug logs are rarely useful.
    if (severity == 'V' || severity == 'D') {
      return false;
    }

    if (knownNoiseTags.contains(name)) {
      return false;
    }

    if (knownUsefulTags.contains(name)) {
      return true;
    }

    if (severity == 'E' && knownUsefulErrorTags.contains(name)) {
      return true;
    }

    // If a process ID is specified, exclude logs _not_ from that process.
    if (filterProcessId == null) {
      // YOLO, let's keep it anyway.
      return name.toLowerCase().contains('flutter') || message.toLowerCase().contains('flutter');
    }

    return process == filterProcessId;
  }

  /// Logs the line to the console.
  void printFormatted() {
    final String formatted = '$time [$severity] $name: $message';
    if (severity == 'W' || severity == 'E' || severity == 'F') {
      logWarning(formatted);
    } else if (name == 'TestRunner') {
      logImportant(formatted);
    } else {
      log(formatted);
    }
  }

  /// The full line of `adb logcat` output.
  String get line => _match.group(0)!;

  /// The time of the log message, such as `02-22 13:54:39.839`.
  String get time => _match.group(1)!;

  /// The process ID.
  String get process => _match.group(2)!;

  /// The thread ID.
  String get thread => _match.group(3)!;

  /// The character representing the severity of the log message, such as `I`.
  String get severity => _match.group(4)!;

  /// The tag, such as `ActivityManager`.
  String get name => _match.group(5)!;

  /// The actual log message.
  String get message => _match.group(6)!;

  String toDebugString() {
    return 'AdbLogLine(time: $time, process: $process, thread: $thread, severity: $severity, name: $name, message: $message)';
  }
}
