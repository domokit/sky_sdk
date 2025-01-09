// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';
import 'package:file/file.dart';
import 'package:matcher/matcher.dart';
import 'package:meta/meta.dart';

import 'package:path/path.dart' as path;
import 'package:vm_service/vm_service.dart' as vms;
import 'package:webdriver/async_io.dart' as async_io;
import 'package:webdriver/support/async.dart';

import '../common/error.dart';
import '../common/message.dart';

import 'common.dart';
import 'driver.dart';
import 'timeline.dart';

/// An implementation of the Flutter Driver using the WebDriver.
///
/// Example of how to test WebFlutterDriver:
///   1. Launch WebDriver binary: ./chromedriver --port=4444
///   2. Run test script: flutter drive --target=test_driver/scroll_perf_web.dart -d web-server --release
class WebFlutterDriver extends FlutterDriver {
  /// Creates a driver that uses a connection provided by the given
  /// [_connection].
  WebFlutterDriver.connectedTo(
    this._connection, {
    bool printCommunication = false,
    bool logCommunicationToFile = true,
  }) : _printCommunication = printCommunication,
       _logCommunicationToFile = logCommunicationToFile,
       _startTime = DateTime.now(),
       _driverId = _nextDriverId++ {
    _logFilePathName = path.join(testOutputsDirectory, 'flutter_driver_commands_$_driverId.log');
  }

  final FlutterWebConnection _connection;
  DateTime _startTime;
  static int _nextDriverId = 0;

  /// The unique ID of this driver instance.
  final int _driverId;

  /// Start time for tracing.
  @visibleForTesting
  DateTime get startTime => _startTime;

  @override
  vms.Isolate get appIsolate =>
      throw UnsupportedError('WebFlutterDriver does not support appIsolate');

  @override
  vms.VmService get serviceClient =>
      throw UnsupportedError('WebFlutterDriver does not support serviceClient');

  @override
  async_io.WebDriver get webDriver => _connection._driver;

  /// Whether to print communication between host and app to `stdout`.
  final bool _printCommunication;

  /// Whether to log communication between host and app to `flutter_driver_commands.log`.
  final bool _logCommunicationToFile;

  /// Logs are written here when _logCommunicationToFile is true.
  late final String _logFilePathName;

  /// Getter for file pathname where logs are written when _logCommunicationToFile is true
  String get logFilePathName => _logFilePathName;

  /// Creates a driver that uses a connection provided by the given
  /// [hostUrl] which would fallback to environment variable VM_SERVICE_URL.
  /// Driver also depends on environment variables DRIVER_SESSION_ID,
  /// BROWSER_SUPPORTS_TIMELINE, DRIVER_SESSION_URI, DRIVER_SESSION_SPEC,
  /// DRIVER_SESSION_CAPABILITIES and ANDROID_CHROME_ON_EMULATOR for
  /// configurations.
  ///
  /// See [FlutterDriver.connect] for more documentation.
  static Future<FlutterDriver> connectWeb({
    String? hostUrl,
    bool printCommunication = false,
    bool logCommunicationToFile = true,
    Duration? timeout,
  }) async {
    hostUrl ??= Platform.environment['VM_SERVICE_URL'];
    final Map<String, dynamic> settings = <String, dynamic>{
      'support-timeline-action': Platform.environment['SUPPORT_TIMELINE_ACTION'] == 'true',
      'session-id': Platform.environment['DRIVER_SESSION_ID'],
      'session-uri': Platform.environment['DRIVER_SESSION_URI'],
      'session-spec': Platform.environment['DRIVER_SESSION_SPEC'],
      'android-chrome-on-emulator': Platform.environment['ANDROID_CHROME_ON_EMULATOR'] == 'true',
      'session-capabilities': Platform.environment['DRIVER_SESSION_CAPABILITIES'],
    };
    final FlutterWebConnection connection = await FlutterWebConnection.connect(
      hostUrl!,
      settings,
      timeout: timeout,
    );
    return WebFlutterDriver.connectedTo(
      connection,
      printCommunication: printCommunication,
      logCommunicationToFile: logCommunicationToFile,
    );
  }

  static DriverError _createMalformedExtensionResponseError(Object? data) {
    throw DriverError(
      'Received malformed response from the FlutterDriver extension.\n'
      'Expected a JSON map containing a "response" field and, optionally, an '
      '"isError" field, but got ${data.runtimeType}: $data',
    );
  }

  @override
  Future<Map<String, dynamic>> sendCommand(Command command) async {
    final Map<String, dynamic> response;
    final Object? data;
    final Map<String, String> serialized = command.serialize();
    _logCommunication('>>> $serialized');
    try {
      data = await _connection.sendCommand(
        "window.\$flutterDriver('${jsonEncode(serialized)}')",
        command.timeout,
      );

      // The returned data is expected to be a string. If it's null or anything
      // other than a string, something's wrong.
      if (data is! String) {
        throw _createMalformedExtensionResponseError(data);
      }

      final Object? decoded = json.decode(data);
      if (decoded is! Map<String, dynamic>) {
        throw _createMalformedExtensionResponseError(data);
      } else {
        response = decoded;
      }

      _logCommunication('<<< $response');
    } on DriverError catch (_) {
      rethrow;
    } catch (error, stackTrace) {
      throw DriverError(
        'FlutterDriver command ${command.runtimeType} failed due to a remote error.\n'
        'Command sent: ${jsonEncode(serialized)}',
        error,
        stackTrace,
      );
    }

    final Object? isError = response['isError'];
    final Object? responseData = response['response'];
    if (isError is! bool?) {
      throw _createMalformedExtensionResponseError(data);
    } else if (isError ?? false) {
      throw DriverError('Error in Flutter application: $responseData');
    }

    if (responseData is! Map<String, dynamic>) {
      throw _createMalformedExtensionResponseError(data);
    }
    return responseData;
  }

  @override
  Future<void> close() => _connection.close();

  @override
  Future<void> waitUntilFirstFrameRasterized() async {
    throw UnimplementedError();
  }

  void _logCommunication(String message) {
    if (_printCommunication) {
      driverLog('WebFlutterDriver', message);
    }
    if (_logCommunicationToFile) {
      final File file = fs.file(_logFilePathName);
      file.createSync(recursive: true); // no-op if file exists
      file.writeAsStringSync('${DateTime.now()} $message\n', mode: FileMode.append, flush: true);
    }
  }

  @override
  Future<List<int>> screenshot({ScreenshotFormat format = ScreenshotFormat.png}) async {
    if (format != ScreenshotFormat.png) {
      throw ArgumentError.value(format, 'format', 'Web Driver only supports PNG screenshot format');
    }
    await Future<void>.delayed(const Duration(seconds: 2));

    return _connection.screenshot();
  }

  @override
  Future<void> startTracing({
    List<TimelineStream> streams = const <TimelineStream>[TimelineStream.all],
    Duration timeout = kUnusuallyLongTimeout,
  }) async {
    _checkBrowserSupportsTimeline();
  }

  @override
  Future<Timeline> stopTracingAndDownloadTimeline({
    Duration timeout = kUnusuallyLongTimeout,
  }) async {
    _checkBrowserSupportsTimeline();

    final List<Map<String, dynamic>> events = <Map<String, dynamic>>[];
    for (final async_io.LogEntry entry in await _connection.logs.toList()) {
      if (_startTime.isBefore(entry.timestamp)) {
        final Map<String, dynamic> data =
            (jsonDecode(entry.message!) as Map<String, dynamic>)['message'] as Map<String, dynamic>;
        if (data['method'] == 'Tracing.dataCollected') {
          // 'ts' data collected from Chrome is in double format, conversion needed
          try {
            final Map<String, dynamic> params = data['params'] as Map<String, dynamic>;
            params['ts'] = double.parse(params['ts'].toString()).toInt();
          } on FormatException catch (_) {
            // data is corrupted, skip
            continue;
          }
          events.add(data['params']! as Map<String, dynamic>);
        }
      }
    }
    final Map<String, dynamic> json = <String, dynamic>{'traceEvents': events};
    return Timeline.fromJson(json);
  }

  @override
  Future<Timeline> traceAction(
    Future<dynamic> Function() action, {
    List<TimelineStream> streams = const <TimelineStream>[TimelineStream.all],
    bool retainPriorEvents = false,
  }) async {
    _checkBrowserSupportsTimeline();
    if (!retainPriorEvents) {
      await clearTimeline();
    }
    await startTracing(streams: streams);
    await action();

    return stopTracingAndDownloadTimeline();
  }

  @override
  Future<void> clearTimeline({Duration timeout = kUnusuallyLongTimeout}) async {
    _checkBrowserSupportsTimeline();

    // Reset start time
    _startTime = DateTime.now();
  }

  /// Checks whether browser supports Timeline related operations.
  void _checkBrowserSupportsTimeline() {
    if (!_connection.supportsTimelineAction) {
      throw UnsupportedError('Timeline action is not supported by current testing browser');
    }
  }
}

/// Encapsulates connection information to an instance of a Flutter Web application.
class FlutterWebConnection {
  /// Creates a FlutterWebConnection with WebDriver
  /// and whether the WebDriver supports timeline action.
  FlutterWebConnection(this._driver, this.supportsTimelineAction);

  final async_io.WebDriver _driver;

  /// Whether the connected WebDriver supports timeline action for Flutter Web Driver.
  bool supportsTimelineAction;

  /// Starts WebDriver with the given [settings] and
  /// establishes the connection to Flutter Web application.
  static Future<FlutterWebConnection> connect(
    String url,
    Map<String, dynamic> settings, {
    Duration? timeout,
  }) async {
    final String sessionId = settings['session-id'].toString();
    final Uri sessionUri = Uri.parse(settings['session-uri'].toString());
    final async_io.WebDriver driver = async_io.WebDriver(
      sessionUri,
      sessionId,
      json.decode(settings['session-capabilities'] as String) as Map<String, dynamic>,
      async_io.AsyncIoRequestClient(sessionUri.resolve('session/$sessionId/')),
      async_io.WebDriverSpec.W3c,
    );
    if (settings['android-chrome-on-emulator'] == true) {
      final Uri localUri = Uri.parse(url);
      // Converts to Android Emulator Uri.
      // Hardcode the host to 10.0.2.2 based on
      // https://developer.android.com/studio/run/emulator-networking
      url = Uri(scheme: localUri.scheme, host: '10.0.2.2', port: localUri.port).toString();
    }
    await driver.get(url);

    await waitUntilExtensionInstalled(driver, timeout);
    return FlutterWebConnection(driver, settings['support-timeline-action'] as bool);
  }

  /// Sends command via WebDriver to Flutter web application.
  Future<dynamic> sendCommand(String script, Duration? duration) async {
    // This code should not be reachable before the VM service extension is
    // initialized. The VM service extension is expected to initialize both
    // `$flutterDriverResult` and `$flutterDriver` variables before attempting
    // to send commands. This part checks that `$flutterDriverResult` is present.
    // `$flutterDriver` is not checked because it is covered by the `script`
    // that's executed next.
    try {
      await _driver.execute(r'return $flutterDriverResult', <String>[]);
    } catch (error, stackTrace) {
      throw DriverError(
        'Driver extension has not been initialized correctly.\n'
        'If the test uses a custom VM service extension, make sure it conforms '
        'to the protocol used by package:integration_test and '
        'package:flutter_driver.\n'
        'If the test uses VM service extensions provided by the Flutter SDK, '
        'then this error is likely caused by a bug in Flutter. Please report it '
        'by filing a bug on GitHub:\n'
        '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
        error,
        stackTrace,
      );
    }

    String phase = 'executing';
    try {
      // Execute the script, which should leave the result in the `$flutterDriverResult` global variable.
      await _driver.execute(script, <void>[]);

      // Read the result.
      phase = 'reading';
      final dynamic result = await waitFor<dynamic>(
        () => _driver.execute(r'return $flutterDriverResult', <String>[]),
        matcher: isNotNull,
        timeout: duration ?? const Duration(days: 30),
      );

      // Reset the result to null to avoid polluting the results of future commands.
      phase = 'resetting';
      await _driver.execute(r'$flutterDriverResult = null', <void>[]);
      return result;
    } catch (error, stackTrace) {
      throw DriverError(
        'Error while $phase FlutterDriver result for command: $script',
        error,
        stackTrace,
      );
    }
  }

  /// Gets performance log from WebDriver.
  Stream<async_io.LogEntry> get logs => _driver.logs.get(async_io.LogType.performance);

  /// Takes screenshot via WebDriver.
  Future<List<int>> screenshot() => _driver.captureScreenshotAsList();

  /// Closes the WebDriver.
  Future<void> close() async {
    await _driver.quit(closeSession: false);
  }
}

/// Waits until extension is installed.
Future<void> waitUntilExtensionInstalled(async_io.WebDriver driver, Duration? timeout) async {
  await waitFor<void>(
    () => driver.execute(r'return typeof(window.$flutterDriver)', <String>[]),
    matcher: 'function',
    timeout: timeout ?? const Duration(days: 365),
  );
}
