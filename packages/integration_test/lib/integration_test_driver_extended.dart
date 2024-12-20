// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is a CLI library; we use prints as part of the interface.
// ignore_for_file: avoid_print

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter_driver/flutter_driver.dart';
import 'package:path/path.dart' as path;

import 'common.dart';

export 'package:flutter_driver/flutter_driver.dart' show testOutputsDirectory;

/// The callback type to handle [Response.data] after the test
/// succeeds.
typedef ResponseDataCallback = FutureOr<void> Function(Map<String, dynamic>?);

/// Writes a json-serializable data to
/// [testOutputsDirectory]/`testOutputFilename.json`.
///
/// This is the default `responseDataCallback` in [integrationDriver].
Future<void> writeResponseData(
  Map<String, dynamic>? data, {
  String testOutputFilename = 'integration_response_data',
  String? destinationDirectory,
}) async {
  destinationDirectory ??= testOutputsDirectory;
  await fs.directory(destinationDirectory).create(recursive: true);
  final File file = fs.file(path.join(destinationDirectory, '$testOutputFilename.json'));
  final String resultString = _encodeJson(data, true);
  await file.writeAsString(resultString);
}

/// Adaptor to run an integration test using `flutter drive`.
///
/// To an integration test `<test_name>.dart` using `flutter drive`, put a file named
/// `<test_name>_test.dart` in the app's `test_driver` directory:
///
/// ```dart
/// import 'dart:async';
///
/// import 'package:flutter_driver/flutter_driver.dart';
/// import 'package:integration_test/integration_test_driver_extended.dart';
///
/// Future<void> main() async {
///   final FlutterDriver driver = await FlutterDriver.connect();
///   await integrationDriver(
///     driver: driver,
///     onScreenshot: (String name, List<int> image, [Map<String, Object?>? args]) async {
///       return true;
///     },
///   );
/// }
/// ```
///
/// ## Parameters:
///
/// `driver` A custom driver. Defaults to `FlutterDriver.connect()`.
///
/// `onScreenshot` can be used to process the screenshots taken during the test.
/// An example could be that this callback compares the byte array against a baseline image,
/// and it returns `true` if both images are equal.
///
/// As a result, returning `false` from `onScreenshot` will make the test fail.
///
/// `responseDataCallback` is the handler for processing [Response.data].
/// The default value is `writeResponseData`.
///
/// `writeResponseOnFailure` determines whether the `responseDataCallback`
/// function will be called to process the [Response.data] when a test fails.
/// The default value is `false`.
Future<void> integrationDriver({
  FlutterDriver? driver,
  ScreenshotCallback? onScreenshot,
  ResponseDataCallback? responseDataCallback = writeResponseData,
  bool writeResponseOnFailure = false,
}) async {
  driver ??= await FlutterDriver.connect();
  // Test states that it's waiting on web driver commands.
  // [DriverTestMessage] is converted to string since json format causes an
  // error if it's used as a message for requestData.
  String jsonResponse = await driver.requestData(DriverTestMessage.pending().toString());

  final Map<String, bool> onScreenshotResults = <String, bool>{};

  Response response = Response.fromJson(jsonResponse);

  // Until `integration_test` returns a [WebDriverCommandType.noop], keep
  // executing WebDriver commands.
  while (response.data != null &&
      response.data!['web_driver_command'] != null &&
      response.data!['web_driver_command'] != '${WebDriverCommandType.noop}') {
    final String? webDriverCommand = response.data!['web_driver_command'] as String?;
    if (webDriverCommand == '${WebDriverCommandType.screenshot}') {
      assert(onScreenshot != null, 'screenshot command requires an onScreenshot callback');
      // Use `driver.screenshot()` method to get a screenshot of the web page.
      final List<int> screenshotImage = await driver.screenshot();
      final String screenshotName = response.data!['screenshot_name']! as String;
      final Map<String, Object?>? args =
          (response.data!['args'] as Map<String, Object?>?)?.cast<String, Object?>();

      final bool screenshotSuccess = await onScreenshot!(screenshotName, screenshotImage, args);
      onScreenshotResults[screenshotName] = screenshotSuccess;
      if (screenshotSuccess) {
        jsonResponse = await driver.requestData(DriverTestMessage.complete().toString());
      } else {
        jsonResponse = await driver.requestData(DriverTestMessage.error().toString());
      }

      response = Response.fromJson(jsonResponse);
    } else if (webDriverCommand == '${WebDriverCommandType.ack}') {
      // Previous command completed ask for a new one.
      jsonResponse = await driver.requestData(DriverTestMessage.pending().toString());

      response = Response.fromJson(jsonResponse);
    } else {
      break;
    }
  }

  // If No-op command is sent, ask for the result of all tests.
  if (response.data != null &&
      response.data!['web_driver_command'] != null &&
      response.data!['web_driver_command'] == '${WebDriverCommandType.noop}') {
    jsonResponse = await driver.requestData(null);

    response = Response.fromJson(jsonResponse);
    print('result $jsonResponse');
  }

  if (response.data != null && response.data!['screenshots'] != null && onScreenshot != null) {
    final List<dynamic> screenshots = response.data!['screenshots'] as List<dynamic>;
    final List<String> failures = <String>[];
    for (final dynamic screenshot in screenshots) {
      final Map<String, dynamic> data = screenshot as Map<String, dynamic>;
      final List<dynamic> screenshotBytes = data['bytes'] as List<dynamic>;
      final String screenshotName = data['screenshotName'] as String;

      bool ok = false;
      try {
        ok =
            onScreenshotResults[screenshotName] ??
            await onScreenshot(screenshotName, screenshotBytes.cast<int>());
      } catch (exception) {
        throw StateError(
          'Screenshot failure:\n'
          'onScreenshot("$screenshotName", <bytes>) threw an exception: $exception',
        );
      }
      if (!ok) {
        failures.add(screenshotName);
      }
    }
    if (failures.isNotEmpty) {
      throw StateError('The following screenshot tests failed: ${failures.join(', ')}');
    }
  }

  await driver.close();

  if (response.allTestsPassed) {
    print('All tests passed.');
    if (responseDataCallback != null) {
      await responseDataCallback(response.data);
    }
    exit(0);
  } else {
    print('Failure Details:\n${response.formattedFailureDetails}');
    if (responseDataCallback != null && writeResponseOnFailure) {
      await responseDataCallback(response.data);
    }
    exit(1);
  }
}

const JsonEncoder _prettyEncoder = JsonEncoder.withIndent('  ');

String _encodeJson(Map<String, dynamic>? jsonObject, bool pretty) {
  return pretty ? _prettyEncoder.convert(jsonObject) : json.encode(jsonObject);
}
