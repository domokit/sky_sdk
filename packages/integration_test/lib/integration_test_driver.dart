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
/// To run an integration test `<test_name>.dart` using `flutter drive`, put a file named
/// `<test_name>_test.dart` in the app's `test_driver` directory:
///
/// ```dart
/// import 'dart:async';
///
/// import 'package:integration_test/integration_test_driver.dart';
///
/// Future<void> main() async => integrationDriver();
///
/// ```
///
/// ## Parameters:
///
/// `timeout` controls the longest time waited before the test ends.
/// It is not necessarily the execution time for the test app: the test may
/// finish sooner than the `timeout`.
///
/// `responseDataCallback` is the handler for processing [Response.data].
/// The default value is `writeResponseData`.
///
/// `writeResponseOnFailure` determines whether the `responseDataCallback`
/// function will be called to process the [Response.data] when a test fails.
/// The default value is `false`.
Future<void> integrationDriver({
  Duration timeout = const Duration(minutes: 20),
  ResponseDataCallback? responseDataCallback = writeResponseData,
  bool writeResponseOnFailure = false,
}) async {
  final FlutterDriver driver = await FlutterDriver.connect();
  final String jsonResult = await driver.requestData(null, timeout: timeout);
  final Response response = Response.fromJson(jsonResult);

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
