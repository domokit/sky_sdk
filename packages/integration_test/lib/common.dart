// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter_driver/flutter_driver.dart';
///
/// @docImport 'integration_test.dart';
/// @docImport 'integration_test_driver_extended.dart';
library;

import 'dart:async';
import 'dart:convert';

/// A callback to use with [integrationDriver].
///
/// The callback receives the name of screenshot passed to `binding.takeScreenshot(<name>)`,
/// a PNG byte buffer representing the screenshot, and an optional `Map` of arguments.
///
/// The callback returns `true` if the test passes or `false` otherwise.
///
/// You can use this callback to store the bytes locally in a file or upload them to a service
/// that compares the image against a gold or baseline version.
///
/// The optional `Map` of arguments can be passed from the
/// `binding.takeScreenshot(<name>, <args>)` callsite in the integration test,
/// and then the arguments can be used in the `onScreenshot` handler that is defined by
/// the Flutter driver. This `Map` should only contain values that can be serialized
/// to JSON.
///
/// Since the function is executed on the host driving the test, you can access any environment
/// variable from it.
typedef ScreenshotCallback =
    Future<bool> Function(String name, List<int> image, [Map<String, Object?>? args]);

/// Classes shared between `integration_test.dart` and `flutter drive` based
/// adaptor (ex: `integration_test_driver.dart`).

/// An object sent from integration_test back to the Flutter Driver in response to
/// `request_data` command.
class Response {
  /// Constructor to use for positive response.
  Response.allTestsPassed({this.data}) : _allTestsPassed = true, _failureDetails = null;

  /// Constructor for failure response.
  Response.someTestsFailed(this._failureDetails, {this.data}) : _allTestsPassed = false;

  /// Constructor for failure response.
  Response.toolException({String? ex})
    : _allTestsPassed = false,
      _failureDetails = <Failure>[Failure('ToolException', ex)];

  /// Constructor for web driver commands response.
  Response.webDriverCommand({this.data}) : _allTestsPassed = false, _failureDetails = null;

  final List<Failure>? _failureDetails;

  final bool _allTestsPassed;

  /// The extra information to be added along side the test result.
  Map<String, dynamic>? data;

  /// Whether the test ran successfully or not.
  bool get allTestsPassed => _allTestsPassed;

  /// If the result are failures get the formatted details.
  String get formattedFailureDetails => _allTestsPassed ? '' : formatFailures(_failureDetails!);

  /// Failure details as a list.
  List<Failure>? get failureDetails => _failureDetails;

  /// Serializes this message to a JSON map.
  String toJson() => json.encode(<String, dynamic>{
    'result': allTestsPassed.toString(),
    'failureDetails': _failureDetailsAsString(),
    if (data != null) 'data': data,
  });

  /// Deserializes the result from JSON.
  static Response fromJson(String source) {
    final Map<String, dynamic> responseJson = json.decode(source) as Map<String, dynamic>;
    if ((responseJson['result'] as String?) == 'true') {
      return Response.allTestsPassed(data: responseJson['data'] as Map<String, dynamic>?);
    } else {
      return Response.someTestsFailed(
        _failureDetailsFromJson(responseJson['failureDetails'] as List<dynamic>),
        data: responseJson['data'] as Map<String, dynamic>?,
      );
    }
  }

  /// Method for formatting the test failures' details.
  String formatFailures(List<Failure> failureDetails) {
    if (failureDetails.isEmpty) {
      return '';
    }

    final StringBuffer sb = StringBuffer();
    int failureCount = 1;
    for (final Failure failure in failureDetails) {
      sb.writeln('Failure in method: ${failure.methodName}');
      sb.writeln(failure.details);
      sb.writeln('end of failure $failureCount\n\n');
      failureCount++;
    }
    return sb.toString();
  }

  /// Create a list of Strings from [_failureDetails].
  List<String> _failureDetailsAsString() {
    return <String>[
      if (_failureDetails != null)
        for (final Failure failure in _failureDetails) failure.toJson(),
    ];
  }

  /// Creates a [Failure] list using a json response.
  static List<Failure> _failureDetailsFromJson(List<dynamic> list) {
    return list.map((dynamic s) {
      return Failure.fromJsonString(s as String);
    }).toList();
  }
}

/// Representing a failure includes the method name and the failure details.
class Failure {
  /// Constructor requiring all fields during initialization.
  Failure(this.methodName, this.details);

  /// The name of the test method which failed.
  final String methodName;

  /// The details of the failure such as stack trace.
  final String? details;

  /// Serializes the object to JSON.
  String toJson() {
    return json.encode(<String, String?>{'methodName': methodName, 'details': details});
  }

  @override
  String toString() => toJson();

  /// Decode a JSON string to create a Failure object.
  static Failure fromJsonString(String jsonString) {
    final Map<String, dynamic> failure = json.decode(jsonString) as Map<String, dynamic>;
    return Failure(failure['methodName'] as String, failure['details'] as String?);
  }
}

/// Message used to communicate between app side tests and driver tests.
///
/// Not all `integration_tests` use this message. They are only used when app
/// side tests are sending [WebDriverCommand]s to the driver side.
///
/// These messages are used for the handshake since they carry information on
/// the driver side test such as: status pending or tests failed.
class DriverTestMessage {
  /// When tests are failed on the driver side.
  DriverTestMessage.error() : _isSuccess = false, _isPending = false;

  /// When driver side is waiting on [WebDriverCommand]s to be sent from the
  /// app side.
  DriverTestMessage.pending() : _isSuccess = false, _isPending = true;

  /// When driver side successfully completed executing the [WebDriverCommand].
  DriverTestMessage.complete() : _isSuccess = true, _isPending = false;

  final bool _isSuccess;
  final bool _isPending;

  // /// Status of this message.
  // ///
  // /// The status will be use to notify `integration_test` of driver side's
  // /// state.
  // String get status => _status;

  /// Has the command completed successfully by the driver.
  bool get isSuccess => _isSuccess;

  /// Is the driver waiting for a command.
  bool get isPending => _isPending;

  /// Depending on the values of [isPending] and [isSuccess], returns a string
  /// to represent the [DriverTestMessage].
  ///
  /// Used as an alternative method to converting the object to json since
  /// [RequestData] is only accepting string as `message`.
  @override
  String toString() {
    if (isPending) {
      return 'pending';
    } else if (isSuccess) {
      return 'complete';
    } else {
      return 'error';
    }
  }

  /// Return a DriverTestMessage depending on `status`.
  static DriverTestMessage fromString(String status) {
    return switch (status) {
      'error' => DriverTestMessage.error(),
      'pending' => DriverTestMessage.pending(),
      'complete' => DriverTestMessage.complete(),
      _ => throw StateError('This type of status does not exist: $status'),
    };
  }
}

/// Types of different WebDriver commands that can be used in web integration
/// tests.
///
/// These commands are either commands that WebDriver can execute or used
/// for the communication between `integration_test` and the driver test.
enum WebDriverCommandType {
  /// Acknowledgement for the previously sent message.
  ack,

  /// No further WebDriver commands is requested by the app-side tests.
  noop,

  /// Asking WebDriver to take a screenshot of the Web page.
  screenshot,
}

/// Command for WebDriver to execute.
///
/// Only works on Web when tests are run via `flutter driver` command.
///
/// See: https://www.w3.org/TR/webdriver/
class WebDriverCommand {
  /// Constructor for [WebDriverCommandType.noop] command.
  WebDriverCommand.noop() : type = WebDriverCommandType.noop, values = <String, dynamic>{};

  /// Constructor for [WebDriverCommandType.noop] screenshot.
  WebDriverCommand.screenshot(String screenshotName, [Map<String, Object?>? args])
    : type = WebDriverCommandType.screenshot,
      values = <String, dynamic>{'screenshot_name': screenshotName, if (args != null) 'args': args};

  /// Type of the [WebDriverCommand].
  ///
  /// Currently the only command that triggers a WebDriver API is `screenshot`.
  ///
  /// There are also `ack` and `noop` commands defined to manage the handshake
  /// during the communication.
  final WebDriverCommandType type;

  /// Used for adding extra values to the commands such as file name for
  /// `screenshot`.
  final Map<String, dynamic> values;

  /// Util method for converting [WebDriverCommandType] to a map entry.
  ///
  /// Used for converting messages to json format.
  static Map<String, dynamic> typeToMap(WebDriverCommandType type) => <String, dynamic>{
    'web_driver_command': '$type',
  };
}

/// Template methods each class that responses the driver side inputs must
/// implement.
///
/// Depending on the platform the communication between `integration_tests` and
/// the `driver_tests` can be different.
abstract class CallbackManager {
  /// The callback function to response the driver side input.
  Future<Map<String, dynamic>> callback(
    Map<String, String> params,
    IntegrationTestResults testRunner,
  );

  /// Takes a screenshot of the application.
  /// Returns the data that is sent back to the host.
  Future<Map<String, dynamic>> takeScreenshot(String screenshot, [Map<String, Object?>? args]);

  /// Android only. Converts the Flutter surface to an image view.
  Future<void> convertFlutterSurfaceToImage();

  /// Cleanup and completers or locks used during the communication.
  void cleanup();
}

/// Interface that surfaces test results of integration tests.
///
/// Implemented by [IntegrationTestWidgetsFlutterBinding]s.
///
/// Any class which needs to access the test results but do not want to create
/// a cyclic dependency [IntegrationTestWidgetsFlutterBinding]s can use this
/// interface. Example [CallbackManager].
abstract class IntegrationTestResults {
  /// Stores failure details.
  ///
  /// Failed test method's names used as key.
  List<Failure> get failureMethodsDetails;

  /// The extra data for the reported result.
  Map<String, dynamic>? get reportData;

  /// Whether all the test methods completed successfully.
  ///
  /// Completes when the tests have finished. The boolean value will be true if
  /// all tests have passed, and false otherwise.
  Completer<bool> get allTestsPassed;
}
