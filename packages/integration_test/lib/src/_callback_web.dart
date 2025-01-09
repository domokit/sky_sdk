// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import '../common.dart';

/// The web implementation of [CallbackManager].
///
/// See also:
///
///  * `_callback_io.dart`, which has the dart:io implementation
CallbackManager get callbackManager => _singletonWebDriverCommandManager;

/// WebDriverCommandManager singleton.
final _WebCallbackManager _singletonWebDriverCommandManager = _WebCallbackManager();

/// Manages communication between `integration_tests` and the `driver_tests`.
///
/// Along with responding to callbacks from the driver side this calls enables
/// usage of Web Driver commands by sending [WebDriverCommand]s to driver side.
///
/// Tests can execute an Web Driver commands such as `screenshot` using browsers'
/// WebDriver APIs.
///
/// See: https://www.w3.org/TR/webdriver/
class _WebCallbackManager implements CallbackManager {
  /// App side tests will put the command requests from WebDriver to this pipe.
  Completer<WebDriverCommand> _webDriverCommandPipe = Completer<WebDriverCommand>();

  /// Updated when WebDriver completes the request by the test method.
  ///
  /// For example, a test method will ask for a screenshot by calling
  /// `takeScreenshot`. When this screenshot is taken [_driverCommandComplete]
  /// will complete.
  Completer<bool> _driverCommandComplete = Completer<bool>();

  /// Takes screenshot using WebDriver screenshot command.
  ///
  /// Only works on Web when tests are run via `flutter driver` command.
  ///
  /// See: https://www.w3.org/TR/webdriver/#screen-capture.
  @override
  Future<Map<String, dynamic>> takeScreenshot(
    String screenshotName, [
    Map<String, Object?>? args,
  ]) async {
    await _sendWebDriverCommand(WebDriverCommand.screenshot(screenshotName, args));
    return <String, dynamic>{
      'screenshotName': screenshotName,
      // Flutter Web doesn't provide the bytes.
      'bytes': <int>[],
    };
  }

  @override
  Future<void> convertFlutterSurfaceToImage() async {
    // Noop on Web.
  }

  Future<void> _sendWebDriverCommand(WebDriverCommand command) async {
    try {
      _webDriverCommandPipe.complete(command);
      final bool awaitCommand = await _driverCommandComplete.future;
      if (!awaitCommand) {
        throw Exception(
          'Web Driver Command ${command.type} failed while waiting for '
          'driver side',
        );
      }
    } catch (exception) {
      throw Exception('Web Driver Command failed: ${command.type} with exception $exception');
    } finally {
      // Reset the completer.
      _driverCommandComplete = Completer<bool>();
    }
  }

  /// The callback function to response the driver side input.
  ///
  /// Provides a handshake mechanism for executing [WebDriverCommand]s on the
  /// driver side.
  @override
  Future<Map<String, dynamic>> callback(
    Map<String, String> params,
    IntegrationTestResults testRunner,
  ) async {
    final String command = params['command']!;
    Map<String, String> response;
    switch (command) {
      case 'request_data':
        return params['message'] == null
            ? _requestData(testRunner)
            : _requestDataWithMessage(params['message']!, testRunner);
      case 'get_health':
        response = <String, String>{'status': 'ok'};
      default:
        throw UnimplementedError('$command is not implemented');
    }
    return <String, dynamic>{'isError': false, 'response': response};
  }

  Future<Map<String, dynamic>> _requestDataWithMessage(
    String extraMessage,
    IntegrationTestResults testRunner,
  ) async {
    Map<String, String> response;
    // Driver side tests' status is added as an extra message.
    final DriverTestMessage message = DriverTestMessage.fromString(extraMessage);
    // If driver side tests are pending send the first command in the
    // `commandPipe` to the tests.
    if (message.isPending) {
      final WebDriverCommand command = await _webDriverCommandPipe.future;
      switch (command.type) {
        case WebDriverCommandType.screenshot:
          final Map<String, dynamic> data = Map<String, dynamic>.from(command.values);
          data.addAll(WebDriverCommand.typeToMap(WebDriverCommandType.screenshot));
          response = <String, String>{'message': Response.webDriverCommand(data: data).toJson()};
        case WebDriverCommandType.noop:
          final Map<String, dynamic> data = <String, dynamic>{};
          data.addAll(WebDriverCommand.typeToMap(WebDriverCommandType.noop));
          response = <String, String>{'message': Response.webDriverCommand(data: data).toJson()};
        case WebDriverCommandType.ack:
          throw UnimplementedError('${command.type} is not implemented');
      }
    } else {
      final Map<String, dynamic> data = <String, dynamic>{};
      data.addAll(WebDriverCommand.typeToMap(WebDriverCommandType.ack));
      response = <String, String>{'message': Response.webDriverCommand(data: data).toJson()};
      _driverCommandComplete.complete(message.isSuccess);
      _webDriverCommandPipe = Completer<WebDriverCommand>();
    }
    return <String, dynamic>{'isError': false, 'response': response};
  }

  Future<Map<String, dynamic>> _requestData(IntegrationTestResults testRunner) async {
    final bool allTestsPassed = await testRunner.allTestsPassed.future;
    final Map<String, String> response = <String, String>{
      'message':
          allTestsPassed
              ? Response.allTestsPassed(data: testRunner.reportData).toJson()
              : Response.someTestsFailed(
                testRunner.failureMethodsDetails,
                data: testRunner.reportData,
              ).toJson(),
    };
    return <String, dynamic>{'isError': false, 'response': response};
  }

  @override
  void cleanup() {
    if (!_webDriverCommandPipe.isCompleted) {
      _webDriverCommandPipe.complete(Future<WebDriverCommand>.value(WebDriverCommand.noop()));
    }

    if (!_driverCommandComplete.isCompleted) {
      _driverCommandComplete.complete(Future<bool>.value(false));
    }
  }
}
