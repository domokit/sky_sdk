// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/src/common/error.dart';
import 'package:flutter_driver/src/common/health.dart';
import 'package:flutter_driver/src/driver/web_driver.dart';
import 'package:webdriver/async_io.dart';

import '../../common.dart';

void main() {
  group('WebDriver', () {
    late FakeFlutterWebConnection fakeConnection;
    late WebFlutterDriver driver;

    setUp(() {
      fakeConnection = FakeFlutterWebConnection();
      driver = WebFlutterDriver.connectedTo(fakeConnection);
    });

    test('sendCommand succeeds', () async {
      fakeConnection.fakeResponse = '''
{
  "isError": false,
  "response": {
    "test": "hello"
  }
}
''';

      final Map<String, Object?> response = await driver.sendCommand(const GetHealth());
      expect(response['test'], 'hello');
    });

    test('sendCommand fails on communication error', () async {
      fakeConnection.communicationError = Error();
      expect(
        () => driver.sendCommand(const GetHealth()),
        _throwsDriverErrorWithMessage(
          'FlutterDriver command GetHealth failed due to a remote error.\n'
          'Command sent: {"command":"get_health"}',
        ),
      );
    });

    test('sendCommand fails on null', () async {
      fakeConnection.fakeResponse = null;
      expect(
        () => driver.sendCommand(const GetHealth()),
        _throwsDriverErrorWithDataString('Null', 'null'),
      );
    });

    test('sendCommand fails when response data is not a string', () async {
      fakeConnection.fakeResponse = 1234;
      expect(
        () => driver.sendCommand(const GetHealth()),
        _throwsDriverErrorWithDataString('int', '1234'),
      );
    });

    test('sendCommand fails when isError is true', () async {
      fakeConnection.fakeResponse = '''
{
  "isError": true,
  "response": "test error message"
}
''';
      expect(
        () => driver.sendCommand(const GetHealth()),
        _throwsDriverErrorWithMessage('Error in Flutter application: test error message'),
      );
    });

    test('sendCommand fails when isError is not bool', () async {
      fakeConnection.fakeResponse = '{ "isError": 5 }';
      expect(
        () => driver.sendCommand(const GetHealth()),
        _throwsDriverErrorWithDataString('String', '{ "isError": 5 }'),
      );
    });

    test('sendCommand fails when "response" field is not a JSON map', () async {
      fakeConnection.fakeResponse = '{ "response": 5 }';
      expect(
        () => driver.sendCommand(const GetHealth()),
        _throwsDriverErrorWithDataString('String', '{ "response": 5 }'),
      );
    });
  });
}

Matcher _throwsDriverErrorWithMessage(String expectedMessage) {
  return throwsA(
    allOf(
      isA<DriverError>(),
      predicate<DriverError>((DriverError error) {
        final String actualMessage = error.message;
        return actualMessage == expectedMessage;
      }, 'contains message: $expectedMessage'),
    ),
  );
}

Matcher _throwsDriverErrorWithDataString(String dataType, String dataString) {
  return _throwsDriverErrorWithMessage(
    'Received malformed response from the FlutterDriver extension.\n'
    'Expected a JSON map containing a "response" field and, optionally, an '
    '"isError" field, but got $dataType: $dataString',
  );
}

class FakeFlutterWebConnection implements FlutterWebConnection {
  @override
  bool supportsTimelineAction = false;

  @override
  Future<void> close() async {}

  @override
  Stream<LogEntry> get logs => throw UnimplementedError();

  @override
  Future<List<int>> screenshot() {
    throw UnimplementedError();
  }

  Object? fakeResponse;
  Error? communicationError;

  @override
  Future<Object?> sendCommand(String script, Duration? duration) async {
    if (communicationError != null) {
      throw communicationError!;
    }
    return fakeResponse;
  }
}
