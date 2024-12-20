// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:isolate';
import 'dart:ui';

import 'package:test/test.dart';

const int kErrorCode = -1;
const int kStartCode = 0;
const int kCloseCode = 1;
const int kDeletedCode = 2;

class IsolateSpawnInfo {
  IsolateSpawnInfo(this.sendPort, this.portName);

  final SendPort sendPort;
  final String portName;
}

void isolateSpawnEntrypoint(IsolateSpawnInfo info) {
  final SendPort port = info.sendPort;
  final String portName = info.portName;
  void sendHelper(int code, [String message = '']) {
    port.send(<dynamic>[code, message]);
  }

  final SendPort? shared = IsolateNameServer.lookupPortByName(portName);
  if (shared == null) {
    sendHelper(kErrorCode, 'Could not find port: $portName');
    return;
  }

  // ack that the SendPort lookup was successful.
  sendHelper(kStartCode);

  shared.send(portName);
  sendHelper(kCloseCode);

  // We'll fail if the ReceivePort's callback is called more than once. Try to
  // send another message to ensure we don't crash.
  shared.send('garbage');

  final bool result = IsolateNameServer.removePortNameMapping(portName);
  if (result) {
    sendHelper(kDeletedCode);
  } else {
    sendHelper(kErrorCode, 'Was unable to remove mapping for $portName');
  }
}

void main() {
  test('simple isolate name server', () {
    const String portName = 'foobar1';
    try {
      // Mapping for 'foobar' isn't set. Check these cases to ensure correct
      // negative response.
      expect(IsolateNameServer.lookupPortByName(portName), isNull);
      expect(IsolateNameServer.removePortNameMapping(portName), isFalse);

      // Register a SendPort.
      final ReceivePort receivePort = ReceivePort();
      final SendPort sendPort = receivePort.sendPort;
      expect(IsolateNameServer.registerPortWithName(sendPort, portName), isTrue);
      expect(IsolateNameServer.lookupPortByName(portName), sendPort);

      // Check we can't register the same name twice.
      final ReceivePort receivePort2 = ReceivePort();
      final SendPort sendPort2 = receivePort2.sendPort;
      expect(IsolateNameServer.registerPortWithName(sendPort2, portName), isFalse);
      expect(IsolateNameServer.lookupPortByName(portName), sendPort);

      // Remove the mapping.
      expect(IsolateNameServer.removePortNameMapping(portName), isTrue);
      expect(IsolateNameServer.lookupPortByName(portName), isNull);

      // Ensure registering a new port with the old name returns the new port.
      expect(IsolateNameServer.registerPortWithName(sendPort2, portName), isTrue);
      expect(IsolateNameServer.lookupPortByName(portName), sendPort2);

      // Close so the test runner doesn't hang.
      receivePort.close();
      receivePort2.close();
    } finally {
      IsolateNameServer.removePortNameMapping(portName);
    }
  });

  test('isolate name server multi-isolate', () async {
    const String portName = 'foobar2';
    try {
      // Register our send port with the name server.
      final ReceivePort receivePort = ReceivePort();
      final SendPort sendPort = receivePort.sendPort;
      expect(IsolateNameServer.registerPortWithName(sendPort, portName), isTrue);

      // Test driver.
      final ReceivePort testReceivePort = ReceivePort();
      final Completer<void> testPortCompleter = Completer<void>();
      testReceivePort.listen(
        expectAsync1<void, dynamic>((dynamic response) {
          final List<dynamic> typedResponse = response as List<dynamic>;
          final int code = typedResponse[0] as int;
          final String message = typedResponse[1] as String;
          switch (code) {
            case kStartCode:
              break;
            case kCloseCode:
              receivePort.close();
            case kDeletedCode:
              expect(IsolateNameServer.lookupPortByName(portName), isNull);
              // Test is done, close the last ReceivePort.
              testReceivePort.close();
            case kErrorCode:
              throw message;
            default:
              throw 'UNREACHABLE';
          }
        }, count: 3),
        onDone: testPortCompleter.complete,
      );

      final Completer<void> portCompleter = Completer<void>();
      receivePort.listen(
        expectAsync1<void, dynamic>((dynamic message) {
          // If we don't get this message, we timeout and fail.
          expect(message, portName);
        }),
        onDone: portCompleter.complete,
      );

      // Run the test.
      await Isolate.spawn(
        isolateSpawnEntrypoint,
        IsolateSpawnInfo(testReceivePort.sendPort, portName),
      );
      await testPortCompleter.future;
      await portCompleter.future;
    } finally {
      IsolateNameServer.removePortNameMapping(portName);
    }
  });
}
