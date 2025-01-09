// Copyright 2024 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:isolate';
import 'dart:ui';

import 'package:test/test.dart';

int counter = 0;

void main() {
  test('PlatformIsolate isRunningOnPlatformThread, false cases', () async {
    final bool isPlatThread = await Isolate.run(() => isRunningOnPlatformThread);
    expect(isPlatThread, isFalse);
  });

  test('PlatformIsolate runOnPlatformThread', () async {
    final bool isPlatThread = await runOnPlatformThread(() => isRunningOnPlatformThread);
    expect(isPlatThread, isTrue);
  });

  test('PlatformIsolate runOnPlatformThread, async operations', () async {
    final bool isPlatThread = await runOnPlatformThread(() async {
      await Future<void>.delayed(const Duration(milliseconds: 100));
      await Future<void>.delayed(const Duration(milliseconds: 100));
      await Future<void>.delayed(const Duration(milliseconds: 100));
      return isRunningOnPlatformThread;
    });
    expect(isPlatThread, isTrue);
  });

  test('PlatformIsolate runOnPlatformThread, retains state', () async {
    await runOnPlatformThread(() => ++counter);
    await Future<void>.delayed(const Duration(milliseconds: 100));
    await runOnPlatformThread(() => ++counter);
    await Future<void>.delayed(const Duration(milliseconds: 100));
    await runOnPlatformThread(() => ++counter);
    await Future<void>.delayed(const Duration(milliseconds: 100));
    final int counterValue = await runOnPlatformThread(() => counter);
    expect(counterValue, 3);
  });

  test('PlatformIsolate runOnPlatformThread, concurrent jobs', () async {
    final Future<int> future1 = runOnPlatformThread(() async {
      await Future<void>.delayed(const Duration(milliseconds: 100));
      return 1;
    });
    final Future<int> future2 = runOnPlatformThread(() async {
      await Future<void>.delayed(const Duration(milliseconds: 100));
      return 2;
    });
    final Future<int> future3 = runOnPlatformThread(() async {
      await Future<void>.delayed(const Duration(milliseconds: 100));
      return 3;
    });
    expect(await future1, 1);
    expect(await future2, 2);
    expect(await future3, 3);
  });

  test('PlatformIsolate runOnPlatformThread, send/receive messages', () async {
    late SendPort toPlatformThread;
    var sumOfReceivedMessages = 0;
    var countofReceivedMessages = 0;
    final recvPort = RawReceivePort((Object message) {
      switch (message) {
        case final SendPort sendPort:
          toPlatformThread = sendPort;
          for (int i = 1; i <= 10; i++) {
            sendPort.send(i);
          }
        case final int value:
          sumOfReceivedMessages += value;
          countofReceivedMessages++;
          if (countofReceivedMessages == 10) {
            toPlatformThread.send(true);
          }
        default:
          fail('Unexpected message: $message');
      }
    });

    final sendPort = recvPort.sendPort;
    await runOnPlatformThread(() async {
      final completer = Completer<void>();
      final recvPort = RawReceivePort((Object message) {
        switch (message) {
          case final int value:
            sendPort.send(value * 100);
          case true:
            completer.complete();
          default:
            fail('Unexpected message: $message');
        }
      });
      sendPort.send(recvPort.sendPort);
      await completer.future;
      recvPort.close();
    });

    expect(sumOfReceivedMessages, 5500); // sum(1 to 10) * 100
    recvPort.close();
  });

  test('PlatformIsolate runOnPlatformThread, throws', () async {
    bool throws = false;
    try {
      await runOnPlatformThread(() => throw 'Oh no!');
    } catch (error) {
      expect(error, 'Oh no!');
      throws = true;
    }
    expect(throws, true);
  });

  test('PlatformIsolate runOnPlatformThread, async throws', () async {
    bool throws = false;
    try {
      await runOnPlatformThread(() async {
        await Future<void>.delayed(const Duration(milliseconds: 100));
        await Future<void>.delayed(const Duration(milliseconds: 100));
        await Future<void>.delayed(const Duration(milliseconds: 100));
        throw 'Oh no!';
      });
    } catch (error) {
      expect(error, 'Oh no!');
      throws = true;
    }
    expect(throws, true);
  });

  test('PlatformIsolate runOnPlatformThread, disabled on helper isolates', () async {
    await expectLater(() async {
      await Isolate.run(() {
        return runOnPlatformThread(() => print('Unreachable'));
      });
    }, throws);
  });

  test('PlatformIsolate runOnPlatformThread, on platform isolate', () async {
    final int result = await runOnPlatformThread(
      () => runOnPlatformThread(() => runOnPlatformThread(() => runOnPlatformThread(() => 123))),
    );
    expect(result, 123);
  });

  test('PlatformIsolate runOnPlatformThread, exit disabled', () async {
    await expectLater(runOnPlatformThread(() => Isolate.exit()), throws);
  });

  test('PlatformIsolate runOnPlatformThread, unsendable object', () async {
    bool throws = false;
    try {
      await runOnPlatformThread(() => ReceivePort());
    } catch (error) {
      throws = true;
    }
    expect(throws, true);
  });

  test('PlatformIsolate runOnPlatformThread, unsendable object async', () async {
    bool throws = false;
    try {
      await runOnPlatformThread(() async {
        await Future<void>.delayed(const Duration(milliseconds: 100));
        return ReceivePort();
      });
    } catch (error) {
      throws = true;
    }
    expect(throws, true);
  });

  test('PlatformIsolate runOnPlatformThread, throws unsendable', () async {
    bool throws = false;
    try {
      await runOnPlatformThread(() => throw ReceivePort());
    } catch (error) {
      throws = true;
    }
    expect(throws, true);
  });

  test('PlatformIsolate runOnPlatformThread, throws unsendable async', () async {
    bool throws = false;
    try {
      await runOnPlatformThread(() async {
        await Future<void>.delayed(const Duration(milliseconds: 100));
        throw ReceivePort();
      });
    } catch (error) {
      throws = true;
    }
    expect(throws, true);
  });
}
