// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: avoid_print

import 'dart:async';
import 'dart:isolate';

import 'split_lib_test.dart' deferred as splitlib;

void main() {}

@pragma('vm:entry-point')
void sayHi() {
  print('Hi');
}

@pragma('vm:entry-point')
void throwExceptionNow() {
  throw AssertionError('Hello');
}

@pragma('vm:entry-point')
Future<void> canRegisterNativeCallback() async {
  print('In function canRegisterNativeCallback');
  notifyNative();
  print('Called native method from canRegisterNativeCallback');
}

Future<void>? splitLoadFuture;

@pragma('vm:entry-point')
void canCallDeferredLibrary() {
  print('In function canCallDeferredLibrary');
  splitLoadFuture = splitlib
      .loadLibrary()
      .then((_) {
        print('Deferred load complete');
        notifySuccess(splitlib.splitAdd(10, 23) == 33);
      })
      .catchError((_) {
        print('Deferred load error');
        notifySuccess(false);
      });
  notifyNative();
}

@pragma('vm:external-name', 'NotifyNative')
external void notifyNative();

@pragma('vm:entry-point')
void testIsolateShutdown() {}

@pragma('vm:external-name', 'NotifyNative')
external void notifyResult(bool success);
@pragma('vm:external-name', 'PassMessage')
external void passMessage(String message);

void secondaryIsolateMain(String message) {
  print('Secondary isolate got message: $message');
  passMessage('Hello from code is secondary isolate.');
  notifyNative();
}

@pragma('vm:entry-point')
void testCanLaunchSecondaryIsolate() {
  final onExit = RawReceivePort((_) {
    notifyNative();
  });
  Isolate.spawn(secondaryIsolateMain, 'Hello from root isolate.', onExit: onExit.sendPort);
}

@pragma('vm:entry-point')
Future<void> testIsolateStartupFailure() async {
  Future<void> mainTest(void _) async {
    Future<void> testSuccessfullIsolateLaunch() async {
      final onMessage = ReceivePort();
      final onExit = ReceivePort();

      final messages = StreamIterator<dynamic>(onMessage);
      final exits = StreamIterator<dynamic>(onExit);

      await Isolate.spawn(
        (SendPort port) => port.send('good'),
        onMessage.sendPort,
        onExit: onExit.sendPort,
      );
      if (!await messages.moveNext()) {
        throw AssertionError('Failed to receive message');
      }
      if (messages.current != 'good') {
        throw AssertionError('Failed to receive correct message');
      }
      if (!await exits.moveNext()) {
        throw AssertionError('Failed to receive onExit');
      }
      messages.cancel();
      exits.cancel();
    }

    Future<void> testUnsuccessfullIsolateLaunch() async {
      IsolateSpawnException? error;
      try {
        await Isolate.spawn((_) {}, null);
      } on IsolateSpawnException catch (e) {
        error = e;
      }
      if (error == null) {
        throw AssertionError('Expected isolate spawn to fail.');
      }
    }

    await testSuccessfullIsolateLaunch();
    makeNextIsolateSpawnFail();
    await testUnsuccessfullIsolateLaunch();
    notifyNative();
  }

  // The root isolate will not run an eventloop, so we have to run the actual
  // test in an isolate.
  Isolate.spawn(mainTest, null);
}

@pragma('vm:external-name', 'MakeNextIsolateSpawnFail')
external void makeNextIsolateSpawnFail();

@pragma('vm:entry-point')
void testCanReceiveArguments(List<String> args) {
  notifyResult(args.length == 1 && args[0] == 'arg1');
}

@pragma('vm:entry-point')
void trampoline() {
  notifyNative();
}

@pragma('vm:external-name', 'NotifySuccess')
external void notifySuccess(bool success);

@pragma('vm:entry-point')
void testCanConvertEmptyList(List<int> args) {
  notifySuccess(args.isEmpty);
}

@pragma('vm:entry-point')
void testCanConvertListOfStrings(List<String> args) {
  notifySuccess(
    args.length == 4 &&
        args[0] == 'tinker' &&
        args[1] == 'tailor' &&
        args[2] == 'soldier' &&
        args[3] == 'sailor',
  );
}

@pragma('vm:entry-point')
void testCanConvertListOfDoubles(List<double> args) {
  notifySuccess(
    args.length == 4 && args[0] == 1.0 && args[1] == 2.0 && args[2] == 3.0 && args[3] == 4.0,
  );
}

@pragma('vm:entry-point')
void testCanConvertListOfInts(List<int> args) {
  notifySuccess(args.length == 4 && args[0] == 1 && args[1] == 2 && args[2] == 3 && args[3] == 4);
}

bool didCallRegistrantBeforeEntrypoint = false;

// Test the Dart plugin registrant.
@pragma('vm:entry-point')
class _PluginRegistrant {
  @pragma('vm:entry-point')
  static void register() {
    if (didCallRegistrantBeforeEntrypoint) {
      throw AssertionError('_registerPlugins is being called twice');
    }
    didCallRegistrantBeforeEntrypoint = true;
  }
}

@pragma('vm:entry-point')
void mainForPluginRegistrantTest() {
  if (didCallRegistrantBeforeEntrypoint) {
    passMessage('_PluginRegistrant.register() was called');
  } else {
    passMessage('_PluginRegistrant.register() was not called');
  }
}

@pragma('vm:entry-point')
void mainForPlatformIsolates() {
  passMessage('Platform isolate is ready');
}

@pragma('vm:entry-point')
void emptyMain(List<Object?> args) {}

@pragma('vm:entry-point')
Function createEntryPointForPlatIsoSendAndRecvTest() {
  final port = RawReceivePort();
  port.handler = (Object? message) {
    port.close();
    (message! as SendPort).send('Hello from root isolate!');
  };
  final SendPort sendPort = port.sendPort;
  return () {
    final replyPort = RawReceivePort();
    replyPort.handler = (Object? message) {
      replyPort.close();
      passMessage('Platform isolate received: $message');
    };
    sendPort.send(replyPort.sendPort);
  };
}

@pragma('vm:entry-point')
void mainForPlatformIsolatesThrowError() {
  throw AssertionError('Error from platform isolate');
}
