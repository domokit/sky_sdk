// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:android_driver_extensions/src/backend/android/adb.dart';
import 'package:process/process.dart';
import 'package:test/test.dart';

import 'src/fake_process_manager.dart';

// These tests are fake IO tests, so they don't actually run any commands.
void main() {
  group('AndroidDeviceTarget', () {
    late List<String> caughtArgs;
    late FakeProcessManager processManager;

    setUp(() {
      caughtArgs = <String>[];
      processManager = FakeProcessManager((String exec, List<String> args) async {
        assert(caughtArgs.isEmpty, 'Should only catch one call to run');
        caughtArgs = args;
        return FakeProcessManager.ok();
      });
    });

    test('bySerial emits "-s <number>"', () async {
      await Adb.create(
        target: const AndroidDeviceTarget.bySerial('1234'),
        processManager: processManager,
      );

      expect(caughtArgs.take(2), <String>['-s', '1234']);
    });

    test('onlyEmulatorOrDevice emits no arguments', () async {
      await Adb.create(
        target: const AndroidDeviceTarget.onlyEmulatorOrDevice(),
        processManager: processManager,
      );

      expect(caughtArgs, <String>['shell', 'echo', 'connected']);
    });

    test('onlyEmulator emits "-e"', () async {
      await Adb.create(
        target: const AndroidDeviceTarget.onlyEmulator(),
        processManager: processManager,
      );

      expect(caughtArgs.take(1), <String>['-e']);
    });

    test('onlyDevice emits "-d"', () async {
      await Adb.create(
        target: const AndroidDeviceTarget.onlyDevice(),
        processManager: processManager,
      );

      expect(caughtArgs.take(1), <String>['-d']);
    });
  });

  test('if isDeviceConnected fails, bootstrap fails', () async {
    final ProcessManager processManager = FakeProcessManager((
      String exec,
      List<String> args,
    ) async {
      return FakeProcessManager.error('error');
    });

    expect(
      Adb.create(processManager: processManager),
      throwsA(
        isA<StateError>().having(
          (StateError e) => e.message,
          'message',
          'No device connected: error',
        ),
      ),
    );
  });

  test('screencap invokes "exec-out screencap -p"', () async {
    final FakeProcessManager processManager = FakeProcessManager((
      String exec,
      List<String> args,
    ) async {
      switch (args) {
        case ['shell', 'echo', 'connected']:
          return FakeProcessManager.ok('connected');
        case ['exec-out', 'screencap', '-p']:
          return FakeProcessManager.okBinary(<int>[0, 1, 2, 3]);
        default:
          throw UnsupportedError('Unknown command: $args');
      }
    });

    final Adb adb = await Adb.create(processManager: processManager);

    final Uint8List result = await adb.screencap();
    expect(result, <int>[0, 1, 2, 3]);
  });

  test('tap invokes "shell input tap"', () async {
    final FakeProcessManager processManager = FakeProcessManager((
      String exec,
      List<String> args,
    ) async {
      switch (args) {
        case ['shell', 'echo', 'connected']:
          return FakeProcessManager.ok('connected');
        case ['shell', 'input', 'tap', '1', '2']:
          return FakeProcessManager.ok();
        default:
          throw UnsupportedError('Unknown command: $args');
      }
    });

    final Adb adb = await Adb.create(processManager: processManager);

    await adb.tap(1, 2);
  });
}
