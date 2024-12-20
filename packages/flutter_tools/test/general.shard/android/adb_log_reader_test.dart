// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/android/android_device.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

const int kLollipopVersionCode = 21;
const String kLastLogcatTimestamp = '11-27 15:39:04.506';

/// By default the android log reader accepts lines that match no patterns
/// if the previous line was a match. Include an intentionally non-matching
/// line as the first input to disable this behavior.
const String kDummyLine = 'Contents are not important\n';

class _FakeVm extends Fake implements VM {
  _FakeVm(this._appPid);

  final int _appPid;

  @override
  int? get pid => _appPid;
}

class _FakeFlutterVmService extends Fake implements FlutterVmService {
  _FakeFlutterVmService(this._appPid);

  final int _appPid;

  @override
  Future<VM?> getVmGuarded() async => _FakeVm(_appPid);
}

void main() {
  testWithoutContext('AdbLogReader ignores spam from SurfaceSyncer', () async {
    const int appPid = 1;
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: const <String>['adb', '-s', '1234', 'shell', '-x', 'logcat', '-v', 'time'],
        completer: Completer<void>.sync(),
        stdout:
            '$kDummyLine'
            '05-11 12:54:46.665 W/flutter($appPid): Hello there!\n'
            '05-11 12:54:46.665 E/SurfaceSyncer($appPid): Failed to find sync for id=9\n'
            '05-11 12:54:46.665 E/SurfaceSyncer($appPid): Failed to find sync for id=10\n',
      ),
    ]);
    final AdbLogReader logReader = await AdbLogReader.createLogReader(
      createFakeDevice(null),
      processManager,
      BufferLogger.test(),
    );
    await logReader.provideVmService(_FakeFlutterVmService(appPid));
    final Completer<void> onDone = Completer<void>.sync();
    final List<String> emittedLines = <String>[];
    logReader.logLines.listen((String line) {
      emittedLines.add(line);
    }, onDone: onDone.complete);
    await null;
    logReader.dispose();
    await onDone.future;
    expect(emittedLines, const <String>['W/flutter($appPid): Hello there!']);
  });

  testWithoutContext('AdbLogReader calls adb logcat with expected flags apiVersion 21', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>[
          'adb',
          '-s',
          '1234',
          'shell',
          '-x',
          'logcat',
          '-v',
          'time',
          '-T',
          "'$kLastLogcatTimestamp'",
        ],
      ),
    ]);
    await AdbLogReader.createLogReader(
      createFakeDevice(kLollipopVersionCode),
      processManager,
      BufferLogger.test(),
    );

    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('AdbLogReader calls adb logcat with expected flags apiVersion < 21', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', '-x', 'logcat', '-v', 'time'],
      ),
    ]);
    await AdbLogReader.createLogReader(
      createFakeDevice(kLollipopVersionCode - 1),
      processManager,
      BufferLogger.test(),
    );

    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('AdbLogReader calls adb logcat with expected flags null apiVersion', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['adb', '-s', '1234', 'shell', '-x', 'logcat', '-v', 'time'],
      ),
    ]);
    await AdbLogReader.createLogReader(createFakeDevice(null), processManager, BufferLogger.test());

    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext(
    'AdbLogReader calls adb logcat with expected flags when requesting past logs',
    () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            'adb',
            '-s',
            '1234',
            'shell',
            '-x',
            'logcat',
            '-v',
            'time',
            '-s',
            'flutter',
          ],
        ),
      ]);
      await AdbLogReader.createLogReader(
        createFakeDevice(null),
        processManager,
        BufferLogger.test(),
        includePastLogs: true,
      );

      expect(processManager, hasNoRemainingExpectations);
    },
  );

  testWithoutContext('AdbLogReader handles process early exit', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: const <String>['adb', '-s', '1234', 'shell', '-x', 'logcat', '-v', 'time'],
        completer: Completer<void>.sync(),
        stdout: 'Hello There\n',
      ),
    ]);
    final AdbLogReader logReader = await AdbLogReader.createLogReader(
      createFakeDevice(null),
      processManager,
      BufferLogger.test(),
    );
    final Completer<void> onDone = Completer<void>.sync();
    logReader.logLines.listen((String _) {}, onDone: onDone.complete);

    logReader.dispose();
    await onDone.future;
  });

  testWithoutContext('AdbLogReader does not filter output from AndroidRuntime crashes', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: const <String>['adb', '-s', '1234', 'shell', '-x', 'logcat', '-v', 'time'],
        completer: Completer<void>.sync(),
        // Example stack trace from an incorrectly named application:name in the AndroidManifest.xml
        stdout:
            '$kDummyLine'
            '05-11 12:54:46.665 E/AndroidRuntime(11787): FATAL EXCEPTION: main\n'
            '05-11 12:54:46.665 E/AndroidRuntime(11787): Process: com.example.foobar, PID: 11787\n'
            '05-11 12:54:46.665 java.lang.RuntimeException: Unable to instantiate application '
            'io.flutter.app.FlutterApplication2: java.lang.ClassNotFoundException:\n',
      ),
    ]);
    final AdbLogReader logReader = await AdbLogReader.createLogReader(
      createFakeDevice(null),
      processManager,
      BufferLogger.test(),
    );
    await expectLater(
      logReader.logLines,
      emitsInOrder(<String>[
        'E/AndroidRuntime(11787): FATAL EXCEPTION: main',
        'E/AndroidRuntime(11787): Process: com.example.foobar, PID: 11787',
        'java.lang.RuntimeException: Unable to instantiate application io.flutter.app.FlutterApplication2: java.lang.ClassNotFoundException:',
      ]),
    );

    logReader.dispose();
  });
}

AndroidDevice createFakeDevice(int? sdkLevel) {
  return FakeAndroidDevice(sdkLevel.toString(), kLastLogcatTimestamp);
}

class FakeAndroidDevice extends Fake implements AndroidDevice {
  FakeAndroidDevice(this._apiVersion, this._lastLogcatTimestamp);

  final String _lastLogcatTimestamp;
  final String _apiVersion;

  @override
  String get name => 'test-device';

  @override
  Future<String> get apiVersion => Future<String>.value(_apiVersion);

  @override
  Future<String> lastLogcatTimestamp() async => _lastLogcatTimestamp;

  @override
  List<String> adbCommandForDevice(List<String> command) {
    return <String>['adb', '-s', '1234', ...command];
  }
}
