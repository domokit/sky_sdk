// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:process/process.dart';

import '../src/common.dart';
import 'test_data/basic_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;
  final BasicProject project = BasicProject();
  late FlutterRunTestDriver flutter;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterRunTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.stop();
    tryToDelete(tempDir);
  });

  testWithoutContext('flutter run reports an error if an invalid device is supplied', () async {
    // This test forces flutter to check for all possible devices to catch issues
    // like https://github.com/flutter/flutter/issues/21418 which were skipped
    // over because other integration tests run using flutter-tester which short-cuts
    // some of the checks for devices.

    const ProcessManager processManager = LocalProcessManager();
    final ProcessResult proc = await processManager.run(<String>[
      flutterBin,
      'run',
      '-d',
      'invalid-device-id',
    ], workingDirectory: tempDir.path);

    expect(proc.stdout, isNot(contains('flutter has exited unexpectedly')));
    expect(proc.stderr, isNot(contains('flutter has exited unexpectedly')));
    if (!proc.stderr.toString().contains('Unable to locate a development') &&
        !proc.stdout.toString().contains('No supported devices found with name or id matching')) {
      fail("'flutter run -d invalid-device-id' did not produce the expected error");
    }
  });

  testWithoutContext('sets activeDevToolsServerAddress extension', () async {
    await flutter.run(
      startPaused: true,
      withDebugger: true,
      additionalCommandArgs: <String>['--devtools-server-address', 'http://127.0.0.1:9110'],
    );
    await flutter.resume();
    await pollForServiceExtensionValue<String>(
      testDriver: flutter,
      extension: 'ext.flutter.activeDevToolsServerAddress',
      continuePollingValue: '',
      matches: equals('http://127.0.0.1:9110'),
    );
    await pollForServiceExtensionValue<String>(
      testDriver: flutter,
      extension: 'ext.flutter.connectedVmServiceUri',
      continuePollingValue: '',
      matches: isNotEmpty,
    );
  });

  testWithoutContext('reports deviceId and mode in app.start event', () async {
    await flutter.run();
    expect(flutter.currentRunningDeviceId, 'flutter-tester');
    expect(flutter.currentRunningMode, 'debug');
  });
}
