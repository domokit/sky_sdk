// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' hide Directory;

import 'package:file/file.dart';
import 'package:process/process.dart';

import '../src/common.dart';
import 'test_data/basic_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;
  late Process daemonProcess;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('daemon_mode_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
    daemonProcess.kill();
  });

  testWithoutContext('startup events', () async {
    final BasicProject project = BasicProject();
    await project.setUpIn(tempDir);

    const ProcessManager processManager = LocalProcessManager();
    daemonProcess = await processManager.start(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      '--show-test-device',
      'daemon',
    ], workingDirectory: tempDir.path);

    final StreamController<String> stdout = StreamController<String>.broadcast();
    transformToLines(daemonProcess.stdout).listen((String line) => stdout.add(line));
    final Stream<Map<String, Object?>> stream =
        stdout.stream
            .map<Map<String, Object?>?>(parseFlutterResponse)
            .where((Map<String, Object?>? value) => value != null)
            .cast<Map<String, Object?>>();

    final [
      Map<String, Object?> connectedEvent,
      Map<String, Object?> logMessage,
    ] = await Future.wait(<Future<Map<String, Object?>>>[
      stream.firstWhere((Map<String, Object?> e) => e['event'] == 'daemon.connected'),
      stream.firstWhere((Map<String, Object?> e) => e['event'] == 'daemon.logMessage'),
    ]);

    // Check the connected message has a version.
    final Map<String, Object?> connectedParams = connectedEvent['params']! as Map<String, Object?>;
    expect(connectedParams['version'], isNotNull);

    // Check we got the startup message.
    final Map<String, Object?> logParams = logMessage['params']! as Map<String, Object?>;
    expect(logParams['level'], 'status');
    expect(logParams['message'], 'Device daemon started.');
  });

  testWithoutContext('device.getDevices', () async {
    final BasicProject project = BasicProject();
    await project.setUpIn(tempDir);

    const ProcessManager processManager = LocalProcessManager();
    daemonProcess = await processManager.start(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      '--show-test-device',
      'daemon',
    ], workingDirectory: tempDir.path);

    final StreamController<String> stdout = StreamController<String>.broadcast();
    transformToLines(daemonProcess.stdout).listen((String line) => stdout.add(line));
    final Stream<Map<String, Object?>?> stream = stdout.stream
        .map<Map<String, Object?>?>(parseFlutterResponse)
        .where((Map<String, Object?>? value) => value != null);

    Map<String, Object?> response = (await stream.first)!;
    expect(response['event'], 'daemon.connected');

    // start listening for devices
    daemonProcess.stdin.writeln(
      '[${jsonEncode(<String, dynamic>{'id': 1, 'method': 'device.enable'})}]',
    );
    response = (await stream.firstWhere((Map<String, Object?>? json) => json!['id'] == 1))!;
    expect(response['id'], 1);
    expect(response['error'], isNull);

    // [{"event":"device.added","params":{"id":"flutter-tester","name":
    //   "Flutter test device","platform":"flutter-tester","emulator":false}}]
    response = (await stream.first)!;
    expect(response['event'], 'device.added');

    // get the list of all devices
    daemonProcess.stdin.writeln(
      '[${jsonEncode(<String, dynamic>{'id': 2, 'method': 'device.getDevices'})}]',
    );
    // Skip other device.added events that may fire (desktop/web devices).
    response =
        (await stream.firstWhere(
          (Map<String, Object?>? response) => response!['event'] != 'device.added',
        ))!;
    expect(response['id'], 2);
    expect(response['error'], isNull);

    final dynamic result = response['result'];
    expect(result, isList);
    expect(result, isNotEmpty);
  });
}
