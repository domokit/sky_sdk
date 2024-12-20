// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:engine_tool/src/flutter_tool_interop/device.dart';
import 'package:engine_tool/src/flutter_tool_interop/flutter_tool.dart';
import 'package:engine_tool/src/flutter_tool_interop/target_platform.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:test/test.dart';

import '../src/matchers.dart';
import '../src/utils.dart';

void main() {
  test('devices handles a non-zero exit code', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('devices'),
          exitCode: 1,
          stdout: 'stdout',
          stderr: 'stderr',
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final flutterTool = FlutterTool.fromEnvironment(testEnv.environment);
    expect(
      () => flutterTool.devices(),
      throwsA(
        isA<FatalError>().having(
          (a) => a.toString(),
          'toString()',
          allOf([
            contains('Failed to run'),
            contains('EXITED: 1'),
            contains('STDOUT:\nstdout'),
            contains('STDERR:\nstderr'),
          ]),
        ),
      ),
    );
  });

  test('devices handles unparseable data', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess((List<String> command) => command.contains('devices'), stdout: 'not json'),
      ],
    );
    addTearDown(testEnv.cleanup);

    final flutterTool = FlutterTool.fromEnvironment(testEnv.environment);
    expect(
      () => flutterTool.devices(),
      throwsA(
        isA<FatalError>().having(
          (a) => a.toString(),
          'toString()',
          allOf([contains('Failed to parse'), contains('STDOUT:\nnot json')]),
        ),
      ),
    );
  });

  test('parses a single device successfully', () async {
    const testAndroidArm64Device = Device(
      name: 'test_device',
      id: 'test_id',
      targetPlatform: TargetPlatform.androidArm64,
    );

    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('devices'),
          stdout: jsonEncode([testAndroidArm64Device]),
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final flutterTool = FlutterTool.fromEnvironment(testEnv.environment);
    final devices = await flutterTool.devices();
    expect(devices, equals([testAndroidArm64Device]));
  });

  test('parses multiple devices successfully', () async {
    const testAndroidArm64Device = Device(
      name: 'test_device',
      id: 'test_id',
      targetPlatform: TargetPlatform.androidArm64,
    );
    const testIosArm64Device = Device(
      name: 'test_ios_device',
      id: 'test_ios_id',
      targetPlatform: TargetPlatform.iOSArm64,
    );

    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('devices'),
          stdout: jsonEncode([testAndroidArm64Device, testIosArm64Device]),
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final flutterTool = FlutterTool.fromEnvironment(testEnv.environment);
    final devices = await flutterTool.devices();
    expect(devices, equals([testAndroidArm64Device, testIosArm64Device]));
  });

  test('skips entry that is not a JSON map and emits a log error', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('devices'),
          stdout: jsonEncode(['not a map']),
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final flutterTool = FlutterTool.fromEnvironment(testEnv.environment);
    final devices = await flutterTool.devices();
    expect(devices, isEmpty);

    expect(
      testEnv.testLogs,
      contains(
        logRecord(contains('Skipping device: Expected a JSON Object'), level: Logger.errorLevel),
      ),
    );
  });

  test('skips entry that is missing an expected property', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('devices'),
          stdout: jsonEncode([
            <String, Object?>{'name': 'test_device', 'id': 'test_id'},
          ]),
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final flutterTool = FlutterTool.fromEnvironment(testEnv.environment);
    final devices = await flutterTool.devices();
    expect(devices, isEmpty);

    expect(
      testEnv.testLogs,
      contains(
        logRecord(
          contains('Skipping device: Failed to parse JSON Object'),
          level: Logger.errorLevel,
        ),
      ),
    );
  });

  test('skips entry with an unrecognized targetPlatform', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('devices'),
          stdout: jsonEncode([
            <String, Object?>{'name': 'test_device', 'id': 'test_id', 'targetPlatform': 'unknown'},
          ]),
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final flutterTool = FlutterTool.fromEnvironment(testEnv.environment);
    final devices = await flutterTool.devices();
    expect(devices, isEmpty);

    expect(
      testEnv.testLogs,
      contains(logRecord(contains('Unrecognized TargetPlatform'), level: Logger.errorLevel)),
    );
  });
}
