// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:args/args.dart';

import '../framework/devices.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

/// [Task] for defining build-test separation.
///
/// Using this [Task] allows DeviceLab capacity to only be spent on the [test].
abstract class BuildTestTask {
  BuildTestTask(this.args, {this.workingDirectory, this.runFlutterClean = true}) {
    final ArgResults argResults = argParser.parse(args);
    applicationBinaryPath = argResults[kApplicationBinaryPathOption] as String?;
    buildOnly = argResults[kBuildOnlyFlag] as bool;
    testOnly = argResults[kTestOnlyFlag] as bool;
  }

  static const String kApplicationBinaryPathOption = 'application-binary-path';
  static const String kBuildOnlyFlag = 'build';
  static const String kTestOnlyFlag = 'test';

  final ArgParser argParser =
      ArgParser()
        ..addOption(kApplicationBinaryPathOption)
        ..addFlag(kBuildOnlyFlag)
        ..addFlag(kTestOnlyFlag);

  /// Args passed from the test runner via "--task-arg".
  final List<String> args;

  /// If true, skip [test].
  bool buildOnly = false;

  /// If true, skip [build].
  bool testOnly = false;

  /// Whether to run `flutter clean` before building the application under test.
  final bool runFlutterClean;

  /// Path to a built application to use in [test].
  ///
  /// If not given, will default to child's expected location.
  String? applicationBinaryPath;

  /// Where the test artifacts are stored, such as performance results.
  final Directory? workingDirectory;

  /// Run Flutter build to create [applicationBinaryPath].
  Future<void> build() async {
    await inDirectory<void>(workingDirectory, () async {
      if (runFlutterClean) {
        section('FLUTTER CLEAN');
        await flutter('clean');
      }
      section('BUILDING APPLICATION');
      await flutter('build', options: getBuildArgs(deviceOperatingSystem));
      copyArtifacts();
    });
  }

  /// Run Flutter drive test from [getTestArgs] against the application under test on the device.
  ///
  /// This assumes that [applicationBinaryPath] exists.
  Future<TaskResult> test() async {
    final Device device = await devices.workingDevice;
    await device.unlock();
    await inDirectory<void>(workingDirectory, () async {
      section('DRIVE START');
      await flutter('drive', options: getTestArgs(deviceOperatingSystem, device.deviceId));
    });

    return parseTaskResult();
  }

  /// Args passed to flutter build to build the application under test.
  List<String> getBuildArgs(DeviceOperatingSystem deviceOperatingSystem) =>
      throw UnimplementedError('getBuildArgs is not implemented');

  /// Args passed to flutter drive to test the built application.
  List<String> getTestArgs(DeviceOperatingSystem deviceOperatingSystem, String deviceId) =>
      throw UnimplementedError('getTestArgs is not implemented');

  /// Copy artifacts to [applicationBinaryPath] if specified.
  ///
  /// This is needed when running from CI, so that LUCI recipes know where to locate and upload artifacts to GCS.
  void copyArtifacts() => throw UnimplementedError('copyArtifacts is not implemented');

  /// Logic to construct [TaskResult] from this test's results.
  Future<TaskResult> parseTaskResult() =>
      throw UnimplementedError('parseTaskResult is not implemented');

  /// Path to the built application under test.
  ///
  /// Tasks can override to support default values. Otherwise, it will default
  /// to needing to be passed as an argument in the test runner.
  String? getApplicationBinaryPath() => applicationBinaryPath;

  /// Run this task.
  ///
  /// Throws [Exception] when unnecessary arguments are passed.
  Future<TaskResult> call() async {
    if (buildOnly && testOnly) {
      throw Exception('Both build and test should not be passed. Pass only one.');
    }

    if (!testOnly) {
      await build();
    }

    if (buildOnly) {
      return TaskResult.buildOnly();
    }

    return test();
  }
}
