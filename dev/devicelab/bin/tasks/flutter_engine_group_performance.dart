// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart' as utils;
import 'package:flutter_devicelab/tasks/perf_tests.dart' show ListStatistics;
import 'package:path/path.dart' as path;

const String _bundleName = 'dev.flutter.multipleflutters';
const String _activityName = 'MainActivity';
const int _numberOfIterations = 10;

Future<void> _withApkInstall(
  String apkPath,
  String bundleName,
  Future<void> Function(AndroidDevice) body,
) async {
  final DeviceDiscovery devices = DeviceDiscovery();
  final AndroidDevice device = await devices.workingDevice as AndroidDevice;
  await device.unlock();
  await device.adb(<String>['uninstall', bundleName], canFail: true);
  await device.adb(<String>['install', '-r', apkPath]);
  try {
    await body(device);
  } finally {
    await device.adb(<String>['uninstall', bundleName]);
  }
}

/// Since we don't check the gradle wrapper in with the android host project we
/// yank the gradle wrapper from the module (which is added by the Flutter tool).
void _copyGradleFromModule(String source, String destination) {
  print('copying gradle from module $source to $destination');
  final String wrapperPath = path.join(source, '.android', 'gradlew');
  final String windowsWrapperPath = path.join(source, '.android', 'gradlew.bat');
  final String wrapperDestinationPath = path.join(destination, 'gradlew');
  final String windowsWrapperDestinationPath = path.join(destination, 'gradlew.bat');
  File(wrapperPath).copySync(wrapperDestinationPath);
  File(windowsWrapperPath).copySync(windowsWrapperDestinationPath);
  final Directory gradleDestinationDirectory = Directory(
    path.join(destination, 'gradle', 'wrapper'),
  );
  if (!gradleDestinationDirectory.existsSync()) {
    gradleDestinationDirectory.createSync(recursive: true);
  }
  final String gradleDestinationPath = path.join(
    gradleDestinationDirectory.path,
    'gradle-wrapper.jar',
  );
  final String gradlePath = path.join(
    source,
    '.android',
    'gradle',
    'wrapper',
    'gradle-wrapper.jar',
  );
  File(gradlePath).copySync(gradleDestinationPath);
}

Future<TaskResult> _doTest() async {
  try {
    final String flutterDirectory = utils.flutterDirectory.path;
    final String multipleFluttersPath = path.join(
      flutterDirectory,
      'dev',
      'benchmarks',
      'multiple_flutters',
    );
    final String modulePath = path.join(multipleFluttersPath, 'module');
    final String androidPath = path.join(multipleFluttersPath, 'android');

    final String gradlew = Platform.isWindows ? 'gradlew.bat' : 'gradlew';
    final String gradlewExecutable = Platform.isWindows ? '.\\$gradlew' : './$gradlew';
    await utils.flutter('precache', options: <String>['--android'], workingDirectory: modulePath);
    await utils.flutter('pub', options: <String>['get'], workingDirectory: modulePath);
    _copyGradleFromModule(modulePath, androidPath);

    await utils.eval(gradlewExecutable, <String>['assembleRelease'], workingDirectory: androidPath);

    final String apkPath = path.join(
      multipleFluttersPath,
      'android',
      'app',
      'build',
      'outputs',
      'apk',
      'release',
      'app-release.apk',
    );

    TaskResult? result;
    await _withApkInstall(apkPath, _bundleName, (AndroidDevice device) async {
      final List<int> totalMemorySamples = <int>[];
      for (int i = 0; i < _numberOfIterations; ++i) {
        await device.adb(<String>[
          'shell',
          'am',
          'start',
          '-n',
          '$_bundleName/$_bundleName.$_activityName',
        ]);
        await Future<void>.delayed(const Duration(seconds: 10));
        final Map<String, dynamic> memoryStats = await device.getMemoryStats(_bundleName);
        final int totalMemory = memoryStats['total_kb'] as int;
        totalMemorySamples.add(totalMemory);
        await device.stop(_bundleName);
      }
      final ListStatistics totalMemoryStatistics = ListStatistics(totalMemorySamples);

      final Map<String, dynamic> results = <String, dynamic>{
        ...totalMemoryStatistics.asMap('totalMemory'),
      };
      result = TaskResult.success(results, benchmarkScoreKeys: results.keys.toList());
    });

    return result ?? TaskResult.failure('no results found');
  } catch (ex) {
    return TaskResult.failure(ex.toString());
  }
}

Future<void> main() async {
  await task(_doTest);
}
