// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show File, Platform;
import 'dart:typed_data';

import 'package:collection/collection.dart';
import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:flutter_devicelab/tasks/integration_tests.dart';
import 'package:path/path.dart' as path;
import 'package:standard_message_codec/standard_message_codec.dart';

Future<void> main() async {
  deviceOperatingSystem = DeviceOperatingSystem.android;
  await task(() async {
    await createFlavorsTest().call();
    await createIntegrationTestFlavorsTest().call();

    final String projectPath = '${flutterDirectory.path}/dev/integration_tests/flavors';
    final TaskResult installTestsResult = await inDirectory(projectPath, () async {
      final List<TaskResult> testResults = <TaskResult>[
        await _testInstallDebugPaidFlavor(projectPath),
        await _testInstallBogusFlavor(),
      ];

      final TaskResult? firstInstallFailure = testResults.firstWhereOrNull(
        (TaskResult element) => element.failed,
      );

      return firstInstallFailure ?? TaskResult.success(null);
    });

    await _testFlavorsWhenBuildStartsWithGradle(projectPath);

    return installTestsResult;
  });
}

// Ensures installation works. Also tests asset bundling while we are at it.
Future<TaskResult> _testInstallDebugPaidFlavor(String projectDir) async {
  await evalFlutter('install', options: <String>['--debug', '--flavor', 'paid']);

  final Uint8List assetManifestFileData =
      File(
        path.join(
          projectDir,
          'build',
          'app',
          'intermediates',
          'assets',
          'paidDebug',
          'mergePaidDebugAssets',
          'flutter_assets',
          'AssetManifest.bin',
        ),
      ).readAsBytesSync();

  final Map<Object?, Object?> assetManifest =
      const StandardMessageCodec().decodeMessage(ByteData.sublistView(assetManifestFileData))
          as Map<Object?, Object?>;

  if (assetManifest.containsKey('assets/free/free.txt')) {
    return TaskResult.failure(
      'Expected the asset "assets/free/free.txt", which '
      ' was declared with a flavor of "free" to not be included in the asset bundle '
      ' because the --flavor was set to "paid".',
    );
  }

  if (!assetManifest.containsKey('assets/paid/paid.txt')) {
    return TaskResult.failure(
      'Expected the asset "assets/paid/paid.txt", which '
      ' was declared with a flavor of "paid" to be included in the asset bundle '
      ' because the --flavor was set to "paid".',
    );
  }

  await flutter('install', options: <String>['--debug', '--flavor', 'paid', '--uninstall-only']);

  return TaskResult.success(null);
}

Future<TaskResult> _testInstallBogusFlavor() async {
  final StringBuffer stderr = StringBuffer();
  await evalFlutter(
    'install',
    canFail: true,
    stderr: stderr,
    options: <String>['--flavor', 'bogus'],
  );

  final String stderrString = stderr.toString();
  final String expectedApkPath = path.join(
    'build',
    'app',
    'outputs',
    'flutter-apk',
    'app-bogus-release.apk',
  );
  if (!stderrString.contains('"$expectedApkPath" does not exist.')) {
    print(stderrString);
    return TaskResult.failure('Should not succeed with bogus flavor');
  }

  return TaskResult.success(null);
}

Future<TaskResult> _testFlavorsWhenBuildStartsWithGradle(String projectDir) async {
  final String gradlew = Platform.isWindows ? 'gradlew.bat' : 'gradlew';
  final String gradlewExecutable = Platform.isWindows ? '.\\$gradlew' : './$gradlew';

  final String androidDirPath = '$projectDir/android';
  final StringBuffer stdout = StringBuffer();

  // Prebuild the project to generate the Android gradle wrapper files.
  await inDirectory(projectDir, () async {
    await flutter('build', options: <String>['apk', '--config-only']);
  });

  await inDirectory(androidDirPath, () async {
    await exec(gradlewExecutable, <String>['clean']);
    await exec(gradlewExecutable, <String>[':app:assemblePaidDebug', '--info'], output: stdout);
  });

  final String stdoutString = stdout.toString();

  if (!stdoutString.contains('-dFlavor=paid')) {
    return TaskResult.failure('Expected to see -dFlavor=paid in the gradle verbose output');
  }

  final String appPath = path.join(
    projectDir,
    'build',
    'app',
    'outputs',
    'flutter-apk',
    'app-paid-debug.apk',
  );

  return createFlavorsTest(
    extraOptions: <String>['--flavor', 'paid', '--use-application-binary=$appPath'],
  ).call();
}
