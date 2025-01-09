// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

final String gradlew = Platform.isWindows ? 'gradlew.bat' : 'gradlew';
final String gradlewExecutable = Platform.isWindows ? '.\\$gradlew' : './$gradlew';

/// Tests that the Android app containing a Flutter module can be built when
/// it has custom build types and flavors.
Future<void> main() async {
  await task(() async {
    section('Find Java');

    final String? javaHome = await findJavaHome();
    if (javaHome == null) {
      return TaskResult.failure('Could not find Java');
    }

    print('\nUsing JAVA_HOME=$javaHome');

    section('Create Flutter module project');

    await flutter('precache', options: <String>['--android', '--no-ios']);

    final Directory tempDir = Directory.systemTemp.createTempSync('flutter_module_test.');
    final Directory projectDir = Directory(path.join(tempDir.path, 'hello'));
    try {
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>['--org', 'io.flutter.devicelab', '--template=module', 'hello'],
        );
      });

      section('Run flutter pub get');

      await inDirectory(projectDir, () async {
        await flutter('pub', options: <String>['get']);
      });

      section('Add to existing Android app');

      final Directory hostAppDir = Directory(
        path.join(tempDir.path, 'hello_host_app_with_custom_build'),
      );
      mkdir(hostAppDir);
      recursiveCopy(
        Directory(
          path.join(
            flutterDirectory.path,
            'dev',
            'integration_tests',
            'module_host_with_custom_build_v2_embedding',
          ),
        ),
        hostAppDir,
      );
      copy(File(path.join(projectDir.path, '.android', gradlew)), hostAppDir);
      copy(
        File(path.join(projectDir.path, '.android', 'gradle', 'wrapper', 'gradle-wrapper.jar')),
        Directory(path.join(hostAppDir.path, 'gradle', 'wrapper')),
      );

      Future<void> clean() async {
        section('Clean');
        await inDirectory(hostAppDir, () async {
          await exec(
            gradlewExecutable,
            <String>['clean'],
            environment: <String, String>{'JAVA_HOME': javaHome},
          );
        });
      }

      if (!Platform.isWindows) {
        section('Make $gradlewExecutable executable');
        await inDirectory(hostAppDir, () async {
          await exec('chmod', <String>['+x', gradlewExecutable]);
        });
      }

      section('Build debug APKs');

      section('Run app:assembleDemoDebug');

      await inDirectory(hostAppDir, () async {
        await exec(
          gradlewExecutable,
          <String>['app:assembleDemoDebug'],
          environment: <String, String>{'JAVA_HOME': javaHome},
        );
      });

      final String demoDebugApk = path.join(
        hostAppDir.path,
        'app',
        'build',
        'outputs',
        'apk',
        'demo',
        'debug',
        'app-demo-debug.apk',
      );

      if (!exists(File(demoDebugApk))) {
        return TaskResult.failure('Failed to build app-demo-debug.apk');
      }

      section('Verify snapshots in app-demo-debug.apk');

      checkCollectionContains<String>(<String>[
        ...flutterAssets,
        ...debugAssets,
      ], await getFilesInApk(demoDebugApk));

      await clean();

      // Change the order of the task and ensure that flutter_assets are in the APK.
      // https://github.com/flutter/flutter/pull/41333
      section('Run app:assembleDemoDebug - Merge assets before processing manifest');

      await inDirectory(hostAppDir, () async {
        await exec(
          gradlewExecutable,
          <String>[
            // Normally, `app:processDemoDebugManifest` runs before `app:mergeDemoDebugAssets`.
            // In this case, we run `app:mergeDemoDebugAssets` first.
            'app:mergeDemoDebugAssets',
            'app:processDemoDebugManifest',
            'app:assembleDemoDebug',
          ],
          environment: <String, String>{'JAVA_HOME': javaHome},
        );
      });

      final String demoDebugApk2 = path.join(
        hostAppDir.path,
        'app',
        'build',
        'outputs',
        'apk',
        'demo',
        'debug',
        'app-demo-debug.apk',
      );

      if (!exists(File(demoDebugApk2))) {
        return TaskResult.failure('Failed to build app-demo-debug.apk');
      }

      section('Verify snapshots in app-demo-debug.apk');

      checkCollectionContains<String>(<String>[
        ...flutterAssets,
        ...debugAssets,
      ], await getFilesInApk(demoDebugApk2));

      await clean();

      section('Run app:assembleDemoStaging');

      await inDirectory(hostAppDir, () async {
        await exec(
          gradlewExecutable,
          <String>['app:assembleDemoStaging'],
          environment: <String, String>{'JAVA_HOME': javaHome},
        );
      });

      final String demoStagingApk = path.join(
        hostAppDir.path,
        'app',
        'build',
        'outputs',
        'apk',
        'demo',
        'staging',
        'app-demo-staging.apk',
      );

      if (!exists(File(demoStagingApk))) {
        return TaskResult.failure('Failed to build app-demo-staging.apk');
      }

      section('Verify snapshots in app-demo-staging.apk');

      checkCollectionContains<String>(<String>[
        ...flutterAssets,
        ...debugAssets,
      ], await getFilesInApk(demoStagingApk));

      await clean();

      section('Build release APKs');

      section('Run app:assembleDemoRelease');

      await inDirectory(hostAppDir, () async {
        await exec(
          gradlewExecutable,
          <String>['app:assembleDemoRelease'],
          environment: <String, String>{'JAVA_HOME': javaHome},
        );
      });

      final String demoReleaseApk = path.join(
        hostAppDir.path,
        'app',
        'build',
        'outputs',
        'apk',
        'demo',
        'release',
        'app-demo-release-unsigned.apk',
      );

      if (!exists(File(demoReleaseApk))) {
        return TaskResult.failure('Failed to build app-demo-release-unsigned.apk');
      }

      section('Verify AOT ELF in app-demo-release-unsigned.apk');

      checkCollectionContains<String>(<String>[
        ...flutterAssets,
        'lib/arm64-v8a/libflutter.so',
        'lib/arm64-v8a/libapp.so',
        'lib/armeabi-v7a/libflutter.so',
        'lib/armeabi-v7a/libapp.so',
      ], await getFilesInApk(demoReleaseApk));

      await clean();

      section('Run app:assembleDemoProd');

      await inDirectory(hostAppDir, () async {
        await exec(
          gradlewExecutable,
          <String>['app:assembleDemoProd'],
          environment: <String, String>{'JAVA_HOME': javaHome},
        );
      });

      final String demoProdApk = path.join(
        hostAppDir.path,
        'app',
        'build',
        'outputs',
        'apk',
        'demo',
        'prod',
        'app-demo-prod-unsigned.apk',
      );

      if (!exists(File(demoProdApk))) {
        return TaskResult.failure('Failed to build app-demo-prod-unsigned.apk');
      }

      section('Verify AOT ELF in app-demo-prod-unsigned.apk');

      checkCollectionContains<String>(<String>[
        ...flutterAssets,
        'lib/arm64-v8a/libapp.so',
        'lib/arm64-v8a/libflutter.so',
        'lib/armeabi-v7a/libapp.so',
        'lib/armeabi-v7a/libflutter.so',
      ], await getFilesInApk(demoProdApk));

      return TaskResult.success(null);
    } on TaskResult catch (taskResult) {
      return taskResult;
    } catch (e) {
      return TaskResult.failure(e.toString());
    } finally {
      rmTree(tempDir);
    }
  });
}
