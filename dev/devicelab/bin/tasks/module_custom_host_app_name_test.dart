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
final String fileReadWriteMode = Platform.isWindows ? 'rw-rw-rw-' : 'rw-r--r--';

/// Tests that the Flutter module project template works and supports
/// adding Flutter to an existing Android app.
Future<void> main() async {
  await task(() async {
    section('Find Java');

    final String? javaHome = await findJavaHome();
    if (javaHome == null) {
      return TaskResult.failure('Could not find Java');
    }
    print('\nUsing JAVA_HOME=$javaHome');

    section('Create Flutter module project');

    final Directory tempDir = Directory.systemTemp.createTempSync('flutter_module_test.');
    final Directory projectDir = Directory(path.join(tempDir.path, 'hello'));
    try {
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>['--org', 'io.flutter.devicelab', '--template=module', 'hello'],
        );
      });

      section('Add read-only asset');

      final File readonlyTxtAssetFile = await File(
        path.join(projectDir.path, 'assets', 'read-only.txt'),
      ).create(recursive: true);

      if (!exists(readonlyTxtAssetFile)) {
        return TaskResult.failure('Failed to create read-only asset');
      }

      if (!Platform.isWindows) {
        await exec('chmod', <String>['444', readonlyTxtAssetFile.path]);
      }

      final File pubspec = File(path.join(projectDir.path, 'pubspec.yaml'));
      String content = await pubspec.readAsString();
      content = content.replaceFirst(
        '${Platform.lineTerminator}  # assets:${Platform.lineTerminator}',
        '${Platform.lineTerminator}  assets:${Platform.lineTerminator}    - assets/read-only.txt${Platform.lineTerminator}',
      );
      await pubspec.writeAsString(content, flush: true);

      section('Add plugins');

      content = await pubspec.readAsString();
      content = content.replaceFirst(
        '${Platform.lineTerminator}dependencies:${Platform.lineTerminator}',
        '${Platform.lineTerminator}dependencies:${Platform.lineTerminator}',
      );
      await pubspec.writeAsString(content, flush: true);
      await inDirectory(projectDir, () async {
        await flutter('packages', options: <String>['get']);
      });

      section('Build Flutter module library archive');

      await inDirectory(Directory(path.join(projectDir.path, '.android')), () async {
        await exec(
          gradlewExecutable,
          <String>['flutter:assembleDebug'],
          environment: <String, String>{'JAVA_HOME': javaHome},
        );
      });

      final bool aarBuilt = exists(
        File(
          path.join(
            projectDir.path,
            '.android',
            'Flutter',
            'build',
            'outputs',
            'aar',
            'flutter-debug.aar',
          ),
        ),
      );

      if (!aarBuilt) {
        return TaskResult.failure('Failed to build .aar');
      }

      section('Build ephemeral host app');

      await inDirectory(projectDir, () async {
        await flutter('build', options: <String>['apk']);
      });

      final bool ephemeralHostApkBuilt = exists(
        File(
          path.join(
            projectDir.path,
            'build',
            'host',
            'outputs',
            'apk',
            'release',
            'app-release.apk',
          ),
        ),
      );

      if (!ephemeralHostApkBuilt) {
        return TaskResult.failure('Failed to build ephemeral host .apk');
      }

      section('Clean build');

      await inDirectory(projectDir, () async {
        await flutter('clean');
      });

      section('Make Android host app editable');

      await inDirectory(projectDir, () async {
        await flutter('make-host-app-editable', options: <String>['android']);
      });

      section('Build editable host app');

      await inDirectory(projectDir, () async {
        await flutter('build', options: <String>['apk']);
      });

      final bool editableHostApkBuilt = exists(
        File(
          path.join(
            projectDir.path,
            'build',
            'host',
            'outputs',
            'apk',
            'release',
            'app-release.apk',
          ),
        ),
      );

      if (!editableHostApkBuilt) {
        return TaskResult.failure('Failed to build editable host .apk');
      }

      section('Add to existing Android app');

      final Directory hostApp = Directory(path.join(tempDir.path, 'hello_host_app'));
      mkdir(hostApp);
      recursiveCopy(
        Directory(
          path.join(
            flutterDirectory.path,
            'dev',
            'integration_tests',
            'pure_android_host_apps',
            'android_custom_host_app',
          ),
        ),
        hostApp,
      );
      copy(File(path.join(projectDir.path, '.android', gradlew)), hostApp);
      copy(
        File(path.join(projectDir.path, '.android', 'gradle', 'wrapper', 'gradle-wrapper.jar')),
        Directory(path.join(hostApp.path, 'gradle', 'wrapper')),
      );

      section('Build debug host APK');

      await inDirectory(hostApp, () async {
        if (!Platform.isWindows) {
          await exec('chmod', <String>['+x', 'gradlew']);
        }
        await exec(
          gradlewExecutable,
          <String>['SampleApp:assembleDebug'],
          environment: <String, String>{
            'JAVA_HOME': javaHome,
            'FLUTTER_SUPPRESS_ANALYTICS': 'true',
          },
        );
      });

      section('Check debug APK exists');

      final String debugHostApk = path.join(
        hostApp.path,
        'SampleApp',
        'build',
        'outputs',
        'apk',
        'debug',
        'SampleApp-debug.apk',
      );
      if (!exists(File(debugHostApk))) {
        return TaskResult.failure('Failed to build debug host APK');
      }

      section('Check files in debug APK');

      checkCollectionContains<String>(<String>[
        ...flutterAssets,
        ...debugAssets,
        ...baseApkFiles,
      ], await getFilesInApk(debugHostApk));

      section('Check debug AndroidManifest.xml');

      final String androidManifestDebug = await getAndroidManifest(debugHostApk);
      if (!androidManifestDebug.contains('''
        <meta-data
            android:name="flutterProjectType"
            android:value="module" />''')) {
        return TaskResult.failure(
          "Debug host APK doesn't contain metadata: flutterProjectType = module ",
        );
      }

      section('Check file access modes for read-only asset from Flutter module');

      final String readonlyDebugAssetFilePath = path.joinAll(<String>[
        hostApp.path,
        'SampleApp',
        'build',
        'intermediates',
        'assets',
        'debug',
        'mergeDebugAssets',
        'flutter_assets',
        'assets',
        'read-only.txt',
      ]);
      final File readonlyDebugAssetFile = File(readonlyDebugAssetFilePath);
      if (!exists(readonlyDebugAssetFile)) {
        return TaskResult.failure('Failed to copy read-only asset file');
      }

      String modes = readonlyDebugAssetFile.statSync().modeString();
      print('\nread-only.txt file access modes = $modes');
      if (modes.compareTo(fileReadWriteMode) != 0) {
        return TaskResult.failure('Failed to make assets user-readable and writable');
      }

      section('Build release host APK');

      await inDirectory(hostApp, () async {
        await exec(
          gradlewExecutable,
          <String>['SampleApp:assembleRelease'],
          environment: <String, String>{
            'JAVA_HOME': javaHome,
            'FLUTTER_SUPPRESS_ANALYTICS': 'true',
          },
        );
      });

      final String releaseHostApk = path.join(
        hostApp.path,
        'SampleApp',
        'build',
        'outputs',
        'apk',
        'release',
        'SampleApp-release-unsigned.apk',
      );
      if (!exists(File(releaseHostApk))) {
        return TaskResult.failure('Failed to build release host APK');
      }

      section('Check files in release APK');

      checkCollectionContains<String>(<String>[
        ...flutterAssets,
        ...baseApkFiles,
        'lib/arm64-v8a/libapp.so',
        'lib/arm64-v8a/libflutter.so',
        'lib/armeabi-v7a/libapp.so',
        'lib/armeabi-v7a/libflutter.so',
      ], await getFilesInApk(releaseHostApk));

      section('Check release AndroidManifest.xml');

      final String androidManifestRelease = await getAndroidManifest(debugHostApk);
      if (!androidManifestRelease.contains('''
        <meta-data
            android:name="flutterProjectType"
            android:value="module" />''')) {
        return TaskResult.failure(
          "Release host APK doesn't contain metadata: flutterProjectType = module ",
        );
      }

      section('Check file access modes for read-only asset from Flutter module');

      final String readonlyReleaseAssetFilePath = path.joinAll(<String>[
        hostApp.path,
        'SampleApp',
        'build',
        'intermediates',
        'assets',
        'release',
        'mergeReleaseAssets',
        'flutter_assets',
        'assets',
        'read-only.txt',
      ]);
      final File readonlyReleaseAssetFile = File(readonlyReleaseAssetFilePath);
      if (!exists(readonlyReleaseAssetFile)) {
        return TaskResult.failure('Failed to copy read-only asset file');
      }

      modes = readonlyReleaseAssetFile.statSync().modeString();
      print('\nread-only.txt file access modes = $modes');
      if (modes.compareTo(fileReadWriteMode) != 0) {
        return TaskResult.failure('Failed to make assets user-readable and writable');
      }

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
