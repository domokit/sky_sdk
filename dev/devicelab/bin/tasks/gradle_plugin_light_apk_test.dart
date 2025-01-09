// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

Future<void> main() async {
  await task(() async {
    try {
      await runPluginProjectTest((FlutterPluginProject pluginProject) async {
        section('APK content for task assembleDebug with target platform = android-arm');

        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter(
            'build',
            options: <String>['apk', '--debug', '--target-platform=android-arm'],
          );
        });

        Iterable<String> apkFiles = await getFilesInApk(pluginProject.debugApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...debugAssets,
          ...baseApkFiles,
          'lib/armeabi-v7a/libflutter.so',
        ], apkFiles);

        checkCollectionDoesNotContain<String>(<String>[
          'lib/arm64-v8a/libapp.so',
          'lib/armeabi-v7a/libapp.so',
          'lib/x86/libapp.so',
          'lib/x86_64/libapp.so',
        ], apkFiles);

        section('APK content for task assembleDebug with target platform = android-x86');
        // This is used by `flutter run`
        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter(
            'build',
            options: <String>['apk', '--debug', '--target-platform=android-x86'],
          );
        });

        apkFiles = await getFilesInApk(pluginProject.debugApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...debugAssets,
          ...baseApkFiles,
          'lib/x86/libflutter.so',
        ], apkFiles);

        checkCollectionDoesNotContain<String>(<String>[
          'lib/armeabi-v7a/libapp.so',
          'lib/x86/libapp.so',
          'lib/x86_64/libapp.so',
        ], apkFiles);

        section('APK content for task assembleDebug with target platform = android-x64');
        // This is used by `flutter run`

        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter(
            'build',
            options: <String>['apk', '--debug', '--target-platform=android-x64'],
          );
        });

        apkFiles = await getFilesInApk(pluginProject.debugApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...debugAssets,
          ...baseApkFiles,
          'lib/x86_64/libflutter.so',
        ], apkFiles);

        checkCollectionDoesNotContain<String>(<String>[
          'lib/armeabi-v7a/libapp.so',
          'lib/x86/libapp.so',
          'lib/x86_64/libapp.so',
        ], apkFiles);

        section('APK content for task assembleRelease with target platform = android-arm');

        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter(
            'build',
            options: <String>['apk', '--release', '--target-platform=android-arm'],
          );
        });

        apkFiles = await getFilesInApk(pluginProject.releaseApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...baseApkFiles,
          'lib/armeabi-v7a/libflutter.so',
          'lib/armeabi-v7a/libapp.so',
        ], apkFiles);

        checkCollectionDoesNotContain<String>(<String>[
          ...debugAssets,
          'lib/arm64-v8a/libflutter.so',
          'lib/arm64-v8a/libapp.so',
        ], apkFiles);

        section('APK content for task assembleRelease with target platform = android-arm64');

        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter(
            'build',
            options: <String>['apk', '--release', '--target-platform=android-arm64'],
          );
        });

        apkFiles = await getFilesInApk(pluginProject.releaseApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...baseApkFiles,
          'lib/arm64-v8a/libflutter.so',
          'lib/arm64-v8a/libapp.so',
        ], apkFiles);

        checkCollectionDoesNotContain<String>(<String>[
          ...debugAssets,
          'lib/armeabi-v7a/libflutter.so',
          'lib/armeabi-v7a/libapp.so',
        ], apkFiles);
      });

      await runProjectTest((FlutterProject project) async {
        section('gradlew assembleDebug');
        await inDirectory(project.rootPath, () {
          return flutter('build', options: <String>['apk', '--debug']);
        });
        final String? errorMessage = validateSnapshotDependency(project, 'kernel_blob.bin');
        if (errorMessage != null) {
          throw TaskResult.failure(errorMessage);
        }

        section('gradlew assembleProfile');
        await inDirectory(project.rootPath, () {
          return flutter('build', options: <String>['apk', '--profile']);
        });

        section('gradlew assembleLocal (custom debug build)');
        await project.addCustomBuildType('local', initWith: 'debug');
        await project.runGradleTask('assembleLocal');
      });

      await runProjectTest((FlutterProject project) async {
        section('gradlew assembleLocal with plugin (custom debug build)');

        final Directory tempDir = Directory.systemTemp.createTempSync('flutter_plugin.');
        final Directory pluginDir = Directory(path.join(tempDir.path, 'plugin_under_test'));

        section('Create plugin');
        await inDirectory(tempDir, () async {
          await flutter(
            'create',
            options: <String>[
              '--org',
              'io.flutter.devicelab.plugin',
              '--template=plugin',
              '--platforms=android,ios',
              pluginDir.path,
            ],
          );
        });

        section('Configure');
        await project.addPlugin('plugin_under_test', options: <String>['--path', pluginDir.path]);
        await project.addCustomBuildType('local', initWith: 'debug');
        await project.getPackages();

        section('Build APK');
        await project.runGradleTask('assembleLocal');
      });

      await runProjectTest((FlutterProject project) async {
        section('gradlew assembleBeta (custom release build)');
        await project.addCustomBuildType('beta', initWith: 'release');
        await project.runGradleTask('assembleBeta');
      });

      await runProjectTest((FlutterProject project) async {
        section('gradlew assembleLocal (plugin with custom build type)');
        await project.addCustomBuildType('local', initWith: 'debug');
        section('Add plugin');
        await project.addPlugin('path_provider');
        await project.getPackages();

        await project.runGradleTask('assembleLocal');
      });

      await runProjectTest((FlutterProject project) async {
        section('gradlew assembleFreeDebug (product flavor)');
        await project.addProductFlavors(<String>['free']);
        await project.runGradleTask('assembleFreeDebug');
      });

      await runProjectTest((FlutterProject project) async {
        section('gradlew on build script with error');
        await project.introduceError();
        ProcessResult result = await inDirectory(project.rootPath, () {
          return executeFlutter('build', options: <String>['apk', '--release'], canFail: true);
        });

        if (result.exitCode == 0) {
          throw failure('Gradle did not exit with error as expected', result);
        }
        String output = '${result.stdout}\n${result.stderr}';
        if (output.contains('GradleException') ||
            output.contains('Failed to notify') ||
            output.contains('at org.gradle')) {
          throw failure('Gradle output should not contain stacktrace', result);
        }
        if (!output.contains('Build failed')) {
          throw failure('Gradle output should contain a readable error message', result);
        }

        section('flutter build apk on build script with error');
        await project.introduceError();
        result = await inDirectory(project.rootPath, () {
          return executeFlutter('build', options: <String>['apk', '--release'], canFail: true);
        });
        if (result.exitCode == 0) {
          throw failure('flutter build apk should fail when Gradle does', result);
        }
        output = '${result.stdout}\n${result.stderr}';
        if (!output.contains('Build failed')) {
          throw failure(
            'flutter build apk output should contain a readable Gradle error message',
            result,
          );
        }
        if (hasMultipleOccurrences(output, 'Build failed')) {
          throw failure('flutter build apk should not invoke Gradle repeatedly on error', result);
        }
      });

      await runProjectTest((FlutterProject project) async {
        section('gradlew assembleDebug forwards stderr');
        await project.introducePubspecError();
        final ProcessResult result = await inDirectory(project.rootPath, () {
          return executeFlutter('build', options: <String>['apk', '--release'], canFail: true);
        });
        if (result.exitCode == 0) {
          throw failure('Gradle did not exit with error as expected', result);
        }
        final String output = '${result.stdout}\n${result.stderr}';
        if (!output.contains(
          'No file or variants found for asset: lib/gallery/example_code.dart.',
        )) {
          throw failure(output, result);
        }
      });

      return TaskResult.success(null);
    } on TaskResult catch (taskResult) {
      return taskResult;
    } catch (e) {
      return TaskResult.failure(e.toString());
    }
  });
}
