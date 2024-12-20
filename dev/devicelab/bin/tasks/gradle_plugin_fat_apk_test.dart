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
        section('APK content for task assembleDebug without explicit target platform');
        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter('build', options: <String>['apk', '--debug', '--verbose']);
        });

        Iterable<String> apkFiles = await getFilesInApk(pluginProject.debugApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...debugAssets,
          ...baseApkFiles,
          'lib/armeabi-v7a/libflutter.so',
          'lib/arm64-v8a/libflutter.so',
          // Debug mode intentionally includes `x86` and `x86_64`.
          'lib/x86/libflutter.so',
          'lib/x86_64/libflutter.so',
        ], apkFiles);

        checkCollectionDoesNotContain<String>(<String>[
          'lib/arm64-v8a/libapp.so',
          'lib/armeabi-v7a/libapp.so',
          'lib/x86/libapp.so',
          'lib/x86_64/libapp.so',
        ], apkFiles);

        section('APK content for task assembleRelease without explicit target platform');

        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter('build', options: <String>['apk', '--release', '--verbose']);
        });

        apkFiles = await getFilesInApk(pluginProject.releaseApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...baseApkFiles,
          'lib/armeabi-v7a/libflutter.so',
          'lib/armeabi-v7a/libapp.so',
          'lib/arm64-v8a/libflutter.so',
          'lib/arm64-v8a/libapp.so',
          'lib/x86_64/libflutter.so',
          'lib/x86_64/libapp.so',
        ], apkFiles);

        checkCollectionDoesNotContain<String>(debugAssets, apkFiles);

        section(
          'APK content for task assembleRelease with target platform = android-arm, android-arm64',
        );

        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter(
            'build',
            options: <String>[
              'apk',
              '--release',
              '--verbose',
              '--target-platform=android-arm,android-arm64',
            ],
          );
        });

        apkFiles = await getFilesInApk(pluginProject.releaseApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...baseApkFiles,
          'lib/armeabi-v7a/libflutter.so',
          'lib/armeabi-v7a/libapp.so',
          'lib/arm64-v8a/libflutter.so',
          'lib/arm64-v8a/libapp.so',
        ], apkFiles);

        checkCollectionDoesNotContain<String>(debugAssets, apkFiles);

        section(
          'APK content for task assembleRelease with '
          'target platform = android-arm, android-arm64 and split per ABI',
        );

        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter(
            'build',
            options: <String>[
              'apk',
              '--release',
              '--verbose',
              '--split-per-abi',
              '--target-platform=android-arm,android-arm64',
            ],
          );
        });

        final Iterable<String> armApkFiles = await getFilesInApk(pluginProject.releaseArmApkPath);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...baseApkFiles,
          'lib/armeabi-v7a/libflutter.so',
          'lib/armeabi-v7a/libapp.so',
        ], armApkFiles);

        checkCollectionDoesNotContain<String>(debugAssets, armApkFiles);

        final Iterable<String> arm64ApkFiles = await getFilesInApk(
          pluginProject.releaseArm64ApkPath,
        );

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...baseApkFiles,
          'lib/arm64-v8a/libflutter.so',
          'lib/arm64-v8a/libapp.so',
        ], arm64ApkFiles);

        checkCollectionDoesNotContain<String>(debugAssets, arm64ApkFiles);
      });

      await runProjectTest((FlutterProject project) async {
        section('gradlew assembleRelease');

        await inDirectory(project.rootPath, () {
          return flutter('build', options: <String>['apk', '--release', '--verbose']);
        });

        // When the platform-target isn't specified, we generate the snapshots
        // for arm and arm64.
        final List<String> targetPlatforms = <String>['arm64-v8a', 'armeabi-v7a'];
        for (final String targetPlatform in targetPlatforms) {
          final String androidArmSnapshotPath = path.join(
            project.rootPath,
            'build',
            'app',
            'intermediates',
            'flutter',
            'release',
            targetPlatform,
          );

          final String sharedLibrary = path.join(androidArmSnapshotPath, 'app.so');
          if (!File(sharedLibrary).existsSync()) {
            throw TaskResult.failure("Shared library doesn't exist");
          }
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
