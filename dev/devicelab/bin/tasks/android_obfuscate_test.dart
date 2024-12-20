// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

Future<void> main() async {
  await task(() async {
    try {
      bool foundApkProjectName = false;
      await runProjectTest((FlutterProject flutterProject) async {
        section('APK content for task assembleRelease with --obfuscate');
        await inDirectory(flutterProject.rootPath, () async {
          await flutter(
            'build',
            options: <String>[
              'apk',
              '--target-platform=android-arm',
              '--obfuscate',
              '--split-debug-info=foo/',
              '--verbose',
            ],
          );
        });
        final String outputApkDirectory = path.join(
          flutterProject.rootPath,
          'build/app/outputs/apk/release/app-release.apk',
        );
        final Iterable<String> apkFiles = await getFilesInApk(outputApkDirectory);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          ...baseApkFiles,
          'lib/armeabi-v7a/libapp.so',
        ], apkFiles);

        // Verify that an identifier from the Dart project code is not present
        // in the compiled binary.
        await inDirectory(flutterProject.rootPath, () async {
          await exec('unzip', <String>[outputApkDirectory]);
          checkFileExists(path.join(flutterProject.rootPath, 'lib/armeabi-v7a/libapp.so'));
          final String response = await eval('grep', <String>[
            flutterProject.name,
            'lib/armeabi-v7a/libapp.so',
          ], canFail: true);
          if (response.trim().contains('matches')) {
            foundApkProjectName = true;
          }
        });
      });

      bool foundAarProjectName = false;
      await runModuleProjectTest((FlutterModuleProject flutterProject) async {
        section('AAR content with --obfuscate');

        await inDirectory(flutterProject.rootPath, () async {
          await flutter(
            'build',
            options: <String>[
              'aar',
              '--target-platform=android-arm',
              '--obfuscate',
              '--split-debug-info=foo/',
              '--no-debug',
              '--no-profile',
              '--verbose',
            ],
          );
        });

        final String outputAarDirectory = path.join(
          flutterProject.rootPath,
          'build/host/outputs/repo/com/example/${flutterProject.name}/flutter_release/1.0/flutter_release-1.0.aar',
        );
        final Iterable<String> aarFiles = await getFilesInAar(outputAarDirectory);

        checkCollectionContains<String>(<String>[
          ...flutterAssets,
          'jni/armeabi-v7a/libapp.so',
        ], aarFiles);

        // Verify that an identifier from the Dart project code is not present
        // in the compiled binary.
        await inDirectory(flutterProject.rootPath, () async {
          await exec('unzip', <String>[outputAarDirectory]);
          checkFileExists(path.join(flutterProject.rootPath, 'jni/armeabi-v7a/libapp.so'));
          final String response = await eval('grep', <String>[
            flutterProject.name,
            'jni/armeabi-v7a/libapp.so',
          ], canFail: true);
          if (response.trim().contains('matches')) {
            foundAarProjectName = true;
          }
        });
      });

      if (foundApkProjectName) {
        return TaskResult.failure('Found project name in obfuscated APK dart library');
      }
      if (foundAarProjectName) {
        return TaskResult.failure('Found project name in obfuscated AAR dart library');
      }

      return TaskResult.success(null);
    } on TaskResult catch (taskResult) {
      return taskResult;
    } catch (e) {
      return TaskResult.failure(e.toString());
    }
  });
}
