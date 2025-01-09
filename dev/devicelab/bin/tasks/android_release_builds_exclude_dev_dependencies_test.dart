// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

Future<void> main() async {
  await task(() async {
    try {
      await runProjectTest((FlutterProject flutterProject) async {
        // Create dev_dependency plugin to use for test.
        final Directory tempDir = Directory.systemTemp.createTempSync(
          'android_release_builds_exclude_dev_dependencies_test.',
        );
        const String devDependencyPluginOrg = 'com.example.dev_dependency_plugin';
        await FlutterPluginProject.create(
          tempDir,
          'dev_dependency_plugin',
          options: <String>['--platforms=android', '--org=$devDependencyPluginOrg'],
        );

        // Add devDependencyPlugin as dependency of flutterProject.
        await flutterProject.addPlugin(
          'dev_dependency_plugin',
          options: <String>['--path', path.join(tempDir.path, 'dev_dependency_plugin')],
        );

        final List<String> buildModesToTest = <String>['debug', 'profile', 'release'];
        for (final String buildMode in buildModesToTest) {
          section('APK does contain methods from dev dependency in $buildMode mode');

          // Build APK in buildMode and check that devDependencyPlugin is included/excluded in the APK as expected.
          await inDirectory(flutterProject.rootPath, () async {
            await flutter(
              'build',
              options: <String>['apk', '--$buildMode', '--target-platform=android-arm'],
            );

            final File apk = File(
              path.join(
                flutterProject.rootPath,
                'build',
                'app',
                'outputs',
                'flutter-apk',
                'app-debug.apk',
              ),
            );
            if (!apk.existsSync()) {
              throw TaskResult.failure("Expected ${apk.path} to exist, but it doesn't.");
            }

            // We expect the APK to include the devDependencyPlugin except in release mode.
            final bool isTestingReleaseMode = buildMode == 'release';
            final bool apkIncludesDevDependency = await checkApkContainsMethodsFromLibrary(
              apk,
              devDependencyPluginOrg,
            );
            final bool apkIncludesDevDependencyAsExpected =
                isTestingReleaseMode ? !apkIncludesDevDependency : apkIncludesDevDependency;
            if (!apkIncludesDevDependencyAsExpected) {
              return TaskResult.failure(
                'Expected to${isTestingReleaseMode ? ' not' : ''} find dev_dependency_plugin in APK built with debug mode but did${isTestingReleaseMode ? '' : ' not'}.',
              );
            }
          });
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
