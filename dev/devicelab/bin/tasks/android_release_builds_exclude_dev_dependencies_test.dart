// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart' as utils;
import 'package:path/path.dart' as path;

Future<void> main() async {
  await task(() async {
    try {
      await runProjectTest((FlutterProject flutterProject) async {
        utils.section(
          'Configure plugins to be marked as dev dependencies in .flutter-plugins-dependencies file',
        );

        // Enable plugins being marked as dev dependncies in the .flutter-plugins-dependencies file.
        await utils.flutter('config', options: <String>['--explicit-package-dependencies']);

        // Create dev_dependency plugin to use for test.
        final Directory tempDir = Directory.systemTemp.createTempSync(
          'android_release_builds_exclude_dev_dependencies_test.',
        );
        const String devDependencyPluginOrg = 'com.example.dev_dependency_plugin';

        utils.section('Create plugin dev_dependency_plugin that supports Android');

        await FlutterPluginProject.create(
          tempDir,
          'dev_dependency_plugin',
          options: <String>['--platforms=android', '--org=$devDependencyPluginOrg'],
        );

        utils.section('Add dev_dependency_plugin as a dev dependency to the Flutter app project');

        // Add devDependencyPlugin as dependency of flutterProject.
        await flutterProject.addPlugin(
          'dev:dev_dependency_plugin',
          options: <String>['--path', path.join(tempDir.path, 'dev_dependency_plugin')],
        );

        utils.section(
          'Verify the app includes/excludes dev_dependency_plugin as dependency in each build mode as expected',
        );
        final List<String> buildModesToTest = <String>['debug', 'profile', 'release'];
        for (final String buildMode in buildModesToTest) {
          final String gradlew = Platform.isWindows ? 'gradlew.bat' : 'gradlew';
          final String gradlewExecutable = Platform.isWindows ? '.\\$gradlew' : './$gradlew';
          final RegExp regExpToMatchDevDependencyPlugin = RegExp(
            r'--- project :dev_dependency_plugin',
          );
          final RegExp regExpToMatchDevDependencyPluginWithTransitiveDependencies = RegExp(
            r'--- project :dev_dependency_plugin\n(\s)*\+--- org.jetbrains.kotlin.*\s\(\*\)\n(\s)*\\---\sio.flutter:flutter_embedding_' +
                buildMode,
          );
          const String stringToMatchFlutterEmbedding = '+--- io.flutter:flutter_embedding_release:';
          final bool isTestingReleaseMode = buildMode == 'release';

          utils.section('Query the dependencies of the app built with $buildMode');

          final String appDependencies = await utils.eval(gradlewExecutable, <String>[
            'app:dependencies',
            '--configuration',
            '${buildMode}RuntimeClasspath',
          ], workingDirectory: flutterProject.androidPath);

          if (isTestingReleaseMode) {
            utils.section(
              'Check that the release build includes Flutter embedding as a direct dependency',
            );

            if (!appDependencies.contains(stringToMatchFlutterEmbedding)) {
              // We expect dev_dependency_plugin to not be included in the dev dependency, but the Flutter
              // embedding should still be a dependency of the app project (regardless of the fact
              // that the app does not depend on any plugins that support Android, which would cause the
              // Flutter embedding to be included as a transitive dependency).
              throw TaskResult.failure(
                'Expected to find the Flutter embedding as a dependency of the release app build, but did not.',
              );
            }
          }

          utils.section(
            'Check that the $buildMode build includes/excludes the dev dependency plugin as expected',
          );

          // Ensure that release builds have no reference to the dev dependency plugin and make sure
          // that it is included with expected transitive dependencies for debug, profile builds.
          final bool appIncludesDevDependencyAsExpected =
              isTestingReleaseMode
                  ? !appDependencies.contains(regExpToMatchDevDependencyPlugin)
                  : appDependencies.contains(
                    regExpToMatchDevDependencyPluginWithTransitiveDependencies,
                  );
          if (!appIncludesDevDependencyAsExpected) {
            throw TaskResult.failure(
              'Expected to${isTestingReleaseMode ? ' not' : ''} find dev_dependency_plugin as a dependency of the app built in $buildMode mode but did${isTestingReleaseMode ? '' : ' not'}.',
            );
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
