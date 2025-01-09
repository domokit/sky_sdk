// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

Future<void> main() async {
  await task(() async {
    section('Copy test Flutter App with watchOS Companion');

    final Directory tempDir = Directory.systemTemp.createTempSync(
      'flutter_ios_app_with_extensions_test.',
    );
    final Directory projectDir = Directory(path.join(tempDir.path, 'app_with_extensions'));
    try {
      mkdir(projectDir);
      recursiveCopy(
        Directory(
          path.join(flutterDirectory.path, 'dev', 'integration_tests', 'ios_app_with_extensions'),
        ),
        projectDir,
      );

      section('Create release build');

      // This attempts to build the companion watchOS app. However, the watchOS
      // SDK is not available in CI and therefore the build will fail.
      // Check to make sure that the tool attempts to build the companion watchOS app.
      // See https://github.com/flutter/flutter/pull/94190.
      await inDirectory(projectDir, () async {
        final String buildOutput = await evalFlutter(
          'build',
          options: <String>['ios', '--no-codesign', '--release', '--verbose'],
        );
        if (!buildOutput.contains('-destination generic/platform=watchOS')) {
          print(buildOutput);
          throw TaskResult.failure('Did not try to get watch build settings');
        }
      });

      return TaskResult.success(null);
    } catch (e) {
      return TaskResult.failure(e.toString());
    } finally {
      rmTree(tempDir);
    }
  });
}
