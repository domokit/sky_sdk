// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';

Future<void> main() async {
  await task(() async {
    try {
      await runProjectTest((FlutterProject flutterProject) async {
        section('APK contains plugin classes');
        await flutterProject.setMinSdkVersion(21);
        await flutterProject.addPlugin('google_maps_flutter:^2.2.1');

        await inDirectory(flutterProject.rootPath, () async {
          await flutter(
            'build',
            options: <String>['apk', '--debug', '--target-platform=android-arm'],
          );
          final File apk = File(
            '${flutterProject.rootPath}/build/app/outputs/flutter-apk/app-debug.apk',
          );
          if (!apk.existsSync()) {
            throw TaskResult.failure("Expected ${apk.path} to exist, but it doesn't");
          }
          // https://github.com/flutter/flutter/issues/72185
          await checkApkContainsMethods(apk, <String>[
            'io.flutter.plugins.googlemaps.GoogleMapController void onFlutterViewAttached(android.view.View)',
            'io.flutter.plugins.googlemaps.GoogleMapController void onFlutterViewDetached()',
          ]);
        });
      });
      return TaskResult.success(null);
    } on TaskResult catch (taskResult) {
      return taskResult;
    } catch (e) {
      return TaskResult.failure(e.toString());
    }
  });
}
