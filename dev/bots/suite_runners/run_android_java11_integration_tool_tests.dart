// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:path/path.dart' as path;

import '../utils.dart';

/// To run this test locally:
///
/// 1. Connect an Android device or emulator.
/// 2. Run `dart pub get` in dev/bots
/// 3. Set flutter to use Java 11
///    flutter config --jdk-dir=<Path to java 11>
///    On a Mac you can run `/usr/libexec/java_home -V1 to find the installed
///    versions of java. Remember to clear this value after testing.
///    `flutter config --jdk-dir=`
/// 4. Run the following command from the root of the Flutter repository:
///
/// ```sh
/// SHARD=android_java11_tool_integration_tests bin/cache/dart-sdk/bin/dart dev/bots/test.dart
/// ```
Future<void> androidJava11IntegrationToolTestsRunner() async {
  final String toolsPath = path.join(flutterRoot, 'packages', 'flutter_tools');

  final List<String> allTests =
      Directory(path.join(toolsPath, 'test', 'android_java11_integration.shard'))
          .listSync(recursive: true)
          .whereType<File>()
          .map<String>((FileSystemEntity entry) => path.relative(entry.path, from: toolsPath))
          .where((String testPath) => path.basename(testPath).endsWith('_test.dart'))
          .toList();

  await runDartTest(
    toolsPath,
    forceSingleCore: true,
    testPaths: selectIndexOfTotalSubshard<String>(allTests),
    collectMetrics: true,
  );
}
