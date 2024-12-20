// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:path/path.dart' as path;

import '../utils.dart';

Future<void> androidPreviewIntegrationToolTestsRunner() async {
  final String toolsPath = path.join(flutterRoot, 'packages', 'flutter_tools');

  final List<String> allTests =
      Directory(path.join(toolsPath, 'test', 'android_preview_integration.shard'))
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
