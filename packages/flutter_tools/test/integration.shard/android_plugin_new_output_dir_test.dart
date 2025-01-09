// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/cache.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() {
    Cache.flutterRoot = getFlutterRoot();
    tempDir = createResolvedTempDirectorySync('android_plugin_new_output_dir_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  test('plugins use individualized build directories based on their name.', () async {
    // create flutter module project
    ProcessResult result = processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'create',
      '--template=module',
      'flutter_project',
    ], workingDirectory: tempDir.path);

    final String projectPath = fileSystem.path.join(tempDir.path, 'flutter_project');

    final File modulePubspec = fileSystem.file(fileSystem.path.join(projectPath, 'pubspec.yaml'));
    String pubspecContent = modulePubspec.readAsStringSync();
    pubspecContent = pubspecContent.replaceFirst('dependencies:', '''
dependencies:
  image_picker: 0.8.5+3''');
    modulePubspec.writeAsStringSync(pubspecContent);

    // Run flutter build apk to build module example project
    result = processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'aar',
      '--target-platform=android-arm',
    ], workingDirectory: projectPath);

    log(result.exitCode);

    // Check outputDir existed
    final Directory outputDir = fileSystem.directory(
      fileSystem.path.join(projectPath, '.android', 'plugins_build_output', 'image_picker_android'),
    );
    expect(outputDir, exists);
  });
}
