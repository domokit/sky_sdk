// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

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
    tempDir = createResolvedTempDirectorySync('flutter_plugin_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  test('error logged when plugin Android ndkVersion higher than project', () async {
    // Create dummy plugin
    processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'create',
      '--template=plugin_ffi',
      '--platforms=android',
      'test_plugin',
    ], workingDirectory: tempDir.path);

    final Directory pluginAppDir = tempDir.childDirectory('test_plugin');
    final File pluginGradleFile = pluginAppDir.childDirectory('android').childFile('build.gradle');
    expect(pluginGradleFile, exists);

    final String pluginBuildGradle = pluginGradleFile.readAsStringSync();

    // Bump up plugin ndkVersion to 21.4.7075529.
    final RegExp androidNdkVersionRegExp = RegExp(
      r'ndkVersion = (\"[0-9\.]+\"|flutter.ndkVersion|android.ndkVersion)',
    );
    final String newPluginGradleFile = pluginBuildGradle.replaceAll(
      androidNdkVersionRegExp,
      'ndkVersion = "21.4.7075529"',
    );
    expect(newPluginGradleFile, contains('21.4.7075529'));
    pluginGradleFile.writeAsStringSync(newPluginGradleFile);

    final Directory pluginExampleAppDir = pluginAppDir.childDirectory('example');

    final File projectGradleFile = pluginExampleAppDir
        .childDirectory('android')
        .childDirectory('app')
        .childFile('build.gradle.kts');
    expect(projectGradleFile, exists);

    final String projectBuildGradle = projectGradleFile.readAsStringSync();

    // Bump down plugin example app ndkVersion to 21.1.6352462.
    final String newProjectGradleFile = projectBuildGradle.replaceAll(
      androidNdkVersionRegExp,
      'ndkVersion = "21.1.6352462"',
    );
    expect(newProjectGradleFile, contains('21.1.6352462'));
    projectGradleFile.writeAsStringSync(newProjectGradleFile);

    // Run flutter build apk to build plugin example project
    final ProcessResult result = processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'apk',
      '--target-platform=android-arm',
    ], workingDirectory: pluginExampleAppDir.path);

    // Check that an error message is thrown.
    expect(
      result.stderr,
      contains('''
Your project is configured with Android NDK 21.1.6352462, but the following plugin(s) depend on a different Android NDK version:
- test_plugin requires Android NDK 21.4.7075529
Fix this issue by using the highest Android NDK version (they are backward compatible).
Add the following to ${projectGradleFile.path}:

    android {
        ndkVersion = "21.4.7075529"
        ...
    }
'''),
    );
  });
}
