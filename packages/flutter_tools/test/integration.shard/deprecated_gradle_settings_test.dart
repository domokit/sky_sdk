// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

/// Tests that apps can be built using the deprecated `android/settings.gradle` file.
/// This test should be removed once apps have been migrated to this new file.
// TODO(egarciad): Migrate existing files, https://github.com/flutter/flutter/issues/54566
void main() {
  test('android project using deprecated settings.gradle will still build', () async {
    final String workingDirectory = fileSystem.path.join(
      getFlutterRoot(),
      'dev',
      'integration_tests',
      'gradle_deprecated_settings',
    );

    final File settingsDotGradleFile = fileSystem.file(
      fileSystem.path.join(workingDirectory, 'android', 'settings.gradle'),
    );
    const String expectedSettingsDotGradle = r"""
// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is the `settings.gradle` file that apps were created with until Flutter
// v1.22.0. This file has changed, so it must be migrated in existing projects.

include ':app'

def flutterProjectRoot = rootProject.projectDir.parentFile.toPath()
def localPropertiesFile = new File(rootProject.projectDir, "local.properties")
def properties = new Properties()

def plugins = new Properties()
def pluginsFile = new File(flutterProjectRoot.toFile(), '.flutter-plugins')
if (pluginsFile.exists()) {
    pluginsFile.withReader('UTF-8') { reader -> plugins.load(reader) }
}

plugins.each { name, path ->
    def pluginDirectory = flutterProjectRoot.resolve(path).resolve('android').toFile()
    include ":$name"
    project(":$name").projectDir = pluginDirectory
}
""";

    expect(
      settingsDotGradleFile.readAsStringSync().trim().replaceAll('\r', ''),
      equals(expectedSettingsDotGradle.trim()),
    );

    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      'build',
      'apk',
      '--debug',
      '--target-platform',
      'android-arm',
      '--verbose',
    ], workingDirectory: workingDirectory);

    expect(result, const ProcessResultMatcher());

    final String apkPath = fileSystem.path.join(
      workingDirectory,
      'build',
      'app',
      'outputs',
      'flutter-apk',
      'app-debug.apk',
    );
    expect(fileSystem.file(apkPath), exists);
  });
}
