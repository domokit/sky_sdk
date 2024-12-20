// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/cache.dart';

import '../src/common.dart';
import 'test_data/deferred_components_config.dart';
import 'test_data/plugin_each_settings_gradle_project.dart';
import 'test_data/plugin_project.dart';
import 'test_data/project.dart';
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

  // Regression test for https://github.com/flutter/flutter/issues/97729 (#137115).
  /// Creates a project which uses a plugin, which is not supported on Android.
  /// This means it has no entry in pubspec.yaml for:
  /// flutter -> plugin -> platforms -> android
  ///
  /// [createAndroidPluginFolder] indicates that the plugin can additionally
  /// have a functioning `android` folder.
  Future<ProcessResult> testUnsupportedPlugin({
    required Project project,
    required bool createAndroidPluginFolder,
  }) async {
    // Create dummy plugin that supports iOS and optionally Android.
    processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'create',
      '--template=plugin',
      '--platforms=ios${createAndroidPluginFolder ? ',android' : ''}',
      'test_plugin',
    ], workingDirectory: tempDir.path);

    final Directory pluginAppDir = tempDir.childDirectory('test_plugin');

    final File pubspecFile = pluginAppDir.childFile('pubspec.yaml');
    String pubspecYamlSrc = pubspecFile.readAsStringSync().replaceAll('\r\n', '\n');
    if (createAndroidPluginFolder) {
      // Override pubspec to drop support for the Android implementation.
      pubspecYamlSrc = pubspecYamlSrc
          .replaceFirst(RegExp(r'name:.*\n'), 'name: test_plugin\n')
          .replaceFirst(
            '''
      android:
        package: com.example.test_plugin
        pluginClass: TestPlugin
''',
            '''
#      android:
#        package: com.example.test_plugin
#        pluginClass: TestPlugin
''',
          );

      pubspecFile.writeAsStringSync(pubspecYamlSrc);

      // Check the android directory and the build.gradle file within.
      final File pluginGradleFile = pluginAppDir
          .childDirectory('android')
          .childFile('build.gradle');
      expect(pluginGradleFile, exists);
    } else {
      expect(pubspecYamlSrc, isNot(contains('android:')));
    }

    // Create a project which includes the plugin to test against
    final Directory pluginExampleAppDir = pluginAppDir.childDirectory('example');

    await project.setUpIn(pluginExampleAppDir);

    // Run flutter build apk to build plugin example project.
    return processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'apk',
      '--debug',
      // TODO(bkonyi): remove once https://github.com/flutter/flutter/pull/158933 is resolved
      '--verbose',
    ], workingDirectory: pluginExampleAppDir.path);
  }

  test('skip plugin if it does not support the Android platform', () async {
    final Project project = PluginWithPathAndroidProject();
    final ProcessResult buildApkResult = await testUnsupportedPlugin(
      project: project,
      createAndroidPluginFolder: false,
    );
    expect(buildApkResult.stderr.toString(), isNot(contains('Please fix your settings.gradle')));
    expect(buildApkResult, const ProcessResultMatcher());
  }, skip: Platform.isWindows); // https://github.com/flutter/flutter/issues/157640

  test(
    'skip plugin with android folder if it does not support the Android platform',
    () async {
      final Project project = PluginWithPathAndroidProjectWithoutDeferred();
      final ProcessResult buildApkResult = await testUnsupportedPlugin(
        project: project,
        createAndroidPluginFolder: true,
      );
      expect(buildApkResult.stderr.toString(), isNot(contains('Please fix your settings.gradle')));
      expect(buildApkResult, const ProcessResultMatcher());

      // Regression check for https://github.com/flutter/flutter/issues/158962.
      {
        final Directory androidDir = project.dir.childDirectory('android');
        expect(
          androidDir.childFile('settings.gradle.kts'),
          exists,
          reason: 'Modern flutter create --platforms android template creates this',
        );
        expect(
          androidDir.childFile('settings.gradle'),
          isNot(exists),
          reason:
              ''
              'flutter create should have created a settings.gradle.kts file '
              'but not a settings.gradle file. Prior to the change in the PR '
              'addressing https://github.com/flutter/flutter/issues/158962 '
              'both files were created, which means that tooling picked one '
              'and not the other, which causes ambiguity for debugging test '
              'flakes.',
        );
      }
    },
    skip: Platform.isWindows, // https://github.com/flutter/flutter/issues/157640
  );

  // TODO(54566): Remove test when issue is resolved.
  /// Test project with a `settings.gradle` (PluginEach) that apps were created
  /// with until Flutter v1.22.0.
  /// It uses the `.flutter-plugins` file to load EACH plugin.
  test(
    'skip plugin if it does not support the Android platform with a _plugin.each_ settings.gradle',
    () async {
      final Project project = PluginEachWithPathAndroidProject();
      final ProcessResult buildApkResult = await testUnsupportedPlugin(
        project: project,
        createAndroidPluginFolder: false,
      );
      expect(buildApkResult.stderr.toString(), isNot(contains('Please fix your settings.gradle')));
      expect(buildApkResult, const ProcessResultMatcher());
    },
    skip: Platform.isWindows, // https://github.com/flutter/flutter/issues/157640
  );

  // TODO(54566): Remove test when issue is resolved.
  /// Test project with a `settings.gradle` (PluginEach) that apps were created
  /// with until Flutter v1.22.0.
  /// It uses the `.flutter-plugins` file to load EACH plugin.
  /// The plugin includes a functional 'android' folder.
  test(
    'skip plugin with android folder if it does not support the Android platform with a _plugin.each_ settings.gradle',
    () async {
      final Project project = PluginEachWithPathAndroidProject();
      final ProcessResult buildApkResult = await testUnsupportedPlugin(
        project: project,
        createAndroidPluginFolder: true,
      );
      expect(buildApkResult.stderr.toString(), isNot(contains('Please fix your settings.gradle')));
      expect(buildApkResult, const ProcessResultMatcher());
    },
    skip: Platform.isWindows, // https://github.com/flutter/flutter/issues/157640
  );

  // TODO(54566): Remove test when issue is resolved.
  /// Test project with a `settings.gradle` (PluginEach) that apps were created
  /// with until Flutter v1.22.0.
  /// It is compromised by removing the 'include' statement of the plugins.
  /// As the "'.flutter-plugins'" keyword is still present, the framework
  /// assumes that all plugins are included, which is not the case.
  /// Therefore it should throw an error.
  test(
    'skip plugin if it does not support the Android platform with a compromised _plugin.each_ settings.gradle',
    () async {
      final Project project = PluginCompromisedEachWithPathAndroidProject();
      final ProcessResult buildApkResult = await testUnsupportedPlugin(
        project: project,
        createAndroidPluginFolder: true,
      );
      expect(
        buildApkResult,
        const ProcessResultMatcher(stderrPattern: 'Please fix your settings.gradle'),
      );
    },
    skip: Platform.isWindows, // https://github.com/flutter/flutter/issues/157640
  );
}

const String pubspecWithPluginPath = r'''
name: test
environment:
  sdk: ^3.7.0-0
dependencies:
  flutter:
    sdk: flutter

  test_plugin:
    path: ../
''';

/// Project that load's a plugin from the specified path.
class PluginWithPathAndroidProjectWithoutDeferred extends PluginProject {
  // Intentionally omit; this test case has nothing to do with deferred
  // components and a DeferredComponentsConfig will cause duplicates of files
  // such as build.gradle{.kts}, settings.gradle{kts} and related to be
  // generated, which in turn adds ambiguity to how the tests are built and
  // executed.
  //
  // See https://github.com/flutter/flutter/issues/158962.
  @override
  DeferredComponentsConfig? get deferredComponents => null;

  @override
  String get pubspec => pubspecWithPluginPath;
}

/// Project that load's a plugin from the specified path.
class PluginWithPathAndroidProject extends PluginProject {
  @override
  String get pubspec => pubspecWithPluginPath;
}

// TODO(matanlurey): Remove class when `.flutter-plugins` is no longer emitted.
// See https://github.com/flutter/flutter/issues/48918.

/// [PluginEachSettingsGradleProject] that load's a plugin from the specified
/// path.
class PluginEachWithPathAndroidProject extends PluginEachSettingsGradleProject {
  @override
  String get pubspec => pubspecWithPluginPath;
}

// TODO(matanlurey): Remove class when `.flutter-plugins` is no longer emitted.
// See https://github.com/flutter/flutter/issues/48918.

/// [PluginCompromisedEachSettingsGradleProject] that load's a plugin from the
/// specified path.
class PluginCompromisedEachWithPathAndroidProject
    extends PluginCompromisedEachSettingsGradleProject {
  @override
  String get pubspec => pubspecWithPluginPath;
}
