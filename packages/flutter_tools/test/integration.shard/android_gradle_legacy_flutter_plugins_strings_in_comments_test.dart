// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This test can be removed once https://github.com/flutter/flutter/issues/155484 is resolved.

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

  test(
    'should build Android app with commented-out ".flutter-plugins" in settings.gradle',
    () async {
      // Create Android app project instead of plugin
      processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'create',
        '--template=app',
        'test_android_app',
      ], workingDirectory: tempDir.path);

      final Directory appDir = tempDir.childDirectory('test_android_app');
      final Directory androidDir = appDir.childDirectory('android');

      // Create settings.gradle with commented .flutter-plugins
      final File settingsGradle = androidDir.childFile('settings.gradle');
      settingsGradle.writeAsStringSync(r'''
// The following block uses '.flutter-plugins' but is commented out.
/*
def plugins = new Properties()
def pluginsFile = new File(flutterProjectRoot.toFile(), '.flutter-plugins')
if (pluginsFile.exists()) {
    pluginsFile.withReader('UTF-8') { reader -> plugins.load(reader) }
}
*/

pluginManagement {
    def flutterSdkPath = {
        def properties = new Properties()
        file("local.properties").withInputStream { properties.load(it) }
        def flutterSdkPath = properties.getProperty("flutter.sdk")
        assert flutterSdkPath != null, "flutter.sdk not set in local.properties"
        return flutterSdkPath
    }()

    includeBuild("$flutterSdkPath/packages/flutter_tools/gradle")

    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

plugins {
    id "dev.flutter.flutter-plugin-loader" version "1.0.0"
    id "com.android.application" version "8.1.0" apply false
    id "org.jetbrains.kotlin.android" version "1.8.22" apply false
}

include ":app"
''');

      // Run flutter build apk with release mode
      final ProcessResult buildApkResult = await processManager.run(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'build',
        'apk',
        '--release',
      ], workingDirectory: appDir.path);

      // Build should succeed and not throw any errors about settings.gradle
      expect(buildApkResult.stderr.toString(), isNot(contains('Please fix your settings.gradle')));
      expect(buildApkResult, const ProcessResultMatcher());
    },
  );
}
