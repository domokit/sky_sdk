// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/deferred_components_prebuild_validator.dart';
import 'package:flutter_tools/src/android/deferred_components_validator.dart';
import 'package:flutter_tools/src/base/deferred_component.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';

import '../../src/common.dart';
import '../../src/context.dart';

void main() {
  late FileSystem fileSystem;
  late BufferLogger logger;
  late Directory projectDir;
  late Platform platform;
  late Directory flutterRootDir;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    logger = BufferLogger.test();
    projectDir = fileSystem.directory('/project');
    flutterRootDir = fileSystem.directory('/flutter_root');
    platform = FakePlatform();
  });

  testWithoutContext('No checks passes', () async {
    final DeferredComponentsPrebuildValidator validator = DeferredComponentsPrebuildValidator(
      projectDir,
      logger,
      platform,
      exitOnFail: false,
      title: 'test check',
    );
    validator.displayResults();
    validator.attemptToolExit();
    expect(logger.statusText, 'test check passed.\n');
  });

  testWithoutContext('clearTempDir passes', () async {
    final DeferredComponentsPrebuildValidator validator = DeferredComponentsPrebuildValidator(
      projectDir,
      logger,
      platform,
      exitOnFail: false,
      title: 'test check',
    );
    validator.displayResults();
    validator.attemptToolExit();

    expect(logger.statusText, 'test check passed.\n');
  });

  testUsingContext(
    'androidComponentSetup build.gradle does not exist',
    () async {
      final Directory templatesDir = flutterRootDir.childDirectory('templates');
      final Directory deferredComponentDir = templatesDir
          .childDirectory('module')
          .childDirectory('android')
          .childDirectory('deferred_component');
      final File buildGradleTemplate = deferredComponentDir.childFile('build.gradle.tmpl');
      final File androidManifestTemplate = deferredComponentDir
          .childDirectory('src')
          .childDirectory('main')
          .childFile('AndroidManifest.xml.tmpl');

      deferredComponentDir.createSync(recursive: true);
      buildGradleTemplate.createSync(recursive: true);
      androidManifestTemplate.createSync(recursive: true);
      buildGradleTemplate.writeAsStringSync(
        'fake build.gradle template {{componentName}}',
        flush: true,
        mode: FileMode.append,
      );
      androidManifestTemplate.writeAsStringSync(
        'fake AndroidManifest.xml template {{componentName}}',
        flush: true,
        mode: FileMode.append,
      );

      final DeferredComponentsPrebuildValidator validator = DeferredComponentsPrebuildValidator(
        projectDir,
        logger,
        platform,
        exitOnFail: false,
        title: 'test check',
        templatesDir: templatesDir,
      );
      final Directory componentDir = projectDir
          .childDirectory('android')
          .childDirectory('component1');
      final File file = componentDir
          .childDirectory('src')
          .childDirectory('main')
          .childFile('AndroidManifest.xml');
      if (file.existsSync()) {
        file.deleteSync();
      }
      file.createSync(recursive: true);
      await validator.checkAndroidDynamicFeature(<DeferredComponent>[
        DeferredComponent(name: 'component1'),
      ]);
      validator.displayResults();
      validator.attemptToolExit();

      file.deleteSync();
      expect(logger.statusText.contains('Newly generated android files:\n'), true);
      expect(
        logger.statusText.contains(
          'build/${DeferredComponentsValidator.kDeferredComponentsTempDirectory}/component1/build.gradle\n',
        ),
        true,
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'androidComponentSetup AndroidManifest.xml does not exist',
    () async {
      final Directory templatesDir = flutterRootDir.childDirectory('templates');
      final Directory deferredComponentDir = templatesDir
          .childDirectory('module')
          .childDirectory('android')
          .childDirectory('deferred_component');
      final File buildGradleTemplate = deferredComponentDir.childFile('build.gradle.tmpl');
      final File androidManifestTemplate = deferredComponentDir
          .childDirectory('src')
          .childDirectory('main')
          .childFile('AndroidManifest.xml.tmpl');

      deferredComponentDir.createSync(recursive: true);
      buildGradleTemplate.createSync(recursive: true);
      androidManifestTemplate.createSync(recursive: true);
      buildGradleTemplate.writeAsStringSync(
        'fake build.gradle template {{componentName}}',
        flush: true,
        mode: FileMode.append,
      );
      androidManifestTemplate.writeAsStringSync(
        'fake AndroidManifest.xml template {{componentName}}',
        flush: true,
        mode: FileMode.append,
      );

      final DeferredComponentsPrebuildValidator validator = DeferredComponentsPrebuildValidator(
        projectDir,
        logger,
        platform,
        exitOnFail: false,
        title: 'test check',
        templatesDir: templatesDir,
      );
      final Directory componentDir = projectDir
          .childDirectory('android')
          .childDirectory('component1');
      final File file = componentDir.childFile('build.gradle');
      if (file.existsSync()) {
        file.deleteSync();
      }
      file.createSync(recursive: true);
      await validator.checkAndroidDynamicFeature(<DeferredComponent>[
        DeferredComponent(name: 'component1'),
      ]);
      validator.displayResults();
      validator.attemptToolExit();

      file.deleteSync();
      expect(logger.statusText.contains('Newly generated android files:\n'), true);
      expect(
        logger.statusText.contains(
          'build/${DeferredComponentsValidator.kDeferredComponentsTempDirectory}/component1/src/main/AndroidManifest.xml\n',
        ),
        true,
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext('androidComponentSetup all files exist passes', () async {
    final Directory templatesDir = flutterRootDir
        .childDirectory('templates')
        .childDirectory('deferred_component');
    final File buildGradleTemplate = templatesDir.childFile('build.gradle.tmpl');
    final File androidManifestTemplate = templatesDir
        .childDirectory('src')
        .childDirectory('main')
        .childFile('AndroidManifest.xml.tmpl');
    if (templatesDir.existsSync()) {
      templatesDir.deleteSync(recursive: true);
    }
    buildGradleTemplate.createSync(recursive: true);
    androidManifestTemplate.createSync(recursive: true);
    buildGradleTemplate.writeAsStringSync(
      'fake build.gradle template {{componentName}}',
      flush: true,
      mode: FileMode.append,
    );
    androidManifestTemplate.writeAsStringSync(
      'fake AndroidManifest.xml template {{componentName}}',
      flush: true,
      mode: FileMode.append,
    );

    final DeferredComponentsPrebuildValidator validator = DeferredComponentsPrebuildValidator(
      projectDir,
      logger,
      platform,
      exitOnFail: false,
      title: 'test check',
      templatesDir: templatesDir,
    );
    final Directory componentDir = projectDir
        .childDirectory('android')
        .childDirectory('component1');
    final File buildGradle = componentDir.childFile('build.gradle');
    if (buildGradle.existsSync()) {
      buildGradle.deleteSync();
    }
    buildGradle.createSync(recursive: true);
    final File manifest = componentDir
        .childDirectory('src')
        .childDirectory('main')
        .childFile('AndroidManifest.xml');
    if (manifest.existsSync()) {
      manifest.deleteSync();
    }
    manifest.createSync(recursive: true);
    await validator.checkAndroidDynamicFeature(<DeferredComponent>[
      DeferredComponent(name: 'component1'),
    ]);
    validator.displayResults();
    validator.attemptToolExit();

    manifest.deleteSync();
    buildGradle.deleteSync();
    expect(logger.statusText, 'test check passed.\n');
  });

  testWithoutContext('androidStringMapping creates new file', () async {
    final DeferredComponentsPrebuildValidator validator = DeferredComponentsPrebuildValidator(
      projectDir,
      logger,
      platform,
      exitOnFail: false,
      title: 'test check',
    );
    final Directory baseModuleDir = projectDir.childDirectory('android').childDirectory('app');
    final File stringRes = baseModuleDir
        .childDirectory('src')
        .childDirectory('main')
        .childDirectory('res')
        .childDirectory('values')
        .childFile('strings.xml');
    if (stringRes.existsSync()) {
      stringRes.deleteSync();
    }
    final File manifest = baseModuleDir
        .childDirectory('src')
        .childDirectory('main')
        .childFile('AndroidManifest.xml');
    if (manifest.existsSync()) {
      manifest.deleteSync();
    }
    manifest.createSync(recursive: true);
    manifest.writeAsStringSync(
      '''
<manifest xmlns:android="http://schemas.android.com/apk/res/android"
    package="com.example.splitaot">
    <application
        android:name="io.flutter.app.FlutterPlayStoreSplitApplication"
        android:label="splitaot"
        android:extractNativeLibs="false">
        <activity
            android:name=".MainActivity"
            android:launchMode="singleTop"
            android:windowSoftInputMode="adjustResize">
        </activity>
        <!-- Don't delete the meta-data below.
             This is used by the Flutter tool to generate GeneratedPluginRegistrant.java -->
        <meta-data
            android:name="flutterEmbedding"
            android:value="2" />
        <meta-data
            android:name="io.flutter.embedding.engine.deferredcomponents.DeferredComponentManager.loadingUnitMapping"
            android:value="invalidmapping" />
    </application>
</manifest>
''',
      flush: true,
      mode: FileMode.append,
    );
    validator.checkAndroidResourcesStrings(<DeferredComponent>[
      DeferredComponent(name: 'component1', libraries: <String>['lib2']),
      DeferredComponent(name: 'component2', libraries: <String>['lib1', 'lib4']),
    ]);
    validator.displayResults();
    validator.attemptToolExit();

    expect(logger.statusText.contains('Modified android files:\n'), false);
    expect(logger.statusText.contains('Newly generated android files:\n'), true);
    expect(
      logger.statusText.contains(
        'build/${DeferredComponentsValidator.kDeferredComponentsTempDirectory}/app/src/main/res/values/strings.xml\n',
      ),
      true,
    );

    final File stringsOutput = projectDir
        .childDirectory('build')
        .childDirectory(DeferredComponentsValidator.kDeferredComponentsTempDirectory)
        .childDirectory('app')
        .childDirectory('src')
        .childDirectory('main')
        .childDirectory('res')
        .childDirectory('values')
        .childFile('strings.xml');
    expect(stringsOutput.existsSync(), true);
    expect(
      stringsOutput.readAsStringSync().contains(
        '<string name="component1Name">component1</string>',
      ),
      true,
    );
    expect(
      stringsOutput.readAsStringSync().contains(
        '<string name="component2Name">component2</string>',
      ),
      true,
    );
  });

  testWithoutContext('androidStringMapping modifies strings file', () async {
    final DeferredComponentsPrebuildValidator validator = DeferredComponentsPrebuildValidator(
      projectDir,
      logger,
      platform,
      exitOnFail: false,
      title: 'test check',
    );
    final Directory baseModuleDir = projectDir.childDirectory('android').childDirectory('app');
    final File stringRes = baseModuleDir
        .childDirectory('src')
        .childDirectory('main')
        .childDirectory('res')
        .childDirectory('values')
        .childFile('strings.xml');
    if (stringRes.existsSync()) {
      stringRes.deleteSync();
    }
    stringRes.createSync(recursive: true);
    stringRes.writeAsStringSync(
      '''
<?xml version="1.0" encoding="utf-8"?>
<resources>
    <string name="component1Name">component1</string>
</resources>

''',
      flush: true,
      mode: FileMode.append,
    );
    validator.checkAndroidResourcesStrings(<DeferredComponent>[
      DeferredComponent(name: 'component1', libraries: <String>['lib2']),
      DeferredComponent(name: 'component2', libraries: <String>['lib1', 'lib4']),
    ]);
    validator.displayResults();
    validator.attemptToolExit();

    expect(logger.statusText.contains('Newly generated android files:\n'), false);
    expect(logger.statusText.contains('Modified android files:\n'), true);
    expect(
      logger.statusText.contains(
        'build/${DeferredComponentsValidator.kDeferredComponentsTempDirectory}/app/src/main/res/values/strings.xml\n',
      ),
      true,
    );

    final File stringsOutput = projectDir
        .childDirectory('build')
        .childDirectory(DeferredComponentsValidator.kDeferredComponentsTempDirectory)
        .childDirectory('app')
        .childDirectory('src')
        .childDirectory('main')
        .childDirectory('res')
        .childDirectory('values')
        .childFile('strings.xml');
    expect(stringsOutput.existsSync(), true);
    expect(
      stringsOutput.readAsStringSync().contains(
        '<string name="component1Name">component1</string>',
      ),
      true,
    );
    expect(
      stringsOutput.readAsStringSync().contains(
        '<string name="component2Name">component2</string>',
      ),
      true,
    );
  });
}
