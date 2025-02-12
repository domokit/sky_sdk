// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_tools/src/android/android_builder.dart';
import 'package:flutter_tools/src/base/file_system.dart' as file_system;
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';

import '../integration.shard/test_utils.dart';
import 'common.dart';

/// A fake implementation of [AndroidBuilder].
class FakeAndroidBuilder implements AndroidBuilder {
  @override
  Future<void> buildAar({
    required FlutterProject project,
    required Set<AndroidBuildInfo> androidBuildInfo,
    required String target,
    required Future<void> Function(FlutterProject, {required bool releaseMode}) generateTooling,
    String? outputDirectoryPath,
    required String buildNumber,
  }) async {}

  @override
  Future<void> buildApk({
    required FlutterProject project,
    required AndroidBuildInfo androidBuildInfo,
    required String target,
    bool configOnly = false,
  }) async {}

  @override
  Future<void> buildAab({
    required FlutterProject project,
    required AndroidBuildInfo androidBuildInfo,
    required String target,
    bool validateDeferredComponents = true,
    bool deferredComponentsEnabled = false,
    bool configOnly = false,
  }) async {}

  @override
  Future<List<String>> getBuildVariants({required FlutterProject project}) async =>
      const <String>[];

  @override
  Future<String> outputsAppLinkSettings(
    String buildVariant, {
    required FlutterProject project,
  }) async => '/';
}

/// Creates a [FlutterProject] in a directory named [flutter_project]
/// within [directoryOverride].
class FakeFlutterProjectFactory extends FlutterProjectFactory {
  FakeFlutterProjectFactory(this.directoryOverride)
    : super(fileSystem: globals.fs, logger: globals.logger);

  final file_system.Directory directoryOverride;

  @override
  FlutterProject fromDirectory(Directory _) {
    projects.clear();
    return super.fromDirectory(directoryOverride.childDirectory('flutter_project'));
  }
}

// The following test outline shares a lot of similarities with the one in
// dev/devicelab/lib/framework/dependency_smoke_test_task_definition.dart
// When making changes here, consider making the corresponding changes to that
// file as well.

const String gradleSettingsFileContent = r'''
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
    id "com.android.application" version "AGP_REPLACE_ME" apply false
    id "org.jetbrains.kotlin.android" version "KGP_REPLACE_ME" apply false
}

include ":app"

''';

const String agpReplacementString = 'AGP_REPLACE_ME';
const String kgpReplacementString = 'KGP_REPLACE_ME';

const String gradleWrapperPropertiesFileContent = r'''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
distributionUrl=https\://services.gradle.org/distributions/gradle-GRADLE_REPLACE_ME-all.zip

''';

const String gradleReplacementString = 'GRADLE_REPLACE_ME';
const String flutterCompileSdkString = 'flutter.compileSdkVersion';

class VersionTuple {
  VersionTuple({
    required this.agpVersion,
    required this.gradleVersion,
    required this.kotlinVersion,
    this.compileSdkVersion,
  });

  String agpVersion;
  String gradleVersion;
  String kotlinVersion;
  String? compileSdkVersion;

  @override
  String toString() {
    return '(AGP version: $agpVersion, Gradle version: $gradleVersion, Kotlin version: $kotlinVersion'
        '${(compileSdkVersion == null) ? '' : ', compileSdk version: $compileSdkVersion)'}';
  }
}

/// Creates a new Flutter project with the specified AGP, Gradle, and Kotlin
/// versions and then tries to call `flutter build apk`, returning the
/// ProcessResult.
Future<ProcessResult> buildFlutterApkWithSpecifiedDependencyVersions({
  required VersionTuple versions,
  required Directory tempDir,
}) async {
  // Create a new flutter project.
  final String flutterBin = fileSystem.path.join(getFlutterRoot(), 'bin', 'flutter');
  ProcessResult result = await processManager.run(<String>[
    flutterBin,
    'create',
    'dependency_checker_app',
    '--platforms=android',
  ], workingDirectory: tempDir.path);
  expect(result, const ProcessResultMatcher());

  final Directory app = Directory(fileSystem.path.join(tempDir.path, 'dependency_checker_app'));

  if (versions.compileSdkVersion != null) {
    final File appGradleBuild = File(
      fileSystem.path.join(app.path, 'android', 'app', 'build.gradle.kts'),
    );
    final String appBuildContent = appGradleBuild.readAsStringSync().replaceFirst(
      flutterCompileSdkString,
      versions.compileSdkVersion!,
    );
    appGradleBuild.writeAsStringSync(appBuildContent);
  }

  // Modify gradle version to passed in version.
  final File gradleWrapperProperties = File(
    fileSystem.path.join(app.path, 'android', 'gradle', 'wrapper', 'gradle-wrapper.properties'),
  );
  final String propertyContent = gradleWrapperPropertiesFileContent.replaceFirst(
    gradleReplacementString,
    versions.gradleVersion,
  );
  await gradleWrapperProperties.writeAsString(propertyContent, flush: true);

  final File gradleSettings = File(fileSystem.path.join(app.path, 'android', 'settings.gradle'));
  final String settingsContent = gradleSettingsFileContent
      .replaceFirst(agpReplacementString, versions.agpVersion)
      .replaceFirst(kgpReplacementString, versions.kotlinVersion);
  await gradleSettings.writeAsString(settingsContent, flush: true);

  // Ensure that gradle files exists from templates.
  result = await processManager.run(<String>[
    flutterBin,
    'build',
    'apk',
    '--debug',
  ], workingDirectory: app.path);
  return result;
}
