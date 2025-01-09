// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file/local.dart';

import 'apk_utils.dart';
import 'task_result.dart';
import 'utils.dart';

// The following test outline shares a lot of similarities with
// the one in packages/flutter_tools/test/src/android_common.dart. When making
// changes here, consider making the corresponding changes to that file as well.

/// The template settings.gradle content, with AGP and Kotlin versions replaced
/// by an easily find/replaceable string.
const String gradleSettingsFileContent = r'''
pluginManagement {
    val flutterSdkPath = run {
        val properties = java.util.Properties()
        file("local.properties").inputStream().use { properties.load(it) }
        val flutterSdkPath = properties.getProperty("flutter.sdk")
        require(flutterSdkPath != null) { "flutter.sdk not set in local.properties" }
        flutterSdkPath
    }

    includeBuild("$flutterSdkPath/packages/flutter_tools/gradle")

    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

plugins {
    id("dev.flutter.flutter-plugin-loader") version "1.0.0"
    id("com.android.application") version "AGP_REPLACE_ME" apply false
    id("org.jetbrains.kotlin.android") version "KGP_REPLACE_ME" apply false
}

include(":app")

''';

const String agpReplacementString = 'AGP_REPLACE_ME';
const String kgpReplacementString = 'KGP_REPLACE_ME';

/// The template gradle-wrapper.properties content, with the Gradle version replaced
/// by an easily find/replaceable string.
const String gradleWrapperPropertiesFileContent = r'''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
distributionUrl=https\://services.gradle.org/distributions/gradle-GRADLE_REPLACE_ME-all.zip

''';

const String gradleReplacementString = 'GRADLE_REPLACE_ME';
final RegExp flutterCompileSdkString = RegExp(r'flutter\.compileSdkVersion|flutter\.compileSdk');

/// A simple class containing a Kotlin, Gradle, and AGP version.
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

/// For each [VersionTuple] in versionTuples:
/// 1. Calls `flutter create`
/// 2. Replaces the template AGP, Gradle, and Kotlin versions with those in the
///    tuple.
/// 3. Calls `flutter build apk`.
/// Returns a failed task result if any of the `create` or `build apk` calls
/// fails, returns a successful result otherwise. Cleans up in either case.
Future<TaskResult> buildFlutterApkWithSpecifiedDependencyVersions({
  required List<VersionTuple> versionTuples,
  required Directory tempDir,
  required LocalFileSystem localFileSystem,
}) async {
  for (final VersionTuple versions in versionTuples) {
    final Directory innerTempDir = tempDir.createTempSync(versions.gradleVersion);
    try {
      // Create a new flutter project.
      section('Create new app with dependency versions: $versions');
      await flutter(
        'create',
        options: <String>['dependency_checker_app', '--platforms=android'],
        workingDirectory: innerTempDir.path,
      );

      final String appPath = '${innerTempDir.absolute.path}/dependency_checker_app';

      if (versions.compileSdkVersion != null) {
        final File appGradleBuild = getAndroidBuildFile(
          localFileSystem.path.join(appPath, 'android', 'app'),
        );
        final String appBuildContent = appGradleBuild.readAsStringSync().replaceFirst(
          flutterCompileSdkString,
          versions.compileSdkVersion!,
        );
        appGradleBuild.writeAsStringSync(appBuildContent);
      }

      // Modify gradle version to passed in version.
      final File gradleWrapperProperties = localFileSystem.file(
        localFileSystem.path.join(
          appPath,
          'android',
          'gradle',
          'wrapper',
          'gradle-wrapper.properties',
        ),
      );
      final String propertyContent = gradleWrapperPropertiesFileContent.replaceFirst(
        gradleReplacementString,
        versions.gradleVersion,
      );
      await gradleWrapperProperties.writeAsString(propertyContent, flush: true);

      final File gradleSettingsFile = getAndroidBuildFile(
        localFileSystem.path.join(appPath, 'android'),
        settings: true,
      );
      final String settingsContent = gradleSettingsFileContent
          .replaceFirst(agpReplacementString, versions.agpVersion)
          .replaceFirst(kgpReplacementString, versions.kotlinVersion);
      await gradleSettingsFile.writeAsString(settingsContent, flush: true);

      // Ensure that gradle files exists from templates.
      section(
        "Ensure 'flutter build apk' succeeds with Gradle ${versions.gradleVersion}, AGP ${versions.agpVersion}, and Kotlin ${versions.kotlinVersion}",
      );
      await flutter('build', options: <String>['apk', '--debug'], workingDirectory: appPath);
    } catch (e) {
      tempDir.deleteSync(recursive: true);
      return TaskResult.failure(
        'Failed to build app with Gradle ${versions.gradleVersion}, AGP ${versions.agpVersion}, and Kotlin ${versions.kotlinVersion}, error was:\n$e',
      );
    }
  }
  tempDir.deleteSync(recursive: true);
  return TaskResult.success(null);
}
