// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file/local.dart';
import 'package:flutter_devicelab/framework/dependency_smoke_test_task_definition.dart';
import 'package:flutter_devicelab/framework/framework.dart';

// Methodology:
// - AGP: all versions within our support range (*).
// - Gradle: The version that AGP lists as the default Gradle version for that
//           AGP version under the release notes, e.g.
//           https://developer.android.com/build/releases/past-releases/agp-8-4-0-release-notes.
// - Kotlin: No methodology as of yet.
// (*) - support range defined in packages/flutter_tools/gradle/src/main/kotlin/dependency_version_checker.gradle.kts.
List<VersionTuple> versionTuples = <VersionTuple>[
  VersionTuple(
    agpVersion: '7.0.1',
    gradleVersion: '7.0.2',
    kotlinVersion: '1.7.10',
    compileSdkVersion: '34',
  ),
  VersionTuple(
    agpVersion: '7.1.0',
    gradleVersion: '7.2',
    kotlinVersion: '1.7.10',
    compileSdkVersion: '34',
  ),
  VersionTuple(
    agpVersion: '7.2.0',
    gradleVersion: '7.3.3',
    kotlinVersion: '1.7.10',
    compileSdkVersion: '34',
  ),
  VersionTuple(
    agpVersion: '7.3.0',
    gradleVersion: '7.4',
    kotlinVersion: '1.7.10',
    compileSdkVersion: '34',
  ),
  VersionTuple(
    agpVersion: '7.4.0',
    gradleVersion: '7.5',
    kotlinVersion: '1.8.10',
    compileSdkVersion: '34',
  ),
];

// This test requires a Java version less than 17 due to the intentionally low
// version of Gradle. We choose 11 because this was the primary version used in
// CI before 17, and hence it is also hosted on CIPD. It also overrides to
// compileSdkVersion 34 because compileSdk 35 requires AGP 8.0+.
// https://docs.gradle.org/current/userguide/compatibility.html
Future<void> main() async {
  /// The [FileSystem] for the integration test environment.
  const LocalFileSystem fileSystem = LocalFileSystem();

  final Directory tempDir = fileSystem.systemTempDirectory.createTempSync(
    'flutter_android_dependency_version_tests',
  );
  await task(() {
    return buildFlutterApkWithSpecifiedDependencyVersions(
      versionTuples: versionTuples,
      tempDir: tempDir,
      localFileSystem: fileSystem,
    );
  });
}
