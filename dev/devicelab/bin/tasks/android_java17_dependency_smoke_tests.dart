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
// Note that compileSdk 35 requires AGP 8.1.0+, so override to compileSdk 34 for AGP 8.0.
List<VersionTuple> versionTuples = <VersionTuple>[
  VersionTuple(
    agpVersion: '8.0.0',
    gradleVersion: '8.0',
    kotlinVersion: '1.8.22',
    compileSdkVersion: '34',
  ),
  VersionTuple(agpVersion: '8.1.0', gradleVersion: '8.0', kotlinVersion: '1.8.22'),
  VersionTuple(agpVersion: '8.2.0', gradleVersion: '8.2', kotlinVersion: '1.8.22'),
  VersionTuple(agpVersion: '8.3.0', gradleVersion: '8.4', kotlinVersion: '1.8.22'),
  VersionTuple(agpVersion: '8.4.0', gradleVersion: '8.6', kotlinVersion: '1.8.22'),
  VersionTuple(agpVersion: '8.5.0', gradleVersion: '8.7', kotlinVersion: '1.8.22'),
  VersionTuple(agpVersion: '8.6.0', gradleVersion: '8.7', kotlinVersion: '1.8.22'),
  VersionTuple(agpVersion: '8.7.0', gradleVersion: '8.9', kotlinVersion: '1.8.22'),
];

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
