// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file/src/interface/file_system_entity.dart';

import '../integration.shard/test_utils.dart';
import '../src/android_common.dart';
import '../src/common.dart';
import '../src/context.dart';

// This test requires Java 11 due to the intentionally low version of Gradle.
// It also overrides the template compile sdk version to 34, because using 35
// requires AGP 8.0+.
void main() {
  late Directory tempDir;

  setUpAll(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
  });

  tearDownAll(() async {
    tryToDelete(tempDir as FileSystemEntity);
  });

  testUsingContext('AGP version out of "warn" support band but in "error" band builds '
      'successfully and prints warning', () async {
    final VersionTuple versionTuple = VersionTuple(
      agpVersion: '7.3.0',
      gradleVersion: '7.5',
      kotlinVersion: '1.7.10',
      compileSdkVersion: '34',
    );
    final ProcessResult result = await buildFlutterApkWithSpecifiedDependencyVersions(
      versions: versionTuple,
      tempDir: tempDir,
    );
    expect(result, const ProcessResultMatcher());
    expect(result.stderr, contains('Please upgrade your Android Gradle Plugin version'));
  });

  testUsingContext('Gradle version out of "warn" support band but in "error" band builds '
      'successfully and prints warning', () async {
    // Create a new flutter project.
    final VersionTuple versionTuple = VersionTuple(
      agpVersion: '7.3.0',
      gradleVersion: '7.4.1',
      kotlinVersion: '1.7.10',
      compileSdkVersion: '34',
    );
    final ProcessResult result = await buildFlutterApkWithSpecifiedDependencyVersions(
      versions: versionTuple,
      tempDir: tempDir,
    );
    expect(result, const ProcessResultMatcher());
    expect(result.stderr, contains('Please upgrade your Gradle version'));
  });

  testUsingContext('Kotlin version out of "warn" support band but in "error" band builds '
      'successfully and prints warning', () async {
    final VersionTuple versionTuple = VersionTuple(
      agpVersion: '7.4.0',
      gradleVersion: '7.5',
      kotlinVersion: '1.8.0',
      compileSdkVersion: '34',
    );
    final ProcessResult result = await buildFlutterApkWithSpecifiedDependencyVersions(
      versions: versionTuple,
      tempDir: tempDir,
    );

    expect(result, const ProcessResultMatcher());
    expect(result.stderr, contains('Please upgrade your Kotlin version'));
  });
}
