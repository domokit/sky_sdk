// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'dart:convert';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/features.dart';

import '../src/common.dart';
import 'test_data/basic_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

// TODO(matanlurey): Remove this test; https://github.com/flutter/flutter/issues/102983.
//
// This is a legacy test that verifies that the old (package:flutter_gen) synthetic package
// works end-to-end. It's sister test, which verifies the supported in-package source
// generation, is flutter_tools/test/integration.shard/gen_l10n_test.dart, which tests the
// same workflow.
void main() {
  late Directory tempDir;
  final BasicProjectWithFlutterGen project = BasicProjectWithFlutterGen();
  late FlutterRunTestDriver flutter;

  setUpAll(() async {
    // Disable the --explicit-package-dependencies flag *if* it is on by default.
    if (!explicitPackageDependencies.master.enabledByDefault) {
      return;
    }
    final ProcessResult result = processManager.runSync(<String>[
      flutterBin,
      'config',
      '--no-explicit-package-dependencies',
    ]);
    expect(result, const ProcessResultMatcher());
  });

  tearDownAll(() async {
    // Enable the --explicit-package-dependencies flag *if* it is on by default.
    if (!explicitPackageDependencies.master.enabledByDefault) {
      return;
    }
    final ProcessResult result = processManager.runSync(<String>[
      flutterBin,
      'config',
      '--explicit-package-dependencies',
    ]);
    expect(result, const ProcessResultMatcher());
  });

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterRunTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.stop();
    tryToDelete(tempDir);
  });

  testWithoutContext('can correctly reference flutter generated code.', () async {
    await flutter.run();
    final dynamic jsonContent = json.decode(
      project.dir.childDirectory('.dart_tool').childFile('package_config.json').readAsStringSync(),
    );
    final Map<String, dynamic> collection =
        ((jsonContent as Map<String, dynamic>)['packages'] as Iterable<dynamic>).firstWhere(
              (dynamic entry) => (entry as Map<String, dynamic>)['name'] == 'collection',
            )
            as Map<String, dynamic>;
    expect(
      Uri.parse(collection['rootUri'] as String).isAbsolute,
      isTrue,
      reason: 'The generated package_config.json should use absolute root urls',
    );
    expect(
      collection['packageUri'] as String,
      'lib/',
      reason: 'The generated package_config.json should have package urls ending with /',
    );
  });
}
