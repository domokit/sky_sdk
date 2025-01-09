// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'package:file/file.dart';
import 'package:file_testing/file_testing.dart';

import '../src/common.dart';

import 'test_data/test_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('flutter_coverage_collection_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext('Can collect coverage in machine mode', () async {
    final TestProject project = TestProject();
    await project.setUpIn(tempDir);
    final FlutterTestTestDriver flutter = FlutterTestTestDriver(tempDir);
    await flutter.test(coverage: true);
    await flutter.done;

    final File lcovFile = tempDir.childDirectory('coverage').childFile('lcov.info');

    expect(lcovFile, exists);
    expect(
      lcovFile.readAsStringSync(),
      contains('main.dart'),
    ); // either 'SF:lib/main.dart or SF:lib\\main.dart
  });
}
