// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:vm_service/vm_service.dart';

import '../integration.shard/test_data/basic_project.dart';
import '../integration.shard/test_driver.dart';
import '../integration.shard/test_utils.dart';
import '../src/common.dart';

void main() {
  late Directory tempDir;
  final BasicProjectWithUnaryMain project = BasicProjectWithUnaryMain();
  late FlutterRunTestDriver flutter;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterRunTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.stop();
    tryToDelete(tempDir);
  });

  Future<void> start({bool verbose = false}) async {
    // The non-test project has a loop around its breakpoints.
    // No need to start paused as all breakpoint would be eventually reached.
    await flutter.run(
      withDebugger: true,
      chrome: true,
      additionalCommandArgs: <String>[if (verbose) '--verbose'],
    );
  }

  Future<void> evaluate() async {
    final ObjRef res = await flutter.evaluate('package:characters/characters.dart', 'true');
    expect(res, isA<InstanceRef>().having((InstanceRef o) => o.kind, 'kind', 'Bool'));
  }

  testWithoutContext('flutter run outputs info messages from dwds in verbose mode', () async {
    final Future<dynamic> info = expectLater(
      flutter.stdout,
      emitsThrough(contains('Loaded debug metadata')),
    );
    await start(verbose: true);
    await evaluate();
    await flutter.stop();
    await info;
  });
}
