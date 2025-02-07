// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';

import '../../src/common.dart';
import '../test_driver.dart';
import '../test_utils.dart';
import 'hot_reload_const_project.dart';

void testAll({
  bool chrome = false,
  List<String> additionalCommandArgs = const <String>[],
  String constClassFieldRemovalErrorMessage = 'Try performing a hot restart instead.',
  Object? skip = false,
}) {
  group('chrome: $chrome'
      '${additionalCommandArgs.isEmpty ? '' : ' with args: $additionalCommandArgs'}', () {
    late Directory tempDir;
    final HotReloadConstProject project = HotReloadConstProject();
    late FlutterRunTestDriver flutter;

    setUp(() async {
      tempDir = createResolvedTempDirectorySync('hot_reload_test.');
      await project.setUpIn(tempDir);
      flutter = FlutterRunTestDriver(tempDir);
    });

    tearDown(() async {
      await flutter.stop();
      tryToDelete(tempDir);
    });

    testWithoutContext(
      'hot reload displays a formatted error message when removing a field from a const class',
      () async {
        await flutter.run();
        project.removeFieldFromConstClass();

        expect(
          flutter.hotReload(),
          throwsA(
            isA<Exception>().having(
              (Exception e) => e.toString(),
              'message',
              contains(constClassFieldRemovalErrorMessage),
            ),
          ),
        );
      },
    );

    testWithoutContext('hot restart succeeds when removing a field from a const class', () async {
      await flutter.run(chrome: true, additionalCommandArgs: additionalCommandArgs);
      project.removeFieldFromConstClass();
      await flutter.hotRestart();
    });
  });
}
