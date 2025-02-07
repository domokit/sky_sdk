// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';

import '../../src/common.dart';
import '../test_driver.dart';
import '../test_utils.dart';
import 'hot_reload_with_asset.dart';

void testAll({bool chrome = false, List<String> additionalCommandArgs = const <String>[]}) {
  group('chrome: $chrome'
      '${additionalCommandArgs.isEmpty ? '' : ' with args: $additionalCommandArgs'}', () {
    late Directory tempDir;
    final HotReloadWithAssetProject project = HotReloadWithAssetProject();
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

    testWithoutContext('hot reload does not need to sync assets on the first reload', () async {
      final Completer<void> onFirstLoad = Completer<void>();
      final Completer<void> onSecondLoad = Completer<void>();

      flutter.stdout.listen((String line) {
        // If the asset fails to load, this message will be printed instead.
        // this indicates that the devFS was not able to locate the asset
        // after the hot reload.
        if (line.contains('FAILED TO LOAD')) {
          fail('Did not load asset: $line');
        }
        if (line.contains('LOADED DATA')) {
          onFirstLoad.complete();
        }
        if (line.contains('SECOND DATA')) {
          onSecondLoad.complete();
        }
      });
      flutter.stdout.listen(printOnFailure);
      await flutter.run(chrome: chrome, additionalCommandArgs: additionalCommandArgs);
      await onFirstLoad.future;

      project.uncommentHotReloadPrint();
      await flutter.hotReload();
      await onSecondLoad.future;
    });

    testWithoutContext('hot restart does not need to sync assets on the first reload', () async {
      final Completer<void> onFirstLoad = Completer<void>();
      final Completer<void> onSecondLoad = Completer<void>();

      flutter.stdout.listen((String line) {
        // If the asset fails to load, this message will be printed instead.
        // this indicates that the devFS was not able to locate the asset
        // after the hot reload.
        if (line.contains('FAILED TO LOAD')) {
          fail('Did not load asset: $line');
        }
        if (line.contains('LOADED DATA')) {
          onFirstLoad.complete();
        }
        if (line.contains('SECOND DATA')) {
          onSecondLoad.complete();
        }
      });
      flutter.stdout.listen(printOnFailure);
      await flutter.run(chrome: chrome, additionalCommandArgs: additionalCommandArgs);
      await onFirstLoad.future;

      project.uncommentHotReloadPrint();
      await flutter.hotRestart();
      await onSecondLoad.future;
    });
  });
}
