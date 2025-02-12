// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';

import '../../integration.shard/test_data/hot_reload_project.dart';
import '../../integration.shard/test_driver.dart';
import '../../integration.shard/test_utils.dart';
import '../../src/common.dart';

import 'hot_reload_index_html_samples.dart';

void main() async {
  await testAll(useDDCLibraryBundleFormat: false);
}

Future<void> testAll({required bool useDDCLibraryBundleFormat}) async {
  await _testProject(
    HotReloadProject(),
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  ); // default
  await _testProject(
    HotReloadProject(constApp: true),
    name: 'Default) (with `const MyApp()`)',
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  ); // runApp(const MyApp());
  await _testProject(
    HotReloadProject(indexHtml: indexHtmlFlutterJsCallback),
    name: 'flutter.js (callback)',
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  );
  await _testProject(
    HotReloadProject(indexHtml: indexHtmlFlutterJsPromisesFull),
    name: 'flutter.js (promises)',
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  );
  await _testProject(
    HotReloadProject(indexHtml: indexHtmlFlutterJsPromisesShort),
    name: 'flutter.js (promises, short)',
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  );
  await _testProject(
    HotReloadProject(indexHtml: indexHtmlFlutterJsLoad),
    name: 'flutter.js (load)',
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  );
  await _testProject(
    HotReloadProject(indexHtml: indexHtmlNoFlutterJs),
    name: 'No flutter.js',
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  );
  await _testProject(
    HotReloadProject(indexHtml: indexHtmlWithFlutterBootstrapScriptTag),
    name: 'Using flutter_bootstrap.js script tag',
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  );
  await _testProject(
    HotReloadProject(indexHtml: indexHtmlWithInlinedFlutterBootstrapScript),
    name: 'Using inlined flutter_bootstrap.js',
    useDDCLibraryBundleFormat: useDDCLibraryBundleFormat,
  );
}

Future<void> _testProject(
  HotReloadProject project, {
  String name = 'Default',
  required bool useDDCLibraryBundleFormat,
}) async {
  late Directory tempDir;
  late FlutterRunTestDriver flutter;

  final List<String> additionalCommandArgs =
      useDDCLibraryBundleFormat ? <String>['--web-experimental-hot-reload'] : <String>[];
  final String testName =
      'Hot restart (index.html: $name)'
      '${additionalCommandArgs.isEmpty ? '' : ' with args: $additionalCommandArgs'}';

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('hot_restart_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterRunTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.stop();
    await flutter.done;
    tryToDelete(tempDir);
  });

  testWithoutContext('$testName: hot restart works without error', () async {
    flutter.stdout.listen(printOnFailure);
    await flutter.run(
      chrome: true,
      additionalCommandArgs: <String>['--verbose', ...additionalCommandArgs],
    );
    await flutter.hotRestart();
  });

  testWithoutContext(
    '$testName: newly added code executes during hot restart - canvaskit',
    () async {
      final Completer<void> completer = Completer<void>();
      final StreamSubscription<String> subscription = flutter.stdout.listen((String line) {
        printOnFailure(line);
        if (line.contains('(((((RELOAD WORKED)))))')) {
          completer.complete();
        }
      });
      await flutter.run(
        chrome: true,
        additionalCommandArgs: <String>['--verbose', ...additionalCommandArgs],
      );
      project.uncommentHotReloadPrint();
      try {
        await flutter.hotRestart();
        await completer.future.timeout(const Duration(seconds: 15));
      } finally {
        await subscription.cancel();
      }
    },
    // Skipped for https://github.com/flutter/flutter/issues/110879.
    skip: true,
  );
}
