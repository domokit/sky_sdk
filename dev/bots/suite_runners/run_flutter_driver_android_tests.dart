// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:glob/glob.dart';
import 'package:glob/list_local_fs.dart';
import 'package:path/path.dart' as path;

import '../run_command.dart';
import '../utils.dart';

/// To run this test locally:
///
/// 1. Connect an Android device or emulator.
/// 2. Run `dart pub get` in dev/bots
/// 3. Run the following command from the root of the Flutter repository:
///
/// ```sh
/// SHARD=flutter_driver_android bin/cache/dart-sdk/bin/dart dev/bots/test.dart
/// ```
///
/// For debugging, you need to instead run and launch these tests
/// individually _in_ the `dev/integration_tests/android_engine_test` directory.
/// Comparisons against goldens cant happen locally.
Future<void> runFlutterDriverAndroidTests() async {
  print('Running Flutter Driver Android tests...');

  final String androidEngineTestPath = path.join('dev', 'integration_tests', 'android_engine_test');
  final List<FileSystemEntity> mains = Glob('$androidEngineTestPath/lib/**_main.dart').listSync();
  for (final FileSystemEntity file in mains) {
    await runCommand('flutter', <String>[
      'drive',
      path.relative(file.path, from: androidEngineTestPath),
      // There are no reason to enable development flags for this test.
      // Disable them to work around flakiness issues, and in general just
      // make less things start up unnecessarily.
      '--no-dds',
      '--no-enable-dart-profiling',
      '--test-arguments=test',
      '--test-arguments=--reporter=expanded',
    ], workingDirectory: androidEngineTestPath);
  }
}
