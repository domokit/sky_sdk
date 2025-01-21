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
/// # Generate a baseline of local golden files.
/// SHARD=android_engine_tests UPDATE_GOLDENS=1 bin/cache/dart-sdk/bin/dart dev/bots/test.dart
/// ```
///
/// 4. Then, re-run the command against the baseline images:
///
/// ```sh
/// SHARD=android_engine_tests bin/cache/dart-sdk/bin/dart dev/bots/test.dart
/// ```
///
/// If you are trying to debug a commit, you will want to run step (3) first,
/// then apply the commit (or flag), and then run step (4). If you are trying
/// to determine flakiness in the *same* state, or want better debugging, see
/// `dev/integration_tests/android_engine_test/README.md`.
Future<void> runAndroidEngineTests() async {
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
