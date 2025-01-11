// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

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
/// SHARD=flutter_engine_android_vulkan bin/cache/dart-sdk/bin/dart dev/bots/test.dart
///
/// # Or
///
/// SHARD=flutter_engine_android_openlges bin/cache/dart-sdk/bin/dart dev/bots/test.dart
/// ```
///
/// For debugging, you need to instead run and launch these tests
/// individually _in_ the `dev/integration_tests/android_engine_test` directory.
/// Comparisons against goldens cant happen locally.
Future<void> runFlutterEngineAndroidTests({required String impellerBackend}) async {
  print('Running Flutter Android Engine (Impeller Backend: $impellerBackend) tests...');

  final String androidEngineTestPath = path.join('dev', 'integration_tests', 'android_engine_test');
  final io.File androidManifestXml = io.File(
    path.join(androidEngineTestPath, 'android', 'app', 'src', 'main', 'AndroidManifest.xml'),
  );

  // Make a copy of the manifest to restore.
  final String originalManifest = await androidManifestXml.readAsString();
  final RegExp findImpellerBackend = RegExp(r'ImpellerBackend"\sandroid:value="(.*)"');

  // Replace the backend string.
  final String replacedManifest = originalManifest.replaceAllMapped(findImpellerBackend, (Match m) {
    return 'ImpellerBackend" android:value="$impellerBackend"';
  });
  await androidManifestXml.writeAsString(replacedManifest);

  final RegExp impellerBackendNotice = RegExp(r'Using the Impeller rendering backend (\(.*\))');

  try {
    final List<FileSystemEntity> mains = Glob('$androidEngineTestPath/lib/**_main.dart').listSync();
    for (final FileSystemEntity file in mains) {
      final CommandResult result = await runCommand('flutter', <String>[
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

      // Check that the application actually ran with the requested backend.
      final String? stdout = result.flattenedStdout;
      if (stdout == null) {
        io.stderr.writeln(result.flattenedStderr);
        throw StateError('No stdout received from flutter CLI.');
      }
      final Match? match = impellerBackendNotice.firstMatch(stdout);
      if (match == null) {
        throw StateError('Expected an Impeller run notice, but none found');
      }
      final String backend = match.group(1)!.toLowerCase();
      if (backend != impellerBackend) {
        throw StateError('Requested "$impellerBackend", got "$backend".');
      }
    }
  } finally {
    await androidManifestXml.writeAsString(originalManifest);
  }
}
