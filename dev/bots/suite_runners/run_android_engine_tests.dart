// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/local.dart';
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
/// SHARD=android_engine_vulkan_tests UPDATE_GOLDENS=1 bin/cache/dart-sdk/bin/dart dev/bots/test.dart
/// ```
///
/// 4. Then, re-run the command against the baseline images:
///
/// ```sh
/// SHARD=android_engine_vulkan_tests bin/cache/dart-sdk/bin/dart dev/bots/test.dart
/// ```
///
/// If you are trying to debug a commit, you will want to run step (3) first,
/// then apply the commit (or flag), and then run step (4). If you are trying
/// to determine flakiness in the *same* state, or want better debugging, see
/// `dev/integration_tests/android_engine_test/README.md`.
Future<void> runAndroidEngineTests({required ImpellerBackend impellerBackend}) async {
  print('Running Flutter Driver Android tests (backend=$impellerBackend)');

  final String androidEngineTestPath = path.join('dev', 'integration_tests', 'android_engine_test');
  final List<FileSystemEntity> mains = Glob('$androidEngineTestPath/lib/**_main.dart').listSync();

  final File androidManifestXml = const LocalFileSystem().file(
    path.join(androidEngineTestPath, 'android', 'app', 'src', 'main', 'AndroidManifest.xml'),
  );
  final String androidManifestContents = androidManifestXml.readAsStringSync();

  try {
    // Replace whatever the current backend is with the specified backend.
    final RegExp impellerBackendMetadata = RegExp(_impellerBackendMetadata(value: '.*'));
    androidManifestXml.writeAsStringSync(
      androidManifestContents.replaceFirst(
        impellerBackendMetadata,
        _impellerBackendMetadata(value: impellerBackend.name),
      ),
    );

    // Stdout will produce: "Using the Impeller rendering backend (.*)"
    // TODO(matanlurey): Enable once `flutter drive` retains error logs.
    // final RegExp impellerStdoutPattern = RegExp('Using the Imepller rendering backend (.*)');

    Future<void> runTest(FileSystemEntity file) async {
      final CommandResult result = await runCommand(
        'flutter',
        <String>[
          'drive',
          path.relative(file.path, from: androidEngineTestPath),
          // There are no reason to enable development flags for this test.
          // Disable them to work around flakiness issues, and in general just
          // make less things start up unnecessarily.
          '--no-dds',
          '--no-enable-dart-profiling',
          '--test-arguments=test',
          '--test-arguments=--reporter=expanded',
        ],
        workingDirectory: androidEngineTestPath,
        environment: <String, String>{'ANDROID_ENGINE_TEST_GOLDEN_VARIANT': impellerBackend.name},
      );
      final String? stdout = result.flattenedStdout;
      if (stdout == null) {
        foundError(<String>['No stdout produced.']);
        return;
      }

      // TODO(matanlurey): Enable once `flutter drive` retains error logs.
      // https://github.com/flutter/flutter/issues/162087.
      //
      // final Match? stdoutMatch = impellerStdoutPattern.firstMatch(stdout);
      // if (stdoutMatch == null) {
      //   foundError(<String>['Could not find pattern ${impellerStdoutPattern.pattern}.', stdout]);
      //   return;
      // }

      // final String reportedBackend = stdoutMatch.group(1)!.toLowerCase();
      // if (reportedBackend != impellerBackend.name) {
      //   foundError(<String>[
      //     'Reported Imepller backend was $reportedBackend, expected ${impellerBackend.name}',
      //   ]);
      //   return;
      // }
    }

    for (final FileSystemEntity file in mains) {
      if (file.path.contains('hcpp')) {
        continue;
      }
      await runTest(file);
    }

    // Test HCPP Platform Views on Vulkan.
    if (impellerBackend == ImpellerBackend.vulkan) {
      androidManifestXml.writeAsStringSync(
        androidManifestXml.readAsStringSync().replaceFirst(
          kSurfaceControlMetadataDisabled,
          kSurfaceControlMetadataEnabled,
        ),
      );
      for (final FileSystemEntity file in mains) {
        if (!file.path.contains('hcpp')) {
          continue;
        }
        await runTest(file);
      }
    }
  } finally {
    // Restore original contents.
    androidManifestXml.writeAsStringSync(androidManifestContents);
  }
}

const String kSurfaceControlMetadataDisabled =
    '<meta-data android:name="io.flutter.embedding.android.EnableSurfaceControl" android:value="false" />';
const String kSurfaceControlMetadataEnabled =
    '<meta-data android:name="io.flutter.embedding.android.EnableSurfaceControl" android:value="true" />';

String _impellerBackendMetadata({required String value}) {
  return '<meta-data android:name="io.flutter.embedding.android.ImpellerBackend" android:value="$value" />';
}

enum ImpellerBackend { vulkan, opengles }
