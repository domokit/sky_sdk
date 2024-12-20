// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'package:archive/archive.dart';
import 'package:file/file.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_data/deferred_components_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;
  late FlutterRunTestDriver flutter;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
    flutter = FlutterRunTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.stop();
    tryToDelete(tempDir);
  });

  testWithoutContext('simple build appbundle android-arm64 target succeeds', () async {
    final DeferredComponentsProject project = DeferredComponentsProject(
      BasicDeferredComponentsConfig(),
    );
    await project.setUpIn(tempDir);
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'appbundle',
      '--target-platform=android-arm64',
    ], workingDirectory: tempDir.path);

    expect(result, const ProcessResultMatcher(stdoutPattern: 'app-release.aab'));
    expect(result.stdout.toString(), contains('Deferred components prebuild validation passed.'));
    expect(
      result.stdout.toString(),
      contains('Deferred components gen_snapshot validation passed.'),
    );

    final String line = result.stdout
        .toString()
        .split('\n')
        .firstWhere((String line) => line.contains('app-release.aab'));

    final String outputFilePath = line.split(' ')[2].trim();
    final File outputFile = fileSystem.file(fileSystem.path.join(tempDir.path, outputFilePath));
    expect(outputFile, exists);

    final Archive archive = ZipDecoder().decodeBytes(outputFile.readAsBytesSync());

    expect(archive.findFile('base/lib/arm64-v8a/libapp.so') != null, true);
    expect(archive.findFile('base/lib/arm64-v8a/libflutter.so') != null, true);
    expect(archive.findFile('component1/lib/arm64-v8a/libapp.so-2.part.so') != null, true);

    expect(
      archive.findFile('component1/assets/flutter_assets/test_assets/asset2.txt') != null,
      true,
    );
    expect(archive.findFile('base/assets/flutter_assets/test_assets/asset1.txt') != null, true);
  });

  testWithoutContext('simple build appbundle all targets succeeds', () async {
    final DeferredComponentsProject project = DeferredComponentsProject(
      BasicDeferredComponentsConfig(),
    );
    await project.setUpIn(tempDir);
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'appbundle',
    ], workingDirectory: tempDir.path);

    printOnFailure('stdout:\n${result.stdout}');
    printOnFailure('stderr:\n${result.stderr}');
    expect(result.stdout.toString(), contains('app-release.aab'));
    expect(result.stdout.toString(), contains('Deferred components prebuild validation passed.'));
    expect(
      result.stdout.toString(),
      contains('Deferred components gen_snapshot validation passed.'),
    );

    final String line = result.stdout
        .toString()
        .split('\n')
        .firstWhere((String line) => line.contains('app-release.aab'));

    final String outputFilePath = line.split(' ')[2].trim();
    final File outputFile = fileSystem.file(fileSystem.path.join(tempDir.path, outputFilePath));
    expect(outputFile, exists);

    final Archive archive = ZipDecoder().decodeBytes(outputFile.readAsBytesSync());

    expect(archive.findFile('base/lib/arm64-v8a/libapp.so') != null, true);
    expect(archive.findFile('base/lib/arm64-v8a/libflutter.so') != null, true);
    expect(archive.findFile('component1/lib/arm64-v8a/libapp.so-2.part.so') != null, true);

    expect(archive.findFile('base/lib/armeabi-v7a/libapp.so') != null, true);
    expect(archive.findFile('base/lib/armeabi-v7a/libflutter.so') != null, true);
    expect(archive.findFile('component1/lib/armeabi-v7a/libapp.so-2.part.so') != null, true);

    expect(archive.findFile('base/lib/x86_64/libapp.so') != null, true);
    expect(archive.findFile('base/lib/x86_64/libflutter.so') != null, true);
    expect(archive.findFile('component1/lib/x86_64/libapp.so-2.part.so') != null, true);

    expect(
      archive.findFile('component1/assets/flutter_assets/test_assets/asset2.txt') != null,
      true,
    );
    expect(archive.findFile('base/assets/flutter_assets/test_assets/asset1.txt') != null, true);

    expect(result, const ProcessResultMatcher());
  });

  testWithoutContext('simple build appbundle no-deferred-components succeeds', () async {
    final DeferredComponentsProject project = DeferredComponentsProject(
      BasicDeferredComponentsConfig(),
    );
    await project.setUpIn(tempDir);
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'appbundle',
      '--no-deferred-components',
    ], workingDirectory: tempDir.path);

    expect(result, const ProcessResultMatcher(stdoutPattern: 'app-release.aab'));
    expect(
      result.stdout.toString(),
      isNot(contains('Deferred components prebuild validation passed.')),
    );
    expect(
      result.stdout.toString(),
      isNot(contains('Deferred components gen_snapshot validation passed.')),
    );

    final String line = result.stdout
        .toString()
        .split('\n')
        .firstWhere((String line) => line.contains('app-release.aab'));

    final String outputFilePath = line.split(' ')[2].trim();
    final File outputFile = fileSystem.file(fileSystem.path.join(tempDir.path, outputFilePath));
    expect(outputFile, exists);

    final Archive archive = ZipDecoder().decodeBytes(outputFile.readAsBytesSync());

    expect(archive.findFile('base/lib/arm64-v8a/libapp.so') != null, true);
    expect(archive.findFile('base/lib/arm64-v8a/libflutter.so') != null, true);
    expect(archive.findFile('component1/lib/arm64-v8a/libapp.so-2.part.so') != null, false);

    expect(archive.findFile('base/lib/armeabi-v7a/libapp.so') != null, true);
    expect(archive.findFile('base/lib/armeabi-v7a/libflutter.so') != null, true);
    expect(archive.findFile('component1/lib/armeabi-v7a/libapp.so-2.part.so') != null, false);

    expect(archive.findFile('base/lib/x86_64/libapp.so') != null, true);
    expect(archive.findFile('base/lib/x86_64/libflutter.so') != null, true);
    expect(archive.findFile('component1/lib/x86_64/libapp.so-2.part.so') != null, false);

    // Asset 2 is merged into the base module assets.
    expect(
      archive.findFile('component1/assets/flutter_assets/test_assets/asset2.txt') != null,
      false,
    );
    expect(archive.findFile('base/assets/flutter_assets/test_assets/asset2.txt') != null, true);
    expect(archive.findFile('base/assets/flutter_assets/test_assets/asset1.txt') != null, true);
  });

  testWithoutContext(
    'simple build appbundle mismatched golden no-validate-deferred-components succeeds',
    () async {
      final DeferredComponentsProject project = DeferredComponentsProject(
        MismatchedGoldenDeferredComponentsConfig(),
      );
      await project.setUpIn(tempDir);
      final ProcessResult result = await processManager.run(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'build',
        'appbundle',
        '--no-validate-deferred-components',
      ], workingDirectory: tempDir.path);

      expect(result, const ProcessResultMatcher(stdoutPattern: 'app-release.aab'));
      printOnFailure('stdout:\n${result.stdout}');
      printOnFailure('stderr:\n${result.stderr}');
      expect(
        result.stdout.toString(),
        isNot(contains('Deferred components prebuild validation passed.')),
      );
      expect(
        result.stdout.toString(),
        isNot(contains('Deferred components gen_snapshot validation passed.')),
      );
      expect(result.stdout.toString(), isNot(contains('New loading units were found:')));
      expect(
        result.stdout.toString(),
        isNot(contains('Previously existing loading units no longer exist:')),
      );

      final String line = result.stdout
          .toString()
          .split('\n')
          .firstWhere((String line) => line.contains('app-release.aab'));

      final String outputFilePath = line.split(' ')[2].trim();
      final File outputFile = fileSystem.file(fileSystem.path.join(tempDir.path, outputFilePath));
      expect(outputFile, exists);

      final Archive archive = ZipDecoder().decodeBytes(outputFile.readAsBytesSync());

      expect(archive.findFile('base/lib/arm64-v8a/libapp.so') != null, true);
      expect(archive.findFile('base/lib/arm64-v8a/libflutter.so') != null, true);
      expect(archive.findFile('component1/lib/arm64-v8a/libapp.so-2.part.so') != null, true);

      expect(archive.findFile('base/lib/armeabi-v7a/libapp.so') != null, true);
      expect(archive.findFile('base/lib/armeabi-v7a/libflutter.so') != null, true);
      expect(archive.findFile('component1/lib/armeabi-v7a/libapp.so-2.part.so') != null, true);

      expect(archive.findFile('base/lib/x86_64/libapp.so') != null, true);
      expect(archive.findFile('base/lib/x86_64/libflutter.so') != null, true);
      expect(archive.findFile('component1/lib/x86_64/libapp.so-2.part.so') != null, true);

      expect(
        archive.findFile('component1/assets/flutter_assets/test_assets/asset2.txt') != null,
        true,
      );
      expect(archive.findFile('base/assets/flutter_assets/test_assets/asset1.txt') != null, true);
    },
  );

  testWithoutContext('simple build appbundle missing android dynamic feature module fails', () async {
    final DeferredComponentsProject project = DeferredComponentsProject(
      NoAndroidDynamicFeatureModuleDeferredComponentsConfig(),
    );
    await project.setUpIn(tempDir);
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'appbundle',
    ], workingDirectory: tempDir.path);

    expect(
      result,
      const ProcessResultMatcher(exitCode: 1, stdoutPattern: 'Newly generated android files:'),
    );

    expect(result.stdout.toString(), isNot(contains('app-release.aab')));
    expect(
      result.stdout.toString(),
      isNot(contains('Deferred components prebuild validation passed.')),
    );
    expect(
      result.stdout.toString(),
      isNot(contains('Deferred components gen_snapshot validation passed.')),
    );

    final String pathSeparator = fileSystem.path.separator;
    expect(
      result.stdout.toString(),
      contains(
        'build${pathSeparator}android_deferred_components_setup_files${pathSeparator}component1${pathSeparator}build.gradle',
      ),
    );
    expect(
      result.stdout.toString(),
      contains(
        'build${pathSeparator}android_deferred_components_setup_files${pathSeparator}component1${pathSeparator}src${pathSeparator}main${pathSeparator}AndroidManifest.xml',
      ),
    );
  });

  testWithoutContext('simple build appbundle missing golden fails', () async {
    final DeferredComponentsProject project = DeferredComponentsProject(
      NoGoldenDeferredComponentsConfig(),
    );
    await project.setUpIn(tempDir);
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'appbundle',
    ], workingDirectory: tempDir.path);

    expect(result, const ProcessResultMatcher(exitCode: 1));
    expect(result.stdout.toString(), isNot(contains('app-release.aab')));
    expect(result.stdout.toString(), contains('Deferred components prebuild validation passed.'));
    expect(
      result.stdout.toString(),
      isNot(contains('Deferred components gen_snapshot validation passed.')),
    );

    expect(result.stdout.toString(), contains('New loading units were found:'));
    expect(result.stdout.toString(), contains('- package:test/deferred_library.dart'));

    expect(
      result.stdout.toString(),
      isNot(contains('Previously existing loading units no longer exist:')),
    );
  });

  testWithoutContext('simple build appbundle mismatched golden fails', () async {
    final DeferredComponentsProject project = DeferredComponentsProject(
      MismatchedGoldenDeferredComponentsConfig(),
    );
    await project.setUpIn(tempDir);
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'appbundle',
    ], workingDirectory: tempDir.path);

    expect(
      result,
      const ProcessResultMatcher(
        exitCode: 1,
        stdoutPattern: 'Deferred components prebuild validation passed.',
      ),
    );
    expect(result.stdout.toString(), isNot(contains('app-release.aab')));
    expect(
      result.stdout.toString(),
      isNot(contains('Deferred components gen_snapshot validation passed.')),
    );

    expect(result.stdout.toString(), contains('New loading units were found:'));
    expect(result.stdout.toString(), contains('- package:test/deferred_library.dart'));

    expect(
      result.stdout.toString(),
      contains('Previously existing loading units no longer exist:'),
    );
    expect(result.stdout.toString(), contains('- package:test/invalid_lib_name.dart'));

    expect(
      result.stdout.toString(),
      contains('This loading unit check will not fail again on the next build attempt'),
    );
  });
}
