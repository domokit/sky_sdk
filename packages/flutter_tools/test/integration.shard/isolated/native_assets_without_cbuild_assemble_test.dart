// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:process/process.dart';
import 'package:yaml/yaml.dart';

import '../../src/common.dart';
import '../test_utils.dart';
import 'native_assets_test_utils.dart';

/// Regression test as part of https://github.com/flutter/flutter/pull/150742.
///
/// Previously, creating a new (blank, i.e. from `flutter create`) Flutter
/// project, adding `native_assets_cli`, and adding an otherwise valid build hook
/// (`/hook/build.dart`) would fail to build due to the accompanying shell script
/// (at least on macOS) assuming the glob would find at least one output.
///
/// This test verifies that a blank Flutter project with native_assets_cli can
/// build, and does so across all of the host platform and target platform
/// combinations that could trigger this error.
///
/// The version of `native_assets_cli` is derived from the template used by
/// `flutter create --type=packages_ffi`. See
/// [_getPackageFfiTemplatePubspecVersion].
void main() {
  if (!platform.isMacOS && !platform.isLinux && !platform.isWindows) {
    // TODO(dacoharkes): Implement Fuchsia. https://github.com/flutter/flutter/issues/129757
    return;
  }

  const ProcessManager processManager = LocalProcessManager();
  final String constraint = _getPackageFfiTemplatePubspecVersion();

  setUpAll(() {
    processManager.runSync(<String>[flutterBin, 'config', '--enable-native-assets']);
  });

  // Test building a host, iOS, and APK (Android) target where possible.
  for (final String buildCommand in <String>[
    // Current (Host) OS.
    platform.operatingSystem,

    // On macOS, also test iOS.
    if (platform.isMacOS) 'ios',

    // On every host platform, test Android.
    'apk',
  ]) {
    _testBuildCommand(
      buildCommand: buildCommand,
      processManager: processManager,
      nativeAssetsCliVersionConstraint: constraint,
      codeSign: buildCommand != 'ios',
    );
  }
}

void _testBuildCommand({
  required String buildCommand,
  required String nativeAssetsCliVersionConstraint,
  required ProcessManager processManager,
  required bool codeSign,
}) {
  testWithoutContext('flutter build "$buildCommand" succeeds without libraries', () async {
    await inTempDir((Directory tempDirectory) async {
      const String packageName = 'uses_package_native_assets_cli';

      // Create a new (plain Dart SDK) project.
      await expectLater(
        processManager.run(<String>[
          flutterBin,
          'create',
          '--no-pub',
          packageName,
        ], workingDirectory: tempDirectory.path),
        completion(const ProcessResultMatcher()),
      );

      final Directory packageDirectory = tempDirectory.childDirectory(packageName);

      // Add native_assets_cli and resolve implicitly (pub add does pub get).
      // See https://dart.dev/tools/pub/cmd/pub-add#version-constraint.
      await expectLater(
        processManager.run(<String>[
          flutterBin,
          'packages',
          'add',
          'native_assets_cli:$nativeAssetsCliVersionConstraint',
        ], workingDirectory: packageDirectory.path),
        completion(const ProcessResultMatcher()),
      );

      // Add a build hook that does nothing to the package.
      packageDirectory.childDirectory('hook').childFile('build.dart')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
import 'package:native_assets_cli/native_assets_cli.dart';

void main(List<String> args) async {
  await build(args, (config, output) async {});
}
''');

      // Try building.
      //
      // TODO(matanlurey): Stream the app so that we can see partial output.
      final List<String> args = <String>[
        flutterBin,
        'build',
        buildCommand,
        '--debug',
        if (!codeSign) '--no-codesign',
      ];
      io.stderr.writeln('Running $args...');
      final io.Process process = await processManager.start(
        args,
        workingDirectory: packageDirectory.path,
        mode: ProcessStartMode.inheritStdio,
      );
      expect(await process.exitCode, 0);
    });
  }, tags: <String>['flutter-build-apk']);
}

/// Reads `templates/package_ffi/pubspec.yaml.tmpl` to use the package version.
///
/// For example, if the template would output:
/// ```yaml
/// dependencies:
///   native_assets_cli: ^0.8.0
/// ```
///
/// ... then this function would return `'^0.8.0'`.
String _getPackageFfiTemplatePubspecVersion() {
  final String path = Context().join(
    getFlutterRoot(),
    'packages',
    'flutter_tools',
    'templates',
    'package_ffi',
    'pubspec.yaml.tmpl',
  );
  final YamlDocument yaml = loadYamlDocument(
    io.File(path).readAsStringSync(),
    sourceUrl: Uri.parse(path),
  );
  final YamlMap rootNode = yaml.contents as YamlMap;
  final YamlMap dependencies = rootNode.nodes['dependencies']! as YamlMap;
  final String version = dependencies['native_assets_cli']! as String;
  return version;
}
