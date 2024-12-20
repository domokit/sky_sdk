// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';

import '../../../src/context.dart'; // legacy
import '../../../src/fake_pub_deps.dart';
import '../../../src/fakes.dart';
import '../../../src/test_build_system.dart';
import '../../../src/test_flutter_command_runner.dart'; // legacy

void main() {
  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(
      isExplicitPackageDependenciesEnabled: true,
      // Assumed to be true below.
      isWebEnabled: true,
    );
  }

  setUpAll(() {
    Cache.flutterRoot = '';
    Cache.disableLocking();
  });

  group('ScrubGeneratedPluginRegistrant', () {
    // The files this migration deals with
    late File gitignore;
    late File registrant;

    // Environment overrides
    late Artifacts artifacts;
    late FileSystem fileSystem;
    late ProcessManager processManager;
    late BuildSystem buildSystem;
    late ProcessUtils processUtils;
    late BufferLogger logger;

    setUp(() {
      // Prepare environment overrides
      fileSystem = MemoryFileSystem.test();
      artifacts = Artifacts.test(fileSystem: fileSystem);
      processManager = FakeProcessManager.any();
      logger = BufferLogger.test();
      processUtils = ProcessUtils(processManager: processManager, logger: logger);

      buildSystem = TestBuildSystem.all(BuildResult(success: true));
      // Write some initial state into our testing filesystem
      setupFileSystemForEndToEndTest(fileSystem);
      // Initialize fileSystem references
      gitignore = fileSystem.file('.gitignore');
      registrant = fileSystem.file(fileSystem.path.join('lib', 'generated_plugin_registrant.dart'));
    });

    testUsingContext(
      'noop - nothing to do - build runs',
      () async {
        expect(gitignore.existsSync(), isFalse);
        expect(registrant.existsSync(), isFalse);

        await createTestCommandRunner(
          BuildCommand(
            artifacts: artifacts,
            androidSdk: FakeAndroidSdk(),
            buildSystem: buildSystem,
            fileSystem: fileSystem,
            logger: BufferLogger.test(),
            osUtils: FakeOperatingSystemUtils(),
            processUtils: processUtils,
          ),
        ).run(<String>['build', 'web', '--no-pub']);

        final Directory buildDir = fileSystem.directory(fileSystem.path.join('build', 'web'));
        expect(buildDir.existsSync(), true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        BuildSystem: () => buildSystem,
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );

    testUsingContext(
      'noop - .gitignore does not reference generated_plugin_registrant.dart - untouched',
      () async {
        writeGitignore(fileSystem, mentionsPluginRegistrant: false);

        final String contentsBeforeBuild = gitignore.readAsStringSync();
        expect(contentsBeforeBuild, isNot(contains('lib/generated_plugin_registrant.dart')));

        await createTestCommandRunner(
          BuildCommand(
            artifacts: artifacts,
            androidSdk: FakeAndroidSdk(),
            buildSystem: buildSystem,
            fileSystem: fileSystem,
            logger: logger,
            osUtils: FakeOperatingSystemUtils(),
            processUtils: processUtils,
          ),
        ).run(<String>['build', 'web', '--no-pub']);

        expect(gitignore.readAsStringSync(), contentsBeforeBuild);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        BuildSystem: () => buildSystem,
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );

    testUsingContext(
      '.gitignore references generated_plugin_registrant - cleans it up',
      () async {
        writeGitignore(fileSystem);

        expect(gitignore.existsSync(), isTrue);
        expect(gitignore.readAsStringSync(), contains('lib/generated_plugin_registrant.dart'));

        await createTestCommandRunner(
          BuildCommand(
            artifacts: artifacts,
            androidSdk: FakeAndroidSdk(),
            buildSystem: buildSystem,
            fileSystem: fileSystem,
            logger: logger,
            processUtils: processUtils,
            osUtils: FakeOperatingSystemUtils(),
          ),
        ).run(<String>['build', 'web', '--no-pub']);

        expect(
          gitignore.readAsStringSync(),
          isNot(contains('lib/generated_plugin_registrant.dart')),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        BuildSystem: () => buildSystem,
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );

    testUsingContext(
      'generated_plugin_registrant.dart exists - gets deleted',
      () async {
        writeGeneratedPluginRegistrant(fileSystem);

        expect(registrant.existsSync(), isTrue);

        await createTestCommandRunner(
          BuildCommand(
            artifacts: artifacts,
            androidSdk: FakeAndroidSdk(),
            buildSystem: buildSystem,
            fileSystem: fileSystem,
            logger: logger,
            processUtils: processUtils,
            osUtils: FakeOperatingSystemUtils(),
          ),
        ).run(<String>['build', 'web', '--no-pub']);

        expect(registrant.existsSync(), isFalse);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        BuildSystem: () => buildSystem,
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );

    testUsingContext(
      'scrubs generated_plugin_registrant file and cleans .gitignore',
      () async {
        writeGitignore(fileSystem);
        writeGeneratedPluginRegistrant(fileSystem);

        expect(registrant.existsSync(), isTrue);
        expect(gitignore.readAsStringSync(), contains('lib/generated_plugin_registrant.dart'));

        await createTestCommandRunner(
          BuildCommand(
            artifacts: artifacts,
            androidSdk: FakeAndroidSdk(),
            buildSystem: buildSystem,
            fileSystem: fileSystem,
            logger: logger,
            processUtils: processUtils,
            osUtils: FakeOperatingSystemUtils(),
          ),
        ).run(<String>['build', 'web', '--no-pub']);

        expect(registrant.existsSync(), isFalse);
        expect(
          gitignore.readAsStringSync(),
          isNot(contains('lib/generated_plugin_registrant.dart')),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        BuildSystem: () => buildSystem,
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );
  });
}

// Writes something that resembles the contents of Flutter's .gitignore file
void writeGitignore(FileSystem fs, {bool mentionsPluginRegistrant = true}) {
  fs.file('.gitignore').createSync(recursive: true);
  fs.file('.gitignore').writeAsStringSync('''
/build/

# Web related
${mentionsPluginRegistrant ? 'lib/generated_plugin_registrant.dart' : 'another_file.dart'}

# Symbolication related
''');
}

// Creates an empty generated_plugin_registrant.dart file
void writeGeneratedPluginRegistrant(FileSystem fs) {
  final String path = fs.path.join('lib', 'generated_plugin_registrant.dart');
  fs.file(path).createSync(recursive: true);
}

// Adds a bunch of files to the filesystem
// (taken from commands.shard/hermetic/build_web_test.dart)
void setupFileSystemForEndToEndTest(FileSystem fileSystem) {
  final List<String> dependencies = <String>[
    fileSystem.path.join('.dart_tool', 'package_config.json'),
    fileSystem.path.join('web', 'index.html'),
    fileSystem.path.join('lib', 'main.dart'),
    fileSystem.path.join(
      'packages',
      'flutter_tools',
      'lib',
      'src',
      'build_system',
      'targets',
      'web.dart',
    ),
    fileSystem.path.join('bin', 'cache', 'flutter_web_sdk'),
    fileSystem.path.join('bin', 'cache', 'dart-sdk', 'bin', 'dart'),
    fileSystem.path.join('bin', 'cache', 'dart-sdk', 'bin', 'dartaotruntime'),
    fileSystem.path.join('bin', 'cache', 'dart-sdk '),
  ];
  for (final String dependency in dependencies) {
    fileSystem.file(dependency).createSync(recursive: true);
  }

  // Project files.
  fileSystem.file('pubspec.yaml').writeAsStringSync('''
name: foo

dependencies:
  flutter:
    sdk: flutter
  fizz:
    path:
      bar/
''');
  fileSystem.file(fileSystem.path.join('bar', 'pubspec.yaml'))
    ..createSync(recursive: true)
    ..writeAsStringSync('''
name: bar

flutter:
  plugin:
    platforms:
      web:
        pluginClass: UrlLauncherPlugin
        fileName: url_launcher_web.dart
''');
  fileSystem.file(fileSystem.path.join('bar', 'lib', 'url_launcher_web.dart'))
    ..createSync(recursive: true)
    ..writeAsStringSync('''
class UrlLauncherPlugin {}
''');
  fileSystem.file(fileSystem.path.join('lib', 'main.dart')).writeAsStringSync('void main() { }');
}
