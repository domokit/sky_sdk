// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/packages.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';
import 'package:test/fake.dart';

import '../../src/context.dart';
import '../../src/fake_pub_deps.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';

const String minimalV2EmbeddingManifest = r'''
<manifest xmlns:android="http://schemas.android.com/apk/res/android">
    <application
        android:name="${applicationName}">
        <meta-data
            android:name="flutterEmbedding"
            android:value="2" />
    </application>
</manifest>
''';

void main() {
  late FileSystem fileSystem;
  late FakePub pub;

  // TODO(matanlurey): Remove after `flutter_gen` is removed.
  // See https://github.com/flutter/flutter/issues/102983 for details.
  FeatureFlags disableExplicitPackageDependencies() {
    return TestFeatureFlags(
      // ignore: avoid_redundant_argument_values
      isExplicitPackageDependenciesEnabled: false,
    );
  }

  setUp(() {
    Cache.disableLocking();
    fileSystem = MemoryFileSystem.test();
    pub = FakePub(fileSystem);
  });

  tearDown(() {
    Cache.enableLocking();
  });

  testUsingContext('pub shows help', () async {
    Object? usage;
    final PackagesCommand command = PackagesCommand(
      usagePrintFn: (Object? object) => usage = object,
    );
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await runner.run(<String>['pub']);

    expect(
      usage,
      allOf(
        contains('Commands for managing Flutter packages.'),
        contains('Usage: flutter pub <subcommand> [arguments]'),
      ),
    );
  });

  testUsingContext(
    'pub get usage values are resilient to missing package config files before running "pub get"',
    () async {
      fileSystem.currentDirectory.childFile('pubspec.yaml').createSync();
      fileSystem.currentDirectory.childFile('.flutter-plugins').createSync();
      fileSystem.currentDirectory.childFile('.flutter-plugins-dependencies').createSync();
      fileSystem.currentDirectory.childDirectory('android').childFile('AndroidManifest.xml')
        ..createSync(recursive: true)
        ..writeAsStringSync(minimalV2EmbeddingManifest);

      final PackagesGetCommand command = PackagesGetCommand('get', '', PubContext.pubGet);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);

      await commandRunner.run(<String>['get']);

      expect(
        await command.usageValues,
        const CustomDimensions(
          commandPackagesNumberPlugins: 0,
          commandPackagesProjectModule: false,
          commandPackagesAndroidEmbeddingVersion: 'v2',
        ),
      );
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
    },
  );

  testUsingContext(
    'pub get usage values are resilient to poorly formatted package config before "pub get"',
    () async {
      fileSystem.currentDirectory.childFile('pubspec.yaml').createSync();
      fileSystem.currentDirectory.childFile('.flutter-plugins').createSync();
      fileSystem.currentDirectory.childFile('.flutter-plugins-dependencies').createSync();
      fileSystem.currentDirectory.childFile('.dart_tool/package_config.json')
        ..createSync(recursive: true)
        ..writeAsBytesSync(<int>[0]);
      fileSystem.currentDirectory.childDirectory('android').childFile('AndroidManifest.xml')
        ..createSync(recursive: true)
        ..writeAsStringSync(minimalV2EmbeddingManifest);

      final PackagesGetCommand command = PackagesGetCommand('get', '', PubContext.pubGet);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);

      await commandRunner.run(<String>['get']);

      expect(
        await command.usageValues,
        const CustomDimensions(
          commandPackagesNumberPlugins: 0,
          commandPackagesProjectModule: false,
          commandPackagesAndroidEmbeddingVersion: 'v2',
        ),
      );
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
    },
  );

  testUsingContext(
    'pub get on target directory',
    () async {
      fileSystem.currentDirectory.childDirectory('target').createSync();
      final Directory targetDirectory = fileSystem.currentDirectory.childDirectory('target');
      targetDirectory.childFile('pubspec.yaml').createSync();

      final PackagesGetCommand command = PackagesGetCommand('get', '', PubContext.pubGet);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);

      await commandRunner.run(<String>['get', '--directory=${targetDirectory.path}']);
      final FlutterProject rootProject = FlutterProject.fromDirectory(targetDirectory);
      final File packageConfigFile = rootProject.dartTool.childFile('package_config.json');
      expect(packageConfigFile.existsSync(), true);
      expect(packageConfigFile.readAsStringSync(), '{"configVersion":2,"packages":[]}');
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
    },
  );

  testUsingContext(
    "pub get doesn't treat unknown flag as directory",
    () async {
      fileSystem.currentDirectory.childDirectory('target').createSync();
      fileSystem.currentDirectory.childFile('pubspec.yaml').createSync();
      final PackagesGetCommand command = PackagesGetCommand('get', '', PubContext.pubGet);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);
      pub.expectedArguments = <String>['get', '--unknown-flag', '--example', '--directory', '.'];
      await commandRunner.run(<String>['get', '--unknown-flag']);
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
    },
  );

  testUsingContext(
    "pub get doesn't treat -v as directory",
    () async {
      fileSystem.currentDirectory.childDirectory('target').createSync();
      fileSystem.currentDirectory.childFile('pubspec.yaml').createSync();
      final PackagesGetCommand command = PackagesGetCommand('get', '', PubContext.pubGet);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);
      pub.expectedArguments = <String>['get', '-v', '--example', '--directory', '.'];
      await commandRunner.run(<String>['get', '-v']);
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/144898
  // Regression test for https://github.com/flutter/flutter/issues/160145
  testUsingContext(
    "pub add doesn't treat dependency syntax as directory",
    () async {
      fileSystem.currentDirectory.childDirectory('target').createSync();
      fileSystem.currentDirectory.childFile('pubspec.yaml').createSync();
      fileSystem.currentDirectory.childDirectory('example').createSync(recursive: true);
      fileSystem.currentDirectory.childDirectory('android').childFile('AndroidManifest.xml')
        ..createSync(recursive: true)
        ..writeAsStringSync(minimalV2EmbeddingManifest);

      final PackagesGetCommand command = PackagesGetCommand('add', '', PubContext.pubAdd);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);
      const List<String> availableSyntax = <String>[
        'foo:{"path":"../foo"}',
        'foo:{"hosted":"my-pub.dev"}',
        'foo:{"sdk":"flutter"}',
        'foo:{"git":"https://github.com/foo/foo"}',
      ];
      for (final String syntax in availableSyntax) {
        pub.expectedArguments = <String>['add', syntax, '--example', '--directory', '.'];
        await commandRunner.run(<String>['add', syntax]);
      }
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
    },
  );

  testUsingContext(
    "pub get skips example directory if it doesn't contain a pubspec.yaml",
    () async {
      fileSystem.currentDirectory.childFile('pubspec.yaml').createSync();
      fileSystem.currentDirectory.childDirectory('example').createSync(recursive: true);
      fileSystem.currentDirectory.childDirectory('android').childFile('AndroidManifest.xml')
        ..createSync(recursive: true)
        ..writeAsStringSync(minimalV2EmbeddingManifest);

      final PackagesGetCommand command = PackagesGetCommand('get', '', PubContext.pubGet);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);

      await commandRunner.run(<String>['get']);

      expect(
        await command.usageValues,
        const CustomDimensions(
          commandPackagesNumberPlugins: 0,
          commandPackagesProjectModule: false,
          commandPackagesAndroidEmbeddingVersion: 'v2',
        ),
      );
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
    },
  );

  testUsingContext(
    'pub get throws error on missing directory',
    () async {
      final PackagesGetCommand command = PackagesGetCommand('get', '', PubContext.pubGet);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);

      try {
        await commandRunner.run(<String>['get', '--directory=missing_dir']);
        fail('expected an exception');
      } on Exception catch (e) {
        expect(e.toString(), contains('Expected to find project root in missing_dir'));
      }
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
    },
  );

  testUsingContext(
    'pub get triggers localizations generation when generate: true',
    () async {
      final File pubspecFile = fileSystem.currentDirectory.childFile('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync('''
      flutter:
        generate: true
      ''');
      fileSystem.currentDirectory.childFile('l10n.yaml')
        ..createSync()
        ..writeAsStringSync('''
        arb-dir: lib/l10n
        ''');
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
      {
        "helloWorld": "Hello, World!",
        "@helloWorld": {
          "description": "Sample description"
        }
      }
      ''');

      final PackagesGetCommand command = PackagesGetCommand('get', '', PubContext.pubGet);
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);

      await commandRunner.run(<String>['get']);
      final FlutterCommandResult result = await command.runCommand();

      expect(result.exitStatus, ExitStatus.success);
      final Directory outputDirectory = fileSystem.directory(
        fileSystem.path.join('.dart_tool', 'flutter_gen', 'gen_l10n'),
      );
      expect(outputDirectory.existsSync(), true);
      expect(outputDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(outputDirectory.childFile('app_localizations.dart').existsSync(), true);
    },
    overrides: <Type, Generator>{
      Pub: () => pub,
      ProcessManager: () => FakeProcessManager.any(),
      FileSystem: () => fileSystem,
      FeatureFlags: disableExplicitPackageDependencies,
    },
  );
}

class FakePub extends Fake implements Pub {
  FakePub(this.fileSystem);

  final FileSystem fileSystem;
  List<String>? expectedArguments;

  @override
  Future<void> interactively(
    List<String> arguments, {
    FlutterProject? project,
    required PubContext context,
    required String command,
    bool touchesPackageConfig = false,
    bool generateSyntheticPackage = false,
    PubOutputMode outputMode = PubOutputMode.all,
  }) async {
    if (expectedArguments != null) {
      expect(arguments, expectedArguments);
    }
    if (project != null) {
      fileSystem
          .directory(project.directory)
          .childDirectory('.dart_tool')
          .childFile('package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('{"configVersion":2,"packages":[]}');
    }
  }

  @override
  Future<Map<String, Object?>> deps(FlutterProject project) {
    return FakePubWithPrimedDeps().deps(project);
  }
}
