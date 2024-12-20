// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/localizations.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/generate_localizations.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/localizations/gen_l10n_types.dart';

import '../../integration.shard/test_data/basic_project.dart';
import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  late FileSystem fileSystem;
  late BufferLogger logger;
  late Artifacts artifacts;
  late FakeProcessManager processManager;

  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(isExplicitPackageDependenciesEnabled: true);
  }

  setUpAll(() {
    Cache.disableLocking();
  });

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    logger = BufferLogger.test();
    artifacts = Artifacts.test();
    processManager = FakeProcessManager.empty();
  });

  testUsingContext(
    'default l10n settings',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      await createTestCommandRunner(command).run(<String>['gen-l10n']);

      final Directory outputDirectory = fileSystem.directory(
        fileSystem.path.join('.dart_tool', 'flutter_gen', 'gen_l10n'),
      );
      expect(outputDirectory.existsSync(), true);
      expect(outputDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(outputDirectory.childFile('app_localizations.dart').existsSync(), true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'not using synthetic packages (explicitly)',
    () async {
      final Directory l10nDirectory = fileSystem.directory(fileSystem.path.join('lib', 'l10n'));
      final File arbFile = l10nDirectory.childFile('app_en.arb')..createSync(recursive: true);

      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      fileSystem.file('pubspec.yaml').writeAsStringSync('''
flutter:
  generate: true''');

      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      await createTestCommandRunner(command).run(<String>['gen-l10n', '--no-synthetic-package']);

      expect(l10nDirectory.existsSync(), true);
      expect(l10nDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(l10nDirectory.childFile('app_localizations.dart').existsSync(), true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'not using synthetic packages (due to --explicit-package-dependencies)',
    () async {
      final Directory l10nDirectory = fileSystem.directory(fileSystem.path.join('lib', 'l10n'));
      final File arbFile = l10nDirectory.childFile('app_en.arb')..createSync(recursive: true);

      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      fileSystem.file('pubspec.yaml').writeAsStringSync('''
flutter:
  generate: true''');

      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      await createTestCommandRunner(command).run(<String>['gen-l10n']);

      expect(l10nDirectory.existsSync(), true);
      expect(l10nDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(l10nDirectory.childFile('app_localizations.dart').existsSync(), true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: enableExplicitPackageDependencies,
    },
  );

  testUsingContext(
    'throws error when arguments are invalid',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      fileSystem.file('header.txt').writeAsStringSync('a header file');
      fileSystem.file('pubspec.yaml').writeAsStringSync('''
flutter:
  generate: true''');
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      expect(
        () => createTestCommandRunner(
          command,
        ).run(<String>['gen-l10n', '--header="some header', '--header-file="header.txt"']),
        throwsToolExit(),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'l10n yaml file takes precedence over command line arguments',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      fileSystem.file('l10n.yaml').createSync();
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      await createTestCommandRunner(command).run(<String>['gen-l10n']);

      expect(
        logger.statusText,
        contains('Because l10n.yaml exists, the options defined there will be used instead.'),
      );
      final Directory outputDirectory = fileSystem.directory(
        fileSystem.path.join('.dart_tool', 'flutter_gen', 'gen_l10n'),
      );
      expect(outputDirectory.existsSync(), true);
      expect(outputDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(outputDirectory.childFile('app_localizations.dart').existsSync(), true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'nullable-getter help message is expected string',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      fileSystem.file('l10n.yaml').createSync();
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      await createTestCommandRunner(command).run(<String>['gen-l10n']);
      expect(command.usage, contains(' If this value is set to false, then '));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'dart format is run when --format is passed',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
      processManager.addCommand(
        const FakeCommand(
          command: <String>[
            'Artifact.engineDartBinary',
            'format',
            '/.dart_tool/flutter_gen/gen_l10n/app_localizations_en.dart',
            '/.dart_tool/flutter_gen/gen_l10n/app_localizations.dart',
          ],
        ),
      );

      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );

      await createTestCommandRunner(command).run(<String>['gen-l10n', '--format']);

      final Directory outputDirectory = fileSystem.directory(
        fileSystem.path.join('.dart_tool', 'flutter_gen', 'gen_l10n'),
      );
      expect(outputDirectory.existsSync(), true);
      expect(outputDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(outputDirectory.childFile('app_localizations.dart').existsSync(), true);
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'dart format is run when format: true is passed into l10n.yaml',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      final File configFile = fileSystem.file('l10n.yaml')..createSync();
      configFile.writeAsStringSync('''
format: true
''');
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
      processManager.addCommand(
        const FakeCommand(
          command: <String>[
            'Artifact.engineDartBinary',
            'format',
            '/.dart_tool/flutter_gen/gen_l10n/app_localizations_en.dart',
            '/.dart_tool/flutter_gen/gen_l10n/app_localizations.dart',
          ],
        ),
      );
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      await createTestCommandRunner(command).run(<String>['gen-l10n']);

      final Directory outputDirectory = fileSystem.directory(
        fileSystem.path.join('.dart_tool', 'flutter_gen', 'gen_l10n'),
      );
      expect(outputDirectory.existsSync(), true);
      expect(outputDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(outputDirectory.childFile('app_localizations.dart').existsSync(), true);
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/119594
  testUsingContext(
    'dart format is working when the untranslated messages file is produced',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "untranslated": "Test untranslated message."
}''');
      fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_es.arb'))
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "helloWorld": "Hello, World!"
}''');
      final File configFile = fileSystem.file('l10n.yaml')..createSync();
      configFile.writeAsStringSync('''
format: true
untranslated-messages-file: lib/l10n/untranslated.json
''');
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
      processManager.addCommand(
        const FakeCommand(
          command: <String>[
            'Artifact.engineDartBinary',
            'format',
            '/.dart_tool/flutter_gen/gen_l10n/app_localizations_en.dart',
            '/.dart_tool/flutter_gen/gen_l10n/app_localizations_es.dart',
            '/.dart_tool/flutter_gen/gen_l10n/app_localizations.dart',
          ],
        ),
      );
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      await createTestCommandRunner(command).run(<String>['gen-l10n']);

      final Directory outputDirectory = fileSystem.directory(
        fileSystem.path.join('.dart_tool', 'flutter_gen', 'gen_l10n'),
      );
      expect(outputDirectory.existsSync(), true);
      expect(outputDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(outputDirectory.childFile('app_localizations_es.dart').existsSync(), true);
      expect(outputDirectory.childFile('app_localizations.dart').existsSync(), true);
      final File untranslatedMessagesFile = fileSystem.file(
        fileSystem.path.join('lib', 'l10n', 'untranslated.json'),
      );
      expect(untranslatedMessagesFile.existsSync(), true);
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/120530.
  testUsingContext(
    'dart format is run when generateLocalizations is called through build target',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      final File configFile = fileSystem.file('l10n.yaml')..createSync();
      configFile.writeAsStringSync('''
format: true
''');
      const Target buildTarget = GenerateLocalizationsTarget();
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
      processManager.addCommand(
        const FakeCommand(
          command: <String>[
            'Artifact.engineDartBinary',
            'format',
            '/lib/l10n/app_localizations_en.dart',
            '/lib/l10n/app_localizations.dart',
          ],
        ),
      );
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        artifacts: artifacts,
        processManager: processManager,
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
      );
      await buildTarget.build(environment);

      final Directory outputDirectory = fileSystem.directory(fileSystem.path.join('lib', 'l10n'));
      expect(outputDirectory.existsSync(), true);
      expect(outputDirectory.childFile('app_localizations_en.dart').existsSync(), true);
      expect(outputDirectory.childFile('app_localizations.dart').existsSync(), true);
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{FeatureFlags: enableExplicitPackageDependencies},
  );

  testUsingContext(
    'nullable-getter defaults to true',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      await createTestCommandRunner(command).run(<String>['gen-l10n']);

      final Directory outputDirectory = fileSystem.directory(
        fileSystem.path.join('.dart_tool', 'flutter_gen', 'gen_l10n'),
      );
      expect(outputDirectory.existsSync(), isTrue);
      expect(outputDirectory.childFile('app_localizations.dart').existsSync(), isTrue);
      expect(
        outputDirectory.childFile('app_localizations.dart').readAsStringSync(),
        contains('static AppLocalizations? of(BuildContext context)'),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'throw when generate: false and uses synthetic package when run with l10n.yaml',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      fileSystem.file('l10n.yaml').createSync();
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync('''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter

  flutter:
    generate: false
  ''');
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      expect(
        () async => createTestCommandRunner(command).run(<String>['gen-l10n']),
        throwsToolExit(
          message:
              'Attempted to generate localizations code without having the flutter: generate flag turned on.',
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'throw when generate: false and uses synthetic package when run via commandline options',
    () async {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      arbFile.writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
      pubspecFile.writeAsStringSync('''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter

  flutter:
    generate: false
  ''');
      final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
        fileSystem: fileSystem,
        logger: logger,
        artifacts: artifacts,
        processManager: processManager,
      );
      expect(
        () async =>
            createTestCommandRunner(command).run(<String>['gen-l10n', '--synthetic-package']),
        throwsToolExit(
          message:
              'Attempted to generate localizations code without having the flutter: generate flag turned on.',
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext('throws error when unexpected positional argument is provided', () {
    final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
      fileSystem: fileSystem,
      logger: logger,
      artifacts: artifacts,
      processManager: processManager,
    );
    expect(
      () async => createTestCommandRunner(
        command,
      ).run(<String>['gen-l10n', '--synthetic-package', 'false']),
      throwsToolExit(message: 'Unexpected positional argument "false".'),
    );
  });

  group(AppResourceBundle, () {
    testWithoutContext("can be parsed without FormatException when it's content is empty", () {
      final File arbFile = fileSystem.file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true);
      expect(AppResourceBundle(arbFile), isA<AppResourceBundle>());
    });

    testUsingContext(
      "would not fail the gen-l10n command when it's content is empty",
      () async {
        fileSystem
            .file(fileSystem.path.join('lib', 'l10n', 'app_en.arb'))
            .createSync(recursive: true);
        final File pubspecFile = fileSystem.file('pubspec.yaml')..createSync();
        pubspecFile.writeAsStringSync(BasicProjectWithFlutterGen().pubspec);
        final GenerateLocalizationsCommand command = GenerateLocalizationsCommand(
          fileSystem: fileSystem,
          logger: logger,
          artifacts: artifacts,
          processManager: processManager,
        );
        await createTestCommandRunner(command).run(<String>['gen-l10n']);

        final Directory outputDirectory = fileSystem.directory(
          fileSystem.path.join('.dart_tool', 'flutter_gen', 'gen_l10n'),
        );
        expect(outputDirectory.existsSync(), true);
        expect(outputDirectory.childFile('app_localizations_en.dart').existsSync(), true);
        expect(outputDirectory.childFile('app_localizations.dart').existsSync(), true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });
}
