// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/bot_detector.dart';
import 'package:flutter_tools/src/base/error_handling_io.dart';
import 'package:flutter_tools/src/base/file_system.dart' hide IOSink;
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/packages.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:unified_analytics/unified_analytics.dart';
import 'package:yaml/yaml.dart';

import '../../integration.shard/test_utils.dart';
import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  late FakeStdio mockStdio;

  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(isExplicitPackageDependenciesEnabled: true);
  }

  // TODO(matanlurey): Remove after `flutter_gen` is removed.
  // See https://github.com/flutter/flutter/issues/102983 for details.
  FeatureFlags disableExplicitPackageDependencies() {
    return TestFeatureFlags(
      // ignore: avoid_redundant_argument_values
      isExplicitPackageDependenciesEnabled: false,
    );
  }

  setUp(() {
    mockStdio = FakeStdio()..stdout.terminalColumns = 80;

    // Some tests below override this with a blank root, always reset it.
    Cache.flutterRoot = null;
  });

  setUpAll(() {
    Cache.disableLocking();
  });

  group('packages get/upgrade', () {
    late Directory tempDir;
    late FakeAnalytics fakeAnalytics;

    setUp(() {
      tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_packages_test.');
      fakeAnalytics = getInitializedFakeAnalyticsInstance(
        fs: MemoryFileSystem.test(),
        fakeFlutterVersion: FakeFlutterVersion(),
      );
    });

    tearDown(() {
      tryToDelete(tempDir);
    });

    Future<String> createProjectWithPlugin(String plugin, {List<String>? arguments}) async {
      final String projectPath = await createProject(tempDir, arguments: arguments);
      final File pubspec = globals.fs.file(globals.fs.path.join(projectPath, 'pubspec.yaml'));
      String content = await pubspec.readAsString();
      final List<String> contentLines = LineSplitter.split(content).toList();
      final int depsIndex = contentLines.indexOf('dependencies:');
      expect(depsIndex, isNot(-1));
      contentLines.replaceRange(depsIndex, depsIndex + 1, <String>['dependencies:', '  $plugin:']);
      content = contentLines.join('\n');
      await pubspec.writeAsString(content, flush: true);
      return projectPath;
    }

    Future<PackagesCommand> runCommandIn(
      String projectPath,
      String verb, {
      List<String>? args,
      List<String>? globalArgs,
    }) async {
      final PackagesCommand command = PackagesCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        ...?globalArgs,
        'packages',
        verb,
        ...?args,
        '--directory',
        projectPath,
      ]);
      return command;
    }

    void expectExists(String projectPath, String relPath) {
      expect(
        globals.fs.isFileSync(globals.fs.path.join(projectPath, relPath)),
        true,
        reason: '$projectPath/$relPath should exist, but does not',
      );
    }

    void expectContains(String projectPath, String relPath, String substring) {
      expectExists(projectPath, relPath);
      expect(
        globals.fs.file(globals.fs.path.join(projectPath, relPath)).readAsStringSync(),
        contains(substring),
        reason: '$projectPath/$relPath has unexpected content',
      );
    }

    void expectNotExists(String projectPath, String relPath) {
      expect(
        globals.fs.isFileSync(globals.fs.path.join(projectPath, relPath)),
        false,
        reason: '$projectPath/$relPath should not exist, but does',
      );
    }

    void expectNotContains(String projectPath, String relPath, String substring) {
      expectExists(projectPath, relPath);
      expect(
        globals.fs.file(globals.fs.path.join(projectPath, relPath)).readAsStringSync(),
        isNot(contains(substring)),
        reason: '$projectPath/$relPath has unexpected content',
      );
    }

    final List<String> pubOutput = <String>[
      globals.fs.path.join('.dart_tool', 'package_config.json'),
      'pubspec.lock',
    ];

    const List<String> pluginRegistrants = <String>[
      'ios/Runner/GeneratedPluginRegistrant.h',
      'ios/Runner/GeneratedPluginRegistrant.m',
      'android/app/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
    ];

    const List<String> modulePluginRegistrants = <String>[
      '.ios/Flutter/FlutterPluginRegistrant/Classes/GeneratedPluginRegistrant.h',
      '.ios/Flutter/FlutterPluginRegistrant/Classes/GeneratedPluginRegistrant.m',
      '.android/Flutter/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
    ];

    List<String> pluginWitnesses({required bool includeLegacyPluginsList}) {
      return <String>[
        if (includeLegacyPluginsList) '.flutter-plugins',
        '.flutter-plugins-dependencies',
        'ios/Podfile',
      ];
    }

    List<String> modulePluginWitnesses({required bool includeLegacyPluginsList}) {
      return <String>[
        if (includeLegacyPluginsList) '.flutter-plugins',
        '.flutter-plugins-dependencies',
        '.ios/Podfile',
      ];
    }

    const Map<String, String> pluginContentWitnesses = <String, String>{
      'ios/Flutter/Debug.xcconfig':
          '#include? "Pods/Target Support Files/Pods-Runner/Pods-Runner.debug.xcconfig"',
      'ios/Flutter/Release.xcconfig':
          '#include? "Pods/Target Support Files/Pods-Runner/Pods-Runner.release.xcconfig"',
    };

    const Map<String, String> modulePluginContentWitnesses = <String, String>{
      '.ios/Config/Debug.xcconfig':
          '#include? "Pods/Target Support Files/Pods-Runner/Pods-Runner.debug.xcconfig"',
      '.ios/Config/Release.xcconfig':
          '#include? "Pods/Target Support Files/Pods-Runner/Pods-Runner.release.xcconfig"',
    };

    void expectDependenciesResolved(String projectPath) {
      for (final String output in pubOutput) {
        expectExists(projectPath, output);
      }
    }

    void expectZeroPluginsInjected(String projectPath) {
      for (final String registrant in modulePluginRegistrants) {
        expectExists(projectPath, registrant);
      }
      for (final String witness in pluginWitnesses(includeLegacyPluginsList: true)) {
        expectNotExists(projectPath, witness);
      }
      for (final String witness in modulePluginWitnesses(includeLegacyPluginsList: true)) {
        expectNotExists(projectPath, witness);
      }
      modulePluginContentWitnesses.forEach((String witness, String content) {
        expectNotContains(projectPath, witness, content);
      });
    }

    void expectPluginInjected(String projectPath, {required bool includeLegacyPluginsList}) {
      for (final String registrant in pluginRegistrants) {
        expectExists(projectPath, registrant);
      }
      for (final String witness in pluginWitnesses(
        includeLegacyPluginsList: includeLegacyPluginsList,
      )) {
        expectExists(projectPath, witness);
      }
      if (!includeLegacyPluginsList) {
        expectNotExists(projectPath, '.flutter-plugins');
      }
      pluginContentWitnesses.forEach((String witness, String content) {
        expectContains(projectPath, witness, content);
      });
    }

    void expectModulePluginInjected(String projectPath, {required bool includeLegacyPluginsList}) {
      for (final String registrant in modulePluginRegistrants) {
        expectExists(projectPath, registrant);
      }
      for (final String witness in modulePluginWitnesses(
        includeLegacyPluginsList: includeLegacyPluginsList,
      )) {
        expectExists(projectPath, witness);
      }
      modulePluginContentWitnesses.forEach((String witness, String content) {
        expectContains(projectPath, witness, content);
      });
    }

    void removeGeneratedFiles(String projectPath) {
      final Iterable<String> allFiles = <List<String>>[
        pubOutput,
        modulePluginRegistrants,
        pluginWitnesses(includeLegacyPluginsList: true),
      ].expand<String>((List<String> list) => list);
      for (final String path in allFiles) {
        final File file = globals.fs.file(globals.fs.path.join(projectPath, path));
        ErrorHandlingFileSystem.deleteIfExists(file);
      }
    }

    testUsingContext(
      'get fetches packages and has output from pub',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );
        removeGeneratedFiles(projectPath);

        await runCommandIn(projectPath, 'get');

        expect(
          mockStdio.stdout.writes.map(utf8.decode),
          allOf(
            // The output of pub changed, adding backticks around the directory name.
            // These regexes are tolerant of the backticks being present or absent.
            contains(matches(RegExp(r'Resolving dependencies in .+flutter_project`?\.\.\.'))),
            contains(matches(RegExp(r'\+ flutter 0\.0\.0 from sdk flutter'))),
            contains(matches(RegExp(r'Changed \d+ dependencies in .+flutter_project`?!'))),
          ),
        );

        expectDependenciesResolved(projectPath);
        expectZeroPluginsInjected(projectPath);
        expect(
          analyticsTimingEventExists(
            sentEvents: fakeAnalytics.sentEvents,
            workflow: 'pub',
            variableName: 'get',
            label: 'success',
          ),
          true,
        );
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'get --offline fetches packages',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );
        removeGeneratedFiles(projectPath);

        await runCommandIn(projectPath, 'get', args: <String>['--offline']);

        expectDependenciesResolved(projectPath);
        expectZeroPluginsInjected(projectPath);
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'get generates synthetic package when l10n.yaml has synthetic-package: true',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );
        final Directory projectDir = globals.fs.directory(projectPath);
        projectDir.childDirectory('lib').childDirectory('l10n').childFile('app_en.arb')
          ..createSync(recursive: true)
          ..writeAsStringSync('{ "hello": "Hello world!" }');
        String pubspecFileContent = projectDir.childFile('pubspec.yaml').readAsStringSync();
        pubspecFileContent = pubspecFileContent.replaceFirst(RegExp(r'\nflutter\:'), '''
flutter:
  generate: true
''');
        projectDir.childFile('pubspec.yaml').writeAsStringSync(pubspecFileContent);
        projectDir.childFile('l10n.yaml').writeAsStringSync('synthetic-package: true');
        await runCommandIn(projectPath, 'get');
        expect(
          projectDir
              .childDirectory('.dart_tool')
              .childDirectory('flutter_gen')
              .childDirectory('gen_l10n')
              .childFile('app_localizations.dart')
              .existsSync(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Pub:
            () => Pub(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
            ),
        FeatureFlags: disableExplicitPackageDependencies,
      },
    );

    testUsingContext(
      'get fetches packages for a workspace',
      () async {
        tempDir.childFile('pubspec.yaml').writeAsStringSync('''
name: workspace
environment:
  sdk: ^3.7.0-0
workspace:
  - flutter_project
''');
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );
        final File pubspecFile = fileSystem.file(fileSystem.path.join(projectPath, 'pubspec.yaml'));
        final YamlMap pubspecYaml = loadYaml(pubspecFile.readAsStringSync()) as YamlMap;
        final Map<String, Object?> pubspec = <String, Object?>{
          ...pubspecYaml.value.cast<String, Object?>(),
          'resolution': 'workspace',
          'environment': <String, Object?>{
            ...(pubspecYaml['environment'] as YamlMap).value.cast<String, Object?>(),
            'sdk': '^3.5.0-0',
          },
        };
        pubspecFile.writeAsStringSync(jsonEncode(pubspec));
        await runCommandIn(projectPath, 'get');

        expect(
          mockStdio.stdout.writes.map(utf8.decode),
          allOf(
            // The output of pub changed, adding backticks around the directory name.
            // These regexes are tolerant of the backticks being present or absent.
            contains(
              matches(
                RegExp(
                  r'Resolving dependencies in .+' + RegExp.escape(tempDir.basename) + r'`?\.\.\.',
                ),
              ),
            ),
            contains(matches(RegExp(r'\+ flutter 0\.0\.0 from sdk flutter'))),
            contains(
              matches(
                RegExp(
                  r'Changed \d+ dependencies in .+' + RegExp.escape(tempDir.basename) + r'`?!',
                ),
              ),
            ),
          ),
        );
        expectDependenciesResolved(tempDir.path);
        expectZeroPluginsInjected(projectPath);
        expect(
          analyticsTimingEventExists(
            sentEvents: fakeAnalytics.sentEvents,
            workflow: 'pub',
            variableName: 'get',
            label: 'success',
          ),
          true,
        );
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'get generates normal files when l10n.yaml has synthetic-package: false',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );
        final Directory projectDir = globals.fs.directory(projectPath);
        projectDir.childDirectory('lib').childDirectory('l10n').childFile('app_en.arb')
          ..createSync(recursive: true)
          ..writeAsStringSync('{ "hello": "Hello world!" }');
        String pubspecFileContent = projectDir.childFile('pubspec.yaml').readAsStringSync();
        pubspecFileContent = pubspecFileContent.replaceFirst(RegExp(r'\nflutter\:'), '''
flutter:
  generate: true
''');
        projectDir.childFile('pubspec.yaml').writeAsStringSync(pubspecFileContent);
        projectDir.childFile('l10n.yaml').writeAsStringSync('synthetic-package: false');
        await runCommandIn(projectPath, 'get');
        expect(
          projectDir
              .childDirectory('lib')
              .childDirectory('l10n')
              .childFile('app_localizations.dart')
              .existsSync(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Pub:
            () => Pub(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
            ),
      },
    );

    testUsingContext(
      'set no plugins as usage value',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );
        removeGeneratedFiles(projectPath);

        final PackagesCommand command = await runCommandIn(projectPath, 'get');
        final PackagesGetCommand getCommand = command.subcommands['get']! as PackagesGetCommand;

        expect((await getCommand.usageValues).commandPackagesNumberPlugins, 0);
        expect(
          (await getCommand.unifiedAnalyticsUsageValues(
            'pub/get',
          )).eventData['packagesNumberPlugins'],
          0,
        );
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'set the number of plugins as usage value',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>[
            '--template=plugin',
            '--no-pub',
            '--platforms=ios,android,macos,windows',
          ],
        );
        final String exampleProjectPath = globals.fs.path.join(projectPath, 'example');

        final PackagesCommand command = await runCommandIn(exampleProjectPath, 'get');
        final PackagesGetCommand getCommand = command.subcommands['get']! as PackagesGetCommand;

        // A plugin example depends on the plugin itself, and integration_test.
        expect((await getCommand.usageValues).commandPackagesNumberPlugins, 2);
        expect(
          (await getCommand.unifiedAnalyticsUsageValues(
            'pub/get',
          )).eventData['packagesNumberPlugins'],
          2,
        );
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'indicate that the project is not a module in usage value',
      () async {
        final String projectPath = await createProject(tempDir, arguments: <String>['--no-pub']);
        removeGeneratedFiles(projectPath);

        final PackagesCommand command = await runCommandIn(projectPath, 'get');
        final PackagesGetCommand getCommand = command.subcommands['get']! as PackagesGetCommand;

        expect((await getCommand.usageValues).commandPackagesProjectModule, false);
        expect(
          (await getCommand.unifiedAnalyticsUsageValues(
            'pub/get',
          )).eventData['packagesProjectModule'],
          false,
        );
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'indicate that the project is a module in usage value',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );
        removeGeneratedFiles(projectPath);

        final PackagesCommand command = await runCommandIn(projectPath, 'get');
        final PackagesGetCommand getCommand = command.subcommands['get']! as PackagesGetCommand;

        expect((await getCommand.usageValues).commandPackagesProjectModule, true);
        expect(
          (await getCommand.unifiedAnalyticsUsageValues(
            'pub/get',
          )).eventData['packagesProjectModule'],
          true,
        );
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'indicate that Android project reports v2 in usage value',
      () async {
        final String projectPath = await createProject(tempDir, arguments: <String>['--no-pub']);
        removeGeneratedFiles(projectPath);

        final PackagesCommand command = await runCommandIn(projectPath, 'get');
        final PackagesGetCommand getCommand = command.subcommands['get']! as PackagesGetCommand;

        expect((await getCommand.usageValues).commandPackagesAndroidEmbeddingVersion, 'v2');
        expect(
          (await getCommand.unifiedAnalyticsUsageValues(
            'pub/get',
          )).eventData['packagesAndroidEmbeddingVersion'],
          'v2',
        );
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'upgrade fetches packages',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--no-pub', '--template=module'],
        );
        removeGeneratedFiles(projectPath);

        await runCommandIn(projectPath, 'upgrade');

        expectDependenciesResolved(projectPath);
        expectZeroPluginsInjected(projectPath);
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'get fetches packages and injects plugin',
      () async {
        final String projectPath = await createProjectWithPlugin(
          'path_provider',
          arguments: <String>['--no-pub', '--template=module'],
        );
        removeGeneratedFiles(projectPath);

        await runCommandIn(projectPath, 'get');

        expectDependenciesResolved(projectPath);
        expectModulePluginInjected(projectPath, includeLegacyPluginsList: false);
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
        FeatureFlags: enableExplicitPackageDependencies,
      },
    );

    testUsingContext(
      'get fetches packages and injects plugin in plugin project',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--template=plugin', '--no-pub', '--platforms=ios,android'],
        );
        final String exampleProjectPath = globals.fs.path.join(projectPath, 'example');
        removeGeneratedFiles(projectPath);
        removeGeneratedFiles(exampleProjectPath);

        await runCommandIn(projectPath, 'get');

        expectDependenciesResolved(projectPath);

        await runCommandIn(exampleProjectPath, 'get');

        expectDependenciesResolved(exampleProjectPath);
        expectPluginInjected(exampleProjectPath, includeLegacyPluginsList: false);
      },
      overrides: <Type, Generator>{
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
        FeatureFlags: enableExplicitPackageDependencies,
      },
    );

    testUsingContext(
      'get explicit-packages-resolution omits ".flutter-plugins"',
      () async {
        final String projectPath = await createProject(
          tempDir,
          arguments: <String>['--template=plugin', '--no-pub', '--platforms=ios,android'],
        );
        final String exampleProjectPath = globals.fs.path.join(projectPath, 'example');
        removeGeneratedFiles(projectPath);
        removeGeneratedFiles(exampleProjectPath);

        // Running flutter packages get also resolves the dependencies in the example/ project.
        await runCommandIn(projectPath, 'get');

        expectDependenciesResolved(projectPath);
        expectDependenciesResolved(exampleProjectPath);
        expectPluginInjected(exampleProjectPath, includeLegacyPluginsList: false);
      },
      overrides: <Type, Generator>{
        FeatureFlags: () => TestFeatureFlags(isExplicitPackageDependenciesEnabled: true),
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );
  });

  group('packages test/pub', () {
    late FakeProcessManager processManager;
    late FakeStdio mockStdio;

    setUp(() {
      processManager = FakeProcessManager.empty();
      mockStdio = FakeStdio()..stdout.terminalColumns = 80;
    });

    testUsingContext(
      'test without bot',
      () async {
        Cache.flutterRoot = '';
        globals.fs.directory('/packages/flutter_tools').createSync(recursive: true);
        globals.fs.file('pubspec.yaml').createSync();
        processManager.addCommand(
          const FakeCommand(
            command: <String>[
              '/bin/cache/dart-sdk/bin/dart',
              'pub',
              '--suppress-analytics',
              'run',
              'test',
            ],
          ),
        );
        await createTestCommandRunner(PackagesCommand()).run(<String>['packages', 'test']);

        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        Platform: () => FakePlatform(environment: <String, String>{}),
        ProcessManager: () => processManager,
        Stdio: () => mockStdio,
        BotDetector: () => const FakeBotDetector(false),
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'test with bot',
      () async {
        Cache.flutterRoot = '';
        globals.fs.file('pubspec.yaml').createSync();
        processManager.addCommand(
          const FakeCommand(
            command: <String>[
              '/bin/cache/dart-sdk/bin/dart',
              'pub',
              '--suppress-analytics',
              '--trace',
              'run',
              'test',
            ],
          ),
        );
        await createTestCommandRunner(PackagesCommand()).run(<String>['packages', 'test']);

        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        Platform: () => FakePlatform(environment: <String, String>{}),
        ProcessManager: () => processManager,
        Stdio: () => mockStdio,
        BotDetector: () => const FakeBotDetector(true),
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'run pass arguments through to pub',
      () async {
        Cache.flutterRoot = '';
        globals.fs.file('pubspec.yaml').createSync();
        final IOSink stdin = IOSink(StreamController<List<int>>().sink);
        processManager.addCommand(
          FakeCommand(
            command: const <String>[
              '/bin/cache/dart-sdk/bin/dart',
              'pub',
              '--suppress-analytics',
              'run',
              '--foo',
              'bar',
            ],
            stdin: stdin,
          ),
        );
        await createTestCommandRunner(
          PackagesCommand(),
        ).run(<String>['packages', '--verbose', 'pub', 'run', '--foo', 'bar']);

        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        Platform: () => FakePlatform(environment: <String, String>{}),
        ProcessManager: () => processManager,
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'token pass arguments through to pub',
      () async {
        Cache.flutterRoot = '';
        globals.fs.file('pubspec.yaml').createSync();
        final IOSink stdin = IOSink(StreamController<List<int>>().sink);
        processManager.addCommand(
          FakeCommand(
            command: const <String>[
              '/bin/cache/dart-sdk/bin/dart',
              'pub',
              '--suppress-analytics',
              'token',
              'list',
            ],
            stdin: stdin,
          ),
        );
        await createTestCommandRunner(
          PackagesCommand(),
        ).run(<String>['packages', '--verbose', 'pub', 'token', 'list']);

        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        Platform: () => FakePlatform(environment: <String, String>{}),
        ProcessManager: () => processManager,
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );

    testUsingContext(
      'upgrade does not check for pubspec.yaml if -h/--help is passed',
      () async {
        Cache.flutterRoot = '';
        processManager.addCommand(
          FakeCommand(
            command: const <String>[
              '/bin/cache/dart-sdk/bin/dart',
              'pub',
              '--suppress-analytics',
              'upgrade',
              '-h',
            ],
            stdin: IOSink(StreamController<List<int>>().sink),
          ),
        );
        await createTestCommandRunner(PackagesCommand()).run(<String>['pub', 'upgrade', '-h']);

        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        Platform: () => FakePlatform(environment: <String, String>{}),
        ProcessManager: () => processManager,
        Stdio: () => mockStdio,
        Pub:
            () => Pub.test(
              fileSystem: globals.fs,
              logger: globals.logger,
              processManager: globals.processManager,
              usage: globals.flutterUsage,
              botDetector: globals.botDetector,
              platform: globals.platform,
              stdio: mockStdio,
            ),
      },
    );
  });
}
