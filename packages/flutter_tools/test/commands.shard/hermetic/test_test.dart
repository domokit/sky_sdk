// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/async_guard.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/test.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/native_assets.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';
import 'package:flutter_tools/src/test/coverage_collector.dart';
import 'package:flutter_tools/src/test/runner.dart';
import 'package:flutter_tools/src/test/test_device.dart';
import 'package:flutter_tools/src/test/test_time_recorder.dart';
import 'package:flutter_tools/src/test/test_wrapper.dart';
import 'package:flutter_tools/src/test/watcher.dart';
import 'package:flutter_tools/src/web/compile.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_devices.dart';
import '../../src/fake_vm_services.dart';
import '../../src/logging_logger.dart';
import '../../src/test_flutter_command_runner.dart';

final String _flutterToolsPackageConfigContents = json.encode(<String, Object>{
  'configVersion': 2,
  'packages': <Map<String, Object>>[
    <String, String>{
      'name': 'ffi',
      'rootUri': 'file:///path/to/pubcache/.pub-cache/hosted/pub.dev/ffi-2.1.2',
      'packageUri': 'lib/',
      'languageVersion': '3.3',
    },
    <String, String>{
      'name': 'test',
      'rootUri': 'file:///path/to/pubcache/.pub-cache/hosted/pub.dev/test-1.24.9',
      'packageUri': 'lib/',
      'languageVersion': '3.0',
    },
    <String, String>{
      'name': 'test_api',
      'rootUri': 'file:///path/to/pubcache/.pub-cache/hosted/pub.dev/test_api-0.6.1',
      'packageUri': 'lib/',
      'languageVersion': '3.0',
    },
    <String, String>{
      'name': 'test_core',
      'rootUri': 'file:///path/to/pubcache/.pub-cache/hosted/pub.dev/test_core-0.5.9',
      'packageUri': 'lib/',
      'languageVersion': '3.0',
    },
  ],
  'generated': '2021-02-24T07:55:20.084834Z',
  'generator': 'pub',
  'generatorVersion': '2.13.0-68.0.dev',
});
const String _pubspecContents = '''
dev_dependencies:
  flutter_test:
    sdk: flutter
  integration_test:
    sdk: flutter''';
final String _packageConfigContents = json.encode(<String, Object>{
  'configVersion': 2,
  'packages': <Map<String, Object>>[
    <String, String>{
      'name': 'test_api',
      'rootUri': 'file:///path/to/pubcache/.pub-cache/hosted/pub.dartlang.org/test_api-0.2.19',
      'packageUri': 'lib/',
      'languageVersion': '2.12',
    },
    <String, String>{
      'name': 'integration_test',
      'rootUri': 'file:///path/to/flutter/packages/integration_test',
      'packageUri': 'lib/',
      'languageVersion': '2.12',
    },
  ],
  'generated': '2021-02-24T07:55:20.084834Z',
  'generator': 'pub',
  'generatorVersion': '2.13.0-68.0.dev',
});

void main() {
  Cache.disableLocking();
  late MemoryFileSystem fs;
  late LoggingLogger logger;

  setUp(() {
    fs = MemoryFileSystem.test(
      style: globals.platform.isWindows ? FileSystemStyle.windows : FileSystemStyle.posix,
    );

    final Directory package = fs.directory('package');
    package.childFile('pubspec.yaml').createSync(recursive: true);
    package.childFile('pubspec.yaml').writeAsStringSync(_pubspecContents);
    (package.childDirectory('.dart_tool').childFile('package_config.json')
      ..createSync(recursive: true)).writeAsStringSync(_packageConfigContents);
    package.childDirectory('test').childFile('some_test.dart').createSync(recursive: true);
    package
        .childDirectory('integration_test')
        .childFile('some_integration_test.dart')
        .createSync(recursive: true);

    final File flutterToolsPackageConfigFile = fs
        .directory(fs.path.join(getFlutterRoot(), 'packages', 'flutter_tools'))
        .childDirectory('.dart_tool')
        .childFile('package_config.json');
    flutterToolsPackageConfigFile.createSync(recursive: true);
    flutterToolsPackageConfigFile.writeAsStringSync(_flutterToolsPackageConfigContents);

    fs.currentDirectory = package.path;

    logger = LoggingLogger();
  });

  testUsingContext(
    'Missing dependencies in pubspec',
    () async {
      // Clear the dependencies already added in [setUp].
      fs.file('pubspec.yaml').writeAsStringSync('');
      fs.directory('.dart_tool').childFile('package_config.json').writeAsStringSync('');

      final FakePackageTest fakePackageTest = FakePackageTest();
      final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      expect(() => commandRunner.run(const <String>['test', '--no-pub']), throwsToolExit());
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'Missing dependencies in pubspec for integration tests',
    () async {
      // Only use the flutter_test dependency, integration_test is deliberately
      // absent.
      fs.file('pubspec.yaml').writeAsStringSync('''
dev_dependencies:
  flutter_test:
    sdk: flutter
    ''');
      fs
          .directory('.dart_tool')
          .childFile('package_config.json')
          .writeAsStringSync(
            json.encode(<String, Object>{
              'configVersion': 2,
              'packages': <Map<String, Object>>[
                <String, String>{
                  'name': 'test_api',
                  'rootUri':
                      'file:///path/to/pubcache/.pub-cache/hosted/pub.dartlang.org/test_api-0.2.19',
                  'packageUri': 'lib/',
                  'languageVersion': '2.12',
                },
              ],
              'generated': '2021-02-24T07:55:20.084834Z',
              'generator': 'pub',
              'generatorVersion': '2.13.0-68.0.dev',
            }),
          );
      final FakePackageTest fakePackageTest = FakePackageTest();
      final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      expect(
        () => commandRunner.run(const <String>['test', '--no-pub', 'integration_test']),
        throwsToolExit(),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'Confirmation that the reporter, timeout, and concurrency args are not set by default',
    () async {
      final FakePackageTest fakePackageTest = FakePackageTest();

      final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub']);

      expect(fakePackageTest.lastArgs, isNot(contains('-r')));
      expect(fakePackageTest.lastArgs, isNot(contains('compact')));
      expect(fakePackageTest.lastArgs, isNot(contains('--timeout')));
      expect(fakePackageTest.lastArgs, isNot(contains('30s')));
      expect(fakePackageTest.lastArgs, isNot(contains('--concurrency')));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
    },
  );

  group('shard-index and total-shards', () {
    testUsingContext(
      'with the params they are Piped to package:test',
      () async {
        final FakePackageTest fakePackageTest = FakePackageTest();

        final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>[
          'test',
          '--total-shards=1',
          '--shard-index=2',
          '--no-pub',
        ]);

        expect(fakePackageTest.lastArgs, contains('--total-shards=1'));
        expect(fakePackageTest.lastArgs, contains('--shard-index=2'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      },
    );

    testUsingContext(
      'without the params they not Piped to package:test',
      () async {
        final FakePackageTest fakePackageTest = FakePackageTest();

        final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub']);

        expect(fakePackageTest.lastArgs, isNot(contains('--total-shards')));
        expect(fakePackageTest.lastArgs, isNot(contains('--shard-index')));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      },
    );
  });

  group('--reporter/-r', () {
    String? passedReporter(List<String> args) {
      final int i = args.indexOf('-r');
      if (i < 0) {
        expect(args, isNot(contains('--reporter')));
        expect(args, isNot(contains(matches(RegExp(r'^(-r|--reporter=)')))));
        return null;
      } else {
        return args[i + 1];
      }
    }

    Future<void> expectPassesReporter(String value) async {
      final FakePackageTest fakePackageTest = FakePackageTest();
      final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(<String>['test', '--no-pub', '-r', value]);
      expect(passedReporter(fakePackageTest.lastArgs!), equals(value));

      await commandRunner.run(<String>['test', '--no-pub', '-r$value']);
      expect(passedReporter(fakePackageTest.lastArgs!), equals(value));

      await commandRunner.run(<String>['test', '--no-pub', '--reporter', value]);
      expect(passedReporter(fakePackageTest.lastArgs!), equals(value));

      await commandRunner.run(<String>['test', '--no-pub', '--reporter=$value']);
      expect(passedReporter(fakePackageTest.lastArgs!), equals(value));
    }

    testUsingContext(
      'accepts valid values and passes them through',
      () async {
        await expectPassesReporter('compact');
        await expectPassesReporter('expanded');
        await expectPassesReporter('failures-only');
        await expectPassesReporter('github');
        await expectPassesReporter('json');
        await expectPassesReporter('silent');
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      },
    );

    testUsingContext(
      'by default, passes no reporter',
      () async {
        final FakePackageTest fakePackageTest = FakePackageTest();
        final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(<String>['test', '--no-pub']);
        expect(passedReporter(fakePackageTest.lastArgs!), isNull);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      },
    );
  });

  testUsingContext(
    'Supports coverage and machine',
    () async {
      final FakePackageTest fakePackageTest = FakePackageTest();

      final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      expect(
        () => commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '--machine',
          '--coverage',
          '--',
          'test/fake_test.dart',
        ]),
        throwsA(isA<ToolExit>().having((ToolExit toolExit) => toolExit.message, 'message', isNull)),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
    },
  );

  testUsingContext(
    'Coverage provides current library name to Coverage Collector by default',
    () async {
      const String currentPackageName = '';
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          FakeVmServiceRequest(
            method: 'getVM',
            jsonResponse:
                (VM.parse(<String, Object>{})!
                      ..isolates = <IsolateRef>[
                        IsolateRef.parse(<String, Object>{'id': '1'})!,
                      ])
                    .toJson(),
          ),
          FakeVmServiceRequest(
            method: 'getSourceReport',
            args: <String, Object>{
              'isolateId': '1',
              'reports': <Object>['Coverage'],
              'forceCompile': true,
              'reportLines': true,
              'libraryFilters': <String>['package:$currentPackageName/'],
              'librariesAlreadyCompiled': <Object>[],
            },
            jsonResponse: SourceReport(ranges: <SourceReportRange>[]).toJson(),
          ),
        ],
      );
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0, null, fakeVmServiceHost);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);
      await commandRunner.run(const <String>[
        'test',
        '--no-pub',
        '--coverage',
        '--',
        'test/some_test.dart',
      ]);
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
      expect((testRunner.lastTestWatcher! as CoverageCollector).libraryNames, <String>{
        currentPackageName,
      });
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
    },
  );

  testUsingContext(
    'Coverage provides library names matching regexps to Coverage Collector',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          FakeVmServiceRequest(
            method: 'getVM',
            jsonResponse:
                (VM.parse(<String, Object>{})!
                      ..isolates = <IsolateRef>[
                        IsolateRef.parse(<String, Object>{'id': '1'})!,
                      ])
                    .toJson(),
          ),
          FakeVmServiceRequest(
            method: 'getSourceReport',
            args: <String, Object>{
              'isolateId': '1',
              'reports': <Object>['Coverage'],
              'forceCompile': true,
              'reportLines': true,
              'libraryFilters': <String>['package:test_api/'],
              'librariesAlreadyCompiled': <Object>[],
            },
            jsonResponse: SourceReport(ranges: <SourceReportRange>[]).toJson(),
          ),
        ],
      );
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0, null, fakeVmServiceHost);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);
      await commandRunner.run(const <String>[
        'test',
        '--no-pub',
        '--coverage',
        '--coverage-package=^test',
        '--',
        'test/some_test.dart',
      ]);
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
      expect((testRunner.lastTestWatcher! as CoverageCollector).libraryNames, <String>{'test_api'});
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
    },
  );

  testUsingContext(
    'Coverage provides error message if regular expression syntax is invalid',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      expect(
        () => commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '--coverage',
          r'--coverage-package="$+"',
          '--',
          'test/some_test.dart',
        ]),
        throwsToolExit(
          message: RegExp(
            r'Regular expression syntax is invalid. FormatException: Nothing to repeat[ \t]*"\$\+"',
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  group('Pipes to package:test', () {
    Future<void> expectPassesArgument(String value, [String? passValue]) async {
      final FakePackageTest fakePackageTest = FakePackageTest();
      final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(<String>['test', '--no-pub', value]);
      expect(fakePackageTest.lastArgs, contains(passValue ?? value));
    }

    testUsingContext(
      'passes various CLI options through to package:test',
      () async {
        await expectPassesArgument('--start-paused', '--pause-after-load');
        await expectPassesArgument('--fail-fast');
        await expectPassesArgument('--run-skipped');
        await expectPassesArgument('--test-randomize-ordering-seed=random');
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      },
    );
  });

  testUsingContext(
    'Pipes enable-vmService',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>[
        'test',
        '--no-pub',
        '--enable-vmservice',
        '--',
        'test/fake_test.dart',
      ]);
      expect(testRunner.lastEnableVmServiceValue, true);

      await commandRunner.run(const <String>[
        'test',
        '--no-pub',
        '--start-paused',
        '--no-enable-vmservice',
        '--',
        'test/fake_test.dart',
      ]);
      expect(testRunner.lastEnableVmServiceValue, true);

      await commandRunner.run(const <String>['test', '--no-pub', '--', 'test/fake_test.dart']);
      expect(testRunner.lastEnableVmServiceValue, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
    },
  );

  testUsingContext(
    'Generates a satisfactory test runner package_config.json when --experimental-faster-testing is set',
    () async {
      final TestCommand testCommand = TestCommand();
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      bool caughtToolExit = false;
      await asyncGuard<void>(
        () => commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '--experimental-faster-testing',
          '--',
          'test/fake_test.dart',
          'test/fake_test_2.dart',
        ]),
        onError: (Object error) async {
          expect(error, isA<ToolExit>());
          // We expect this message because we are using a fake ProcessManager.
          expect((error as ToolExit).message, contains('the Dart compiler exited unexpectedly.'));
          caughtToolExit = true;

          final File isolateSpawningTesterPackageConfigFile = fs
              .directory(fs.path.join('build', 'isolate_spawning_tester'))
              .childDirectory('.dart_tool')
              .childFile('package_config.json');
          expect(isolateSpawningTesterPackageConfigFile.existsSync(), true);
          // We expect [isolateSpawningTesterPackageConfigFile] to contain the
          // union of the packages in [_packageConfigContents] and
          // [_flutterToolsPackageConfigContents].
          expect(
            isolateSpawningTesterPackageConfigFile.readAsStringSync().contains(
              '"name": "integration_test"',
            ),
            true,
          );
          expect(
            isolateSpawningTesterPackageConfigFile.readAsStringSync().contains('"name": "ffi"'),
            true,
          );
          expect(
            isolateSpawningTesterPackageConfigFile.readAsStringSync().contains('"name": "test"'),
            true,
          );
          expect(
            isolateSpawningTesterPackageConfigFile.readAsStringSync().contains(
              '"name": "test_api"',
            ),
            true,
          );
          expect(
            isolateSpawningTesterPackageConfigFile.readAsStringSync().contains(
              '"name": "test_core"',
            ),
            true,
          );
        },
      );
      expect(caughtToolExit, true);
    },
    overrides: <Type, Generator>{
      AnsiTerminal: () => _FakeTerminal(),
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  testUsingContext(
    'Pipes specified arguments to package:test when --experimental-faster-testing is set',
    () async {
      final TestCommand testCommand = TestCommand();
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      bool caughtToolExit = false;
      await asyncGuard<void>(
        () => commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '--experimental-faster-testing',
          '--reporter=compact',
          '--file-reporter=json:reports/tests.json',
          '--timeout=100',
          '--concurrency=3',
          '--name=name1',
          '--plain-name=name2',
          '--test-randomize-ordering-seed=random',
          '--tags=tag1',
          '--exclude-tags=tag2',
          '--fail-fast',
          '--run-skipped',
          '--total-shards=1',
          '--shard-index=1',
          '--',
          'test/fake_test.dart',
          'test/fake_test_2.dart',
        ]),
        onError: (Object error) async {
          expect(error, isA<ToolExit>());
          // We expect this message because we are using a fake ProcessManager.
          expect((error as ToolExit).message, contains('the Dart compiler exited unexpectedly.'));
          caughtToolExit = true;

          final File childTestIsolateSpawnerSourceFile = fs
              .directory(fs.path.join('build', 'isolate_spawning_tester'))
              .childFile('child_test_isolate_spawner.dart');
          expect(childTestIsolateSpawnerSourceFile.existsSync(), true);
          expect(
            childTestIsolateSpawnerSourceFile.readAsStringSync().contains('''
const List<String> packageTestArgs = <String>[
  '--no-color',
  '-r',
  'compact',
  '--file-reporter=json:reports/tests.json',
  '--timeout',
  '100',
  '--concurrency=3',
  '--name',
  'name1',
  '--plain-name',
  'name2',
  '--test-randomize-ordering-seed=random',
  '--tags',
  'tag1',
  '--exclude-tags',
  'tag2',
  '--fail-fast',
  '--run-skipped',
  '--total-shards=1',
  '--shard-index=1',
  '--chain-stack-traces',
];
'''),
            true,
          );
        },
      );
      expect(caughtToolExit, true);
    },
    overrides: <Type, Generator>{
      AnsiTerminal: () => _FakeTerminal(),
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  testUsingContext(
    'Only passes --no-color and --chain-stack-traces to package:test by default when --experimental-faster-testing is set',
    () async {
      final TestCommand testCommand = TestCommand();
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      bool caughtToolExit = false;
      await asyncGuard<void>(
        () => commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '--experimental-faster-testing',
          '--',
          'test/fake_test.dart',
          'test/fake_test_2.dart',
        ]),
        onError: (Object error) async {
          expect(error, isA<ToolExit>());
          // We expect this message because we are using a fake ProcessManager.
          expect((error as ToolExit).message, contains('the Dart compiler exited unexpectedly.'));
          caughtToolExit = true;

          final File childTestIsolateSpawnerSourceFile = fs
              .directory(fs.path.join('build', 'isolate_spawning_tester'))
              .childFile('child_test_isolate_spawner.dart');
          expect(childTestIsolateSpawnerSourceFile.existsSync(), true);
          expect(
            childTestIsolateSpawnerSourceFile.readAsStringSync().contains('''
const List<String> packageTestArgs = <String>[
  '--no-color',
  '--chain-stack-traces',
];
'''),
            true,
          );
        },
      );
      expect(caughtToolExit, true);
    },
    overrides: <Type, Generator>{
      AnsiTerminal: () => _FakeTerminal(),
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  testUsingContext(
    'Verbose prints phase timings',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(
        0,
        const Duration(milliseconds: 1),
      );

      final TestCommand testCommand = TestCommand(testRunner: testRunner, verbose: true);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub', '--', 'test/fake_test.dart']);

      // Expect one message for each phase.
      final List<String> logPhaseMessages =
          logger.messages.where((String m) => m.startsWith('Runtime for phase ')).toList();
      expect(logPhaseMessages, hasLength(TestTimePhases.values.length));

      // As we force the `runTests` command to take at least 1 ms expect at least
      // one phase to take a non-zero amount of time.
      final List<String> logPhaseMessagesNonZero =
          logPhaseMessages.where((String m) => !m.contains(Duration.zero.toString())).toList();
      expect(logPhaseMessagesNonZero, isNotEmpty);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'Non-verbose does not prints phase timings',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(
        0,
        const Duration(milliseconds: 1),
      );

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub', '--', 'test/fake_test.dart']);

      final List<String> logPhaseMessages =
          logger.messages.where((String m) => m.startsWith('Runtime for phase ')).toList();
      expect(logPhaseMessages, isEmpty);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'Pipes different args when running Integration Tests',
    () async {
      final FakePackageTest fakePackageTest = FakePackageTest();

      final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub', 'integration_test']);

      expect(fakePackageTest.lastArgs, contains('--concurrency=1'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager:
          () => _FakeDeviceManager(<Device>[
            FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
          ]),
    },
  );

  testUsingContext(
    'Overrides concurrency when running Integration Tests',
    () async {
      final FakePackageTest fakePackageTest = FakePackageTest();

      final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>[
        'test',
        '--no-pub',
        '--concurrency=100',
        'integration_test',
      ]);

      expect(fakePackageTest.lastArgs, contains('--concurrency=1'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager:
          () => _FakeDeviceManager(<Device>[
            FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
          ]),
    },
  );

  group('Detecting Integration Tests', () {
    testUsingContext(
      'when integration_test is not passed',
      () async {
        final FakePackageTest fakePackageTest = FakePackageTest();

        final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub']);

        expect(testCommand.isIntegrationTest, false);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager:
            () => _FakeDeviceManager(<Device>[
              FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
            ]),
      },
    );

    testUsingContext(
      'when integration_test is passed',
      () async {
        final FakePackageTest fakePackageTest = FakePackageTest();

        final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub', 'integration_test']);

        expect(testCommand.isIntegrationTest, true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager:
            () => _FakeDeviceManager(<Device>[
              FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
            ]),
      },
    );

    testUsingContext(
      'when relative path to integration test is passed',
      () async {
        final FakePackageTest fakePackageTest = FakePackageTest();

        final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>[
          'test',
          '--no-pub',
          'integration_test/some_integration_test.dart',
        ]);

        expect(testCommand.isIntegrationTest, true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager:
            () => _FakeDeviceManager(<Device>[
              FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
            ]),
      },
    );

    testUsingContext(
      'when absolute path to integration test is passed',
      () async {
        final FakePackageTest fakePackageTest = FakePackageTest();

        final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '/package/integration_test/some_integration_test.dart',
        ]);

        expect(testCommand.isIntegrationTest, true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager:
            () => _FakeDeviceManager(<Device>[
              FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
            ]),
      },
    );

    testUsingContext(
      'when absolute unnormalized path to integration test is passed',
      () async {
        final FakePackageTest fakePackageTest = FakePackageTest();

        final TestCommand testCommand = TestCommand(testWrapper: fakePackageTest);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '/package/../package/integration_test/some_integration_test.dart',
        ]);

        expect(testCommand.isIntegrationTest, true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager:
            () => _FakeDeviceManager(<Device>[
              FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
            ]),
      },
    );

    testUsingContext(
      'when both test and integration test are passed',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        expect(
          () => commandRunner.run(const <String>[
            'test',
            '--no-pub',
            'test/some_test.dart',
            'integration_test/some_integration_test.dart',
          ]),
          throwsToolExit(),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  group('Required artifacts', () {
    testUsingContext(
      'for default invocation',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub']);

        expect(await testCommand.requiredArtifacts, isEmpty);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'when platform is chrome',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub', '--platform=chrome']);

        expect(await testCommand.requiredArtifacts, <DevelopmentArtifact>[DevelopmentArtifact.web]);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'Overrides concurrency when running web tests',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '--concurrency=100',
          '--platform=chrome',
        ]);

        expect(testRunner.lastConcurrency, 1);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'when running integration tests',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub', 'integration_test']);

        expect(await testCommand.requiredArtifacts, <DevelopmentArtifact>[
          DevelopmentArtifact.universal,
          DevelopmentArtifact.androidGenSnapshot,
        ]);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        DeviceManager:
            () => _FakeDeviceManager(<Device>[
              FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
            ]),
      },
    );
  });

  testUsingContext(
    'Integration tests when no devices are connected',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      expect(
        () => commandRunner.run(const <String>['test', '--no-pub', 'integration_test']),
        throwsToolExit(),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  // TODO(jiahaog): Remove this when web is supported. https://github.com/flutter/flutter/issues/66264
  testUsingContext(
    'Integration tests when only web devices are connected',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      expect(
        () => commandRunner.run(const <String>['test', '--no-pub', 'integration_test']),
        throwsToolExit(),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => _FakeDeviceManager(<Device>[FakeDevice('ephemeral', 'ephemeral')]),
    },
  );

  testUsingContext(
    'Integration tests set the correct dart-defines',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub', 'integration_test']);

      expect(
        testRunner.lastDebuggingOptionsValue.buildInfo.dartDefines,
        contains('INTEGRATION_TEST_SHOULD_REPORT_RESULTS_TO_NATIVE=false'),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager:
          () => _FakeDeviceManager(<Device>[
            FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
          ]),
    },
  );

  testUsingContext(
    'Integration tests given flavor',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>[
        'test',
        '--no-pub',
        '--flavor',
        'dev',
        'integration_test',
      ]);

      expect(testRunner.lastDebuggingOptionsValue.buildInfo.flavor, contains('dev'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager:
          () => _FakeDeviceManager(<Device>[
            FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android, supportsFlavors: true),
          ]),
    },
  );

  testUsingContext(
    'Builds the asset manifest by default',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub']);

      final bool fileExists = await fs.isFile(
        globals.fs.path.join('build', 'unit_test_assets', 'AssetManifest.bin'),
      );
      expect(fileExists, true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  testUsingContext(
    'builds asset bundle using --flavor',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);
      fs.file('vanilla.txt').writeAsStringSync('vanilla');
      fs.file('orange.txt').writeAsStringSync('orange');
      fs.file('pubspec.yaml').writeAsStringSync('''
flutter:
  assets:
    - path: vanilla.txt
      flavors:
        - vanilla
    - path: orange.txt
      flavors:
        - orange
dev_dependencies:
  flutter_test:
    sdk: flutter
  integration_test:
    sdk: flutter''');
      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub', '--flavor', 'vanilla']);

      final bool vanillaExists = await fs.isFile(
        globals.fs.path.join('build', 'unit_test_assets', 'vanilla.txt'),
      );
      expect(vanillaExists, true, reason: 'vanilla.txt should be bundled');
      final bool orangeExists = await fs.isFile(
        globals.fs.path.join('build', 'unit_test_assets', 'orange.txt'),
      );
      expect(orangeExists, false, reason: 'orange.txt should not be bundled');
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  testUsingContext(
    'correctly considers --flavor when validating the cached asset bundle',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);
      fs.file('vanilla.txt').writeAsStringSync('vanilla');
      fs.file('flavorless.txt').writeAsStringSync('flavorless');
      fs.file('pubspec.yaml').writeAsStringSync('''
flutter:
  assets:
    - path: vanilla.txt
      flavors:
        - vanilla
    - flavorless.txt
dev_dependencies:
  flutter_test:
    sdk: flutter
  integration_test:
    sdk: flutter''');
      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      const List<String> buildArgsFlavorless = <String>['test', '--no-pub'];

      const List<String> buildArgsVanilla = <String>['test', '--no-pub', '--flavor', 'vanilla'];

      final File builtVanillaAssetFile = fs.file(
        fs.path.join('build', 'unit_test_assets', 'vanilla.txt'),
      );
      final File builtFlavorlessAssetFile = fs.file(
        fs.path.join('build', 'unit_test_assets', 'flavorless.txt'),
      );

      await commandRunner.run(buildArgsVanilla);
      await commandRunner.run(buildArgsFlavorless);

      expect(builtVanillaAssetFile, isNot(exists));
      expect(builtFlavorlessAssetFile, exists);

      await commandRunner.run(buildArgsVanilla);

      expect(builtVanillaAssetFile, exists);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.empty(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  testUsingContext(
    "Don't build the asset manifest if --no-test-assets if informed",
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub', '--no-test-assets']);

      final bool fileExists = await fs.isFile(
        globals.fs.path.join('build', 'unit_test_assets', 'AssetManifest.bin'),
      );
      expect(fileExists, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  testUsingContext(
    'Rebuild the asset bundle if an asset file has changed since previous build',
    () async {
      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);
      fs.file('asset.txt').writeAsStringSync('1');
      fs.file('pubspec.yaml').writeAsStringSync('''
flutter:
  assets:
    - asset.txt
dev_dependencies:
  flutter_test:
    sdk: flutter
  integration_test:
    sdk: flutter''');
      final TestCommand testCommand = TestCommand(testRunner: testRunner);
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub']);

      fs.file('asset.txt').writeAsStringSync('2');

      await commandRunner.run(const <String>['test', '--no-pub']);

      final String fileContent =
          fs
              .file(globals.fs.path.join('build', 'unit_test_assets', 'asset.txt'))
              .readAsStringSync();
      expect(fileContent, '2');
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.empty(),
      DeviceManager: () => _FakeDeviceManager(<Device>[]),
    },
  );

  group('Fatal Logs', () {
    testUsingContext(
      "doesn't fail when --fatal-warnings is set and no warning output",
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        try {
          await commandRunner.run(const <String>[
            'test',
            '--no-pub',
            '--${FlutterOptions.kFatalWarnings}',
          ]);
        } on Exception {
          fail('Unexpected exception thrown');
        }
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
    testUsingContext(
      'fails if --fatal-warnings specified and warnings emitted',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        testLogger.printWarning('Warning: Mild annoyance, Will Robinson!');
        expect(
          commandRunner.run(const <String>[
            'test',
            '--no-pub',
            '--${FlutterOptions.kFatalWarnings}',
          ]),
          throwsToolExit(
            message:
                'Logger received warning output during the run, and "--${FlutterOptions.kFatalWarnings}" is enabled.',
          ),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
    testUsingContext(
      'fails when --fatal-warnings is set and only errors emitted',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        testLogger.printError('Error: Danger Will Robinson!');
        expect(
          commandRunner.run(const <String>[
            'test',
            '--no-pub',
            '--${FlutterOptions.kFatalWarnings}',
          ]),
          throwsToolExit(
            message:
                'Logger received error output during the run, and "--${FlutterOptions.kFatalWarnings}" is enabled.',
          ),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  group('File Reporter', () {
    testUsingContext(
      'defaults to unset null value',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub']);
        expect(testRunner.lastFileReporterValue, null);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'when set --file-reporter value is passed on',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>[
          'test',
          '--no-pub',
          '--file-reporter=json:out.jsonl',
        ]);
        expect(testRunner.lastFileReporterValue, 'json:out.jsonl');
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'Enables Impeller',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub', '--enable-impeller']);
        expect(testRunner.lastDebuggingOptionsValue.enableImpeller, ImpellerStatus.enabled);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'Passes web renderer into debugging options',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(<String>[
          'test',
          '--no-pub',
          '--platform=chrome',
          ...WebRendererMode.canvaskit.toCliDartDefines,
        ]);
        expect(testRunner.lastDebuggingOptionsValue.webRenderer, WebRendererMode.canvaskit);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'Web renderer defaults to Skwasm when using wasm',
      () async {
        final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

        final TestCommand testCommand = TestCommand(testRunner: testRunner);
        final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

        await commandRunner.run(const <String>['test', '--no-pub', '--platform=chrome', '--wasm']);
        expect(testRunner.lastDebuggingOptionsValue.webRenderer, WebRendererMode.skwasm);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  // Tests whether using a deprecated webRenderer toggles a warningText.
  Future<void> testWebRendererDeprecationMessage(WebRendererMode webRenderer) async {
    testUsingContext(
      'Using --web-renderer=${webRenderer.name} triggers a warningText.',
      () async {
        // Run the command so it parses --web-renderer, but ignore all errors.
        // We only care about the logger.
        try {
          final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);

          final TestCommand testCommand = TestCommand(testRunner: testRunner);
          await createTestCommandRunner(testCommand).run(<String>[
            'test',
            'web',
            '--no-pub',
            '--platform=chrome',
            ...webRenderer.toCliDartDefines,
          ]);
        } on ToolExit catch (error) {
          expect(error, isA<ToolExit>());
        }
        expect(
          logger.warningText,
          contains('See: https://docs.flutter.dev/to/web-html-renderer-deprecation'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        Logger: () => logger,
      },
    );
  }

  /// Do test all the deprecated WebRendererModes
  WebRendererMode.values
      .where((WebRendererMode mode) => mode.isDeprecated)
      .forEach(testWebRendererDeprecationMessage);

  testUsingContext(
    'Can test in a pub workspace',
    () async {
      final String root = fs.path.rootPrefix(fs.currentDirectory.absolute.path);
      final Directory package = fs.directory('${root}package').absolute;
      package.childFile('pubspec.yaml').createSync(recursive: true);
      package.childFile('pubspec.yaml').writeAsStringSync('''
workspace:
  - app/
''');

      final Directory app = package.childDirectory('app');
      app.createSync();
      app.childFile('pubspec.yaml').writeAsStringSync('''
$_pubspecContents
resolution: workspace
''');
      app.childDirectory('test').childFile('some_test.dart').createSync(recursive: true);
      app
          .childDirectory('integration_test')
          .childFile('some_integration_test.dart')
          .createSync(recursive: true);

      fs.currentDirectory = app;

      final FakeFlutterTestRunner testRunner = FakeFlutterTestRunner(0);
      final FakePackageTest fakePackageTest = FakePackageTest();
      final TestCommand testCommand = TestCommand(
        testWrapper: fakePackageTest,
        testRunner: testRunner,
      );
      final CommandRunner<void> commandRunner = createTestCommandRunner(testCommand);

      await commandRunner.run(const <String>['test', '--no-pub']);
      expect(
        testRunner.lastDebuggingOptionsValue.buildInfo.packageConfigPath,
        package.childDirectory('.dart_tool').childFile('package_config.json').path,
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fs,
      ProcessManager: () => FakeProcessManager.any(),
      Logger: () => logger,
    },
  );
}

class FakeFlutterTestRunner implements FlutterTestRunner {
  FakeFlutterTestRunner(this.exitCode, [this.leastRunTime, this.fakeVmServiceHost]);

  int exitCode;
  Duration? leastRunTime;
  bool? lastEnableVmServiceValue;
  late DebuggingOptions lastDebuggingOptionsValue;
  String? lastFileReporterValue;
  String? lastReporterOption;
  int? lastConcurrency;
  TestWatcher? lastTestWatcher;
  FakeVmServiceHost? fakeVmServiceHost;

  @override
  Future<int> runTests(
    TestWrapper testWrapper,
    List<Uri> testFiles, {
    required DebuggingOptions debuggingOptions,
    List<String> names = const <String>[],
    List<String> plainNames = const <String>[],
    String? tags,
    String? excludeTags,
    bool enableVmService = false,
    bool ipv6 = false,
    bool machine = false,
    String? precompiledDillPath,
    Map<String, String>? precompiledDillFiles,
    bool updateGoldens = false,
    TestWatcher? watcher,
    required int? concurrency,
    String? testAssetDirectory,
    FlutterProject? flutterProject,
    String? icudtlPath,
    Directory? coverageDirectory,
    bool web = false,
    bool useWasm = false,
    String? randomSeed,
    String? reporter,
    String? fileReporter,
    String? timeout,
    bool failFast = false,
    bool runSkipped = false,
    int? shardIndex,
    int? totalShards,
    Device? integrationTestDevice,
    String? integrationTestUserIdentifier,
    TestTimeRecorder? testTimeRecorder,
    TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
    BuildInfo? buildInfo,
  }) async {
    lastEnableVmServiceValue = enableVmService;
    lastDebuggingOptionsValue = debuggingOptions;
    lastFileReporterValue = fileReporter;
    lastReporterOption = reporter;
    lastConcurrency = concurrency;
    lastTestWatcher = watcher;

    if (leastRunTime != null) {
      await Future<void>.delayed(leastRunTime!);
    }

    if (watcher is CoverageCollector) {
      await watcher.collectCoverage(
        TestTestDevice(),
        serviceOverride: fakeVmServiceHost?.vmService,
      );
    }

    return exitCode;
  }

  @override
  Never runTestsBySpawningLightweightEngines(
    List<Uri> testFiles, {
    required DebuggingOptions debuggingOptions,
    List<String> names = const <String>[],
    List<String> plainNames = const <String>[],
    String? tags,
    String? excludeTags,
    bool machine = false,
    bool updateGoldens = false,
    required int? concurrency,
    String? testAssetDirectory,
    FlutterProject? flutterProject,
    String? icudtlPath,
    String? randomSeed,
    String? reporter,
    String? fileReporter,
    String? timeout,
    bool failFast = false,
    bool runSkipped = false,
    int? shardIndex,
    int? totalShards,
    TestTimeRecorder? testTimeRecorder,
    TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
  }) {
    throw UnimplementedError();
  }
}

class TestTestDevice extends TestDevice {
  @override
  Future<void> get finished => Future<void>.delayed(const Duration(seconds: 1));

  @override
  Future<void> kill() => Future<void>.value();

  @override
  Future<Uri?> get vmServiceUri => Future<Uri?>.value(Uri());

  @override
  Future<StreamChannel<String>> start(String entrypointPath) {
    throw UnimplementedError();
  }
}

class FakePackageTest implements TestWrapper {
  List<String>? lastArgs;

  @override
  Future<void> main(List<String> args) async {
    lastArgs = args;
  }

  @override
  void registerPlatformPlugin(
    Iterable<Runtime> runtimes,
    FutureOr<PlatformPlugin> Function() platforms,
  ) {}
}

class _FakeTerminal extends Fake implements AnsiTerminal {
  @override
  final bool supportsColor = false;

  @override
  bool get isCliAnimationEnabled => supportsColor;
}

class _FakeDeviceManager extends DeviceManager {
  _FakeDeviceManager(this._connectedDevices) : super(logger: testLogger);

  final List<Device> _connectedDevices;

  @override
  Future<List<Device>> getAllDevices({DeviceDiscoveryFilter? filter}) async =>
      filteredDevices(filter);

  @override
  List<DeviceDiscovery> get deviceDiscoverers => <DeviceDiscovery>[];

  List<Device> filteredDevices(DeviceDiscoveryFilter? filter) {
    if (filter?.deviceConnectionInterface == DeviceConnectionInterface.wireless) {
      return <Device>[];
    }
    return _connectedDevices;
  }
}
