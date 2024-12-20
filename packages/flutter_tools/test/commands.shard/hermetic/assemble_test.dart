// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/assemble.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  Cache.disableLocking();
  Cache.flutterRoot = '';
  final StackTrace stackTrace = StackTrace.current;
  late FakeAnalytics fakeAnalytics;

  setUp(() {
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: MemoryFileSystem.test(),
      fakeFlutterVersion: FakeFlutterVersion(),
    );
  });

  testUsingContext(
    'flutter assemble can run a build',
    () async {
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(buildSystem: TestBuildSystem.all(BuildResult(success: true))),
      );
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        'debug_macos_bundle_flutter_assets',
      ]);

      expect(testLogger.traceText, contains('build succeeded.'));
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble can parse defines whose values contain =',
    () async {
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(
          buildSystem: TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, containsPair('FooBar', 'fizz=2'));
          }),
        ),
      );
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '-dFooBar=fizz=2',
        'debug_macos_bundle_flutter_assets',
      ]);

      expect(testLogger.traceText, contains('build succeeded.'));
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble can parse inputs',
    () async {
      final AssembleCommand command = AssembleCommand(
        buildSystem: TestBuildSystem.all(BuildResult(success: true), (
          Target target,
          Environment environment,
        ) {
          expect(environment.inputs, containsPair('Foo', 'Bar.txt'));
        }),
      );
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '-iFoo=Bar.txt',
        'debug_macos_bundle_flutter_assets',
      ]);

      expect(testLogger.traceText, contains('build succeeded.'));
      expect(await command.requiredArtifacts, isEmpty);
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble sets required artifacts from target platform',
    () async {
      final AssembleCommand command = AssembleCommand(
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
      );
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '-dTargetPlatform=darwin',
        '-dDarwinArchs=x86_64',
        'debug_macos_bundle_flutter_assets',
      ]);

      expect(await command.requiredArtifacts, <DevelopmentArtifact>{DevelopmentArtifact.macOS});
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'flutter assemble sends assemble-deferred-components',
    () async {
      final AssembleCommand command = AssembleCommand(
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
      );
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '-dTargetPlatform=android',
        '-dBuildMode=release',
        'android_aot_deferred_components_bundle_release_android-arm64',
      ]);
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.flutterBuildInfo(
            label: 'assemble-deferred-components',
            buildType: 'android',
            settings: 'android_aot_deferred_components_bundle_release_android-arm64',
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      Analytics: () => fakeAnalytics,
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble sends usage values correctly with platform',
    () async {
      final AssembleCommand command = AssembleCommand(
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
      );
      final CommandRunner<void> commandRunner = createTestCommandRunner(command);
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '-dTargetPlatform=darwin',
        '-dDarwinArchs=x86_64',
        'debug_macos_bundle_flutter_assets',
      ]);

      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.commandUsageValues(
            workflow: 'assemble',
            commandHasTerminal: false,
            buildBundleTargetPlatform: 'darwin',
            buildBundleIsModule: false,
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      Analytics: () => fakeAnalytics,
    },
  );

  testUsingContext(
    'flutter assemble throws ToolExit if not provided with output',
    () async {
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(buildSystem: TestBuildSystem.all(BuildResult(success: true))),
      );

      expect(
        commandRunner.run(<String>['assemble', 'debug_macos_bundle_flutter_assets']),
        throwsToolExit(),
      );
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble throws ToolExit if dart-defines are not base64 encoded',
    () async {
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(buildSystem: TestBuildSystem.all(BuildResult(success: true))),
      );

      final List<String> command = <String>[
        'assemble',
        '--output',
        'Output',
        '--DartDefines=flutter.inspector.structuredErrors%3Dtrue',
        'debug_macos_bundle_flutter_assets',
      ];
      expect(
        commandRunner.run(command),
        throwsToolExit(
          message:
              'Error parsing assemble command: your generated configuration may be out of date',
        ),
      );
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble throws ToolExit if called with non-existent rule',
    () async {
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(buildSystem: TestBuildSystem.all(BuildResult(success: true))),
      );

      expect(commandRunner.run(<String>['assemble', '-o Output', 'undefined']), throwsToolExit());
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble does not log stack traces during build failure',
    () async {
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(
          buildSystem: TestBuildSystem.all(
            BuildResult(
              success: false,
              exceptions: <String, ExceptionMeasurement>{
                'hello': ExceptionMeasurement('hello', 'bar', stackTrace, fatal: true),
              },
            ),
          ),
        ),
      );

      await expectLater(
        commandRunner.run(<String>['assemble', '-o Output', 'debug_macos_bundle_flutter_assets']),
        throwsToolExit(),
      );
      expect(testLogger.errorText, contains('Target hello failed: bar'));
      expect(testLogger.errorText, isNot(contains(stackTrace.toString())));
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble outputs JSON performance data to provided file',
    () async {
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(
          buildSystem: TestBuildSystem.all(
            BuildResult(
              success: true,
              performance: <String, PerformanceMeasurement>{
                'hello': PerformanceMeasurement(
                  target: 'hello',
                  analyticsName: 'bar',
                  elapsedMilliseconds: 123,
                  skipped: false,
                  succeeded: true,
                ),
              },
            ),
          ),
        ),
      );

      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '--performance-measurement-file=out.json',
        'debug_macos_bundle_flutter_assets',
      ]);

      expect(globals.fs.file('out.json'), exists);
      expect(
        json.decode(globals.fs.file('out.json').readAsStringSync()),
        containsPair('targets', contains(containsPair('name', 'bar'))),
      );
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble does not inject engine revision with local-engine',
    () async {
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(
          buildSystem: TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.engineVersion, isNull);
          }),
        ),
      );
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        'debug_macos_bundle_flutter_assets',
      ]);
    },
    overrides: <Type, Generator>{
      Artifacts:
          () => Artifacts.testLocalEngine(
            localEngine: 'out/host_release',
            localEngineHost: 'out/host_release',
          ),
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flutter assemble only writes input and output files when the values change',
    () async {
      final BuildSystem buildSystem = TestBuildSystem.list(<BuildResult>[
        BuildResult(
          success: true,
          inputFiles: <File>[globals.fs.file('foo')..createSync()],
          outputFiles: <File>[globals.fs.file('bar')..createSync()],
        ),
        BuildResult(
          success: true,
          inputFiles: <File>[globals.fs.file('foo')..createSync()],
          outputFiles: <File>[globals.fs.file('bar')..createSync()],
        ),
        BuildResult(
          success: true,
          inputFiles: <File>[globals.fs.file('foo'), globals.fs.file('fizz')..createSync()],
          outputFiles: <File>[
            globals.fs.file('bar'),
            globals.fs.file(globals.fs.path.join('.dart_tool', 'fizz2'))
              ..createSync(recursive: true),
          ],
        ),
      ]);
      final CommandRunner<void> commandRunner = createTestCommandRunner(
        AssembleCommand(buildSystem: buildSystem),
      );
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '--build-outputs=outputs',
        '--build-inputs=inputs',
        'debug_macos_bundle_flutter_assets',
      ]);

      final File inputs = globals.fs.file('inputs');
      final File outputs = globals.fs.file('outputs');
      expect(inputs.readAsStringSync(), contains('foo'));
      expect(outputs.readAsStringSync(), contains('bar'));

      final DateTime theDistantPast = DateTime(1991, 8, 23);
      inputs.setLastModifiedSync(theDistantPast);
      outputs.setLastModifiedSync(theDistantPast);
      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '--build-outputs=outputs',
        '--build-inputs=inputs',
        'debug_macos_bundle_flutter_assets',
      ]);

      expect(inputs.lastModifiedSync(), theDistantPast);
      expect(outputs.lastModifiedSync(), theDistantPast);

      await commandRunner.run(<String>[
        'assemble',
        '-o Output',
        '--build-outputs=outputs',
        '--build-inputs=inputs',
        'debug_macos_bundle_flutter_assets',
      ]);

      expect(inputs.readAsStringSync(), contains('foo'));
      expect(inputs.readAsStringSync(), contains('fizz'));
      expect(inputs.lastModifiedSync(), isNot(theDistantPast));
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext('writePerformanceData outputs performance data in JSON form', () {
    final List<PerformanceMeasurement> performanceMeasurement = <PerformanceMeasurement>[
      PerformanceMeasurement(
        analyticsName: 'foo',
        target: 'hidden',
        skipped: false,
        succeeded: true,
        elapsedMilliseconds: 123,
      ),
    ];
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File outFile = fileSystem.currentDirectory.childDirectory('foo').childFile('out.json');

    writePerformanceData(performanceMeasurement, outFile);

    expect(outFile, exists);
    expect(json.decode(outFile.readAsStringSync()), <String, Object>{
      'targets': <Object>[
        <String, Object>{
          'name': 'foo',
          'skipped': false,
          'succeeded': true,
          'elapsedMilliseconds': 123,
        },
      ],
    });
  });
}
