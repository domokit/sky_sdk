// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build.dart';
import 'package:flutter_tools/src/commands/build_macos.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fake_pub_deps.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';
import '../../src/test_flutter_command_runner.dart';

class FakeXcodeProjectInterpreterWithProfile extends FakeXcodeProjectInterpreter {
  @override
  Future<XcodeProjectInfo> getInfo(String projectPath, {String? projectFilename}) async {
    return XcodeProjectInfo(<String>['Runner'], <String>['Debug', 'Profile', 'Release'], <String>[
      'Runner',
    ], BufferLogger.test());
  }
}

final Platform macosPlatform = FakePlatform(
  operatingSystem: 'macos',
  environment: <String, String>{'FLUTTER_ROOT': '/', 'HOME': '/'},
);

final FakePlatform macosPlatformCustomEnv = FakePlatform(
  operatingSystem: 'macos',
  environment: <String, String>{'FLUTTER_ROOT': '/', 'HOME': '/'},
);

final Platform notMacosPlatform = FakePlatform(environment: <String, String>{'FLUTTER_ROOT': '/'});

void main() {
  late MemoryFileSystem fileSystem;
  late FakeProcessManager fakeProcessManager;
  late ProcessUtils processUtils;
  late BufferLogger logger;
  late XcodeProjectInterpreter xcodeProjectInterpreter;
  late Artifacts artifacts;
  late FakeAnalytics fakeAnalytics;

  setUpAll(() {
    Cache.disableLocking();
  });

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    artifacts = Artifacts.test(fileSystem: fileSystem);
    logger = BufferLogger.test();
    fakeProcessManager = FakeProcessManager.empty();
    processUtils = ProcessUtils(logger: logger, processManager: fakeProcessManager);
    xcodeProjectInterpreter = FakeXcodeProjectInterpreter();
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: fileSystem,
      fakeFlutterVersion: FakeFlutterVersion(),
    );
  });

  // Sets up the minimal mock project files necessary to look like a Flutter project.
  void createCoreMockProjectFiles() {
    fileSystem.file('pubspec.yaml').createSync();
    fileSystem.file('.dart_tool/package_config.json').createSync(recursive: true);
    fileSystem.file(fileSystem.path.join('lib', 'main.dart')).createSync(recursive: true);
  }

  // Sets up the minimal mock project files necessary for macOS builds to succeed.
  void createMinimalMockProjectFiles() {
    fileSystem
        .directory(fileSystem.path.join('macos', 'Runner.xcworkspace'))
        .createSync(recursive: true);
    createCoreMockProjectFiles();
  }

  // Creates a FakeCommand for the xcodebuild call to build the app
  // in the given configuration.
  FakeCommand setUpFakeXcodeBuildHandler(
    String configuration, {
    bool verbose = false,
    void Function(List<String> command)? onRun,
    List<String>? additionalCommandArguments,
  }) {
    final FlutterProject flutterProject = FlutterProject.fromDirectory(fileSystem.currentDirectory);
    final Directory flutterBuildDir = fileSystem.directory(getMacOSBuildDirectory());
    return FakeCommand(
      command: <String>[
        '/usr/bin/env',
        'xcrun',
        'xcodebuild',
        '-workspace',
        flutterProject.macos.xcodeWorkspace!.path,
        '-configuration',
        configuration,
        '-scheme',
        'Runner',
        '-derivedDataPath',
        flutterBuildDir.absolute.path,
        '-destination',
        'platform=macOS',
        'OBJROOT=${fileSystem.path.join(flutterBuildDir.absolute.path, 'Build', 'Intermediates.noindex')}',
        'SYMROOT=${fileSystem.path.join(flutterBuildDir.absolute.path, 'Build', 'Products')}',
        if (verbose) 'VERBOSE_SCRIPT_LOGGING=YES' else '-quiet',
        'COMPILER_INDEX_STORE_ENABLE=NO',
        if (additionalCommandArguments != null) ...additionalCommandArguments,
      ],
      stdout: '''
STDOUT STUFF
note: Using new build system
note: Planning
note: Build preparation complete
note: Building targets in dependency order
''',
      stderr: '''
2022-03-24 10:07:21.954 xcodebuild[2096:1927385] Requested but did not find extension point with identifier Xcode.IDEKit.ExtensionSentinelHostApplications for extension Xcode.DebuggerFoundation.AppExtensionHosts.watchOS of plug-in com.apple.dt.IDEWatchSupportCore
2022-03-24 10:07:21.954 xcodebuild[2096:1927385] Requested but did not find extension point with identifier Xcode.IDEKit.ExtensionPointIdentifierToBundleIdentifier for extension Xcode.DebuggerFoundation.AppExtensionToBundleIdentifierMap.watchOS of plug-in com.apple.dt.IDEWatchSupportCore
2023-11-10 10:44:58.030 xcodebuild[61115:1017566] [MT] DVTAssertions: Warning in /System/Volumes/Data/SWE/Apps/DT/BuildRoots/BuildRoot11/ActiveBuildRoot/Library/Caches/com.apple.xbs/Sources/IDEFrameworks/IDEFrameworks-22267/IDEFoundation/Provisioning/Capabilities Infrastructure/IDECapabilityQuerySelection.swift:103
Details:  createItemModels creation requirements should not create capability item model for a capability item model that already exists.
Function: createItemModels(for:itemModelSource:)
Thread:   <_NSMainThread: 0x6000027c0280>{number = 1, name = main}
Please file a bug at https://feedbackassistant.apple.com with this warning message and any useful information you can provide.
STDERR STUFF
''',
      onRun: (List<String> command) {
        fileSystem.file(fileSystem.path.join('macos', 'Flutter', 'ephemeral', '.app_filename'))
          ..createSync(recursive: true)
          ..writeAsStringSync('example.app');
        if (onRun != null) {
          onRun(command);
        }
      },
    );
  }

  testUsingContext(
    'macOS build fails when there is no macos project',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createCoreMockProjectFiles();

      expect(
        createTestCommandRunner(command).run(const <String>['build', 'macos', '--no-pub']),
        throwsToolExit(
          message:
              'No macOS desktop project configured. See '
              'https://flutter.dev/to/add-desktop-support '
              'to learn about adding macOS support to a project.',
        ),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => macosPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build successfully with renamed .xcodeproj/.xcworkspace files',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );

      fileSystem
          .directory(fileSystem.path.join('macos', 'RenamedProj.xcodeproj'))
          .createSync(recursive: true);
      fileSystem
          .directory(fileSystem.path.join('macos', 'RenamedWorkspace.xcworkspace'))
          .createSync(recursive: true);
      createCoreMockProjectFiles();

      await createTestCommandRunner(command).run(const <String>['build', 'macos', '--no-pub']);

      expect(
        analyticsTimingEventExists(
          sentEvents: fakeAnalytics.sentEvents,
          workflow: 'build',
          variableName: 'xcode-macos',
        ),
        true,
      );
    },
    overrides: <Type, Generator>{
      Platform: () => macosPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      Analytics: () => fakeAnalytics,
    },
  );

  testUsingContext(
    'macOS build fails on non-macOS platform',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.file(fileSystem.path.join('lib', 'main.dart')).createSync(recursive: true);

      expect(
        createTestCommandRunner(command).run(const <String>['build', 'macos', '--no-pub']),
        throwsA(isA<UsageException>()),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => notMacosPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build fails when feature is disabled',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.file(fileSystem.path.join('lib', 'main.dart')).createSync(recursive: true);

      expect(
        createTestCommandRunner(command).run(const <String>['build', 'macos', '--no-pub']),
        throwsToolExit(
          message:
              '"build macos" is not currently supported. To enable, run "flutter config --enable-macos-desktop".',
        ),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => macosPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(),
    },
  );

  testUsingContext(
    'macOS build forwards error stdout to status logger error',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--debug', '--no-pub']);
      expect(testLogger.statusText, isNot(contains('STDOUT STUFF')));
      expect(testLogger.traceText, isNot(contains('STDOUT STUFF')));
      expect(testLogger.errorText, contains('STDOUT STUFF'));
      expect(testLogger.errorText, contains('STDERR STUFF'));
      // Filters out some xcodebuild logging spew.
      expect(testLogger.errorText, isNot(contains('xcodebuild[2096:1927385]')));
      expect(testLogger.errorText, isNot(contains('Using new build system')));
      expect(testLogger.errorText, isNot(contains('Building targets in dependency order')));
      expect(testLogger.errorText, isNot(contains('DVTAssertions: Warning in')));
      expect(testLogger.errorText, isNot(contains('createItemModels')));
      expect(testLogger.errorText, isNot(contains('_NSMainThread:')));
      expect(
        testLogger.errorText,
        isNot(contains('Please file a bug at https://feedbackassistant')),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[setUpFakeXcodeBuildHandler('Debug')]),
      Pub: FakePubWithPrimedDeps.new,
      Platform: () => macosPlatform,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build outputs path and size when successful',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: MemoryFileSystem.test(),
        processUtils: processUtils,
        logger: BufferLogger.test(),
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      await createTestCommandRunner(command).run(const <String>['build', 'macos', '--no-pub']);
      expect(
        testLogger.statusText,
        contains(RegExp(r'✓ Built build/macos/Build/Products/Release/example.app \(\d+\.\d+MB\)')),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[setUpFakeXcodeBuildHandler('Release')]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build invokes xcode build (debug)',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--debug', '--no-pub']);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[setUpFakeXcodeBuildHandler('Debug')]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build invokes xcode build (debug) with verbosity',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--debug', '--no-pub', '-v']);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            setUpFakeXcodeBuildHandler('Debug', verbose: true),
          ]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build invokes xcode build (profile)',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--profile', '--no-pub']);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[setUpFakeXcodeBuildHandler('Profile')]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithProfile(),
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build invokes xcode build (release)',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--release', '--no-pub']);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[setUpFakeXcodeBuildHandler('Release')]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build supports standard desktop build options',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();
      fileSystem.file('lib/other.dart').createSync(recursive: true);
      fileSystem.file('foo/bar.sksl.json').createSync(recursive: true);

      await createTestCommandRunner(command).run(const <String>[
        'build',
        'macos',
        '--target=lib/other.dart',
        '--no-pub',
        '--track-widget-creation',
        '--split-debug-info=foo/',
        '--enable-experiment=non-nullable',
        '--obfuscate',
        '--dart-define=foo.bar=2',
        '--dart-define=fizz.far=3',
        '--tree-shake-icons',
        '--bundle-sksl-path=foo/bar.sksl.json',
      ]);
      final List<String> contents =
          fileSystem.file('./macos/Flutter/ephemeral/Flutter-Generated.xcconfig').readAsLinesSync();

      expect(
        contents,
        containsAll(<String>[
          'FLUTTER_APPLICATION_PATH=/',
          'FLUTTER_TARGET=lib/other.dart',
          'FLUTTER_BUILD_DIR=build',
          'FLUTTER_BUILD_NAME=1.0.0',
          'FLUTTER_BUILD_NUMBER=1',
          'DART_DEFINES=Zm9vLmJhcj0y,Zml6ei5mYXI9Mw==',
          'DART_OBFUSCATION=true',
          'EXTRA_FRONT_END_OPTIONS=--enable-experiment=non-nullable',
          'EXTRA_GEN_SNAPSHOT_OPTIONS=--enable-experiment=non-nullable',
          'SPLIT_DEBUG_INFO=foo/',
          'TRACK_WIDGET_CREATION=true',
          'TREE_SHAKE_ICONS=true',
          'BUNDLE_SKSL_PATH=foo/bar.sksl.json',
          'PACKAGE_CONFIG=/.dart_tool/package_config.json',
          'COCOAPODS_PARALLEL_CODE_SIGN=true',
        ]),
      );
      expect(contents, isNot(contains('EXCLUDED_ARCHS')));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[setUpFakeXcodeBuildHandler('Release')]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      Artifacts: () => Artifacts.test(),
    },
  );

  testUsingContext(
    'build settings contains Flutter Xcode environment variables',
    () async {
      macosPlatformCustomEnv.environment = Map<String, String>.unmodifiable(<String, String>{
        'FLUTTER_XCODE_ASSETCATALOG_COMPILER_APPICON_NAME': 'AppIcon.special',
      });

      final FlutterProject flutterProject = FlutterProject.fromDirectory(
        fileSystem.currentDirectory,
      );
      final Directory flutterBuildDir = fileSystem.directory(getMacOSBuildDirectory());
      createMinimalMockProjectFiles();

      fakeProcessManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            '/usr/bin/env',
            'xcrun',
            'xcodebuild',
            '-workspace',
            flutterProject.macos.xcodeWorkspace!.path,
            '-configuration',
            'Debug',
            '-scheme',
            'Runner',
            '-derivedDataPath',
            flutterBuildDir.absolute.path,
            '-destination',
            'platform=macOS',
            'OBJROOT=${fileSystem.path.join(flutterBuildDir.absolute.path, 'Build', 'Intermediates.noindex')}',
            'SYMROOT=${fileSystem.path.join(flutterBuildDir.absolute.path, 'Build', 'Products')}',
            '-quiet',
            'COMPILER_INDEX_STORE_ENABLE=NO',
            'ASSETCATALOG_COMPILER_APPICON_NAME=AppIcon.special',
          ],
        ),
      ]);

      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--debug', '--no-pub']);

      expect(fakeProcessManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => fakeProcessManager,
      Platform: () => macosPlatformCustomEnv,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      XcodeProjectInterpreter: () => xcodeProjectInterpreter,
    },
  );

  testUsingContext(
    'macOS build supports build-name and build-number',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      await createTestCommandRunner(command).run(const <String>[
        'build',
        'macos',
        '--debug',
        '--no-pub',
        '--build-name=1.2.3',
        '--build-number=42',
      ]);
      final String contents =
          fileSystem
              .file('./macos/Flutter/ephemeral/Flutter-Generated.xcconfig')
              .readAsStringSync();

      expect(contents, contains('FLUTTER_BUILD_NAME=1.2.3'));
      expect(contents, contains('FLUTTER_BUILD_NUMBER=42'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[setUpFakeXcodeBuildHandler('Debug')]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'Refuses to build for macOS when feature is disabled',
    () {
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildCommand(
          artifacts: artifacts,
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: fileSystem,
          logger: logger,
          processUtils: processUtils,
          osUtils: FakeOperatingSystemUtils(),
        ),
      );

      final bool supported =
          BuildMacosCommand(logger: BufferLogger.test(), verboseHelp: false).supported;
      expect(
        () => runner.run(<String>['build', 'macos', '--no-pub']),
        supported ? throwsToolExit() : throwsA(isA<UsageException>()),
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags()},
  );

  testUsingContext(
    'hidden when not enabled on macOS host',
    () {
      expect(BuildMacosCommand(logger: BufferLogger.test(), verboseHelp: false).hidden, true);
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(),
      Platform: () => macosPlatform,
    },
  );

  testUsingContext(
    'Not hidden when enabled and on macOS host',
    () {
      expect(BuildMacosCommand(logger: BufferLogger.test(), verboseHelp: false).hidden, false);
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      Platform: () => macosPlatform,
    },
  );

  testUsingContext(
    'code size analysis throws StateError if no code size snapshot generated by gen_snapshot',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      fileSystem.file('build/macos/Build/Products/Release/Runner.app/App')
        ..createSync(recursive: true)
        ..writeAsBytesSync(List<int>.generate(10000, (int index) => 0));

      expect(
        () => createTestCommandRunner(
          command,
        ).run(const <String>['build', 'macos', '--no-pub', '--analyze-size']),
        throwsA(
          isA<StateError>().having(
            (StateError err) => err.message,
            'message',
            'No code size snapshot file (snapshot.<ARCH>.json) found in build/flutter_size_01',
          ),
        ),
      );

      expect(testLogger.statusText, isEmpty);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            // we never generate code size snapshot here
            setUpFakeXcodeBuildHandler('Release'),
          ]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: macosPlatform),
      Analytics: () => fakeAnalytics,
    },
  );
  testUsingContext(
    'Performs code size analysis and sends analytics from arm64 host',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      fileSystem.file('build/macos/Build/Products/Release/example.app/App')
        ..createSync(recursive: true)
        ..writeAsBytesSync(List<int>.generate(10000, (int index) => 0));

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--no-pub', '--analyze-size']);

      expect(
        testLogger.statusText,
        contains('A summary of your macOS bundle analysis can be found at'),
      );
      expect(testLogger.statusText, contains('dart devtools --appSizeBase='));
      expect(fakeAnalytics.sentEvents, contains(Event.codeSizeAnalysis(platform: 'macos')));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            // These are generated by gen_snapshot because flutter assemble passes
            // extra flags specifying this output path
            setUpFakeXcodeBuildHandler(
              'Release',
              onRun: (_) {
                fileSystem.file('build/flutter_size_01/snapshot.arm64.json')
                  ..createSync(recursive: true)
                  ..writeAsStringSync('''
[
  {
    "l": "dart:_internal",
    "c": "SubListIterable",
    "n": "[Optimized] skip",
    "s": 2400
  }
]''');
                fileSystem.file('build/flutter_size_01/trace.arm64.json')
                  ..createSync(recursive: true)
                  ..writeAsStringSync('{}');
              },
            ),
          ]),
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: macosPlatform),
      Analytics: () => fakeAnalytics,
    },
  );

  testUsingContext(
    'macOS build overrides CODE_SIGN_ENTITLEMENTS when in CI if entitlement file exists (debug)',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      final File entitlementFile = fileSystem.file(
        fileSystem.path.join('macos', 'Runner', 'DebugProfile.entitlements'),
      );
      entitlementFile.createSync(recursive: true);
      entitlementFile.writeAsStringSync('''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>com.apple.security.app-sandbox</key>
	<true/>
</dict>
</plist>

''');

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--debug', '--no-pub']);

      final File tempEntitlementFile = fileSystem.systemTempDirectory.childFile(
        'flutter_disable_sandbox_entitlement.rand0/DebugProfileWithDisabledSandboxing.entitlements',
      );
      expect(tempEntitlementFile, exists);
      expect(tempEntitlementFile.readAsStringSync(), '''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>com.apple.security.app-sandbox</key>
	<false/>
</dict>
</plist>

''');
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            setUpFakeXcodeBuildHandler(
              'Debug',
              additionalCommandArguments: <String>[
                'CODE_SIGN_ENTITLEMENTS=/.tmp_rand0/flutter_disable_sandbox_entitlement.rand0/DebugProfileWithDisabledSandboxing.entitlements',
              ],
            ),
          ]),
      Platform:
          () => FakePlatform(
            operatingSystem: 'macos',
            environment: <String, String>{'FLUTTER_ROOT': '/', 'HOME': '/', 'LUCI_CI': 'True'},
          ),
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'macOS build overrides CODE_SIGN_ENTITLEMENTS when in CI if entitlement file exists (release)',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      final File entitlementFile = fileSystem.file(
        fileSystem.path.join('macos', 'Runner', 'Release.entitlements'),
      );
      entitlementFile.createSync(recursive: true);
      entitlementFile.writeAsStringSync('''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>com.apple.security.app-sandbox</key>
	<true/>
</dict>
</plist>

''');

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'macos', '--release', '--no-pub']);

      final File tempEntitlementFile = fileSystem.systemTempDirectory.childFile(
        'flutter_disable_sandbox_entitlement.rand0/ReleaseWithDisabledSandboxing.entitlements',
      );
      expect(tempEntitlementFile, exists);
      expect(tempEntitlementFile.readAsStringSync(), '''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>com.apple.security.app-sandbox</key>
	<false/>
</dict>
</plist>

''');
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            setUpFakeXcodeBuildHandler(
              'Release',
              additionalCommandArguments: <String>[
                'CODE_SIGN_ENTITLEMENTS=/.tmp_rand0/flutter_disable_sandbox_entitlement.rand0/ReleaseWithDisabledSandboxing.entitlements',
              ],
            ),
          ]),
      Platform:
          () => FakePlatform(
            operatingSystem: 'macos',
            environment: <String, String>{'FLUTTER_ROOT': '/', 'HOME': '/', 'LUCI_CI': 'True'},
          ),
      Pub: FakePubWithPrimedDeps.new,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );
}
