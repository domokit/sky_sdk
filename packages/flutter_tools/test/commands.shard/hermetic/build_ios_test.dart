// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build.dart';
import 'package:flutter_tools/src/commands/build_ios.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/ios/code_signing.dart';
import 'package:flutter_tools/src/ios/mac.dart';
import 'package:flutter_tools/src/ios/plist_parser.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../general.shard/ios/xcresult_test_data.dart';
import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_pub_deps.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';
import '../../src/test_flutter_command_runner.dart';

class FakeXcodeProjectInterpreterWithBuildSettings extends FakeXcodeProjectInterpreter {
  FakeXcodeProjectInterpreterWithBuildSettings({
    this.productBundleIdentifier,
    this.developmentTeam = 'abc',
  });

  @override
  Future<Map<String, String>> getBuildSettings(
    String projectPath, {
    XcodeProjectBuildContext? buildContext,
    Duration timeout = const Duration(minutes: 1),
  }) async {
    return <String, String>{
      'PRODUCT_BUNDLE_IDENTIFIER': productBundleIdentifier ?? 'io.flutter.someProject',
      'TARGET_BUILD_DIR': 'build/ios/Release-iphoneos',
      'WRAPPER_NAME': 'Runner.app',
      if (developmentTeam != null) 'DEVELOPMENT_TEAM': developmentTeam!,
    };
  }

  /// The value of 'PRODUCT_BUNDLE_IDENTIFIER'.
  final String? productBundleIdentifier;

  final String? developmentTeam;
}

final Platform macosPlatform = FakePlatform(
  operatingSystem: 'macos',
  environment: <String, String>{'FLUTTER_ROOT': '/', 'HOME': '/'},
);
final Platform notMacosPlatform = FakePlatform(environment: <String, String>{'FLUTTER_ROOT': '/'});

void main() {
  late MemoryFileSystem fileSystem;
  late FakeAnalytics fakeAnalytics;
  late BufferLogger logger;
  late FakeProcessManager processManager;
  late ProcessUtils processUtils;
  late Artifacts artifacts;

  setUpAll(() {
    Cache.disableLocking();
  });

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    artifacts = Artifacts.test(fileSystem: fileSystem);
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: fileSystem,
      fakeFlutterVersion: FakeFlutterVersion(),
    );
    logger = BufferLogger.test();
    processManager = FakeProcessManager.empty();
    processUtils = ProcessUtils(logger: logger, processManager: processManager);
  });

  // Sets up the minimal mock project files necessary to look like a Flutter project.
  void createCoreMockProjectFiles() {
    fileSystem.file('pubspec.yaml').createSync();
    fileSystem.directory('.dart_tool').childFile('package_config.json').createSync(recursive: true);
    fileSystem.file(fileSystem.path.join('lib', 'main.dart')).createSync(recursive: true);
  }

  // Sets up the minimal mock project files necessary for iOS builds to succeed.
  void createMinimalMockProjectFiles() {
    fileSystem
        .directory(fileSystem.path.join('ios', 'Runner.xcodeproj'))
        .createSync(recursive: true);
    fileSystem
        .directory(fileSystem.path.join('ios', 'Runner.xcworkspace'))
        .createSync(recursive: true);
    fileSystem
        .file(fileSystem.path.join('ios', 'Runner.xcodeproj', 'project.pbxproj'))
        .createSync();
    createCoreMockProjectFiles();
  }

  const FakeCommand xattrCommand = FakeCommand(
    command: <String>['xattr', '-r', '-d', 'com.apple.FinderInfo', '/'],
  );

  FakeCommand setUpRsyncCommand({void Function(List<String> command)? onRun}) {
    return FakeCommand(
      command: const <String>[
        'rsync',
        '-8',
        '-av',
        '--delete',
        'build/ios/Release-iphoneos/Runner.app',
        'build/ios/iphoneos',
      ],
      onRun: onRun,
    );
  }

  FakeCommand setUpXCResultCommand({
    String stdout = '',
    void Function(List<String> command)? onRun,
  }) {
    return FakeCommand(
      command: const <String>[
        'xcrun',
        'xcresulttool',
        'get',
        '--path',
        _xcBundleDirectoryPath,
        '--format',
        'json',
      ],
      stdout: stdout,
      onRun: onRun,
    );
  }

  // Creates a FakeCommand for the xcodebuild call to build the app
  // in the given configuration.
  FakeCommand setUpFakeXcodeBuildHandler({
    bool verbose = false,
    bool simulator = false,
    bool customNaming = false,
    bool disablePortPublication = false,
    String? deviceId,
    int exitCode = 0,
    String? stdout,
    void Function(List<String> command)? onRun,
  }) {
    return FakeCommand(
      command: <String>[
        'xcrun',
        'xcodebuild',
        '-configuration',
        if (simulator) 'Debug' else 'Release',
        if (verbose) 'VERBOSE_SCRIPT_LOGGING=YES' else '-quiet',
        '-workspace',
        if (customNaming) 'RenamedWorkspace.xcworkspace' else 'Runner.xcworkspace',
        '-scheme',
        'Runner',
        'BUILD_DIR=/build/ios',
        '-sdk',
        if (simulator) ...<String>['iphonesimulator'] else ...<String>['iphoneos'],
        if (deviceId != null) ...<String>[
          '-destination',
          'id=$deviceId',
        ] else if (simulator) ...<String>[
          '-destination',
          'generic/platform=iOS Simulator',
        ] else ...<String>['-destination', 'generic/platform=iOS'],
        '-resultBundlePath',
        _xcBundleDirectoryPath,
        '-resultBundleVersion',
        '3',
        if (disablePortPublication) 'DISABLE_PORT_PUBLICATION=YES',
        'FLUTTER_SUPPRESS_ANALYTICS=true',
        'COMPILER_INDEX_STORE_ENABLE=NO',
      ],
      stdout: '''
      TARGET_BUILD_DIR=build/ios/Release-iphoneos
      WRAPPER_NAME=Runner.app
      $stdout
''',
      exitCode: exitCode,
      onRun: onRun,
    );
  }

  testUsingContext(
    'ios build fails when there is no ios project',
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
        createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
        throwsToolExit(message: 'Application not configured for iOS'),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => macosPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build fails in debug with code analysis',
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
        createTestCommandRunner(
          command,
        ).run(const <String>['build', 'ios', '--no-pub', '--debug', '--analyze-size']),
        throwsToolExit(message: '--analyze-size" can only be used on release builds'),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => macosPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build fails on non-macOS platform',
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
      fileSystem
          .directory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);
      fileSystem.file(fileSystem.path.join('lib', 'main.dart')).createSync(recursive: true);

      final bool supported =
          BuildIOSCommand(logger: BufferLogger.test(), verboseHelp: false).supported;
      expect(
        createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
        supported ? throwsToolExit() : throwsA(isA<UsageException>()),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => notMacosPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build outputs path and size when successful',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      createMinimalMockProjectFiles();

      await createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']);
      expect(
        testLogger.statusText,
        contains(RegExp(r'✓ Built build/ios/iphoneos/Runner\.app \(\d+\.\d+MB\)')),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      Pub: FakePubWithPrimedDeps.new,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            xattrCommand,
            setUpFakeXcodeBuildHandler(
              onRun: (_) {
                fileSystem
                    .directory('build/ios/Release-iphoneos/Runner.app')
                    .createSync(recursive: true);
              },
            ),
            setUpRsyncCommand(),
          ]),
      Platform: () => macosPlatform,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build invokes xcode build',
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

      processManager.addCommands(<FakeCommand>[
        xattrCommand,
        setUpFakeXcodeBuildHandler(
          onRun: (_) {
            fileSystem
                .directory('build/ios/Release-iphoneos/Runner.app')
                .createSync(recursive: true);
          },
        ),
        setUpRsyncCommand(),
      ]);

      await createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']);
      expect(testLogger.statusText, contains('build/ios/iphoneos/Runner.app'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Pub: FakePubWithPrimedDeps.new,
      Platform: () => macosPlatform,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build invokes xcode build with disable port publication setting',
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
      ).run(const <String>['build', 'ios', '--no-pub', '--no-publish-port', '--ci']);
      expect(testLogger.statusText, contains('build/ios/iphoneos/Runner.app'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            xattrCommand,
            setUpFakeXcodeBuildHandler(
              disablePortPublication: true,
              onRun: (_) {
                fileSystem
                    .directory('build/ios/Release-iphoneos/Runner.app')
                    .createSync(recursive: true);
              },
            ),
            setUpRsyncCommand(),
          ]),
      Pub: FakePubWithPrimedDeps.new,
      Platform: () => macosPlatform,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build invokes xcode build without disable port publication setting when not in CI',
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
      ).run(const <String>['build', 'ios', '--no-pub', '--no-publish-port']);
      expect(testLogger.statusText, contains('build/ios/iphoneos/Runner.app'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            xattrCommand,
            setUpFakeXcodeBuildHandler(
              onRun: (_) {
                fileSystem
                    .directory('build/ios/Release-iphoneos/Runner.app')
                    .createSync(recursive: true);
              },
            ),
            setUpRsyncCommand(),
          ]),
      Pub: FakePubWithPrimedDeps.new,
      Platform: () => macosPlatform,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build invokes xcode build with renamed xcodeproj and xcworkspace',
    () async {
      final BuildCommand command = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        osUtils: FakeOperatingSystemUtils(),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
      );

      processManager.addCommands(<FakeCommand>[
        xattrCommand,
        setUpFakeXcodeBuildHandler(
          customNaming: true,
          onRun: (_) {
            fileSystem
                .directory('build/ios/Release-iphoneos/Runner.app')
                .createSync(recursive: true);
          },
        ),
        setUpRsyncCommand(),
      ]);

      fileSystem
          .directory(fileSystem.path.join('ios', 'RenamedProj.xcodeproj'))
          .createSync(recursive: true);
      fileSystem
          .directory(fileSystem.path.join('ios', 'RenamedWorkspace.xcworkspace'))
          .createSync(recursive: true);
      fileSystem
          .file(fileSystem.path.join('ios', 'RenamedProj.xcodeproj', 'project.pbxproj'))
          .createSync();
      createCoreMockProjectFiles();

      await createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']);
      expect(testLogger.statusText, contains('build/ios/iphoneos/Runner.app'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Pub: FakePubWithPrimedDeps.new,
      Platform: () => macosPlatform,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build invokes xcode build with device ID',
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
      processManager.addCommands(<FakeCommand>[
        xattrCommand,
        setUpFakeXcodeBuildHandler(
          deviceId: '1234',
          onRun: (_) {
            fileSystem
                .directory('build/ios/Release-iphoneos/Runner.app')
                .createSync(recursive: true);
          },
        ),
        setUpRsyncCommand(),
      ]);
      createMinimalMockProjectFiles();

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'ios', '--no-pub', '--device-id', '1234']);
      expect(testLogger.statusText, contains('build/ios/iphoneos/Runner.app'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Pub: FakePubWithPrimedDeps.new,
      Platform: () => macosPlatform,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios simulator build invokes xcode build',
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
      processManager.addCommands(<FakeCommand>[
        xattrCommand,
        setUpFakeXcodeBuildHandler(
          simulator: true,
          onRun: (_) {
            fileSystem
                .directory('build/ios/Debug-iphonesimulator/Runner.app')
                .createSync(recursive: true);
          },
        ),
        setUpRsyncCommand(),
      ]);
      createMinimalMockProjectFiles();

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'ios', '--simulator', '--no-pub']);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Pub: FakePubWithPrimedDeps.new,
      Platform: () => macosPlatform,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'ios build invokes xcode build with verbosity',
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
      processManager.addCommands(<FakeCommand>[
        xattrCommand,
        setUpFakeXcodeBuildHandler(
          verbose: true,
          onRun: (_) {
            fileSystem
                .directory('build/ios/Release-iphoneos/Runner.app')
                .createSync(recursive: true);
          },
        ),
        setUpRsyncCommand(),
      ]);

      await createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub', '-v']);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Pub: FakePubWithPrimedDeps.new,
      Platform: () => macosPlatform,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  testUsingContext(
    'Performs code size analysis and sends analytics',
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
      processManager.addCommands(<FakeCommand>[
        xattrCommand,
        setUpFakeXcodeBuildHandler(
          onRun: (_) {
            fileSystem
                .directory('build/ios/Release-iphoneos/Runner.app')
                .createSync(recursive: true);
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
        setUpRsyncCommand(
          onRun:
              (_) =>
                  fileSystem.file('build/ios/iphoneos/Runner.app/Frameworks/App.framework/App')
                    ..createSync(recursive: true)
                    ..writeAsBytesSync(List<int>.generate(10000, (int index) => 0)),
        ),
      ]);
      createMinimalMockProjectFiles();

      await createTestCommandRunner(
        command,
      ).run(const <String>['build', 'ios', '--no-pub', '--analyze-size']);

      expect(logger.statusText, contains('A summary of your iOS bundle analysis can be found at'));
      expect(logger.statusText, contains('dart devtools --appSizeBase='));
      expect(fakeAnalytics.sentEvents, contains(Event.codeSizeAnalysis(platform: 'ios')));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      Logger: () => logger,
      ProcessManager: () => processManager,
      Platform: () => macosPlatform,
      Pub: FakePubWithPrimedDeps.new,
      FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: macosPlatform),
      Analytics: () => fakeAnalytics,
      XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
    },
  );

  group('Analytics for impeller plist setting', () {
    const String plistContents = '''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>FLTEnableImpeller</key>
  <false/>
</dict>
</plist>
''';
    const FakeCommand plutilCommand = FakeCommand(
      command: <String>['/usr/bin/plutil', '-convert', 'xml1', '-o', '-', '/ios/Runner/Info.plist'],
      stdout: plistContents,
    );

    testUsingContext(
      'Sends an analytics event when Impeller is enabled',
      () async {
        final BuildCommand command = BuildCommand(
          artifacts: artifacts,
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: MemoryFileSystem.test(),
          logger: BufferLogger.test(),
          osUtils: FakeOperatingSystemUtils(),
          processUtils: processUtils,
        );
        createMinimalMockProjectFiles();

        await createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']);

        expect(
          fakeAnalytics.sentEvents,
          contains(Event.flutterBuildInfo(label: 'plist-impeller-enabled', buildType: 'ios')),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager:
            () => FakeProcessManager.list(<FakeCommand>[
              xattrCommand,
              setUpFakeXcodeBuildHandler(
                onRun: (_) {
                  fileSystem
                      .directory('build/ios/Release-iphoneos/Runner.app')
                      .createSync(recursive: true);
                },
              ),
              setUpRsyncCommand(
                onRun:
                    (_) =>
                        fileSystem.file(
                            'build/ios/iphoneos/Runner.app/Frameworks/App.framework/App',
                          )
                          ..createSync(recursive: true)
                          ..writeAsBytesSync(List<int>.generate(10000, (int index) => 0)),
              ),
            ]),
        Platform: () => macosPlatform,
        FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: macosPlatform),
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
        Pub: FakePubWithPrimedDeps.new,
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'Sends an analytics event when Impeller is disabled',
      () async {
        final BuildCommand command = BuildCommand(
          artifacts: artifacts,
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
          osUtils: FakeOperatingSystemUtils(),
          processUtils: processUtils,
        );
        createMinimalMockProjectFiles();

        fileSystem.file(fileSystem.path.join('usr', 'bin', 'plutil')).createSync(recursive: true);

        final File infoPlist = fileSystem.file(fileSystem.path.join('ios', 'Runner', 'Info.plist'))
          ..createSync(recursive: true);

        infoPlist.writeAsStringSync(plistContents);

        await createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']);

        expect(
          fakeAnalytics.sentEvents,
          contains(Event.flutterBuildInfo(label: 'plist-impeller-disabled', buildType: 'ios')),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager:
            () => FakeProcessManager.list(<FakeCommand>[
              xattrCommand,
              setUpFakeXcodeBuildHandler(
                onRun: (_) {
                  fileSystem
                      .directory('build/ios/Release-iphoneos/Runner.app')
                      .createSync(recursive: true);
                },
              ),
              setUpRsyncCommand(
                onRun:
                    (_) =>
                        fileSystem.file(
                            'build/ios/iphoneos/Runner.app/Frameworks/App.framework/App',
                          )
                          ..createSync(recursive: true)
                          ..writeAsBytesSync(List<int>.generate(10000, (int index) => 0)),
              ),
            ]),
        Platform: () => macosPlatform,
        FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: macosPlatform),
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
        FlutterProjectFactory:
            () => FlutterProjectFactory(fileSystem: fileSystem, logger: BufferLogger.test()),
        PlistParser:
            () => PlistParser(
              fileSystem: fileSystem,
              logger: BufferLogger.test(),
              processManager: FakeProcessManager.list(<FakeCommand>[
                plutilCommand,
                plutilCommand,
                plutilCommand,
              ]),
            ),
        Pub: FakePubWithPrimedDeps.new,
        Analytics: () => fakeAnalytics,
      },
    );
  });

  group('xcresults device', () {
    testUsingContext(
      'Trace error if xcresult is empty.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.traceText, contains('xcresult parser: Unrecognized top level json format.'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Display xcresult issues on console if parsed, suppress Xcode output',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
            stdout: 'Lots of spew from Xcode',
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithIssues),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains("Use of undeclared identifier 'asdas'"));
        expect(
          logger.errorText,
          contains('/Users/m/Projects/test_create/ios/Runner/AppDelegate.m:7:56'),
        );
        expect(logger.statusText, isNot(contains("Xcode's output")));
        expect(logger.statusText, isNot(contains('Lots of spew from Xcode')));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Do not display xcresult issues that needs to be discarded.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithIssuesToBeDiscarded),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains("Use of undeclared identifier 'asdas'"));
        expect(
          logger.errorText,
          contains('/Users/m/Projects/test_create/ios/Runner/AppDelegate.m:7:56'),
        );
        expect(
          logger.errorText,
          isNot(contains('Command PhaseScriptExecution failed with a nonzero exit code')),
        );
        expect(
          logger.warningText,
          isNot(contains('but the range of supported deployment target versions is')),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Trace if xcresult bundle does not exist.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(exitCode: 1),
          setUpXCResultCommand(stdout: kSampleResultJsonWithIssues),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(
          testLogger.traceText,
          contains('The xcresult bundle are not generated. Displaying xcresult is disabled.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Extra error message for provision profile issue in xcresult bundle.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithProvisionIssue),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains('Some Provisioning profile issue.'));
        expect(
          logger.errorText,
          contains(
            'It appears that there was a problem signing your application prior to installation on the device.',
          ),
        );
        expect(
          logger.errorText,
          contains('Verify that the Bundle Identifier in your project is your signing id in Xcode'),
        );
        expect(logger.errorText, contains('open ios/Runner.xcworkspace'));
        expect(
          logger.errorText,
          contains("Also try selecting 'Product > Build' to fix the problem."),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Display xcresult issues with no provisioning profile.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithNoProvisioningProfileIssue),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(
          logger.errorText,
          contains(
            'Runner requires a provisioning profile. Select a provisioning profile in the Signing & Capabilities editor',
          ),
        );
        expect(logger.errorText, contains(noProvisioningProfileInstruction));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Extra error message for missing simulator platform in xcresult bundle.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithActionIssues),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains(missingPlatformInstructions('iOS 17.0')));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Delete xcresult bundle before each xcodebuild command.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          // Intentionally fail the first xcodebuild command with concurrent run failure message.
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            stdout: '$kConcurrentRunFailureMessage1 $kConcurrentRunFailureMessage2',
            onRun: (_) {
              fileSystem.systemTempDirectory
                  .childDirectory(_xcBundleDirectoryPath)
                  .childFile('result.xcresult')
                  .createSync(recursive: true);
            },
          ),
          // The second xcodebuild is triggered due to above concurrent run failure message.
          setUpFakeXcodeBuildHandler(
            onRun: (_) {
              // If the file is not cleaned, throw an error, test failure.
              if (fileSystem.systemTempDirectory
                  .childDirectory(_xcBundleDirectoryPath)
                  .existsSync()) {
                throwToolExit('xcresult bundle file existed.', exitCode: 2);
              }
              fileSystem.systemTempDirectory
                  .childDirectory(_xcBundleDirectoryPath)
                  .childFile('result.xcresult')
                  .createSync(recursive: true);
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonNoIssues),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']);

        expect(logger.statusText, contains('Xcode build done.'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Failed to parse xcresult but display missing provisioning profile issue from stdout.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            stdout: '''
Runner requires a provisioning profile. Select a provisioning profile in the Signing & Capabilities editor
''',
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonInvalidIssuesMap),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains(noProvisioningProfileInstruction));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Failed to parse xcresult but detected no development team issue.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonInvalidIssuesMap),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains(noDevelopmentTeamInstruction));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter:
            () => FakeXcodeProjectInterpreterWithBuildSettings(developmentTeam: null),
      },
    );

    testUsingContext(
      'xcresult did not detect issue but detected by stdout.',
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

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(testLogger.errorText, contains(noProvisioningProfileInstruction));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager:
            () => FakeProcessManager.list(<FakeCommand>[
              xattrCommand,
              setUpFakeXcodeBuildHandler(
                exitCode: 1,
                stdout: '''
Runner requires a provisioning profile. Select a provisioning profile in the Signing & Capabilities editor
''',
                onRun: (_) {
                  fileSystem.systemTempDirectory
                      .childDirectory(_xcBundleDirectoryPath)
                      .createSync();
                },
              ),
              setUpXCResultCommand(stdout: kSampleResultJsonNoIssues),
              setUpRsyncCommand(),
            ]),
        Pub: FakePubWithPrimedDeps.new,
        EnvironmentType: () => EnvironmentType.physical,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'xcresult did not detect issue, no development team is detected from build setting.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonInvalidIssuesMap),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(testLogger.errorText, contains(noDevelopmentTeamInstruction));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter:
            () => FakeXcodeProjectInterpreterWithBuildSettings(developmentTeam: null),
      },
    );

    testUsingContext(
      'No development team issue error message is not displayed if no provisioning profile issue is detected from xcresult first.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithNoProvisioningProfileIssue),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains(noProvisioningProfileInstruction));
        expect(logger.errorText, isNot(contains(noDevelopmentTeamInstruction)));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter:
            () => FakeXcodeProjectInterpreterWithBuildSettings(developmentTeam: null),
      },
    );

    testUsingContext(
      'General provisioning profile issue error message is not displayed if no development team issue is detected first.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithProvisionIssue),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(command).run(const <String>['build', 'ios', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains(noDevelopmentTeamInstruction));
        expect(
          logger.errorText,
          isNot(
            contains(
              'It appears that there was a problem signing your application prior to installation on the device.',
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter:
            () => FakeXcodeProjectInterpreterWithBuildSettings(developmentTeam: null),
      },
    );
  });

  group('xcresults simulator', () {
    testUsingContext(
      'Trace error if xcresult is empty.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            simulator: true,
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(
            command,
          ).run(const <String>['build', 'ios', '--simulator', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.traceText, contains('xcresult parser: Unrecognized top level json format.'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Display xcresult issues on console if parsed.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            simulator: true,
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithIssues),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(
            command,
          ).run(const <String>['build', 'ios', '--simulator', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains("Use of undeclared identifier 'asdas'"));
        expect(
          logger.errorText,
          contains('/Users/m/Projects/test_create/ios/Runner/AppDelegate.m:7:56'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Do not display xcresult issues that needs to be discarded.',
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

        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(
            simulator: true,
            exitCode: 1,
            onRun: (_) {
              fileSystem.systemTempDirectory.childDirectory(_xcBundleDirectoryPath).createSync();
            },
          ),
          setUpXCResultCommand(stdout: kSampleResultJsonWithIssuesToBeDiscarded),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(
            command,
          ).run(const <String>['build', 'ios', '--simulator', '--no-pub']),
          throwsToolExit(),
        );

        expect(logger.errorText, contains("Use of undeclared identifier 'asdas'"));
        expect(
          logger.errorText,
          contains('/Users/m/Projects/test_create/ios/Runner/AppDelegate.m:7:56'),
        );
        expect(
          logger.errorText,
          isNot(contains('Command PhaseScriptExecution failed with a nonzero exit code')),
        );
        expect(
          logger.warningText,
          isNot(contains('but the range of supported deployment target versions is')),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );

    testUsingContext(
      'Trace if xcresult bundle does not exist.',
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
        processManager.addCommands(<FakeCommand>[
          xattrCommand,
          setUpFakeXcodeBuildHandler(simulator: true, exitCode: 1),
          setUpXCResultCommand(stdout: kSampleResultJsonWithIssues),
          setUpRsyncCommand(),
        ]);

        createMinimalMockProjectFiles();

        await expectLater(
          createTestCommandRunner(
            command,
          ).run(const <String>['build', 'ios', '--simulator', '--no-pub']),
          throwsToolExit(),
        );

        expect(
          logger.traceText,
          contains('The xcresult bundle are not generated. Displaying xcresult is disabled.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        Logger: () => logger,
        ProcessManager: () => processManager,
        Pub: FakePubWithPrimedDeps.new,
        Platform: () => macosPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreterWithBuildSettings(),
      },
    );
  });
}

const String _xcBundleDirectoryPath =
    '/.tmp_rand0/flutter_ios_build_temp_dirrand0/temporary_xcresult_bundle';

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  late bool platformToolsAvailable;

  @override
  late bool licensesAvailable;

  @override
  AndroidSdkVersion? latestVersion;
}

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  FakeOperatingSystemUtils({this.hostPlatform = HostPlatform.linux_x64});

  @override
  HostPlatform hostPlatform = HostPlatform.linux_x64;
}
