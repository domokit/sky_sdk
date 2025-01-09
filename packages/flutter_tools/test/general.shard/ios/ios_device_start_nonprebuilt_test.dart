// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:fake_async/fake_async.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/ios/application_package.dart';
import 'package:flutter_tools/src/ios/core_devices.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/ios/ios_deploy.dart';
import 'package:flutter_tools/src/ios/iproxy.dart';
import 'package:flutter_tools/src/ios/mac.dart';
import 'package:flutter_tools/src/ios/xcode_debug.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/macos/xcode.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart' hide FakeXcodeProjectInterpreter;
import '../../src/fake_devices.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fake_pub_deps.dart';
import '../../src/fakes.dart';

List<String> _xattrArgs(FlutterProject flutterProject) {
  return <String>['xattr', '-r', '-d', 'com.apple.FinderInfo', flutterProject.directory.path];
}

const List<String> kRunReleaseArgs = <String>[
  'xcrun',
  'xcodebuild',
  '-configuration',
  'Release',
  '-quiet',
  '-workspace',
  'Runner.xcworkspace',
  '-scheme',
  'Runner',
  'BUILD_DIR=/build/ios',
  '-sdk',
  'iphoneos',
  '-destination',
  'id=123',
  'ONLY_ACTIVE_ARCH=YES',
  'ARCHS=arm64',
  '-resultBundlePath',
  '/.tmp_rand0/flutter_ios_build_temp_dirrand0/temporary_xcresult_bundle',
  '-resultBundleVersion',
  '3',
  'FLUTTER_SUPPRESS_ANALYTICS=true',
  'COMPILER_INDEX_STORE_ENABLE=NO',
];

// TODO(matanlurey): XCode builds call processPodsIfNeeded -> refreshPluginsList
// ... which in turn requires that `dart pub deps --json` is called in order to
// label which plugins are dependency plugins.
//
// Ideally processPodsIfNeeded should rely on the command (removing this call).
final Pub fakePubBecauseRefreshPluginsList = FakePubWithPrimedDeps();

const String kConcurrentBuildErrorMessage = '''
"/Developer/Xcode/DerivedData/foo/XCBuildData/build.db":
database is locked
Possibly there are two concurrent builds running in the same filesystem location.
''';

final FakePlatform macPlatform = FakePlatform(
  operatingSystem: 'macos',
  environment: <String, String>{},
);

final FakeOperatingSystemUtils os = FakeOperatingSystemUtils(
  hostPlatform: HostPlatform.darwin_arm64,
);

void main() {
  late Artifacts artifacts;
  late String iosDeployPath;

  setUp(() {
    artifacts = Artifacts.test();
    iosDeployPath = artifacts.getHostArtifact(HostArtifact.iosDeploy).path;
  });

  group('IOSDevice.startApp succeeds in release mode', () {
    late MemoryFileSystem fileSystem;
    late FakeProcessManager processManager;
    late BufferLogger logger;
    late Xcode xcode;
    late FakeXcodeProjectInterpreter fakeXcodeProjectInterpreter;
    late XcodeProjectInfo projectInfo;
    late FakeAnalytics fakeAnalytics;

    setUp(() {
      logger = BufferLogger.test();
      fileSystem = MemoryFileSystem.test();
      processManager = FakeProcessManager.empty();
      projectInfo = XcodeProjectInfo(<String>['Runner'], <String>['Debug', 'Release'], <String>[
        'Runner',
      ], logger);
      fakeXcodeProjectInterpreter = FakeXcodeProjectInterpreter(projectInfo: projectInfo);
      xcode = Xcode.test(
        processManager: FakeProcessManager.any(),
        xcodeProjectInterpreter: fakeXcodeProjectInterpreter,
      );
      fakeAnalytics = getInitializedFakeAnalyticsInstance(
        fs: fileSystem,
        fakeFlutterVersion: FakeFlutterVersion(),
      );
    });

    testUsingContext(
      'missing TARGET_BUILD_DIR',
      () async {
        final IOSDevice iosDevice = setUpIOSDevice(
          fileSystem: fileSystem,
          processManager: processManager,
          logger: logger,
          artifacts: artifacts,
        );
        setUpIOSProject(fileSystem);
        final FlutterProject flutterProject = FlutterProject.fromDirectory(
          fileSystem.currentDirectory,
        );
        final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
          flutterProject.ios,
          'flutter',
          'My Super Awesome App',
        );

        processManager.addCommand(FakeCommand(command: _xattrArgs(flutterProject)));
        processManager.addCommand(const FakeCommand(command: kRunReleaseArgs));

        final LaunchResult launchResult = await iosDevice.startApp(
          buildableIOSApp,
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
          platformArgs: <String, Object>{},
        );

        expect(launchResult.started, false);
        expect(
          logger.errorText,
          contains('Xcode build is missing expected TARGET_BUILD_DIR build setting'),
        );
        expect(processManager, hasNoRemainingExpectations);
        expect(
          analyticsTimingEventExists(
            sentEvents: fakeAnalytics.sentEvents,
            workflow: 'build',
            variableName: 'xcode-ios',
          ),
          true,
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => processManager,
        Pub: () => fakePubBecauseRefreshPluginsList,
        FileSystem: () => fileSystem,
        Logger: () => logger,
        OperatingSystemUtils: () => os,
        Platform: () => macPlatform,
        XcodeProjectInterpreter:
            () => FakeXcodeProjectInterpreter(
              buildSettings: const <String, String>{
                'WRAPPER_NAME': 'My Super Awesome App.app',
                'DEVELOPMENT_TEAM': '3333CCCC33',
              },
              projectInfo: projectInfo,
            ),
        Xcode: () => xcode,
        Analytics: () => fakeAnalytics,
      },
    );

    testUsingContext(
      'missing project info',
      () async {
        final IOSDevice iosDevice = setUpIOSDevice(
          fileSystem: fileSystem,
          processManager: FakeProcessManager.any(),
          logger: logger,
          artifacts: artifacts,
        );
        setUpIOSProject(fileSystem);
        final FlutterProject flutterProject = FlutterProject.fromDirectory(
          fileSystem.currentDirectory,
        );
        final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
          flutterProject.ios,
          'flutter',
          'My Super Awesome App',
        );

        final LaunchResult launchResult = await iosDevice.startApp(
          buildableIOSApp,
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
          platformArgs: <String, Object>{},
        );

        expect(launchResult.started, false);
        expect(logger.errorText, contains('Xcode project not found'));
      },
      overrides: <Type, Generator>{
        ProcessManager: () => FakeProcessManager.any(),
        FileSystem: () => fileSystem,
        Logger: () => logger,
        Platform: () => macPlatform,
        XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(),
        Xcode: () => xcode,
      },
    );

    testUsingContext(
      'with buildable app',
      () async {
        final IOSDevice iosDevice = setUpIOSDevice(
          fileSystem: fileSystem,
          processManager: processManager,
          logger: logger,
          artifacts: artifacts,
        );
        setUpIOSProject(fileSystem);
        final FlutterProject flutterProject = FlutterProject.fromDirectory(
          fileSystem.currentDirectory,
        );
        final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
          flutterProject.ios,
          'flutter',
          'My Super Awesome App',
        );
        fileSystem
            .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
            .createSync(recursive: true);

        processManager.addCommand(FakeCommand(command: _xattrArgs(flutterProject)));
        processManager.addCommand(const FakeCommand(command: kRunReleaseArgs));
        processManager.addCommand(
          const FakeCommand(
            command: <String>[
              'rsync',
              '-8',
              '-av',
              '--delete',
              'build/ios/Release-iphoneos/My Super Awesome App.app',
              'build/ios/iphoneos',
            ],
          ),
        );
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              iosDeployPath,
              '--id',
              '123',
              '--bundle',
              'build/ios/iphoneos/My Super Awesome App.app',
              '--app_deltas',
              'build/ios/app-delta',
              '--no-wifi',
              '--justlaunch',
              '--args',
              const <String>['--enable-dart-profiling'].join(' '),
            ],
          ),
        );

        final LaunchResult launchResult = await iosDevice.startApp(
          buildableIOSApp,
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
          platformArgs: <String, Object>{},
        );

        expect(fileSystem.directory('build/ios/iphoneos'), exists);
        expect(launchResult.started, true);
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => processManager,
        Pub: () => fakePubBecauseRefreshPluginsList,
        FileSystem: () => fileSystem,
        Logger: () => logger,
        OperatingSystemUtils: () => os,
        Platform: () => macPlatform,
        XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
        Xcode: () => xcode,
      },
    );

    testUsingContext(
      'ONLY_ACTIVE_ARCH is NO if different host and target architectures',
      () async {
        // Host architecture is x64, target architecture is arm64.
        final IOSDevice iosDevice = setUpIOSDevice(
          fileSystem: fileSystem,
          processManager: processManager,
          logger: logger,
          artifacts: artifacts,
        );
        setUpIOSProject(fileSystem);
        final FlutterProject flutterProject = FlutterProject.fromDirectory(
          fileSystem.currentDirectory,
        );
        final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
          flutterProject.ios,
          'flutter',
          'My Super Awesome App',
        );
        fileSystem
            .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
            .createSync(recursive: true);

        processManager.addCommand(FakeCommand(command: _xattrArgs(flutterProject)));
        processManager.addCommand(
          const FakeCommand(
            command: <String>[
              'xcrun',
              'xcodebuild',
              '-configuration',
              'Release',
              '-quiet',
              '-workspace',
              'Runner.xcworkspace',
              '-scheme',
              'Runner',
              'BUILD_DIR=/build/ios',
              '-sdk',
              'iphoneos',
              '-destination',
              'id=123',
              'ONLY_ACTIVE_ARCH=NO',
              'ARCHS=arm64',
              '-resultBundlePath',
              '/.tmp_rand0/flutter_ios_build_temp_dirrand0/temporary_xcresult_bundle',
              '-resultBundleVersion',
              '3',
              'FLUTTER_SUPPRESS_ANALYTICS=true',
              'COMPILER_INDEX_STORE_ENABLE=NO',
            ],
          ),
        );
        processManager.addCommand(
          const FakeCommand(
            command: <String>[
              'rsync',
              '-8',
              '-av',
              '--delete',
              'build/ios/Release-iphoneos/My Super Awesome App.app',
              'build/ios/iphoneos',
            ],
          ),
        );
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              iosDeployPath,
              '--id',
              '123',
              '--bundle',
              'build/ios/iphoneos/My Super Awesome App.app',
              '--app_deltas',
              'build/ios/app-delta',
              '--no-wifi',
              '--justlaunch',
              '--args',
              const <String>['--enable-dart-profiling'].join(' '),
            ],
          ),
        );

        final LaunchResult launchResult = await iosDevice.startApp(
          buildableIOSApp,
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
          platformArgs: <String, Object>{},
        );

        expect(fileSystem.directory('build/ios/iphoneos'), exists);
        expect(launchResult.started, true);
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => processManager,
        FileSystem: () => fileSystem,
        Logger: () => logger,
        OperatingSystemUtils: () => FakeOperatingSystemUtils(hostPlatform: HostPlatform.darwin_x64),
        Pub: () => fakePubBecauseRefreshPluginsList,
        Platform: () => macPlatform,
        XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
        Xcode: () => xcode,
      },
    );

    testUsingContext(
      'with concurrent build failures',
      () async {
        final IOSDevice iosDevice = setUpIOSDevice(
          fileSystem: fileSystem,
          processManager: processManager,
          logger: logger,
          artifacts: artifacts,
        );
        setUpIOSProject(fileSystem);
        final FlutterProject flutterProject = FlutterProject.fromDirectory(
          fileSystem.currentDirectory,
        );
        final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
          flutterProject.ios,
          'flutter',
          'My Super Awesome App',
        );

        processManager.addCommand(FakeCommand(command: _xattrArgs(flutterProject)));
        // The first xcrun call should fail with a
        // concurrent build exception.
        processManager.addCommand(
          const FakeCommand(
            command: kRunReleaseArgs,
            exitCode: 1,
            stdout: kConcurrentBuildErrorMessage,
          ),
        );
        processManager.addCommand(const FakeCommand(command: kRunReleaseArgs));
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              iosDeployPath,
              '--id',
              '123',
              '--bundle',
              'build/ios/iphoneos/My Super Awesome App.app',
              '--app_deltas',
              'build/ios/app-delta',
              '--no-wifi',
              '--justlaunch',
              '--args',
              '--enable-dart-profiling',
            ],
          ),
        );

        final FakeAsync fakeAsync = FakeAsync();
        final Future<LaunchResult> pendingResult = fakeAsync.run((_) async {
          return iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
            platformArgs: <String, Object>{},
          );
        });

        unawaited(
          pendingResult.then(
            expectAsync1((LaunchResult launchResult) {
              expect(
                logger.statusText,
                contains('Xcode build failed due to concurrent builds, will retry in 2 seconds'),
              );
              expect(launchResult.started, true);
              expect(processManager, hasNoRemainingExpectations);
            }),
          ),
        );

        // Wait until all asynchronous time has been elapsed.
        do {
          fakeAsync.elapse(const Duration(seconds: 2));
        } while (fakeAsync.pendingTimers.isNotEmpty);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => processManager,
        FileSystem: () => fileSystem,
        Logger: () => logger,
        OperatingSystemUtils:
            () => FakeOperatingSystemUtils(hostPlatform: HostPlatform.darwin_arm64),
        Platform: () => macPlatform,
        Pub: () => fakePubBecauseRefreshPluginsList,
        XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
        Xcode: () => xcode,
      },
    );
  });

  group('IOSDevice.startApp for CoreDevice', () {
    late FileSystem fileSystem;
    late FakeProcessManager processManager;
    late BufferLogger logger;
    late Xcode xcode;
    late FakeXcodeProjectInterpreter fakeXcodeProjectInterpreter;
    late XcodeProjectInfo projectInfo;

    setUp(() {
      logger = BufferLogger.test();
      fileSystem = MemoryFileSystem.test();
      processManager = FakeProcessManager.empty();
      projectInfo = XcodeProjectInfo(<String>['Runner'], <String>['Debug', 'Release'], <String>[
        'Runner',
      ], logger);
      fakeXcodeProjectInterpreter = FakeXcodeProjectInterpreter(projectInfo: projectInfo);
      xcode = Xcode.test(
        processManager: FakeProcessManager.any(),
        xcodeProjectInterpreter: fakeXcodeProjectInterpreter,
      );
    });

    group('in release mode', () {
      testUsingContext(
        'succeeds when install and launch succeed',
        () async {
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
          );
          setUpIOSProject(fileSystem);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final LaunchResult launchResult = await iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
            platformArgs: <String, Object>{},
          );

          expect(fileSystem.directory('build/ios/iphoneos'), exists);
          expect(launchResult.started, true);
          expect(processManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          Pub: () => fakePubBecauseRefreshPluginsList,
          FileSystem: () => fileSystem,
          Logger: () => logger,
          OperatingSystemUtils: () => os,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
          Xcode: () => xcode,
        },
      );

      testUsingContext(
        'fails when install fails',
        () async {
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(installSuccess: false),
          );
          setUpIOSProject(fileSystem);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final LaunchResult launchResult = await iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
            platformArgs: <String, Object>{},
          );

          expect(fileSystem.directory('build/ios/iphoneos'), exists);
          expect(launchResult.started, false);
          expect(processManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          Pub: () => fakePubBecauseRefreshPluginsList,
          FileSystem: () => fileSystem,
          Logger: () => logger,
          OperatingSystemUtils: () => os,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
          Xcode: () => xcode,
        },
      );

      testUsingContext(
        'fails when launch fails',
        () async {
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(launchSuccess: false),
          );
          setUpIOSProject(fileSystem);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final LaunchResult launchResult = await iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
            platformArgs: <String, Object>{},
          );

          expect(fileSystem.directory('build/ios/iphoneos'), exists);
          expect(launchResult.started, false);
          expect(processManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          Pub: () => fakePubBecauseRefreshPluginsList,
          FileSystem: () => fileSystem,
          Logger: () => logger,
          OperatingSystemUtils: () => os,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
          Xcode: () => xcode,
        },
      );

      testUsingContext(
        'ensure arguments passed to launch',
        () async {
          final FakeIOSCoreDeviceControl coreDeviceControl = FakeIOSCoreDeviceControl();
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: coreDeviceControl,
          );
          setUpIOSProject(fileSystem);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final LaunchResult launchResult = await iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
            platformArgs: <String, Object>{},
          );

          expect(fileSystem.directory('build/ios/iphoneos'), exists);
          expect(launchResult.started, true);
          expect(processManager, hasNoRemainingExpectations);
          expect(coreDeviceControl.argumentsUsedForLaunch, isNotNull);
          expect(coreDeviceControl.argumentsUsedForLaunch, contains('--enable-dart-profiling'));
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          Pub: () => fakePubBecauseRefreshPluginsList,
          FileSystem: () => fileSystem,
          Logger: () => logger,
          OperatingSystemUtils: () => os,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
          Xcode: () => xcode,
        },
      );
    });

    group('in debug mode', () {
      testUsingContext(
        'succeeds',
        () async {
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
            xcodeDebug: FakeXcodeDebug(
              expectedProject: XcodeDebugProject(
                scheme: 'Runner',
                xcodeWorkspace: fileSystem.directory('/ios/Runner.xcworkspace'),
                xcodeProject: fileSystem.directory('/ios/Runner.xcodeproj'),
                hostAppProjectName: 'Runner',
              ),
              expectedDeviceId: '123',
              expectedLaunchArguments: <String>['--enable-dart-profiling'],
            ),
          );

          setUpIOSProject(fileSystem);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

          iosDevice.portForwarder = const NoOpDevicePortForwarder();
          iosDevice.setLogReader(buildableIOSApp, deviceLogReader);

          // Start writing messages to the log reader.
          Timer.run(() {
            deviceLogReader.addLine('Foo');
            deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
          });

          final LaunchResult launchResult = await iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.enabled(
              const BuildInfo(
                BuildMode.debug,
                null,
                buildName: '1.2.3',
                buildNumber: '4',
                treeShakeIcons: false,
                packageConfigPath: '.dart_tool/package_config.json',
              ),
            ),
            platformArgs: <String, Object>{},
          );

          expect(logger.errorText, isEmpty);
          expect(fileSystem.directory('build/ios/iphoneos'), exists);
          expect(launchResult.started, true);
          expect(processManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          Pub: () => fakePubBecauseRefreshPluginsList,
          FileSystem: () => fileSystem,
          Logger: () => logger,
          OperatingSystemUtils: () => os,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
          Xcode: () => xcode,
        },
      );

      group('with flavor', () {
        setUp(() {
          projectInfo = XcodeProjectInfo(
            <String>['Runner'],
            <String>['Debug', 'Release', 'Debug-free', 'Release-free'],
            <String>['Runner', 'free'],
            logger,
          );
          fakeXcodeProjectInterpreter = FakeXcodeProjectInterpreter(projectInfo: projectInfo);
          xcode = Xcode.test(
            processManager: FakeProcessManager.any(),
            xcodeProjectInterpreter: fakeXcodeProjectInterpreter,
          );
        });

        testUsingContext(
          'succeeds',
          () async {
            final IOSDevice iosDevice = setUpIOSDevice(
              fileSystem: fileSystem,
              processManager: FakeProcessManager.any(),
              logger: logger,
              artifacts: artifacts,
              isCoreDevice: true,
              coreDeviceControl: FakeIOSCoreDeviceControl(),
              xcodeDebug: FakeXcodeDebug(
                expectedProject: XcodeDebugProject(
                  scheme: 'free',
                  xcodeWorkspace: fileSystem.directory('/ios/Runner.xcworkspace'),
                  xcodeProject: fileSystem.directory('/ios/Runner.xcodeproj'),
                  hostAppProjectName: 'Runner',
                ),
                expectedDeviceId: '123',
                expectedLaunchArguments: <String>['--enable-dart-profiling'],
                expectedSchemeFilePath:
                    '/ios/Runner.xcodeproj/xcshareddata/xcschemes/free.xcscheme',
              ),
            );

            setUpIOSProject(fileSystem);
            final FlutterProject flutterProject = FlutterProject.fromDirectory(
              fileSystem.currentDirectory,
            );
            final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
              flutterProject.ios,
              'flutter',
              'My Super Awesome App',
            );
            fileSystem
                .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
                .createSync(recursive: true);

            final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

            iosDevice.portForwarder = const NoOpDevicePortForwarder();
            iosDevice.setLogReader(buildableIOSApp, deviceLogReader);

            // Start writing messages to the log reader.
            Timer.run(() {
              deviceLogReader.addLine('Foo');
              deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
            });

            final LaunchResult launchResult = await iosDevice.startApp(
              buildableIOSApp,
              debuggingOptions: DebuggingOptions.enabled(
                const BuildInfo(
                  BuildMode.debug,
                  'free',
                  buildName: '1.2.3',
                  buildNumber: '4',
                  treeShakeIcons: false,
                  packageConfigPath: '.dart_tool/package_config.json',
                ),
              ),
              platformArgs: <String, Object>{},
            );

            expect(logger.errorText, isEmpty);
            expect(fileSystem.directory('build/ios/iphoneos'), exists);
            expect(launchResult.started, true);
            expect(processManager, hasNoRemainingExpectations);
          },
          overrides: <Type, Generator>{
            ProcessManager: () => FakeProcessManager.any(),
            Pub: () => fakePubBecauseRefreshPluginsList,
            FileSystem: () => fileSystem,
            Logger: () => logger,
            OperatingSystemUtils: () => os,
            Platform: () => macPlatform,
            XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
            Xcode: () => xcode,
          },
        );
      });

      testUsingContext(
        'updates Generated.xcconfig before and after launch',
        () async {
          final Completer<void> debugStartedCompleter = Completer<void>();
          final Completer<void> debugEndedCompleter = Completer<void>();
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
            xcodeDebug: FakeXcodeDebug(
              expectedProject: XcodeDebugProject(
                scheme: 'Runner',
                xcodeWorkspace: fileSystem.directory('/ios/Runner.xcworkspace'),
                xcodeProject: fileSystem.directory('/ios/Runner.xcodeproj'),
                hostAppProjectName: 'Runner',
                expectedConfigurationBuildDir: '/build/ios/iphoneos',
              ),
              expectedDeviceId: '123',
              expectedLaunchArguments: <String>['--enable-dart-profiling'],
              debugStartedCompleter: debugStartedCompleter,
              debugEndedCompleter: debugEndedCompleter,
            ),
          );

          setUpIOSProject(fileSystem);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

          iosDevice.portForwarder = const NoOpDevicePortForwarder();
          iosDevice.setLogReader(buildableIOSApp, deviceLogReader);

          // Start writing messages to the log reader.
          Timer.run(() {
            deviceLogReader.addLine('Foo');
            deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
          });

          final Future<LaunchResult> futureLaunchResult = iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.enabled(
              const BuildInfo(
                BuildMode.debug,
                null,
                buildName: '1.2.3',
                buildNumber: '4',
                treeShakeIcons: false,
                packageConfigPath: '.dart_tool/package_config.json',
              ),
            ),
            platformArgs: <String, Object>{},
          );

          await debugStartedCompleter.future;

          // Validate CoreDevice build settings were used
          final File config = fileSystem.directory('ios').childFile('Flutter/Generated.xcconfig');
          expect(config.existsSync(), isTrue);

          String contents = config.readAsStringSync();
          expect(contents, contains('CONFIGURATION_BUILD_DIR=/build/ios/iphoneos'));

          debugEndedCompleter.complete();

          await futureLaunchResult;

          // Validate CoreDevice build settings were removed after launch
          contents = config.readAsStringSync();
          expect(contents.contains('CONFIGURATION_BUILD_DIR'), isFalse);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          Pub: () => fakePubBecauseRefreshPluginsList,
          FileSystem: () => fileSystem,
          Logger: () => logger,
          OperatingSystemUtils: () => os,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
          Xcode: () => xcode,
        },
      );

      testUsingContext(
        'fails when Xcode project is not found',
        () async {
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
          );
          setUpIOSProject(fileSystem);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final LaunchResult launchResult = await iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.enabled(
              const BuildInfo(
                BuildMode.debug,
                null,
                buildName: '1.2.3',
                buildNumber: '4',
                treeShakeIcons: false,
                packageConfigPath: '.dart_tool/package_config.json',
              ),
            ),
            platformArgs: <String, Object>{},
          );
          expect(logger.errorText, contains('Xcode project not found'));
          expect(fileSystem.directory('build/ios/iphoneos'), exists);
          expect(launchResult.started, false);
          expect(processManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          Pub: () => fakePubBecauseRefreshPluginsList,
          FileSystem: () => fileSystem,
          Logger: () => logger,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => FakeXcodeProjectInterpreter(),
          Xcode: () => xcode,
        },
      );

      testUsingContext(
        'fails when Xcode workspace is not found',
        () async {
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
          );
          setUpIOSProject(fileSystem, createWorkspace: false);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final LaunchResult launchResult = await iosDevice.startApp(
            buildableIOSApp,
            debuggingOptions: DebuggingOptions.enabled(
              const BuildInfo(
                BuildMode.debug,
                null,
                buildName: '1.2.3',
                buildNumber: '4',
                treeShakeIcons: false,
                packageConfigPath: '.dart_tool/package_config.json',
              ),
            ),
            platformArgs: <String, Object>{},
          );
          expect(logger.errorText, contains('Unable to get Xcode workspace'));
          expect(fileSystem.directory('build/ios/iphoneos'), exists);
          expect(launchResult.started, false);
          expect(processManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          Pub: () => fakePubBecauseRefreshPluginsList,
          FileSystem: () => fileSystem,
          Logger: () => logger,
          OperatingSystemUtils: () => os,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
          Xcode: () => xcode,
        },
      );

      testUsingContext(
        'fails when scheme is not found',
        () async {
          final IOSDevice iosDevice = setUpIOSDevice(
            fileSystem: fileSystem,
            processManager: FakeProcessManager.any(),
            logger: logger,
            artifacts: artifacts,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
          );
          setUpIOSProject(fileSystem);
          final FlutterProject flutterProject = FlutterProject.fromDirectory(
            fileSystem.currentDirectory,
          );
          final BuildableIOSApp buildableIOSApp = BuildableIOSApp(
            flutterProject.ios,
            'flutter',
            'My Super Awesome App',
          );
          fileSystem
              .directory('build/ios/Release-iphoneos/My Super Awesome App.app')
              .createSync(recursive: true);

          final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

          iosDevice.portForwarder = const NoOpDevicePortForwarder();
          iosDevice.setLogReader(buildableIOSApp, deviceLogReader);

          // Start writing messages to the log reader.
          Timer.run(() {
            deviceLogReader.addLine('Foo');
            deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
          });

          expect(
            () async => iosDevice.startApp(
              buildableIOSApp,
              debuggingOptions: DebuggingOptions.enabled(
                const BuildInfo(
                  BuildMode.debug,
                  'Flavor',
                  buildName: '1.2.3',
                  buildNumber: '4',
                  treeShakeIcons: false,
                  packageConfigPath: '.dart_tool/package_config.json',
                ),
              ),
              platformArgs: <String, Object>{},
            ),
            throwsToolExit(),
          );
        },
        overrides: <Type, Generator>{
          ProcessManager: () => FakeProcessManager.any(),
          FileSystem: () => fileSystem,
          Logger: () => logger,
          Platform: () => macPlatform,
          XcodeProjectInterpreter: () => fakeXcodeProjectInterpreter,
          Xcode: () => xcode,
        },
      );
    });
  });
}

void setUpIOSProject(FileSystem fileSystem, {bool createWorkspace = true}) {
  fileSystem.file('pubspec.yaml').createSync();
  fileSystem.directory('.dart_tool').childFile('package_config.json').createSync(recursive: true);
  fileSystem.directory('ios').createSync();
  if (createWorkspace) {
    fileSystem.directory('ios/Runner.xcworkspace').createSync();
  }
  fileSystem.file('ios/Runner.xcodeproj/project.pbxproj').createSync(recursive: true);
  // This is the expected output directory.
  fileSystem.directory('build/ios/iphoneos/My Super Awesome App.app').createSync(recursive: true);
}

IOSDevice setUpIOSDevice({
  String sdkVersion = '13.0.1',
  FileSystem? fileSystem,
  Logger? logger,
  ProcessManager? processManager,
  Artifacts? artifacts,
  bool isCoreDevice = false,
  IOSCoreDeviceControl? coreDeviceControl,
  FakeXcodeDebug? xcodeDebug,
  DarwinArch cpuArchitecture = DarwinArch.arm64,
}) {
  artifacts ??= Artifacts.test();
  final Cache cache = Cache.test(
    artifacts: <ArtifactSet>[FakeDyldEnvironmentArtifact()],
    processManager: FakeProcessManager.any(),
  );

  logger ??= BufferLogger.test();
  return IOSDevice(
    '123',
    name: 'iPhone 1',
    sdkVersion: sdkVersion,
    fileSystem: fileSystem ?? MemoryFileSystem.test(),
    platform: macPlatform,
    iProxy: IProxy.test(logger: logger, processManager: processManager ?? FakeProcessManager.any()),
    logger: logger,
    iosDeploy: IOSDeploy(
      logger: logger,
      platform: macPlatform,
      processManager: processManager ?? FakeProcessManager.any(),
      artifacts: artifacts,
      cache: cache,
    ),
    iMobileDevice: IMobileDevice(
      logger: logger,
      processManager: processManager ?? FakeProcessManager.any(),
      artifacts: artifacts,
      cache: cache,
    ),
    coreDeviceControl: coreDeviceControl ?? FakeIOSCoreDeviceControl(),
    xcodeDebug: xcodeDebug ?? FakeXcodeDebug(),
    cpuArchitecture: cpuArchitecture,
    connectionInterface: DeviceConnectionInterface.attached,
    isConnected: true,
    isPaired: true,
    devModeEnabled: true,
    isCoreDevice: isCoreDevice,
  );
}

class FakeXcodeProjectInterpreter extends Fake implements XcodeProjectInterpreter {
  FakeXcodeProjectInterpreter({
    this.projectInfo,
    this.buildSettings = const <String, String>{
      'TARGET_BUILD_DIR': 'build/ios/Release-iphoneos',
      'WRAPPER_NAME': 'My Super Awesome App.app',
      'DEVELOPMENT_TEAM': '3333CCCC33',
    },
  });

  final Map<String, String> buildSettings;
  final XcodeProjectInfo? projectInfo;

  @override
  final bool isInstalled = true;

  @override
  final Version version = Version(1000, 0, 0);

  @override
  String get versionText => version.toString();

  @override
  List<String> xcrunCommand() => <String>['xcrun'];

  @override
  Future<XcodeProjectInfo?> getInfo(String projectPath, {String? projectFilename}) async =>
      projectInfo;

  @override
  Future<Map<String, String>> getBuildSettings(
    String projectPath, {
    required XcodeProjectBuildContext buildContext,
    Duration timeout = const Duration(minutes: 1),
  }) async => buildSettings;
}

class FakeXcodeDebug extends Fake implements XcodeDebug {
  FakeXcodeDebug({
    this.debugSuccess = true,
    this.expectedProject,
    this.expectedDeviceId,
    this.expectedLaunchArguments,
    this.expectedSchemeFilePath = '/ios/Runner.xcodeproj/xcshareddata/xcschemes/Runner.xcscheme',
    this.debugStartedCompleter,
    this.debugEndedCompleter,
  });

  final bool debugSuccess;

  final XcodeDebugProject? expectedProject;
  final String? expectedDeviceId;
  final List<String>? expectedLaunchArguments;
  final Completer<void>? debugStartedCompleter;
  final Completer<void>? debugEndedCompleter;
  final String expectedSchemeFilePath;

  @override
  Future<bool> debugApp({
    required XcodeDebugProject project,
    required String deviceId,
    required List<String> launchArguments,
  }) async {
    debugStartedCompleter?.complete();
    if (expectedProject != null) {
      expect(project.scheme, expectedProject!.scheme);
      expect(project.xcodeWorkspace.path, expectedProject!.xcodeWorkspace.path);
      expect(project.xcodeProject.path, expectedProject!.xcodeProject.path);
      expect(project.isTemporaryProject, expectedProject!.isTemporaryProject);
    }
    if (expectedDeviceId != null) {
      expect(deviceId, expectedDeviceId);
    }
    if (expectedLaunchArguments != null) {
      expect(expectedLaunchArguments, launchArguments);
    }
    await debugEndedCompleter?.future;
    return debugSuccess;
  }

  @override
  void ensureXcodeDebuggerLaunchAction(File schemeFile) {
    expect(schemeFile.path, expectedSchemeFilePath);
  }
}

class FakeIOSCoreDeviceControl extends Fake implements IOSCoreDeviceControl {
  FakeIOSCoreDeviceControl({this.installSuccess = true, this.launchSuccess = true});

  final bool installSuccess;
  final bool launchSuccess;
  List<String>? _launchArguments;

  List<String>? get argumentsUsedForLaunch => _launchArguments;

  @override
  Future<bool> installApp({required String deviceId, required String bundlePath}) async {
    return installSuccess;
  }

  @override
  Future<bool> launchApp({
    required String deviceId,
    required String bundleId,
    List<String> launchArguments = const <String>[],
  }) async {
    _launchArguments = launchArguments;
    return launchSuccess;
  }
}
