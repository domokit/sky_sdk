// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/ios/code_signing.dart';
import 'package:flutter_tools/src/ios/mac.dart';
import 'package:flutter_tools/src/ios/xcresult.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fake_pub_deps.dart';
import '../../src/fakes.dart';

void main() {
  late BufferLogger logger;

  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(isExplicitPackageDependenciesEnabled: true);
  }

  setUp(() {
    logger = BufferLogger.test();
  });

  group('IMobileDevice', () {
    late Artifacts artifacts;
    late Cache cache;

    setUp(() {
      artifacts = Artifacts.test();
      cache = Cache.test(
        artifacts: <ArtifactSet>[FakeDyldEnvironmentArtifact()],
        processManager: FakeProcessManager.any(),
      );
    });

    group('startLogger', () {
      testWithoutContext('starts idevicesyslog when USB connected', () async {
        final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>['HostArtifact.idevicesyslog', '-u', '1234'],
            environment: <String, String>{'DYLD_LIBRARY_PATH': '/path/to/libraries'},
          ),
        ]);

        final IMobileDevice iMobileDevice = IMobileDevice(
          artifacts: artifacts,
          cache: cache,
          processManager: fakeProcessManager,
          logger: logger,
        );

        await iMobileDevice.startLogger('1234', false);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });

      testWithoutContext('starts idevicesyslog when wirelessly connected', () async {
        final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>['HostArtifact.idevicesyslog', '-u', '1234', '--network'],
            environment: <String, String>{'DYLD_LIBRARY_PATH': '/path/to/libraries'},
          ),
        ]);

        final IMobileDevice iMobileDevice = IMobileDevice(
          artifacts: artifacts,
          cache: cache,
          processManager: fakeProcessManager,
          logger: logger,
        );

        await iMobileDevice.startLogger('1234', true);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });
    });

    group('screenshot', () {
      late FakeProcessManager fakeProcessManager;
      late File outputFile;

      setUp(() {
        fakeProcessManager = FakeProcessManager.empty();
        outputFile = MemoryFileSystem.test().file('image.png');
      });

      testWithoutContext('error if idevicescreenshot is not installed', () async {
        // Let `idevicescreenshot` fail with exit code 1.
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>['HostArtifact.idevicescreenshot', outputFile.path, '--udid', '1234'],
            environment: const <String, String>{'DYLD_LIBRARY_PATH': '/path/to/libraries'},
            exitCode: 1,
          ),
        );

        final IMobileDevice iMobileDevice = IMobileDevice(
          artifacts: artifacts,
          cache: cache,
          processManager: fakeProcessManager,
          logger: logger,
        );

        expect(
          () async =>
              iMobileDevice.takeScreenshot(outputFile, '1234', DeviceConnectionInterface.attached),
          throwsA(anything),
        );
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });

      testWithoutContext('idevicescreenshot captures and returns USB screenshot', () async {
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>['HostArtifact.idevicescreenshot', outputFile.path, '--udid', '1234'],
            environment: const <String, String>{'DYLD_LIBRARY_PATH': '/path/to/libraries'},
          ),
        );

        final IMobileDevice iMobileDevice = IMobileDevice(
          artifacts: artifacts,
          cache: cache,
          processManager: fakeProcessManager,
          logger: logger,
        );

        await iMobileDevice.takeScreenshot(outputFile, '1234', DeviceConnectionInterface.attached);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });

      testWithoutContext('idevicescreenshot captures and returns network screenshot', () async {
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'HostArtifact.idevicescreenshot',
              outputFile.path,
              '--udid',
              '1234',
              '--network',
            ],
            environment: const <String, String>{'DYLD_LIBRARY_PATH': '/path/to/libraries'},
          ),
        );

        final IMobileDevice iMobileDevice = IMobileDevice(
          artifacts: artifacts,
          cache: cache,
          processManager: fakeProcessManager,
          logger: logger,
        );

        await iMobileDevice.takeScreenshot(outputFile, '1234', DeviceConnectionInterface.wireless);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });
    });
  });

  group('Diagnose Xcode build failure', () {
    late Map<String, String> buildSettings;
    late FakeAnalytics fakeAnalytics;

    setUp(() {
      buildSettings = <String, String>{'PRODUCT_BUNDLE_IDENTIFIER': 'test.app'};

      final MemoryFileSystem fs = MemoryFileSystem.test();
      fakeAnalytics = getInitializedFakeAnalyticsInstance(
        fs: fs,
        fakeFlutterVersion: FakeFlutterVersion(),
      );
    });

    testWithoutContext('Sends analytics when bitcode fails', () async {
      const List<String> buildCommands = <String>['xcrun', 'cc', 'blah'];
      final XcodeBuildResult buildResult = XcodeBuildResult(
        success: false,
        stdout: 'BITCODE_ENABLED = YES',
        xcodeBuildExecution: XcodeBuildExecution(
          buildCommands: buildCommands,
          appDirectory: '/blah/blah',
          environmentType: EnvironmentType.physical,
          buildSettings: buildSettings,
        ),
      );
      final MemoryFileSystem fs = MemoryFileSystem.test();
      await diagnoseXcodeBuildFailure(
        buildResult,
        logger: logger,
        analytics: fakeAnalytics,
        fileSystem: fs,
        platform: SupportedPlatform.ios,
        project: FakeFlutterProject(fileSystem: fs),
      );
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.flutterBuildInfo(
            label: 'xcode-bitcode-failure',
            buildType: 'ios',
            command: '[xcrun, cc, blah]',
            settings: '{PRODUCT_BUNDLE_IDENTIFIER: test.app}',
          ),
        ),
      );
    });

    testWithoutContext('fallback to stdout: No provisioning profile shows message', () async {
      final Map<String, String> buildSettingsWithDevTeam = <String, String>{
        'PRODUCT_BUNDLE_IDENTIFIER': 'test.app',
        'DEVELOPMENT_TEAM': 'a team',
      };
      final XcodeBuildResult buildResult = XcodeBuildResult(
        success: false,
        stdout: '''
Launching lib/main.dart on iPhone in debug mode...
Signing iOS app for device deployment using developer identity: "iPhone Developer: test@flutter.io (1122334455)"
Running Xcode build...                                1.3s
Failed to build iOS app
Error output from Xcode build:
↳
    ** BUILD FAILED **


    The following build commands failed:
    	Check dependencies
    (1 failure)
Xcode's output:
↳
    Build settings from command line:
        ARCHS = arm64
        BUILD_DIR = /Users/blah/blah
        DEVELOPMENT_TEAM = AABBCCDDEE
        ONLY_ACTIVE_ARCH = YES
        SDKROOT = iphoneos10.3

    === CLEAN TARGET Runner OF PROJECT Runner WITH CONFIGURATION Release ===

    Check dependencies
    [BCEROR]"Runner" requires a provisioning profile. Select a provisioning profile in the Signing & Capabilities editor.
    [BCEROR]Code signing is required for product type 'Application' in SDK 'iOS 10.3'
    [BCEROR]Code signing is required for product type 'Application' in SDK 'iOS 10.3'
    [BCEROR]Code signing is required for product type 'Application' in SDK 'iOS 10.3'

    Create product structure
    /bin/mkdir -p /Users/blah/Runner.app

    Clean.Remove clean /Users/blah/Runner.app.dSYM
        builtin-rm -rf /Users/blah/Runner.app.dSYM

    Clean.Remove clean /Users/blah/Runner.app
        builtin-rm -rf /Users/blah/Runner.app

    Clean.Remove clean /Users/blah/Runner-dfvicjniknvzghgwsthwtgcjhtsk/Build/Intermediates/Runner.build/Release-iphoneos/Runner.build
        builtin-rm -rf /Users/blah/Runner-dfvicjniknvzghgwsthwtgcjhtsk/Build/Intermediates/Runner.build/Release-iphoneos/Runner.build

    ** CLEAN SUCCEEDED **

    === BUILD TARGET Runner OF PROJECT Runner WITH CONFIGURATION Release ===

    Check dependencies
    No profiles for 'com.example.test' were found:  Xcode couldn't find a provisioning profile matching 'com.example.test'.
    Code signing is required for product type 'Application' in SDK 'iOS 10.3'
    Code signing is required for product type 'Application' in SDK 'iOS 10.3'
    Code signing is required for product type 'Application' in SDK 'iOS 10.3'

Could not build the precompiled application for the device.

Error launching application on iPhone.''',
        xcodeBuildExecution: XcodeBuildExecution(
          buildCommands: <String>['xcrun', 'xcodebuild', 'blah'],
          appDirectory: '/blah/blah',
          environmentType: EnvironmentType.physical,
          buildSettings: buildSettingsWithDevTeam,
        ),
      );
      final MemoryFileSystem fs = MemoryFileSystem.test();
      await diagnoseXcodeBuildFailure(
        buildResult,
        logger: logger,
        analytics: fakeAnalytics,
        fileSystem: fs,
        platform: SupportedPlatform.ios,
        project: FakeFlutterProject(fileSystem: fs),
      );
      expect(logger.errorText, contains(noProvisioningProfileInstruction));
    });

    testWithoutContext('fallback to stdout: Ineligible destinations', () async {
      final Map<String, String> buildSettingsWithDevTeam = <String, String>{
        'PRODUCT_BUNDLE_IDENTIFIER': 'test.app',
        'DEVELOPMENT_TEAM': 'a team',
      };
      final XcodeBuildResult buildResult = XcodeBuildResult(
        success: false,
        stderr: '''
Launching lib/main.dart on iPhone in debug mode...
Signing iOS app for device deployment using developer identity: "iPhone Developer: test@flutter.io (1122334455)"
Running Xcode build...                                1.3s
Failed to build iOS app
Error output from Xcode build:
↳
    xcodebuild: error: Unable to find a destination matching the provided destination specifier:
               		{ id:1234D567-890C-1DA2-34E5-F6789A0123C4 }

               	Ineligible destinations for the "Runner" scheme:
               		{ platform:iOS, id:dvtdevice-DVTiPhonePlaceholder-iphoneos:placeholder, name:Any iOS Device, error:iOS 17.0 is not installed. To use with Xcode, first download and install the platform }

Could not build the precompiled application for the device.

Error launching application on iPhone.''',
        xcodeBuildExecution: XcodeBuildExecution(
          buildCommands: <String>['xcrun', 'xcodebuild', 'blah'],
          appDirectory: '/blah/blah',
          environmentType: EnvironmentType.physical,
          buildSettings: buildSettingsWithDevTeam,
        ),
      );
      final MemoryFileSystem fs = MemoryFileSystem.test();
      await diagnoseXcodeBuildFailure(
        buildResult,
        logger: logger,
        analytics: fakeAnalytics,
        fileSystem: fs,
        platform: SupportedPlatform.ios,
        project: FakeFlutterProject(fileSystem: fs),
      );
      expect(logger.errorText, contains(missingPlatformInstructions('iOS 17.0')));
    });

    testWithoutContext('No development team shows message', () async {
      final XcodeBuildResult buildResult = XcodeBuildResult(
        success: false,
        stdout: '''
Running "flutter pub get" in flutter_gallery...  0.6s
Launching lib/main.dart on x in release mode...
Running pod install...                                1.2s
Running Xcode build...                                1.4s
Failed to build iOS app
Error output from Xcode build:
↳
    ** BUILD FAILED **


    The following build commands failed:
    	Check dependencies
    (1 failure)
Xcode's output:
↳
    blah

    Check dependencies
    [BCEROR]Signing for "Runner" requires a development team. Select a development team in the project editor.

Could not build the precompiled application for the device.''',
        xcodeBuildExecution: XcodeBuildExecution(
          buildCommands: <String>['xcrun', 'xcodebuild', 'blah'],
          appDirectory: '/blah/blah',
          environmentType: EnvironmentType.physical,
          buildSettings: buildSettings,
        ),
      );
      final MemoryFileSystem fs = MemoryFileSystem.test();
      await diagnoseXcodeBuildFailure(
        buildResult,
        logger: logger,
        analytics: fakeAnalytics,
        fileSystem: fs,
        platform: SupportedPlatform.ios,
        project: FakeFlutterProject(fileSystem: fs),
      );
      expect(
        logger.errorText,
        contains(
          'Building a deployable iOS app requires a selected Development Team with a \nProvisioning Profile.',
        ),
      );
    });

    testWithoutContext(
      'does not show no development team message when other Xcode issues detected',
      () async {
        final XcodeBuildResult buildResult = XcodeBuildResult(
          success: false,
          stdout: '''
Running "flutter pub get" in flutter_gallery...  0.6s
Launching lib/main.dart on x in release mode...
Running pod install...                                1.2s
Running Xcode build...                                1.4s
Failed to build iOS app
Error output from Xcode build:
↳
    ** BUILD FAILED **


    The following build commands failed:
    	Check dependencies
    (1 failure)
Xcode's output:
↳
    blah

    Check dependencies
    [BCEROR]Signing for "Runner" requires a development team. Select a development team in the project editor.

Could not build the precompiled application for the device.''',
          xcodeBuildExecution: XcodeBuildExecution(
            buildCommands: <String>['xcrun', 'xcodebuild', 'blah'],
            appDirectory: '/blah/blah',
            environmentType: EnvironmentType.physical,
            buildSettings: buildSettings,
          ),
          xcResult: XCResult.test(
            issues: <XCResultIssue>[
              XCResultIssue.test(message: 'Target aot_assembly_release failed', subType: 'Error'),
            ],
          ),
        );

        final MemoryFileSystem fs = MemoryFileSystem.test();
        await diagnoseXcodeBuildFailure(
          buildResult,
          logger: logger,
          analytics: fakeAnalytics,
          fileSystem: fs,
          platform: SupportedPlatform.ios,
          project: FakeFlutterProject(fileSystem: fs),
        );
        expect(logger.errorText, contains('Error (Xcode): Target aot_assembly_release failed'));
        expect(
          logger.errorText,
          isNot(contains('Building a deployable iOS app requires a selected Development Team')),
        );
      },
    );

    testWithoutContext('parses redefinition of module error', () async {
      const List<String> buildCommands = <String>['xcrun', 'cc', 'blah'];
      final XcodeBuildResult buildResult = XcodeBuildResult(
        success: false,
        stdout: '',
        xcodeBuildExecution: XcodeBuildExecution(
          buildCommands: buildCommands,
          appDirectory: '/blah/blah',
          environmentType: EnvironmentType.physical,
          buildSettings: buildSettings,
        ),
        xcResult: XCResult.test(
          issues: <XCResultIssue>[
            XCResultIssue.test(message: "Redefinition of module 'plugin_1_name'", subType: 'Error'),
            XCResultIssue.test(message: "Redefinition of module 'plugin_2_name'", subType: 'Error'),
          ],
        ),
      );
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final FakeFlutterProject project = FakeFlutterProject(
        fileSystem: fs,
        usesSwiftPackageManager: true,
      );
      project.ios.podfile.createSync(recursive: true);
      await diagnoseXcodeBuildFailure(
        buildResult,
        logger: logger,
        analytics: fakeAnalytics,
        fileSystem: fs,
        platform: SupportedPlatform.ios,
        project: project,
      );
      expect(
        logger.errorText,
        contains(
          'Your project uses both CocoaPods and Swift Package Manager, which can '
          'cause the above error. It may be caused by there being both a CocoaPod '
          'and Swift Package Manager dependency for the following module(s): '
          'plugin_1_name, plugin_2_name.',
        ),
      );
    });

    testWithoutContext('parses duplicate symbols error with arch and number', () async {
      const List<String> buildCommands = <String>['xcrun', 'cc', 'blah'];
      final XcodeBuildResult buildResult = XcodeBuildResult(
        success: false,
        stdout: r'''
duplicate symbol '_$s29plugin_1_name23PluginNamePluginC9setDouble3key5valueySS_SdtF' in:
               /Users/username/path/to/app/build/ios/Debug-iphonesimulator/plugin_1_name/plugin_1_name.framework/plugin_1_name[arm64][5](PluginNamePlugin.o)
               /Users/username/path/to/app/build/ios/Debug-iphonesimulator/plugin_1_name.o
           duplicate symbol '_$s29plugin_1_name23PluginNamePluginCAA15UserDefaultsApiAAWP' in:
               /Users/username/path/to/app/build/ios/Debug-iphonesimulator/plugin_1_name/plugin_1_name.framework/plugin_1_name[arm64][5](PluginNamePlugin.o)
               /Users/username/path/to/app/build/ios/Debug-iphonesimulator/plugin_1_name.o
''',
        xcodeBuildExecution: XcodeBuildExecution(
          buildCommands: buildCommands,
          appDirectory: '/blah/blah',
          environmentType: EnvironmentType.physical,
          buildSettings: buildSettings,
        ),
        xcResult: XCResult.test(
          issues: <XCResultIssue>[
            XCResultIssue.test(message: '37 duplicate symbols', subType: 'Error'),
          ],
        ),
      );
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final FakeFlutterProject project = FakeFlutterProject(
        fileSystem: fs,
        usesSwiftPackageManager: true,
      );
      project.ios.podfile.createSync(recursive: true);
      await diagnoseXcodeBuildFailure(
        buildResult,
        logger: logger,
        analytics: fakeAnalytics,
        fileSystem: fs,
        platform: SupportedPlatform.ios,
        project: project,
      );
      expect(
        logger.errorText,
        contains(
          'Your project uses both CocoaPods and Swift Package Manager, which can '
          'cause the above error. It may be caused by there being both a CocoaPod '
          'and Swift Package Manager dependency for the following module(s): '
          'plugin_1_name.',
        ),
      );
    });

    testWithoutContext('parses duplicate symbols error with number', () async {
      const List<String> buildCommands = <String>['xcrun', 'cc', 'blah'];
      final XcodeBuildResult buildResult = XcodeBuildResult(
        success: false,
        stdout: r'''
duplicate symbol '_$s29plugin_1_name23PluginNamePluginC9setDouble3key5valueySS_SdtF' in:
               /Users/username/path/to/app/build/ios/Debug-iphonesimulator/plugin_1_name/plugin_1_name.framework/plugin_1_name[5](PluginNamePlugin.o)
               /Users/username/path/to/app/build/ios/Debug-iphonesimulator/plugin_1_name.o
''',
        xcodeBuildExecution: XcodeBuildExecution(
          buildCommands: buildCommands,
          appDirectory: '/blah/blah',
          environmentType: EnvironmentType.physical,
          buildSettings: buildSettings,
        ),
        xcResult: XCResult.test(
          issues: <XCResultIssue>[
            XCResultIssue.test(message: '37 duplicate symbols', subType: 'Error'),
          ],
        ),
      );
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final FakeFlutterProject project = FakeFlutterProject(
        fileSystem: fs,
        usesSwiftPackageManager: true,
      );
      project.ios.podfile.createSync(recursive: true);
      await diagnoseXcodeBuildFailure(
        buildResult,
        logger: logger,
        analytics: fakeAnalytics,
        fileSystem: fs,
        platform: SupportedPlatform.ios,
        project: project,
      );
      expect(
        logger.errorText,
        contains(
          'Your project uses both CocoaPods and Swift Package Manager, which can '
          'cause the above error. It may be caused by there being both a CocoaPod '
          'and Swift Package Manager dependency for the following module(s): '
          'plugin_1_name.',
        ),
      );
    });

    testWithoutContext('parses duplicate symbols error without arch and number', () async {
      const List<String> buildCommands = <String>['xcrun', 'cc', 'blah'];
      final XcodeBuildResult buildResult = XcodeBuildResult(
        success: false,
        stdout: r'''
duplicate symbol '_$s29plugin_1_name23PluginNamePluginC9setDouble3key5valueySS_SdtF' in:
               /Users/username/path/to/app/build/ios/Debug-iphonesimulator/plugin_1_name/plugin_1_name.framework/plugin_1_name(PluginNamePlugin.o)
''',
        xcodeBuildExecution: XcodeBuildExecution(
          buildCommands: buildCommands,
          appDirectory: '/blah/blah',
          environmentType: EnvironmentType.physical,
          buildSettings: buildSettings,
        ),
        xcResult: XCResult.test(
          issues: <XCResultIssue>[
            XCResultIssue.test(message: '37 duplicate symbols', subType: 'Error'),
          ],
        ),
      );
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final FakeFlutterProject project = FakeFlutterProject(
        fileSystem: fs,
        usesSwiftPackageManager: true,
      );
      project.ios.podfile.createSync(recursive: true);
      await diagnoseXcodeBuildFailure(
        buildResult,
        logger: logger,
        analytics: fakeAnalytics,
        fileSystem: fs,
        platform: SupportedPlatform.ios,
        project: project,
      );
      expect(
        logger.errorText,
        contains(
          'Your project uses both CocoaPods and Swift Package Manager, which can '
          'cause the above error. It may be caused by there being both a CocoaPod '
          'and Swift Package Manager dependency for the following module(s): '
          'plugin_1_name.',
        ),
      );
    });

    testUsingContext(
      'parses missing module error',
      () async {
        const List<String> buildCommands = <String>['xcrun', 'cc', 'blah'];
        final XcodeBuildResult buildResult = XcodeBuildResult(
          success: false,
          stdout: '',
          xcodeBuildExecution: XcodeBuildExecution(
            buildCommands: buildCommands,
            appDirectory: '/blah/blah',
            environmentType: EnvironmentType.physical,
            buildSettings: buildSettings,
          ),
          xcResult: XCResult.test(
            issues: <XCResultIssue>[
              XCResultIssue.test(message: "Module 'plugin_1_name' not found", subType: 'Error'),
              XCResultIssue.test(message: "Module 'plugin_2_name' not found", subType: 'Error'),
            ],
          ),
        );
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final FakeFlutterProject project = FakeFlutterProject(fileSystem: fs);
        project.ios.podfile.createSync(recursive: true);
        project.manifest = FakeFlutterManifest();
        createFakePlugins(project, fs, <String>['plugin_1_name', 'plugin_2_name']);
        fs.systemTempDirectory
            .childFile('cache/plugin_1_name/ios/plugin_1_name/Package.swift')
            .createSync(recursive: true);
        fs.systemTempDirectory
            .childFile('cache/plugin_2_name/ios/plugin_2_name/Package.swift')
            .createSync(recursive: true);
        await diagnoseXcodeBuildFailure(
          buildResult,
          logger: logger,
          analytics: fakeAnalytics,
          fileSystem: fs,
          platform: SupportedPlatform.ios,
          project: project,
        );
        expect(
          logger.errorText,
          contains(
            'Your project uses CocoaPods as a dependency manager, but the following plugin(s) '
            'only support Swift Package Manager: plugin_1_name, plugin_2_name.',
          ),
        );
      },
      overrides: <Type, Generator>{
        ProcessManager: () => FakeProcessManager.any(),
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );
  });

  group('Upgrades project.pbxproj for old asset usage', () {
    const String flutterAssetPbxProjLines =
        '/* flutter_assets */\n'
        '/* App.framework\n'
        'another line';

    const String appFlxPbxProjLines =
        '/* app.flx\n'
        '/* App.framework\n'
        'another line';

    const String cleanPbxProjLines =
        '/* App.framework\n'
        'another line';

    testWithoutContext('upgradePbxProjWithFlutterAssets', () async {
      final FakeIosProject project = FakeIosProject(fileSystem: MemoryFileSystem.test());
      final File pbxprojFile =
          project.xcodeProjectInfoFile
            ..createSync(recursive: true)
            ..writeAsStringSync(flutterAssetPbxProjLines);

      bool result = upgradePbxProjWithFlutterAssets(project, logger);
      expect(result, true);
      expect(logger.statusText, contains('Removing obsolete reference to flutter_assets'));
      logger.clear();

      pbxprojFile.writeAsStringSync(appFlxPbxProjLines);
      result = upgradePbxProjWithFlutterAssets(project, logger);
      expect(result, true);
      expect(logger.statusText, contains('Removing obsolete reference to app.flx'));
      logger.clear();

      pbxprojFile.writeAsStringSync(cleanPbxProjLines);
      result = upgradePbxProjWithFlutterAssets(project, logger);
      expect(result, true);
      expect(logger.statusText, isEmpty);
    });
  });

  group('remove Finder extended attributes', () {
    late Directory projectDirectory;
    setUp(() {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      projectDirectory = fs.directory('flutter_project');
    });

    testWithoutContext('removes xattr', () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>['xattr', '-r', '-d', 'com.apple.FinderInfo', projectDirectory.path],
        ),
      ]);

      await removeFinderExtendedAttributes(
        projectDirectory,
        ProcessUtils(processManager: processManager, logger: logger),
        logger,
      );
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('ignores errors', () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>['xattr', '-r', '-d', 'com.apple.FinderInfo', projectDirectory.path],
          exitCode: 1,
        ),
      ]);

      await removeFinderExtendedAttributes(
        projectDirectory,
        ProcessUtils(processManager: processManager, logger: logger),
        logger,
      );
      expect(logger.traceText, contains('Failed to remove xattr com.apple.FinderInfo'));
      expect(processManager, hasNoRemainingExpectations);
    });
  });
}

void addToPackageConfig(FlutterProject flutterProject, String name, Directory packageDir) {
  final File packageConfigFile = flutterProject.directory
      .childDirectory('.dart_tool')
      .childFile('package_config.json');

  final Map<String, Object?> packageConfig =
      jsonDecode(packageConfigFile.readAsStringSync()) as Map<String, Object?>;

  (packageConfig['packages']! as List<Object?>).add(<String, Object?>{
    'name': name,
    'rootUri': packageDir.uri.toString(),
    'packageUri': 'lib/',
  });

  packageConfigFile.writeAsStringSync(jsonEncode(packageConfig));
}

void createFakePlugins(
  FlutterProject flutterProject,
  FileSystem fileSystem,
  List<String> pluginNames,
) {
  const String pluginYamlTemplate = '''
  flutter:
    plugin:
      platforms:
        ios:
          pluginClass: PLUGIN_CLASS
        macos:
          pluginClass: PLUGIN_CLASS
  ''';

  final Directory fakePubCache = fileSystem.systemTempDirectory.childDirectory('cache');
  flutterProject.directory.childDirectory('.dart_tool').childFile('package_config.json')
    ..createSync(recursive: true)
    ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');
  for (final String name in pluginNames) {
    final Directory pluginDirectory = fakePubCache.childDirectory(name);
    addToPackageConfig(flutterProject, name, pluginDirectory);
    pluginDirectory.childFile('pubspec.yaml')
      ..createSync(recursive: true)
      ..writeAsStringSync(pluginYamlTemplate.replaceAll('PLUGIN_CLASS', name));
  }
}

class FakeIosProject extends Fake implements IosProject {
  FakeIosProject({required MemoryFileSystem fileSystem, this.usesSwiftPackageManager = false})
    : hostAppRoot = fileSystem.directory('app_name').childDirectory('ios');

  @override
  Directory hostAppRoot;

  @override
  File get xcodeProjectInfoFile => xcodeProject.childFile('project.pbxproj');

  @override
  Future<String> productName(BuildInfo? buildInfo) async => 'UnitTestRunner';

  @override
  Directory get xcodeProject => hostAppRoot.childDirectory('Runner.xcodeproj');

  @override
  File get podfile => hostAppRoot.childFile('Podfile');

  @override
  final bool usesSwiftPackageManager;
}

class FakeFlutterProject extends Fake implements FlutterProject {
  FakeFlutterProject({
    required this.fileSystem,
    this.usesSwiftPackageManager = false,
    this.isModule = false,
  });

  final MemoryFileSystem fileSystem;
  final bool usesSwiftPackageManager;

  @override
  late final Directory directory = fileSystem.directory('app_name');

  @override
  late FlutterManifest manifest;

  @override
  File get flutterPluginsFile => directory.childFile('.flutter-plugins');

  @override
  File get flutterPluginsDependenciesFile => directory.childFile('.flutter-plugins-dependencies');

  @override
  late final IosProject ios = FakeIosProject(
    fileSystem: fileSystem,
    usesSwiftPackageManager: usesSwiftPackageManager,
  );

  @override
  final bool isModule;
}

class FakeFlutterManifest extends Fake implements FlutterManifest {
  @override
  Set<String> get dependencies => <String>{};
}
