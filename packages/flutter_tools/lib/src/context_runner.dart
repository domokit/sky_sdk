// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import 'android/android_builder.dart';
import 'android/android_sdk.dart';
import 'android/android_studio.dart';
import 'android/android_workflow.dart';
import 'android/gradle.dart';
import 'android/gradle_utils.dart';
import 'android/java.dart';
import 'application_package.dart';
import 'artifacts.dart';
import 'asset.dart';
import 'base/config.dart';
import 'base/context.dart';
import 'base/error_handling_io.dart';
import 'base/io.dart';
import 'base/logger.dart';
import 'base/os.dart';
import 'base/process.dart';
import 'base/terminal.dart';
import 'base/time.dart';
import 'base/user_messages.dart';
import 'build_system/build_system.dart';
import 'cache.dart';
import 'custom_devices/custom_devices_config.dart';
import 'dart/pub.dart';
import 'devfs.dart';
import 'device.dart';
import 'devtools_launcher.dart';
import 'doctor.dart';
import 'emulator.dart';
import 'features.dart';
import 'flutter_application_package.dart';
import 'flutter_cache.dart';
import 'flutter_device_manager.dart';
import 'flutter_features.dart';
import 'globals.dart' as globals;
import 'ios/ios_workflow.dart';
import 'ios/iproxy.dart';
import 'ios/simulators.dart';
import 'ios/xcodeproj.dart';
import 'macos/cocoapods.dart';
import 'macos/cocoapods_validator.dart';
import 'macos/macos_workflow.dart';
import 'macos/xcdevice.dart';
import 'macos/xcode.dart';
import 'mdns_discovery.dart';
import 'persistent_tool_state.dart';
import 'reporting/crash_reporting.dart';
import 'reporting/first_run.dart';
import 'reporting/reporting.dart';
import 'reporting/unified_analytics.dart';
import 'resident_runner.dart';
import 'run_hot.dart';
import 'runner/local_engine.dart';
import 'version.dart';
import 'web/workflow.dart';
import 'windows/visual_studio.dart';
import 'windows/visual_studio_validator.dart';
import 'windows/windows_workflow.dart';

Future<T> runInContext<T>(FutureOr<T> Function() runner, {Map<Type, Generator>? overrides}) async {
  // Wrap runner with any asynchronous initialization that should run with the
  // overrides and callbacks.
  late bool runningOnBot;
  FutureOr<T> runnerWrapper() async {
    runningOnBot = await globals.isRunningOnBot;
    return runner();
  }

  // TODO(ianh): We should split this into two, one for tests (which should be
  // in test/), and one for production (which should be in executable.dart).
  return context.run<T>(
    name: 'global fallbacks',
    body: runnerWrapper,
    overrides: overrides,
    fallbacks: <Type, Generator>{
      Analytics:
          () => getAnalytics(
            runningOnBot: runningOnBot,
            flutterVersion: globals.flutterVersion,
            environment: globals.platform.environment,
            clientIde: globals.platform.environment['FLUTTER_HOST'],
            config: globals.config,
          ),
      AndroidBuilder:
          () => AndroidGradleBuilder(
            java: globals.java,
            logger: globals.logger,
            processManager: globals.processManager,
            fileSystem: globals.fs,
            artifacts: globals.artifacts!,
            analytics: globals.analytics,
            gradleUtils: globals.gradleUtils!,
            platform: globals.platform,
            androidStudio: globals.androidStudio,
          ),
      AndroidLicenseValidator:
          () => AndroidLicenseValidator(
            platform: globals.platform,
            userMessages: globals.userMessages,
            processManager: globals.processManager,
            java: globals.java,
            androidSdk: globals.androidSdk,
            logger: globals.logger,
            stdio: globals.stdio,
          ),
      AndroidSdk: AndroidSdk.locateAndroidSdk,
      AndroidStudio: AndroidStudio.latestValid,
      AndroidValidator:
          () => AndroidValidator(
            java: globals.java,
            androidSdk: globals.androidSdk,
            logger: globals.logger,
            platform: globals.platform,
            userMessages: globals.userMessages,
          ),
      AndroidWorkflow:
          () => AndroidWorkflow(androidSdk: globals.androidSdk, featureFlags: featureFlags),
      ApplicationPackageFactory:
          () => FlutterApplicationPackageFactory(
            userMessages: globals.userMessages,
            processManager: globals.processManager,
            logger: globals.logger,
            fileSystem: globals.fs,
            androidSdk: globals.androidSdk,
          ),
      Artifacts:
          () => CachedArtifacts(
            fileSystem: globals.fs,
            cache: globals.cache,
            platform: globals.platform,
            operatingSystemUtils: globals.os,
          ),
      AssetBundleFactory: () {
        return AssetBundleFactory.defaultInstance(
          logger: globals.logger,
          fileSystem: globals.fs,
          platform: globals.platform,
        );
      },
      BuildSystem:
          () => FlutterBuildSystem(
            fileSystem: globals.fs,
            logger: globals.logger,
            platform: globals.platform,
          ),
      Cache:
          () => FlutterCache(
            fileSystem: globals.fs,
            logger: globals.logger,
            platform: globals.platform,
            osUtils: globals.os,
            projectFactory: globals.projectFactory,
          ),
      CocoaPods:
          () => CocoaPods(
            fileSystem: globals.fs,
            processManager: globals.processManager,
            logger: globals.logger,
            platform: globals.platform,
            xcodeProjectInterpreter: globals.xcodeProjectInterpreter!,
            usage: globals.flutterUsage,
            analytics: globals.analytics,
          ),
      CocoaPodsValidator: () => CocoaPodsValidator(globals.cocoaPods!, globals.userMessages),
      Config:
          () => Config(
            Config.kFlutterSettings,
            fileSystem: globals.fs,
            logger: globals.logger,
            platform: globals.platform,
          ),
      CustomDevicesConfig:
          () => CustomDevicesConfig(
            fileSystem: globals.fs,
            logger: globals.logger,
            platform: globals.platform,
          ),
      CrashReporter:
          () => CrashReporter(
            fileSystem: globals.fs,
            logger: globals.logger,
            flutterProjectFactory: globals.projectFactory,
          ),
      DevFSConfig: () => DevFSConfig(),
      DeviceManager:
          () => FlutterDeviceManager(
            logger: globals.logger,
            processManager: globals.processManager,
            platform: globals.platform,
            androidSdk: globals.androidSdk,
            iosSimulatorUtils: globals.iosSimulatorUtils!,
            featureFlags: featureFlags,
            fileSystem: globals.fs,
            iosWorkflow: globals.iosWorkflow!,
            artifacts: globals.artifacts!,
            flutterVersion: globals.flutterVersion,
            androidWorkflow: androidWorkflow!,
            xcDevice: globals.xcdevice!,
            userMessages: globals.userMessages,
            windowsWorkflow: windowsWorkflow!,
            macOSWorkflow: MacOSWorkflow(platform: globals.platform, featureFlags: featureFlags),
            operatingSystemUtils: globals.os,
            customDevicesConfig: globals.customDevicesConfig,
            nativeAssetsBuilder: globals.nativeAssetsBuilder,
          ),
      DevtoolsLauncher:
          () => DevtoolsServerLauncher(
            processManager: globals.processManager,
            artifacts: globals.artifacts!,
            logger: globals.logger,
            botDetector: globals.botDetector,
          ),
      Doctor: () => Doctor(logger: globals.logger, clock: globals.systemClock),
      DoctorValidatorsProvider: () => DoctorValidatorsProvider.defaultInstance,
      EmulatorManager:
          () => EmulatorManager(
            java: globals.java,
            androidSdk: globals.androidSdk,
            processManager: globals.processManager,
            logger: globals.logger,
            fileSystem: globals.fs,
            androidWorkflow: androidWorkflow!,
          ),
      FeatureFlags:
          () => FlutterFeatureFlags(
            flutterVersion: globals.flutterVersion,
            config: globals.config,
            platform: globals.platform,
          ),
      FlutterVersion: () => FlutterVersion(fs: globals.fs, flutterRoot: Cache.flutterRoot!),
      GradleUtils:
          () => GradleUtils(
            operatingSystemUtils: globals.os,
            logger: globals.logger,
            platform: globals.platform,
            cache: globals.cache,
          ),
      HotRunnerConfig: () => HotRunnerConfig(),
      IOSSimulatorUtils:
          () => IOSSimulatorUtils(
            logger: globals.logger,
            processManager: globals.processManager,
            xcode: globals.xcode!,
          ),
      IOSWorkflow:
          () => IOSWorkflow(
            featureFlags: featureFlags,
            xcode: globals.xcode!,
            platform: globals.platform,
          ),
      Java:
          () => Java.find(
            config: globals.config,
            androidStudio: globals.androidStudio,
            logger: globals.logger,
            fileSystem: globals.fs,
            platform: globals.platform,
            processManager: globals.processManager,
          ),
      LocalEngineLocator:
          () => LocalEngineLocator(
            userMessages: globals.userMessages,
            logger: globals.logger,
            platform: globals.platform,
            fileSystem: globals.fs,
            flutterRoot: Cache.flutterRoot!,
          ),
      Logger:
          () =>
              globals.platform.isWindows
                  ? WindowsStdoutLogger(
                    terminal: globals.terminal,
                    stdio: globals.stdio,
                    outputPreferences: globals.outputPreferences,
                  )
                  : StdoutLogger(
                    terminal: globals.terminal,
                    stdio: globals.stdio,
                    outputPreferences: globals.outputPreferences,
                  ),
      MacOSWorkflow: () => MacOSWorkflow(featureFlags: featureFlags, platform: globals.platform),
      MDnsVmServiceDiscovery:
          () => MDnsVmServiceDiscovery(
            logger: globals.logger,
            flutterUsage: globals.flutterUsage,
            analytics: globals.analytics,
          ),
      OperatingSystemUtils:
          () => OperatingSystemUtils(
            fileSystem: globals.fs,
            logger: globals.logger,
            platform: globals.platform,
            processManager: globals.processManager,
          ),
      OutputPreferences:
          () => OutputPreferences(
            wrapText: globals.stdio.hasTerminal,
            showColor: globals.platform.stdoutSupportsAnsi,
            stdio: globals.stdio,
          ),
      PersistentToolState:
          () => PersistentToolState(
            fileSystem: globals.fs,
            logger: globals.logger,
            platform: globals.platform,
          ),
      ProcessInfo: () => ProcessInfo(globals.fs),
      ProcessManager:
          () => ErrorHandlingProcessManager(
            delegate: const LocalProcessManager(),
            platform: globals.platform,
          ),
      ProcessUtils:
          () => ProcessUtils(processManager: globals.processManager, logger: globals.logger),
      Pub:
          () => Pub(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            usage: globals.flutterUsage,
          ),
      Stdio: () => Stdio(),
      SystemClock: () => const SystemClock(),
      Usage:
          () => Usage(
            runningOnBot: runningOnBot,
            firstRunMessenger: FirstRunMessenger(persistentToolState: globals.persistentToolState!),
          ),
      UserMessages: () => UserMessages(),
      VisualStudioValidator:
          () => VisualStudioValidator(
            userMessages: globals.userMessages,
            visualStudio: VisualStudio(
              fileSystem: globals.fs,
              platform: globals.platform,
              logger: globals.logger,
              processManager: globals.processManager,
              osUtils: globals.os,
            ),
          ),
      WebWorkflow: () => WebWorkflow(featureFlags: featureFlags, platform: globals.platform),
      WindowsWorkflow:
          () => WindowsWorkflow(featureFlags: featureFlags, platform: globals.platform),
      Xcode:
          () => Xcode(
            logger: globals.logger,
            processManager: globals.processManager,
            platform: globals.platform,
            fileSystem: globals.fs,
            xcodeProjectInterpreter: globals.xcodeProjectInterpreter!,
            userMessages: globals.userMessages,
          ),
      XCDevice:
          () => XCDevice(
            processManager: globals.processManager,
            logger: globals.logger,
            artifacts: globals.artifacts!,
            cache: globals.cache,
            platform: globals.platform,
            xcode: globals.xcode!,
            iproxy: IProxy(
              iproxyPath: globals.artifacts!.getHostArtifact(HostArtifact.iproxy).path,
              logger: globals.logger,
              processManager: globals.processManager,
              dyLdLibEntry: globals.cache.dyLdLibEntry,
            ),
            fileSystem: globals.fs,
            analytics: globals.analytics,
            shutdownHooks: globals.shutdownHooks,
          ),
      XcodeProjectInterpreter:
          () => XcodeProjectInterpreter(
            logger: globals.logger,
            processManager: globals.processManager,
            platform: globals.platform,
            fileSystem: globals.fs,
            usage: globals.flutterUsage,
            analytics: globals.analytics,
          ),
    },
  );
}
