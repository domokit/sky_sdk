// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/bundle.dart';
import 'package:flutter_tools/src/bundle_builder.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/custom_devices/custom_device.dart';
import 'package:flutter_tools/src/custom_devices/custom_device_config.dart';
import 'package:flutter_tools/src/custom_devices/custom_devices_config.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/linux/application_package.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:meta/meta.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';

void _writeCustomDevicesConfigFile(Directory dir, List<CustomDeviceConfig> configs) {
  dir.createSync();

  final File file = dir.childFile('.flutter_custom_devices.json');
  file.writeAsStringSync(
    jsonEncode(<String, dynamic>{
      'custom-devices': configs.map<dynamic>((CustomDeviceConfig c) => c.toJson()).toList(),
    }),
  );
}

FlutterProject _setUpFlutterProject(Directory directory) {
  final FlutterProjectFactory flutterProjectFactory = FlutterProjectFactory(
    fileSystem: directory.fileSystem,
    logger: BufferLogger.test(),
  );
  return flutterProjectFactory.fromDirectory(directory);
}

void main() {
  testWithoutContext(
    'replacing string interpolation occurrences in custom device commands',
    () async {
      expect(
        interpolateCommand(
          <String>['scp', r'${localPath}', r'/tmp/${appName}', 'pi@raspberrypi'],
          <String, String>{'localPath': 'build/flutter_assets', 'appName': 'hello_world'},
        ),
        <String>['scp', 'build/flutter_assets', '/tmp/hello_world', 'pi@raspberrypi'],
      );

      expect(
        interpolateCommand(
          <String>[r'${test1}', r' ${test2}', r'${test3}'],
          <String, String>{'test1': '_test1', 'test2': '_test2'},
        ),
        <String>['_test1', ' _test2', r''],
      );

      expect(
        interpolateCommand(
          <String>[r'${test1}', r' ${test2}', r'${test3}'],
          <String, String>{'test1': '_test1', 'test2': '_test2'},
          additionalReplacementValues: <String, String>{'test2': '_nottest2', 'test3': '_test3'},
        ),
        <String>['_test1', ' _test2', r'_test3'],
      );
    },
  );

  final CustomDeviceConfig testConfig = CustomDeviceConfig(
    id: 'testid',
    label: 'testlabel',
    sdkNameAndVersion: 'testsdknameandversion',
    enabled: true,
    pingCommand: const <String>['testping'],
    pingSuccessRegex: RegExp('testpingsuccess'),
    postBuildCommand: const <String>['testpostbuild'],
    installCommand: const <String>['testinstall'],
    uninstallCommand: const <String>['testuninstall'],
    runDebugCommand: const <String>['testrundebug'],
    forwardPortCommand: const <String>['testforwardport'],
    forwardPortSuccessRegex: RegExp('testforwardportsuccess'),
    screenshotCommand: const <String>['testscreenshot'],
  );

  const String testConfigPingSuccessOutput = 'testpingsuccess\n';
  const String testConfigForwardPortSuccessOutput = 'testforwardportsuccess\n';
  final CustomDeviceConfig disabledTestConfig = testConfig.copyWith(enabled: false);
  final CustomDeviceConfig testConfigNonForwarding = testConfig.copyWith(
    explicitForwardPortCommand: true,
    explicitForwardPortSuccessRegex: true,
  );

  testUsingContext(
    'CustomDevice defaults',
    () async {
      final CustomDevice device = CustomDevice(
        config: testConfig,
        processManager: FakeProcessManager.any(),
        logger: BufferLogger.test(),
      );

      final PrebuiltLinuxApp linuxApp = PrebuiltLinuxApp(executable: 'foo');

      expect(device.id, 'testid');
      expect(device.name, 'testlabel');
      expect(device.platformType, PlatformType.custom);
      expect(await device.sdkNameAndVersion, 'testsdknameandversion');
      expect(await device.targetPlatform, TargetPlatform.linux_arm64);
      expect(await device.installApp(linuxApp), true);
      expect(await device.uninstallApp(linuxApp), true);
      expect(await device.isLatestBuildInstalled(linuxApp), false);
      expect(await device.isAppInstalled(linuxApp), false);
      expect(await device.stopApp(linuxApp), false);
      expect(await device.stopApp(null), false);
      expect(device.category, Category.mobile);

      expect(device.supportsRuntimeMode(BuildMode.debug), true);
      expect(device.supportsRuntimeMode(BuildMode.profile), false);
      expect(device.supportsRuntimeMode(BuildMode.release), false);
      expect(device.supportsRuntimeMode(BuildMode.jitRelease), false);
    },
    overrides: <Type, dynamic Function()>{
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext(
    'CustomDevice: no devices listed if only disabled devices configured',
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory dir = fs.directory('custom_devices_config_dir');

      _writeCustomDevicesConfigFile(dir, <CustomDeviceConfig>[disabledTestConfig]);

      expect(
        await CustomDevices(
          featureFlags: TestFeatureFlags(areCustomDevicesEnabled: true),
          logger: BufferLogger.test(),
          processManager: FakeProcessManager.any(),
          config: CustomDevicesConfig.test(
            fileSystem: fs,
            directory: dir,
            logger: BufferLogger.test(),
          ),
        ).devices(),
        <Device>[],
      );
    },
  );

  testWithoutContext(
    'CustomDevice: no devices listed if custom devices feature flag disabled',
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory dir = fs.directory('custom_devices_config_dir');

      _writeCustomDevicesConfigFile(dir, <CustomDeviceConfig>[testConfig]);

      expect(
        await CustomDevices(
          featureFlags: TestFeatureFlags(),
          logger: BufferLogger.test(),
          processManager: FakeProcessManager.any(),
          config: CustomDevicesConfig.test(
            fileSystem: fs,
            directory: dir,
            logger: BufferLogger.test(),
          ),
        ).devices(),
        <Device>[],
      );
    },
  );

  testWithoutContext('CustomDevices.devices', () async {
    final MemoryFileSystem fs = MemoryFileSystem.test();
    final Directory dir = fs.directory('custom_devices_config_dir');

    _writeCustomDevicesConfigFile(dir, <CustomDeviceConfig>[testConfig]);

    expect(
      await CustomDevices(
        featureFlags: TestFeatureFlags(areCustomDevicesEnabled: true),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(command: testConfig.pingCommand, stdout: testConfigPingSuccessOutput),
        ]),
        config: CustomDevicesConfig.test(
          fileSystem: fs,
          directory: dir,
          logger: BufferLogger.test(),
        ),
      ).devices(),
      hasLength(1),
    );
  });

  testWithoutContext(
    'CustomDevices.discoverDevices successfully discovers devices and executes ping command',
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory dir = fs.directory('custom_devices_config_dir');

      _writeCustomDevicesConfigFile(dir, <CustomDeviceConfig>[testConfig]);

      bool pingCommandWasExecuted = false;

      final CustomDevices discovery = CustomDevices(
        featureFlags: TestFeatureFlags(areCustomDevicesEnabled: true),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(
            command: testConfig.pingCommand,
            onRun: (_) => pingCommandWasExecuted = true,
            stdout: testConfigPingSuccessOutput,
          ),
        ]),
        config: CustomDevicesConfig.test(
          fileSystem: fs,
          directory: dir,
          logger: BufferLogger.test(),
        ),
      );

      final List<Device> discoveredDevices = await discovery.discoverDevices();

      expect(discoveredDevices, hasLength(1));
      expect(pingCommandWasExecuted, true);
    },
  );

  testWithoutContext(
    "CustomDevices.discoverDevices doesn't report device when ping command fails",
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory dir = fs.directory('custom_devices_config_dir');

      _writeCustomDevicesConfigFile(dir, <CustomDeviceConfig>[testConfig]);

      final CustomDevices discovery = CustomDevices(
        featureFlags: TestFeatureFlags(areCustomDevicesEnabled: true),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(
            command: testConfig.pingCommand,
            stdout: testConfigPingSuccessOutput,
            exitCode: 1,
          ),
        ]),
        config: CustomDevicesConfig.test(
          fileSystem: fs,
          directory: dir,
          logger: BufferLogger.test(),
        ),
      );

      expect(await discovery.discoverDevices(), hasLength(0));
    },
  );

  testWithoutContext(
    "CustomDevices.discoverDevices doesn't report device when ping command output doesn't match ping success regex",
    () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final Directory dir = fs.directory('custom_devices_config_dir');

      _writeCustomDevicesConfigFile(dir, <CustomDeviceConfig>[testConfig]);

      final CustomDevices discovery = CustomDevices(
        featureFlags: TestFeatureFlags(areCustomDevicesEnabled: true),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(command: testConfig.pingCommand),
        ]),
        config: CustomDevicesConfig.test(
          fileSystem: fs,
          directory: dir,
          logger: BufferLogger.test(),
        ),
      );

      expect(await discovery.discoverDevices(), hasLength(0));
    },
  );

  testWithoutContext('CustomDevice.isSupportedForProject is true with editable host app', () async {
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file('pubspec.yaml').createSync();
    fileSystem.directory('.dart_tool').childFile('package_config.json').createSync(recursive: true);

    final FlutterProject flutterProject = _setUpFlutterProject(fileSystem.currentDirectory);

    expect(
      CustomDevice(
        config: testConfig,
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
      ).isSupportedForProject(flutterProject),
      true,
    );
  });

  testUsingContext(
    'CustomDevice.install invokes uninstall and install command',
    () async {
      bool bothCommandsWereExecuted = false;

      final CustomDevice device = CustomDevice(
        config: testConfig,
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(command: testConfig.uninstallCommand),
          FakeCommand(
            command: testConfig.installCommand,
            onRun: (_) => bothCommandsWereExecuted = true,
          ),
        ]),
      );

      expect(await device.installApp(PrebuiltLinuxApp(executable: 'exe')), true);
      expect(bothCommandsWereExecuted, true);
    },
    overrides: <Type, dynamic Function()>{
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext(
    'CustomDevicePortForwarder will run and terminate forwardPort command',
    () async {
      final Completer<void> forwardPortCommandCompleter = Completer<void>();

      final CustomDevicePortForwarder forwarder = CustomDevicePortForwarder(
        deviceName: 'testdevicename',
        forwardPortCommand: testConfig.forwardPortCommand!,
        forwardPortSuccessRegex: testConfig.forwardPortSuccessRegex!,
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          FakeCommand(
            command: testConfig.forwardPortCommand!,
            stdout: testConfigForwardPortSuccessOutput,
            completer: forwardPortCommandCompleter,
          ),
        ]),
      );

      // this should start the command
      expect(await forwarder.forward(12345), 12345);
      expect(forwardPortCommandCompleter.isCompleted, false);

      // this should terminate it
      await forwarder.dispose();

      // the termination should have completed our completer
      expect(forwardPortCommandCompleter.isCompleted, true);
    },
  );

  testWithoutContext(
    'CustomDevice forwards VM Service port correctly when port forwarding is configured',
    () async {
      final Completer<void> runDebugCompleter = Completer<void>();
      final Completer<void> forwardPortCompleter = Completer<void>();

      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: testConfig.runDebugCommand,
          completer: runDebugCompleter,
          stdout: 'The Dart VM service is listening on http://127.0.0.1:12345/abcd/\n',
        ),
        FakeCommand(
          command: testConfig.forwardPortCommand!,
          completer: forwardPortCompleter,
          stdout: testConfigForwardPortSuccessOutput,
        ),
      ]);

      final CustomDeviceAppSession appSession = CustomDeviceAppSession(
        name: 'testname',
        device: CustomDevice(
          config: testConfig,
          logger: BufferLogger.test(),
          processManager: processManager,
        ),
        appPackage: PrebuiltLinuxApp(executable: 'testexecutable'),
        logger: BufferLogger.test(),
        processManager: processManager,
      );

      final LaunchResult launchResult = await appSession.start(
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
      );

      expect(launchResult.started, true);
      expect(launchResult.vmServiceUri, Uri.parse('http://127.0.0.1:12345/abcd/'));
      expect(runDebugCompleter.isCompleted, false);
      expect(forwardPortCompleter.isCompleted, false);

      expect(await appSession.stop(), true);
      expect(runDebugCompleter.isCompleted, true);
      expect(forwardPortCompleter.isCompleted, true);
    },
  );

  testWithoutContext(
    'CustomDeviceAppSession forwards VM Service port correctly when port forwarding is not configured',
    () async {
      final Completer<void> runDebugCompleter = Completer<void>();

      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: testConfigNonForwarding.runDebugCommand,
          completer: runDebugCompleter,
          stdout: 'The Dart VM service is listening on http://192.168.178.123:12345/abcd/\n',
        ),
      ]);

      final CustomDeviceAppSession appSession = CustomDeviceAppSession(
        name: 'testname',
        device: CustomDevice(
          config: testConfigNonForwarding,
          logger: BufferLogger.test(),
          processManager: processManager,
        ),
        appPackage: PrebuiltLinuxApp(executable: 'testexecutable'),
        logger: BufferLogger.test(),
        processManager: processManager,
      );

      final LaunchResult launchResult = await appSession.start(
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
      );

      expect(launchResult.started, true);
      expect(launchResult.vmServiceUri, Uri.parse('http://192.168.178.123:12345/abcd/'));
      expect(runDebugCompleter.isCompleted, false);

      expect(await appSession.stop(), true);
      expect(runDebugCompleter.isCompleted, true);
    },
  );

  testUsingContext(
    'custom device end-to-end test',
    () async {
      final Completer<void> runDebugCompleter = Completer<void>();
      final Completer<void> forwardPortCompleter = Completer<void>();

      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(command: testConfig.pingCommand, stdout: testConfigPingSuccessOutput),
        FakeCommand(command: testConfig.postBuildCommand!),
        FakeCommand(command: testConfig.uninstallCommand),
        FakeCommand(command: testConfig.installCommand),
        FakeCommand(
          command: testConfig.runDebugCommand,
          completer: runDebugCompleter,
          stdout: 'The Dart VM service is listening on http://127.0.0.1:12345/abcd/\n',
        ),
        FakeCommand(
          command: testConfig.forwardPortCommand!,
          completer: forwardPortCompleter,
          stdout: testConfigForwardPortSuccessOutput,
        ),
      ]);

      // Reuse our filesystem from context instead of mixing two filesystem instances
      // together
      final FileSystem fs = globals.fs;

      // CustomDevice.startApp doesn't care whether we pass a prebuilt app or
      // buildable app as long as we pass prebuiltApplication as false
      final PrebuiltLinuxApp app = PrebuiltLinuxApp(executable: 'testexecutable');

      final Directory configFileDir = fs.directory('custom_devices_config_dir');
      _writeCustomDevicesConfigFile(configFileDir, <CustomDeviceConfig>[testConfig]);

      // finally start actually testing things
      final CustomDevices customDevices = CustomDevices(
        featureFlags: TestFeatureFlags(areCustomDevicesEnabled: true),
        processManager: processManager,
        logger: BufferLogger.test(),
        config: CustomDevicesConfig.test(
          fileSystem: fs,
          directory: configFileDir,
          logger: BufferLogger.test(),
        ),
      );

      final List<Device> devices = await customDevices.discoverDevices();
      expect(devices.length, 1);
      expect(devices.single, isA<CustomDevice>());

      final CustomDevice device = devices.single as CustomDevice;
      expect(device.id, testConfig.id);
      expect(device.name, testConfig.label);
      expect(await device.sdkNameAndVersion, testConfig.sdkNameAndVersion);

      final LaunchResult result = await device.startApp(
        app,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        bundleBuilder: FakeBundleBuilder(),
      );
      expect(result.started, true);
      expect(result.hasVmService, true);
      expect(result.vmServiceUri, Uri.tryParse('http://127.0.0.1:12345/abcd/'));
      expect(runDebugCompleter.isCompleted, false);
      expect(forwardPortCompleter.isCompleted, false);

      expect(await device.stopApp(app), true);
      expect(runDebugCompleter.isCompleted, true);
      expect(forwardPortCompleter.isCompleted, true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext('CustomDevice screenshotting', () async {
    bool screenshotCommandWasExecuted = false;

    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: testConfig.screenshotCommand!,
        onRun: (_) => screenshotCommandWasExecuted = true,
      ),
    ]);

    final MemoryFileSystem fs = MemoryFileSystem.test();
    final File screenshotFile = fs.file('screenshot.png');

    final CustomDevice device = CustomDevice(
      config: testConfig,
      logger: BufferLogger.test(),
      processManager: processManager,
    );

    expect(device.supportsScreenshot, true);

    await device.takeScreenshot(screenshotFile);
    expect(screenshotCommandWasExecuted, true);
    expect(screenshotFile, exists);
  });

  testWithoutContext('CustomDevice without screenshotting support', () async {
    bool screenshotCommandWasExecuted = false;

    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: testConfig.screenshotCommand!,
        onRun: (_) => screenshotCommandWasExecuted = true,
      ),
    ]);

    final MemoryFileSystem fs = MemoryFileSystem.test();
    final File screenshotFile = fs.file('screenshot.png');

    final CustomDevice device = CustomDevice(
      config: testConfig.copyWith(explicitScreenshotCommand: true),
      logger: BufferLogger.test(),
      processManager: processManager,
    );

    expect(device.supportsScreenshot, false);
    expect(
      () => device.takeScreenshot(screenshotFile),
      throwsA(const TypeMatcher<UnsupportedError>()),
    );
    expect(screenshotCommandWasExecuted, false);
    expect(screenshotFile.existsSync(), false);
  });

  testWithoutContext('CustomDevice returns correct target platform', () async {
    final CustomDevice device = CustomDevice(
      config: testConfig.copyWith(platform: TargetPlatform.linux_x64),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.empty(),
    );

    expect(await device.targetPlatform, TargetPlatform.linux_x64);
  });

  testWithoutContext(
    'CustomDeviceLogReader cancels subscriptions before closing logLines stream',
    () async {
      final CustomDeviceLogReader logReader = CustomDeviceLogReader('testname');

      final Iterable<List<int>> lines = Iterable<List<int>>.generate(
        5,
        (int _) => utf8.encode('test'),
      );

      logReader.listenToProcessOutput(
        FakeProcess(
          exitCode: Future<int>.value(0),
          stdout: Stream<List<int>>.fromIterable(lines),
          stderr: Stream<List<int>>.fromIterable(lines),
        ),
      );

      final List<MyFakeStreamSubscription<String>> subscriptions =
          <MyFakeStreamSubscription<String>>[];
      bool logLinesStreamDone = false;
      logReader.logLines.listen(
        (_) {},
        onDone: () {
          expect(subscriptions, everyElement((MyFakeStreamSubscription<String> s) => s.canceled));
          logLinesStreamDone = true;
        },
      );

      logReader.subscriptions.replaceRange(
        0,
        logReader.subscriptions.length,
        logReader.subscriptions.map(
          (StreamSubscription<String> e) => MyFakeStreamSubscription<String>(e),
        ),
      );

      subscriptions.addAll(logReader.subscriptions.cast());

      await logReader.dispose();

      expect(logLinesStreamDone, true);
    },
  );
}

class MyFakeStreamSubscription<T> extends Fake implements StreamSubscription<T> {
  MyFakeStreamSubscription(this.parent);

  StreamSubscription<T> parent;
  bool canceled = false;

  @override
  Future<void> cancel() {
    canceled = true;
    return parent.cancel();
  }
}

class FakeBundleBuilder extends Fake implements BundleBuilder {
  @override
  Future<void> build({
    TargetPlatform? platform,
    BuildInfo? buildInfo,
    FlutterProject? project,
    String? mainPath,
    String manifestPath = defaultManifestPath,
    String? applicationKernelFilePath,
    String? depfilePath,
    String? assetDirPath,
    Uri? nativeAssets,
    bool buildNativeAssets = true,
    @visibleForTesting BuildSystem? buildSystem,
  }) async {}
}
