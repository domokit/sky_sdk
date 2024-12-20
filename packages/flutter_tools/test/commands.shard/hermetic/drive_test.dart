// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:fake_async/fake_async.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/base/async_guard.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/signals.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/drive.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/drive/drive_service.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:package_config/package_config.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  late FileSystem fileSystem;
  late BufferLogger logger;
  late Platform platform;
  late FakeDeviceManager fakeDeviceManager;
  late FakeSignals signals;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    logger = BufferLogger.test();
    platform = FakePlatform();
    fakeDeviceManager = FakeDeviceManager();
    signals = FakeSignals();
  });

  setUpAll(() {
    Cache.disableLocking();
  });

  tearDownAll(() {
    Cache.enableLocking();
  });

  testUsingContext(
    'warns if screenshot is not supported but continues test',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
      );
      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.directory('drive_screenshots').createSync();

      final Device screenshotDevice = ThrowingScreenshotDevice()..supportsScreenshot = false;
      fakeDeviceManager.attachedDevices = <Device>[screenshotDevice];

      await expectLater(
        () => createTestCommandRunner(command).run(<String>[
          'drive',
          '--no-pub',
          '-d',
          screenshotDevice.id,
          '--screenshot',
          'drive_screenshots',
        ]),
        throwsToolExit(message: 'cannot start app'),
      );

      expect(logger.errorText, contains('Screenshot not supported for FakeDevice'));
      expect(logger.statusText, isEmpty);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: () => FakePub(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'does not register screenshot signal handler if --screenshot not provided',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
        flutterDriverFactory: FailingFakeFlutterDriverFactory(),
      );
      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.directory('drive_screenshots').createSync();

      final Device screenshotDevice = ScreenshotDevice();
      fakeDeviceManager.attachedDevices = <Device>[screenshotDevice];

      await expectLater(
        () => createTestCommandRunner(command).run(<String>[
          'drive',
          '--no-pub',
          '-d',
          screenshotDevice.id,
          '--use-existing-app',
          'http://localhost:8181',
          '--keep-app-running',
        ]),
        throwsToolExit(),
      );
      expect(logger.statusText, isNot(contains('Screenshot written to ')));
      expect(signals.addedHandlers, isEmpty);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: () => FakePub(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'takes screenshot and rethrows on drive exception',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
      );
      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.directory('drive_screenshots').createSync();

      final Device screenshotDevice = ThrowingScreenshotDevice();
      fakeDeviceManager.attachedDevices = <Device>[screenshotDevice];

      await expectLater(
        () => createTestCommandRunner(command).run(<String>[
          'drive',
          '--no-pub',
          '-d',
          screenshotDevice.id,
          '--screenshot',
          'drive_screenshots',
        ]),
        throwsToolExit(message: 'cannot start app'),
      );

      expect(logger.statusText, contains('Screenshot written to drive_screenshots/drive_01.png'));
      expect(logger.statusText, isNot(contains('drive_02.png')));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: () => FakePub(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'takes screenshot on drive test failure',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
        flutterDriverFactory: FailingFakeFlutterDriverFactory(),
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.directory('drive_screenshots').createSync();

      final Device screenshotDevice = ScreenshotDevice();
      fakeDeviceManager.attachedDevices = <Device>[screenshotDevice];

      await expectLater(
        () => createTestCommandRunner(command).run(<String>[
          'drive',
          '--no-pub',
          '-d',
          screenshotDevice.id,
          '--use-existing-app',
          'http://localhost:8181',
          '--keep-app-running',
          '--screenshot',
          'drive_screenshots',
        ]),
        throwsToolExit(),
      );

      // Takes the screenshot before the application would be killed (if --keep-app-running not passed).
      expect(
        logger.statusText,
        contains(
          'Screenshot written to drive_screenshots/drive_01.png\n'
          'Leaving the application running.',
        ),
      );
      expect(logger.statusText, isNot(contains('drive_02.png')));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: () => FakePub(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'drive --screenshot errors but does not fail if screenshot fails',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.file('drive_screenshots').createSync();

      final Device screenshotDevice = ThrowingScreenshotDevice();
      fakeDeviceManager.attachedDevices = <Device>[screenshotDevice];

      await expectLater(
        () => createTestCommandRunner(command).run(<String>[
          'drive',
          '--no-pub',
          '-d',
          screenshotDevice.id,
          '--screenshot',
          'drive_screenshots',
        ]),
        throwsToolExit(message: 'cannot start app'),
      );

      expect(logger.statusText, isEmpty);
      expect(
        logger.errorText,
        contains('Error taking screenshot: FileSystemException: Not a directory'),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: () => FakePub(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'drive --timeout takes screenshot and tool exits after timeout',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: Signals.test(),
        flutterDriverFactory: NeverEndingFlutterDriverFactory(() {}),
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.directory('drive_screenshots').createSync();

      final ScreenshotDevice screenshotDevice = ScreenshotDevice();
      fakeDeviceManager.attachedDevices = <Device>[screenshotDevice];

      expect(screenshotDevice.screenshots, isEmpty);
      bool caughtToolExit = false;
      FakeAsync().run<void>((FakeAsync time) {
        // Because the tool exit will be thrown asynchronously by a [Timer],
        // use [asyncGuard] to catch it
        asyncGuard<void>(
          () => createTestCommandRunner(command).run(<String>[
            'drive',
            '--no-pub',
            '-d',
            screenshotDevice.id,
            '--use-existing-app',
            'http://localhost:8181',
            '--screenshot',
            'drive_screenshots',
            '--timeout',
            '300', // 5 minutes
          ]),
          onError: (Object error) {
            expect(error, isA<ToolExit>());
            expect((error as ToolExit).message, contains('Timed out after 300 seconds'));
            caughtToolExit = true;
          },
        );
        time.elapse(const Duration(seconds: 299));
        expect(screenshotDevice.screenshots, isEmpty);
        time.elapse(const Duration(seconds: 2));
        expect(
          screenshotDevice.screenshots,
          contains(
            isA<File>().having((File file) => file.path, 'path', 'drive_screenshots/drive_01.png'),
          ),
        );
      });
      expect(caughtToolExit, isTrue);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: () => FakePub(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'drive --screenshot takes screenshot if sent a registered signal',
    () async {
      final FakeProcessSignal signal = FakeProcessSignal();
      final ProcessSignal signalUnderTest = ProcessSignal(signal);
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: Signals.test(),
        flutterDriverFactory: NeverEndingFlutterDriverFactory(() {
          signal.controller.add(signal);
        }),
        signalsToHandle: <ProcessSignal>{signalUnderTest},
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.directory('drive_screenshots').createSync();

      final ScreenshotDevice screenshotDevice = ScreenshotDevice();
      fakeDeviceManager.attachedDevices = <Device>[screenshotDevice];

      expect(screenshotDevice.screenshots, isEmpty);

      // This command will never complete. In reality, a real signal would have
      // shut down the Dart process.
      unawaited(
        createTestCommandRunner(command).run(<String>[
          'drive',
          '--no-pub',
          '-d',
          screenshotDevice.id,
          '--use-existing-app',
          'http://localhost:8181',
          '--screenshot',
          'drive_screenshots',
        ]),
      );

      await screenshotDevice.firstScreenshot;
      expect(
        screenshotDevice.screenshots,
        contains(
          isA<File>().having((File file) => file.path, 'path', 'drive_screenshots/drive_01.png'),
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: () => FakePub(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'shouldRunPub is true unless user specifies --no-pub',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();

      try {
        await createTestCommandRunner(command).run(const <String>['drive', '--no-pub']);
      } on Exception {
        // Expected to throw
      }

      expect(command.shouldRunPub, false);

      try {
        await createTestCommandRunner(command).run(const <String>['drive']);
      } on Exception {
        // Expected to throw
      }

      expect(command.shouldRunPub, true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      Pub: () => FakePub(),
    },
  );

  testUsingContext(
    'flags propagate to debugging options',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();

      await expectLater(
        () => createTestCommandRunner(command).run(<String>[
          'drive',
          '--start-paused',
          '--disable-service-auth-codes',
          '--trace-skia',
          '--trace-systrace',
          '--trace-to-file=path/to/trace.binpb',
          '--verbose-system-logs',
          '--null-assertions',
          '--native-null-assertions',
          '--enable-impeller',
          '--trace-systrace',
          '--enable-software-rendering',
          '--skia-deterministic-rendering',
          '--enable-embedder-api',
          '--ci',
          '--debug-logs-dir=path/to/logs',
        ]),
        throwsToolExit(),
      );

      final DebuggingOptions options = await command.createDebuggingOptions(false);

      expect(options.startPaused, true);
      expect(options.disableServiceAuthCodes, true);
      expect(options.traceSkia, true);
      expect(options.traceSystrace, true);
      expect(options.traceToFile, 'path/to/trace.binpb');
      expect(options.verboseSystemLogs, true);
      expect(options.nullAssertions, true);
      expect(options.nativeNullAssertions, true);
      expect(options.enableImpeller, ImpellerStatus.enabled);
      expect(options.traceSystrace, true);
      expect(options.enableSoftwareRendering, true);
      expect(options.skiaDeterministicRendering, true);
      expect(options.usingCISystem, true);
      expect(options.debugLogsDirectoryPath, 'path/to/logs');
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'Port publication not disabled for wireless device',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();

      final Device wirelessDevice =
          FakeIosDevice()..connectionInterface = DeviceConnectionInterface.wireless;
      fakeDeviceManager.wirelessDevices = <Device>[wirelessDevice];

      await expectLater(
        () => createTestCommandRunner(command).run(<String>['drive']),
        throwsToolExit(),
      );

      final DebuggingOptions options = await command.createDebuggingOptions(false);
      expect(options.disablePortPublication, false);
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'Port publication is disabled for wired device',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();

      await expectLater(
        () => createTestCommandRunner(command).run(<String>['drive']),
        throwsToolExit(),
      );

      final Device usbDevice =
          FakeIosDevice()..connectionInterface = DeviceConnectionInterface.attached;
      fakeDeviceManager.attachedDevices = <Device>[usbDevice];

      final DebuggingOptions options = await command.createDebuggingOptions(false);
      expect(options.disablePortPublication, true);
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => fakeDeviceManager,
    },
  );

  testUsingContext(
    'Port publication does not default to enabled for wireless device if flag manually added',
    () async {
      final DriveCommand command = DriveCommand(
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        signals: signals,
      );

      fileSystem.file('lib/main.dart').createSync(recursive: true);
      fileSystem.file('test_driver/main_test.dart').createSync(recursive: true);
      fileSystem.file('pubspec.yaml').createSync();

      final Device wirelessDevice =
          FakeIosDevice()..connectionInterface = DeviceConnectionInterface.wireless;
      fakeDeviceManager.wirelessDevices = <Device>[wirelessDevice];

      await expectLater(
        () => createTestCommandRunner(command).run(<String>['drive', '--no-publish-port']),
        throwsToolExit(),
      );

      final DebuggingOptions options = await command.createDebuggingOptions(false);
      expect(options.disablePortPublication, true);
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
      DeviceManager: () => fakeDeviceManager,
    },
  );
}

class ThrowingScreenshotDevice extends ScreenshotDevice {
  @override
  Future<LaunchResult> startApp(
    ApplicationPackage? package, {
    String? mainPath,
    String? route,
    DebuggingOptions? debuggingOptions,
    Map<String, dynamic>? platformArgs,
    bool prebuiltApplication = false,
    bool usesTerminalUi = true,
    bool ipv6 = false,
    String? userIdentifier,
  }) async {
    throwToolExit('cannot start app');
  }
}

class ScreenshotDevice extends Fake implements Device {
  final List<File> screenshots = <File>[];

  final Completer<void> _firstScreenshotCompleter = Completer<void>();

  /// A Future that completes when [takeScreenshot] is called the first time.
  Future<void> get firstScreenshot => _firstScreenshotCompleter.future;

  @override
  final String name = 'FakeDevice';

  @override
  final Category category = Category.mobile;

  @override
  final String id = 'fake_device';

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.android;

  @override
  bool supportsScreenshot = true;

  @override
  bool get isConnected => true;

  @override
  Future<LaunchResult> startApp(
    ApplicationPackage? package, {
    String? mainPath,
    String? route,
    DebuggingOptions? debuggingOptions,
    Map<String, dynamic>? platformArgs,
    bool prebuiltApplication = false,
    bool usesTerminalUi = true,
    bool ipv6 = false,
    String? userIdentifier,
  }) async => LaunchResult.succeeded();

  @override
  Future<void> takeScreenshot(File outputFile) async {
    if (!_firstScreenshotCompleter.isCompleted) {
      _firstScreenshotCompleter.complete();
    }
    screenshots.add(outputFile);
  }
}

class FakePub extends Fake implements Pub {
  @override
  Future<void> get({
    PubContext? context,
    required FlutterProject project,
    bool upgrade = false,
    bool offline = false,
    bool generateSyntheticPackage = false,
    String? flutterRootOverride,
    bool checkUpToDate = false,
    bool shouldSkipThirdPartyGenerator = true,
    PubOutputMode outputMode = PubOutputMode.all,
  }) async {}
}

/// A [FlutterDriverFactory] that creates a [NeverEndingDriverService].
class NeverEndingFlutterDriverFactory extends Fake implements FlutterDriverFactory {
  NeverEndingFlutterDriverFactory(this.callback);

  final void Function() callback;

  @override
  DriverService createDriverService(bool web) => NeverEndingDriverService(callback);
}

/// A [DriverService] that will return a Future from [startTest] that will never complete.
///
/// This is to simulate when the test will take a long time, but a signal is
/// expected to interrupt the process.
class NeverEndingDriverService extends Fake implements DriverService {
  NeverEndingDriverService(this.callback);

  final void Function() callback;
  @override
  Future<void> reuseApplication(
    Uri vmServiceUri,
    Device device,
    DebuggingOptions debuggingOptions,
  ) async {}

  @override
  Future<int> startTest(
    String testFile,
    List<String> arguments,
    Map<String, String> environment,
    PackageConfig packageConfig, {
    bool? headless,
    String? chromeBinary,
    String? browserName,
    bool? androidEmulator,
    int? driverPort,
    List<String>? webBrowserFlags,
    List<String>? browserDimension,
    String? profileMemory,
  }) async {
    callback();
    // return a Future that will never complete.
    return Completer<int>().future;
  }
}

class FailingFakeFlutterDriverFactory extends Fake implements FlutterDriverFactory {
  @override
  DriverService createDriverService(bool web) => FailingFakeDriverService();
}

class FailingFakeDriverService extends Fake implements DriverService {
  @override
  Future<void> reuseApplication(
    Uri vmServiceUri,
    Device device,
    DebuggingOptions debuggingOptions,
  ) async {}

  @override
  Future<int> startTest(
    String testFile,
    List<String> arguments,
    Map<String, String> environment,
    PackageConfig packageConfig, {
    bool? headless,
    String? chromeBinary,
    String? browserName,
    bool? androidEmulator,
    int? driverPort,
    List<String>? webBrowserFlags,
    List<String>? browserDimension,
    String? profileMemory,
  }) async => 1;
}

class FakeProcessSignal extends Fake implements io.ProcessSignal {
  final StreamController<io.ProcessSignal> controller = StreamController<io.ProcessSignal>();

  @override
  Stream<io.ProcessSignal> watch() => controller.stream;
}

class FakeIosDevice extends Fake implements IOSDevice {
  @override
  DeviceConnectionInterface connectionInterface = DeviceConnectionInterface.attached;

  @override
  bool get isWirelesslyConnected => connectionInterface == DeviceConnectionInterface.wireless;

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.ios;
}

class FakeSignals extends Fake implements Signals {
  List<SignalHandler> addedHandlers = <SignalHandler>[];

  @override
  Object addHandler(ProcessSignal signal, SignalHandler handler) {
    addedHandlers.add(handler);
    return const Object();
  }

  @override
  Future<bool> removeHandler(ProcessSignal signal, Object token) async => true;
}
