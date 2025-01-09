// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_device.dart';
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/commands/daemon.dart';
import 'package:flutter_tools/src/daemon.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/proxied_devices/devices.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_devices.dart';
import '../../src/fakes.dart';

void main() {
  Daemon? daemon;
  late NotifyingLogger notifyingLogger;
  late BufferLogger bufferLogger;
  late FakeAndroidDevice fakeDevice;

  late FakeApplicationPackageFactory applicationPackageFactory;
  late MemoryFileSystem memoryFileSystem;
  late FakeProcessManager fakeProcessManager;

  group('ProxiedDevices', () {
    late DaemonConnection serverDaemonConnection;
    late DaemonConnection clientDaemonConnection;
    setUp(() {
      bufferLogger = BufferLogger.test();
      notifyingLogger = NotifyingLogger(verbose: false, parent: bufferLogger);
      final FakeDaemonStreams serverDaemonStreams = FakeDaemonStreams();
      serverDaemonConnection = DaemonConnection(
        daemonStreams: serverDaemonStreams,
        logger: bufferLogger,
      );
      final FakeDaemonStreams clientDaemonStreams = FakeDaemonStreams();
      clientDaemonConnection = DaemonConnection(
        daemonStreams: clientDaemonStreams,
        logger: bufferLogger,
      );

      serverDaemonStreams.inputs.addStream(clientDaemonStreams.outputs.stream);
      clientDaemonStreams.inputs.addStream(serverDaemonStreams.outputs.stream);

      applicationPackageFactory = FakeApplicationPackageFactory();
      memoryFileSystem = MemoryFileSystem();
      fakeProcessManager = FakeProcessManager.empty();
    });

    tearDown(() async {
      if (daemon != null) {
        return daemon!.shutdown();
      }
      notifyingLogger.dispose();
      await serverDaemonConnection.dispose();
      await clientDaemonConnection.dispose();
    });

    testUsingContext('can list devices', () async {
      daemon = Daemon(serverDaemonConnection, notifyingLogger: notifyingLogger);
      fakeDevice = FakeAndroidDevice();
      final FakePollingDeviceDiscovery discoverer = FakePollingDeviceDiscovery();
      daemon!.deviceDomain.addDeviceDiscoverer(discoverer);
      discoverer.addDevice(fakeDevice);

      final ProxiedDevices proxiedDevices = ProxiedDevices(
        clientDaemonConnection,
        logger: bufferLogger,
      );

      final List<Device> devices = await proxiedDevices.discoverDevices();
      expect(devices, hasLength(1));
      final Device device = devices[0];
      expect(device.id, fakeDevice.id);
      expect(device.name, 'Proxied ${fakeDevice.name}');
      expect(await device.targetPlatform, await fakeDevice.targetPlatform);
      expect(await device.isLocalEmulator, await fakeDevice.isLocalEmulator);
    });

    testUsingContext('calls supportsRuntimeMode', () async {
      daemon = Daemon(serverDaemonConnection, notifyingLogger: notifyingLogger);
      fakeDevice = FakeAndroidDevice();
      final FakePollingDeviceDiscovery discoverer = FakePollingDeviceDiscovery();
      daemon!.deviceDomain.addDeviceDiscoverer(discoverer);
      discoverer.addDevice(fakeDevice);

      final ProxiedDevices proxiedDevices = ProxiedDevices(
        clientDaemonConnection,
        logger: bufferLogger,
      );

      final List<Device> devices = await proxiedDevices.devices();
      expect(devices, hasLength(1));
      final Device device = devices[0];
      final bool supportsRuntimeMode = await device.supportsRuntimeMode(BuildMode.release);
      expect(fakeDevice.supportsRuntimeModeCalledBuildMode, BuildMode.release);
      expect(supportsRuntimeMode, true);
    }, overrides: <Type, Generator>{Java: () => FakeJava()});

    testUsingContext('redirects logs', () async {
      daemon = Daemon(serverDaemonConnection, notifyingLogger: notifyingLogger);
      fakeDevice = FakeAndroidDevice();
      final FakePollingDeviceDiscovery discoverer = FakePollingDeviceDiscovery();
      daemon!.deviceDomain.addDeviceDiscoverer(discoverer);
      discoverer.addDevice(fakeDevice);

      final ProxiedDevices proxiedDevices = ProxiedDevices(
        clientDaemonConnection,
        logger: bufferLogger,
      );

      final FakeDeviceLogReader fakeLogReader = FakeDeviceLogReader();
      fakeDevice.logReader = fakeLogReader;

      final List<Device> devices = await proxiedDevices.devices();
      expect(devices, hasLength(1));
      final Device device = devices[0];
      final DeviceLogReader logReader = await device.getLogReader();
      fakeLogReader.logLinesController.add('Some log line');

      final String receivedLogLine = await logReader.logLines.first;
      expect(receivedLogLine, 'Some log line');

      // Now try to stop the log reader
      expect(fakeLogReader.disposeCalled, false);
      logReader.dispose();
      await pumpEventQueue();
      expect(fakeLogReader.disposeCalled, true);
    });
    testUsingContext(
      'starts and stops app',
      () async {
        daemon = Daemon(serverDaemonConnection, notifyingLogger: notifyingLogger);
        fakeDevice = FakeAndroidDevice();
        final FakePollingDeviceDiscovery discoverer = FakePollingDeviceDiscovery();
        daemon!.deviceDomain.addDeviceDiscoverer(discoverer);
        discoverer.addDevice(fakeDevice);

        final ProxiedDevices proxiedDevices = ProxiedDevices(
          clientDaemonConnection,
          logger: bufferLogger,
        );
        final FakePrebuiltApplicationPackage prebuiltApplicationPackage =
            FakePrebuiltApplicationPackage();
        final File dummyApplicationBinary = memoryFileSystem.file('/directory/dummy_file');
        dummyApplicationBinary.parent.createSync();
        dummyApplicationBinary.writeAsStringSync('dummy content');
        prebuiltApplicationPackage.applicationPackage = dummyApplicationBinary;

        final List<Device> devices = await proxiedDevices.devices();
        expect(devices, hasLength(1));
        final Device device = devices[0];

        // Now try to start the app
        final FakeApplicationPackage applicationPackage = FakeApplicationPackage();
        applicationPackageFactory.applicationPackage = applicationPackage;

        final Uri vmServiceUri = Uri.parse('http://127.0.0.1:12345/vmService');
        fakeDevice.launchResult = LaunchResult.succeeded(vmServiceUri: vmServiceUri);

        final LaunchResult launchResult = await device.startApp(
          prebuiltApplicationPackage,
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        );

        expect(launchResult.started, true);
        // The returned vmServiceUri was a forwarded port, so we cannot compare them directly.
        expect(launchResult.vmServiceUri!.path, vmServiceUri.path);

        expect(
          applicationPackageFactory.applicationBinaryRequested!.readAsStringSync(),
          'dummy content',
        );
        expect(applicationPackageFactory.platformRequested, TargetPlatform.android_arm);

        expect(fakeDevice.startAppPackage, applicationPackage);

        // Now try to stop the app
        final bool stopAppResult = await device.stopApp(prebuiltApplicationPackage);
        expect(fakeDevice.stopAppPackage, applicationPackage);
        expect(stopAppResult, true);
      },
      overrides: <Type, Generator>{
        Java: () => FakeJava(),
        ApplicationPackageFactory: () => applicationPackageFactory,
        FileSystem: () => memoryFileSystem,
        ProcessManager: () => fakeProcessManager,
      },
    );

    testUsingContext(
      'takes screenshot',
      () async {
        daemon = Daemon(serverDaemonConnection, notifyingLogger: notifyingLogger);
        fakeDevice = FakeAndroidDevice();
        final FakePollingDeviceDiscovery discoverer = FakePollingDeviceDiscovery();
        daemon!.deviceDomain.addDeviceDiscoverer(discoverer);
        discoverer.addDevice(fakeDevice);

        final ProxiedDevices proxiedDevices = ProxiedDevices(
          clientDaemonConnection,
          logger: bufferLogger,
        );

        final List<Device> devices = await proxiedDevices.devices();
        expect(devices, hasLength(1));
        final Device device = devices[0];

        final List<int> screenshot = <int>[1, 2, 3, 4, 5];
        fakeDevice.screenshot = screenshot;

        final File screenshotOutputFile = memoryFileSystem.file('screenshot_file');
        await device.takeScreenshot(screenshotOutputFile);

        expect(await screenshotOutputFile.readAsBytes(), screenshot);
      },
      overrides: <Type, Generator>{
        Java: () => FakeJava(),
        FileSystem: () => memoryFileSystem,
        ProcessManager: () => fakeProcessManager,
      },
    );
  });
}

class FakeDaemonStreams implements DaemonStreams {
  final StreamController<DaemonMessage> inputs = StreamController<DaemonMessage>();
  final StreamController<DaemonMessage> outputs = StreamController<DaemonMessage>();

  @override
  Stream<DaemonMessage> get inputStream {
    return inputs.stream;
  }

  @override
  void send(Map<String, dynamic> message, [List<int>? binary]) {
    outputs.add(DaemonMessage(message, binary != null ? Stream<List<int>>.value(binary) : null));
  }

  @override
  Future<void> dispose() async {
    await inputs.close();
    // In some tests, outputs have no listeners. We don't wait for outputs to close.
    unawaited(outputs.close());
  }
}

class FakeAndroidDevice extends Fake implements AndroidDevice {
  @override
  final String id = 'device';

  @override
  final String name = 'device';

  @override
  Future<String> get emulatorId async => 'device';

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.android_arm;

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  final Category category = Category.mobile;

  @override
  final PlatformType platformType = PlatformType.android;

  @override
  final bool ephemeral = false;

  @override
  bool get isConnected => true;

  @override
  final DeviceConnectionInterface connectionInterface = DeviceConnectionInterface.attached;

  @override
  Future<String> get sdkNameAndVersion async => 'Android 12';

  @override
  bool get supportsHotReload => true;

  @override
  bool get supportsHotRestart => true;

  @override
  bool get supportsScreenshot => true;

  @override
  bool get supportsFastStart => true;

  @override
  bool get supportsFlutterExit => true;

  @override
  Future<bool> get supportsHardwareRendering async => true;

  @override
  bool get supportsStartPaused => true;

  BuildMode? supportsRuntimeModeCalledBuildMode;
  @override
  Future<bool> supportsRuntimeMode(BuildMode buildMode) async {
    supportsRuntimeModeCalledBuildMode = buildMode;
    return true;
  }

  late DeviceLogReader logReader;
  @override
  FutureOr<DeviceLogReader> getLogReader({ApplicationPackage? app, bool includePastLogs = false}) =>
      logReader;

  ApplicationPackage? startAppPackage;
  late LaunchResult launchResult;
  @override
  Future<LaunchResult> startApp(
    ApplicationPackage? package, {
    String? mainPath,
    String? route,
    DebuggingOptions? debuggingOptions,
    Map<String, Object?> platformArgs = const <String, Object>{},
    bool prebuiltApplication = false,
    bool ipv6 = false,
    String? userIdentifier,
  }) async {
    startAppPackage = package;
    return launchResult;
  }

  ApplicationPackage? stopAppPackage;
  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    stopAppPackage = app;
    return true;
  }

  late List<int> screenshot;
  @override
  Future<void> takeScreenshot(File outputFile) {
    return outputFile.writeAsBytes(screenshot);
  }
}

class FakeDeviceLogReader implements DeviceLogReader {
  final StreamController<String> logLinesController = StreamController<String>();
  bool disposeCalled = false;

  @override
  Future<void> provideVmService(FlutterVmService? connectedVmService) async {}

  @override
  void dispose() {
    disposeCalled = true;
  }

  @override
  Stream<String> get logLines => logLinesController.stream;

  @override
  String get name => 'device';
}

class FakeApplicationPackageFactory implements ApplicationPackageFactory {
  TargetPlatform? platformRequested;
  File? applicationBinaryRequested;
  ApplicationPackage? applicationPackage;

  @override
  Future<ApplicationPackage?> getPackageForPlatform(
    TargetPlatform platform, {
    BuildInfo? buildInfo,
    File? applicationBinary,
  }) async {
    platformRequested = platform;
    applicationBinaryRequested = applicationBinary;
    return applicationPackage;
  }
}

class FakeApplicationPackage extends Fake implements ApplicationPackage {}

class FakePrebuiltApplicationPackage extends Fake implements PrebuiltApplicationPackage {
  @override
  late File applicationPackage;
}
