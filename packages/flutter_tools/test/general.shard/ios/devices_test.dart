// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/ios/application_package.dart';
import 'package:flutter_tools/src/ios/core_devices.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/ios/ios_deploy.dart';
import 'package:flutter_tools/src/ios/ios_workflow.dart';
import 'package:flutter_tools/src/ios/iproxy.dart';
import 'package:flutter_tools/src/ios/mac.dart';
import 'package:flutter_tools/src/ios/xcode_debug.dart';
import 'package:flutter_tools/src/macos/xcdevice.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

void main() {
  final FakePlatform macPlatform = FakePlatform(operatingSystem: 'macos');
  final FakePlatform linuxPlatform = FakePlatform();
  final FakePlatform windowsPlatform = FakePlatform(operatingSystem: 'windows');

  group('IOSDevice', () {
    final List<Platform> unsupportedPlatforms = <Platform>[linuxPlatform, windowsPlatform];
    late Cache cache;
    late Logger logger;
    late IOSDeploy iosDeploy;
    late IMobileDevice iMobileDevice;
    late FileSystem fileSystem;
    late IOSCoreDeviceControl coreDeviceControl;
    late XcodeDebug xcodeDebug;

    setUp(() {
      final Artifacts artifacts = Artifacts.test();
      cache = Cache.test(processManager: FakeProcessManager.any());
      logger = BufferLogger.test();
      fileSystem = MemoryFileSystem.test();
      iosDeploy = IOSDeploy(
        artifacts: artifacts,
        cache: cache,
        logger: logger,
        platform: macPlatform,
        processManager: FakeProcessManager.any(),
      );
      iMobileDevice = IMobileDevice(
        artifacts: artifacts,
        cache: cache,
        logger: logger,
        processManager: FakeProcessManager.any(),
      );
      coreDeviceControl = FakeIOSCoreDeviceControl();
      xcodeDebug = FakeXcodeDebug();
    });

    testWithoutContext('successfully instantiates on Mac OS', () {
      final IOSDevice device = IOSDevice(
        'device-123',
        iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
        fileSystem: fileSystem,
        logger: logger,
        platform: macPlatform,
        iosDeploy: iosDeploy,
        iMobileDevice: iMobileDevice,
        coreDeviceControl: coreDeviceControl,
        xcodeDebug: xcodeDebug,
        name: 'iPhone 1',
        sdkVersion: '13.3',
        cpuArchitecture: DarwinArch.arm64,
        connectionInterface: DeviceConnectionInterface.attached,
        isConnected: true,
        isPaired: true,
        devModeEnabled: true,
        isCoreDevice: false,
      );
      expect(device.isSupported(), isTrue);
    });

    testWithoutContext('32-bit devices are unsupported', () {
      final IOSDevice device = IOSDevice(
        'device-123',
        iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
        fileSystem: fileSystem,
        logger: logger,
        platform: macPlatform,
        iosDeploy: iosDeploy,
        iMobileDevice: iMobileDevice,
        coreDeviceControl: coreDeviceControl,
        xcodeDebug: xcodeDebug,
        name: 'iPhone 1',
        cpuArchitecture: DarwinArch.armv7,
        connectionInterface: DeviceConnectionInterface.attached,
        isConnected: true,
        isPaired: true,
        devModeEnabled: true,
        isCoreDevice: false,
      );
      expect(device.isSupported(), isFalse);
    });

    testWithoutContext('parses major version', () {
      expect(
        IOSDevice(
          'device-123',
          iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
          fileSystem: fileSystem,
          logger: logger,
          platform: macPlatform,
          iosDeploy: iosDeploy,
          iMobileDevice: iMobileDevice,
          coreDeviceControl: coreDeviceControl,
          xcodeDebug: xcodeDebug,
          name: 'iPhone 1',
          cpuArchitecture: DarwinArch.arm64,
          sdkVersion: '1.0.0',
          connectionInterface: DeviceConnectionInterface.attached,
          isConnected: true,
          isPaired: true,
          devModeEnabled: true,
          isCoreDevice: false,
        ).majorSdkVersion,
        1,
      );
      expect(
        IOSDevice(
          'device-123',
          iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
          fileSystem: fileSystem,
          logger: logger,
          platform: macPlatform,
          iosDeploy: iosDeploy,
          iMobileDevice: iMobileDevice,
          coreDeviceControl: coreDeviceControl,
          xcodeDebug: xcodeDebug,
          name: 'iPhone 1',
          cpuArchitecture: DarwinArch.arm64,
          sdkVersion: '13.1.1',
          connectionInterface: DeviceConnectionInterface.attached,
          isConnected: true,
          isPaired: true,
          devModeEnabled: true,
          isCoreDevice: false,
        ).majorSdkVersion,
        13,
      );
      expect(
        IOSDevice(
          'device-123',
          iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
          fileSystem: fileSystem,
          logger: logger,
          platform: macPlatform,
          iosDeploy: iosDeploy,
          iMobileDevice: iMobileDevice,
          coreDeviceControl: coreDeviceControl,
          xcodeDebug: xcodeDebug,
          name: 'iPhone 1',
          cpuArchitecture: DarwinArch.arm64,
          sdkVersion: '10',
          connectionInterface: DeviceConnectionInterface.attached,
          isConnected: true,
          isPaired: true,
          devModeEnabled: true,
          isCoreDevice: false,
        ).majorSdkVersion,
        10,
      );
      expect(
        IOSDevice(
          'device-123',
          iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
          fileSystem: fileSystem,
          logger: logger,
          platform: macPlatform,
          iosDeploy: iosDeploy,
          iMobileDevice: iMobileDevice,
          coreDeviceControl: coreDeviceControl,
          xcodeDebug: xcodeDebug,
          name: 'iPhone 1',
          cpuArchitecture: DarwinArch.arm64,
          sdkVersion: '0',
          connectionInterface: DeviceConnectionInterface.attached,
          isConnected: true,
          isPaired: true,
          devModeEnabled: true,
          isCoreDevice: false,
        ).majorSdkVersion,
        0,
      );
      expect(
        IOSDevice(
          'device-123',
          iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
          fileSystem: fileSystem,
          logger: logger,
          platform: macPlatform,
          iosDeploy: iosDeploy,
          iMobileDevice: iMobileDevice,
          coreDeviceControl: coreDeviceControl,
          xcodeDebug: xcodeDebug,
          name: 'iPhone 1',
          cpuArchitecture: DarwinArch.arm64,
          sdkVersion: 'bogus',
          connectionInterface: DeviceConnectionInterface.attached,
          isConnected: true,
          isPaired: true,
          devModeEnabled: true,
          isCoreDevice: false,
        ).majorSdkVersion,
        0,
      );
    });

    testWithoutContext('parses sdk version', () {
      Version? sdkVersion =
          IOSDevice(
            'device-123',
            iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
            fileSystem: fileSystem,
            logger: logger,
            platform: macPlatform,
            iosDeploy: iosDeploy,
            iMobileDevice: iMobileDevice,
            coreDeviceControl: coreDeviceControl,
            xcodeDebug: xcodeDebug,
            name: 'iPhone 1',
            cpuArchitecture: DarwinArch.arm64,
            sdkVersion: '13.3.1',
            connectionInterface: DeviceConnectionInterface.attached,
            isConnected: true,
            isPaired: true,
            devModeEnabled: true,
            isCoreDevice: false,
          ).sdkVersion;
      Version expectedVersion = Version(13, 3, 1, text: '13.3.1');
      expect(sdkVersion, isNotNull);
      expect(sdkVersion!.toString(), expectedVersion.toString());
      expect(sdkVersion.compareTo(expectedVersion), 0);

      sdkVersion =
          IOSDevice(
            'device-123',
            iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
            fileSystem: fileSystem,
            logger: logger,
            platform: macPlatform,
            iosDeploy: iosDeploy,
            iMobileDevice: iMobileDevice,
            coreDeviceControl: coreDeviceControl,
            xcodeDebug: xcodeDebug,
            name: 'iPhone 1',
            cpuArchitecture: DarwinArch.arm64,
            sdkVersion: '13.3.1 (20ADBC)',
            connectionInterface: DeviceConnectionInterface.attached,
            isConnected: true,
            isPaired: true,
            devModeEnabled: true,
            isCoreDevice: false,
          ).sdkVersion;
      expectedVersion = Version(13, 3, 1, text: '13.3.1 (20ADBC)');
      expect(sdkVersion, isNotNull);
      expect(sdkVersion!.toString(), expectedVersion.toString());
      expect(sdkVersion.compareTo(expectedVersion), 0);

      sdkVersion =
          IOSDevice(
            'device-123',
            iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
            fileSystem: fileSystem,
            logger: logger,
            platform: macPlatform,
            iosDeploy: iosDeploy,
            iMobileDevice: iMobileDevice,
            coreDeviceControl: coreDeviceControl,
            xcodeDebug: xcodeDebug,
            name: 'iPhone 1',
            cpuArchitecture: DarwinArch.arm64,
            sdkVersion: '16.4.1(a) (20ADBC)',
            connectionInterface: DeviceConnectionInterface.attached,
            isConnected: true,
            isPaired: true,
            devModeEnabled: true,
            isCoreDevice: false,
          ).sdkVersion;
      expectedVersion = Version(16, 4, 1, text: '16.4.1(a) (20ADBC)');
      expect(sdkVersion, isNotNull);
      expect(sdkVersion!.toString(), expectedVersion.toString());
      expect(sdkVersion.compareTo(expectedVersion), 0);

      sdkVersion =
          IOSDevice(
            'device-123',
            iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
            fileSystem: fileSystem,
            logger: logger,
            platform: macPlatform,
            iosDeploy: iosDeploy,
            iMobileDevice: iMobileDevice,
            coreDeviceControl: coreDeviceControl,
            xcodeDebug: xcodeDebug,
            name: 'iPhone 1',
            cpuArchitecture: DarwinArch.arm64,
            sdkVersion: '0',
            connectionInterface: DeviceConnectionInterface.attached,
            isConnected: true,
            isPaired: true,
            devModeEnabled: true,
            isCoreDevice: false,
          ).sdkVersion;
      expectedVersion = Version(0, 0, 0, text: '0');
      expect(sdkVersion, isNotNull);
      expect(sdkVersion!.toString(), expectedVersion.toString());
      expect(sdkVersion.compareTo(expectedVersion), 0);

      sdkVersion =
          IOSDevice(
            'device-123',
            iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
            fileSystem: fileSystem,
            logger: logger,
            platform: macPlatform,
            iosDeploy: iosDeploy,
            iMobileDevice: iMobileDevice,
            coreDeviceControl: coreDeviceControl,
            xcodeDebug: xcodeDebug,
            name: 'iPhone 1',
            cpuArchitecture: DarwinArch.arm64,
            connectionInterface: DeviceConnectionInterface.attached,
            isConnected: true,
            isPaired: true,
            devModeEnabled: true,
            isCoreDevice: false,
          ).sdkVersion;
      expect(sdkVersion, isNull);

      sdkVersion =
          IOSDevice(
            'device-123',
            iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
            fileSystem: fileSystem,
            logger: logger,
            platform: macPlatform,
            iosDeploy: iosDeploy,
            iMobileDevice: iMobileDevice,
            coreDeviceControl: coreDeviceControl,
            xcodeDebug: xcodeDebug,
            name: 'iPhone 1',
            cpuArchitecture: DarwinArch.arm64,
            sdkVersion: 'bogus',
            connectionInterface: DeviceConnectionInterface.attached,
            isConnected: true,
            isPaired: true,
            devModeEnabled: true,
            isCoreDevice: false,
          ).sdkVersion;
      expect(sdkVersion, isNull);
    });

    testWithoutContext('has build number in sdkNameAndVersion', () async {
      final IOSDevice device = IOSDevice(
        'device-123',
        iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
        fileSystem: fileSystem,
        logger: logger,
        platform: macPlatform,
        iosDeploy: iosDeploy,
        iMobileDevice: iMobileDevice,
        coreDeviceControl: coreDeviceControl,
        xcodeDebug: xcodeDebug,
        name: 'iPhone 1',
        sdkVersion: '13.3 17C54',
        cpuArchitecture: DarwinArch.arm64,
        connectionInterface: DeviceConnectionInterface.attached,
        isConnected: true,
        isPaired: true,
        devModeEnabled: true,
        isCoreDevice: false,
      );

      expect(await device.sdkNameAndVersion, 'iOS 13.3 17C54');
    });

    testWithoutContext('Supports debug, profile, and release modes', () {
      final IOSDevice device = IOSDevice(
        'device-123',
        iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
        fileSystem: fileSystem,
        logger: logger,
        platform: macPlatform,
        iosDeploy: iosDeploy,
        iMobileDevice: iMobileDevice,
        coreDeviceControl: coreDeviceControl,
        xcodeDebug: xcodeDebug,
        name: 'iPhone 1',
        sdkVersion: '13.3',
        cpuArchitecture: DarwinArch.arm64,
        connectionInterface: DeviceConnectionInterface.attached,
        isConnected: true,
        isPaired: true,
        devModeEnabled: true,
        isCoreDevice: false,
      );

      expect(device.supportsRuntimeMode(BuildMode.debug), true);
      expect(device.supportsRuntimeMode(BuildMode.profile), true);
      expect(device.supportsRuntimeMode(BuildMode.release), true);
      expect(device.supportsRuntimeMode(BuildMode.jitRelease), false);
    });

    for (final Platform platform in unsupportedPlatforms) {
      testWithoutContext(
        'throws UnsupportedError exception if instantiated on ${platform.operatingSystem}',
        () {
          expect(() {
            IOSDevice(
              'device-123',
              iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
              fileSystem: fileSystem,
              logger: logger,
              platform: platform,
              iosDeploy: iosDeploy,
              iMobileDevice: iMobileDevice,
              coreDeviceControl: coreDeviceControl,
              xcodeDebug: xcodeDebug,
              name: 'iPhone 1',
              sdkVersion: '13.3',
              cpuArchitecture: DarwinArch.arm64,
              connectionInterface: DeviceConnectionInterface.attached,
              isConnected: true,
              isPaired: true,
              devModeEnabled: true,
              isCoreDevice: false,
            );
          }, throwsAssertionError);
        },
      );
    }

    group('.dispose()', () {
      late IOSDevice device;
      late FakeIOSApp appPackage1;
      late FakeIOSApp appPackage2;
      late IOSDeviceLogReader logReader1;
      late IOSDeviceLogReader logReader2;
      late FakeProcess process1;
      late FakeProcess process2;
      late FakeProcess process3;
      late IOSDevicePortForwarder portForwarder;
      late ForwardedPort forwardedPort;
      late Cache cache;
      late Logger logger;
      late IOSDeploy iosDeploy;
      late FileSystem fileSystem;
      late IProxy iproxy;

      IOSDevicePortForwarder createPortForwarder(ForwardedPort forwardedPort, IOSDevice device) {
        iproxy = IProxy.test(logger: logger, processManager: FakeProcessManager.any());
        final IOSDevicePortForwarder portForwarder = IOSDevicePortForwarder(
          id: device.id,
          logger: logger,
          operatingSystemUtils: OperatingSystemUtils(
            fileSystem: fileSystem,
            logger: logger,
            platform: FakePlatform(operatingSystem: 'macos'),
            processManager: FakeProcessManager.any(),
          ),
          iproxy: iproxy,
        );
        portForwarder.addForwardedPorts(<ForwardedPort>[forwardedPort]);
        return portForwarder;
      }

      IOSDeviceLogReader createLogReader(IOSDevice device, IOSApp appPackage, Process process) {
        final IOSDeviceLogReader logReader = IOSDeviceLogReader.create(
          device: device,
          app: appPackage,
          iMobileDevice: IMobileDevice.test(processManager: FakeProcessManager.any()),
        );
        logReader.idevicesyslogProcess = process;
        return logReader;
      }

      setUp(() {
        appPackage1 = FakeIOSApp('flutterApp1');
        appPackage2 = FakeIOSApp('flutterApp2');
        process1 = FakeProcess();
        process2 = FakeProcess();
        process3 = FakeProcess();
        forwardedPort = ForwardedPort.withContext(123, 456, process3);
        cache = Cache.test(processManager: FakeProcessManager.any());
        fileSystem = MemoryFileSystem.test();
        logger = BufferLogger.test();
        iosDeploy = IOSDeploy(
          artifacts: Artifacts.test(),
          cache: cache,
          logger: logger,
          platform: macPlatform,
          processManager: FakeProcessManager.any(),
        );
      });

      testWithoutContext('kills all log readers & port forwarders', () async {
        device = IOSDevice(
          '123',
          iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
          fileSystem: fileSystem,
          logger: logger,
          platform: macPlatform,
          iosDeploy: iosDeploy,
          iMobileDevice: iMobileDevice,
          coreDeviceControl: coreDeviceControl,
          xcodeDebug: xcodeDebug,
          name: 'iPhone 1',
          sdkVersion: '13.3',
          cpuArchitecture: DarwinArch.arm64,
          connectionInterface: DeviceConnectionInterface.attached,
          isConnected: true,
          isPaired: true,
          devModeEnabled: true,
          isCoreDevice: false,
        );
        logReader1 = createLogReader(device, appPackage1, process1);
        logReader2 = createLogReader(device, appPackage2, process2);
        portForwarder = createPortForwarder(forwardedPort, device);
        device.setLogReader(appPackage1, logReader1);
        device.setLogReader(appPackage2, logReader2);
        device.portForwarder = portForwarder;

        await device.dispose();

        expect(process1.killed, true);
        expect(process2.killed, true);
        expect(process3.killed, true);
      });
    });
  });

  group('polling', () {
    late FakeXcdevice xcdevice;
    late Cache cache;
    late FakeProcessManager fakeProcessManager;
    late BufferLogger logger;
    late IOSDeploy iosDeploy;
    late IMobileDevice iMobileDevice;
    late IOSWorkflow iosWorkflow;
    late IOSCoreDeviceControl coreDeviceControl;
    late XcodeDebug xcodeDebug;
    late IOSDevice device1;
    late IOSDevice device2;

    setUp(() {
      xcdevice = FakeXcdevice();
      final Artifacts artifacts = Artifacts.test();
      cache = Cache.test(processManager: FakeProcessManager.any());
      logger = BufferLogger.test();
      iosWorkflow = FakeIOSWorkflow();
      fakeProcessManager = FakeProcessManager.any();
      iosDeploy = IOSDeploy(
        artifacts: artifacts,
        cache: cache,
        logger: logger,
        platform: macPlatform,
        processManager: fakeProcessManager,
      );
      iMobileDevice = IMobileDevice(
        artifacts: artifacts,
        cache: cache,
        processManager: fakeProcessManager,
        logger: logger,
      );
      coreDeviceControl = FakeIOSCoreDeviceControl();
      xcodeDebug = FakeXcodeDebug();

      device1 = IOSDevice(
        'd83d5bc53967baa0ee18626ba87b6254b2ab5418',
        name: 'Paired iPhone',
        sdkVersion: '13.3',
        cpuArchitecture: DarwinArch.arm64,
        iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
        iosDeploy: iosDeploy,
        iMobileDevice: iMobileDevice,
        coreDeviceControl: coreDeviceControl,
        xcodeDebug: xcodeDebug,
        logger: logger,
        platform: macPlatform,
        fileSystem: MemoryFileSystem.test(),
        connectionInterface: DeviceConnectionInterface.attached,
        isConnected: true,
        isPaired: true,
        devModeEnabled: true,
        isCoreDevice: false,
      );

      device2 = IOSDevice(
        '00008027-00192736010F802E',
        name: 'iPad Pro',
        sdkVersion: '13.3',
        cpuArchitecture: DarwinArch.arm64,
        iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
        iosDeploy: iosDeploy,
        iMobileDevice: iMobileDevice,
        coreDeviceControl: coreDeviceControl,
        xcodeDebug: xcodeDebug,
        logger: logger,
        platform: macPlatform,
        fileSystem: MemoryFileSystem.test(),
        connectionInterface: DeviceConnectionInterface.attached,
        isConnected: true,
        isPaired: true,
        devModeEnabled: true,
        isCoreDevice: false,
      );
    });

    testWithoutContext('start polling without Xcode', () async {
      final IOSDevices iosDevices = IOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = false;

      await iosDevices.startPolling();
      expect(xcdevice.getAvailableIOSDevicesCount, 0);
    });

    testWithoutContext('start polling', () async {
      final TestIOSDevices iosDevices = TestIOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = true;
      xcdevice.devices
        ..add(<IOSDevice>[])
        ..add(<IOSDevice>[device1, device2]);

      int addedCount = 0;
      final Completer<void> added = Completer<void>();
      iosDevices.onAdded.listen((Device device) {
        addedCount++;
        // 2 devices will be added.
        // Will throw over-completion if called more than twice.
        if (addedCount >= 2) {
          added.complete();
        }
      });

      final Completer<void> removed = Completer<void>();
      iosDevices.onRemoved.listen((Device device) {
        // Will throw over-completion if called more than once.
        removed.complete();
      });

      await iosDevices.startPolling();
      expect(xcdevice.getAvailableIOSDevicesCount, 1);

      expect(iosDevices.deviceNotifier.items, isEmpty);
      expect(xcdevice.deviceEventController.hasListener, isTrue);

      xcdevice.deviceEventController.add(
        XCDeviceEventNotification(
          XCDeviceEvent.attach,
          XCDeviceEventInterface.usb,
          'd83d5bc53967baa0ee18626ba87b6254b2ab5418',
        ),
      );
      await added.future;
      expect(iosDevices.deviceNotifier.items.length, 2);
      expect(iosDevices.deviceNotifier.items, contains(device1));
      expect(iosDevices.deviceNotifier.items, contains(device2));
      expect(iosDevices.eventsReceived, 1);

      iosDevices.resetEventCompleter();
      xcdevice.deviceEventController.add(
        XCDeviceEventNotification(
          XCDeviceEvent.attach,
          XCDeviceEventInterface.wifi,
          'd83d5bc53967baa0ee18626ba87b6254b2ab5418',
        ),
      );
      await iosDevices.receivedEvent.future;
      expect(iosDevices.deviceNotifier.items.length, 2);
      expect(iosDevices.deviceNotifier.items, contains(device1));
      expect(iosDevices.deviceNotifier.items, contains(device2));
      expect(iosDevices.eventsReceived, 2);

      iosDevices.resetEventCompleter();
      xcdevice.deviceEventController.add(
        XCDeviceEventNotification(
          XCDeviceEvent.detach,
          XCDeviceEventInterface.usb,
          'd83d5bc53967baa0ee18626ba87b6254b2ab5418',
        ),
      );
      await iosDevices.receivedEvent.future;
      expect(iosDevices.deviceNotifier.items.length, 2);
      expect(iosDevices.deviceNotifier.items, contains(device1));
      expect(iosDevices.deviceNotifier.items, contains(device2));
      expect(iosDevices.eventsReceived, 3);

      xcdevice.deviceEventController.add(
        XCDeviceEventNotification(
          XCDeviceEvent.detach,
          XCDeviceEventInterface.wifi,
          'd83d5bc53967baa0ee18626ba87b6254b2ab5418',
        ),
      );
      await removed.future;
      expect(iosDevices.deviceNotifier.items, <Device>[device2]);
      expect(iosDevices.eventsReceived, 4);

      iosDevices.resetEventCompleter();
      xcdevice.deviceEventController.add(
        XCDeviceEventNotification(XCDeviceEvent.detach, XCDeviceEventInterface.usb, 'bogus'),
      );
      await iosDevices.receivedEvent.future;
      expect(iosDevices.eventsReceived, 5);

      expect(addedCount, 2);

      await iosDevices.stopPolling();

      expect(xcdevice.deviceEventController.hasListener, isFalse);
    });

    testWithoutContext('polling can be restarted if stream is closed', () async {
      final IOSDevices iosDevices = IOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = true;
      xcdevice.devices.add(<IOSDevice>[]);
      xcdevice.devices.add(<IOSDevice>[]);

      final StreamController<XCDeviceEventNotification> rescheduledStream =
          StreamController<XCDeviceEventNotification>();

      unawaited(
        xcdevice.deviceEventController.done.whenComplete(() {
          xcdevice.deviceEventController = rescheduledStream;
        }),
      );

      await iosDevices.startPolling();
      expect(xcdevice.deviceEventController.hasListener, isTrue);
      expect(xcdevice.getAvailableIOSDevicesCount, 1);

      // Pretend xcdevice crashed.
      await xcdevice.deviceEventController.close();
      expect(logger.traceText, contains('xcdevice observe stopped'));

      // Confirm a restart still gets streamed events.
      await iosDevices.startPolling();

      expect(rescheduledStream.hasListener, isTrue);

      await iosDevices.stopPolling();
      expect(rescheduledStream.hasListener, isFalse);
    });

    testWithoutContext('dispose cancels polling subscription', () async {
      final IOSDevices iosDevices = IOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = true;
      xcdevice.devices.add(<IOSDevice>[]);

      await iosDevices.startPolling();
      expect(iosDevices.deviceNotifier.items, isEmpty);
      expect(xcdevice.deviceEventController.hasListener, isTrue);

      iosDevices.dispose();
      expect(xcdevice.deviceEventController.hasListener, isFalse);
    });

    final List<Platform> unsupportedPlatforms = <Platform>[linuxPlatform, windowsPlatform];
    for (final Platform unsupportedPlatform in unsupportedPlatforms) {
      testWithoutContext(
        'pollingGetDevices throws Unsupported Operation exception on ${unsupportedPlatform.operatingSystem}',
        () async {
          final IOSDevices iosDevices = IOSDevices(
            platform: unsupportedPlatform,
            xcdevice: xcdevice,
            iosWorkflow: iosWorkflow,
            logger: logger,
          );
          xcdevice.isInstalled = false;
          expect(() async {
            await iosDevices.pollingGetDevices();
          }, throwsUnsupportedError);
        },
      );
    }

    testWithoutContext('pollingGetDevices returns attached devices', () async {
      final IOSDevices iosDevices = IOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = true;
      xcdevice.devices.add(<IOSDevice>[device1]);

      final List<Device> devices = await iosDevices.pollingGetDevices();

      expect(devices, hasLength(1));
      expect(devices.first, same(device1));
    });
  });

  group('getDiagnostics', () {
    late FakeXcdevice xcdevice;
    late IOSWorkflow iosWorkflow;
    late Logger logger;

    setUp(() {
      xcdevice = FakeXcdevice();
      iosWorkflow = FakeIOSWorkflow();
      logger = BufferLogger.test();
    });

    final List<Platform> unsupportedPlatforms = <Platform>[linuxPlatform, windowsPlatform];
    for (final Platform unsupportedPlatform in unsupportedPlatforms) {
      testWithoutContext(
        'throws returns platform diagnostic exception on ${unsupportedPlatform.operatingSystem}',
        () async {
          final IOSDevices iosDevices = IOSDevices(
            platform: unsupportedPlatform,
            xcdevice: xcdevice,
            iosWorkflow: iosWorkflow,
            logger: logger,
          );
          xcdevice.isInstalled = false;
          expect(
            (await iosDevices.getDiagnostics()).first,
            'Control of iOS devices or simulators only supported on macOS.',
          );
        },
      );
    }

    testWithoutContext('returns diagnostics', () async {
      final IOSDevices iosDevices = IOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = true;
      xcdevice.diagnostics.add('Generic pairing error');

      final List<String> diagnostics = await iosDevices.getDiagnostics();
      expect(diagnostics, hasLength(1));
      expect(diagnostics.first, 'Generic pairing error');
    });
  });

  group('waitForDeviceToConnect', () {
    late FakeXcdevice xcdevice;
    late Cache cache;
    late FakeProcessManager fakeProcessManager;
    late BufferLogger logger;
    late IOSDeploy iosDeploy;
    late IMobileDevice iMobileDevice;
    late IOSWorkflow iosWorkflow;
    late IOSCoreDeviceControl coreDeviceControl;
    late XcodeDebug xcodeDebug;
    late IOSDevice notConnected1;

    setUp(() {
      xcdevice = FakeXcdevice();
      final Artifacts artifacts = Artifacts.test();
      cache = Cache.test(processManager: FakeProcessManager.any());
      logger = BufferLogger.test();
      iosWorkflow = FakeIOSWorkflow();
      fakeProcessManager = FakeProcessManager.any();
      iosDeploy = IOSDeploy(
        artifacts: artifacts,
        cache: cache,
        logger: logger,
        platform: macPlatform,
        processManager: fakeProcessManager,
      );
      iMobileDevice = IMobileDevice(
        artifacts: artifacts,
        cache: cache,
        processManager: fakeProcessManager,
        logger: logger,
      );
      coreDeviceControl = FakeIOSCoreDeviceControl();
      xcodeDebug = FakeXcodeDebug();
      notConnected1 = IOSDevice(
        '00000001-0000000000000000',
        name: 'iPad',
        sdkVersion: '13.3',
        cpuArchitecture: DarwinArch.arm64,
        iProxy: IProxy.test(logger: logger, processManager: FakeProcessManager.any()),
        iosDeploy: iosDeploy,
        iMobileDevice: iMobileDevice,
        coreDeviceControl: coreDeviceControl,
        xcodeDebug: xcodeDebug,
        logger: logger,
        platform: macPlatform,
        fileSystem: MemoryFileSystem.test(),
        connectionInterface: DeviceConnectionInterface.attached,
        isConnected: false,
        isPaired: true,
        devModeEnabled: true,
        isCoreDevice: false,
      );
    });

    testWithoutContext('wait for device to connect via wifi', () async {
      final IOSDevices iosDevices = IOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = true;

      xcdevice.waitForDeviceEvent = XCDeviceEventNotification(
        XCDeviceEvent.attach,
        XCDeviceEventInterface.wifi,
        '00000001-0000000000000000',
      );

      final Device? device = await iosDevices.waitForDeviceToConnect(notConnected1, logger);

      expect(device?.isConnected, isTrue);
      expect(device?.connectionInterface, DeviceConnectionInterface.wireless);
    });

    testWithoutContext('wait for device to connect via usb', () async {
      final IOSDevices iosDevices = IOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = true;

      xcdevice.waitForDeviceEvent = XCDeviceEventNotification(
        XCDeviceEvent.attach,
        XCDeviceEventInterface.usb,
        '00000001-0000000000000000',
      );

      final Device? device = await iosDevices.waitForDeviceToConnect(notConnected1, logger);

      expect(device?.isConnected, isTrue);
      expect(device?.connectionInterface, DeviceConnectionInterface.attached);
    });

    testWithoutContext('wait for device returns null', () async {
      final IOSDevices iosDevices = IOSDevices(
        platform: macPlatform,
        xcdevice: xcdevice,
        iosWorkflow: iosWorkflow,
        logger: logger,
      );
      xcdevice.isInstalled = true;

      xcdevice.waitForDeviceEvent = null;

      final Device? device = await iosDevices.waitForDeviceToConnect(notConnected1, logger);

      expect(device, isNull);
    });
  });
}

class FakeIOSApp extends Fake implements IOSApp {
  FakeIOSApp(this.name);

  @override
  final String name;
}

class TestIOSDevices extends IOSDevices {
  TestIOSDevices({
    required super.platform,
    required super.xcdevice,
    required super.iosWorkflow,
    required super.logger,
  });

  Completer<void> receivedEvent = Completer<void>();
  int eventsReceived = 0;

  void resetEventCompleter() {
    receivedEvent = Completer<void>();
  }

  @override
  Future<void> onDeviceEvent(XCDeviceEventNotification event) async {
    await super.onDeviceEvent(event);
    if (!receivedEvent.isCompleted) {
      receivedEvent.complete();
    }
    eventsReceived++;
    return;
  }
}

class FakeIOSWorkflow extends Fake implements IOSWorkflow {}

class FakeXcdevice extends Fake implements XCDevice {
  int getAvailableIOSDevicesCount = 0;
  final List<List<IOSDevice>> devices = <List<IOSDevice>>[];
  final List<String> diagnostics = <String>[];
  StreamController<XCDeviceEventNotification> deviceEventController =
      StreamController<XCDeviceEventNotification>();

  XCDeviceEventNotification? waitForDeviceEvent;

  @override
  bool isInstalled = true;

  @override
  Future<List<String>> getDiagnostics() async {
    return diagnostics;
  }

  @override
  Stream<XCDeviceEventNotification> observedDeviceEvents() {
    return deviceEventController.stream;
  }

  @override
  Future<List<IOSDevice>> getAvailableIOSDevices({Duration? timeout}) async {
    return devices[getAvailableIOSDevicesCount++];
  }

  @override
  Future<XCDeviceEventNotification?> waitForDeviceToConnect(String deviceId) async {
    final XCDeviceEventNotification? waitEvent = waitForDeviceEvent;
    if (waitEvent != null) {
      return XCDeviceEventNotification(
        waitEvent.eventType,
        waitEvent.eventInterface,
        waitEvent.deviceIdentifier,
      );
    } else {
      return null;
    }
  }
}

class FakeProcess extends Fake implements Process {
  bool killed = false;

  @override
  bool kill([io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    killed = true;
    return true;
  }
}

class FakeXcodeDebug extends Fake implements XcodeDebug {
  @override
  bool get debugStarted => false;
}

class FakeIOSCoreDeviceControl extends Fake implements IOSCoreDeviceControl {}
