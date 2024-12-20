// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:fake_async/fake_async.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/utils.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_devices.dart';

void main() {
  group('DeviceManager', () {
    testWithoutContext('getDevices', () async {
      final FakeDevice device1 = FakeDevice('Nexus 5', '0553790d0a4e726f');
      final FakeDevice device2 = FakeDevice('Nexus 5X', '01abfc49119c410e');
      final FakeDevice device3 = FakeDevice('iPod touch', '82564b38861a9a5');
      final List<Device> devices = <Device>[device1, device2, device3];

      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());

      expect(await deviceManager.getDevices(), devices);
    });

    testWithoutContext('getDeviceById exact matcher', () async {
      final FakeDevice device1 = FakeDevice('Nexus 5', '0553790d0a4e726f');
      final FakeDevice device2 = FakeDevice('Nexus 5X', '01abfc49119c410e');
      final FakeDevice device3 = FakeDevice('iPod touch', '82564b38861a9a5');
      final List<Device> devices = <Device>[device1, device2, device3];
      final BufferLogger logger = BufferLogger.test();

      // Include different device discoveries:
      // 1. One that never completes to prove the first exact match is
      // returned quickly.
      // 2. One that throws, to prove matches can return when some succeed
      // and others fail.
      // 3. A device discoverer that succeeds.
      final DeviceManager deviceManager = TestDeviceManager(
        devices,
        deviceDiscoveryOverrides: <DeviceDiscovery>[
          ThrowingPollingDeviceDiscovery(),
          LongPollingDeviceDiscovery(),
        ],
        logger: logger,
      );

      Future<void> expectDevice(String id, List<Device> expected) async {
        expect(await deviceManager.getDevicesById(id), expected);
      }

      await expectDevice('01abfc49119c410e', <Device>[device2]);
      expect(logger.traceText, contains('Ignored error discovering 01abfc49119c410e'));
      await expectDevice('Nexus 5X', <Device>[device2]);
      expect(logger.traceText, contains('Ignored error discovering Nexus 5X'));
      await expectDevice('0553790d0a4e726f', <Device>[device1]);
      expect(logger.traceText, contains('Ignored error discovering 0553790d0a4e726f'));
    });

    testWithoutContext('getDeviceById exact matcher with well known ID', () async {
      final FakeDevice device1 = FakeDevice('Windows', 'windows');
      final FakeDevice device2 = FakeDevice('Nexus 5X', '01abfc49119c410e');
      final FakeDevice device3 = FakeDevice('iPod touch', '82564b38861a9a5');
      final List<Device> devices = <Device>[device1, device2, device3];
      final BufferLogger logger = BufferLogger.test();

      // Because the well known ID will match, no other device discovery will run.
      final DeviceManager deviceManager = TestDeviceManager(
        devices,
        deviceDiscoveryOverrides: <DeviceDiscovery>[
          ThrowingPollingDeviceDiscovery(),
          LongPollingDeviceDiscovery(),
        ],
        logger: logger,
        wellKnownId: 'windows',
      );

      Future<void> expectDevice(String id, List<Device> expected) async {
        deviceManager.specifiedDeviceId = id;
        expect(await deviceManager.getDevicesById(id), expected);
      }

      await expectDevice('windows', <Device>[device1]);
      expect(logger.traceText, isEmpty);
    });

    testWithoutContext('getDeviceById prefix matcher', () async {
      final FakeDevice device1 = FakeDevice('Nexus 5', '0553790d0a4e726f');
      final FakeDevice device2 = FakeDevice('Nexus 5X', '01abfc49119c410e');
      final FakeDevice device3 = FakeDevice('iPod touch', '82564b38861a9a5');
      final List<Device> devices = <Device>[device1, device2, device3];
      final BufferLogger logger = BufferLogger.test();

      // Include different device discoveries:
      // 1. One that throws, to prove matches can return when some succeed
      // and others fail.
      // 2. A device discoverer that succeeds.
      final DeviceManager deviceManager = TestDeviceManager(
        devices,
        deviceDiscoveryOverrides: <DeviceDiscovery>[ThrowingPollingDeviceDiscovery()],
        logger: logger,
      );

      Future<void> expectDevice(String id, List<Device> expected) async {
        expect(await deviceManager.getDevicesById(id), expected);
      }

      await expectDevice('Nexus 5', <Device>[device1]);
      expect(logger.traceText, contains('Ignored error discovering Nexus 5'));
      await expectDevice('0553790', <Device>[device1]);
      expect(logger.traceText, contains('Ignored error discovering 0553790'));
      await expectDevice('Nexus', <Device>[device1, device2]);
      expect(logger.traceText, contains('Ignored error discovering Nexus'));
    });

    testWithoutContext('getDeviceById two exact matches, matches on first', () async {
      final FakeDevice device1 = FakeDevice('Nexus 5', '0553790d0a4e726f');
      final FakeDevice device2 = FakeDevice('Nexus 5', '01abfc49119c410e');
      final List<Device> devices = <Device>[device1, device2];
      final BufferLogger logger = BufferLogger.test();

      final DeviceManager deviceManager = TestDeviceManager(devices, logger: logger);

      Future<void> expectDevice(String id, List<Device> expected) async {
        expect(await deviceManager.getDevicesById(id), expected);
      }

      await expectDevice('Nexus 5', <Device>[device1]);
    });

    testWithoutContext('getAllDevices caches', () async {
      final FakePollingDeviceDiscovery notSupportedDiscoverer = FakePollingDeviceDiscovery();
      final FakePollingDeviceDiscovery supportedDiscoverer = FakePollingDeviceDiscovery(
        requiresExtendedWirelessDeviceDiscovery: true,
      );

      final FakeDevice attachedDevice = FakeDevice('Nexus 5', '0553790d0a4e726f');
      final FakeDevice wirelessDevice = FakeDevice(
        'Wireless device',
        'wireless-device',
        connectionInterface: DeviceConnectionInterface.wireless,
      );

      notSupportedDiscoverer.addDevice(attachedDevice);
      supportedDiscoverer.addDevice(wirelessDevice);

      final TestDeviceManager deviceManager = TestDeviceManager(
        <Device>[],
        logger: BufferLogger.test(),
        deviceDiscoveryOverrides: <DeviceDiscovery>[notSupportedDiscoverer, supportedDiscoverer],
      );
      expect(await deviceManager.getAllDevices(), <Device>[attachedDevice, wirelessDevice]);

      final FakeDevice newAttachedDevice = FakeDevice('Nexus 5X', '01abfc49119c410e');
      notSupportedDiscoverer.addDevice(newAttachedDevice);

      final FakeDevice newWirelessDevice = FakeDevice(
        'New wireless device',
        'new-wireless-device',
        connectionInterface: DeviceConnectionInterface.wireless,
      );
      supportedDiscoverer.addDevice(newWirelessDevice);

      expect(await deviceManager.getAllDevices(), <Device>[attachedDevice, wirelessDevice]);
    });

    testWithoutContext('refreshAllDevices does not cache', () async {
      final FakePollingDeviceDiscovery notSupportedDiscoverer = FakePollingDeviceDiscovery();
      final FakePollingDeviceDiscovery supportedDiscoverer = FakePollingDeviceDiscovery(
        requiresExtendedWirelessDeviceDiscovery: true,
      );

      final FakeDevice attachedDevice = FakeDevice('Nexus 5', '0553790d0a4e726f');
      final FakeDevice wirelessDevice = FakeDevice(
        'Wireless device',
        'wireless-device',
        connectionInterface: DeviceConnectionInterface.wireless,
      );

      notSupportedDiscoverer.addDevice(attachedDevice);
      supportedDiscoverer.addDevice(wirelessDevice);

      final TestDeviceManager deviceManager = TestDeviceManager(
        <Device>[],
        logger: BufferLogger.test(),
        deviceDiscoveryOverrides: <DeviceDiscovery>[notSupportedDiscoverer, supportedDiscoverer],
      );
      expect(await deviceManager.refreshAllDevices(), <Device>[attachedDevice, wirelessDevice]);

      final FakeDevice newAttachedDevice = FakeDevice('Nexus 5X', '01abfc49119c410e');
      notSupportedDiscoverer.addDevice(newAttachedDevice);

      final FakeDevice newWirelessDevice = FakeDevice(
        'New wireless device',
        'new-wireless-device',
        connectionInterface: DeviceConnectionInterface.wireless,
      );
      supportedDiscoverer.addDevice(newWirelessDevice);

      expect(await deviceManager.refreshAllDevices(), <Device>[
        attachedDevice,
        newAttachedDevice,
        wirelessDevice,
        newWirelessDevice,
      ]);
    });

    testWithoutContext(
      'refreshExtendedWirelessDeviceDiscoverers only refreshes discoverers that require extended time',
      () async {
        final FakePollingDeviceDiscovery normalDiscoverer = FakePollingDeviceDiscovery();
        final FakePollingDeviceDiscovery extendedDiscoverer = FakePollingDeviceDiscovery(
          requiresExtendedWirelessDeviceDiscovery: true,
        );

        final FakeDevice attachedDevice = FakeDevice('Nexus 5', '0553790d0a4e726f');
        final FakeDevice wirelessDevice = FakeDevice(
          'Wireless device',
          'wireless-device',
          connectionInterface: DeviceConnectionInterface.wireless,
        );

        normalDiscoverer.addDevice(attachedDevice);
        extendedDiscoverer.addDevice(wirelessDevice);

        final TestDeviceManager deviceManager = TestDeviceManager(
          <Device>[],
          logger: BufferLogger.test(),
          deviceDiscoveryOverrides: <DeviceDiscovery>[normalDiscoverer, extendedDiscoverer],
        );
        await deviceManager.refreshExtendedWirelessDeviceDiscoverers();
        expect(await deviceManager.getAllDevices(), <Device>[attachedDevice, wirelessDevice]);

        final FakeDevice newAttachedDevice = FakeDevice('Nexus 5X', '01abfc49119c410e');
        normalDiscoverer.addDevice(newAttachedDevice);

        final FakeDevice newWirelessDevice = FakeDevice(
          'New wireless device',
          'new-wireless-device',
          connectionInterface: DeviceConnectionInterface.wireless,
        );
        extendedDiscoverer.addDevice(newWirelessDevice);

        await deviceManager.refreshExtendedWirelessDeviceDiscoverers();
        expect(await deviceManager.getAllDevices(), <Device>[
          attachedDevice,
          wirelessDevice,
          newWirelessDevice,
        ]);
      },
    );
  });

  testWithoutContext('PollingDeviceDiscovery startPolling', () {
    FakeAsync().run((FakeAsync time) {
      final FakePollingDeviceDiscovery pollingDeviceDiscovery = FakePollingDeviceDiscovery();
      pollingDeviceDiscovery.startPolling();

      // First check should use the default polling timeout
      // to quickly populate the list.
      expect(pollingDeviceDiscovery.lastPollingTimeout, isNull);

      time.elapse(const Duration(milliseconds: 4001));

      // Subsequent polling should be much longer.
      expect(pollingDeviceDiscovery.lastPollingTimeout, const Duration(seconds: 30));
      pollingDeviceDiscovery.stopPolling();
    });
  });

  group('Filter devices', () {
    final FakeDevice ephemeralOne = FakeDevice('ephemeralOne', 'ephemeralOne');
    final FakeDevice ephemeralTwo = FakeDevice('ephemeralTwo', 'ephemeralTwo');
    final FakeDevice nonEphemeralOne = FakeDevice(
      'nonEphemeralOne',
      'nonEphemeralOne',
      ephemeral: false,
    );
    final FakeDevice nonEphemeralTwo = FakeDevice(
      'nonEphemeralTwo',
      'nonEphemeralTwo',
      ephemeral: false,
    );
    final FakeDevice unsupported = FakeDevice('unsupported', 'unsupported', isSupported: false);
    final FakeDevice unsupportedForProject = FakeDevice(
      'unsupportedForProject',
      'unsupportedForProject',
      isSupportedForProject: false,
    );
    final FakeDevice webDevice = FakeDevice('webby', 'webby')
      ..targetPlatform = Future<TargetPlatform>.value(TargetPlatform.web_javascript);
    final FakeDevice fuchsiaDevice = FakeDevice('fuchsiay', 'fuchsiay')
      ..targetPlatform = Future<TargetPlatform>.value(TargetPlatform.fuchsia_x64);
    final FakeDevice unconnectedDevice = FakeDevice(
      'ephemeralTwo',
      'ephemeralTwo',
      isConnected: false,
    );
    final FakeDevice wirelessDevice = FakeDevice(
      'ephemeralTwo',
      'ephemeralTwo',
      connectionInterface: DeviceConnectionInterface.wireless,
    );

    testUsingContext('chooses ephemeral device', () async {
      final List<Device> devices = <Device>[ephemeralOne, nonEphemeralOne, nonEphemeralTwo];

      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());

      final Device? ephemeralDevice = deviceManager.getSingleEphemeralDevice(devices);

      expect(ephemeralDevice, ephemeralOne);
    }, overrides: <Type, Generator>{FlutterProject: () => FakeFlutterProject()});

    testUsingContext(
      'returns null when multiple non ephemeral devices are found',
      () async {
        final List<Device> devices = <Device>[
          ephemeralOne,
          ephemeralTwo,
          nonEphemeralOne,
          nonEphemeralTwo,
        ];

        final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());

        final Device? ephemeralDevice = deviceManager.getSingleEphemeralDevice(devices);

        expect(ephemeralDevice, isNull);
      },
      overrides: <Type, Generator>{FlutterProject: () => FakeFlutterProject()},
    );

    testUsingContext(
      'return null when hasSpecifiedDeviceId is true',
      () async {
        final List<Device> devices = <Device>[ephemeralOne, nonEphemeralOne, nonEphemeralTwo];

        final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
        deviceManager.specifiedDeviceId = 'device';

        final Device? ephemeralDevice = deviceManager.getSingleEphemeralDevice(devices);

        expect(ephemeralDevice, isNull);
      },
      overrides: <Type, Generator>{FlutterProject: () => FakeFlutterProject()},
    );

    testUsingContext(
      'returns null when no ephemeral devices are found',
      () async {
        final List<Device> devices = <Device>[nonEphemeralOne, nonEphemeralTwo];

        final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());

        final Device? ephemeralDevice = deviceManager.getSingleEphemeralDevice(devices);

        expect(ephemeralDevice, isNull);
      },
      overrides: <Type, Generator>{FlutterProject: () => FakeFlutterProject()},
    );

    testWithoutContext('Unsupported devices listed in all devices', () async {
      final List<Device> devices = <Device>[unsupported, unsupportedForProject];

      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      final List<Device> filtered = await deviceManager.getDevices();

      expect(filtered, <Device>[unsupported, unsupportedForProject]);
    });

    testUsingContext('Removes unsupported devices', () async {
      final List<Device> devices = <Device>[unsupported, unsupportedForProject];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      final List<Device> filtered = await deviceManager.getDevices(
        filter: DeviceDiscoveryFilter(supportFilter: deviceManager.deviceSupportFilter()),
      );

      expect(filtered, <Device>[]);
    });

    testUsingContext(
      'Retains devices unsupported by the project if includeDevicesUnsupportedByProject is true',
      () async {
        final List<Device> devices = <Device>[unsupported, unsupportedForProject];

        final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
        final List<Device> filtered = await deviceManager.getDevices(
          filter: DeviceDiscoveryFilter(
            supportFilter: deviceManager.deviceSupportFilter(
              includeDevicesUnsupportedByProject: true,
            ),
          ),
        );

        expect(filtered, <Device>[unsupportedForProject]);
      },
    );

    testUsingContext('Removes web and fuchsia from --all', () async {
      final List<Device> devices = <Device>[webDevice, fuchsiaDevice];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      deviceManager.specifiedDeviceId = 'all';

      final List<Device> filtered = await deviceManager.getDevices(
        filter: DeviceDiscoveryFilter(supportFilter: deviceManager.deviceSupportFilter()),
      );

      expect(filtered, <Device>[]);
    });

    testUsingContext('Removes devices unsupported by the project from --all', () async {
      final List<Device> devices = <Device>[
        nonEphemeralOne,
        nonEphemeralTwo,
        unsupported,
        unsupportedForProject,
      ];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      deviceManager.specifiedDeviceId = 'all';

      final List<Device> filtered = await deviceManager.getDevices(
        filter: DeviceDiscoveryFilter(supportFilter: deviceManager.deviceSupportFilter()),
      );

      expect(filtered, <Device>[nonEphemeralOne, nonEphemeralTwo]);
    });

    testUsingContext('Returns device with the specified id', () async {
      final List<Device> devices = <Device>[nonEphemeralOne, nonEphemeralTwo];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      deviceManager.specifiedDeviceId = nonEphemeralOne.id;

      final List<Device> filtered = await deviceManager.getDevices(
        filter: DeviceDiscoveryFilter(supportFilter: deviceManager.deviceSupportFilter()),
      );

      expect(filtered, <Device>[nonEphemeralOne]);
    });

    testUsingContext(
      'Returns multiple devices when multiple devices matches the specified id',
      () async {
        final List<Device> devices = <Device>[nonEphemeralOne, nonEphemeralTwo];
        final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
        deviceManager.specifiedDeviceId = 'nonEphemeral'; // This prefix matches both devices

        final List<Device> filtered = await deviceManager.getDevices(
          filter: DeviceDiscoveryFilter(supportFilter: deviceManager.deviceSupportFilter()),
        );

        expect(filtered, <Device>[nonEphemeralOne, nonEphemeralTwo]);
      },
    );

    testUsingContext('Returns empty when device of specified id is not found', () async {
      final List<Device> devices = <Device>[nonEphemeralOne];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      deviceManager.specifiedDeviceId = nonEphemeralTwo.id;

      final List<Device> filtered = await deviceManager.getDevices(
        filter: DeviceDiscoveryFilter(supportFilter: deviceManager.deviceSupportFilter()),
      );

      expect(filtered, <Device>[]);
    });

    testWithoutContext(
      'uses DeviceDiscoverySupportFilter.isDeviceSupportedForProject instead of device.isSupportedForProject',
      () async {
        final List<Device> devices = <Device>[unsupported, unsupportedForProject];
        final TestDeviceManager deviceManager = TestDeviceManager(
          devices,
          logger: BufferLogger.test(),
        );
        final TestDeviceDiscoverySupportFilter supportFilter =
            TestDeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutterOrProject(
              flutterProject: FakeFlutterProject(),
            );
        supportFilter.isAlwaysSupportedForProjectOverride = true;
        final DeviceDiscoveryFilter filter = DeviceDiscoveryFilter(supportFilter: supportFilter);

        final List<Device> filtered = await deviceManager.getDevices(filter: filter);

        expect(filtered, <Device>[unsupportedForProject]);
      },
    );

    testUsingContext('Unconnected devices filtered out by default', () async {
      final List<Device> devices = <Device>[unconnectedDevice];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());

      final List<Device> filtered = await deviceManager.getDevices();

      expect(filtered, <Device>[]);
    });

    testUsingContext('Return unconnected devices when filter allows', () async {
      final List<Device> devices = <Device>[unconnectedDevice];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      final DeviceDiscoveryFilter filter = DeviceDiscoveryFilter(excludeDisconnected: false);

      final List<Device> filtered = await deviceManager.getDevices(filter: filter);

      expect(filtered, <Device>[unconnectedDevice]);
    });

    testUsingContext('Filter to only include wireless devices', () async {
      final List<Device> devices = <Device>[ephemeralOne, wirelessDevice];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      final DeviceDiscoveryFilter filter = DeviceDiscoveryFilter(
        deviceConnectionInterface: DeviceConnectionInterface.wireless,
      );

      final List<Device> filtered = await deviceManager.getDevices(filter: filter);

      expect(filtered, <Device>[wirelessDevice]);
    });

    testUsingContext('Filter to only include attached devices', () async {
      final List<Device> devices = <Device>[ephemeralOne, wirelessDevice];
      final DeviceManager deviceManager = TestDeviceManager(devices, logger: BufferLogger.test());
      final DeviceDiscoveryFilter filter = DeviceDiscoveryFilter(
        deviceConnectionInterface: DeviceConnectionInterface.attached,
      );

      final List<Device> filtered = await deviceManager.getDevices(filter: filter);

      expect(filtered, <Device>[ephemeralOne]);
    });
  });

  group('Simultaneous device discovery', () {
    testWithoutContext(
      'Run getAllDevices and refreshAllDevices at same time with refreshAllDevices finishing last',
      () async {
        FakeAsync().run((FakeAsync time) {
          final FakeDevice device1 = FakeDevice('Nexus 5', '0553790d0a4e726f');
          final FakeDevice device2 = FakeDevice('Nexus 5X', '01abfc49119c410e');

          const Duration timeToGetInitialDevices = Duration(seconds: 1);
          const Duration timeToRefreshDevices = Duration(seconds: 5);
          final List<Device> initialDevices = <Device>[device2];
          final List<Device> refreshDevices = <Device>[device1];

          final TestDeviceManager deviceManager = TestDeviceManager(
            <Device>[],
            logger: BufferLogger.test(),
            fakeDiscoverer: FakePollingDeviceDiscoveryWithTimeout(<List<Device>>[
              initialDevices,
              refreshDevices,
            ], timeout: timeToGetInitialDevices),
          );

          // Expect that the cache is set by getOrSetCache process (1 second timeout)
          // and then later updated by refreshCache process (5 second timeout).
          // Ending with devices from the refreshCache process.
          final Future<List<Device>> refreshCache = deviceManager.refreshAllDevices(
            timeout: timeToRefreshDevices,
          );
          final Future<List<Device>> getOrSetCache = deviceManager.getAllDevices();

          // After 1 second, the getAllDevices should be done
          time.elapse(const Duration(seconds: 1));
          expect(getOrSetCache, completion(<Device>[device2]));
          // double check values in cache are as expected
          Future<List<Device>> getFromCache = deviceManager.getAllDevices();
          expect(getFromCache, completion(<Device>[device2]));

          // After 5 seconds, getOrSetCache should be done
          time.elapse(const Duration(seconds: 5));
          expect(refreshCache, completion(<Device>[device1]));
          // double check values in cache are as expected
          getFromCache = deviceManager.getAllDevices();
          expect(getFromCache, completion(<Device>[device1]));

          time.flushMicrotasks();
        });
      },
    );

    testWithoutContext(
      'Run getAllDevices and refreshAllDevices at same time with refreshAllDevices finishing first',
      () async {
        fakeAsync((FakeAsync async) {
          final FakeDevice device1 = FakeDevice('Nexus 5', '0553790d0a4e726f');
          final FakeDevice device2 = FakeDevice('Nexus 5X', '01abfc49119c410e');

          const Duration timeToGetInitialDevices = Duration(seconds: 5);
          const Duration timeToRefreshDevices = Duration(seconds: 1);
          final List<Device> initialDevices = <Device>[device2];
          final List<Device> refreshDevices = <Device>[device1];

          final TestDeviceManager deviceManager = TestDeviceManager(
            <Device>[],
            logger: BufferLogger.test(),
            fakeDiscoverer: FakePollingDeviceDiscoveryWithTimeout(<List<Device>>[
              initialDevices,
              refreshDevices,
            ], timeout: timeToGetInitialDevices),
          );

          // Expect that the cache is set by refreshCache process (1 second timeout).
          // Then later when getOrSetCache finishes (5 second timeout), it does not update the cache.
          // Ending with devices from the refreshCache process.
          final Future<List<Device>> refreshCache = deviceManager.refreshAllDevices(
            timeout: timeToRefreshDevices,
          );
          final Future<List<Device>> getOrSetCache = deviceManager.getAllDevices();

          // After 1 second, the refreshCache should be done
          async.elapse(const Duration(seconds: 1));
          expect(refreshCache, completion(<Device>[device2]));
          // double check values in cache are as expected
          Future<List<Device>> getFromCache = deviceManager.getAllDevices();
          expect(getFromCache, completion(<Device>[device2]));

          // After 5 seconds, getOrSetCache should be done
          async.elapse(const Duration(seconds: 5));
          expect(getOrSetCache, completion(<Device>[device2]));
          // double check values in cache are as expected
          getFromCache = deviceManager.getAllDevices();
          expect(getFromCache, completion(<Device>[device2]));

          async.flushMicrotasks();
        });
      },
    );

    testWithoutContext('refreshAllDevices twice', () async {
      fakeAsync((FakeAsync async) {
        final FakeDevice device1 = FakeDevice('Nexus 5', '0553790d0a4e726f');
        final FakeDevice device2 = FakeDevice('Nexus 5X', '01abfc49119c410e');

        const Duration timeToFirstRefresh = Duration(seconds: 1);
        const Duration timeToSecondRefresh = Duration(seconds: 5);
        final List<Device> firstRefreshDevices = <Device>[device2];
        final List<Device> secondRefreshDevices = <Device>[device1];

        final TestDeviceManager deviceManager = TestDeviceManager(
          <Device>[],
          logger: BufferLogger.test(),
          fakeDiscoverer: FakePollingDeviceDiscoveryWithTimeout(<List<Device>>[
            firstRefreshDevices,
            secondRefreshDevices,
          ]),
        );

        // Expect that the cache is updated by each refresh in order of completion.
        final Future<List<Device>> firstRefresh = deviceManager.refreshAllDevices(
          timeout: timeToFirstRefresh,
        );
        final Future<List<Device>> secondRefresh = deviceManager.refreshAllDevices(
          timeout: timeToSecondRefresh,
        );

        // After 1 second, the firstRefresh should be done
        async.elapse(const Duration(seconds: 1));
        expect(firstRefresh, completion(<Device>[device2]));
        // double check values in cache are as expected
        Future<List<Device>> getFromCache = deviceManager.getAllDevices();
        expect(getFromCache, completion(<Device>[device2]));

        // After 5 seconds, secondRefresh should be done
        async.elapse(const Duration(seconds: 5));
        expect(secondRefresh, completion(<Device>[device1]));
        // double check values in cache are as expected
        getFromCache = deviceManager.getAllDevices();
        expect(getFromCache, completion(<Device>[device1]));

        async.flushMicrotasks();
      });
    });
  });

  group('JSON encode devices', () {
    testWithoutContext('Consistency of JSON representation', () async {
      expect(
        // This tests that fakeDevices is a list of tuples where "second" is the
        // correct JSON representation of the "first". Actual values are irrelevant
        await Future.wait(fakeDevices.map((FakeDeviceJsonData d) => d.dev.toJson())),
        fakeDevices.map((FakeDeviceJsonData d) => d.json),
      );
    });
  });

  testWithoutContext(
    'computeDartVmFlags handles various combinations of Dart VM flags and null_assertions',
    () {
      expect(computeDartVmFlags(DebuggingOptions.enabled(BuildInfo.debug)), '');
      expect(
        computeDartVmFlags(DebuggingOptions.enabled(BuildInfo.debug, dartFlags: '--foo')),
        '--foo',
      );
      expect(
        computeDartVmFlags(DebuggingOptions.enabled(BuildInfo.debug, nullAssertions: true)),
        '--null_assertions',
      );
      expect(
        computeDartVmFlags(
          DebuggingOptions.enabled(BuildInfo.debug, dartFlags: '--foo', nullAssertions: true),
        ),
        '--foo,--null_assertions',
      );
    },
  );

  group('JSON encode DebuggingOptions', () {
    testWithoutContext('can preserve the original options', () {
      final DebuggingOptions original = DebuggingOptions.enabled(
        BuildInfo.debug,
        startPaused: true,
        disableServiceAuthCodes: true,
        enableDds: false,
        dartEntrypointArgs: <String>['a', 'b'],
        dartFlags: 'c',
        deviceVmServicePort: 1234,
        enableImpeller: ImpellerStatus.enabled,
        enableDartProfiling: false,
        enableEmbedderApi: true,
      );
      final String jsonString = json.encode(original.toJson());
      final Map<String, dynamic> decoded = castStringKeyedMap(json.decode(jsonString))!;
      final DebuggingOptions deserialized = DebuggingOptions.fromJson(decoded, BuildInfo.debug);
      expect(deserialized.startPaused, original.startPaused);
      expect(deserialized.disableServiceAuthCodes, original.disableServiceAuthCodes);
      expect(deserialized.enableDds, original.enableDds);
      expect(deserialized.dartEntrypointArgs, original.dartEntrypointArgs);
      expect(deserialized.dartFlags, original.dartFlags);
      expect(deserialized.deviceVmServicePort, original.deviceVmServicePort);
      expect(deserialized.enableImpeller, original.enableImpeller);
      expect(deserialized.enableDartProfiling, original.enableDartProfiling);
      expect(deserialized.enableEmbedderApi, original.enableEmbedderApi);
    });
  });

  group('Get iOS launch arguments from DebuggingOptions', () {
    testWithoutContext(
      'Get launch arguments for physical device with debugging enabled with all launch arguments',
      () {
        final DebuggingOptions original = DebuggingOptions.enabled(
          BuildInfo.debug,
          startPaused: true,
          disableServiceAuthCodes: true,
          disablePortPublication: true,
          dartFlags: '--foo',
          useTestFonts: true,
          enableSoftwareRendering: true,
          skiaDeterministicRendering: true,
          traceSkia: true,
          traceAllowlist: 'foo',
          traceSkiaAllowlist: 'skia.a,skia.b',
          traceSystrace: true,
          traceToFile: 'path/to/trace.binpb',
          endlessTraceBuffer: true,
          dumpSkpOnShaderCompilation: true,
          cacheSkSL: true,
          purgePersistentCache: true,
          verboseSystemLogs: true,
          enableImpeller: ImpellerStatus.disabled,
          nullAssertions: true,
          deviceVmServicePort: 0,
          hostVmServicePort: 1,
        );

        final List<String> launchArguments = original.getIOSLaunchArguments(
          EnvironmentType.physical,
          '/test',
          <String, dynamic>{'trace-startup': true},
        );

        expect(
          launchArguments.join(' '),
          <String>[
            '--enable-dart-profiling',
            '--disable-service-auth-codes',
            '--disable-vm-service-publication',
            '--start-paused',
            '--dart-flags="--foo,--null_assertions"',
            '--use-test-fonts',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-software-rendering',
            '--trace-systrace',
            '--trace-to-file="path/to/trace.binpb"',
            '--skia-deterministic-rendering',
            '--trace-skia',
            '--trace-allowlist="foo"',
            '--trace-skia-allowlist="skia.a,skia.b"',
            '--endless-trace-buffer',
            '--dump-skp-on-shader-compilation',
            '--verbose-logging',
            '--cache-sksl',
            '--purge-persistent-cache',
            '--route=/test',
            '--trace-startup',
            '--enable-impeller=false',
            '--vm-service-port=0',
          ].join(' '),
        );
      },
    );

    testWithoutContext(
      'Get launch arguments for physical device with debugging enabled with no launch arguments',
      () {
        final DebuggingOptions original = DebuggingOptions.enabled(BuildInfo.debug);

        final List<String> launchArguments = original.getIOSLaunchArguments(
          EnvironmentType.physical,
          null,
          <String, Object?>{},
        );

        expect(
          launchArguments.join(' '),
          <String>[
            '--enable-dart-profiling',
            '--enable-checked-mode',
            '--verify-entry-points',
          ].join(' '),
        );
      },
    );

    testWithoutContext(
      'Get launch arguments for physical CoreDevice with debugging enabled with no launch arguments',
      () {
        final DebuggingOptions original = DebuggingOptions.enabled(BuildInfo.debug);

        final List<String> launchArguments = original.getIOSLaunchArguments(
          EnvironmentType.physical,
          null,
          <String, Object?>{},
          isCoreDevice: true,
        );

        expect(launchArguments.join(' '), <String>['--enable-dart-profiling'].join(' '));
      },
    );

    testWithoutContext('Get launch arguments for physical device with iPv4 network connection', () {
      final DebuggingOptions original = DebuggingOptions.enabled(BuildInfo.debug);

      final List<String> launchArguments = original.getIOSLaunchArguments(
        EnvironmentType.physical,
        null,
        <String, Object?>{},
        interfaceType: DeviceConnectionInterface.wireless,
      );

      expect(
        launchArguments.join(' '),
        <String>[
          '--enable-dart-profiling',
          '--enable-checked-mode',
          '--verify-entry-points',
          '--vm-service-host=0.0.0.0',
        ].join(' '),
      );
    });

    testWithoutContext('Get launch arguments for physical device with iPv6 network connection', () {
      final DebuggingOptions original = DebuggingOptions.enabled(BuildInfo.debug, ipv6: true);

      final List<String> launchArguments = original.getIOSLaunchArguments(
        EnvironmentType.physical,
        null,
        <String, Object?>{},
        interfaceType: DeviceConnectionInterface.wireless,
      );

      expect(
        launchArguments.join(' '),
        <String>[
          '--enable-dart-profiling',
          '--enable-checked-mode',
          '--verify-entry-points',
          '--vm-service-host=::0',
        ].join(' '),
      );
    });

    testWithoutContext(
      'Get launch arguments for physical device with debugging disabled with available launch arguments',
      () {
        final DebuggingOptions original = DebuggingOptions.disabled(
          BuildInfo.debug,
          traceAllowlist: 'foo',
          cacheSkSL: true,
          enableImpeller: ImpellerStatus.disabled,
        );

        final List<String> launchArguments = original.getIOSLaunchArguments(
          EnvironmentType.physical,
          '/test',
          <String, dynamic>{'trace-startup': true},
        );

        expect(
          launchArguments.join(' '),
          <String>[
            '--enable-dart-profiling',
            '--trace-allowlist="foo"',
            '--cache-sksl',
            '--route=/test',
            '--trace-startup',
            '--enable-impeller=false',
          ].join(' '),
        );
      },
    );

    testWithoutContext(
      'Get launch arguments for simulator device with debugging enabled with all launch arguments',
      () {
        final DebuggingOptions original = DebuggingOptions.enabled(
          BuildInfo.debug,
          startPaused: true,
          disableServiceAuthCodes: true,
          disablePortPublication: true,
          dartFlags: '--foo',
          useTestFonts: true,
          enableSoftwareRendering: true,
          skiaDeterministicRendering: true,
          traceSkia: true,
          traceAllowlist: 'foo',
          traceSkiaAllowlist: 'skia.a,skia.b',
          traceSystrace: true,
          traceToFile: 'path/to/trace.binpb',
          endlessTraceBuffer: true,
          dumpSkpOnShaderCompilation: true,
          cacheSkSL: true,
          purgePersistentCache: true,
          verboseSystemLogs: true,
          enableImpeller: ImpellerStatus.disabled,
          nullAssertions: true,
          deviceVmServicePort: 0,
          hostVmServicePort: 1,
        );

        final List<String> launchArguments = original.getIOSLaunchArguments(
          EnvironmentType.simulator,
          '/test',
          <String, dynamic>{'trace-startup': true},
        );

        expect(
          launchArguments.join(' '),
          <String>[
            '--enable-dart-profiling',
            '--disable-service-auth-codes',
            '--disable-vm-service-publication',
            '--start-paused',
            '--dart-flags=--foo,--null_assertions',
            '--use-test-fonts',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-software-rendering',
            '--trace-systrace',
            '--trace-to-file="path/to/trace.binpb"',
            '--skia-deterministic-rendering',
            '--trace-skia',
            '--trace-allowlist="foo"',
            '--trace-skia-allowlist="skia.a,skia.b"',
            '--endless-trace-buffer',
            '--dump-skp-on-shader-compilation',
            '--verbose-logging',
            '--cache-sksl',
            '--purge-persistent-cache',
            '--route=/test',
            '--trace-startup',
            '--enable-impeller=false',
            '--vm-service-port=1',
          ].join(' '),
        );
      },
    );

    testWithoutContext(
      'Get launch arguments for simulator device with debugging enabled with no launch arguments',
      () {
        final DebuggingOptions original = DebuggingOptions.enabled(BuildInfo.debug);

        final List<String> launchArguments = original.getIOSLaunchArguments(
          EnvironmentType.simulator,
          null,
          <String, Object?>{},
        );

        expect(
          launchArguments.join(' '),
          <String>[
            '--enable-dart-profiling',
            '--enable-checked-mode',
            '--verify-entry-points',
          ].join(' '),
        );
      },
    );

    testWithoutContext('No --enable-dart-profiling flag when option is false', () {
      final DebuggingOptions original = DebuggingOptions.enabled(
        BuildInfo.debug,
        enableDartProfiling: false,
      );

      final List<String> launchArguments = original.getIOSLaunchArguments(
        EnvironmentType.physical,
        null,
        <String, Object?>{},
      );

      expect(
        launchArguments.join(' '),
        <String>['--enable-checked-mode', '--verify-entry-points'].join(' '),
      );
    });
  });

  group('PollingDeviceDiscovery', () {
    final FakeDevice device1 = FakeDevice('Nexus 5', '0553790d0a4e726f');

    testWithoutContext('initial call to devices returns the correct list', () async {
      final List<Device> deviceList = <Device>[device1];
      final TestPollingDeviceDiscovery testDeviceDiscovery = TestPollingDeviceDiscovery(deviceList);

      // Call `onAdded` to make sure that calling `onAdded` does not affect the
      // result of `devices()`.
      final List<Device> addedDevice = <Device>[];
      final List<Device> removedDevice = <Device>[];
      testDeviceDiscovery.onAdded.listen(addedDevice.add);
      testDeviceDiscovery.onRemoved.listen(removedDevice.add);

      final List<Device> devices = await testDeviceDiscovery.devices();
      expect(devices.length, 1);
      expect(devices.first.id, device1.id);
    });

    testWithoutContext('call to devices triggers onAdded', () async {
      final List<Device> deviceList = <Device>[device1];
      final TestPollingDeviceDiscovery testDeviceDiscovery = TestPollingDeviceDiscovery(deviceList);

      // Call `onAdded` to make sure that calling `onAdded` does not affect the
      // result of `devices()`.
      final List<Device> addedDevice = <Device>[];
      final List<Device> removedDevice = <Device>[];
      testDeviceDiscovery.onAdded.listen(addedDevice.add);
      testDeviceDiscovery.onRemoved.listen(removedDevice.add);

      final List<Device> devices = await testDeviceDiscovery.devices();
      expect(devices.length, 1);
      expect(devices.first.id, device1.id);

      await pumpEventQueue();

      expect(addedDevice.length, 1);
      expect(addedDevice.first.id, device1.id);
    });
  });
}

class TestDeviceManager extends DeviceManager {
  TestDeviceManager(
    List<Device> allDevices, {
    List<DeviceDiscovery>? deviceDiscoveryOverrides,
    required super.logger,
    String? wellKnownId,
    FakePollingDeviceDiscovery? fakeDiscoverer,
  }) : _fakeDeviceDiscoverer = fakeDiscoverer ?? FakePollingDeviceDiscovery(),
       _deviceDiscoverers = <DeviceDiscovery>[],
       super() {
    if (wellKnownId != null) {
      _fakeDeviceDiscoverer.wellKnownIds.add(wellKnownId);
    }
    _deviceDiscoverers.add(_fakeDeviceDiscoverer);
    if (deviceDiscoveryOverrides != null) {
      _deviceDiscoverers.addAll(deviceDiscoveryOverrides);
    }
    resetDevices(allDevices);
  }
  @override
  List<DeviceDiscovery> get deviceDiscoverers => _deviceDiscoverers;
  final List<DeviceDiscovery> _deviceDiscoverers;
  final FakePollingDeviceDiscovery _fakeDeviceDiscoverer;

  void resetDevices(List<Device> allDevices) {
    _fakeDeviceDiscoverer.setDevices(allDevices);
  }
}

class TestDeviceDiscoverySupportFilter extends DeviceDiscoverySupportFilter {
  TestDeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutterOrProject({
    required super.flutterProject,
  }) : super.excludeDevicesUnsupportedByFlutterOrProject();

  bool? isAlwaysSupportedForProjectOverride;

  @override
  bool isDeviceSupportedForProject(Device device) {
    return isAlwaysSupportedForProjectOverride ?? super.isDeviceSupportedForProject(device);
  }
}

class FakePollingDeviceDiscoveryWithTimeout extends FakePollingDeviceDiscovery {
  FakePollingDeviceDiscoveryWithTimeout(this._devices, {Duration? timeout})
    : defaultTimeout = timeout ?? const Duration(seconds: 2);

  final List<List<Device>> _devices;
  int index = 0;

  Duration defaultTimeout;
  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    timeout ??= defaultTimeout;
    await Future<void>.delayed(timeout);
    final List<Device> results = _devices[index];
    index += 1;
    return results;
  }
}

class FakeFlutterProject extends Fake implements FlutterProject {}

class LongPollingDeviceDiscovery extends PollingDeviceDiscovery {
  LongPollingDeviceDiscovery() : super('forever');

  final Completer<List<Device>> _completer = Completer<List<Device>>();

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    return _completer.future;
  }

  @override
  Future<void> stopPolling() async {
    _completer.complete(<Device>[]);
  }

  @override
  Future<void> dispose() async {
    _completer.complete(<Device>[]);
  }

  @override
  bool get supportsPlatform => true;

  @override
  bool get canListAnything => true;

  @override
  final List<String> wellKnownIds = <String>[];
}

class ThrowingPollingDeviceDiscovery extends PollingDeviceDiscovery {
  ThrowingPollingDeviceDiscovery() : super('throw');

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    throw const ProcessException('fake-discovery', <String>[]);
  }

  @override
  bool get supportsPlatform => true;

  @override
  bool get canListAnything => true;

  @override
  List<String> get wellKnownIds => <String>[];
}

class TestPollingDeviceDiscovery extends PollingDeviceDiscovery {
  TestPollingDeviceDiscovery(this._devices) : super('test');

  final List<Device> _devices;

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    return _devices;
  }

  @override
  bool get supportsPlatform => true;

  @override
  bool get canListAnything => true;

  @override
  List<String> get wellKnownIds => <String>[];
}
