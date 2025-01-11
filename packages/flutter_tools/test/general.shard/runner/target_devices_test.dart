// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/doctor.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/ios/ios_workflow.dart';
import 'package:flutter_tools/src/macos/xcdevice.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/runner/target_devices.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';

void main() {
  testWithoutContext(
    'Ensure factory returns TargetDevicesWithExtendedWirelessDeviceDiscovery on MacOS',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final Platform platform = FakePlatform(operatingSystem: 'macos');
      final TestDeviceManager deviceManager = TestDeviceManager(logger: logger, platform: platform);

      final TargetDevices targetDevices = TargetDevices(
        platform: platform,
        deviceManager: deviceManager,
        logger: logger,
      );

      expect(targetDevices is TargetDevicesWithExtendedWirelessDeviceDiscovery, true);
    },
  );

  testWithoutContext('Ensure factory returns default when not on MacOS', () async {
    final BufferLogger logger = BufferLogger.test();
    final Platform platform = FakePlatform();
    final TestDeviceManager deviceManager = TestDeviceManager(logger: logger, platform: platform);

    final TargetDevices targetDevices = TargetDevices(
      platform: platform,
      deviceManager: deviceManager,
      logger: logger,
    );

    expect(targetDevices is TargetDevicesWithExtendedWirelessDeviceDiscovery, false);
  });

  group('findAllTargetDevices on non-MacOS platform', () {
    late Platform platform;

    final FakeDevice attachedAndroidDevice1 = FakeDevice(deviceName: 'target-device-1');
    final FakeDevice attachedAndroidDevice2 = FakeDevice(deviceName: 'target-device-2');
    final FakeDevice attachedUnsupportedAndroidDevice = FakeDevice(
      deviceName: 'target-device-3',
      deviceSupported: false,
    );
    final FakeDevice attachedUnsupportedForProjectAndroidDevice = FakeDevice(
      deviceName: 'target-device-4',
      deviceSupportForProject: false,
    );

    final FakeDevice wirelessAndroidDevice1 = FakeDevice.wireless(deviceName: 'target-device-5');
    final FakeDevice wirelessAndroidDevice2 = FakeDevice.wireless(deviceName: 'target-device-6');
    final FakeDevice wirelessUnsupportedAndroidDevice = FakeDevice.wireless(
      deviceName: 'target-device-7',
      deviceSupported: false,
    );
    final FakeDevice wirelessUnsupportedForProjectAndroidDevice = FakeDevice.wireless(
      deviceName: 'target-device-8',
      deviceSupportForProject: false,
    );

    final FakeDevice nonEphemeralDevice = FakeDevice(
      deviceName: 'target-device-9',
      ephemeral: false,
    );
    final FakeDevice fuchsiaDevice = FakeDevice.fuchsia(deviceName: 'target-device-10');

    final FakeDevice exactMatchAndroidDevice = FakeDevice(deviceName: 'target-device');
    final FakeDevice exactMatchWirelessAndroidDevice = FakeDevice.wireless(
      deviceName: 'target-device',
    );
    final FakeDevice exactMatchAttachedUnsupportedAndroidDevice = FakeDevice(
      deviceName: 'target-device',
      deviceSupported: false,
    );
    final FakeDevice exactMatchUnsupportedByProjectDevice = FakeDevice(
      deviceName: 'target-device',
      deviceSupportForProject: false,
    );

    setUp(() {
      platform = FakePlatform();
    });

    group('when cannot launch anything', () {
      late BufferLogger logger;
      late FakeDoctor doctor;

      setUp(() {
        logger = BufferLogger.test();
        doctor = FakeDoctor(canLaunchAnything: false);
      });

      testUsingContext('does not search for devices', () async {
        final TestDeviceManager deviceManager = TestDeviceManager(
          logger: logger,
          platform: platform,
        );
        deviceManager.androidDiscoverer.deviceList = <Device>[attachedAndroidDevice1];

        final TargetDevices targetDevices = TargetDevices(
          platform: platform,
          deviceManager: deviceManager,
          logger: logger,
        );
        final List<Device>? devices = await targetDevices.findAllTargetDevices();

        expect(
          logger.errorText,
          equals('''
Unable to locate a development device; please run 'flutter doctor' for information about installing additional components.
'''),
        );
        expect(devices, isNull);
        expect(deviceManager.androidDiscoverer.devicesCalled, 0);
        expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
      }, overrides: <Type, Generator>{Doctor: () => doctor});
    });

    testUsingContext('ensure refresh when deviceDiscoveryTimeout is provided', () async {
      final BufferLogger logger = BufferLogger.test();
      final TestDeviceManager deviceManager = TestDeviceManager(logger: logger, platform: platform);
      deviceManager.androidDiscoverer.deviceList = <Device>[attachedAndroidDevice1];
      deviceManager.androidDiscoverer.refreshDeviceList = <Device>[
        attachedAndroidDevice1,
        wirelessAndroidDevice1,
      ];
      deviceManager.hasSpecifiedAllDevices = true;

      final TargetDevices targetDevices = TargetDevices(
        platform: platform,
        deviceManager: deviceManager,
        logger: logger,
      );
      final List<Device>? devices = await targetDevices.findAllTargetDevices(
        deviceDiscoveryTimeout: const Duration(seconds: 2),
      );

      expect(logger.statusText, equals(''));
      expect(devices, <Device>[attachedAndroidDevice1, wirelessAndroidDevice1]);
      expect(deviceManager.androidDiscoverer.devicesCalled, 2);
      expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 1);
      expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
    });

    testUsingContext(
      'ensure unsupported for projects are included when includeDevicesUnsupportedByProject is true',
      () async {
        final BufferLogger logger = BufferLogger.test();
        final TestDeviceManager deviceManager = TestDeviceManager(
          logger: logger,
          platform: platform,
        );
        deviceManager.androidDiscoverer.deviceList = <Device>[
          attachedUnsupportedAndroidDevice,
          attachedUnsupportedForProjectAndroidDevice,
        ];

        final TargetDevices targetDevices = TargetDevices(
          platform: platform,
          deviceManager: deviceManager,
          logger: logger,
        );
        final List<Device>? devices = await targetDevices.findAllTargetDevices(
          includeDevicesUnsupportedByProject: true,
        );

        expect(logger.statusText, equals(''));
        expect(devices, <Device>[attachedUnsupportedForProjectAndroidDevice]);
        expect(deviceManager.androidDiscoverer.devicesCalled, 2);
        expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
        expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
      },
    );

    group('finds no devices', () {
      late BufferLogger logger;
      late TestDeviceManager deviceManager;
      late TargetDevices targetDevices;

      setUp(() {
        logger = BufferLogger.test();
        deviceManager = TestDeviceManager(logger: logger, platform: platform);
        targetDevices = TargetDevices(
          platform: platform,
          deviceManager: deviceManager,
          logger: logger,
        );
      });

      group('with device not specified', () {
        testUsingContext('when no devices', () async {
          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No supported devices connected.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.androidDiscoverer.devicesCalled, 3);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext('when device is unsupported by flutter or project', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[
            attachedUnsupportedAndroidDevice,
            attachedUnsupportedForProjectAndroidDevice,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No supported devices connected.

The following devices were found, but are not supported by this project:
target-device-3 (mobile) • xxx • android • Android 10 (unsupported)
target-device-4 (mobile) • xxx • android • Android 10
If you would like your app to run on android, consider running `flutter create .` to generate projects for these platforms.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.androidDiscoverer.devicesCalled, 3);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        group('when deviceConnectionInterface does not match', () {
          testUsingContext('filter of wireless', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[attachedAndroidDevice1];

            final TargetDevices targetDevices = TargetDevices(
              platform: platform,
              deviceManager: deviceManager,
              logger: logger,
              deviceConnectionInterface: DeviceConnectionInterface.wireless,
            );
            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No supported devices connected.
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 2);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          });

          testUsingContext('filter of attached', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[wirelessAndroidDevice1];

            final TargetDevices targetDevices = TargetDevices(
              platform: platform,
              deviceManager: deviceManager,
              logger: logger,
              deviceConnectionInterface: DeviceConnectionInterface.attached,
            );
            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No supported devices connected.
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 2);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          });
        });
      });

      group('with hasSpecifiedDeviceId', () {
        setUp(() {
          deviceManager.specifiedDeviceId = 'target-device';
        });

        testUsingContext('when no devices', () async {
          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No supported devices found with name or id matching 'target-device'.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.androidDiscoverer.devicesCalled, 4);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext('when no devices match', () async {
          final FakeDevice device1 = FakeDevice(deviceName: 'no-match-1');
          final FakeDevice device2 = FakeDevice.wireless(deviceName: 'no-match-2');
          deviceManager.androidDiscoverer.deviceList = <Device>[device1, device2];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No supported devices found with name or id matching 'target-device'.

The following devices were found:
no-match-1 (mobile)            • xxx • android • Android 10
no-match-2 (wireless) (mobile) • xxx • android • Android 10
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.androidDiscoverer.devicesCalled, 4);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext('when matching device is unsupported by flutter', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[
            exactMatchAttachedUnsupportedAndroidDevice,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No supported devices found with name or id matching 'target-device'.

The following devices were found:
target-device (mobile) • xxx • android • Android 10 (unsupported)
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.androidDiscoverer.devicesCalled, 4);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        group('when deviceConnectionInterface does not match', () {
          testUsingContext('filter of wireless', () async {
            final FakeDevice device1 = FakeDevice(deviceName: 'not-a-match');
            final FakeDevice device2 = FakeDevice.wireless(deviceName: 'not-a-match-2');
            deviceManager.androidDiscoverer.deviceList = <Device>[
              exactMatchAndroidDevice,
              device1,
              device2,
            ];

            final TargetDevices targetDevices = TargetDevices(
              platform: platform,
              deviceManager: deviceManager,
              logger: logger,
              deviceConnectionInterface: DeviceConnectionInterface.wireless,
            );
            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No supported devices found with name or id matching 'target-device'.

The following devices were found:
not-a-match-2 (wireless) (mobile) • xxx • android • Android 10
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 3);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          });

          testUsingContext('filter of attached', () async {
            final FakeDevice device1 = FakeDevice(deviceName: 'not-a-match');
            final FakeDevice device2 = FakeDevice.wireless(deviceName: 'not-a-match-2');
            deviceManager.androidDiscoverer.deviceList = <Device>[
              exactMatchWirelessAndroidDevice,
              device1,
              device2,
            ];

            final TargetDevices targetDevices = TargetDevices(
              platform: platform,
              deviceManager: deviceManager,
              logger: logger,
              deviceConnectionInterface: DeviceConnectionInterface.attached,
            );
            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No supported devices found with name or id matching 'target-device'.

The following devices were found:
not-a-match (mobile) • xxx • android • Android 10
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 3);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          });
        });
      });

      group('with hasSpecifiedAllDevices', () {
        setUp(() {
          deviceManager.hasSpecifiedAllDevices = true;
        });

        testUsingContext('when no devices', () async {
          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.androidDiscoverer.devicesCalled, 3);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext(
          'when devices are either unsupported by flutter or project or all',
          () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              attachedUnsupportedAndroidDevice,
              attachedUnsupportedForProjectAndroidDevice,
            ];
            deviceManager.otherDiscoverer.deviceList = <Device>[fuchsiaDevice];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No devices found.

The following devices were found, but are not supported by this project:
target-device-3 (mobile)  • xxx • android       • Android 10 (unsupported)
target-device-4 (mobile)  • xxx • android       • Android 10
target-device-10 (mobile) • xxx • fuchsia-arm64 • tester
If you would like your app to run on android or fuchsia, consider running `flutter create .` to generate projects for these platforms.
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 3);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          },
        );
      });
    });

    group('finds single device', () {
      late BufferLogger logger;
      late TestDeviceManager deviceManager;
      late TargetDevices targetDevices;

      setUp(() {
        logger = BufferLogger.test();
        deviceManager = TestDeviceManager(logger: logger, platform: platform);
        targetDevices = TargetDevices(
          platform: platform,
          deviceManager: deviceManager,
          logger: logger,
        );
      });

      group('with device not specified', () {
        testUsingContext('when single attached device', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[attachedAndroidDevice1];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[attachedAndroidDevice1]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 2);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext('when single wireless device', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[wirelessAndroidDevice1];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[wirelessAndroidDevice1]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 2);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext('when multiple but only one ephemeral', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[
            nonEphemeralDevice,
            wirelessAndroidDevice1,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[wirelessAndroidDevice1]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 2);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });
      });

      group('with hasSpecifiedDeviceId', () {
        setUp(() {
          deviceManager.specifiedDeviceId = 'target-device';
        });

        testUsingContext('when multiple matches but first is unsupported by flutter', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[
            exactMatchAttachedUnsupportedAndroidDevice,
            exactMatchAndroidDevice,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[exactMatchAndroidDevice]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 1);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext('when matching device is unsupported by project', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[
            exactMatchUnsupportedByProjectDevice,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[exactMatchUnsupportedByProjectDevice]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 1);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext('when matching attached device', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[exactMatchAndroidDevice];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[exactMatchAndroidDevice]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 1);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext('when matching wireless device', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[exactMatchWirelessAndroidDevice];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[exactMatchWirelessAndroidDevice]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 1);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });

        testUsingContext(
          'when exact matching an attached device and partial matching a wireless device',
          () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              exactMatchAndroidDevice,
              wirelessAndroidDevice1,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(logger.statusText, equals(''));
            expect(devices, <Device>[exactMatchAndroidDevice]);
            expect(deviceManager.androidDiscoverer.devicesCalled, 1);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          },
        );
      });

      group('with hasSpecifiedAllDevices', () {
        setUp(() {
          deviceManager.hasSpecifiedAllDevices = true;
        });

        testUsingContext('when only one device', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[attachedAndroidDevice1];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[attachedAndroidDevice1]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 2);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });
      });
    });

    group('finds multiple devices', () {
      late BufferLogger logger;
      late TestDeviceManager deviceManager;
      late TargetDevices targetDevices;

      setUp(() {
        logger = BufferLogger.test();
        deviceManager = TestDeviceManager(logger: logger, platform: platform);
        targetDevices = TargetDevices(
          platform: platform,
          deviceManager: deviceManager,
          logger: logger,
        );
      });

      group('with device not specified', () {
        group('with stdinHasTerminal', () {
          late FakeTerminal terminal;

          setUp(() {
            terminal = FakeTerminal();
          });

          testUsingContext(
            'including attached, wireless, unsupported devices',
            () async {
              deviceManager.androidDiscoverer.deviceList = <Device>[
                attachedAndroidDevice1,
                attachedUnsupportedAndroidDevice,
                attachedUnsupportedForProjectAndroidDevice,
                wirelessAndroidDevice1,
                wirelessUnsupportedAndroidDevice,
                wirelessUnsupportedForProjectAndroidDevice,
              ];
              terminal.setPrompt(<String>['1', '2', 'q', 'Q'], '2');

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Connected devices:
target-device-1 (mobile) • xxx • android • Android 10

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • android • Android 10

[1]: target-device-1 (xxx)
[2]: target-device-5 (wireless) (xxx)
'''),
              );
              expect(devices, <Device>[wirelessAndroidDevice1]);
              expect(deviceManager.androidDiscoverer.devicesCalled, 2);
              expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
              expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext('including only attached devices', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              attachedAndroidDevice1,
              attachedAndroidDevice2,
            ];
            terminal.setPrompt(<String>['1', '2', 'q', 'Q'], '1');

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Connected devices:
target-device-1 (mobile) • xxx • android • Android 10
target-device-2 (mobile) • xxx • android • Android 10
[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
'''),
            );
            expect(devices, <Device>[attachedAndroidDevice1]);
            expect(deviceManager.androidDiscoverer.devicesCalled, 2);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext('including only wireless devices', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              wirelessAndroidDevice1,
              wirelessAndroidDevice2,
            ];
            terminal.setPrompt(<String>['1', '2', 'q', 'Q'], '1');

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Connected devices:

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • android • Android 10
target-device-6 (wireless) (mobile) • xxx • android • Android 10

[1]: target-device-5 (wireless) (xxx)
[2]: target-device-6 (wireless) (xxx)
'''),
            );
            expect(devices, <Device>[wirelessAndroidDevice1]);
            expect(deviceManager.androidDiscoverer.devicesCalled, 2);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});
        });

        group('without stdinHasTerminal', () {
          late FakeTerminal terminal;

          setUp(() {
            terminal = FakeTerminal(stdinHasTerminal: false);
          });

          testUsingContext(
            'including attached, wireless, unsupported devices',
            () async {
              deviceManager.androidDiscoverer.deviceList = <Device>[
                attachedAndroidDevice1,
                attachedUnsupportedAndroidDevice,
                attachedUnsupportedForProjectAndroidDevice,
                wirelessAndroidDevice1,
                wirelessUnsupportedAndroidDevice,
                wirelessUnsupportedForProjectAndroidDevice,
              ];

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
More than one device connected; please specify a device with the '-d <deviceId>' flag, or use '-d all' to act on all devices.

target-device-1 (mobile) • xxx • android • Android 10
target-device-4 (mobile) • xxx • android • Android 10

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • android • Android 10
target-device-8 (wireless) (mobile) • xxx • android • Android 10
'''),
              );
              expect(devices, isNull);
              expect(deviceManager.androidDiscoverer.devicesCalled, 4);
              expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
              expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext('including only attached devices', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              attachedAndroidDevice1,
              attachedAndroidDevice2,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
More than one device connected; please specify a device with the '-d <deviceId>' flag, or use '-d all' to act on all devices.

target-device-1 (mobile) • xxx • android • Android 10
target-device-2 (mobile) • xxx • android • Android 10
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 4);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext('including only wireless devices', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              wirelessAndroidDevice1,
              wirelessAndroidDevice2,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
More than one device connected; please specify a device with the '-d <deviceId>' flag, or use '-d all' to act on all devices.

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • android • Android 10
target-device-6 (wireless) (mobile) • xxx • android • Android 10
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 4);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});
        });
      });

      group('with hasSpecifiedDeviceId', () {
        setUp(() {
          deviceManager.specifiedDeviceId = 'target-device';
        });

        group('with stdinHasTerminal', () {
          late FakeTerminal terminal;

          setUp(() {
            terminal = FakeTerminal();
          });

          testUsingContext(
            'including attached, wireless, unsupported devices',
            () async {
              deviceManager.androidDiscoverer.deviceList = <Device>[
                attachedAndroidDevice1,
                attachedUnsupportedAndroidDevice,
                attachedUnsupportedForProjectAndroidDevice,
                wirelessAndroidDevice1,
                wirelessUnsupportedAndroidDevice,
                wirelessUnsupportedForProjectAndroidDevice,
              ];
              terminal.setPrompt(<String>['1', '2', '3', '4', 'q', 'Q'], '2');

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Found 4 devices with name or id matching target-device:
target-device-1 (mobile) • xxx • android • Android 10
target-device-4 (mobile) • xxx • android • Android 10

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • android • Android 10
target-device-8 (wireless) (mobile) • xxx • android • Android 10

[1]: target-device-1 (xxx)
[2]: target-device-4 (xxx)
[3]: target-device-5 (wireless) (xxx)
[4]: target-device-8 (wireless) (xxx)
'''),
              );
              expect(devices, <Device>[attachedUnsupportedForProjectAndroidDevice]);
              expect(deviceManager.androidDiscoverer.devicesCalled, 3);
              expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
              expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext('including only attached devices', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              attachedAndroidDevice1,
              attachedAndroidDevice2,
            ];
            terminal.setPrompt(<String>['1', '2', 'q', 'Q'], '1');

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Found 2 devices with name or id matching target-device:
target-device-1 (mobile) • xxx • android • Android 10
target-device-2 (mobile) • xxx • android • Android 10
[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
'''),
            );
            expect(devices, <Device>[attachedAndroidDevice1]);
            expect(deviceManager.androidDiscoverer.devicesCalled, 3);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext('including only wireless devices', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              wirelessAndroidDevice1,
              wirelessAndroidDevice2,
            ];
            terminal.setPrompt(<String>['1', '2', 'q', 'Q'], '1');

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Found 2 devices with name or id matching target-device:

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • android • Android 10
target-device-6 (wireless) (mobile) • xxx • android • Android 10

[1]: target-device-5 (wireless) (xxx)
[2]: target-device-6 (wireless) (xxx)
'''),
            );
            expect(devices, <Device>[wirelessAndroidDevice1]);
            expect(deviceManager.androidDiscoverer.devicesCalled, 3);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});
        });

        group('without stdinHasTerminal', () {
          late FakeTerminal terminal;

          setUp(() {
            terminal = FakeTerminal(stdinHasTerminal: false);
          });

          testUsingContext('including only one ephemeral', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              nonEphemeralDevice,
              attachedAndroidDevice1,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Found 2 devices with name or id matching target-device:
target-device-9 (mobile) • xxx • android • Android 10
target-device-1 (mobile) • xxx • android • Android 10
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 3);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext(
            'including matching attached, wireless, unsupported devices',
            () async {
              deviceManager.androidDiscoverer.deviceList = <Device>[
                attachedAndroidDevice1,
                attachedUnsupportedAndroidDevice,
                attachedUnsupportedForProjectAndroidDevice,
                wirelessAndroidDevice1,
                wirelessUnsupportedAndroidDevice,
                wirelessUnsupportedForProjectAndroidDevice,
              ];

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Found 4 devices with name or id matching target-device:
target-device-1 (mobile) • xxx • android • Android 10
target-device-4 (mobile) • xxx • android • Android 10

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • android • Android 10
target-device-8 (wireless) (mobile) • xxx • android • Android 10
'''),
              );
              expect(devices, isNull);
              expect(deviceManager.androidDiscoverer.devicesCalled, 3);
              expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
              expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext('including only attached devices', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              attachedAndroidDevice1,
              attachedAndroidDevice2,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Found 2 devices with name or id matching target-device:
target-device-1 (mobile) • xxx • android • Android 10
target-device-2 (mobile) • xxx • android • Android 10
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 3);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext('including only wireless devices', () async {
            deviceManager.androidDiscoverer.deviceList = <Device>[
              wirelessAndroidDevice1,
              wirelessAndroidDevice2,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Found 2 devices with name or id matching target-device:

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • android • Android 10
target-device-6 (wireless) (mobile) • xxx • android • Android 10
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.androidDiscoverer.devicesCalled, 3);
            expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
            expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});
        });
      });

      group('with hasSpecifiedAllDevices', () {
        setUp(() {
          deviceManager.hasSpecifiedAllDevices = true;
        });

        testUsingContext('including attached, wireless, unsupported devices', () async {
          deviceManager.androidDiscoverer.deviceList = <Device>[
            attachedAndroidDevice1,
            attachedUnsupportedAndroidDevice,
            attachedUnsupportedForProjectAndroidDevice,
            wirelessAndroidDevice1,
            wirelessUnsupportedAndroidDevice,
            wirelessUnsupportedForProjectAndroidDevice,
          ];
          deviceManager.otherDiscoverer.deviceList = <Device>[fuchsiaDevice];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[attachedAndroidDevice1, wirelessAndroidDevice1]);
          expect(deviceManager.androidDiscoverer.devicesCalled, 2);
          expect(deviceManager.androidDiscoverer.discoverDevicesCalled, 0);
          expect(deviceManager.androidDiscoverer.numberOfTimesPolled, 1);
        });
      });
    });
  });

  group('findAllTargetDevices on mac platform', () {
    late Platform platform;

    final FakeIOSDevice attachedIOSDevice1 = FakeIOSDevice(deviceName: 'target-device-1');
    final FakeIOSDevice attachedIOSDevice2 = FakeIOSDevice(deviceName: 'target-device-2');
    final FakeIOSDevice attachedUnsupportedIOSDevice = FakeIOSDevice(
      deviceName: 'target-device-3',
      deviceSupported: false,
    );
    final FakeIOSDevice attachedUnsupportedForProjectIOSDevice = FakeIOSDevice(
      deviceName: 'target-device-4',
      deviceSupportForProject: false,
    );

    final FakeIOSDevice disconnectedWirelessIOSDevice1 = FakeIOSDevice.notConnectedWireless(
      deviceName: 'target-device-5',
    );
    final FakeIOSDevice connectedWirelessIOSDevice1 = FakeIOSDevice.connectedWireless(
      deviceName: 'target-device-5',
    );
    final FakeIOSDevice disconnectedWirelessIOSDevice2 = FakeIOSDevice.notConnectedWireless(
      deviceName: 'target-device-6',
    );
    final FakeIOSDevice connectedWirelessIOSDevice2 = FakeIOSDevice.connectedWireless(
      deviceName: 'target-device-6',
    );
    final FakeIOSDevice disconnectedWirelessUnsupportedIOSDevice =
        FakeIOSDevice.notConnectedWireless(deviceName: 'target-device-7', deviceSupported: false);
    final FakeIOSDevice connectedWirelessUnsupportedIOSDevice = FakeIOSDevice.connectedWireless(
      deviceName: 'target-device-7',
      deviceSupported: false,
    );
    final FakeIOSDevice disconnectedWirelessUnsupportedForProjectIOSDevice =
        FakeIOSDevice.notConnectedWireless(
          deviceName: 'target-device-8',
          deviceSupportForProject: false,
        );
    final FakeIOSDevice connectedWirelessUnsupportedForProjectIOSDevice =
        FakeIOSDevice.connectedWireless(
          deviceName: 'target-device-8',
          deviceSupportForProject: false,
        );

    final FakeIOSDevice nonEphemeralDevice = FakeIOSDevice(
      deviceName: 'target-device-9',
      ephemeral: false,
    );
    final FakeDevice fuchsiaDevice = FakeDevice.fuchsia(deviceName: 'target-device-10');

    final FakeIOSDevice exactMatchAttachedIOSDevice = FakeIOSDevice(deviceName: 'target-device');
    final FakeIOSDevice exactMatchAttachedUnsupportedIOSDevice = FakeIOSDevice(
      deviceName: 'target-device',
      deviceSupported: false,
    );
    final FakeIOSDevice exactMatchUnsupportedByProjectDevice = FakeIOSDevice(
      deviceName: 'target-device',
      deviceSupportForProject: false,
    );

    setUp(() {
      platform = FakePlatform(operatingSystem: 'macos');
    });

    group('when cannot launch anything', () {
      late BufferLogger logger;
      late FakeDoctor doctor;

      setUp(() {
        logger = BufferLogger.test();
        doctor = FakeDoctor(canLaunchAnything: false);
      });

      testUsingContext('does not search for devices', () async {
        final TestDeviceManager deviceManager = TestDeviceManager(
          logger: logger,
          platform: platform,
        );
        deviceManager.iosDiscoverer.deviceList = <Device>[attachedIOSDevice1];

        final TargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices =
            TargetDevicesWithExtendedWirelessDeviceDiscovery(
              deviceManager: deviceManager,
              logger: logger,
            );
        final List<Device>? devices = await targetDevices.findAllTargetDevices();

        expect(
          logger.errorText,
          equals('''
Unable to locate a development device; please run 'flutter doctor' for information about installing additional components.
'''),
        );
        expect(devices, isNull);
        expect(deviceManager.iosDiscoverer.devicesCalled, 0);
        expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 0);
      }, overrides: <Type, Generator>{Doctor: () => doctor});
    });

    testUsingContext('ensure refresh when deviceDiscoveryTimeout is provided', () async {
      final BufferLogger logger = BufferLogger.test();
      final TestDeviceManager deviceManager = TestDeviceManager(logger: logger, platform: platform);
      deviceManager.iosDiscoverer.deviceList = <Device>[disconnectedWirelessIOSDevice1];
      deviceManager.iosDiscoverer.refreshDeviceList = <Device>[connectedWirelessIOSDevice1];

      final TargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices =
          TargetDevicesWithExtendedWirelessDeviceDiscovery(
            deviceManager: deviceManager,
            logger: logger,
          );
      final List<Device>? devices = await targetDevices.findAllTargetDevices(
        deviceDiscoveryTimeout: const Duration(seconds: 2),
      );

      expect(logger.statusText, equals(''));
      expect(devices, <Device>[connectedWirelessIOSDevice1]);
      expect(deviceManager.iosDiscoverer.devicesCalled, 2);
      expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
      expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 1);
    });

    testUsingContext('ensure no refresh when deviceConnectionInterface is attached', () async {
      final BufferLogger logger = BufferLogger.test();
      final TestDeviceManager deviceManager = TestDeviceManager(logger: logger, platform: platform);
      deviceManager.iosDiscoverer.deviceList = <Device>[attachedIOSDevice1];

      final TargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices =
          TargetDevicesWithExtendedWirelessDeviceDiscovery(
            deviceManager: deviceManager,
            logger: logger,
            deviceConnectionInterface: DeviceConnectionInterface.attached,
          );
      final List<Device>? devices = await targetDevices.findAllTargetDevices();

      expect(logger.statusText, equals(''));
      expect(devices, <Device>[attachedIOSDevice1]);
      expect(deviceManager.iosDiscoverer.devicesCalled, 1);
      expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 0);
      expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 1);
    });

    testUsingContext(
      'ensure unsupported for projects are included when includeDevicesUnsupportedByProject is true',
      () async {
        final BufferLogger logger = BufferLogger.test();
        final TestDeviceManager deviceManager = TestDeviceManager(
          logger: logger,
          platform: platform,
        );
        deviceManager.iosDiscoverer.deviceList = <Device>[
          attachedUnsupportedIOSDevice,
          attachedUnsupportedForProjectIOSDevice,
        ];

        final TargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices =
            TargetDevicesWithExtendedWirelessDeviceDiscovery(
              deviceManager: deviceManager,
              logger: logger,
            );
        final List<Device>? devices = await targetDevices.findAllTargetDevices(
          includeDevicesUnsupportedByProject: true,
        );

        expect(logger.statusText, equals(''));
        expect(devices, <Device>[attachedUnsupportedForProjectIOSDevice]);
        expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
        expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
      },
    );

    group('finds no devices', () {
      late BufferLogger logger;
      late TestDeviceManager deviceManager;
      late TargetDevices targetDevices;

      setUp(() {
        logger = BufferLogger.test();
        deviceManager = TestDeviceManager(logger: logger, platform: platform);
        targetDevices = TargetDevices(
          platform: platform,
          deviceManager: deviceManager,
          logger: logger,
        );
      });

      group('with device not specified', () {
        testUsingContext('when no devices', () async {
          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...

No supported devices connected.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.iosDiscoverer.devicesCalled, 3);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        testUsingContext('when device is unsupported by flutter or project', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            attachedUnsupportedIOSDevice,
            attachedUnsupportedForProjectIOSDevice,
            disconnectedWirelessUnsupportedIOSDevice,
            disconnectedWirelessUnsupportedForProjectIOSDevice,
          ];
          deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
            attachedUnsupportedIOSDevice,
            attachedUnsupportedForProjectIOSDevice,
            connectedWirelessUnsupportedIOSDevice,
            connectedWirelessUnsupportedForProjectIOSDevice,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...

No supported devices connected.

The following devices were found, but are not supported by this project:
target-device-3 (mobile)            • xxx • ios • iOS 16 (unsupported)
target-device-4 (mobile)            • xxx • ios • iOS 16
target-device-7 (wireless) (mobile) • xxx • ios • iOS 16 (unsupported)
target-device-8 (wireless) (mobile) • xxx • ios • iOS 16
If you would like your app to run on ios, consider running `flutter create .` to generate projects for these platforms.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.iosDiscoverer.devicesCalled, 3);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        testUsingContext('when all found devices are not connected', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            disconnectedWirelessIOSDevice1,
            disconnectedWirelessIOSDevice2,
          ];
          deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
            disconnectedWirelessIOSDevice1,
            disconnectedWirelessIOSDevice2,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...

No supported devices connected.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.iosDiscoverer.devicesCalled, 3);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        group('when deviceConnectionInterface does not match', () {
          testUsingContext('filter of wireless', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[attachedIOSDevice1];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[attachedIOSDevice1];

            final TestTargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices =
                TestTargetDevicesWithExtendedWirelessDeviceDiscovery(
                  deviceManager: deviceManager,
                  logger: logger,
                  deviceConnectionInterface: DeviceConnectionInterface.wireless,
                );
            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Checking for wireless devices...

No supported devices connected.
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.iosDiscoverer.devicesCalled, 2);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 1);
          });
        });
      });

      group('with hasSpecifiedDeviceId', () {
        setUp(() {
          deviceManager.specifiedDeviceId = 'target-device';
        });

        testUsingContext('when no devices', () async {
          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...

No supported devices found with name or id matching 'target-device'.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.iosDiscoverer.devicesCalled, 4);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
        });

        testUsingContext('when no devices match', () async {
          final FakeIOSDevice device1 = FakeIOSDevice(deviceName: 'no-match-1');
          final FakeIOSDevice device2 = FakeIOSDevice.notConnectedWireless(
            deviceName: 'no-match-2',
          );
          final FakeIOSDevice device2Connected = FakeIOSDevice.connectedWireless(
            deviceName: 'no-match-2',
          );
          deviceManager.iosDiscoverer.deviceList = <Device>[device1, device2];
          deviceManager.iosDiscoverer.refreshDeviceList = <Device>[device1, device2Connected];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...

No supported devices found with name or id matching 'target-device'.

The following devices were found:
no-match-1 (mobile)            • xxx • ios • iOS 16
no-match-2 (wireless) (mobile) • xxx • ios • iOS 16
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.iosDiscoverer.devicesCalled, 4);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
        });

        testUsingContext('when matching device is unsupported by flutter', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[exactMatchAttachedUnsupportedIOSDevice];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...

No supported devices found with name or id matching 'target-device'.

The following devices were found:
target-device (mobile) • xxx • ios • iOS 16 (unsupported)
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.iosDiscoverer.devicesCalled, 4);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
        });

        testUsingContext('when only matching device is dev mode disabled', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            FakeIOSDevice(deviceName: 'target-device', devModeEnabled: false),
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals(r'''
To use 'target-device' for development, enable Developer Mode in Settings → Privacy & Security on the device. If this does not work, open Xcode, reconnect the device, and look for a popup on the device asking you to trust this computer.
'''),
          );
          expect(devices, isNull);
        });

        testUsingContext('when one of the matching devices has dev mode disabled', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            FakeIOSDevice(deviceName: 'target-device-1', devModeEnabled: false, isConnected: false),
            FakeIOSDevice(deviceName: 'target-device-2'),
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();
          expect(
            logger.statusText,
            equals('''
To use 'target-device-1' for development, enable Developer Mode in Settings → Privacy & Security on the device. If this does not work, open Xcode, reconnect the device, and look for a popup on the device asking you to trust this computer.
Checking for wireless devices...
'''),
          );
          expect(devices, isNotNull);
        });

        testUsingContext('when all matching devices are dev mode disabled', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            FakeIOSDevice(deviceName: 'target-device-1', devModeEnabled: false, isConnected: false),
            FakeIOSDevice(deviceName: 'target-device-2', devModeEnabled: false, isConnected: false),
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
To use 'target-device-1' for development, enable Developer Mode in Settings → Privacy & Security on the device. If this does not work, open Xcode, reconnect the device, and look for a popup on the device asking you to trust this computer.
To use 'target-device-2' for development, enable Developer Mode in Settings → Privacy & Security on the device. If this does not work, open Xcode, reconnect the device, and look for a popup on the device asking you to trust this computer.
No devices found yet. Checking for wireless devices...

No supported devices found with name or id matching 'target-device'.
'''),
          );
          expect(devices, isNull);
        });

        testUsingContext('when only matching device is unpaired', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            FakeIOSDevice(deviceName: 'target-device', isPaired: false),
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
'target-device' is not paired. Open Xcode and trust this computer when prompted.
'''),
          );
          expect(devices, isNull);
        });

        testUsingContext('when one of the matching devices is unpaired', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            FakeIOSDevice(deviceName: 'target-device-1', isPaired: false, isConnected: false),
            FakeIOSDevice(deviceName: 'target-device-2'),
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();
          expect(
            logger.statusText,
            contains('''
'target-device-1' is not paired. Open Xcode and trust this computer when prompted.
Checking for wireless devices...
'''),
          );
          expect(devices, isNotNull);
        });

        testUsingContext('when all matching devices are unpaired', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            FakeIOSDevice(deviceName: 'target-device-1', isPaired: false, isConnected: false),
            FakeIOSDevice(deviceName: 'target-device-2', isPaired: false, isConnected: false),
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            contains('''
'target-device-1' is not paired. Open Xcode and trust this computer when prompted.
'target-device-2' is not paired. Open Xcode and trust this computer when prompted.
No devices found yet. Checking for wireless devices...

No supported devices found with name or id matching 'target-device'.
'''),
          );
          expect(devices, isNull);
        });

        group('when deviceConnectionInterface does not match', () {
          testUsingContext('filter of wireless', () async {
            final FakeIOSDevice device1 = FakeIOSDevice.notConnectedWireless(
              deviceName: 'not-a-match',
            );
            final FakeIOSDevice device1Connected = FakeIOSDevice.connectedWireless(
              deviceName: 'not-a-match',
            );
            deviceManager.iosDiscoverer.deviceList = <Device>[exactMatchAttachedIOSDevice, device1];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              exactMatchAttachedIOSDevice,
              device1Connected,
            ];

            final TestTargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices =
                TestTargetDevicesWithExtendedWirelessDeviceDiscovery(
                  deviceManager: deviceManager,
                  logger: logger,
                  deviceConnectionInterface: DeviceConnectionInterface.wireless,
                );
            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Checking for wireless devices...

No supported devices found with name or id matching 'target-device'.

The following devices were found:
not-a-match (wireless) (mobile) • xxx • ios • iOS 16
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          });
        });
      });

      group('with hasSpecifiedAllDevices', () {
        setUp(() {
          deviceManager.hasSpecifiedAllDevices = true;
        });

        testUsingContext('when no devices', () async {
          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...

No devices found.
'''),
          );
          expect(devices, isNull);
          expect(deviceManager.iosDiscoverer.devicesCalled, 3);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        testUsingContext(
          'when devices are either unsupported by flutter or project or all',
          () async {
            deviceManager.otherDiscoverer.deviceList = <Device>[fuchsiaDevice];
            deviceManager.iosDiscoverer.deviceList = <Device>[
              attachedUnsupportedIOSDevice,
              attachedUnsupportedForProjectIOSDevice,
              disconnectedWirelessUnsupportedIOSDevice,
              disconnectedWirelessUnsupportedForProjectIOSDevice,
            ];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              attachedUnsupportedIOSDevice,
              attachedUnsupportedForProjectIOSDevice,
              connectedWirelessUnsupportedIOSDevice,
              connectedWirelessUnsupportedForProjectIOSDevice,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No devices found yet. Checking for wireless devices...

No devices found.

The following devices were found, but are not supported by this project:
target-device-10 (mobile)           • xxx • fuchsia-arm64 • tester
target-device-3 (mobile)            • xxx • ios           • iOS 16 (unsupported)
target-device-4 (mobile)            • xxx • ios           • iOS 16
target-device-7 (wireless) (mobile) • xxx • ios           • iOS 16 (unsupported)
target-device-8 (wireless) (mobile) • xxx • ios           • iOS 16
If you would like your app to run on fuchsia or ios, consider running `flutter create .` to generate projects for these platforms.
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          },
        );
      });
    });

    group('finds single device', () {
      late TestBufferLogger logger;
      late TestDeviceManager deviceManager;
      late TargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices;

      setUp(() {
        logger = TestBufferLogger.test();
        deviceManager = TestDeviceManager(logger: logger, platform: platform);
        targetDevices = TargetDevicesWithExtendedWirelessDeviceDiscovery(
          deviceManager: deviceManager,
          logger: logger,
        );
      });

      group('with device not specified', () {
        testUsingContext('when single ephemeral attached device', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[attachedIOSDevice1];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[attachedIOSDevice1]);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        testUsingContext('when single wireless device', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[disconnectedWirelessIOSDevice1];
          deviceManager.iosDiscoverer.refreshDeviceList = <Device>[connectedWirelessIOSDevice1];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...
'''),
          );
          expect(devices, <Device>[connectedWirelessIOSDevice1]);
          expect(deviceManager.iosDiscoverer.devicesCalled, 2);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        testUsingContext('when multiple but only one attached ephemeral', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            nonEphemeralDevice,
            attachedIOSDevice1,
            disconnectedWirelessIOSDevice1,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[attachedIOSDevice1]);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        group('with stdinHasTerminal', () {
          late FakeTerminal terminal;

          setUp(() {
            terminal = FakeTerminal(supportsColor: true);
            logger = TestBufferLogger.test(terminal: terminal);
          });

          testUsingContext(
            'when single non-ephemeral attached device',
            () async {
              deviceManager.iosDiscoverer.deviceList = <Device>[nonEphemeralDevice];

              final TestTargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices =
                  TestTargetDevicesWithExtendedWirelessDeviceDiscovery(
                    deviceManager: deviceManager,
                    logger: logger,
                  );
              targetDevices.waitForWirelessBeforeInput = true;
              targetDevices.deviceSelection.input = <String>['1'];
              logger.originalStatusText = '''
Connected devices:
target-device-9 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-9 (xxx)
''';

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Connected devices:
target-device-9 (mobile) • xxx • ios • iOS 16

No wireless devices were found.

[1]: target-device-9 (xxx)
Please choose one (or "q" to quit): '''),
              );
              expect(devices, <Device>[nonEphemeralDevice]);
              expect(deviceManager.iosDiscoverer.devicesCalled, 2);
              expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
              expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext(
            'handle invalid options for device',
            () async {
              deviceManager.iosDiscoverer.deviceList = <Device>[nonEphemeralDevice];

              final TestTargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices =
                  TestTargetDevicesWithExtendedWirelessDeviceDiscovery(
                    deviceManager: deviceManager,
                    logger: logger,
                  );
              targetDevices.waitForWirelessBeforeInput = true;

              // Having the '0' first is an invalid choice for a device, the second
              // item in the list is a '2' which is out of range since we only have
              // one item in the deviceList. The final item in the list, is '1'
              // which is a valid option though which will return a valid device
              //
              // Important: if none of the values in the list are valid, the test will
              // hang indefinitely since the [userSelectDevice()] method uses a while
              // loop to listen for valid devices
              targetDevices.deviceSelection.input = <String>['0', '2', '1'];
              logger.originalStatusText = '''
Connected devices:
target-device-9 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-9 (xxx)
''';

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Connected devices:
target-device-9 (mobile) • xxx • ios • iOS 16

No wireless devices were found.

[1]: target-device-9 (xxx)
Please choose one (or "q" to quit): '''),
              );
              expect(devices, <Device>[nonEphemeralDevice]);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );
        });

        group('without stdinHasTerminal', () {
          late FakeTerminal terminal;

          setUp(() {
            terminal = FakeTerminal(stdinHasTerminal: false);
            targetDevices = TargetDevicesWithExtendedWirelessDeviceDiscovery(
              deviceManager: deviceManager,
              logger: logger,
            );
          });

          testUsingContext(
            'when single non-ephemeral attached device',
            () async {
              deviceManager.iosDiscoverer.deviceList = <Device>[nonEphemeralDevice];

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Checking for wireless devices...
'''),
              );
              expect(devices, <Device>[nonEphemeralDevice]);
              expect(deviceManager.iosDiscoverer.devicesCalled, 2);
              expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
              expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );
        });
      });

      group('with hasSpecifiedDeviceId', () {
        setUp(() {
          deviceManager.specifiedDeviceId = 'target-device';
        });

        testUsingContext('when multiple matches but first is unsupported by flutter', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            exactMatchAttachedUnsupportedIOSDevice,
            exactMatchAttachedIOSDevice,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[exactMatchAttachedIOSDevice]);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
        });

        testUsingContext('when matching device is unsupported by project', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[exactMatchUnsupportedByProjectDevice];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[exactMatchUnsupportedByProjectDevice]);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
        });

        testUsingContext('when matching attached device', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            exactMatchAttachedIOSDevice,
            disconnectedWirelessIOSDevice1,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[exactMatchAttachedIOSDevice]);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
        });

        testUsingContext('when exact matching wireless device', () async {
          final FakeIOSDevice exactMatchWirelessDevice = FakeIOSDevice.notConnectedWireless(
            deviceName: 'target-device',
          );
          deviceManager.iosDiscoverer.deviceList = <Device>[
            attachedIOSDevice1,
            exactMatchWirelessDevice,
          ];
          deviceManager.setDeviceToWaitFor(
            exactMatchWirelessDevice,
            DeviceConnectionInterface.wireless,
          );

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
Waiting for target-device (wireless) to connect...
'''),
          );
          expect(devices, <Device>[exactMatchWirelessDevice]);
          expect(devices?.first.isConnected, true);
          expect(devices?.first.connectionInterface, DeviceConnectionInterface.wireless);
          expect(deviceManager.iosDiscoverer.devicesCalled, 1);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isTrue);
        });

        testUsingContext('when partially matching single wireless devices', () async {
          final FakeIOSDevice partialMatchWirelessDevice = FakeIOSDevice.notConnectedWireless(
            deviceName: 'target-device-1',
          );
          deviceManager.iosDiscoverer.deviceList = <Device>[partialMatchWirelessDevice];
          deviceManager.setDeviceToWaitFor(
            partialMatchWirelessDevice,
            DeviceConnectionInterface.wireless,
          );

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
Waiting for target-device-1 (wireless) to connect...
'''),
          );
          expect(devices, <Device>[partialMatchWirelessDevice]);
          expect(devices?.first.isConnected, true);
          expect(devices?.first.connectionInterface, DeviceConnectionInterface.wireless);
          expect(deviceManager.iosDiscoverer.devicesCalled, 1);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isTrue);
        });

        testUsingContext(
          'when exact matching an attached device and partial matching a wireless device',
          () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              exactMatchAttachedIOSDevice,
              connectedWirelessIOSDevice1,
            ];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              exactMatchAttachedIOSDevice,
              connectedWirelessIOSDevice1,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(logger.statusText, equals(''));
            expect(devices, <Device>[exactMatchAttachedIOSDevice]);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
          },
        );

        testUsingContext(
          'when partially matching multiple device but only one is connected',
          () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              attachedIOSDevice1,
              disconnectedWirelessIOSDevice1,
            ];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              attachedIOSDevice1,
              disconnectedWirelessIOSDevice1,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Checking for wireless devices...
'''),
            );
            expect(devices, <Device>[attachedIOSDevice1]);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
          },
        );

        testUsingContext('when partially matching single attached device', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[attachedIOSDevice1];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[attachedIOSDevice1]);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
        });

        testUsingContext(
          'when partially matching wireless device and an attached device from different discoverer',
          () async {
            final FakeDevice androidDevice = FakeDevice(deviceName: 'target-device-android');
            deviceManager.androidDiscoverer.deviceList = <Device>[androidDevice];
            deviceManager.iosDiscoverer.deviceList = <Device>[disconnectedWirelessIOSDevice1];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              disconnectedWirelessIOSDevice1,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Checking for wireless devices...
'''),
            );
            expect(devices, <Device>[androidDevice]);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
          },
        );

        testUsingContext('when matching single non-ephemeral attached device', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[nonEphemeralDevice];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(logger.statusText, equals(''));
          expect(devices, <Device>[nonEphemeralDevice]);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });
      });

      group('with hasSpecifiedAllDevices', () {
        setUp(() {
          deviceManager.hasSpecifiedAllDevices = true;
        });

        testUsingContext('when only one device', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[attachedIOSDevice1];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
Checking for wireless devices...
'''),
          );
          expect(devices, <Device>[attachedIOSDevice1]);
          expect(deviceManager.iosDiscoverer.devicesCalled, 2);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        testUsingContext('when single non-ephemeral attached device', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[nonEphemeralDevice];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
Checking for wireless devices...
'''),
          );
          expect(devices, <Device>[nonEphemeralDevice]);
          expect(deviceManager.iosDiscoverer.devicesCalled, 2);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });
      });
    });

    group('finds multiple devices', () {
      late TestBufferLogger logger;
      late TestDeviceManager deviceManager;

      setUp(() {
        logger = TestBufferLogger.test();
        deviceManager = TestDeviceManager(logger: logger, platform: platform);
      });

      group('with device not specified', () {
        group('with stdinHasTerminal', () {
          late FakeTerminal terminal;
          late TestTargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices;

          setUp(() {
            terminal = FakeTerminal(supportsColor: true);
            logger = TestBufferLogger.test(terminal: terminal);
            targetDevices = TestTargetDevicesWithExtendedWirelessDeviceDiscovery(
              deviceManager: deviceManager,
              logger: logger,
            );
          });

          testUsingContext(
            'including attached, wireless, unsupported devices',
            () async {
              deviceManager.iosDiscoverer.deviceList = <Device>[
                attachedIOSDevice1,
                attachedIOSDevice2,
                attachedUnsupportedIOSDevice,
                attachedUnsupportedForProjectIOSDevice,
                disconnectedWirelessIOSDevice1,
                disconnectedWirelessUnsupportedIOSDevice,
                disconnectedWirelessUnsupportedForProjectIOSDevice,
              ];
              deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
                attachedIOSDevice1,
                attachedIOSDevice2,
                attachedUnsupportedIOSDevice,
                attachedUnsupportedForProjectIOSDevice,
                connectedWirelessIOSDevice1,
                connectedWirelessUnsupportedIOSDevice,
                connectedWirelessUnsupportedForProjectIOSDevice,
              ];

              targetDevices.waitForWirelessBeforeInput = true;
              targetDevices.deviceSelection.input = <String>['3'];
              logger.originalStatusText = '''
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
''';

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
[3]: target-device-5 (wireless) (xxx)
Please choose one (or "q" to quit): '''),
              );
              expect(devices, <Device>[connectedWirelessIOSDevice1]);
              expect(deviceManager.iosDiscoverer.devicesCalled, 2);
              expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
              expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext('including only attached devices', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              attachedIOSDevice1,
              attachedIOSDevice2,
            ];

            targetDevices.waitForWirelessBeforeInput = true;
            targetDevices.deviceSelection.input = <String>['2'];
            logger.originalStatusText = '''
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
''';

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

No wireless devices were found.

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
Please choose one (or "q" to quit): '''),
            );
            expect(devices, <Device>[attachedIOSDevice2]);
            expect(deviceManager.iosDiscoverer.devicesCalled, 2);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext('including only wireless devices', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              disconnectedWirelessIOSDevice1,
              disconnectedWirelessIOSDevice2,
            ];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              connectedWirelessIOSDevice1,
              connectedWirelessIOSDevice2,
            ];

            targetDevices.waitForWirelessBeforeInput = true;
            targetDevices.deviceSelection.input = <String>['2'];
            terminal.setPrompt(<String>['1', '2', 'q', 'Q'], '1');

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No devices found yet. Checking for wireless devices...

Connected devices:

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16
target-device-6 (wireless) (mobile) • xxx • ios • iOS 16

[1]: target-device-5 (wireless) (xxx)
[2]: target-device-6 (wireless) (xxx)
'''),
            );
            expect(devices, <Device>[connectedWirelessIOSDevice1]);
            expect(deviceManager.iosDiscoverer.devicesCalled, 2);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          group('but no color support', () {
            setUp(() {
              terminal = FakeTerminal();
              logger = TestBufferLogger.test(terminal: terminal);
              targetDevices = TestTargetDevicesWithExtendedWirelessDeviceDiscovery(
                deviceManager: deviceManager,
                logger: logger,
              );
            });

            testUsingContext(
              'and waits for wireless devices to return',
              () async {
                deviceManager.iosDiscoverer.deviceList = <Device>[
                  attachedIOSDevice1,
                  attachedIOSDevice2,
                  disconnectedWirelessIOSDevice1,
                ];
                deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
                  attachedIOSDevice1,
                  attachedIOSDevice2,
                  connectedWirelessIOSDevice1,
                ];

                terminal.setPrompt(<String>['1', '2', '3', 'q', 'Q'], '1');
                final List<Device>? devices = await targetDevices.findAllTargetDevices();

                expect(
                  logger.statusText,
                  equals('''
Checking for wireless devices...

Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
[3]: target-device-5 (wireless) (xxx)
'''),
                );
                expect(devices, <Device>[attachedIOSDevice1]);
                expect(deviceManager.iosDiscoverer.devicesCalled, 2);
                expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
                expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
              },
              overrides: <Type, Generator>{AnsiTerminal: () => terminal},
            );
          });

          group('with verbose logging', () {
            setUp(() {
              logger = TestBufferLogger.test(terminal: terminal, verbose: true);
              targetDevices = TestTargetDevicesWithExtendedWirelessDeviceDiscovery(
                deviceManager: deviceManager,
                logger: logger,
              );
            });

            testUsingContext(
              'including only attached devices',
              () async {
                deviceManager.iosDiscoverer.deviceList = <Device>[
                  attachedIOSDevice1,
                  attachedIOSDevice2,
                ];

                targetDevices.waitForWirelessBeforeInput = true;
                targetDevices.deviceSelection.input = <String>['2'];
                logger.originalStatusText = '''
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
''';

                final List<Device>? devices = await targetDevices.findAllTargetDevices();

                expect(
                  logger.statusText,
                  equals('''
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
No wireless devices were found.
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
Please choose one (or "q" to quit): '''),
                );

                expect(devices, <Device>[attachedIOSDevice2]);
                expect(deviceManager.iosDiscoverer.devicesCalled, 2);
                expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
                expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
              },
              overrides: <Type, Generator>{AnsiTerminal: () => terminal},
            );

            testUsingContext(
              'including attached and wireless devices',
              () async {
                deviceManager.iosDiscoverer.deviceList = <Device>[
                  attachedIOSDevice1,
                  attachedIOSDevice2,
                  disconnectedWirelessIOSDevice1,
                ];
                deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
                  attachedIOSDevice1,
                  attachedIOSDevice2,
                  connectedWirelessIOSDevice1,
                ];

                targetDevices.waitForWirelessBeforeInput = true;
                targetDevices.deviceSelection.input = <String>['2'];
                logger.originalStatusText = '''
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
''';
                final List<Device>? devices = await targetDevices.findAllTargetDevices();

                expect(
                  logger.statusText,
                  equals('''
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
Connected devices:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
[3]: target-device-5 (wireless) (xxx)
Please choose one (or "q" to quit): '''),
                );

                expect(devices, <Device>[attachedIOSDevice2]);
                expect(deviceManager.iosDiscoverer.devicesCalled, 2);
                expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
                expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
              },
              overrides: <Type, Generator>{AnsiTerminal: () => terminal},
            );
          });
        });

        group('without stdinHasTerminal', () {
          late FakeTerminal terminal;
          late TargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices;

          setUp(() {
            terminal = FakeTerminal(stdinHasTerminal: false);
            targetDevices = TargetDevicesWithExtendedWirelessDeviceDiscovery(
              deviceManager: deviceManager,
              logger: logger,
            );
          });

          testUsingContext(
            'including attached, wireless, unsupported devices',
            () async {
              deviceManager.iosDiscoverer.deviceList = <Device>[
                attachedIOSDevice1,
                attachedIOSDevice2,
                attachedUnsupportedIOSDevice,
                attachedUnsupportedForProjectIOSDevice,
                disconnectedWirelessIOSDevice1,
                disconnectedWirelessUnsupportedIOSDevice,
                disconnectedWirelessUnsupportedForProjectIOSDevice,
              ];
              deviceManager.iosDiscoverer.deviceList = <Device>[
                attachedIOSDevice1,
                attachedIOSDevice2,
                attachedUnsupportedIOSDevice,
                attachedUnsupportedForProjectIOSDevice,
                connectedWirelessIOSDevice1,
                connectedWirelessUnsupportedIOSDevice,
                connectedWirelessUnsupportedForProjectIOSDevice,
              ];

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Checking for wireless devices...

More than one device connected; please specify a device with the '-d <deviceId>' flag, or use '-d all' to act on all devices.

target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16
target-device-4 (mobile) • xxx • ios • iOS 16

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16
target-device-8 (wireless) (mobile) • xxx • ios • iOS 16
'''),
              );
              expect(devices, isNull);
              expect(deviceManager.iosDiscoverer.devicesCalled, 4);
              expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
              expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext('including only attached devices', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              attachedIOSDevice1,
              attachedIOSDevice2,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Checking for wireless devices...

More than one device connected; please specify a device with the '-d <deviceId>' flag, or use '-d all' to act on all devices.

target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.iosDiscoverer.devicesCalled, 4);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext('including only wireless devices', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              disconnectedWirelessIOSDevice1,
              disconnectedWirelessIOSDevice2,
            ];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              connectedWirelessIOSDevice1,
              connectedWirelessIOSDevice2,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No devices found yet. Checking for wireless devices...

More than one device connected; please specify a device with the '-d <deviceId>' flag, or use '-d all' to act on all devices.

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16
target-device-6 (wireless) (mobile) • xxx • ios • iOS 16
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.iosDiscoverer.devicesCalled, 4);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});
        });
      });

      group('with hasSpecifiedDeviceId', () {
        setUp(() {
          deviceManager.specifiedDeviceId = 'target-device';
        });

        group('with stdinHasTerminal', () {
          late FakeTerminal terminal;
          late TestTargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices;

          setUp(() {
            terminal = FakeTerminal(supportsColor: true);
            logger = TestBufferLogger.test(terminal: terminal);
            targetDevices = TestTargetDevicesWithExtendedWirelessDeviceDiscovery(
              deviceManager: deviceManager,
              logger: logger,
            );
          });

          testUsingContext(
            'including attached, wireless, unsupported devices',
            () async {
              deviceManager.iosDiscoverer.deviceList = <Device>[
                attachedIOSDevice1,
                attachedUnsupportedIOSDevice,
                attachedUnsupportedForProjectIOSDevice,
                disconnectedWirelessIOSDevice1,
                disconnectedWirelessUnsupportedIOSDevice,
                disconnectedWirelessUnsupportedForProjectIOSDevice,
              ];
              deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
                attachedIOSDevice1,
                attachedUnsupportedIOSDevice,
                attachedUnsupportedForProjectIOSDevice,
                connectedWirelessIOSDevice1,
                connectedWirelessUnsupportedIOSDevice,
                connectedWirelessUnsupportedForProjectIOSDevice,
              ];

              targetDevices.waitForWirelessBeforeInput = true;
              targetDevices.deviceSelection.input = <String>['3'];
              logger.originalStatusText = '''
Found multiple devices with name or id matching target-device:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-4 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-1 (xxx)
[2]: target-device-4 (xxx)
''';
              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Found multiple devices with name or id matching target-device:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-4 (mobile) • xxx • ios • iOS 16

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16
target-device-8 (wireless) (mobile) • xxx • ios • iOS 16

[1]: target-device-1 (xxx)
[2]: target-device-4 (xxx)
[3]: target-device-5 (wireless) (xxx)
[4]: target-device-8 (wireless) (xxx)
Please choose one (or "q" to quit): '''),
              );
              expect(devices, <Device>[connectedWirelessIOSDevice1]);
              expect(deviceManager.iosDiscoverer.devicesCalled, 3);
              expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
              expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
              expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext('including only attached devices', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              attachedIOSDevice1,
              attachedIOSDevice2,
            ];

            targetDevices.waitForWirelessBeforeInput = true;
            targetDevices.deviceSelection.input = <String>['2'];
            logger.originalStatusText = '''
Found multiple devices with name or id matching target-device:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

Checking for wireless devices...

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
''';
            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Found multiple devices with name or id matching target-device:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16

No wireless devices were found.

[1]: target-device-1 (xxx)
[2]: target-device-2 (xxx)
Please choose one (or "q" to quit): '''),
            );
            expect(devices, <Device>[attachedIOSDevice2]);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext('including only wireless devices', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              disconnectedWirelessIOSDevice1,
              disconnectedWirelessIOSDevice2,
            ];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              connectedWirelessIOSDevice1,
              connectedWirelessIOSDevice2,
            ];

            terminal.setPrompt(<String>['1', '2', 'q', 'Q'], '1');
            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No devices found yet. Checking for wireless devices...

Found 2 devices with name or id matching target-device:

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16
target-device-6 (wireless) (mobile) • xxx • ios • iOS 16

[1]: target-device-5 (wireless) (xxx)
[2]: target-device-6 (wireless) (xxx)
'''),
            );
            expect(devices, <Device>[connectedWirelessIOSDevice1]);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});
        });

        group('without stdinHasTerminal', () {
          late FakeTerminal terminal;
          late TargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices;

          setUp(() {
            terminal = FakeTerminal(stdinHasTerminal: false);
            targetDevices = TargetDevicesWithExtendedWirelessDeviceDiscovery(
              deviceManager: deviceManager,
              logger: logger,
            );
          });

          testUsingContext('including only one ephemeral', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              nonEphemeralDevice,
              attachedIOSDevice1,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Checking for wireless devices...

Found 2 devices with name or id matching target-device:
target-device-9 (mobile) • xxx • ios • iOS 16
target-device-1 (mobile) • xxx • ios • iOS 16
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext(
            'including matching attached, wireless, unsupported devices',
            () async {
              deviceManager.iosDiscoverer.deviceList = <Device>[
                attachedIOSDevice1,
                attachedUnsupportedIOSDevice,
                attachedUnsupportedForProjectIOSDevice,
                disconnectedWirelessIOSDevice1,
                disconnectedWirelessUnsupportedIOSDevice,
                disconnectedWirelessUnsupportedForProjectIOSDevice,
              ];
              deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
                attachedIOSDevice1,
                attachedUnsupportedIOSDevice,
                attachedUnsupportedForProjectIOSDevice,
                connectedWirelessIOSDevice1,
                connectedWirelessUnsupportedIOSDevice,
                connectedWirelessUnsupportedForProjectIOSDevice,
              ];

              final List<Device>? devices = await targetDevices.findAllTargetDevices();

              expect(
                logger.statusText,
                equals('''
Checking for wireless devices...

Found 4 devices with name or id matching target-device:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-4 (mobile) • xxx • ios • iOS 16

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16
target-device-8 (wireless) (mobile) • xxx • ios • iOS 16
'''),
              );
              expect(devices, isNull);
              expect(deviceManager.iosDiscoverer.devicesCalled, 3);
              expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
              expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
              expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
            },
            overrides: <Type, Generator>{AnsiTerminal: () => terminal},
          );

          testUsingContext('including only attached devices', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              attachedIOSDevice1,
              attachedIOSDevice2,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
Checking for wireless devices...

Found 2 devices with name or id matching target-device:
target-device-1 (mobile) • xxx • ios • iOS 16
target-device-2 (mobile) • xxx • ios • iOS 16
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});

          testUsingContext('including only wireless devices', () async {
            deviceManager.iosDiscoverer.deviceList = <Device>[
              disconnectedWirelessIOSDevice1,
              disconnectedWirelessIOSDevice2,
            ];
            deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
              connectedWirelessIOSDevice1,
              connectedWirelessIOSDevice2,
            ];

            final List<Device>? devices = await targetDevices.findAllTargetDevices();

            expect(
              logger.statusText,
              equals('''
No devices found yet. Checking for wireless devices...

Found 2 devices with name or id matching target-device:

Wirelessly connected devices:
target-device-5 (wireless) (mobile) • xxx • ios • iOS 16
target-device-6 (wireless) (mobile) • xxx • ios • iOS 16
'''),
            );
            expect(devices, isNull);
            expect(deviceManager.iosDiscoverer.devicesCalled, 3);
            expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
            expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
            expect(deviceManager.iosDiscoverer.xcdevice.waitedForDeviceToConnect, isFalse);
          }, overrides: <Type, Generator>{AnsiTerminal: () => terminal});
        });
      });

      group('with hasSpecifiedAllDevices', () {
        late TargetDevicesWithExtendedWirelessDeviceDiscovery targetDevices;
        setUp(() {
          deviceManager.hasSpecifiedAllDevices = true;
          targetDevices = TargetDevicesWithExtendedWirelessDeviceDiscovery(
            deviceManager: deviceManager,
            logger: logger,
          );
        });

        testUsingContext('including attached, wireless, unsupported devices', () async {
          deviceManager.otherDiscoverer.deviceList = <Device>[fuchsiaDevice];
          deviceManager.iosDiscoverer.deviceList = <Device>[
            attachedIOSDevice1,
            attachedUnsupportedIOSDevice,
            attachedUnsupportedForProjectIOSDevice,
            disconnectedWirelessIOSDevice1,
            disconnectedWirelessUnsupportedIOSDevice,
            disconnectedWirelessUnsupportedForProjectIOSDevice,
          ];
          deviceManager.iosDiscoverer.deviceList = <Device>[
            attachedIOSDevice1,
            attachedUnsupportedIOSDevice,
            attachedUnsupportedForProjectIOSDevice,
            connectedWirelessIOSDevice1,
            connectedWirelessUnsupportedIOSDevice,
            connectedWirelessUnsupportedForProjectIOSDevice,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
Checking for wireless devices...
'''),
          );
          expect(devices, <Device>[attachedIOSDevice1, connectedWirelessIOSDevice1]);
          expect(deviceManager.iosDiscoverer.devicesCalled, 2);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        testUsingContext('including only attached devices', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[attachedIOSDevice1, attachedIOSDevice2];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
Checking for wireless devices...
'''),
          );
          expect(devices, <Device>[attachedIOSDevice1, attachedIOSDevice2]);
          expect(deviceManager.iosDiscoverer.devicesCalled, 2);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });

        testUsingContext('including only wireless devices', () async {
          deviceManager.iosDiscoverer.deviceList = <Device>[
            disconnectedWirelessIOSDevice1,
            disconnectedWirelessIOSDevice2,
          ];
          deviceManager.iosDiscoverer.refreshDeviceList = <Device>[
            connectedWirelessIOSDevice1,
            connectedWirelessIOSDevice2,
          ];

          final List<Device>? devices = await targetDevices.findAllTargetDevices();

          expect(
            logger.statusText,
            equals('''
No devices found yet. Checking for wireless devices...
'''),
          );
          expect(devices, <Device>[connectedWirelessIOSDevice1, connectedWirelessIOSDevice2]);
          expect(deviceManager.iosDiscoverer.devicesCalled, 2);
          expect(deviceManager.iosDiscoverer.discoverDevicesCalled, 1);
          expect(deviceManager.iosDiscoverer.numberOfTimesPolled, 2);
        });
      });
    });
  });
}

class TestTargetDevicesWithExtendedWirelessDeviceDiscovery
    extends TargetDevicesWithExtendedWirelessDeviceDiscovery {
  TestTargetDevicesWithExtendedWirelessDeviceDiscovery({
    required super.deviceManager,
    required super.logger,
    super.deviceConnectionInterface,
  }) : _deviceSelection = TestTargetDeviceSelection(logger);

  final TestTargetDeviceSelection _deviceSelection;

  @override
  TestTargetDeviceSelection get deviceSelection => _deviceSelection;
}

class TestTargetDeviceSelection extends TargetDeviceSelection {
  TestTargetDeviceSelection(super.logger);

  List<String> input = <String>[];

  @override
  Future<String> readUserInput() async {
    // If only one value is provided for the input, continue
    // to return that one input value without popping
    //
    // If more than one input values are provided, we are simulating
    // the user selecting more than one option for a device, so we will pop
    // them out from the front
    if (input.length > 1) {
      return input.removeAt(0);
    }

    return input[0];
  }
}

class TestDeviceManager extends DeviceManager {
  TestDeviceManager({required this.logger, required this.platform}) : super(logger: logger);

  final Logger logger;
  final Platform platform;

  @override
  String? specifiedDeviceId;

  @override
  bool hasSpecifiedAllDevices = false;

  final TestPollingDeviceDiscovery androidDiscoverer = TestPollingDeviceDiscovery('android');
  final TestPollingDeviceDiscovery otherDiscoverer = TestPollingDeviceDiscovery('other');
  late final TestIOSDeviceDiscovery iosDiscoverer = TestIOSDeviceDiscovery(
    platform: platform,
    xcdevice: FakeXcdevice(),
    iosWorkflow: FakeIOSWorkflow(),
    logger: logger,
  );

  @override
  List<DeviceDiscovery> get deviceDiscoverers {
    return <DeviceDiscovery>[androidDiscoverer, otherDiscoverer, iosDiscoverer];
  }

  void setDeviceToWaitFor(IOSDevice device, DeviceConnectionInterface connectionInterface) {
    final XCDeviceEventInterface eventInterface =
        connectionInterface == DeviceConnectionInterface.wireless
            ? XCDeviceEventInterface.wifi
            : XCDeviceEventInterface.usb;
    iosDiscoverer.xcdevice.waitForDeviceEvent = XCDeviceEventNotification(
      XCDeviceEvent.attach,
      eventInterface,
      device.id,
    );
  }
}

class TestPollingDeviceDiscovery extends PollingDeviceDiscovery {
  TestPollingDeviceDiscovery(super.name);

  List<Device> deviceList = <Device>[];
  List<Device> refreshDeviceList = <Device>[];
  int devicesCalled = 0;
  int discoverDevicesCalled = 0;
  int numberOfTimesPolled = 0;

  @override
  bool get supportsPlatform => true;

  @override
  List<String> get wellKnownIds => const <String>[];

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    numberOfTimesPolled++;
    return deviceList;
  }

  @override
  Future<List<Device>> devices({DeviceDiscoveryFilter? filter}) async {
    devicesCalled += 1;
    return super.devices(filter: filter);
  }

  @override
  Future<List<Device>> discoverDevices({Duration? timeout, DeviceDiscoveryFilter? filter}) {
    discoverDevicesCalled++;
    if (refreshDeviceList.isNotEmpty) {
      deviceList = refreshDeviceList;
    }
    return super.discoverDevices(timeout: timeout, filter: filter);
  }

  @override
  bool get canListAnything => true;
}

class TestIOSDeviceDiscovery extends IOSDevices {
  TestIOSDeviceDiscovery({
    required super.platform,
    required FakeXcdevice xcdevice,
    required super.iosWorkflow,
    required super.logger,
  }) : _platform = platform,
       _xcdevice = xcdevice,
       super(xcdevice: xcdevice);

  final Platform _platform;
  List<Device> deviceList = <Device>[];
  List<Device> refreshDeviceList = <Device>[];
  int devicesCalled = 0;
  int discoverDevicesCalled = 0;
  int numberOfTimesPolled = 0;

  final FakeXcdevice _xcdevice;

  @override
  FakeXcdevice get xcdevice => _xcdevice;

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    numberOfTimesPolled++;
    if (!_platform.isMacOS) {
      throw UnsupportedError('Control of iOS devices or simulators only supported on macOS.');
    }
    return deviceList;
  }

  @override
  Future<List<Device>> devices({DeviceDiscoveryFilter? filter}) async {
    devicesCalled += 1;
    return super.devices(filter: filter);
  }

  @override
  Future<List<Device>> discoverDevices({Duration? timeout, DeviceDiscoveryFilter? filter}) {
    discoverDevicesCalled++;
    if (refreshDeviceList.isNotEmpty) {
      deviceList = refreshDeviceList;
    }
    return super.discoverDevices(timeout: timeout, filter: filter);
  }

  @override
  bool get canListAnything => true;
}

class FakeXcdevice extends Fake implements XCDevice {
  XCDeviceEventNotification? waitForDeviceEvent;

  bool waitedForDeviceToConnect = false;

  @override
  Future<XCDeviceEventNotification?> waitForDeviceToConnect(String deviceId) async {
    final XCDeviceEventNotification? waitEvent = waitForDeviceEvent;
    if (waitEvent != null) {
      waitedForDeviceToConnect = true;
      return XCDeviceEventNotification(
        waitEvent.eventType,
        waitEvent.eventInterface,
        waitEvent.deviceIdentifier,
      );
    } else {
      return null;
    }
  }

  @override
  void cancelWaitForDeviceToConnect() {}
}

class FakeIOSWorkflow extends Fake implements IOSWorkflow {}

class FakeDevice extends Fake implements Device {
  FakeDevice({
    String? deviceId,
    String? deviceName,
    bool deviceSupported = true,
    bool deviceSupportForProject = true,
    this.ephemeral = true,
    this.isConnected = true,
    this.connectionInterface = DeviceConnectionInterface.attached,
    this.platformType = PlatformType.android,
    TargetPlatform deviceTargetPlatform = TargetPlatform.android,
  }) : id = deviceId ?? 'xxx',
       name = deviceName ?? 'test',
       _isSupported = deviceSupported,
       _isSupportedForProject = deviceSupportForProject,
       _targetPlatform = deviceTargetPlatform;

  FakeDevice.wireless({
    String? deviceId,
    String? deviceName,
    bool deviceSupported = true,
    bool deviceSupportForProject = true,
    this.ephemeral = true,
    this.isConnected = true,
    this.connectionInterface = DeviceConnectionInterface.wireless,
    this.platformType = PlatformType.android,
    TargetPlatform deviceTargetPlatform = TargetPlatform.android,
  }) : id = deviceId ?? 'xxx',
       name = deviceName ?? 'test',
       _isSupported = deviceSupported,
       _isSupportedForProject = deviceSupportForProject,
       _targetPlatform = deviceTargetPlatform;

  FakeDevice.fuchsia({
    String? deviceId,
    String? deviceName,
    bool deviceSupported = true,
    bool deviceSupportForProject = true,
    this.ephemeral = true,
    this.isConnected = true,
    this.connectionInterface = DeviceConnectionInterface.attached,
    this.platformType = PlatformType.fuchsia,
    TargetPlatform deviceTargetPlatform = TargetPlatform.fuchsia_arm64,
  }) : id = deviceId ?? 'xxx',
       name = deviceName ?? 'test',
       _isSupported = deviceSupported,
       _isSupportedForProject = deviceSupportForProject,
       _targetPlatform = deviceTargetPlatform,
       _sdkNameAndVersion = 'tester';

  final bool _isSupported;
  final bool _isSupportedForProject;
  final TargetPlatform _targetPlatform;
  String _sdkNameAndVersion = 'Android 10';

  @override
  String name;

  @override
  String get displayName {
    String result = name;
    if (isWirelesslyConnected) {
      result += ' (wireless)';
    }
    return result;
  }

  @override
  final bool ephemeral;

  @override
  String id;

  @override
  bool isSupported() => _isSupported;

  @override
  bool isSupportedForProject(FlutterProject project) => _isSupportedForProject;

  @override
  DeviceConnectionInterface connectionInterface;

  @override
  bool get isWirelesslyConnected => connectionInterface == DeviceConnectionInterface.wireless;

  @override
  bool isConnected;

  @override
  Future<TargetPlatform> get targetPlatform async => _targetPlatform;

  @override
  final PlatformType? platformType;

  @override
  Future<String> get sdkNameAndVersion async => _sdkNameAndVersion;

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  Category? get category => Category.mobile;

  @override
  Future<String> get targetPlatformDisplayName async =>
      getNameForTargetPlatform(await targetPlatform);
}

class FakeIOSDevice extends Fake implements IOSDevice {
  FakeIOSDevice({
    String? deviceId,
    String? deviceName,
    bool deviceSupported = true,
    bool deviceSupportForProject = true,
    this.ephemeral = true,
    this.isConnected = true,
    this.devModeEnabled = true,
    this.isPaired = true,
    this.platformType = PlatformType.ios,
    this.connectionInterface = DeviceConnectionInterface.attached,
  }) : id = deviceId ?? 'xxx',
       name = deviceName ?? 'test',
       _isSupported = deviceSupported,
       _isSupportedForProject = deviceSupportForProject;

  FakeIOSDevice.notConnectedWireless({
    String? deviceId,
    String? deviceName,
    bool deviceSupported = true,
    bool deviceSupportForProject = true,
    this.ephemeral = true,
    this.isConnected = false,
    this.platformType = PlatformType.ios,
    this.devModeEnabled = true,
    this.isPaired = true,
    this.connectionInterface = DeviceConnectionInterface.wireless,
  }) : id = deviceId ?? 'xxx',
       name = deviceName ?? 'test',
       _isSupported = deviceSupported,
       _isSupportedForProject = deviceSupportForProject;

  FakeIOSDevice.connectedWireless({
    String? deviceId,
    String? deviceName,
    bool deviceSupported = true,
    bool deviceSupportForProject = true,
    this.ephemeral = true,
    this.isConnected = true,
    this.devModeEnabled = true,
    this.isPaired = true,
    this.platformType = PlatformType.ios,
    this.connectionInterface = DeviceConnectionInterface.wireless,
  }) : id = deviceId ?? 'xxx',
       name = deviceName ?? 'test',
       _isSupported = deviceSupported,
       _isSupportedForProject = deviceSupportForProject;

  final bool _isSupported;
  final bool _isSupportedForProject;

  @override
  String name;

  @override
  String get displayName {
    String result = name;
    if (isWirelesslyConnected) {
      result += ' (wireless)';
    }
    return result;
  }

  @override
  final bool ephemeral;

  @override
  final bool devModeEnabled;

  @override
  final bool isPaired;

  @override
  String id;

  @override
  bool isSupported() => _isSupported;

  @override
  bool isSupportedForProject(FlutterProject project) => _isSupportedForProject;

  @override
  DeviceConnectionInterface connectionInterface;

  @override
  bool get isWirelesslyConnected => connectionInterface == DeviceConnectionInterface.wireless;

  @override
  bool isConnected;

  @override
  final PlatformType? platformType;

  @override
  Future<String> get sdkNameAndVersion async => 'iOS 16';

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  Category? get category => Category.mobile;

  @override
  Future<String> get targetPlatformDisplayName async => 'ios';

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.tester;
}

class FakeTerminal extends Fake implements AnsiTerminal {
  FakeTerminal({this.stdinHasTerminal = true, this.supportsColor = false});

  @override
  final bool stdinHasTerminal;

  @override
  final bool supportsColor;

  @override
  bool get isCliAnimationEnabled => supportsColor;

  @override
  bool usesTerminalUi = true;

  @override
  bool singleCharMode = false;

  void setPrompt(List<String> characters, String result) {
    _nextPrompt = characters;
    _nextResult = result;
  }

  List<String>? _nextPrompt;
  late String _nextResult;

  @override
  Future<String> promptForCharInput(
    List<String> acceptedCharacters, {
    Logger? logger,
    String? prompt,
    int? defaultChoiceIndex,
    bool displayAcceptedCharacters = true,
  }) async {
    expect(acceptedCharacters, _nextPrompt);
    return _nextResult;
  }

  @override
  String clearLines(int numberOfLines) {
    return 'CLEAR_LINES_$numberOfLines';
  }
}

class TestBufferLogger extends BufferLogger {
  TestBufferLogger.test({super.terminal, super.outputPreferences, super.verbose}) : super.test();

  String originalStatusText = '';

  @override
  void printStatus(
    String message, {
    bool? emphasis,
    TerminalColor? color,
    bool? newline,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    if (message.startsWith('CLEAR_LINES_')) {
      expect(statusText, equals(originalStatusText));
      final int numberOfLinesToRemove = int.parse(message.split('CLEAR_LINES_')[1]) - 1;
      final List<String> lines = LineSplitter.split(statusText).toList();
      // Clear string buffer and re-add lines not removed
      clear();
      for (int lineNumber = 0; lineNumber < lines.length - numberOfLinesToRemove; lineNumber++) {
        super.printStatus(lines[lineNumber]);
      }
    } else {
      super.printStatus(
        message,
        emphasis: emphasis,
        color: color,
        newline: newline,
        indent: indent,
        hangingIndent: hangingIndent,
        wrap: wrap,
      );
    }
  }
}

class FakeDoctor extends Fake implements Doctor {
  FakeDoctor({this.canLaunchAnything = true});

  @override
  bool canLaunchAnything;
}
