// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/devices.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_devices.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  group('devices', () {
    setUpAll(() {
      Cache.disableLocking();
    });

    late Cache cache;
    late Platform platform;

    group('ensure factory', () {
      late FakeBufferLogger fakeLogger;

      setUpAll(() {
        fakeLogger = FakeBufferLogger();
      });

      testWithoutContext(
        'returns DevicesCommandOutputWithExtendedWirelessDeviceDiscovery on MacOS',
        () async {
          final Platform platform = FakePlatform(operatingSystem: 'macos');
          final DevicesCommandOutput devicesCommandOutput = DevicesCommandOutput(
            platform: platform,
            logger: fakeLogger,
          );

          expect(
            devicesCommandOutput is DevicesCommandOutputWithExtendedWirelessDeviceDiscovery,
            true,
          );
        },
      );

      testWithoutContext('returns default when not on MacOS', () async {
        final Platform platform = FakePlatform();
        final DevicesCommandOutput devicesCommandOutput = DevicesCommandOutput(
          platform: platform,
          logger: fakeLogger,
        );

        expect(
          devicesCommandOutput is DevicesCommandOutputWithExtendedWirelessDeviceDiscovery,
          false,
        );
      });
    });

    group('when Platform is not MacOS', () {
      setUp(() {
        cache = Cache.test(processManager: FakeProcessManager.any());
        platform = FakePlatform();
      });

      testUsingContext(
        'returns 0 when called',
        () async {
          final DevicesCommand command = DevicesCommand();
          await createTestCommandRunner(command).run(<String>['devices']);
        },
        overrides: <Type, Generator>{Cache: () => cache, Artifacts: () => Artifacts.test()},
      );

      testUsingContext(
        'no error when no connected devices',
        () async {
          final DevicesCommand command = DevicesCommand();
          await createTestCommandRunner(command).run(<String>['devices']);
          expect(
            testLogger.statusText,
            equals('''
No authorized devices detected.

Run "flutter emulators" to list and start any available device emulators.

If you expected a device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
'''),
          );
        },
        overrides: <Type, Generator>{
          AndroidSdk: () => null,
          DeviceManager: () => NoDevicesManager(),
          ProcessManager: () => FakeProcessManager.any(),
          Cache: () => cache,
          Artifacts: () => Artifacts.test(),
          Platform: () => platform,
        },
      );

      group('when includes both attached and wireless devices', () {
        List<FakeDeviceJsonData>? deviceList;
        setUp(() {
          deviceList = <FakeDeviceJsonData>[fakeDevices[0], fakeDevices[1], fakeDevices[2]];
        });

        testUsingContext(
          "get devices' platform types",
          () async {
            final List<String> platformTypes = Device.devicesPlatformTypes(
              await globals.deviceManager!.getAllDevices(),
            );
            expect(platformTypes, <String>['android', 'web']);
          },
          overrides: <Type, Generator>{
            DeviceManager: () => _FakeDeviceManager(devices: deviceList),
            ProcessManager: () => FakeProcessManager.any(),
            Cache: () => cache,
            Artifacts: () => Artifacts.test(),
            Platform: () => platform,
          },
        );

        group('with --machine flag', () {
          testUsingContext(
            'Outputs parsable JSON',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(command).run(<String>['devices', '--machine']);
              expect(json.decode(testLogger.statusText), <Map<String, Object>>[
                fakeDevices[0].json,
                fakeDevices[1].json,
                fakeDevices[2].json,
              ]);
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList),
              ProcessManager: () => FakeProcessManager.any(),
              Cache: () => cache,
              Artifacts: () => Artifacts.test(),
              Platform: () => platform,
            },
          );

          group('when deviceConnectionInterface', () {
            testUsingContext(
              'filtered to attached',
              () async {
                final DevicesCommand command = DevicesCommand();
                await createTestCommandRunner(
                  command,
                ).run(<String>['devices', '--machine', '--device-connection', 'attached']);
                expect(json.decode(testLogger.statusText), <Map<String, Object>>[
                  fakeDevices[0].json,
                  fakeDevices[1].json,
                ]);
              },
              overrides: <Type, Generator>{
                DeviceManager: () => _FakeDeviceManager(devices: deviceList),
                ProcessManager: () => FakeProcessManager.any(),
                Cache: () => cache,
                Artifacts: () => Artifacts.test(),
                Platform: () => platform,
              },
            );

            testUsingContext(
              'filtered to wireless',
              () async {
                final DevicesCommand command = DevicesCommand();
                await createTestCommandRunner(
                  command,
                ).run(<String>['devices', '--machine', '--device-connection', 'wireless']);
                expect(json.decode(testLogger.statusText), <Map<String, Object>>[
                  fakeDevices[2].json,
                ]);
              },
              overrides: <Type, Generator>{
                DeviceManager: () => _FakeDeviceManager(devices: deviceList),
                ProcessManager: () => FakeProcessManager.any(),
                Cache: () => cache,
                Artifacts: () => Artifacts.test(),
                Platform: () => platform,
              },
            );
          });
        });

        testUsingContext(
          'available devices and diagnostics',
          () async {
            final DevicesCommand command = DevicesCommand();
            await createTestCommandRunner(command).run(<String>['devices']);
            expect(testLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Found 1 wirelessly connected device:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
          },
          overrides: <Type, Generator>{
            DeviceManager: () => _FakeDeviceManager(devices: deviceList),
            ProcessManager: () => FakeProcessManager.any(),
            Platform: () => platform,
          },
        );

        group('when deviceConnectionInterface', () {
          testUsingContext(
            'filtered to attached',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(
                command,
              ).run(<String>['devices', '--device-connection', 'attached']);
              expect(testLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
            },
          );

          testUsingContext(
            'filtered to wireless',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(
                command,
              ).run(<String>['devices', '--device-connection', 'wireless']);
              expect(testLogger.statusText, '''
Found 1 wirelessly connected device:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
            },
          );
        });
      });

      group('when includes only attached devices', () {
        List<FakeDeviceJsonData>? deviceList;
        setUp(() {
          deviceList = <FakeDeviceJsonData>[fakeDevices[0], fakeDevices[1]];
        });

        testUsingContext(
          'available devices and diagnostics',
          () async {
            final DevicesCommand command = DevicesCommand();
            await createTestCommandRunner(command).run(<String>['devices']);
            expect(testLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
          },
          overrides: <Type, Generator>{
            DeviceManager: () => _FakeDeviceManager(devices: deviceList),
            ProcessManager: () => FakeProcessManager.any(),
            Platform: () => platform,
          },
        );
      });

      group('when includes only wireless devices', () {
        List<FakeDeviceJsonData>? deviceList;
        setUp(() {
          deviceList = <FakeDeviceJsonData>[fakeDevices[2]];
        });

        testUsingContext(
          'available devices and diagnostics',
          () async {
            final DevicesCommand command = DevicesCommand();
            await createTestCommandRunner(command).run(<String>['devices']);
            expect(testLogger.statusText, '''
Found 1 wirelessly connected device:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
          },
          overrides: <Type, Generator>{
            DeviceManager: () => _FakeDeviceManager(devices: deviceList),
            ProcessManager: () => FakeProcessManager.any(),
            Platform: () => platform,
          },
        );
      });
    });

    group('when Platform is MacOS', () {
      setUp(() {
        cache = Cache.test(processManager: FakeProcessManager.any());
        platform = FakePlatform(operatingSystem: 'macos');
      });

      testUsingContext(
        'returns 0 when called',
        () async {
          final DevicesCommand command = DevicesCommand();
          await createTestCommandRunner(command).run(<String>['devices']);
        },
        overrides: <Type, Generator>{
          Cache: () => cache,
          Artifacts: () => Artifacts.test(),
          Platform: () => platform,
        },
      );

      group('when no connected devices', () {
        testUsingContext(
          'no error',
          () async {
            final DevicesCommand command = DevicesCommand();
            await createTestCommandRunner(command).run(<String>['devices']);
            expect(
              testLogger.statusText,
              equals('''
No devices found yet. Checking for wireless devices...

No authorized devices detected.

Run "flutter emulators" to list and start any available device emulators.

If you expected a device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
'''),
            );
          },
          overrides: <Type, Generator>{
            AndroidSdk: () => null,
            DeviceManager: () => NoDevicesManager(),
            ProcessManager: () => FakeProcessManager.any(),
            Cache: () => cache,
            Artifacts: () => Artifacts.test(),
            Platform: () => platform,
          },
        );

        group('when deviceConnectionInterface', () {
          testUsingContext(
            'filtered to attached',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(
                command,
              ).run(<String>['devices', '--device-connection', 'attached']);
              expect(testLogger.statusText, '''
No authorized devices detected.

Run "flutter emulators" to list and start any available device emulators.

If you expected a device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => NoDevicesManager(),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
            },
          );

          testUsingContext(
            'filtered to wireless',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(
                command,
              ).run(<String>['devices', '--device-connection', 'wireless']);
              expect(testLogger.statusText, '''
Checking for wireless devices...

No authorized devices detected.

Run "flutter emulators" to list and start any available device emulators.

If you expected a device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => NoDevicesManager(),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
            },
          );
        });
      });

      group('when includes both attached and wireless devices', () {
        List<FakeDeviceJsonData>? deviceList;
        setUp(() {
          deviceList = <FakeDeviceJsonData>[
            fakeDevices[0],
            fakeDevices[1],
            fakeDevices[2],
            fakeDevices[3],
          ];
        });

        testUsingContext(
          "get devices' platform types",
          () async {
            final List<String> platformTypes = Device.devicesPlatformTypes(
              await globals.deviceManager!.getAllDevices(),
            );
            expect(platformTypes, <String>['android', 'ios', 'web']);
          },
          overrides: <Type, Generator>{
            DeviceManager: () => _FakeDeviceManager(devices: deviceList),
            ProcessManager: () => FakeProcessManager.any(),
            Cache: () => cache,
            Artifacts: () => Artifacts.test(),
            Platform: () => platform,
          },
        );

        group('with --machine flag', () {
          testUsingContext(
            'Outputs parsable JSON',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(command).run(<String>['devices', '--machine']);
              expect(json.decode(testLogger.statusText), <Map<String, Object>>[
                fakeDevices[0].json,
                fakeDevices[1].json,
                fakeDevices[2].json,
                fakeDevices[3].json,
              ]);
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList),
              ProcessManager: () => FakeProcessManager.any(),
              Cache: () => cache,
              Artifacts: () => Artifacts.test(),
              Platform: () => platform,
            },
          );

          group('when deviceConnectionInterface', () {
            testUsingContext(
              'filtered to attached',
              () async {
                final DevicesCommand command = DevicesCommand();
                await createTestCommandRunner(
                  command,
                ).run(<String>['devices', '--machine', '--device-connection', 'attached']);
                expect(json.decode(testLogger.statusText), <Map<String, Object>>[
                  fakeDevices[0].json,
                  fakeDevices[1].json,
                ]);
              },
              overrides: <Type, Generator>{
                DeviceManager: () => _FakeDeviceManager(devices: deviceList),
                ProcessManager: () => FakeProcessManager.any(),
                Cache: () => cache,
                Artifacts: () => Artifacts.test(),
                Platform: () => platform,
              },
            );

            testUsingContext(
              'filtered to wireless',
              () async {
                final DevicesCommand command = DevicesCommand();
                await createTestCommandRunner(
                  command,
                ).run(<String>['devices', '--machine', '--device-connection', 'wireless']);
                expect(json.decode(testLogger.statusText), <Map<String, Object>>[
                  fakeDevices[2].json,
                  fakeDevices[3].json,
                ]);
              },
              overrides: <Type, Generator>{
                DeviceManager: () => _FakeDeviceManager(devices: deviceList),
                ProcessManager: () => FakeProcessManager.any(),
                Cache: () => cache,
                Artifacts: () => Artifacts.test(),
                Platform: () => platform,
              },
            );
          });
        });

        testUsingContext(
          'available devices and diagnostics',
          () async {
            final DevicesCommand command = DevicesCommand();
            await createTestCommandRunner(command).run(<String>['devices']);
            expect(testLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Checking for wireless devices...

Found 2 wirelessly connected devices:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)
  wireless ios (mobile)     • wireless-ios     • ios         • iOS 16 (simulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
          },
          overrides: <Type, Generator>{
            DeviceManager: () => _FakeDeviceManager(devices: deviceList),
            ProcessManager: () => FakeProcessManager.any(),
            Platform: () => platform,
          },
        );

        group('with ansi terminal', () {
          late FakeTerminal terminal;
          late FakeBufferLogger fakeLogger;

          setUp(() {
            terminal = FakeTerminal(supportsColor: true);
            fakeLogger = FakeBufferLogger(terminal: terminal);
            fakeLogger.originalStatusText = '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Checking for wireless devices...
''';
          });

          testUsingContext(
            'available devices and diagnostics',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(command).run(<String>['devices']);

              expect(fakeLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Found 2 wirelessly connected devices:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)
  wireless ios (mobile)     • wireless-ios     • ios         • iOS 16 (simulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList, logger: fakeLogger),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
              AnsiTerminal: () => terminal,
              Logger: () => fakeLogger,
            },
          );
        });

        group('with verbose logging', () {
          late FakeBufferLogger fakeLogger;

          setUp(() {
            fakeLogger = FakeBufferLogger(verbose: true);
          });

          testUsingContext(
            'available devices and diagnostics',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(command).run(<String>['devices']);

              expect(fakeLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Checking for wireless devices...

Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Found 2 wirelessly connected devices:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)
  wireless ios (mobile)     • wireless-ios     • ios         • iOS 16 (simulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList, logger: fakeLogger),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
              Logger: () => fakeLogger,
            },
          );

          testUsingContext(
            'when deviceConnectionInterface filtered to wireless',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(
                command,
              ).run(<String>['devices', '--device-connection', 'wireless']);
              expect(testLogger.statusText, '''
Checking for wireless devices...

Found 2 wirelessly connected devices:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)
  wireless ios (mobile)     • wireless-ios     • ios         • iOS 16 (simulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
            },
          );
        });
      });

      group('when includes only attached devices', () {
        List<FakeDeviceJsonData>? deviceList;
        setUp(() {
          deviceList = <FakeDeviceJsonData>[fakeDevices[0], fakeDevices[1]];
        });

        testUsingContext(
          'available devices and diagnostics',
          () async {
            final DevicesCommand command = DevicesCommand();
            await createTestCommandRunner(command).run(<String>['devices']);
            expect(testLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Checking for wireless devices...

No wireless devices were found.

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
          },
          overrides: <Type, Generator>{
            DeviceManager: () => _FakeDeviceManager(devices: deviceList),
            ProcessManager: () => FakeProcessManager.any(),
            Platform: () => platform,
          },
        );

        group('with ansi terminal', () {
          late FakeTerminal terminal;
          late FakeBufferLogger fakeLogger;

          setUp(() {
            terminal = FakeTerminal(supportsColor: true);
            fakeLogger = FakeBufferLogger(terminal: terminal);
            fakeLogger.originalStatusText = '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Checking for wireless devices...
''';
          });

          testUsingContext(
            'available devices and diagnostics',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(command).run(<String>['devices']);

              expect(fakeLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

No wireless devices were found.

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList, logger: fakeLogger),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
              AnsiTerminal: () => terminal,
              Logger: () => fakeLogger,
            },
          );
        });

        group('with verbose logging', () {
          late FakeBufferLogger fakeLogger;

          setUp(() {
            fakeLogger = FakeBufferLogger(verbose: true);
          });

          testUsingContext(
            'available devices and diagnostics',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(command).run(<String>['devices']);

              expect(fakeLogger.statusText, '''
Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

Checking for wireless devices...

Found 2 connected devices:
  ephemeral (mobile) • ephemeral • android-arm    • Test SDK (1.2.3) (emulator)
  webby (mobile)     • webby     • web-javascript • Web SDK (1.2.4) (emulator)

No wireless devices were found.

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList, logger: fakeLogger),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
              Logger: () => fakeLogger,
            },
          );
        });
      });

      group('when includes only wireless devices', () {
        List<FakeDeviceJsonData>? deviceList;
        setUp(() {
          deviceList = <FakeDeviceJsonData>[fakeDevices[2], fakeDevices[3]];
        });

        testUsingContext(
          'available devices and diagnostics',
          () async {
            final DevicesCommand command = DevicesCommand();
            await createTestCommandRunner(command).run(<String>['devices']);
            expect(testLogger.statusText, '''
No devices found yet. Checking for wireless devices...

Found 2 wirelessly connected devices:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)
  wireless ios (mobile)     • wireless-ios     • ios         • iOS 16 (simulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
          },
          overrides: <Type, Generator>{
            DeviceManager: () => _FakeDeviceManager(devices: deviceList),
            ProcessManager: () => FakeProcessManager.any(),
            Platform: () => platform,
          },
        );

        group('with ansi terminal', () {
          late FakeTerminal terminal;
          late FakeBufferLogger fakeLogger;

          setUp(() {
            terminal = FakeTerminal(supportsColor: true);
            fakeLogger = FakeBufferLogger(terminal: terminal);
            fakeLogger.originalStatusText = '''
No devices found yet. Checking for wireless devices...
''';
          });

          testUsingContext(
            'available devices and diagnostics',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(command).run(<String>['devices']);

              expect(fakeLogger.statusText, '''
Found 2 wirelessly connected devices:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)
  wireless ios (mobile)     • wireless-ios     • ios         • iOS 16 (simulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList, logger: fakeLogger),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
              AnsiTerminal: () => terminal,
              Logger: () => fakeLogger,
            },
          );
        });

        group('with verbose logging', () {
          late FakeBufferLogger fakeLogger;

          setUp(() {
            fakeLogger = FakeBufferLogger(verbose: true);
          });

          testUsingContext(
            'available devices and diagnostics',
            () async {
              final DevicesCommand command = DevicesCommand();
              await createTestCommandRunner(command).run(<String>['devices']);

              expect(fakeLogger.statusText, '''
No devices found yet. Checking for wireless devices...

Found 2 wirelessly connected devices:
  wireless android (mobile) • wireless-android • android-arm • Test SDK (1.2.3) (emulator)
  wireless ios (mobile)     • wireless-ios     • ios         • iOS 16 (simulator)

Cannot connect to device ABC

Run "flutter emulators" to list and start any available device emulators.

If you expected another device to be detected, please run "flutter doctor" to diagnose potential issues. You may also try increasing the time to wait for connected devices with the "--device-timeout" flag. Visit https://flutter.dev/setup/ for troubleshooting tips.
''');
            },
            overrides: <Type, Generator>{
              DeviceManager: () => _FakeDeviceManager(devices: deviceList, logger: fakeLogger),
              ProcessManager: () => FakeProcessManager.any(),
              Platform: () => platform,
              Logger: () => fakeLogger,
            },
          );
        });
      });
    });
  });
}

class _FakeDeviceManager extends DeviceManager {
  _FakeDeviceManager({List<FakeDeviceJsonData>? devices, FakeBufferLogger? logger})
    : fakeDevices = devices ?? <FakeDeviceJsonData>[],
      super(logger: logger ?? testLogger);

  List<FakeDeviceJsonData> fakeDevices = <FakeDeviceJsonData>[];

  @override
  Future<List<Device>> getAllDevices({DeviceDiscoveryFilter? filter}) async {
    final DeviceConnectionInterface? interface = filter?.deviceConnectionInterface;
    return <Device>[
      for (final FakeDeviceJsonData deviceJson in fakeDevices)
        if (interface == null || deviceJson.dev.connectionInterface == interface) deviceJson.dev,
    ];
  }

  @override
  Future<List<Device>> refreshAllDevices({Duration? timeout, DeviceDiscoveryFilter? filter}) =>
      getAllDevices(filter: filter);

  @override
  Future<List<Device>> refreshExtendedWirelessDeviceDiscoverers({
    Duration? timeout,
    DeviceDiscoveryFilter? filter,
  }) => getAllDevices(filter: filter);

  @override
  Future<List<String>> getDeviceDiagnostics() =>
      Future<List<String>>.value(<String>['Cannot connect to device ABC']);

  @override
  List<DeviceDiscovery> get deviceDiscoverers => <DeviceDiscovery>[];
}

class NoDevicesManager extends DeviceManager {
  NoDevicesManager() : super(logger: testLogger);

  @override
  List<DeviceDiscovery> get deviceDiscoverers => <DeviceDiscovery>[];
}

class FakeTerminal extends Fake implements AnsiTerminal {
  FakeTerminal({this.supportsColor = false});

  @override
  final bool supportsColor;

  @override
  bool get isCliAnimationEnabled => supportsColor;

  @override
  bool singleCharMode = false;

  @override
  String clearLines(int numberOfLines) {
    return 'CLEAR_LINES_$numberOfLines';
  }
}

class FakeBufferLogger extends BufferLogger {
  FakeBufferLogger({super.terminal, super.outputPreferences, super.verbose}) : super.test();

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
