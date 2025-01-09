// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/android_workflow.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/emulator.dart';
import 'package:flutter_tools/src/ios/ios_emulators.dart';
import 'package:flutter_tools/src/macos/xcode.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart';

const FakeEmulator emulator1 = FakeEmulator('Nexus_5', 'Nexus 5', 'Google');
const FakeEmulator emulator2 = FakeEmulator('Nexus_5X_API_27_x86', 'Nexus 5X', 'Google');
const FakeEmulator emulator3 = FakeEmulator('iOS Simulator', 'iOS Simulator', 'Apple');
const List<Emulator> emulators = <Emulator>[emulator1, emulator2, emulator3];

// We have to send a command that fails in order to get the list of valid
// system images paths. This is an example of the output to use in the fake.
const String fakeCreateFailureOutput =
    'Error: Package path (-k) not specified. Valid system image paths are:\n'
    'system-images;android-27;google_apis;x86\n'
    'system-images;android-P;google_apis;x86\n'
    'system-images;android-27;google_apis_playstore;x86\n'
    'null\n'; // Yep, these really end with null (on dantup's machine at least)

const FakeCommand kListEmulatorsCommand = FakeCommand(
  command: <String>['avdmanager', 'create', 'avd', '-n', 'temp'],
  stderr: fakeCreateFailureOutput,
  exitCode: 1,
);

void main() {
  late FakeProcessManager fakeProcessManager;
  late FakeAndroidSdk sdk;
  late FileSystem fileSystem;
  late Xcode xcode;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    fakeProcessManager = FakeProcessManager.empty();
    sdk = FakeAndroidSdk();
    xcode = Xcode.test(processManager: fakeProcessManager, fileSystem: fileSystem);

    sdk
      ..avdManagerPath = 'avdmanager'
      ..emulatorPath = 'emulator'
      ..adbPath = 'adb';
  });

  group('EmulatorManager', () {
    // iOS discovery uses context.
    testUsingContext('getEmulators', () async {
      // Test that EmulatorManager.getEmulators() doesn't throw.
      final EmulatorManager emulatorManager = EmulatorManager(
        java: FakeJava(),
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(command: <String>['emulator', '-list-avds'], stdout: 'existing-avd-1'),
        ]),
        androidSdk: sdk,
        androidWorkflow: AndroidWorkflow(androidSdk: sdk, featureFlags: TestFeatureFlags()),
      );

      await expectLater(() async => emulatorManager.getAllAvailableEmulators(), returnsNormally);
    });

    testUsingContext('printEmulators prints the emulators information with header', () {
      Emulator.printEmulators(emulators, testLogger);

      expect(testLogger.statusText, '''
Id                  • Name          • Manufacturer • Platform

Nexus_5             • Nexus 5       • Google       • android
Nexus_5X_API_27_x86 • Nexus 5X      • Google       • android
iOS Simulator       • iOS Simulator • Apple        • android
''');
    });

    testUsingContext('getEmulators with no Android SDK', () async {
      // Test that EmulatorManager.getEmulators() doesn't throw when there's no Android SDK.
      final EmulatorManager emulatorManager = EmulatorManager(
        java: FakeJava(),
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(command: <String>['emulator', '-list-avds'], stdout: 'existing-avd-1'),
        ]),
        androidWorkflow: AndroidWorkflow(androidSdk: sdk, featureFlags: TestFeatureFlags()),
      );

      await expectLater(() async => emulatorManager.getAllAvailableEmulators(), returnsNormally);
    });

    testWithoutContext('getEmulatorsById', () async {
      final TestEmulatorManager testEmulatorManager = TestEmulatorManager(
        emulators,
        java: FakeJava(),
        logger: BufferLogger.test(),
        processManager: fakeProcessManager,
        androidWorkflow: AndroidWorkflow(androidSdk: sdk, featureFlags: TestFeatureFlags()),
        fileSystem: fileSystem,
      );

      expect(await testEmulatorManager.getEmulatorsMatching('Nexus_5'), <Emulator>[emulator1]);
      expect(await testEmulatorManager.getEmulatorsMatching('Nexus_5X'), <Emulator>[emulator2]);
      expect(await testEmulatorManager.getEmulatorsMatching('Nexus_5X_API_27_x86'), <Emulator>[
        emulator2,
      ]);
      expect(await testEmulatorManager.getEmulatorsMatching('Nexus'), <Emulator>[
        emulator1,
        emulator2,
      ]);
      expect(await testEmulatorManager.getEmulatorsMatching('iOS Simulator'), <Emulator>[
        emulator3,
      ]);
      expect(await testEmulatorManager.getEmulatorsMatching('ios'), <Emulator>[emulator3]);
    });

    testUsingContext('create emulator with a missing avdmanager does not crash.', () async {
      sdk.avdManagerPath = null;
      final EmulatorManager emulatorManager = EmulatorManager(
        java: FakeJava(),
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(command: <String>['emulator', '-list-avds'], stdout: 'existing-avd-1'),
        ]),
        androidSdk: sdk,
        androidWorkflow: AndroidWorkflow(androidSdk: sdk, featureFlags: TestFeatureFlags()),
      );
      final CreateEmulatorResult result = await emulatorManager.createEmulator();

      expect(result.success, false);
      expect(result.error, contains('avdmanager is missing from the Android SDK'));
    });

    // iOS discovery uses context.
    testUsingContext('create emulator with an empty name does not fail', () async {
      final EmulatorManager emulatorManager = EmulatorManager(
        java: FakeJava(),
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(command: <String>['emulator', '-list-avds'], stdout: 'existing-avd-1'),
          const FakeCommand(
            command: <String>['avdmanager', 'list', 'device', '-c'],
            stdout: 'test\ntest2\npixel\npixel-xl\n',
          ),
          kListEmulatorsCommand,
          const FakeCommand(
            command: <String>[
              'avdmanager',
              'create',
              'avd',
              '-n',
              'flutter_emulator',
              '-k',
              'system-images;android-27;google_apis_playstore;x86',
              '-d',
              'pixel',
            ],
          ),
        ]),
        androidSdk: sdk,
        androidWorkflow: AndroidWorkflow(androidSdk: sdk, featureFlags: TestFeatureFlags()),
      );
      final CreateEmulatorResult result = await emulatorManager.createEmulator();

      expect(result.success, true);
    });

    testWithoutContext('create emulator with a unique name does not throw', () async {
      final EmulatorManager emulatorManager = EmulatorManager(
        java: FakeJava(),
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>['avdmanager', 'list', 'device', '-c'],
            stdout: 'test\ntest2\npixel\npixel-xl\n',
          ),
          kListEmulatorsCommand,
          const FakeCommand(
            command: <String>[
              'avdmanager',
              'create',
              'avd',
              // The specified name is given with the -n flag.
              '-n',
              'test',
              '-k',
              'system-images;android-27;google_apis_playstore;x86',
              '-d',
              'pixel',
            ],
          ),
        ]),
        androidSdk: sdk,
        androidWorkflow: AndroidWorkflow(androidSdk: sdk, featureFlags: TestFeatureFlags()),
      );
      final CreateEmulatorResult result = await emulatorManager.createEmulator(name: 'test');

      expect(result.success, true);
    });

    testWithoutContext('create emulator with an existing name errors', () async {
      final EmulatorManager emulatorManager = EmulatorManager(
        java: FakeJava(),
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>['avdmanager', 'list', 'device', '-c'],
            stdout: 'test\ntest2\npixel\npixel-xl\n',
          ),
          kListEmulatorsCommand,
          const FakeCommand(
            command: <String>[
              'avdmanager',
              'create',
              'avd',
              '-n',
              'existing-avd-1',
              '-k',
              'system-images;android-27;google_apis_playstore;x86',
              '-d',
              'pixel',
            ],
            exitCode: 1,
            stderr:
                "Error: Android Virtual Device 'existing-avd-1' already exists.\n"
                'Use --force if you want to replace it.',
          ),
        ]),
        androidSdk: sdk,
        androidWorkflow: AndroidWorkflow(androidSdk: sdk, featureFlags: TestFeatureFlags()),
      );
      final CreateEmulatorResult result = await emulatorManager.createEmulator(
        name: 'existing-avd-1',
      );

      expect(result.success, false);
    });

    // iOS discovery uses context.
    testUsingContext(
      'create emulator without a name but when default exists adds a suffix',
      () async {
        final EmulatorManager emulatorManager = EmulatorManager(
          java: FakeJava(),
          fileSystem: MemoryFileSystem.test(),
          logger: BufferLogger.test(),
          processManager: FakeProcessManager.list(<FakeCommand>[
            const FakeCommand(
              command: <String>['emulator', '-list-avds'],
              stdout: 'existing-avd-1\nflutter_emulator',
            ),
            const FakeCommand(
              command: <String>['avdmanager', 'list', 'device', '-c'],
              stdout: 'test\ntest2\npixel\npixel-xl\n',
            ),
            kListEmulatorsCommand,
            const FakeCommand(
              command: <String>[
                'avdmanager',
                'create',
                'avd',
                // a "_2" suffix is added to disambiguate from the existing emulator.
                '-n',
                'flutter_emulator_2',
                '-k',
                'system-images;android-27;google_apis_playstore;x86',
                '-d',
                'pixel',
              ],
            ),
          ]),
          androidSdk: sdk,
          androidWorkflow: AndroidWorkflow(androidSdk: sdk, featureFlags: TestFeatureFlags()),
        );
        final CreateEmulatorResult result = await emulatorManager.createEmulator();

        expect(result.success, true);
        expect(result.emulatorName, 'flutter_emulator_2');
      },
    );
  });

  group('ios_emulators', () {
    testUsingContext(
      'runs correct launch commands',
      () async {
        fileSystem
            .directory('/fake/Xcode.app/Contents/Developer/Applications/Simulator.app')
            .createSync(recursive: true);
        fakeProcessManager.addCommands(<FakeCommand>[
          const FakeCommand(
            command: <String>['/usr/bin/xcode-select', '--print-path'],
            stdout: '/fake/Xcode.app/Contents/Developer',
          ),
          const FakeCommand(
            command: <String>[
              'open',
              '-n',
              '-a',
              '/fake/Xcode.app/Contents/Developer/Applications/Simulator.app',
            ],
          ),
          const FakeCommand(
            command: <String>[
              'open',
              '-a',
              '/fake/Xcode.app/Contents/Developer/Applications/Simulator.app',
            ],
          ),
        ]);

        const Emulator emulator = IOSEmulator('ios');
        await emulator.launch();
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        Xcode: () => xcode,
        FileSystem: () => fileSystem,
      },
    );
  });
}

class TestEmulatorManager extends EmulatorManager {
  TestEmulatorManager(
    this.allEmulators, {
    required super.java,
    required super.logger,
    required super.processManager,
    required super.androidWorkflow,
    required super.fileSystem,
  });

  final List<Emulator> allEmulators;

  @override
  Future<List<Emulator>> getAllAvailableEmulators() {
    return Future<List<Emulator>>.value(allEmulators);
  }
}

class FakeEmulator extends Emulator {
  const FakeEmulator(String id, this.name, this.manufacturer) : super(id, true);

  @override
  final String name;

  @override
  final String manufacturer;

  @override
  Category get category => Category.mobile;

  @override
  PlatformType get platformType => PlatformType.android;

  @override
  Future<void> launch({bool coldBoot = false}) {
    throw UnimplementedError('Not implemented in Mock');
  }
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  String? avdManagerPath;

  @override
  String? emulatorPath;

  @override
  String? adbPath;

  @override
  String? getAvdManagerPath() => avdManagerPath;

  @override
  String getAvdPath() => 'avd';
}
