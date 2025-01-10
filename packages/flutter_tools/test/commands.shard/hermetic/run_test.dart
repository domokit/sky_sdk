// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:args/command_runner.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/daemon.dart';
import 'package:flutter_tools/src/commands/run.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/macos/macos_ipad_device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';
import 'package:flutter_tools/src/web/compile.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart' as analytics;
import 'package:vm_service/vm_service.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_devices.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  setUpAll(() {
    Cache.disableLocking();
  });

  group('run', () {
    late BufferLogger logger;
    late TestDeviceManager testDeviceManager;
    late FileSystem fileSystem;

    setUp(() {
      logger = BufferLogger.test();
      testDeviceManager = TestDeviceManager(logger: logger);
      fileSystem = MemoryFileSystem.test();
    });

    testUsingContext(
      'fails when target not found',
      () async {
        final RunCommand command = RunCommand();
        expect(
          () => createTestCommandRunner(command).run(<String>['run', '-t', 'abc123', '--no-pub']),
          throwsA(
            isA<ToolExit>().having(
              (ToolExit error) => error.exitCode,
              'exitCode',
              anyOf(isNull, 1),
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Logger: () => logger,
      },
    );

    testUsingContext(
      'does not support --no-sound-null-safety by default',
      () async {
        fileSystem.file('lib/main.dart').createSync(recursive: true);
        fileSystem.file('pubspec.yaml').createSync();
        fileSystem.file('.dart_tool/package_config.json').createSync(recursive: true);

        final TestRunCommandThatOnlyValidates command = TestRunCommandThatOnlyValidates();
        await expectLater(
          () => createTestCommandRunner(
            command,
          ).run(<String>['run', '--use-application-binary=app/bar/faz', '--no-sound-null-safety']),
          throwsA(
            isException.having(
              (Exception exception) => exception.toString(),
              'toString',
              contains('Could not find an option named "no-sound-null-safety"'),
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Logger: () => logger,
      },
    );

    testUsingContext(
      'supports --no-sound-null-safety with an overridden NonNullSafeBuilds',
      () async {
        fileSystem.file('lib/main.dart').createSync(recursive: true);
        fileSystem.file('pubspec.yaml').createSync();
        fileSystem.file('.dart_tool/package_config.json').createSync(recursive: true);

        final FakeDevice device = FakeDevice(
          isLocalEmulator: true,
          platformType: PlatformType.android,
        );

        testDeviceManager.devices = <Device>[device];
        final TestRunCommandThatOnlyValidates command = TestRunCommandThatOnlyValidates();
        await createTestCommandRunner(command).run(const <String>[
          'run',
          '--use-application-binary=app/bar/faz',
          '--no-sound-null-safety',
        ]);
      },
      overrides: <Type, Generator>{
        DeviceManager: () => testDeviceManager,
        FileSystem: () => fileSystem,
        Logger: () => logger,
        NonNullSafeBuilds: () => NonNullSafeBuilds.allowed,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'does not support "--use-application-binary" and "--fast-start"',
      () async {
        fileSystem.file('lib/main.dart').createSync(recursive: true);
        fileSystem.file('pubspec.yaml').createSync();
        fileSystem.file('.dart_tool/package_config.json').createSync(recursive: true);

        final RunCommand command = RunCommand();
        await expectLater(
          () => createTestCommandRunner(command).run(<String>[
            'run',
            '--use-application-binary=app/bar/faz',
            '--fast-start',
            '--no-pub',
            '--show-test-device',
          ]),
          throwsA(
            isException.having(
              (Exception exception) => exception.toString(),
              'toString',
              isNot(contains('--fast-start is not supported with --use-application-binary')),
            ),
          ),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Logger: () => logger,
      },
    );

    testUsingContext(
      'Walks upward looking for a pubspec.yaml and succeeds if found',
      () async {
        fileSystem.file('pubspec.yaml').createSync();
        fileSystem.file('.dart_tool/package_config.json')
          ..createSync(recursive: true)
          ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');
        fileSystem.file('lib/main.dart').createSync(recursive: true);
        fileSystem.currentDirectory = fileSystem.directory('a/b/c')..createSync(recursive: true);

        final RunCommand command = RunCommand();
        await expectLater(
          () => createTestCommandRunner(command).run(<String>['run', '--no-pub']),
          throwsToolExit(),
        );
        final BufferLogger bufferLogger = globals.logger as BufferLogger;
        expect(
          bufferLogger.statusText,
          containsIgnoringWhitespace('Changing current working directory to:'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Logger: () => logger,
      },
    );

    testUsingContext(
      'Walks upward looking for a pubspec.yaml and exits if missing',
      () async {
        fileSystem.currentDirectory = fileSystem.directory('a/b/c')..createSync(recursive: true);
        fileSystem.file('lib/main.dart').createSync(recursive: true);

        final RunCommand command = RunCommand();
        await expectLater(
          () => createTestCommandRunner(command).run(<String>['run', '--no-pub']),
          throwsToolExit(message: 'No pubspec.yaml file found'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Logger: () => logger,
      },
    );

    group('run app', () {
      late MemoryFileSystem fs;
      late Artifacts artifacts;
      late TestUsage usage;
      late FakeAnsiTerminal fakeTerminal;
      late analytics.FakeAnalytics fakeAnalytics;

      setUpAll(() {
        Cache.disableLocking();
      });

      setUp(() {
        fakeTerminal = FakeAnsiTerminal();
        artifacts = Artifacts.test();
        usage = TestUsage();
        fs = MemoryFileSystem.test();

        fs.currentDirectory.childFile('pubspec.yaml').writeAsStringSync('name: flutter_app');
        fs.currentDirectory.childDirectory('.dart_tool').childFile('package_config.json')
          ..createSync(recursive: true)
          ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');
        final Directory libDir = fs.currentDirectory.childDirectory('lib');
        libDir.createSync();
        final File mainFile = libDir.childFile('main.dart');
        mainFile.writeAsStringSync('void main() {}');
        fakeAnalytics = getInitializedFakeAnalyticsInstance(
          fs: fs,
          fakeFlutterVersion: FakeFlutterVersion(),
        );
      });

      testUsingContext(
        'exits with a user message when no supported devices attached',
        () async {
          final RunCommand command = RunCommand();
          testDeviceManager.devices = <Device>[];

          await expectLater(
            () => createTestCommandRunner(command).run(<String>['run', '--no-pub', '--no-hot']),
            throwsA(isA<ToolExit>().having((ToolExit error) => error.message, 'message', isNull)),
          );

          expect(
            testLogger.statusText,
            containsIgnoringWhitespace('No supported devices connected.'),
          );
        },
        overrides: <Type, Generator>{
          DeviceManager: () => testDeviceManager,
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        },
      );

      testUsingContext(
        'Using flutter run -d with MacOSDesignedForIPadDevices throws an error',
        () async {
          final RunCommand command = RunCommand();
          testDeviceManager.devices = <Device>[FakeMacDesignedForIpadDevice()];

          await expectLater(
            () => createTestCommandRunner(
              command,
            ).run(<String>['run', '-d', 'mac-designed-for-ipad']),
            throwsToolExit(
              message: 'Mac Designed for iPad is currently not supported for flutter run -d',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          DeviceManager: () => testDeviceManager,
          Stdio: () => FakeStdio(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        },
      );

      testUsingContext(
        'Using flutter run -d all with a single MacOSDesignedForIPadDevices throws a tool error',
        () async {
          final RunCommand command = RunCommand();
          testDeviceManager.devices = <Device>[FakeMacDesignedForIpadDevice()];

          await expectLater(
            () => createTestCommandRunner(command).run(<String>['run', '-d', 'all']),
            throwsToolExit(
              message: 'Mac Designed for iPad is currently not supported for flutter run -d',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          DeviceManager: () => testDeviceManager,
          Stdio: () => FakeStdio(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        },
      );

      testUsingContext(
        'Using flutter run -d all with MacOSDesignedForIPadDevices removes from device list, and attempts to launch',
        () async {
          final RunCommand command = TestRunCommandThatOnlyValidates();
          testDeviceManager.devices = <Device>[FakeMacDesignedForIpadDevice(), FakeDevice()];

          await createTestCommandRunner(command).run(<String>['run', '-d', 'all']);

          expect(command.devices?.length, 1);
          expect(command.devices?.single.id, 'fake_device');
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          DeviceManager: () => testDeviceManager,
          Stdio: () => FakeStdio(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        },
      );

      testUsingContext(
        'exits and lists available devices when specified device not found',
        () async {
          final RunCommand command = RunCommand();
          final FakeDevice device = FakeDevice(isLocalEmulator: true);
          testDeviceManager
            ..devices = <Device>[device]
            ..specifiedDeviceId = 'invalid-device-id';

          await expectLater(
            () => createTestCommandRunner(
              command,
            ).run(<String>['run', '-d', 'invalid-device-id', '--no-pub', '--no-hot']),
            throwsToolExit(),
          );
          expect(
            testLogger.statusText,
            contains("No supported devices found with name or id matching 'invalid-device-id'"),
          );
          expect(testLogger.statusText, contains('The following devices were found:'));
          expect(
            testLogger.statusText,
            contains('FakeDevice (mobile) • fake_device • ios •  (simulator)'),
          );
        },
        overrides: <Type, Generator>{
          DeviceManager: () => testDeviceManager,
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        },
      );

      testUsingContext(
        'fails when targeted device is not Android with --device-user',
        () async {
          final FakeDevice device = FakeDevice(isLocalEmulator: true);

          testDeviceManager.devices = <Device>[device];

          final TestRunCommandThatOnlyValidates command = TestRunCommandThatOnlyValidates();
          await expectLater(
            createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', '--device-user', '10']),
            throwsToolExit(
              message:
                  '--device-user is only supported for Android. At least one Android device is required.',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          DeviceManager: () => testDeviceManager,
          Stdio: () => FakeStdio(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        },
      );

      testUsingContext(
        'succeeds when targeted device is an Android device with --device-user',
        () async {
          final FakeDevice device = FakeDevice(
            isLocalEmulator: true,
            platformType: PlatformType.android,
          );

          testDeviceManager.devices = <Device>[device];

          final TestRunCommandThatOnlyValidates command = TestRunCommandThatOnlyValidates();
          await createTestCommandRunner(
            command,
          ).run(<String>['run', '--no-pub', '--device-user', '10']);
          // Finishes normally without error.
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          DeviceManager: () => testDeviceManager,
          Stdio: () => FakeStdio(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        },
      );

      testUsingContext(
        'shows unsupported devices when no supported devices are found',
        () async {
          final RunCommand command = RunCommand();
          final FakeDevice mockDevice = FakeDevice(
            targetPlatform: TargetPlatform.android_arm,
            isLocalEmulator: true,
            sdkNameAndVersion: 'api-14',
            isSupported: false,
          );
          testDeviceManager.devices = <Device>[mockDevice];

          await expectLater(
            () => createTestCommandRunner(command).run(<String>['run', '--no-pub', '--no-hot']),
            throwsA(isA<ToolExit>().having((ToolExit error) => error.message, 'message', isNull)),
          );

          expect(
            testLogger.statusText,
            containsIgnoringWhitespace('No supported devices connected.'),
          );
          expect(
            testLogger.statusText,
            containsIgnoringWhitespace(
              'The following devices were found, but are not supported by this project:',
            ),
          );
          expect(
            testLogger.statusText,
            containsIgnoringWhitespace(
              globals.userMessages.flutterMissPlatformProjects(
                Device.devicesPlatformTypes(<Device>[mockDevice]),
              ),
            ),
          );
        },
        overrides: <Type, Generator>{
          DeviceManager: () => testDeviceManager,
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        },
      );

      testUsingContext(
        'prints warning when --flavor is used with an unsupported target platform',
        () async {
          const List<String> runCommand = <String>[
            'run',
            '--no-pub',
            '--no-hot',
            '--flavor=vanilla',
            '-d',
            'all',
          ];
          // Useful for test readability.
          // ignore: avoid_redundant_argument_values
          final FakeDevice deviceWithoutFlavorSupport = FakeDevice(supportsFlavors: false);
          final FakeDevice deviceWithFlavorSupport = FakeDevice(supportsFlavors: true);
          testDeviceManager.devices = <Device>[deviceWithoutFlavorSupport, deviceWithFlavorSupport];

          await createTestCommandRunner(TestRunCommandThatOnlyValidates()).run(runCommand);

          expect(
            logger.warningText,
            contains(
              '--flavor is only supported for Android, macOS, and iOS devices. '
              'Flavor-related features may not function properly and could '
              'behave differently in a future release.',
            ),
          );
        },
        overrides: <Type, Generator>{
          DeviceManager: () => testDeviceManager,
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
          Logger: () => logger,
        },
      );

      testUsingContext(
        'forwards --uninstall-only to DebuggingOptions',
        () async {
          final RunCommand command = RunCommand();
          final FakeDevice mockDevice = FakeDevice(sdkNameAndVersion: 'iOS 13')
            ..startAppSuccess = false;

          testDeviceManager.devices = <Device>[mockDevice];

          // Causes swift to be detected in the analytics.
          fs.currentDirectory
              .childDirectory('ios')
              .childFile('AppDelegate.swift')
              .createSync(recursive: true);

          await expectToolExitLater(
            createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', '--no-hot', '--uninstall-first']),
            isNull,
          );

          final DebuggingOptions options = await command.createDebuggingOptions(false);
          expect(options.uninstallFirst, isTrue);
        },
        overrides: <Type, Generator>{
          Artifacts: () => artifacts,
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
          DeviceManager: () => testDeviceManager,
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          Usage: () => usage,
        },
      );

      testUsingContext(
        'passes device target platform to analytics',
        () async {
          final RunCommand command = RunCommand();
          final FakeDevice mockDevice = FakeDevice(sdkNameAndVersion: 'iOS 13')
            ..startAppSuccess = false;

          testDeviceManager.devices = <Device>[mockDevice];

          // Causes swift to be detected in the analytics.
          fs.currentDirectory
              .childDirectory('ios')
              .childFile('AppDelegate.swift')
              .createSync(recursive: true);

          await expectToolExitLater(
            createTestCommandRunner(command).run(<String>['run', '--no-pub', '--no-hot']),
            isNull,
          );

          expect(
            fakeAnalytics.sentEvents,
            contains(
              analytics.Event.commandUsageValues(
                workflow: 'run',
                commandHasTerminal: globals.stdio.hasTerminal,
                runIsEmulator: false,
                runTargetName: 'ios',
                runTargetOsVersion: 'iOS 13',
                runModeName: 'debug',
                runProjectModule: false,
                runProjectHostLanguage: 'swift',
                runIOSInterfaceType: 'usb',
                runIsTest: false,
              ),
            ),
          );
        },
        overrides: <Type, Generator>{
          AnsiTerminal: () => fakeTerminal,
          Artifacts: () => artifacts,
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
          DeviceManager: () => testDeviceManager,
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          Stdio: () => FakeStdio(),
          Usage: () => usage,
          analytics.Analytics: () => fakeAnalytics,
        },
      );

      testUsingContext(
        'correctly reports tests to analytics',
        () async {
          fs.currentDirectory
              .childDirectory('test')
              .childFile('widget_test.dart')
              .createSync(recursive: true);
          fs.currentDirectory
              .childDirectory('ios')
              .childFile('AppDelegate.swift')
              .createSync(recursive: true);
          final RunCommand command = RunCommand();
          final FakeDevice mockDevice = FakeDevice(sdkNameAndVersion: 'iOS 13')
            ..startAppSuccess = false;

          testDeviceManager.devices = <Device>[mockDevice];

          await expectToolExitLater(
            createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', '--no-hot', 'test/widget_test.dart']),
            isNull,
          );

          expect(
            fakeAnalytics.sentEvents,
            contains(
              analytics.Event.commandUsageValues(
                workflow: 'run',
                commandHasTerminal: globals.stdio.hasTerminal,
                runIsEmulator: false,
                runTargetName: 'ios',
                runTargetOsVersion: 'iOS 13',
                runModeName: 'debug',
                runProjectModule: false,
                runProjectHostLanguage: 'swift',
                runIOSInterfaceType: 'usb',
                runIsTest: true,
              ),
            ),
          );
        },
        overrides: <Type, Generator>{
          AnsiTerminal: () => fakeTerminal,
          Artifacts: () => artifacts,
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
          DeviceManager: () => testDeviceManager,
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          Stdio: () => FakeStdio(),
          Usage: () => usage,
          analytics.Analytics: () => fakeAnalytics,
        },
      );

      group('--machine', () {
        testUsingContext(
          'can pass --device-user',
          () async {
            final DaemonCapturingRunCommand command = DaemonCapturingRunCommand();
            final FakeDevice device = FakeDevice(platformType: PlatformType.android);
            testDeviceManager.devices = <Device>[device];

            await expectLater(
              () => createTestCommandRunner(command).run(<String>[
                'run',
                '--no-pub',
                '--machine',
                '--device-user',
                '10',
                '-d',
                device.id,
              ]),
              throwsToolExit(),
            );
            expect(command.appDomain.userIdentifier, '10');
          },
          overrides: <Type, Generator>{
            Artifacts: () => artifacts,
            Cache: () => Cache.test(processManager: FakeProcessManager.any()),
            DeviceManager: () => testDeviceManager,
            FileSystem: () => fs,
            ProcessManager: () => FakeProcessManager.any(),
            Usage: () => usage,
            Stdio: () => FakeStdio(),
            Logger: () => AppRunLogger(parent: logger),
          },
        );

        testUsingContext(
          'can disable devtools with --no-devtools',
          () async {
            final DaemonCapturingRunCommand command = DaemonCapturingRunCommand();
            final FakeDevice device = FakeDevice();
            testDeviceManager.devices = <Device>[device];

            await expectLater(
              () => createTestCommandRunner(
                command,
              ).run(<String>['run', '--no-pub', '--no-devtools', '--machine', '-d', device.id]),
              throwsToolExit(),
            );
            expect(command.appDomain.enableDevTools, isFalse);
          },
          overrides: <Type, Generator>{
            Artifacts: () => artifacts,
            Cache: () => Cache.test(processManager: FakeProcessManager.any()),
            DeviceManager: () => testDeviceManager,
            FileSystem: () => fs,
            ProcessManager: () => FakeProcessManager.any(),
            Usage: () => usage,
            Stdio: () => FakeStdio(),
            Logger: () => AppRunLogger(parent: logger),
          },
        );
      });
    });

    group('Fatal Logs', () {
      late TestRunCommandWithFakeResidentRunner command;
      late MemoryFileSystem fs;

      setUp(() {
        command = TestRunCommandWithFakeResidentRunner()..fakeResidentRunner = FakeResidentRunner();
        fs = MemoryFileSystem.test();
      });

      testUsingContext(
        "doesn't fail if --fatal-warnings specified and no warnings occur",
        () async {
          try {
            await createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', '--no-hot', '--${FlutterOptions.kFatalWarnings}']);
          } on Exception {
            fail('Unexpected exception thrown');
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        "doesn't fail if --fatal-warnings not specified",
        () async {
          testLogger.printWarning('Warning: Mild annoyance Will Robinson!');
          try {
            await createTestCommandRunner(command).run(<String>['run', '--no-pub', '--no-hot']);
          } on Exception {
            fail('Unexpected exception thrown');
          }
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'fails if --fatal-warnings specified and warnings emitted',
        () async {
          testLogger.printWarning('Warning: Mild annoyance Will Robinson!');
          await expectLater(
            createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', '--no-hot', '--${FlutterOptions.kFatalWarnings}']),
            throwsToolExit(
              message:
                  'Logger received warning output during the run, and "--${FlutterOptions.kFatalWarnings}" is enabled.',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'fails if --fatal-warnings specified and errors emitted',
        () async {
          testLogger.printError('Error: Danger Will Robinson!');
          await expectLater(
            createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', '--no-hot', '--${FlutterOptions.kFatalWarnings}']),
            throwsToolExit(
              message:
                  'Logger received error output during the run, and "--${FlutterOptions.kFatalWarnings}" is enabled.',
            ),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );
    });

    testUsingContext(
      'should only request artifacts corresponding to connected devices',
      () async {
        testDeviceManager.devices = <Device>[
          FakeDevice(targetPlatform: TargetPlatform.android_arm),
        ];

        expect(
          await RunCommand().requiredArtifacts,
          unorderedEquals(<DevelopmentArtifact>{
            DevelopmentArtifact.universal,
            DevelopmentArtifact.androidGenSnapshot,
          }),
        );

        testDeviceManager.devices = <Device>[FakeDevice()];

        expect(
          await RunCommand().requiredArtifacts,
          unorderedEquals(<DevelopmentArtifact>{
            DevelopmentArtifact.universal,
            DevelopmentArtifact.iOS,
          }),
        );

        testDeviceManager.devices = <Device>[
          FakeDevice(),
          FakeDevice(targetPlatform: TargetPlatform.android_arm),
        ];

        expect(
          await RunCommand().requiredArtifacts,
          unorderedEquals(<DevelopmentArtifact>{
            DevelopmentArtifact.universal,
            DevelopmentArtifact.iOS,
            DevelopmentArtifact.androidGenSnapshot,
          }),
        );

        testDeviceManager.devices = <Device>[
          FakeDevice(targetPlatform: TargetPlatform.web_javascript),
        ];

        expect(
          await RunCommand().requiredArtifacts,
          unorderedEquals(<DevelopmentArtifact>{
            DevelopmentArtifact.universal,
            DevelopmentArtifact.web,
          }),
        );
      },
      overrides: <Type, Generator>{
        DeviceManager: () => testDeviceManager,
        Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    group('usageValues', () {
      testUsingContext(
        'with only non-iOS usb device',
        () async {
          final List<Device> devices = <Device>[
            FakeDevice(
              targetPlatform: TargetPlatform.android_arm,
              platformType: PlatformType.android,
            ),
          ];
          final TestRunCommandForUsageValues command = TestRunCommandForUsageValues(
            devices: devices,
          );
          final CommandRunner<void> runner = createTestCommandRunner(command);
          try {
            // run the command so that CLI args are parsed
            await runner.run(<String>['run']);
          } on ToolExit catch (error) {
            // we can ignore the ToolExit, as we are only interested in
            // command.usageValues.
            expect(
              error,
              isA<ToolExit>().having(
                (ToolExit exception) => exception.message,
                'message',
                contains('No pubspec.yaml file found'),
              ),
            );
          }

          final CustomDimensions dimensions = await command.usageValues;

          expect(
            dimensions,
            const CustomDimensions(
              commandRunIsEmulator: false,
              commandRunTargetName: 'android-arm',
              commandRunTargetOsVersion: '',
              commandRunModeName: 'debug',
              commandRunProjectModule: false,
              commandRunProjectHostLanguage: '',
              commandRunIsTest: false,
            ),
          );
        },
        overrides: <Type, Generator>{
          DeviceManager: () => testDeviceManager,
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'with only iOS usb device',
        () async {
          final List<Device> devices = <Device>[FakeIOSDevice(sdkNameAndVersion: 'iOS 16.2')];
          final TestRunCommandForUsageValues command = TestRunCommandForUsageValues(
            devices: devices,
          );
          final CommandRunner<void> runner = createTestCommandRunner(command);
          try {
            // run the command so that CLI args are parsed
            await runner.run(<String>['run']);
          } on ToolExit catch (error) {
            // we can ignore the ToolExit, as we are only interested in
            // command.usageValues.
            expect(
              error,
              isA<ToolExit>().having(
                (ToolExit exception) => exception.message,
                'message',
                contains('No pubspec.yaml file found'),
              ),
            );
          }

          final CustomDimensions dimensions = await command.usageValues;

          expect(
            dimensions,
            const CustomDimensions(
              commandRunIsEmulator: false,
              commandRunTargetName: 'ios',
              commandRunTargetOsVersion: 'iOS 16.2',
              commandRunModeName: 'debug',
              commandRunProjectModule: false,
              commandRunProjectHostLanguage: '',
              commandRunIOSInterfaceType: 'usb',
              commandRunIsTest: false,
            ),
          );
        },
        overrides: <Type, Generator>{
          DeviceManager: () => testDeviceManager,
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'with only iOS wireless device',
        () async {
          final List<Device> devices = <Device>[
            FakeIOSDevice(
              connectionInterface: DeviceConnectionInterface.wireless,
              sdkNameAndVersion: 'iOS 16.2',
            ),
          ];
          final TestRunCommandForUsageValues command = TestRunCommandForUsageValues(
            devices: devices,
          );
          final CommandRunner<void> runner = createTestCommandRunner(command);
          try {
            // run the command so that CLI args are parsed
            await runner.run(<String>['run']);
          } on ToolExit catch (error) {
            // we can ignore the ToolExit, as we are only interested in
            // command.usageValues.
            expect(
              error,
              isA<ToolExit>().having(
                (ToolExit exception) => exception.message,
                'message',
                contains('No pubspec.yaml file found'),
              ),
            );
          }

          final CustomDimensions dimensions = await command.usageValues;

          expect(
            dimensions,
            const CustomDimensions(
              commandRunIsEmulator: false,
              commandRunTargetName: 'ios',
              commandRunTargetOsVersion: 'iOS 16.2',
              commandRunModeName: 'debug',
              commandRunProjectModule: false,
              commandRunProjectHostLanguage: '',
              commandRunIOSInterfaceType: 'wireless',
              commandRunIsTest: false,
            ),
          );
        },
        overrides: <Type, Generator>{
          DeviceManager: () => testDeviceManager,
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'with both iOS usb and wireless devices',
        () async {
          final List<Device> devices = <Device>[
            FakeIOSDevice(
              connectionInterface: DeviceConnectionInterface.wireless,
              sdkNameAndVersion: 'iOS 16.2',
            ),
            FakeIOSDevice(sdkNameAndVersion: 'iOS 16.2'),
          ];
          final TestRunCommandForUsageValues command = TestRunCommandForUsageValues(
            devices: devices,
          );
          final CommandRunner<void> runner = createTestCommandRunner(command);
          try {
            // run the command so that CLI args are parsed
            await runner.run(<String>['run']);
          } on ToolExit catch (error) {
            // we can ignore the ToolExit, as we are only interested in
            // command.usageValues.
            expect(
              error,
              isA<ToolExit>().having(
                (ToolExit exception) => exception.message,
                'message',
                contains('No pubspec.yaml file found'),
              ),
            );
          }
          final CustomDimensions dimensions = await command.usageValues;

          expect(
            dimensions,
            const CustomDimensions(
              commandRunIsEmulator: false,
              commandRunTargetName: 'multiple',
              commandRunTargetOsVersion: 'multiple',
              commandRunModeName: 'debug',
              commandRunProjectModule: false,
              commandRunProjectHostLanguage: '',
              commandRunIOSInterfaceType: 'wireless',
              commandRunIsTest: false,
            ),
          );
        },
        overrides: <Type, Generator>{
          DeviceManager: () => testDeviceManager,
          Cache: () => Cache.test(processManager: FakeProcessManager.any()),
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
        },
      );
    });

    group('--web-header', () {
      setUp(() {
        fileSystem.file('lib/main.dart').createSync(recursive: true);
        fileSystem.file('pubspec.yaml').createSync();
        fileSystem.file('.dart_tool/package_config.json')
          ..createSync(recursive: true)
          ..writeAsStringSync('''
{
  "packages": [],
  "configVersion": 2
}
''');
        final FakeDevice device = FakeDevice(
          isLocalEmulator: true,
          platformType: PlatformType.android,
        );
        testDeviceManager.devices = <Device>[device];
      });

      testUsingContext(
        'can accept simple, valid values',
        () async {
          final RunCommand command = RunCommand();
          await expectLater(
            () => createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', '--no-hot', '--web-header', 'foo = bar']),
            throwsToolExit(),
          );

          final DebuggingOptions options = await command.createDebuggingOptions(true);
          expect(options.webHeaders, <String, String>{'foo': 'bar'});
        },
        overrides: <Type, Generator>{
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'throws a ToolExit when no value is provided',
        () async {
          final RunCommand command = RunCommand();
          await expectLater(
            () => createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', '--no-hot', '--web-header', 'foo']),
            throwsToolExit(message: 'Invalid web headers: foo'),
          );

          await expectLater(() => command.createDebuggingOptions(true), throwsToolExit());
        },
        overrides: <Type, Generator>{
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'throws a ToolExit when value includes delimiter characters',
        () async {
          fileSystem.file('lib/main.dart').createSync(recursive: true);
          fileSystem.file('pubspec.yaml').createSync();
          fileSystem.file('.dart_tool/package_config.json').createSync(recursive: true);

          final RunCommand command = RunCommand();
          await expectLater(
            () => createTestCommandRunner(command).run(<String>[
              'run',
              '--no-pub',
              '--no-hot',
              '--web-header',
              'hurray/headers=flutter',
            ]),
            throwsToolExit(),
          );

          await expectLater(
            () => command.createDebuggingOptions(true),
            throwsToolExit(message: 'Invalid web headers: hurray/headers=flutter'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'throws a ToolExit when using --wasm on a non-web platform',
        () async {
          final RunCommand command = RunCommand();
          await expectLater(
            () => createTestCommandRunner(command).run(<String>['run', '--no-pub', '--wasm']),
            throwsToolExit(message: '--wasm is only supported on the web platform'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      testUsingContext(
        'throws a ToolExit when using the skwasm renderer without --wasm',
        () async {
          final RunCommand command = RunCommand();
          await expectLater(
            () => createTestCommandRunner(
              command,
            ).run(<String>['run', '--no-pub', ...WebRendererMode.skwasm.toCliDartDefines]),
            throwsToolExit(message: 'Skwasm renderer requires --wasm'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );

      // Tests whether using a deprecated webRenderer toggles a warningText.
      Future<void> testWebRendererDeprecationMessage(WebRendererMode webRenderer) async {
        testUsingContext(
          'Using --web-renderer=${webRenderer.name} triggers a warningText.',
          () async {
            // Run the command so it parses --web-renderer, but ignore all errors.
            // We only care about the logger.
            try {
              await createTestCommandRunner(
                RunCommand(),
              ).run(<String>['run', '--no-pub', ...webRenderer.toCliDartDefines]);
            } on ToolExit catch (error) {
              expect(error, isA<ToolExit>());
            }
            expect(
              logger.warningText,
              contains('See: https://docs.flutter.dev/to/web-html-renderer-deprecation'),
            );
          },
          overrides: <Type, Generator>{
            FileSystem: () => fileSystem,
            ProcessManager: () => FakeProcessManager.any(),
            Logger: () => logger,
            DeviceManager: () => testDeviceManager,
          },
        );
      }

      /// Do test all the deprecated WebRendererModes
      WebRendererMode.values
          .where((WebRendererMode mode) => mode.isDeprecated)
          .forEach(testWebRendererDeprecationMessage);

      testUsingContext(
        'accepts headers with commas in them',
        () async {
          final RunCommand command = RunCommand();
          await expectLater(
            () => createTestCommandRunner(command).run(<String>[
              'run',
              '--no-pub',
              '--no-hot',
              '--web-header',
              'hurray=flutter,flutter=hurray',
            ]),
            throwsToolExit(),
          );

          final DebuggingOptions options = await command.createDebuggingOptions(true);
          expect(options.webHeaders, <String, String>{'hurray': 'flutter,flutter=hurray'});
        },
        overrides: <Type, Generator>{
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
          Logger: () => logger,
          DeviceManager: () => testDeviceManager,
        },
      );
    });
  });

  group('terminal', () {
    late FakeAnsiTerminal fakeTerminal;

    setUp(() {
      fakeTerminal = FakeAnsiTerminal();
    });

    testUsingContext(
      'Flutter run sets terminal singleCharMode to false on exit',
      () async {
        final FakeResidentRunner residentRunner = FakeResidentRunner();
        final TestRunCommandWithFakeResidentRunner command = TestRunCommandWithFakeResidentRunner();
        command.fakeResidentRunner = residentRunner;

        await createTestCommandRunner(command).run(<String>['run', '--no-pub']);
        // The sync completer where we initially set `terminal.singleCharMode` to
        // `true` does not execute in unit tests, so explicitly check the
        // `setSingleCharModeHistory` that the finally block ran, setting this
        // back to `false`.
        expect(fakeTerminal.setSingleCharModeHistory, contains(false));
      },
      overrides: <Type, Generator>{
        AnsiTerminal: () => fakeTerminal,
        Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'Flutter run catches StdinException while setting terminal singleCharMode to false',
      () async {
        fakeTerminal.hasStdin = false;
        final FakeResidentRunner residentRunner = FakeResidentRunner();
        final TestRunCommandWithFakeResidentRunner command = TestRunCommandWithFakeResidentRunner();
        command.fakeResidentRunner = residentRunner;

        try {
          await createTestCommandRunner(command).run(<String>['run', '--no-pub']);
        } catch (err) {
          // ignore: avoid_catches_without_on_clauses
          fail('Expected no error, got $err');
        }
        expect(fakeTerminal.setSingleCharModeHistory, isEmpty);
      },
      overrides: <Type, Generator>{
        AnsiTerminal: () => fakeTerminal,
        Cache: () => Cache.test(processManager: FakeProcessManager.any()),
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  testUsingContext(
    'Flutter run catches catches errors due to vm service disconnection and throws a tool exit',
    () async {
      final FakeResidentRunner residentRunner = FakeResidentRunner();
      residentRunner.rpcError = RPCError(
        'flutter._listViews',
        RPCErrorKind.kServiceDisappeared.code,
        '',
      );
      final TestRunCommandWithFakeResidentRunner command = TestRunCommandWithFakeResidentRunner();
      command.fakeResidentRunner = residentRunner;

      await expectToolExitLater(
        createTestCommandRunner(command).run(<String>['run', '--no-pub']),
        contains('Lost connection to device.'),
      );

      residentRunner.rpcError = RPCError(
        'flutter._listViews',
        RPCErrorKind.kServerError.code,
        'Service connection disposed.',
      );

      await expectToolExitLater(
        createTestCommandRunner(command).run(<String>['run', '--no-pub']),
        contains('Lost connection to device.'),
      );
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'Flutter run does not catch other RPC errors',
    () async {
      final FakeResidentRunner residentRunner = FakeResidentRunner();
      residentRunner.rpcError = RPCError(
        'flutter._listViews',
        RPCErrorKind.kInvalidParams.code,
        '',
      );
      final TestRunCommandWithFakeResidentRunner command = TestRunCommandWithFakeResidentRunner();
      command.fakeResidentRunner = residentRunner;

      await expectLater(
        () => createTestCommandRunner(command).run(<String>['run', '--no-pub']),
        throwsA(isA<RPCError>()),
      );
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'Passes sksl bundle info the build options',
    () async {
      final TestRunCommandWithFakeResidentRunner command = TestRunCommandWithFakeResidentRunner();

      await expectLater(
        () => createTestCommandRunner(
          command,
        ).run(<String>['run', '--no-pub', '--bundle-sksl-path=foo.json']),
        throwsToolExit(message: 'No SkSL shader bundle found at foo.json'),
      );
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'Configures web connection options to use web sockets by default',
    () async {
      final RunCommand command = RunCommand();
      await expectLater(
        () => createTestCommandRunner(command).run(<String>['run', '--no-pub']),
        throwsToolExit(),
      );

      final DebuggingOptions options = await command.createDebuggingOptions(true);

      expect(options.webUseSseForDebugBackend, false);
      expect(options.webUseSseForDebugProxy, false);
      expect(options.webUseSseForInjectedClient, false);
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'flags propagate to debugging options',
    () async {
      final RunCommand command = RunCommand();
      await expectLater(
        () => createTestCommandRunner(command).run(<String>[
          'run',
          '--start-paused',
          '--disable-service-auth-codes',
          '--use-test-fonts',
          '--trace-skia',
          '--trace-systrace',
          '--trace-to-file=path/to/trace.binpb',
          '--verbose-system-logs',
          '--null-assertions',
          '--native-null-assertions',
          '--enable-impeller',
          '--enable-vulkan-validation',
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
      expect(options.useTestFonts, true);
      expect(options.traceSkia, true);
      expect(options.traceSystrace, true);
      expect(options.traceToFile, 'path/to/trace.binpb');
      expect(options.verboseSystemLogs, true);
      expect(options.nullAssertions, true);
      expect(options.nativeNullAssertions, true);
      expect(options.traceSystrace, true);
      expect(options.enableImpeller, ImpellerStatus.enabled);
      expect(options.enableVulkanValidation, true);
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
    'usingCISystem can also be set by environment LUCI_CI',
    () async {
      final RunCommand command = RunCommand();
      await expectLater(
        () => createTestCommandRunner(command).run(<String>['run']),
        throwsToolExit(),
      );

      final DebuggingOptions options = await command.createDebuggingOptions(false);

      expect(options.usingCISystem, true);
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
      Platform: () => FakePlatform(environment: <String, String>{'LUCI_CI': 'True'}),
    },
  );

  testUsingContext(
    'wasm mode selects skwasm renderer by default',
    () async {
      final RunCommand command = RunCommand();
      await expectLater(
        () => createTestCommandRunner(command).run(<String>['run', '-d chrome', '--wasm']),
        throwsToolExit(),
      );

      final DebuggingOptions options = await command.createDebuggingOptions(false);

      expect(options.webUseWasm, true);
      expect(options.webRenderer, WebRendererMode.skwasm);
    },
    overrides: <Type, Generator>{
      Cache: () => Cache.test(processManager: FakeProcessManager.any()),
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'fails when "--web-launch-url" is not supported',
    () async {
      final RunCommand command = RunCommand();
      await expectLater(
        () => createTestCommandRunner(
          command,
        ).run(<String>['run', '--web-launch-url=http://flutter.dev']),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'toString',
            isNot(contains('web-launch-url')),
          ),
        ),
      );

      final DebuggingOptions options = await command.createDebuggingOptions(true);
      expect(options.webLaunchUrl, 'http://flutter.dev');

      final RegExp pattern = RegExp(r'^((http)?:\/\/)[^\s]+');
      expect(pattern.hasMatch(options.webLaunchUrl!), true);
    },
    overrides: <Type, Generator>{
      ProcessManager: () => FakeProcessManager.any(),
      Logger: () => BufferLogger.test(),
    },
  );
}

class TestDeviceManager extends DeviceManager {
  TestDeviceManager({required super.logger});
  List<Device> devices = <Device>[];

  @override
  List<DeviceDiscovery> get deviceDiscoverers {
    final FakePollingDeviceDiscovery discoverer = FakePollingDeviceDiscovery();
    devices.forEach(discoverer.addDevice);
    return <DeviceDiscovery>[discoverer];
  }
}

class FakeDevice extends Fake implements Device {
  FakeDevice({
    bool isLocalEmulator = false,
    TargetPlatform targetPlatform = TargetPlatform.ios,
    String sdkNameAndVersion = '',
    PlatformType platformType = PlatformType.ios,
    bool isSupported = true,
    bool supportsFlavors = false,
  }) : _isLocalEmulator = isLocalEmulator,
       _targetPlatform = targetPlatform,
       _sdkNameAndVersion = sdkNameAndVersion,
       _platformType = platformType,
       _isSupported = isSupported,
       _supportsFlavors = supportsFlavors;

  static const int kSuccess = 1;
  static const int kFailure = -1;
  final TargetPlatform _targetPlatform;
  final bool _isLocalEmulator;
  final String _sdkNameAndVersion;
  final PlatformType _platformType;
  final bool _isSupported;
  final bool _supportsFlavors;

  @override
  Category get category => Category.mobile;

  @override
  String get id => 'fake_device';

  Never _throwToolExit(int code) => throwToolExit('FakeDevice tool exit', exitCode: code);

  @override
  Future<bool> get isLocalEmulator => Future<bool>.value(_isLocalEmulator);

  @override
  bool supportsRuntimeMode(BuildMode mode) => true;

  @override
  Future<bool> get supportsHardwareRendering async => true;

  @override
  bool supportsHotReload = false;

  @override
  bool get supportsHotRestart => true;

  @override
  bool get supportsFastStart => false;

  @override
  bool get supportsFlavors => _supportsFlavors;

  @override
  bool get ephemeral => true;

  @override
  bool get isConnected => true;

  @override
  DeviceConnectionInterface get connectionInterface => DeviceConnectionInterface.attached;

  bool supported = true;

  @override
  bool isSupportedForProject(FlutterProject flutterProject) => _isSupported;

  @override
  bool isSupported() => supported;

  @override
  Future<String> get sdkNameAndVersion => Future<String>.value(_sdkNameAndVersion);

  @override
  Future<String> get targetPlatformDisplayName async =>
      getNameForTargetPlatform(await targetPlatform);

  @override
  DeviceLogReader getLogReader({ApplicationPackage? app, bool includePastLogs = false}) {
    return FakeDeviceLogReader();
  }

  @override
  String get name => 'FakeDevice';

  @override
  String get displayName => name;

  @override
  Future<TargetPlatform> get targetPlatform async => _targetPlatform;

  @override
  PlatformType get platformType => _platformType;

  late bool startAppSuccess;

  @override
  DevFSWriter? createDevFSWriter(ApplicationPackage? app, String? userIdentifier) {
    return null;
  }

  @override
  Future<LaunchResult> startApp(
    ApplicationPackage? package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, Object?> platformArgs = const <String, Object?>{},
    bool prebuiltApplication = false,
    bool usesTerminalUi = true,
    bool ipv6 = false,
    String? userIdentifier,
  }) async {
    if (!startAppSuccess) {
      return LaunchResult.failed();
    }
    if (startAppSuccess) {
      return LaunchResult.succeeded();
    }
    final String dartFlags = debuggingOptions.dartFlags;
    // In release mode, --dart-flags should be set to the empty string and
    // provided flags should be dropped. In debug and profile modes,
    // --dart-flags should not be empty.
    if (debuggingOptions.buildInfo.isRelease) {
      if (dartFlags.isNotEmpty) {
        _throwToolExit(kFailure);
      }
      _throwToolExit(kSuccess);
    } else {
      if (dartFlags.isEmpty) {
        _throwToolExit(kFailure);
      }
      _throwToolExit(kSuccess);
    }
  }
}

class FakeMacDesignedForIpadDevice extends Fake implements MacOSDesignedForIPadDevice {
  @override
  String get id => 'mac-designed-for-ipad';

  @override
  bool get isConnected => true;

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.darwin;

  @override
  DeviceConnectionInterface connectionInterface = DeviceConnectionInterface.attached;

  @override
  bool isSupported() => true;

  @override
  bool isSupportedForProject(FlutterProject project) => true;
}

class FakeIOSDevice extends Fake implements IOSDevice {
  FakeIOSDevice({
    this.connectionInterface = DeviceConnectionInterface.attached,
    bool isLocalEmulator = false,
    String sdkNameAndVersion = '',
  }) : _isLocalEmulator = isLocalEmulator,
       _sdkNameAndVersion = sdkNameAndVersion;

  final bool _isLocalEmulator;
  final String _sdkNameAndVersion;

  @override
  Future<bool> get isLocalEmulator => Future<bool>.value(_isLocalEmulator);

  @override
  Future<String> get sdkNameAndVersion => Future<String>.value(_sdkNameAndVersion);

  @override
  final DeviceConnectionInterface connectionInterface;

  @override
  bool get isWirelesslyConnected => connectionInterface == DeviceConnectionInterface.wireless;

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.ios;
}

class TestRunCommandForUsageValues extends RunCommand {
  TestRunCommandForUsageValues({List<Device>? devices}) {
    this.devices = devices;
  }

  @override
  Future<BuildInfo> getBuildInfo({
    FlutterProject? project,
    BuildMode? forcedBuildMode,
    File? forcedTargetFile,
    bool? forcedUseLocalCanvasKit,
  }) async {
    return const BuildInfo(
      BuildMode.debug,
      null,
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
  }
}

class TestRunCommandWithFakeResidentRunner extends RunCommand {
  late FakeResidentRunner fakeResidentRunner;

  @override
  Future<ResidentRunner> createRunner({
    required bool hotMode,
    required List<FlutterDevice> flutterDevices,
    required String? applicationBinaryPath,
    required FlutterProject flutterProject,
  }) async {
    return fakeResidentRunner;
  }

  @override
  // ignore: must_call_super
  Future<void> validateCommand() async {
    devices = <Device>[FakeDevice()..supportsHotReload = true];
  }
}

class TestRunCommandThatOnlyValidates extends RunCommand {
  @override
  Future<FlutterCommandResult> runCommand() async {
    return FlutterCommandResult.success();
  }

  @override
  bool get shouldRunPub => false;
}

class FakeResidentRunner extends Fake implements ResidentRunner {
  RPCError? rpcError;

  @override
  Future<int> run({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    bool enableDevTools = false,
    String? route,
  }) async {
    await null;
    if (rpcError != null) {
      throw rpcError!;
    }
    return 0;
  }
}

class DaemonCapturingRunCommand extends RunCommand {
  late Daemon daemon;
  late CapturingAppDomain appDomain;

  @override
  Daemon createMachineDaemon() {
    daemon = super.createMachineDaemon();
    appDomain = daemon.appDomain = CapturingAppDomain(daemon);
    daemon.registerDomain(appDomain);
    return daemon;
  }
}

class CapturingAppDomain extends AppDomain {
  CapturingAppDomain(super.daemon);

  String? userIdentifier;
  bool? enableDevTools;

  @override
  Future<AppInstance> startApp(
    Device device,
    String projectDirectory,
    String target,
    String? route,
    DebuggingOptions options,
    bool enableHotReload, {
    File? applicationBinary,
    required bool trackWidgetCreation,
    String? projectRootPath,
    String? packagesFilePath,
    String? dillOutputPath,
    String? isolateFilter,
    bool machine = true,
    String? userIdentifier,
  }) async {
    this.userIdentifier = userIdentifier;
    enableDevTools = options.enableDevTools;
    throwToolExit('');
  }
}

class FakeAnsiTerminal extends Fake implements AnsiTerminal {
  /// Setting to false will cause operations to Stdin to throw a [StdinException].
  bool hasStdin = true;

  @override
  bool usesTerminalUi = false;

  /// A list of all the calls to the [singleCharMode] setter.
  List<bool> setSingleCharModeHistory = <bool>[];

  @override
  set singleCharMode(bool value) {
    if (!hasStdin) {
      throw const StdinException(
        'Error setting terminal line mode',
        OSError('The handle is invalid', 6),
      );
    }
    setSingleCharModeHistory.add(value);
  }

  @override
  bool get singleCharMode => setSingleCharModeHistory.last;
}
