// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/application_package.dart';
import 'package:flutter_tools/src/ios/plist_parser.dart';
import 'package:flutter_tools/src/ios/simulators.dart';
import 'package:flutter_tools/src/macos/xcode.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

final Platform macosPlatform = FakePlatform(
  operatingSystem: 'macos',
  environment: <String, String>{'HOME': '/'},
);

void main() {
  late FakePlatform osx;
  late FileSystemUtils fsUtils;
  late MemoryFileSystem fileSystem;
  final Logger logger = FakeLogger();

  setUp(() {
    osx = FakePlatform(environment: <String, String>{}, operatingSystem: 'macos');
    fileSystem = MemoryFileSystem.test();
    fsUtils = FileSystemUtils(fileSystem: fileSystem, platform: osx);
  });

  group('_IOSSimulatorDevicePortForwarder', () {
    late FakeSimControl simControl;
    late Xcode xcode;

    setUp(() {
      simControl = FakeSimControl();
      xcode = Xcode.test(processManager: FakeProcessManager.any());
    });

    testUsingContext(
      'dispose() does not throw an exception',
      () async {
        final IOSSimulator simulator = IOSSimulator(
          '123',
          name: 'iPhone 11',
          simControl: simControl,
          simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-14-4',
          logger: logger,
        );
        final DevicePortForwarder portForwarder = simulator.portForwarder;
        await portForwarder.forward(123);
        await portForwarder.forward(124);
        expect(portForwarder.forwardedPorts.length, 2);
        try {
          await portForwarder.dispose();
        } on Exception catch (e) {
          fail('Encountered exception: $e');
        }
        expect(portForwarder.forwardedPorts.length, 0);
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Xcode: () => xcode,
      },
      testOn: 'posix',
    );
  });

  testUsingContext(
    'simulators only support debug mode',
    () async {
      final IOSSimulator simulator = IOSSimulator(
        '123',
        name: 'iPhone 11',
        simControl: FakeSimControl(),
        simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-14-4',
        logger: logger,
      );

      expect(simulator.supportsRuntimeMode(BuildMode.debug), true);
      expect(simulator.supportsRuntimeMode(BuildMode.profile), false);
      expect(simulator.supportsRuntimeMode(BuildMode.release), false);
      expect(simulator.supportsRuntimeMode(BuildMode.jitRelease), false);
    },
    overrides: <Type, Generator>{
      Platform: () => osx,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  group('logFilePath', () {
    late FakeSimControl simControl;

    setUp(() {
      simControl = FakeSimControl();
    });

    testUsingContext(
      'defaults to rooted from HOME',
      () {
        osx.environment['HOME'] = '/foo/bar';
        final IOSSimulator simulator = IOSSimulator(
          '123',
          name: 'iPhone 11',
          simControl: simControl,
          simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-14-4',
          logger: logger,
        );
        expect(simulator.logFilePath, '/foo/bar/Library/Logs/CoreSimulator/123/system.log');
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystemUtils: () => fsUtils,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
      testOn: 'posix',
    );

    testUsingContext(
      'respects IOS_SIMULATOR_LOG_FILE_PATH',
      () {
        osx.environment['HOME'] = '/foo/bar';
        osx.environment['IOS_SIMULATOR_LOG_FILE_PATH'] = '/baz/qux/%{id}/system.log';
        final IOSSimulator simulator = IOSSimulator(
          '456',
          name: 'iPhone 11',
          simControl: simControl,
          simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-14-4',
          logger: logger,
        );
        expect(simulator.logFilePath, '/baz/qux/456/system.log');
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystemUtils: () => fsUtils,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  group('sdkMajorVersion', () {
    late FakeSimControl simControl;

    setUp(() {
      simControl = FakeSimControl();
    });

    // This new version string appears in SimulatorApp-850 CoreSimulator-518.16 beta.
    testWithoutContext('can be parsed from iOS-11-3', () async {
      final IOSSimulator device = IOSSimulator(
        'x',
        name: 'iPhone SE',
        simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
        simControl: simControl,
        logger: logger,
      );

      expect(await device.sdkMajorVersion, 11);
    });

    testWithoutContext('can be parsed from iOS 11.2', () async {
      final IOSSimulator device = IOSSimulator(
        'x',
        name: 'iPhone SE',
        simulatorCategory: 'iOS 11.2',
        simControl: simControl,
        logger: logger,
      );

      expect(await device.sdkMajorVersion, 11);
    });

    testWithoutContext('Has a simulator category', () async {
      final IOSSimulator device = IOSSimulator(
        'x',
        name: 'iPhone SE',
        simulatorCategory: 'iOS 11.2',
        simControl: simControl,
        logger: logger,
      );

      expect(device.category, Category.mobile);
    });
  });

  group('IOSSimulator.isSupported', () {
    late FakeSimControl simControl;

    setUp(() {
      simControl = FakeSimControl();
    });

    testUsingContext(
      'Apple TV is unsupported',
      () {
        final IOSSimulator simulator = IOSSimulator(
          'x',
          name: 'Apple TV',
          simControl: simControl,
          simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.tvOS-14-5',
          logger: logger,
        );
        expect(simulator.isSupported(), false);
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'Apple Watch is unsupported',
      () {
        expect(
          IOSSimulator(
            'x',
            name: 'Apple Watch',
            simControl: simControl,
            simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.watchOS-8-0',
            logger: logger,
          ).isSupported(),
          false,
        );
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'iPad 2 is supported',
      () {
        expect(
          IOSSimulator(
            'x',
            name: 'iPad 2',
            simControl: simControl,
            simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
            logger: logger,
          ).isSupported(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'iPad Retina is supported',
      () {
        expect(
          IOSSimulator(
            'x',
            name: 'iPad Retina',
            simControl: simControl,
            simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
            logger: logger,
          ).isSupported(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'iPhone 5 is supported',
      () {
        expect(
          IOSSimulator(
            'x',
            name: 'iPhone 5',
            simControl: simControl,
            simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
            logger: logger,
          ).isSupported(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'iPhone 5s is supported',
      () {
        expect(
          IOSSimulator(
            'x',
            name: 'iPhone 5s',
            simControl: simControl,
            simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
            logger: logger,
          ).isSupported(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'iPhone SE is supported',
      () {
        expect(
          IOSSimulator(
            'x',
            name: 'iPhone SE',
            simControl: simControl,
            simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
            logger: logger,
          ).isSupported(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'iPhone 7 Plus is supported',
      () {
        expect(
          IOSSimulator(
            'x',
            name: 'iPhone 7 Plus',
            simControl: simControl,
            simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
            logger: logger,
          ).isSupported(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'iPhone X is supported',
      () {
        expect(
          IOSSimulator(
            'x',
            name: 'iPhone X',
            simControl: simControl,
            simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
            logger: logger,
          ).isSupported(),
          true,
        );
      },
      overrides: <Type, Generator>{
        Platform: () => osx,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );
  });

  group('Simulator screenshot', () {
    testWithoutContext('supports screenshots', () async {
      final Xcode xcode = Xcode.test(processManager: FakeProcessManager.any());
      final Logger logger = BufferLogger.test();
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'io', 'x', 'screenshot', 'screenshot.png'],
        ),
      ]);

      // Test a real one. Screenshot doesn't require instance states.
      final SimControl simControl = SimControl(
        processManager: fakeProcessManager,
        logger: logger,
        xcode: xcode,
      );
      // Doesn't matter what the device is.
      final IOSSimulator deviceUnderTest = IOSSimulator(
        'x',
        name: 'iPhone SE',
        simControl: simControl,
        simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
        logger: logger,
      );

      final File screenshot = MemoryFileSystem.test().file('screenshot.png');
      await deviceUnderTest.takeScreenshot(screenshot);
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });
  });

  group('device log tool', () {
    late FakeProcessManager fakeProcessManager;
    late FakeSimControl simControl;

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
      simControl = FakeSimControl();
    });

    testUsingContext(
      'syslog uses tail',
      () async {
        final IOSSimulator device = IOSSimulator(
          'x',
          name: 'iPhone SE',
          simulatorCategory: 'iOS 9.3',
          simControl: simControl,
          logger: logger,
        );
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>['tail', '-n', '0', '-F', '/Library/Logs/CoreSimulator/x/system.log'],
          ),
        );
        await launchDeviceSystemLogTool(device);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => fileSystem,
        Platform: () => macosPlatform,
        FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: macosPlatform),
      },
    );

    testUsingContext(
      'unified logging with app name',
      () async {
        final IOSSimulator device = IOSSimulator(
          'x',
          name: 'iPhone SE',
          simulatorCategory: 'iOS 11.0',
          simControl: simControl,
          logger: logger,
        );
        const String expectedPredicate =
            'eventType = logEvent AND '
            'processImagePath ENDSWITH "My Super Awesome App" AND '
            '(senderImagePath ENDSWITH "/Flutter" OR senderImagePath ENDSWITH "/libswiftCore.dylib" OR processImageUUID == senderImageUUID) AND '
            'NOT(eventMessage CONTAINS ": could not find icon for representation -> com.apple.") AND '
            'NOT(eventMessage BEGINSWITH "assertion failed: ") AND '
            'NOT(eventMessage CONTAINS " libxpc.dylib ")';
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>[
              'xcrun',
              'simctl',
              'spawn',
              'x',
              'log',
              'stream',
              '--style',
              'json',
              '--predicate',
              expectedPredicate,
            ],
          ),
        );

        await launchDeviceUnifiedLogging(device, 'My Super Awesome App');
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => fileSystem,
      },
    );

    testUsingContext(
      'unified logging without app name',
      () async {
        final IOSSimulator device = IOSSimulator(
          'x',
          name: 'iPhone SE',
          simulatorCategory: 'iOS 11.0',
          simControl: simControl,
          logger: logger,
        );
        const String expectedPredicate =
            'eventType = logEvent AND '
            '(senderImagePath ENDSWITH "/Flutter" OR senderImagePath ENDSWITH "/libswiftCore.dylib" OR processImageUUID == senderImageUUID) AND '
            'NOT(eventMessage CONTAINS ": could not find icon for representation -> com.apple.") AND '
            'NOT(eventMessage BEGINSWITH "assertion failed: ") AND '
            'NOT(eventMessage CONTAINS " libxpc.dylib ")';
        fakeProcessManager.addCommand(
          const FakeCommand(
            command: <String>[
              'xcrun',
              'simctl',
              'spawn',
              'x',
              'log',
              'stream',
              '--style',
              'json',
              '--predicate',
              expectedPredicate,
            ],
          ),
        );

        await launchDeviceUnifiedLogging(device, null);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        ProcessManager: () => fakeProcessManager,
        FileSystem: () => fileSystem,
      },
    );
  });

  group('log reader', () {
    late FakeProcessManager fakeProcessManager;
    late FakeIosProject mockIosProject;
    late FakeSimControl simControl;
    late Xcode xcode;

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
      mockIosProject = FakeIosProject();
      simControl = FakeSimControl();
      xcode = Xcode.test(processManager: FakeProcessManager.any());
    });

    group('syslog', () {
      setUp(() {
        final File syslog = fileSystem.file('system.log')..createSync();
        osx.environment['IOS_SIMULATOR_LOG_FILE_PATH'] = syslog.path;
      });

      testUsingContext(
        'simulator can parse Xcode 8/iOS 10-style logs',
        () async {
          fakeProcessManager
            ..addCommand(
              const FakeCommand(
                command: <String>['tail', '-n', '0', '-F', 'system.log'],
                stdout: '''
Dec 20 17:04:32 md32-11-vm1 My Super Awesome App[88374]: flutter: The Dart VM service is listening on http://127.0.0.1:64213/1Uoeu523990=/
Dec 20 17:04:32 md32-11-vm1 Another App[88374]: Ignore this text''',
              ),
            )
            ..addCommand(
              const FakeCommand(
                command: <String>['tail', '-n', '0', '-F', '/private/var/log/system.log'],
              ),
            );

          final IOSSimulator device = IOSSimulator(
            '123456',
            name: 'iPhone 11',
            simulatorCategory: 'iOS 10.0',
            simControl: simControl,
            logger: logger,
          );
          final DeviceLogReader logReader = device.getLogReader(
            app: await BuildableIOSApp.fromProject(mockIosProject, null),
          );

          final List<String> lines = await logReader.logLines.toList();
          expect(lines, <String>[
            'flutter: The Dart VM service is listening on http://127.0.0.1:64213/1Uoeu523990=/',
          ]);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => fakeProcessManager,
          FileSystem: () => fileSystem,
          Platform: () => osx,
          Xcode: () => xcode,
        },
      );

      testUsingContext(
        'simulator can output `)`',
        () async {
          fakeProcessManager
            ..addCommand(
              const FakeCommand(
                command: <String>['tail', '-n', '0', '-F', 'system.log'],
                stdout: '''
2017-09-13 15:26:57.228948-0700  localhost My Super Awesome App[37195]: (Flutter) The Dart VM service is listening on http://127.0.0.1:57701/
2017-09-13 15:26:57.228948-0700  localhost My Super Awesome App[37195]: (Flutter) ))))))))))
2017-09-13 15:26:57.228948-0700  localhost My Super Awesome App[37195]: (Flutter) #0      Object.noSuchMethod (dart:core-patch/dart:core/object_patch.dart:46)''',
              ),
            )
            ..addCommand(
              const FakeCommand(
                command: <String>['tail', '-n', '0', '-F', '/private/var/log/system.log'],
              ),
            );

          final IOSSimulator device = IOSSimulator(
            '123456',
            name: 'iPhone 11',
            simulatorCategory: 'iOS 10.3',
            simControl: simControl,
            logger: logger,
          );
          final DeviceLogReader logReader = device.getLogReader(
            app: await BuildableIOSApp.fromProject(mockIosProject, null),
          );

          final List<String> lines = await logReader.logLines.toList();
          expect(lines, <String>[
            'The Dart VM service is listening on http://127.0.0.1:57701/',
            '))))))))))',
            '#0      Object.noSuchMethod (dart:core-patch/dart:core/object_patch.dart:46)',
          ]);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => fakeProcessManager,
          FileSystem: () => fileSystem,
          Platform: () => osx,
          Xcode: () => xcode,
        },
      );

      testUsingContext(
        'multiline messages',
        () async {
          fakeProcessManager
            ..addCommand(
              const FakeCommand(
                command: <String>['tail', '-n', '0', '-F', 'system.log'],
                stdout: '''
2017-09-13 15:26:57.228948-0700  localhost My Super Awesome App[37195]: (Flutter) Single line message
2017-09-13 15:26:57.228948-0700  localhost My Super Awesome App[37195]: (Flutter) Multi line message
  continues...
  continues...
2020-03-11 15:58:28.207175-0700  localhost My Super Awesome App[72166]: (libnetwork.dylib) [com.apple.network:] [28 www.googleapis.com:443 stream, pid: 72166, tls] cancelled
	[28.1 64A98447-EABF-4983-A387-7DB9D0C1785F 10.0.1.200.57912<->172.217.6.74:443]
	Connected Path: satisfied (Path is satisfied), interface: en18
	Duration: 0.271s, DNS @0.000s took 0.001s, TCP @0.002s took 0.019s, TLS took 0.046s
	bytes in/out: 4468/1933, packets in/out: 11/10, rtt: 0.016s, retransmitted packets: 0, out-of-order packets: 0
2017-09-13 15:36:57.228948-0700  localhost My Super Awesome App[37195]: (Flutter) Multi line message again
  and it goes...
  and goes...
2017-09-13 15:36:57.228948-0700  localhost My Super Awesome App[37195]: (Flutter) Single line message, not the part of the above
''',
              ),
            )
            ..addCommand(
              const FakeCommand(
                command: <String>['tail', '-n', '0', '-F', '/private/var/log/system.log'],
              ),
            );

          final IOSSimulator device = IOSSimulator(
            '123456',
            name: 'iPhone 11',
            simulatorCategory: 'iOS 10.3',
            simControl: simControl,
            logger: logger,
          );
          final DeviceLogReader logReader = device.getLogReader(
            app: await BuildableIOSApp.fromProject(mockIosProject, null),
          );

          final List<String> lines = await logReader.logLines.toList();
          expect(lines, <String>[
            'Single line message',
            'Multi line message',
            '  continues...',
            '  continues...',
            'Multi line message again',
            '  and it goes...',
            '  and goes...',
            'Single line message, not the part of the above',
          ]);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => fakeProcessManager,
          FileSystem: () => fileSystem,
          Platform: () => osx,
          Xcode: () => xcode,
        },
      );
    });

    group('unified logging', () {
      late BufferLogger logger;

      setUp(() {
        logger = BufferLogger.test();
      });

      testUsingContext(
        'log reader handles escaped multiline messages',
        () async {
          const String logPredicate =
              'eventType = logEvent AND processImagePath ENDSWITH "My Super Awesome App" '
              'AND (senderImagePath ENDSWITH "/Flutter" OR senderImagePath ENDSWITH "/libswiftCore.dylib" '
              'OR processImageUUID == senderImageUUID) AND NOT(eventMessage CONTAINS ": could not find icon '
              'for representation -> com.apple.") AND NOT(eventMessage BEGINSWITH "assertion failed: ") '
              'AND NOT(eventMessage CONTAINS " libxpc.dylib ")';
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'xcrun',
                'simctl',
                'spawn',
                '123456',
                'log',
                'stream',
                '--style',
                'json',
                '--predicate',
                logPredicate,
              ],
              stdout: r'''
},{
  "traceID" : 37579774151491588,
  "eventMessage" : "Single line message",
  "eventType" : "logEvent"
},{
  "traceID" : 37579774151491588,
  "eventMessage" : "Multi line message\n  continues...\n  continues..."
},{
  "traceID" : 37579774151491588,
  "eventMessage" : "Single line message, not the part of the above",
  "eventType" : "logEvent"
},{
''',
            ),
          );

          final IOSSimulator device = IOSSimulator(
            '123456',
            name: 'iPhone 11',
            simulatorCategory: 'iOS 11.0',
            simControl: simControl,
            logger: logger,
          );
          final DeviceLogReader logReader = device.getLogReader(
            app: await BuildableIOSApp.fromProject(mockIosProject, null),
          );

          final List<String> lines = await logReader.logLines.toList();
          expect(lines, <String>[
            'Single line message',
            'Multi line message\n  continues...\n  continues...',
            'Single line message, not the part of the above',
          ]);
          expect(fakeProcessManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => fakeProcessManager,
          FileSystem: () => fileSystem,
        },
      );

      testUsingContext(
        'log reader handles bad output',
        () async {
          const String logPredicate =
              'eventType = logEvent AND processImagePath ENDSWITH "My Super Awesome App" '
              'AND (senderImagePath ENDSWITH "/Flutter" OR senderImagePath ENDSWITH "/libswiftCore.dylib" '
              'OR processImageUUID == senderImageUUID) AND NOT(eventMessage CONTAINS ": could not find icon '
              'for representation -> com.apple.") AND NOT(eventMessage BEGINSWITH "assertion failed: ") '
              'AND NOT(eventMessage CONTAINS " libxpc.dylib ")';
          fakeProcessManager.addCommand(
            const FakeCommand(
              command: <String>[
                'xcrun',
                'simctl',
                'spawn',
                '123456',
                'log',
                'stream',
                '--style',
                'json',
                '--predicate',
                logPredicate,
              ],
              stdout: '"eventMessage" : "message with incorrect escaping""',
            ),
          );

          final IOSSimulator device = IOSSimulator(
            '123456',
            name: 'iPhone 11',
            simulatorCategory: 'iOS 11.0',
            simControl: simControl,
            logger: logger,
          );
          final DeviceLogReader logReader = device.getLogReader(
            app: await BuildableIOSApp.fromProject(mockIosProject, null),
          );

          final List<String> lines = await logReader.logLines.toList();
          expect(lines, isEmpty);
          expect(logger.errorText, contains('Logger returned non-JSON response'));
          expect(fakeProcessManager, hasNoRemainingExpectations);
        },
        overrides: <Type, Generator>{
          ProcessManager: () => fakeProcessManager,
          FileSystem: () => fileSystem,
          Logger: () => logger,
        },
      );
    });
  });

  group('SimControl', () {
    const String validSimControlOutput = '''
{
  "devices" : {
    "com.apple.CoreSimulator.SimRuntime.iOS-14-0" : [
      {
        "dataPathSize" : 1734569984,
        "udid" : "iPhone 11-UDID",
        "isAvailable" : true,
        "logPathSize" : 9506816,
        "deviceTypeIdentifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-11",
        "state" : "Booted",
        "name" : "iPhone 11"
      }
    ],
    "com.apple.CoreSimulator.SimRuntime.iOS-13-0" : [
    ],
    "com.apple.CoreSimulator.SimRuntime.iOS-12-4" : [
    ],
    "com.apple.CoreSimulator.SimRuntime.tvOS-16-0" : [
    ],
    "com.apple.CoreSimulator.SimRuntime.watchOS-9-0" : [
    ],
    "com.apple.CoreSimulator.SimRuntime.iOS-16-0" : [
      {
        "dataPathSize" : 552366080,
        "udid" : "Phone w Watch-UDID",
        "isAvailable" : true,
        "logPathSize" : 90112,
        "deviceTypeIdentifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-11",
        "state" : "Booted",
        "name" : "Phone w Watch"
      },
      {
        "dataPathSize" : 2186457088,
        "udid" : "iPhone 13-UDID",
        "isAvailable" : true,
        "logPathSize" : 151552,
        "deviceTypeIdentifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-13",
        "state" : "Booted",
        "name" : "iPhone 13"
      }
    ]
  }
}
    ''';

    late FakeProcessManager fakeProcessManager;
    Xcode xcode;
    late SimControl simControl;
    late BufferLogger logger;
    const String deviceId = 'smart-phone';
    const String appId = 'flutterApp';

    setUp(() {
      fakeProcessManager = FakeProcessManager.empty();
      xcode = Xcode.test(processManager: FakeProcessManager.any());
      logger = BufferLogger.test();
      simControl = SimControl(logger: logger, processManager: fakeProcessManager, xcode: xcode);
    });

    testWithoutContext('getConnectedDevices succeeds', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'list', 'devices', 'booted', 'iOS', '--json'],
          stdout: validSimControlOutput,
        ),
      );

      final List<BootedSimDevice> devices = await simControl.getConnectedDevices();

      final BootedSimDevice phone1 = devices[0];
      expect(phone1.category, 'com.apple.CoreSimulator.SimRuntime.iOS-14-0');
      expect(phone1.name, 'iPhone 11');
      expect(phone1.udid, 'iPhone 11-UDID');

      final BootedSimDevice phone2 = devices[1];
      expect(phone2.category, 'com.apple.CoreSimulator.SimRuntime.iOS-16-0');
      expect(phone2.name, 'Phone w Watch');
      expect(phone2.udid, 'Phone w Watch-UDID');

      final BootedSimDevice phone3 = devices[2];
      expect(phone3.category, 'com.apple.CoreSimulator.SimRuntime.iOS-16-0');
      expect(phone3.name, 'iPhone 13');
      expect(phone3.udid, 'iPhone 13-UDID');
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });

    testWithoutContext('getConnectedDevices handles bad simctl output', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'list', 'devices', 'booted', 'iOS', '--json'],
          stdout: 'Install Started',
        ),
      );

      final List<BootedSimDevice> devices = await simControl.getConnectedDevices();

      expect(devices, isEmpty);
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });

    testWithoutContext('sdkMajorVersion defaults to 11 when sdkNameAndVersion is junk', () async {
      final IOSSimulator iosSimulatorA = IOSSimulator(
        'x',
        name: 'Testo',
        simulatorCategory: 'NaN',
        simControl: simControl,
        logger: logger,
      );

      expect(await iosSimulatorA.sdkMajorVersion, 11);
    });

    testWithoutContext('.install() handles exceptions', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'install', deviceId, appId],
          exception: ProcessException('xcrun', <String>[]),
        ),
      );

      expect(
        () async => simControl.install(deviceId, appId),
        throwsToolExit(message: r'Unable to install'),
      );
    });

    testWithoutContext('.uninstall() handles exceptions', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'uninstall', deviceId, appId],
          exception: ProcessException('xcrun', <String>[]),
        ),
      );

      expect(
        () async => simControl.uninstall(deviceId, appId),
        throwsToolExit(message: r'Unable to uninstall'),
      );
    });

    testWithoutContext('.launch() handles exceptions', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'launch', deviceId, appId],
          exception: ProcessException('xcrun', <String>[]),
        ),
      );

      expect(
        () async => simControl.launch(deviceId, appId),
        throwsToolExit(message: r'Unable to launch'),
      );
    });

    testWithoutContext('.stopApp() handles exceptions', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'terminate', deviceId, appId],
          exception: ProcessException('xcrun', <String>[]),
        ),
      );

      expect(
        () async => simControl.stopApp(deviceId, appId),
        throwsToolExit(message: 'Unable to terminate'),
      );
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });

    testWithoutContext('simulator stopApp handles null app package', () async {
      final IOSSimulator iosSimulator = IOSSimulator(
        'x',
        name: 'Testo',
        simulatorCategory: 'NaN',
        simControl: simControl,
        logger: logger,
      );

      expect(await iosSimulator.stopApp(null), isFalse);
    });

    testWithoutContext('listAvailableIOSRuntimes succeeds', () async {
      const String validRuntimesOutput = '''
{
  "runtimes" : [
    {
      "bundlePath" : "/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS 15.4.simruntime",
      "buildversion" : "19E240",
      "platform" : "iOS",
      "runtimeRoot" : "/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS 15.4.simruntime/Contents/Resources/RuntimeRoot",
      "identifier" : "com.apple.CoreSimulator.SimRuntime.iOS-15-4",
      "version" : "15.4",
      "isInternal" : false,
      "isAvailable" : true,
      "name" : "iOS 15.4",
      "supportedDeviceTypes" : [
        {
          "bundlePath" : "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/DeviceTypes/iPhone 6s.simdevicetype",
          "name" : "iPhone 6s",
          "identifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-6s",
          "productFamily" : "iPhone"
        },
        {
          "bundlePath" : "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/DeviceTypes/iPhone 6s Plus.simdevicetype",
          "name" : "iPhone 6s Plus",
          "identifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-6s-Plus",
          "productFamily" : "iPhone"
        }
      ]
    },
    {
      "bundlePath" : "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS.simruntime",
      "buildversion" : "20E247",
      "platform" : "iOS",
      "runtimeRoot" : "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS.simruntime/Contents/Resources/RuntimeRoot",
      "identifier" : "com.apple.CoreSimulator.SimRuntime.iOS-16-4",
      "version" : "16.4",
      "isInternal" : false,
      "isAvailable" : true,
      "name" : "iOS 16.4",
      "supportedDeviceTypes" : [
        {
          "bundlePath" : "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/DeviceTypes/iPhone 8.simdevicetype",
          "name" : "iPhone 8",
          "identifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-8",
          "productFamily" : "iPhone"
        },
        {
          "bundlePath" : "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/DeviceTypes/iPhone 8 Plus.simdevicetype",
          "name" : "iPhone 8 Plus",
          "identifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-8-Plus",
          "productFamily" : "iPhone"
        }
      ]
    },
    {
      "bundlePath" : "/Library/Developer/CoreSimulator/Volumes/iOS_21A5268h/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS 17.0.simruntime",
      "buildversion" : "21A5268h",
      "platform" : "iOS",
      "runtimeRoot" : "/Library/Developer/CoreSimulator/Volumes/iOS_21A5268h/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS 17.0.simruntime/Contents/Resources/RuntimeRoot",
      "identifier" : "com.apple.CoreSimulator.SimRuntime.iOS-17-0",
      "version" : "17.0",
      "isInternal" : false,
      "isAvailable" : true,
      "name" : "iOS 17.0",
      "supportedDeviceTypes" : [
        {
          "bundlePath" : "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/DeviceTypes/iPhone 8.simdevicetype",
          "name" : "iPhone 8",
          "identifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-8",
          "productFamily" : "iPhone"
        },
        {
          "bundlePath" : "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/DeviceTypes/iPhone 8 Plus.simdevicetype",
          "name" : "iPhone 8 Plus",
          "identifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-8-Plus",
          "productFamily" : "iPhone"
        }
      ]
    }
  ]
}

''';
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'list', 'runtimes', 'available', 'iOS', '--json'],
          stdout: validRuntimesOutput,
        ),
      );

      final List<IOSSimulatorRuntime> runtimes = await simControl.listAvailableIOSRuntimes();

      final IOSSimulatorRuntime runtime1 = runtimes[0];
      expect(
        runtime1.bundlePath,
        '/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS 15.4.simruntime',
      );
      expect(runtime1.buildVersion, '19E240');
      expect(runtime1.platform, 'iOS');
      expect(
        runtime1.runtimeRoot,
        '/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS 15.4.simruntime/Contents/Resources/RuntimeRoot',
      );
      expect(runtime1.identifier, 'com.apple.CoreSimulator.SimRuntime.iOS-15-4');
      expect(runtime1.version, Version(15, 4, null));
      expect(runtime1.isInternal, false);
      expect(runtime1.isAvailable, true);
      expect(runtime1.name, 'iOS 15.4');

      final IOSSimulatorRuntime runtime2 = runtimes[1];
      expect(
        runtime2.bundlePath,
        '/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS.simruntime',
      );
      expect(runtime2.buildVersion, '20E247');
      expect(runtime2.platform, 'iOS');
      expect(
        runtime2.runtimeRoot,
        '/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS.simruntime/Contents/Resources/RuntimeRoot',
      );
      expect(runtime2.identifier, 'com.apple.CoreSimulator.SimRuntime.iOS-16-4');
      expect(runtime2.version, Version(16, 4, null));
      expect(runtime2.isInternal, false);
      expect(runtime2.isAvailable, true);
      expect(runtime2.name, 'iOS 16.4');

      final IOSSimulatorRuntime runtime3 = runtimes[2];
      expect(
        runtime3.bundlePath,
        '/Library/Developer/CoreSimulator/Volumes/iOS_21A5268h/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS 17.0.simruntime',
      );
      expect(runtime3.buildVersion, '21A5268h');
      expect(runtime3.platform, 'iOS');
      expect(
        runtime3.runtimeRoot,
        '/Library/Developer/CoreSimulator/Volumes/iOS_21A5268h/Library/Developer/CoreSimulator/Profiles/Runtimes/iOS 17.0.simruntime/Contents/Resources/RuntimeRoot',
      );
      expect(runtime3.identifier, 'com.apple.CoreSimulator.SimRuntime.iOS-17-0');
      expect(runtime3.version, Version(17, 0, null));
      expect(runtime3.isInternal, false);
      expect(runtime3.isAvailable, true);
      expect(runtime3.name, 'iOS 17.0');
    });

    testWithoutContext('listAvailableIOSRuntimes handles bad simctl output', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['xcrun', 'simctl', 'list', 'runtimes', 'available', 'iOS', '--json'],
          stdout: 'Install Started',
        ),
      );

      final List<IOSSimulatorRuntime> runtimes = await simControl.listAvailableIOSRuntimes();

      expect(runtimes, isEmpty);
      expect(logger.errorText, contains('simctl returned non-JSON response:'));
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });
  });

  group('startApp', () {
    late FakePlistParser testPlistParser;
    late FakeSimControl simControl;
    late Xcode xcode;
    late BufferLogger logger;

    setUp(() {
      simControl = FakeSimControl();
      xcode = Xcode.test(processManager: FakeProcessManager.any());
      testPlistParser = FakePlistParser();
      logger = BufferLogger.test();
    });

    testUsingContext(
      "startApp uses compiled app's Info.plist to find CFBundleIdentifier",
      () async {
        final IOSSimulator device = IOSSimulator(
          'x',
          name: 'iPhone SE',
          simulatorCategory: 'iOS 11.2',
          simControl: simControl,
          logger: logger,
        );
        testPlistParser.setProperty('CFBundleIdentifier', 'correct');

        final Directory mockDir = globals.fs.currentDirectory;
        final IOSApp package = PrebuiltIOSApp(
          projectBundleId: 'incorrect',
          bundleName: 'name',
          uncompressedBundle: mockDir,
          applicationPackage: mockDir,
        );

        const BuildInfo mockInfo = BuildInfo(
          BuildMode.debug,
          'flavor',
          treeShakeIcons: false,
          packageConfigPath: '.dart_tool/package_config.json',
        );
        final DebuggingOptions mockOptions = DebuggingOptions.disabled(mockInfo);
        await device.startApp(package, prebuiltApplication: true, debuggingOptions: mockOptions);

        expect(simControl.requests.single.appIdentifier, 'correct');
      },
      overrides: <Type, Generator>{
        PlistParser: () => testPlistParser,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Xcode: () => xcode,
      },
    );

    testUsingContext(
      'startApp fails when cannot find CFBundleIdentifier',
      () async {
        final IOSSimulator device = IOSSimulator(
          'x',
          name: 'iPhone SE',
          simulatorCategory: 'iOS 11.2',
          simControl: simControl,
          logger: logger,
        );

        final Directory mockDir = globals.fs.currentDirectory;
        final IOSApp package = PrebuiltIOSApp(
          projectBundleId: 'incorrect',
          bundleName: 'name',
          uncompressedBundle: mockDir,
          applicationPackage: mockDir,
        );

        const BuildInfo mockInfo = BuildInfo(
          BuildMode.debug,
          'flavor',
          treeShakeIcons: false,
          packageConfigPath: '.dart_tool/package_config.json',
        );
        final DebuggingOptions mockOptions = DebuggingOptions.disabled(mockInfo);
        final LaunchResult result = await device.startApp(
          package,
          prebuiltApplication: true,
          debuggingOptions: mockOptions,
        );

        expect(result.started, isFalse);
        expect(simControl.requests, isEmpty);
        expect(
          logger.errorText,
          contains('Invalid prebuilt iOS app. Info.plist does not contain bundle identifier'),
        );
      },
      overrides: <Type, Generator>{
        PlistParser: () => testPlistParser,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Logger: () => logger,
        Xcode: () => xcode,
      },
    );

    testUsingContext(
      'startApp forwards all supported debugging options',
      () async {
        final IOSSimulator device = IOSSimulator(
          'x',
          name: 'iPhone SE',
          simulatorCategory: 'iOS 11.2',
          simControl: simControl,
          logger: logger,
        );
        testPlistParser.setProperty('CFBundleIdentifier', 'correct');

        final Directory mockDir = globals.fs.currentDirectory;
        final IOSApp package = PrebuiltIOSApp(
          projectBundleId: 'correct',
          bundleName: 'name',
          uncompressedBundle: mockDir,
          applicationPackage: mockDir,
        );

        const BuildInfo mockInfo = BuildInfo(
          BuildMode.debug,
          'flavor',
          treeShakeIcons: false,
          packageConfigPath: '.dart_tool/package_config.json',
        );
        final DebuggingOptions mockOptions = DebuggingOptions.enabled(
          mockInfo,
          enableSoftwareRendering: true,
          traceSystrace: true,
          traceToFile: 'path/to/trace.binpb',
          startPaused: true,
          disableServiceAuthCodes: true,
          skiaDeterministicRendering: true,
          useTestFonts: true,
          traceSkia: true,
          traceAllowlist: 'foo,bar',
          traceSkiaAllowlist: 'skia.a,skia.b',
          endlessTraceBuffer: true,
          dumpSkpOnShaderCompilation: true,
          verboseSystemLogs: true,
          cacheSkSL: true,
          purgePersistentCache: true,
          dartFlags: '--baz',
          enableImpeller: ImpellerStatus.disabled,
          nullAssertions: true,
          hostVmServicePort: 0,
        );

        await device.startApp(package, prebuiltApplication: true, debuggingOptions: mockOptions);
        expect(
          simControl.requests.single.launchArgs,
          unorderedEquals(<String>[
            '--enable-dart-profiling',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-software-rendering',
            '--trace-systrace',
            '--trace-to-file="path/to/trace.binpb"',
            '--start-paused',
            '--disable-service-auth-codes',
            '--skia-deterministic-rendering',
            '--use-test-fonts',
            '--trace-skia',
            '--trace-allowlist="foo,bar"',
            '--trace-skia-allowlist="skia.a,skia.b"',
            '--endless-trace-buffer',
            '--dump-skp-on-shader-compilation',
            '--verbose-logging',
            '--cache-sksl',
            '--purge-persistent-cache',
            '--enable-impeller=false',
            '--dart-flags=--baz,--null_assertions',
            '--vm-service-port=0',
          ]),
        );
      },
      overrides: <Type, Generator>{
        PlistParser: () => testPlistParser,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Xcode: () => xcode,
      },
    );

    testUsingContext(
      'startApp using route',
      () async {
        final IOSSimulator device = IOSSimulator(
          'x',
          name: 'iPhone SE',
          simulatorCategory: 'iOS 11.2',
          simControl: simControl,
          logger: logger,
        );
        testPlistParser.setProperty('CFBundleIdentifier', 'correct');

        final Directory mockDir = globals.fs.currentDirectory;
        final IOSApp package = PrebuiltIOSApp(
          projectBundleId: 'correct',
          bundleName: 'name',
          uncompressedBundle: mockDir,
          applicationPackage: mockDir,
        );

        const BuildInfo mockInfo = BuildInfo(
          BuildMode.debug,
          'flavor',
          treeShakeIcons: false,
          packageConfigPath: '.dart_tool/package_config.json',
        );
        final DebuggingOptions mockOptions = DebuggingOptions.enabled(
          mockInfo,
          enableSoftwareRendering: true,
        );
        await device.startApp(
          package,
          prebuiltApplication: true,
          debuggingOptions: mockOptions,
          route: '/animation',
        );

        expect(simControl.requests.single.launchArgs, contains('--route=/animation'));
      },
      overrides: <Type, Generator>{
        PlistParser: () => testPlistParser,
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        Xcode: () => xcode,
      },
    );
  });

  group('IOSDevice.isSupportedForProject', () {
    late FakeSimControl simControl;
    late Xcode xcode;

    setUp(() {
      simControl = FakeSimControl();
      xcode = Xcode.test(processManager: FakeProcessManager.any());
    });

    testUsingContext(
      'is true on module project',
      () async {
        globals.fs.file('pubspec.yaml')
          ..createSync()
          ..writeAsStringSync(r'''
name: example

flutter:
  module: {}
''');
        final FlutterProject flutterProject = FlutterProject.fromDirectoryTest(
          globals.fs.currentDirectory,
        );

        final IOSSimulator simulator = IOSSimulator(
          'test',
          name: 'iPhone 11',
          simControl: simControl,
          simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
          logger: logger,
        );
        expect(simulator.isSupportedForProject(flutterProject), true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        Xcode: () => xcode,
      },
    );

    testUsingContext(
      'is true with editable host app',
      () async {
        globals.fs.file('pubspec.yaml').createSync();
        globals.fs.directory('ios').createSync();
        final FlutterProject flutterProject = FlutterProject.fromDirectoryTest(
          globals.fs.currentDirectory,
        );

        final IOSSimulator simulator = IOSSimulator(
          'test',
          name: 'iPhone 11',
          simControl: simControl,
          simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
          logger: logger,
        );
        expect(simulator.isSupportedForProject(flutterProject), true);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        Xcode: () => xcode,
      },
    );

    testUsingContext(
      'is false with no host app and no module',
      () async {
        globals.fs.file('pubspec.yaml').createSync();
        final FlutterProject flutterProject = FlutterProject.fromDirectoryTest(
          globals.fs.currentDirectory,
        );

        final IOSSimulator simulator = IOSSimulator(
          'test',
          name: 'iPhone 11',
          simControl: simControl,
          simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
          logger: logger,
        );
        expect(simulator.isSupportedForProject(flutterProject), false);
      },
      overrides: <Type, Generator>{
        FileSystem: () => MemoryFileSystem.test(),
        ProcessManager: () => FakeProcessManager.any(),
        Xcode: () => xcode,
      },
    );

    testUsingContext('createDevFSWriter returns a LocalDevFSWriter', () {
      final IOSSimulator simulator = IOSSimulator(
        'test',
        name: 'iPhone 11',
        simControl: simControl,
        simulatorCategory: 'com.apple.CoreSimulator.SimRuntime.iOS-11-3',
        logger: logger,
      );

      expect(simulator.createDevFSWriter(null, ''), isA<LocalDevFSWriter>());
    });
  });
}

class FakeIosProject extends Fake implements IosProject {
  @override
  Future<String> productBundleIdentifier(BuildInfo? buildInfo) async => 'com.example.test';

  @override
  Future<String> productName(BuildInfo? buildInfo) async => 'My Super Awesome App';
}

class FakeSimControl extends Fake implements SimControl {
  final List<LaunchRequest> requests = <LaunchRequest>[];

  @override
  Future<RunResult> launch(
    String deviceId,
    String appIdentifier, [
    List<String>? launchArgs,
  ]) async {
    requests.add(LaunchRequest(appIdentifier, launchArgs));
    return RunResult(ProcessResult(0, 0, '', ''), <String>['test']);
  }

  @override
  Future<RunResult> install(String deviceId, String appPath) async {
    return RunResult(ProcessResult(0, 0, '', ''), <String>['test']);
  }
}

class LaunchRequest {
  const LaunchRequest(this.appIdentifier, this.launchArgs);

  final String appIdentifier;
  final List<String>? launchArgs;
}
