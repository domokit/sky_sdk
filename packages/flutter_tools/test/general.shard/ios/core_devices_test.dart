// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/error_handling_io.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/ios/core_devices.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/macos/xcode.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

class LocalFileSystemFake extends Fake implements LocalFileSystem {
  MemoryFileSystem memoryFileSystem = MemoryFileSystem.test();

  @override
  Directory get systemTempDirectory => memoryFileSystem.systemTempDirectory;

  @override
  Directory directory(dynamic path) => memoryFileSystem.directory(path);

  @override
  File file(dynamic path) => memoryFileSystem.file(path);

  @override
  Context get path => memoryFileSystem.path;

  @override
  Future<void> dispose() async {
    _disposed = true;
  }

  @override
  bool get disposed => _disposed;

  bool _disposed = false;
}

void main() {
  late MemoryFileSystem fileSystem;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
  });

  group('Xcode prior to Core Device Control/Xcode 15', () {
    late BufferLogger logger;
    late FakeProcessManager fakeProcessManager;
    late Xcode xcode;
    late IOSCoreDeviceControl deviceControl;

    setUp(() {
      logger = BufferLogger.test();
      fakeProcessManager = FakeProcessManager.empty();
      final XcodeProjectInterpreter xcodeProjectInterpreter = XcodeProjectInterpreter.test(
        processManager: fakeProcessManager,
        version: Version(14, 0, 0),
      );
      xcode = Xcode.test(
        processManager: fakeProcessManager,
        xcodeProjectInterpreter: xcodeProjectInterpreter,
      );
      deviceControl = IOSCoreDeviceControl(
        logger: logger,
        processManager: fakeProcessManager,
        xcode: xcode,
        fileSystem: fileSystem,
      );
    });

    group('devicectl is not installed', () {
      testWithoutContext('fails to get device list', () async {
        final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl is not installed.'));
        expect(devices.isEmpty, isTrue);
      });

      testWithoutContext('fails to install app', () async {
        final bool status = await deviceControl.installApp(
          deviceId: 'device-id',
          bundlePath: '/path/to/bundle',
        );
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl is not installed.'));
        expect(status, isFalse);
      });

      testWithoutContext('fails to launch app', () async {
        final bool status = await deviceControl.launchApp(
          deviceId: 'device-id',
          bundleId: 'com.example.flutterApp',
        );
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl is not installed.'));
        expect(status, isFalse);
      });

      testWithoutContext('fails to check if app is installed', () async {
        final bool status = await deviceControl.isAppInstalled(
          deviceId: 'device-id',
          bundleId: 'com.example.flutterApp',
        );
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl is not installed.'));
        expect(status, isFalse);
      });
    });
  });

  group('Core Device Control', () {
    late BufferLogger logger;
    late FakeProcessManager fakeProcessManager;
    late Xcode xcode;
    late IOSCoreDeviceControl deviceControl;

    setUp(() {
      logger = BufferLogger.test();
      fakeProcessManager = FakeProcessManager.empty();
      // TODO(fujino): re-use fakeProcessManager
      xcode = Xcode.test(processManager: FakeProcessManager.any());
      deviceControl = IOSCoreDeviceControl(
        logger: logger,
        processManager: fakeProcessManager,
        xcode: xcode,
        fileSystem: fileSystem,
      );
    });

    group('install app', () {
      const String deviceId = 'device-id';
      const String bundlePath = '/path/to/com.example.flutterApp';

      testWithoutContext('Successful install', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "install",
      "app",
      "--device",
      "00001234-0001234A3C03401E",
      "build/ios/iphoneos/Runner.app",
      "--json-output",
      "/var/folders/wq/randompath/T/flutter_tools.rand0/core_devices.rand0/install_results.json"
    ],
    "commandType" : "devicectl.device.install.app",
    "environment" : {

    },
    "outcome" : "success",
    "version" : "341"
  },
  "result" : {
    "deviceIdentifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
    "installedApplications" : [
      {
        "bundleID" : "com.example.bundle",
        "databaseSequenceNumber" : 1230,
        "databaseUUID" : "1234A567-D890-1B23-BCF4-D5D67A8D901E",
        "installationURL" : "file:///private/var/containers/Bundle/Application/12345E6A-7F89-0C12-345E-F6A7E890CFF1/Runner.app/",
        "launchServicesIdentifier" : "unknown",
        "options" : {

        }
      }
    ]
  }
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('install_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'install',
              'app',
              '--device',
              deviceId,
              bundlePath,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.installApp(
          deviceId: deviceId,
          bundlePath: bundlePath,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, isEmpty);
        expect(tempFile, isNot(exists));
        expect(status, true);
      });

      testWithoutContext('devicectl fails install', () async {
        const String deviceControlOutput = '''
{
  "error" : {
    "code" : 1005,
    "domain" : "com.apple.dt.CoreDeviceError",
    "userInfo" : {
      "NSLocalizedDescription" : {
        "string" : "Could not obtain access to one or more requested file system resources because CoreDevice was unable to create bookmark data."
      },
      "NSUnderlyingError" : {
        "error" : {
          "code" : 260,
          "domain" : "NSCocoaErrorDomain",
          "userInfo" : {

          }
        }
      }
    }
  },
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "install",
      "app",
      "--device",
      "00001234-0001234A3C03401E",
      "/path/to/app",
      "--json-output",
      "/var/folders/wq/randompath/T/flutter_tools.rand0/core_devices.rand0/install_results.json"
    ],
    "commandType" : "devicectl.device.install.app",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "failed",
    "version" : "341"
  }
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('install_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'install',
              'app',
              '--device',
              deviceId,
              bundlePath,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
            exitCode: 1,
            stderr: '''
ERROR: Could not obtain access to one or more requested file system resources because CoreDevice was unable to create bookmark data. (com.apple.dt.CoreDeviceError error 1005.)
         NSURL = file:///path/to/app
--------------------------------------------------------------------------------
ERROR: The file couldn’t be opened because it doesn’t exist. (NSCocoaErrorDomain error 260.)
''',
          ),
        );

        final bool status = await deviceControl.installApp(
          deviceId: deviceId,
          bundlePath: bundlePath,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(
          logger.errorText,
          contains('ERROR: Could not obtain access to one or more requested file system'),
        );
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('fails install because of unexpected JSON', () async {
        const String deviceControlOutput = '''
{
  "valid_unexpected_json": true
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('install_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'install',
              'app',
              '--device',
              deviceId,
              bundlePath,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.installApp(
          deviceId: deviceId,
          bundlePath: bundlePath,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl returned unexpected JSON response'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('fails install because of invalid JSON', () async {
        const String deviceControlOutput = '''
invalid JSON
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('install_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'install',
              'app',
              '--device',
              deviceId,
              bundlePath,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.installApp(
          deviceId: deviceId,
          bundlePath: bundlePath,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl returned non-JSON response'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });
    });

    group('uninstall app', () {
      const String deviceId = 'device-id';
      const String bundleId = 'com.example.flutterApp';

      testWithoutContext('Successful uninstall', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "uninstall",
      "app",
      "--device",
      "00001234-0001234A3C03401E",
      "build/ios/iphoneos/Runner.app",
      "--json-output",
      "/var/folders/wq/randompath/T/flutter_tools.rand0/core_devices.rand0/uninstall_results.json"
    ],
    "commandType" : "devicectl.device.uninstall.app",
    "environment" : {

    },
    "outcome" : "success",
    "version" : "341"
  },
  "result" : {
    "deviceIdentifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
    "uninstalledApplications" : [
      {
        "bundleID" : "com.example.bundle"
      }
    ]
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('uninstall_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'uninstall',
              'app',
              '--device',
              deviceId,
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.uninstallApp(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, isEmpty);
        expect(tempFile, isNot(exists));
        expect(status, true);
      });

      testWithoutContext('devicectl fails uninstall', () async {
        const String deviceControlOutput = '''
{
  "error" : {
    "code" : 1005,
    "domain" : "com.apple.dt.CoreDeviceError",
    "userInfo" : {
      "NSLocalizedDescription" : {
        "string" : "Could not obtain access to one or more requested file system resources because CoreDevice was unable to create bookmark data."
      },
      "NSUnderlyingError" : {
        "error" : {
          "code" : 260,
          "domain" : "NSCocoaErrorDomain",
          "userInfo" : {

          }
        }
      }
    }
  },
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "uninstall",
      "app",
      "--device",
      "00001234-0001234A3C03401E",
      "com.example.flutterApp",
      "--json-output",
      "/var/folders/wq/randompath/T/flutter_tools.rand0/core_devices.rand0/uninstall_results.json"
    ],
    "commandType" : "devicectl.device.uninstall.app",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "failed",
    "version" : "341"
  }
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('uninstall_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'uninstall',
              'app',
              '--device',
              deviceId,
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
            exitCode: 1,
            stderr: '''
ERROR: Could not obtain access to one or more requested file system resources because CoreDevice was unable to create bookmark data. (com.apple.dt.CoreDeviceError error 1005.)
         NSURL = file:///path/to/app
--------------------------------------------------------------------------------
ERROR: The file couldn’t be opened because it doesn’t exist. (NSCocoaErrorDomain error 260.)
''',
          ),
        );

        final bool status = await deviceControl.uninstallApp(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(
          logger.errorText,
          contains('ERROR: Could not obtain access to one or more requested file system'),
        );
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('fails uninstall because of unexpected JSON', () async {
        const String deviceControlOutput = '''
{
  "valid_unexpected_json": true
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('uninstall_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'uninstall',
              'app',
              '--device',
              deviceId,
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.uninstallApp(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl returned unexpected JSON response'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('fails uninstall because of invalid JSON', () async {
        const String deviceControlOutput = '''
invalid JSON
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('uninstall_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'uninstall',
              'app',
              '--device',
              deviceId,
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.uninstallApp(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl returned non-JSON response'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });
    });

    group('launch app', () {
      const String deviceId = 'device-id';
      const String bundleId = 'com.example.flutterApp';

      testWithoutContext('Successful launch without launch args', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "process",
      "launch",
      "--device",
      "00001234-0001234A3C03401E",
      "com.example.flutterApp",
      "--json-output",
      "/var/folders/wq/randompath/T/flutter_tools.rand0/core_devices.rand0/install_results.json"
    ],
    "commandType" : "devicectl.device.process.launch",
    "environment" : {

    },
    "outcome" : "success",
    "version" : "341"
  },
  "result" : {
    "deviceIdentifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
    "launchOptions" : {
      "activatedWhenStarted" : true,
      "arguments" : [

      ],
      "environmentVariables" : {
        "TERM" : "vt100"
      },
      "platformSpecificOptions" : {

      },
      "startStopped" : false,
      "terminateExistingInstances" : false,
      "user" : {
        "active" : true
      }
    },
    "process" : {
      "auditToken" : [
        12345,
        678
      ],
      "executable" : "file:///private/var/containers/Bundle/Application/12345E6A-7F89-0C12-345E-F6A7E890CFF1/Runner.app/Runner",
      "processIdentifier" : 1234
    }
  }
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('launch_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'process',
              'launch',
              '--device',
              deviceId,
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.launchApp(deviceId: deviceId, bundleId: bundleId);

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, isEmpty);
        expect(tempFile, isNot(exists));
        expect(status, true);
      });

      testWithoutContext('Successful launch with launch args', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "process",
      "launch",
      "--device",
      "00001234-0001234A3C03401E",
      "com.example.flutterApp",
      "--arg1",
      "--arg2",
      "--json-output",
      "/var/folders/wq/randompath/T/flutter_tools.rand0/core_devices.rand0/install_results.json"
    ],
    "commandType" : "devicectl.device.process.launch",
    "environment" : {

    },
    "outcome" : "success",
    "version" : "341"
  },
  "result" : {
    "deviceIdentifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
    "launchOptions" : {
      "activatedWhenStarted" : true,
      "arguments" : [

      ],
      "environmentVariables" : {
        "TERM" : "vt100"
      },
      "platformSpecificOptions" : {

      },
      "startStopped" : false,
      "terminateExistingInstances" : false,
      "user" : {
        "active" : true
      }
    },
    "process" : {
      "auditToken" : [
        12345,
        678
      ],
      "executable" : "file:///private/var/containers/Bundle/Application/12345E6A-7F89-0C12-345E-F6A7E890CFF1/Runner.app/Runner",
      "processIdentifier" : 1234
    }
  }
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('launch_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'process',
              'launch',
              '--device',
              deviceId,
              bundleId,
              '--arg1',
              '--arg2',
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.launchApp(
          deviceId: deviceId,
          bundleId: bundleId,
          launchArguments: <String>['--arg1', '--arg2'],
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, isEmpty);
        expect(tempFile, isNot(exists));
        expect(status, true);
      });

      testWithoutContext('devicectl fails install', () async {
        const String deviceControlOutput = '''
{
  "error" : {
    "code" : -10814,
    "domain" : "NSOSStatusErrorDomain",
    "userInfo" : {
      "_LSFunction" : {
        "string" : "runEvaluator"
      },
      "_LSLine" : {
        "int" : 1608
      }
    }
  },
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "process",
      "launch",
      "--device",
      "00001234-0001234A3C03401E",
      "com.example.flutterApp",
      "--json-output",
      "/var/folders/wq/randompath/T/flutter_tools.rand0/core_devices.rand0/install_results.json"
    ],
    "commandType" : "devicectl.device.process.launch",
    "environment" : {

    },
    "outcome" : "failed",
    "version" : "341"
  }
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('launch_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'process',
              'launch',
              '--device',
              deviceId,
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
            exitCode: 1,
            stderr: '''
ERROR: The operation couldn?t be completed. (OSStatus error -10814.) (NSOSStatusErrorDomain error -10814.)
    _LSFunction = runEvaluator
    _LSLine = 1608
''',
          ),
        );

        final bool status = await deviceControl.launchApp(deviceId: deviceId, bundleId: bundleId);

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('ERROR: The operation couldn?t be completed.'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('fails launch because of unexpected JSON', () async {
        const String deviceControlOutput = '''
{
  "valid_unexpected_json": true
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('launch_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'process',
              'launch',
              '--device',
              deviceId,
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.launchApp(deviceId: deviceId, bundleId: bundleId);

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl returned unexpected JSON response'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('fails launch because of invalid JSON', () async {
        const String deviceControlOutput = '''
invalid JSON
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('launch_results.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'process',
              'launch',
              '--device',
              deviceId,
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.launchApp(deviceId: deviceId, bundleId: bundleId);

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl returned non-JSON response'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });
    });

    group('list apps', () {
      const String deviceId = 'device-id';
      const String bundleId = 'com.example.flutterApp';

      testWithoutContext('Successfully parses apps', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "info",
      "apps",
      "--device",
      "00001234-0001234A3C03401E",
      "--bundle-id",
      "com.example.flutterApp",
      "--json-output",
      "apps.txt"
    ],
    "commandType" : "devicectl.device.info.apps",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "success",
    "version" : "341"
  },
  "result" : {
    "apps" : [
      {
        "appClip" : false,
        "builtByDeveloper" : true,
        "bundleIdentifier" : "com.example.flutterApp",
        "bundleVersion" : "1",
        "defaultApp" : false,
        "hidden" : false,
        "internalApp" : false,
        "name" : "Bundle",
        "removable" : true,
        "url" : "file:///private/var/containers/Bundle/Application/12345E6A-7F89-0C12-345E-F6A7E890CFF1/Runner.app/",
        "version" : "1.0.0"
      },
      {
        "appClip" : true,
        "builtByDeveloper" : false,
        "bundleIdentifier" : "com.example.flutterApp2",
        "bundleVersion" : "2",
        "defaultApp" : true,
        "hidden" : true,
        "internalApp" : true,
        "name" : "Bundle 2",
        "removable" : false,
        "url" : "file:///private/var/containers/Bundle/Application/12345E6A-7F89-0C12-345E-F6A7E890CFF1/Runner.app/",
        "version" : "1.0.0"
      }
    ],
    "defaultAppsIncluded" : false,
    "deviceIdentifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
    "hiddenAppsIncluded" : false,
    "internalAppsIncluded" : false,
    "matchingBundleIdentifier" : "com.example.flutterApp",
    "removableAppsIncluded" : true
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_app_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'info',
              'apps',
              '--device',
              deviceId,
              '--bundle-id',
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final List<IOSCoreDeviceInstalledApp> apps = await deviceControl.getInstalledApps(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, isEmpty);
        expect(tempFile, isNot(exists));
        expect(apps.length, 2);

        expect(apps[0].appClip, isFalse);
        expect(apps[0].builtByDeveloper, isTrue);
        expect(apps[0].bundleIdentifier, 'com.example.flutterApp');
        expect(apps[0].bundleVersion, '1');
        expect(apps[0].defaultApp, isFalse);
        expect(apps[0].hidden, isFalse);
        expect(apps[0].internalApp, isFalse);
        expect(apps[0].name, 'Bundle');
        expect(apps[0].removable, isTrue);
        expect(
          apps[0].url,
          'file:///private/var/containers/Bundle/Application/12345E6A-7F89-0C12-345E-F6A7E890CFF1/Runner.app/',
        );
        expect(apps[0].version, '1.0.0');

        expect(apps[1].appClip, isTrue);
        expect(apps[1].builtByDeveloper, isFalse);
        expect(apps[1].bundleIdentifier, 'com.example.flutterApp2');
        expect(apps[1].bundleVersion, '2');
        expect(apps[1].defaultApp, isTrue);
        expect(apps[1].hidden, isTrue);
        expect(apps[1].internalApp, isTrue);
        expect(apps[1].name, 'Bundle 2');
        expect(apps[1].removable, isFalse);
        expect(
          apps[1].url,
          'file:///private/var/containers/Bundle/Application/12345E6A-7F89-0C12-345E-F6A7E890CFF1/Runner.app/',
        );
        expect(apps[1].version, '1.0.0');
      });

      testWithoutContext('Successfully find installed app', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "info",
      "apps",
      "--device",
      "00001234-0001234A3C03401E",
      "--bundle-id",
      "com.example.flutterApp",
      "--json-output",
      "apps.txt"
    ],
    "commandType" : "devicectl.device.info.apps",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "success",
    "version" : "341"
  },
  "result" : {
    "apps" : [
      {
        "appClip" : false,
        "builtByDeveloper" : true,
        "bundleIdentifier" : "com.example.flutterApp",
        "bundleVersion" : "1",
        "defaultApp" : false,
        "hidden" : false,
        "internalApp" : false,
        "name" : "Bundle",
        "removable" : true,
        "url" : "file:///private/var/containers/Bundle/Application/12345E6A-7F89-0C12-345E-F6A7E890CFF1/Runner.app/",
        "version" : "1.0.0"
      }
    ],
    "defaultAppsIncluded" : false,
    "deviceIdentifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
    "hiddenAppsIncluded" : false,
    "internalAppsIncluded" : false,
    "matchingBundleIdentifier" : "com.example.flutterApp",
    "removableAppsIncluded" : true
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_app_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'info',
              'apps',
              '--device',
              deviceId,
              '--bundle-id',
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.isAppInstalled(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, isEmpty);
        expect(tempFile, isNot(exists));
        expect(status, true);
      });

      testWithoutContext('Succeeds but does not find app', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "info",
      "apps",
      "--device",
      "00001234-0001234A3C03401E",
      "--bundle-id",
      "com.example.flutterApp",
      "--json-output",
      "apps.txt"
    ],
    "commandType" : "devicectl.device.info.apps",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "success",
    "version" : "341"
  },
  "result" : {
    "apps" : [
    ],
    "defaultAppsIncluded" : false,
    "deviceIdentifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
    "hiddenAppsIncluded" : false,
    "internalAppsIncluded" : false,
    "matchingBundleIdentifier" : "com.example.flutterApp",
    "removableAppsIncluded" : true
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_app_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'info',
              'apps',
              '--device',
              deviceId,
              '--bundle-id',
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.isAppInstalled(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, isEmpty);
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('devicectl fails to get apps', () async {
        const String deviceControlOutput = '''
{
  "error" : {
    "code" : 1000,
    "domain" : "com.apple.dt.CoreDeviceError",
    "userInfo" : {
      "NSLocalizedDescription" : {
        "string" : "The specified device was not found."
      }
    }
  },
  "info" : {
    "arguments" : [
      "devicectl",
      "device",
      "info",
      "apps",
      "--device",
      "00001234-0001234A3C03401E",
      "--bundle-id",
      "com.example.flutterApp",
      "--json-output",
      "apps.txt"
    ],
    "commandType" : "devicectl.device.info.apps",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "failed",
    "version" : "341"
  }
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_app_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'info',
              'apps',
              '--device',
              deviceId,
              '--bundle-id',
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
            exitCode: 1,
            stderr: '''
ERROR: The specified device was not found. (com.apple.dt.CoreDeviceError error 1000.)
''',
          ),
        );

        final bool status = await deviceControl.isAppInstalled(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('ERROR: The specified device was not found.'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('fails launch because of unexpected JSON', () async {
        const String deviceControlOutput = '''
{
  "valid_unexpected_json": true
}
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_app_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'info',
              'apps',
              '--device',
              deviceId,
              '--bundle-id',
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.isAppInstalled(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl returned unexpected JSON response'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });

      testWithoutContext('fails launch because of invalid JSON', () async {
        const String deviceControlOutput = '''
invalid JSON
''';
        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_app_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'device',
              'info',
              'apps',
              '--device',
              deviceId,
              '--bundle-id',
              bundleId,
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final bool status = await deviceControl.isAppInstalled(
          deviceId: deviceId,
          bundleId: bundleId,
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(logger.errorText, contains('devicectl returned non-JSON response'));
        expect(tempFile, isNot(exists));
        expect(status, false);
      });
    });

    group('list devices', () {
      testWithoutContext('Handles FileSystemException deleting temp directory', () async {
        final Directory tempDir = fileSystem.systemTempDirectory.childDirectory(
          'core_devices.rand0',
        );
        final File tempFile = tempDir.childFile('core_device_list.json');
        final List<String> args = <String>[
          'xcrun',
          'devicectl',
          'list',
          'devices',
          '--timeout',
          '5',
          '--json-output',
          tempFile.path,
        ];
        fakeProcessManager.addCommand(
          FakeCommand(
            command: args,
            onRun: (_) {
              // Simulate that this command threw and simultaneously the OS
              // deleted the temp directory
              expect(tempFile, exists);
              tempDir.deleteSync(recursive: true);
              expect(tempFile, isNot(exists));
              throw ProcessException(args.first, args.sublist(1));
            },
          ),
        );

        await deviceControl.getCoreDevices();
        expect(logger.errorText, contains('Error executing devicectl: ProcessException'));
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });

      testWithoutContext('Handles json file mysteriously disappearing', () async {
        final Directory tempDir = fileSystem.systemTempDirectory.childDirectory(
          'core_devices.rand0',
        );
        final File tempFile = tempDir.childFile('core_device_list.json');
        final List<String> args = <String>[
          'xcrun',
          'devicectl',
          'list',
          'devices',
          '--timeout',
          '5',
          '--json-output',
          tempFile.path,
        ];
        fakeProcessManager.addCommand(
          FakeCommand(
            command: args,
            onRun: (_) {
              // Simulate that this command deleted tempFile, did not create a
              // new one, and exited successfully
              expect(tempFile, exists);
              tempFile.deleteSync();
              expect(tempFile, isNot(exists));
            },
          ),
        );

        await expectLater(
          () => deviceControl.getCoreDevices(),
          throwsA(
            isA<StateError>().having(
              (StateError e) => e.message,
              'message',
              contains('Expected the file ${tempFile.path} to exist but it did not'),
            ),
          ),
        );
        expect(
          logger.errorText,
          contains(
            'After running the command xcrun devicectl list devices '
            '--timeout 5 --json-output ${tempFile.path} the file\n'
            '${tempFile.path} was expected to exist, but it did not',
          ),
        );
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });

      testWithoutContext('Handles file system disposal', () async {
        final LocalFileSystem localFs = LocalFileSystemFake();
        final ErrorHandlingFileSystem fs = ErrorHandlingFileSystem(
          delegate: localFs,
          platform: FakePlatform(),
        );
        deviceControl = IOSCoreDeviceControl(
          logger: logger,
          processManager: fakeProcessManager,
          xcode: xcode,
          fileSystem: fs,
        );
        final Directory tempDir = localFs.systemTempDirectory.childDirectory('core_devices.rand0');
        final File tempFile = tempDir.childFile('core_device_list.json');
        final List<String> args = <String>[
          'xcrun',
          'devicectl',
          'list',
          'devices',
          '--timeout',
          '5',
          '--json-output',
          tempFile.path,
        ];
        fakeProcessManager.addCommand(
          FakeCommand(
            command: args,
            onRun: (_) {
              // Simulate that the tool started shutting down and disposed the
              // file system, causing the temp directory to be deleted before
              // this program invocation returns a result.
              localFs.dispose();
              expect(localFs.disposed, true);
            },
          ),
        );

        final List<IOSCoreDevice> coreDevices = await deviceControl.getCoreDevices();
        expect(coreDevices, isEmpty);
        expect(
          logger.errorText,
          isNot(
            contains(
              'After running the command xcrun devicectl list devices '
              '--timeout 5 --json-output ${tempFile.path} the file\n'
              '${tempFile.path} was expected to exist, but it did not',
            ),
          ),
        );
        expect(fakeProcessManager, hasNoRemainingExpectations);
      });

      testWithoutContext('No devices', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "list",
      "devices",
      "--json-output",
      "core_device_list.json"
    ],
    "commandType" : "devicectl.list.devices",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "success",
    "version" : "325.3"
  },
  "result" : {
    "devices" : [

    ]
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'list',
              'devices',
              '--timeout',
              '5',
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(devices.isEmpty, isTrue);
      });

      testWithoutContext('All sections parsed', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "list",
      "devices",
      "--json-output",
      "core_device_list.json"
    ],
    "commandType" : "devicectl.list.devices",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "success",
    "version" : "325.3"
  },
  "result" : {
    "devices" : [
      {
        "capabilities" : [
        ],
        "connectionProperties" : {
        },
        "deviceProperties" : {
        },
        "hardwareProperties" : {
        },
        "identifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
        "visibilityClass" : "default"
      }
    ]
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'list',
              'devices',
              '--timeout',
              '5',
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
        expect(devices.length, 1);

        expect(devices[0].capabilities, isNotNull);
        expect(devices[0].connectionProperties, isNotNull);
        expect(devices[0].deviceProperties, isNotNull);
        expect(devices[0].hardwareProperties, isNotNull);
        expect(devices[0].coreDeviceIdentifier, '123456BB5-AEDE-7A22-B890-1234567890DD');
        expect(devices[0].visibilityClass, 'default');

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(tempFile, isNot(exists));
      });

      testWithoutContext('All sections parsed, device missing sections', () async {
        const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "list",
      "devices",
      "--json-output",
      "core_device_list.json"
    ],
    "commandType" : "devicectl.list.devices",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "success",
    "version" : "325.3"
  },
  "result" : {
    "devices" : [
      {
        "identifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
        "visibilityClass" : "default"
      }
    ]
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'list',
              'devices',
              '--timeout',
              '5',
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
        expect(devices.length, 1);

        expect(devices[0].capabilities, isEmpty);
        expect(devices[0].connectionProperties, isNull);
        expect(devices[0].deviceProperties, isNull);
        expect(devices[0].hardwareProperties, isNull);
        expect(devices[0].coreDeviceIdentifier, '123456BB5-AEDE-7A22-B890-1234567890DD');
        expect(devices[0].visibilityClass, 'default');

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(tempFile, isNot(exists));
      });

      testWithoutContext('capabilities parsed', () async {
        const String deviceControlOutput = '''
{
  "result" : {
    "devices" : [
      {
        "capabilities" : [
          {
            "featureIdentifier" : "com.apple.coredevice.feature.spawnexecutable",
            "name" : "Spawn Executable"
          },
          {
            "featureIdentifier" : "com.apple.coredevice.feature.launchapplication",
            "name" : "Launch Application"
          }
        ]
      }
    ]
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'list',
              'devices',
              '--timeout',
              '5',
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
        expect(devices.length, 1);

        expect(devices[0].capabilities.length, 2);
        expect(
          devices[0].capabilities[0].featureIdentifier,
          'com.apple.coredevice.feature.spawnexecutable',
        );
        expect(devices[0].capabilities[0].name, 'Spawn Executable');
        expect(
          devices[0].capabilities[1].featureIdentifier,
          'com.apple.coredevice.feature.launchapplication',
        );
        expect(devices[0].capabilities[1].name, 'Launch Application');

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(tempFile, isNot(exists));
      });

      testWithoutContext('connectionProperties parsed', () async {
        const String deviceControlOutput = '''
{
  "result" : {
    "devices" : [
      {
        "connectionProperties" : {
          "authenticationType" : "manualPairing",
          "isMobileDeviceOnly" : false,
          "lastConnectionDate" : "2023-06-15T15:29:00.082Z",
          "localHostnames" : [
            "Victorias-iPad.coredevice.local",
            "00001234-0001234A3C03401E.coredevice.local",
            "123456BB5-AEDE-7A22-B890-1234567890DD.coredevice.local"
          ],
          "pairingState" : "paired",
          "potentialHostnames" : [
            "00001234-0001234A3C03401E.coredevice.local",
            "123456BB5-AEDE-7A22-B890-1234567890DD.coredevice.local"
          ],
          "transportType" : "wired",
          "tunnelIPAddress" : "fdf1:23c4:cd56::1",
          "tunnelState" : "connected",
          "tunnelTransportProtocol" : "tcp"
        }
      }
    ]
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'list',
              'devices',
              '--timeout',
              '5',
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
        expect(devices.length, 1);

        expect(devices[0].connectionProperties?.authenticationType, 'manualPairing');
        expect(devices[0].connectionProperties?.isMobileDeviceOnly, false);
        expect(devices[0].connectionProperties?.lastConnectionDate, '2023-06-15T15:29:00.082Z');
        expect(devices[0].connectionProperties?.localHostnames, <String>[
          'Victorias-iPad.coredevice.local',
          '00001234-0001234A3C03401E.coredevice.local',
          '123456BB5-AEDE-7A22-B890-1234567890DD.coredevice.local',
        ]);
        expect(devices[0].connectionProperties?.pairingState, 'paired');
        expect(devices[0].connectionProperties?.potentialHostnames, <String>[
          '00001234-0001234A3C03401E.coredevice.local',
          '123456BB5-AEDE-7A22-B890-1234567890DD.coredevice.local',
        ]);
        expect(devices[0].connectionProperties?.transportType, 'wired');
        expect(devices[0].connectionProperties?.tunnelIPAddress, 'fdf1:23c4:cd56::1');
        expect(devices[0].connectionProperties?.tunnelState, 'connected');
        expect(devices[0].connectionProperties?.tunnelTransportProtocol, 'tcp');

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(tempFile, isNot(exists));
      });

      testWithoutContext('deviceProperties parsed', () async {
        const String deviceControlOutput = '''
{
  "result" : {
    "devices" : [
      {
        "deviceProperties" : {
          "bootedFromSnapshot" : true,
          "bootedSnapshotName" : "com.apple.os.update-123456",
          "bootState" : "booted",
          "ddiServicesAvailable" : true,
          "developerModeStatus" : "enabled",
          "hasInternalOSBuild" : false,
          "name" : "iPadName",
          "osBuildUpdate" : "21A5248v",
          "osVersionNumber" : "17.0",
          "rootFileSystemIsWritable" : false,
          "screenViewingURL" : "coredevice-devices:/viewDeviceByUUID?uuid=123456BB5-AEDE-7A22-B890-1234567890DD"
        }
      }
    ]
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'list',
              'devices',
              '--timeout',
              '5',
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
        expect(devices.length, 1);

        expect(devices[0].deviceProperties?.bootedFromSnapshot, true);
        expect(devices[0].deviceProperties?.bootedSnapshotName, 'com.apple.os.update-123456');
        expect(devices[0].deviceProperties?.bootState, 'booted');
        expect(devices[0].deviceProperties?.ddiServicesAvailable, true);
        expect(devices[0].deviceProperties?.developerModeStatus, 'enabled');
        expect(devices[0].deviceProperties?.hasInternalOSBuild, false);
        expect(devices[0].deviceProperties?.name, 'iPadName');
        expect(devices[0].deviceProperties?.osBuildUpdate, '21A5248v');
        expect(devices[0].deviceProperties?.osVersionNumber, '17.0');
        expect(devices[0].deviceProperties?.rootFileSystemIsWritable, false);
        expect(
          devices[0].deviceProperties?.screenViewingURL,
          'coredevice-devices:/viewDeviceByUUID?uuid=123456BB5-AEDE-7A22-B890-1234567890DD',
        );

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(tempFile, isNot(exists));
      });

      testWithoutContext('hardwareProperties parsed', () async {
        const String deviceControlOutput = r'''
{
  "result" : {
    "devices" : [
      {
        "hardwareProperties" : {
          "cpuType" : {
            "name" : "arm64e",
            "subType" : 2,
            "type" : 16777228
          },
          "deviceType" : "iPad",
          "ecid" : 12345678903408542,
          "hardwareModel" : "J617AP",
          "internalStorageCapacity" : 128000000000,
          "marketingName" : "iPad Pro (11-inch) (4th generation)\"",
          "platform" : "iOS",
          "productType" : "iPad14,3",
          "serialNumber" : "HC123DHCQV",
          "supportedCPUTypes" : [
            {
              "name" : "arm64e",
              "subType" : 2,
              "type" : 16777228
            },
            {
              "name" : "arm64",
              "subType" : 0,
              "type" : 16777228
            }
          ],
          "supportedDeviceFamilies" : [
            1,
            2
          ],
          "thinningProductType" : "iPad14,3-A",
          "udid" : "00001234-0001234A3C03401E"
        }
      }
    ]
  }
}
''';

        final File tempFile = fileSystem.systemTempDirectory
            .childDirectory('core_devices.rand0')
            .childFile('core_device_list.json');
        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[
              'xcrun',
              'devicectl',
              'list',
              'devices',
              '--timeout',
              '5',
              '--json-output',
              tempFile.path,
            ],
            onRun: (_) {
              expect(tempFile, exists);
              tempFile.writeAsStringSync(deviceControlOutput);
            },
          ),
        );

        final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
        expect(devices.length, 1);

        expect(devices[0].hardwareProperties?.cpuType, isNotNull);
        expect(devices[0].hardwareProperties?.cpuType?.name, 'arm64e');
        expect(devices[0].hardwareProperties?.cpuType?.subType, 2);
        expect(devices[0].hardwareProperties?.cpuType?.cpuType, 16777228);
        expect(devices[0].hardwareProperties?.deviceType, 'iPad');
        expect(devices[0].hardwareProperties?.ecid, 12345678903408542);
        expect(devices[0].hardwareProperties?.hardwareModel, 'J617AP');
        expect(devices[0].hardwareProperties?.internalStorageCapacity, 128000000000);
        expect(
          devices[0].hardwareProperties?.marketingName,
          'iPad Pro (11-inch) (4th generation)"',
        );
        expect(devices[0].hardwareProperties?.platform, 'iOS');
        expect(devices[0].hardwareProperties?.productType, 'iPad14,3');
        expect(devices[0].hardwareProperties?.serialNumber, 'HC123DHCQV');
        expect(devices[0].hardwareProperties?.supportedCPUTypes, isNotNull);
        expect(devices[0].hardwareProperties?.supportedCPUTypes?[0].name, 'arm64e');
        expect(devices[0].hardwareProperties?.supportedCPUTypes?[0].subType, 2);
        expect(devices[0].hardwareProperties?.supportedCPUTypes?[0].cpuType, 16777228);
        expect(devices[0].hardwareProperties?.supportedCPUTypes?[1].name, 'arm64');
        expect(devices[0].hardwareProperties?.supportedCPUTypes?[1].subType, 0);
        expect(devices[0].hardwareProperties?.supportedCPUTypes?[1].cpuType, 16777228);
        expect(devices[0].hardwareProperties?.supportedDeviceFamilies, <int>[1, 2]);
        expect(devices[0].hardwareProperties?.thinningProductType, 'iPad14,3-A');

        expect(devices[0].hardwareProperties?.udid, '00001234-0001234A3C03401E');

        expect(fakeProcessManager, hasNoRemainingExpectations);
        expect(tempFile, isNot(exists));
      });

      group('Handles errors', () {
        testWithoutContext('invalid json', () async {
          const String deviceControlOutput = '''Invalid JSON''';

          final File tempFile = fileSystem.systemTempDirectory
              .childDirectory('core_devices.rand0')
              .childFile('core_device_list.json');
          fakeProcessManager.addCommand(
            FakeCommand(
              command: <String>[
                'xcrun',
                'devicectl',
                'list',
                'devices',
                '--timeout',
                '5',
                '--json-output',
                tempFile.path,
              ],
              onRun: (_) {
                expect(tempFile, exists);
                tempFile.writeAsStringSync(deviceControlOutput);
              },
            ),
          );

          final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
          expect(devices.isEmpty, isTrue);
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(logger.errorText, contains('devicectl returned non-JSON response: Invalid JSON'));
        });

        testWithoutContext('unexpected json', () async {
          const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "list",
      "devices",
      "--json-output",
      "device_list.json"
    ],
    "commandType" : "devicectl.list.devices",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "success",
    "version" : "325.3"
  },
  "result" : [

  ]
}
''';

          final File tempFile = fileSystem.systemTempDirectory
              .childDirectory('core_devices.rand0')
              .childFile('core_device_list.json');
          fakeProcessManager.addCommand(
            FakeCommand(
              command: <String>[
                'xcrun',
                'devicectl',
                'list',
                'devices',
                '--timeout',
                '5',
                '--json-output',
                tempFile.path,
              ],
              onRun: (_) {
                expect(tempFile, exists);
                tempFile.writeAsStringSync(deviceControlOutput);
              },
            ),
          );

          final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices();
          expect(devices.isEmpty, isTrue);
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(logger.errorText, contains('devicectl returned unexpected JSON response:'));
        });

        testWithoutContext('When timeout is below minimum, default to minimum', () async {
          const String deviceControlOutput = '''
{
  "info" : {
    "arguments" : [
      "devicectl",
      "list",
      "devices",
      "--json-output",
      "core_device_list.json"
    ],
    "commandType" : "devicectl.list.devices",
    "environment" : {
      "TERM" : "xterm-256color"
    },
    "outcome" : "success",
    "version" : "325.3"
  },
  "result" : {
    "devices" : [
      {
        "identifier" : "123456BB5-AEDE-7A22-B890-1234567890DD",
        "visibilityClass" : "default"
      }
    ]
  }
}
''';

          final File tempFile = fileSystem.systemTempDirectory
              .childDirectory('core_devices.rand0')
              .childFile('core_device_list.json');
          fakeProcessManager.addCommand(
            FakeCommand(
              command: <String>[
                'xcrun',
                'devicectl',
                'list',
                'devices',
                '--timeout',
                '5',
                '--json-output',
                tempFile.path,
              ],
              onRun: (_) {
                expect(tempFile, exists);
                tempFile.writeAsStringSync(deviceControlOutput);
              },
            ),
          );

          final List<IOSCoreDevice> devices = await deviceControl.getCoreDevices(
            timeout: const Duration(seconds: 2),
          );
          expect(devices.isNotEmpty, isTrue);
          expect(fakeProcessManager, hasNoRemainingExpectations);
          expect(
            logger.errorText,
            contains(
              'Timeout of 2 seconds is below the minimum timeout value '
              'for devicectl. Changing the timeout to the minimum value of 5.',
            ),
          );
        });
      });
    });
  });
}
