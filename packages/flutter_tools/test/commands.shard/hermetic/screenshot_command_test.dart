// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/screenshot.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_devices.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  setUpAll(() {
    Cache.disableLocking();
  });

  group('Validate screenshot options', () {
    testUsingContext('rasterizer and skia screenshots do not require a device', () async {
      // Throw a specific exception when attempting to make a VM Service connection to
      // verify that we've made it past the initial validation.
      openChannelForTesting = (
        String url, {
        CompressionOptions? compression,
        Logger? logger,
      }) async {
        expect(url, 'ws://localhost:8181/ws');
        throw Exception('dummy');
      };

      await expectLater(
        () => createTestCommandRunner(
          ScreenshotCommand(fs: MemoryFileSystem.test()),
        ).run(<String>['screenshot', '--type=skia', '--vm-service-url=http://localhost:8181']),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'message',
            contains('dummy'),
          ),
        ),
      );
    });

    testUsingContext('rasterizer and skia screenshots require VM Service uri', () async {
      await expectLater(
        () => createTestCommandRunner(
          ScreenshotCommand(fs: MemoryFileSystem.test()),
        ).run(<String>['screenshot', '--type=skia']),
        throwsToolExit(message: 'VM Service URI must be specified for screenshot type skia'),
      );
    });

    testUsingContext('device screenshots require device', () async {
      await expectLater(
        () => createTestCommandRunner(
          ScreenshotCommand(fs: MemoryFileSystem.test()),
        ).run(<String>['screenshot']),
        throwsToolExit(message: 'Must have a connected device for screenshot type device'),
      );
    });

    testUsingContext('device screenshots cannot provided VM Service', () async {
      await expectLater(
        () => createTestCommandRunner(
          ScreenshotCommand(fs: MemoryFileSystem.test()),
        ).run(<String>['screenshot', '--vm-service-url=http://localhost:8181']),
        throwsToolExit(message: 'VM Service URI cannot be provided for screenshot type device'),
      );
    });
  });

  group('Screenshot file validation', () {
    testWithoutContext('successful in pwd', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      fs.file('test.png').createSync();
      fs.directory('sub_dir').createSync();
      fs.file('sub_dir/test.png').createSync();

      expect(() => ScreenshotCommand.checkOutput(fs.file('test.png'), fs), returnsNormally);
      expect(() => ScreenshotCommand.checkOutput(fs.file('sub_dir/test.png'), fs), returnsNormally);
    });

    testWithoutContext('failed in pwd', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      fs.directory('sub_dir').createSync();

      expect(
        () => ScreenshotCommand.checkOutput(fs.file('test.png'), fs),
        throwsToolExit(message: 'File was not created, ensure path is valid'),
      );
      expect(
        () => ScreenshotCommand.checkOutput(fs.file('../'), fs),
        throwsToolExit(message: 'File was not created, ensure path is valid'),
      );
      expect(
        () => ScreenshotCommand.checkOutput(fs.file('.'), fs),
        throwsToolExit(message: 'File was not created, ensure path is valid'),
      );
      expect(
        () => ScreenshotCommand.checkOutput(fs.file('/'), fs),
        throwsToolExit(message: 'File was not created, ensure path is valid'),
      );
      expect(
        () => ScreenshotCommand.checkOutput(fs.file('sub_dir/test.png'), fs),
        throwsToolExit(message: 'File was not created, ensure path is valid'),
      );
    });
  });

  group('Screenshot output validation', () {
    testWithoutContext('successful', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      fs.file('test.png').createSync();

      expect(
        () => ScreenshotCommand.ensureOutputIsNotJsonRpcError(fs.file('test.png')),
        returnsNormally,
      );
    });

    testWithoutContext('failed', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      fs.file('test.png').writeAsStringSync('{"jsonrpc":"2.0", "error":"something"}');

      expect(
        () => ScreenshotCommand.ensureOutputIsNotJsonRpcError(fs.file('test.png')),
        throwsToolExit(
          message: 'It appears the output file contains an error message, not valid output.',
        ),
      );
    });
  });

  group('Screenshot for devices unsupported for project', () {
    late _TestDeviceManager testDeviceManager;

    setUp(() {
      testDeviceManager = _TestDeviceManager(logger: BufferLogger.test());
    });

    testUsingContext(
      'should not throw for a single device',
      () async {
        final ScreenshotCommand command = ScreenshotCommand(fs: MemoryFileSystem.test());

        final _ScreenshotDevice deviceUnsupportedForProject = _ScreenshotDevice(
          id: '123',
          name: 'Device 1',
          isSupportedForProject: false,
        );

        testDeviceManager.devices = <Device>[deviceUnsupportedForProject];

        await createTestCommandRunner(command).run(<String>['screenshot']);
      },
      overrides: <Type, Generator>{DeviceManager: () => testDeviceManager},
    );

    testUsingContext(
      'should tool exit for multiple devices',
      () async {
        final ScreenshotCommand command = ScreenshotCommand(fs: MemoryFileSystem.test());

        final List<_ScreenshotDevice> devicesUnsupportedForProject = <_ScreenshotDevice>[
          _ScreenshotDevice(id: '123', name: 'Device 1', isSupportedForProject: false),
          _ScreenshotDevice(id: '456', name: 'Device 2', isSupportedForProject: false),
        ];

        testDeviceManager.devices = devicesUnsupportedForProject;

        await expectLater(
          () => createTestCommandRunner(command).run(<String>['screenshot']),
          throwsToolExit(message: 'Must have a connected device for screenshot type device'),
        );

        expect(
          testLogger.statusText,
          contains('''
More than one device connected; please specify a device with the '-d <deviceId>' flag, or use '-d all' to act on all devices.

Device 1 (mobile) • 123 • android • 1.2.3
Device 2 (mobile) • 456 • android • 1.2.3
'''),
        );
      },
      overrides: <Type, Generator>{DeviceManager: () => testDeviceManager},
    );
  });
}

class _ScreenshotDevice extends Fake implements Device {
  _ScreenshotDevice({required this.id, required this.name, required bool isSupportedForProject})
    : _isSupportedForProject = isSupportedForProject;

  @override
  final String name;

  @override
  final String id;

  final bool _isSupportedForProject;

  @override
  bool isSupportedForProject(FlutterProject flutterProject) => _isSupportedForProject;

  @override
  bool supportsScreenshot = true;

  @override
  bool get isConnected => true;

  @override
  bool isSupported() => true;

  @override
  bool ephemeral = true;

  @override
  DeviceConnectionInterface connectionInterface = DeviceConnectionInterface.attached;

  @override
  Future<void> takeScreenshot(File outputFile) async {
    outputFile.writeAsBytesSync(<int>[1, 2, 3, 4]);
  }

  @override
  Future<String> get targetPlatformDisplayName async => 'android';

  @override
  Future<String> get sdkNameAndVersion async => '1.2.3';

  @override
  Future<TargetPlatform> get targetPlatform => Future<TargetPlatform>.value(TargetPlatform.android);

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  Category get category => Category.mobile;
}

class _TestDeviceManager extends DeviceManager {
  _TestDeviceManager({required super.logger});
  List<Device> devices = <Device>[];

  @override
  List<DeviceDiscovery> get deviceDiscoverers {
    final FakePollingDeviceDiscovery discoverer = FakePollingDeviceDiscovery();
    devices.forEach(discoverer.addDevice);
    return <DeviceDiscovery>[discoverer];
  }
}
