// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/android/android_device.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

void main() {
  testWithoutContext('AndroidDevicePortForwarder returns the generated host '
      'port from stdout', () async {
    final AndroidDevicePortForwarder forwarder = AndroidDevicePortForwarder(
      adbPath: 'adb',
      deviceId: '1',
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1', 'forward', 'tcp:0', 'tcp:123'],
          stdout: '456',
        ),
      ]),
      logger: BufferLogger.test(),
    );

    expect(await forwarder.forward(123), 456);
  });

  testWithoutContext('AndroidDevicePortForwarder returns the supplied host '
      'port when stdout is empty', () async {
    final AndroidDevicePortForwarder forwarder = AndroidDevicePortForwarder(
      adbPath: 'adb',
      deviceId: '1',
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: <String>['adb', '-s', '1', 'forward', 'tcp:456', 'tcp:123']),
      ]),
      logger: BufferLogger.test(),
    );

    expect(await forwarder.forward(123, hostPort: 456), 456);
  });

  testWithoutContext('AndroidDevicePortForwarder returns the supplied host port '
      'when stdout is the host port', () async {
    final AndroidDevicePortForwarder forwarder = AndroidDevicePortForwarder(
      adbPath: 'adb',
      deviceId: '1',
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1', 'forward', 'tcp:456', 'tcp:123'],
          stdout: '456',
        ),
      ]),
      logger: BufferLogger.test(),
    );

    expect(await forwarder.forward(123, hostPort: 456), 456);
  });

  testWithoutContext('AndroidDevicePortForwarder throws an exception when stdout '
      'is not blank nor the host port', () async {
    final AndroidDevicePortForwarder forwarder = AndroidDevicePortForwarder(
      adbPath: 'adb',
      deviceId: '1',
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', '-s', '1', 'forward', 'tcp:456', 'tcp:123'],
          stdout: '123456',
        ),
      ]),
      logger: BufferLogger.test(),
    );

    expect(forwarder.forward(123, hostPort: 456), throwsProcessException());
  });

  testWithoutContext('AndroidDevicePortForwarder forwardedPorts returns empty '
      'list when forward failed', () {
    final AndroidDevicePortForwarder forwarder = AndroidDevicePortForwarder(
      adbPath: 'adb',
      deviceId: '1',
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: <String>['adb', '-s', '1', 'forward', '--list'], exitCode: 1),
      ]),
      logger: BufferLogger.test(),
    );

    expect(forwarder.forwardedPorts, isEmpty);
  });

  testWithoutContext('disposing device disposes the portForwarder', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['adb', '-s', '1', 'forward', 'tcp:0', 'tcp:123'],
        stdout: '456',
      ),
      const FakeCommand(
        command: <String>['adb', '-s', '1', 'forward', '--list'],
        stdout: '1234 tcp:456 tcp:123',
      ),
      const FakeCommand(command: <String>['adb', '-s', '1', 'forward', '--remove', 'tcp:456']),
    ]);
    final AndroidDevicePortForwarder forwarder = AndroidDevicePortForwarder(
      adbPath: 'adb',
      deviceId: '1',
      processManager: processManager,
      logger: BufferLogger.test(),
    );

    expect(await forwarder.forward(123), equals(456));

    await forwarder.dispose();

    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('failures to unforward port do not throw if the forward is missing', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['adb', '-s', '1', 'forward', '--remove', 'tcp:456'],
        stderr: "error: listener 'tcp:456' not found",
        exitCode: 1,
      ),
    ]);
    final AndroidDevicePortForwarder forwarder = AndroidDevicePortForwarder(
      adbPath: 'adb',
      deviceId: '1',
      processManager: processManager,
      logger: BufferLogger.test(),
    );

    await forwarder.unforward(ForwardedPort(456, 23));
  });

  testWithoutContext('failures to unforward port print error but are non-fatal', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['adb', '-s', '1', 'forward', '--remove', 'tcp:456'],
        stderr: 'error: everything is broken!',
        exitCode: 1,
      ),
    ]);
    final BufferLogger logger = BufferLogger.test();
    final AndroidDevicePortForwarder forwarder = AndroidDevicePortForwarder(
      adbPath: 'adb',
      deviceId: '1',
      processManager: processManager,
      logger: logger,
    );

    await forwarder.unforward(ForwardedPort(456, 23));

    expect(logger.errorText, contains('Failed to unforward port: error: everything is broken!'));
  });
}
