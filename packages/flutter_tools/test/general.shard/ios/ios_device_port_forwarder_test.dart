// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/ios/devices.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

const Map<String, String> kDyLdLibEntry = <String, String>{'DYLD_LIBRARY_PATH': '/path/to/libs'};

void main() {
  // By default, the .forward() method will try every port between 1024
  // and 65535; this test verifies we are killing iproxy processes when
  // we timeout on a port
  testWithoutContext(
    'IOSDevicePortForwarder.forward will kill iproxy processes before invoking a second',
    () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        // iproxy does not exit with 0 when it cannot forward;
        // the FakeCommands below expect an exitCode of 0.
        const FakeCommand(
          command: <String>['iproxy', '12345:456', '--udid', '1234'],
          environment: kDyLdLibEntry,
          // Empty stdout indicates failure.
        ),
        const FakeCommand(
          command: <String>['iproxy', '12346:456', '--udid', '1234'],
          stdout: 'not empty',
          environment: kDyLdLibEntry,
        ),
      ]);
      final FakeOperatingSystemUtils operatingSystemUtils = FakeOperatingSystemUtils();

      final IOSDevicePortForwarder portForwarder = IOSDevicePortForwarder.test(
        processManager: processManager,
        logger: BufferLogger.test(),
        operatingSystemUtils: operatingSystemUtils,
      );
      final int hostPort = await portForwarder.forward(456);

      // First port tried (12345) should fail, then succeed on the next
      expect(hostPort, 12345 + 1);
      expect(processManager, hasNoRemainingExpectations);
    },
  );
}
