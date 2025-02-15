// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/drive/web_driver_service.dart';
import 'package:package_config/package_config_types.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';

void main() {
  testWithoutContext(
    'WebDriverService catches SocketExceptions cleanly and includes link to documentation',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final WebDriverService service = WebDriverService(
        logger: logger,
        terminal: Terminal.test(),
        platform: FakePlatform(),
        outputPreferences: OutputPreferences.test(),
        processUtils: ProcessUtils(logger: logger, processManager: FakeProcessManager.empty()),
        dartSdkPath: 'dart',
      );
      const String link = 'https://flutter.dev/to/integration-test-on-web';
      try {
        await service.startTest(
          'foo.test',
          <String>[],
          PackageConfig(<Package>[Package('test', Uri.base)]),
          driverPort: 1,
          headless: true,
          browserName: 'chrome',
        );
        fail('WebDriverService did not throw as expected.');
      } on ToolExit catch (error) {
        expect(error.message, isNot(contains('SocketException')));
        expect(error.message, contains(link));
      }
    },
  );
}
