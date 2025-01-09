// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/web/chrome.dart';
import 'package:flutter_tools/src/web/web_validator.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

void main() {
  late Platform platform;
  late FakeProcessManager fakeProcessManager;
  late ChromiumLauncher chromeLauncher;
  late FileSystem fileSystem;
  late ChromiumValidator webValidator;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    fakeProcessManager = FakeProcessManager.empty();
    platform = FakePlatform(operatingSystem: 'macos', environment: <String, String>{});
    chromeLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: fakeProcessManager,
      operatingSystemUtils: OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        platform: platform,
        processManager: fakeProcessManager,
      ),
      browserFinder: findChromeExecutable,
      logger: BufferLogger.test(),
    );
    webValidator =
        webValidator = ChromeValidator(platform: platform, chromiumLauncher: chromeLauncher);
  });

  testWithoutContext('WebValidator can find executable on macOS', () async {
    final ValidationResult result = await webValidator.validate();

    expect(result.type, ValidationType.success);
  });

  testWithoutContext('WebValidator Can notice missing macOS executable ', () async {
    fakeProcessManager.excludedExecutables.add(kMacOSExecutable);

    final ValidationResult result = await webValidator.validate();

    expect(result.type, ValidationType.missing);
  });

  testWithoutContext(
    'WebValidator does not warn about CHROME_EXECUTABLE unless it cant find chrome ',
    () async {
      fakeProcessManager.excludedExecutables.add(kMacOSExecutable);

      final ValidationResult result = await webValidator.validate();

      expect(result.messages, const <ValidationMessage>[
        ValidationMessage.hint(
          'Cannot find Chrome. Try setting CHROME_EXECUTABLE to a Chrome executable.',
        ),
      ]);
      expect(result.type, ValidationType.missing);
    },
  );
}
