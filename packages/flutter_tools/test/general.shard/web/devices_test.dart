// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/web/chrome.dart';
import 'package:flutter_tools/src/web/web_device.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

void main() {
  testWithoutContext('No web devices listed if feature is disabled', () async {
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(environment: <String, String>{}),
      processManager: FakeProcessManager.any(),
    );

    expect(await webDevices.pollingGetDevices(), isEmpty);
  });

  testWithoutContext(
    'Successive calls of ChromiumDevice.stopApp() do not try to close chrome',
    () async {
      final TestChromiumLauncher launcher = TestChromiumLauncher(
        launcher: () => _OnceClosableChromium(),
      );

      final _FakeChromiumDevice chromiumDevice = _FakeChromiumDevice(
        chromiumLauncher: launcher,
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
      );

      await chromiumDevice.startApp(
        null,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        platformArgs: <String, Object?>{'uri': '/'},
      );

      await chromiumDevice.stopApp(null);
      await chromiumDevice.stopApp(null);
    },
  );

  testWithoutContext('GoogleChromeDevice defaults', () async {
    final TestChromiumLauncher launcher = TestChromiumLauncher();

    final GoogleChromeDevice chromeDevice = GoogleChromeDevice(
      chromiumLauncher: launcher,
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(),
      processManager: FakeProcessManager.any(),
    );

    expect(chromeDevice.name, 'Chrome');
    expect(chromeDevice.id, 'chrome');
    expect(chromeDevice.supportsHotReload, true);
    expect(chromeDevice.supportsHotRestart, true);
    expect(chromeDevice.supportsStartPaused, true);
    expect(chromeDevice.supportsFlutterExit, false);
    expect(chromeDevice.supportsScreenshot, false);
    expect(await chromeDevice.isLocalEmulator, false);
    expect(chromeDevice.getLogReader(), isA<NoOpDeviceLogReader>());
    expect(chromeDevice.getLogReader(), isA<NoOpDeviceLogReader>());
    expect(await chromeDevice.portForwarder!.forward(1), 1);

    expect(chromeDevice.supportsRuntimeMode(BuildMode.debug), true);
    expect(chromeDevice.supportsRuntimeMode(BuildMode.profile), true);
    expect(chromeDevice.supportsRuntimeMode(BuildMode.release), true);
    expect(chromeDevice.supportsRuntimeMode(BuildMode.jitRelease), false);
  });

  testWithoutContext('MicrosoftEdge defaults', () async {
    final TestChromiumLauncher launcher = TestChromiumLauncher();

    final MicrosoftEdgeDevice chromeDevice = MicrosoftEdgeDevice(
      chromiumLauncher: launcher,
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.any(),
    );

    expect(chromeDevice.name, 'Edge');
    expect(chromeDevice.id, 'edge');
    expect(chromeDevice.supportsHotReload, true);
    expect(chromeDevice.supportsHotRestart, true);
    expect(chromeDevice.supportsStartPaused, true);
    expect(chromeDevice.supportsFlutterExit, false);
    expect(chromeDevice.supportsScreenshot, false);
    expect(await chromeDevice.isLocalEmulator, false);
    expect(chromeDevice.getLogReader(), isA<NoOpDeviceLogReader>());
    expect(chromeDevice.getLogReader(), isA<NoOpDeviceLogReader>());
    expect(await chromeDevice.portForwarder!.forward(1), 1);

    expect(chromeDevice.supportsRuntimeMode(BuildMode.debug), true);
    expect(chromeDevice.supportsRuntimeMode(BuildMode.profile), true);
    expect(chromeDevice.supportsRuntimeMode(BuildMode.release), true);
    expect(chromeDevice.supportsRuntimeMode(BuildMode.jitRelease), false);
  });

  testWithoutContext('Server defaults', () async {
    final WebServerDevice device = WebServerDevice(logger: BufferLogger.test());

    expect(device.name, 'Web Server');
    expect(device.id, 'web-server');
    expect(device.supportsHotReload, true);
    expect(device.supportsHotRestart, true);
    expect(device.supportsStartPaused, true);
    expect(device.supportsFlutterExit, false);
    expect(device.supportsScreenshot, false);
    expect(await device.isLocalEmulator, false);
    expect(device.getLogReader(), isA<NoOpDeviceLogReader>());
    expect(device.getLogReader(), isA<NoOpDeviceLogReader>());
    expect(await device.portForwarder!.forward(1), 1);

    expect(device.supportsRuntimeMode(BuildMode.debug), true);
    expect(device.supportsRuntimeMode(BuildMode.profile), true);
    expect(device.supportsRuntimeMode(BuildMode.release), true);
    expect(device.supportsRuntimeMode(BuildMode.jitRelease), false);
  });

  testWithoutContext('ChromiumDevice accepts null package', () async {
    final MemoryFileSystem fs = MemoryFileSystem.test();
    final FakePlatform platform = FakePlatform();
    final FakeProcessManager pm = FakeProcessManager.any();
    final BufferLogger logger = BufferLogger.test();
    final GoogleChromeDevice device = GoogleChromeDevice(
      fileSystem: fs,
      processManager: pm,
      platform: platform,
      chromiumLauncher: ChromiumLauncher(
        fileSystem: fs,
        platform: platform,
        processManager: pm,
        operatingSystemUtils: FakeOperatingSystemUtils(),
        browserFinder: findChromeExecutable,
        logger: logger,
      ),
      logger: logger,
    );
    await expectLater(
      () => device.startApp(
        null,
        debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
        platformArgs: <String, Object?>{'uri': 'localhost:1234'},
      ),
      // The tool exit here is irrelevant, this test simply ensures ChromiumDevice.startApp
      // will accept a null value for a package.
      throwsToolExit(message: 'Failed to launch browser'),
    );
  });

  testWithoutContext('Chrome device is listed when Chrome can be run', () async {
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(environment: <String, String>{}),
      processManager: FakeProcessManager.any(),
    );

    expect(await webDevices.pollingGetDevices(), contains(isA<GoogleChromeDevice>()));
  });

  testWithoutContext('Has well known device ids chrome, edge, and web-server', () async {
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(environment: <String, String>{}),
      processManager: FakeProcessManager.any(),
    );

    expect(webDevices.wellKnownIds, <String>['chrome', 'web-server', 'edge']);
  });

  testWithoutContext('Chrome device is not listed when Chrome cannot be run', () async {
    final FakeProcessManager processManager = FakeProcessManager.empty();
    processManager.excludedExecutables = <String>{kLinuxExecutable};
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(environment: <String, String>{}),
      processManager: processManager,
    );

    expect(await webDevices.pollingGetDevices(), isNot(contains(isA<GoogleChromeDevice>())));
  });

  testWithoutContext('Web Server device is listed if enabled via showWebServerDevice', () async {
    WebServerDevice.showWebServerDevice = true;
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(environment: <String, String>{}),
      processManager: FakeProcessManager.any(),
    );

    expect(await webDevices.pollingGetDevices(), contains(isA<WebServerDevice>()));
  });

  testWithoutContext(
    'Web Server device is not listed if disabled via showWebServerDevice',
    () async {
      WebServerDevice.showWebServerDevice = false;
      final WebDevices webDevices = WebDevices(
        featureFlags: TestFeatureFlags(isWebEnabled: true),
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        platform: FakePlatform(environment: <String, String>{}),
        processManager: FakeProcessManager.any(),
      );

      expect(await webDevices.pollingGetDevices(), isNot(contains(isA<WebServerDevice>())));
    },
  );

  testWithoutContext('Chrome invokes version command on non-Windows platforms', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(command: <String>[kLinuxExecutable, '--version'], stdout: 'ABC'),
    ]);
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(environment: <String, String>{}),
      processManager: processManager,
    );

    final GoogleChromeDevice chromeDevice =
        (await webDevices.pollingGetDevices()).whereType<GoogleChromeDevice>().first;

    expect(chromeDevice.isSupported(), true);
    expect(await chromeDevice.sdkNameAndVersion, 'ABC');

    // Verify caching works correctly.
    expect(await chromeDevice.sdkNameAndVersion, 'ABC');
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('Chrome and Edge version check invokes registry query on windows.', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>[
          'reg',
          'query',
          r'HKEY_CURRENT_USER\Software\Microsoft\Edge\BLBeacon',
          '/v',
          'version',
        ],
        stdout: r'HKEY_CURRENT_USER\Software\Microsoft\Edge\BLBeacon\ version REG_SZ 83.0.478.44 ',
      ),
      const FakeCommand(
        command: <String>[
          'reg',
          'query',
          r'HKEY_CURRENT_USER\Software\Google\Chrome\BLBeacon',
          '/v',
          'version',
        ],
        stdout: r'HKEY_CURRENT_USER\Software\Google\Chrome\BLBeacon\ version REG_SZ 74.0.0 A',
      ),
    ]);
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(operatingSystem: 'windows', environment: <String, String>{}),
      processManager: processManager,
    );

    final GoogleChromeDevice chromeDevice =
        (await webDevices.pollingGetDevices()).whereType<GoogleChromeDevice>().first;

    expect(chromeDevice.isSupported(), true);
    expect(await chromeDevice.sdkNameAndVersion, 'Google Chrome 74.0.0');

    // Verify caching works correctly.
    expect(await chromeDevice.sdkNameAndVersion, 'Google Chrome 74.0.0');
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('Chrome and Edge version check handles missing registry on Windows', () async {
    final FakeProcessManager processManager = FakeProcessManager.empty();
    processManager.excludedExecutables.add('reg');

    final Platform platform = FakePlatform(
      operatingSystem: 'windows',
      environment: <String, String>{},
    );
    final ChromiumLauncher chromeLauncher = ChromiumLauncher(
      fileSystem: MemoryFileSystem.test(),
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: FakeOperatingSystemUtils(),
      browserFinder: findChromeExecutable,
      logger: BufferLogger.test(),
    );
    final MicrosoftEdgeDevice edgeDevice = MicrosoftEdgeDevice(
      chromiumLauncher: chromeLauncher,
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      processManager: processManager,
    );

    expect(edgeDevice.isSupported(), true);
    expect(await edgeDevice.sdkNameAndVersion, '');

    final GoogleChromeDevice chromeDevice = GoogleChromeDevice(
      chromiumLauncher: chromeLauncher,
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      processManager: processManager,
      platform: platform,
    );

    expect(chromeDevice.isSupported(), true);
    expect(await chromeDevice.sdkNameAndVersion, 'unknown');
  });

  testWithoutContext('Edge is not supported on versions less than 73', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>[
          'reg',
          'query',
          r'HKEY_CURRENT_USER\Software\Microsoft\Edge\BLBeacon',
          '/v',
          'version',
        ],
        stdout: r'HKEY_CURRENT_USER\Software\Microsoft\Edge\BLBeacon\ version REG_SZ 72.0.478.44 ',
      ),
    ]);
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(operatingSystem: 'windows', environment: <String, String>{}),
      processManager: processManager,
    );

    expect((await webDevices.pollingGetDevices()).whereType<MicrosoftEdgeDevice>(), isEmpty);
  });

  testWithoutContext('Edge is not support on non-windows platform', () async {
    final WebDevices webDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(environment: <String, String>{}),
      processManager: FakeProcessManager.empty(),
    );

    expect((await webDevices.pollingGetDevices()).whereType<MicrosoftEdgeDevice>(), isEmpty);

    final WebDevices macosWebDevices = WebDevices(
      featureFlags: TestFeatureFlags(isWebEnabled: true),
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: FakePlatform(operatingSystem: 'macos', environment: <String, String>{}),
      processManager: FakeProcessManager.empty(),
    );

    expect((await macosWebDevices.pollingGetDevices()).whereType<MicrosoftEdgeDevice>(), isEmpty);
  });
}

/// A test implementation of the [ChromiumLauncher] that launches a fixed instance.
class TestChromiumLauncher implements ChromiumLauncher {
  TestChromiumLauncher({Chromium Function()? launcher}) {
    _launcher = launcher ?? () => _UnimplementedChromium();
  }

  late Chromium Function() _launcher;

  @override
  Completer<Chromium> currentCompleter = Completer<Chromium>();

  @override
  bool canFindExecutable() {
    return true;
  }

  @override
  Future<Chromium> get connectedInstance => currentCompleter.future;

  @override
  String findExecutable() {
    return 'chrome';
  }

  @override
  bool get hasChromeInstance => false;

  @override
  Future<Chromium> launch(
    String url, {
    bool headless = false,
    int? debugPort,
    bool skipCheck = false,
    Directory? cacheDir,
    List<String> webBrowserFlags = const <String>[],
  }) async {
    currentCompleter.complete(_launcher());
    return currentCompleter.future;
  }

  @override
  Future<Chromium> connect(Chromium chrome, bool skipCheck) {
    return currentCompleter.future;
  }
}

class _FakeChromiumDevice extends ChromiumDevice {
  _FakeChromiumDevice({
    required ChromiumLauncher chromiumLauncher,
    required super.fileSystem,
    required super.logger,
  }) : super(name: 'fake name', chromeLauncher: chromiumLauncher);

  @override
  Future<String> get sdkNameAndVersion async => 'fake sdkNameAndVersion';

  @override
  String get name => 'fake name';
}

class _OnceClosableChromium extends Fake implements Chromium {
  bool _closed = false;

  @override
  Future<void> close() async {
    if (_closed) {
      throw Exception('Already closed');
    }
    _closed = true;
  }
}

class _UnimplementedChromium extends Fake implements Chromium {}
