// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/context.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/custom_devices.dart';
import 'package:flutter_tools/src/custom_devices/custom_device_config.dart';
import 'package:flutter_tools/src/custom_devices/custom_devices_config.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/runner/flutter_command_runner.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';

const String linuxFlutterRoot = '/flutter';
const String windowsFlutterRoot = r'C:\flutter';

const String defaultConfigLinux1 = r'''
{
  "$schema": "file:///flutter/packages/flutter_tools/static/custom-devices.schema.json",
  "custom-devices": [
    {
      "id": "pi",
      "label": "Raspberry Pi",
      "sdkNameAndVersion": "Raspberry Pi 4 Model B+",
      "platform": "linux-arm64",
      "enabled": false,
      "ping": [
        "ping",
        "-w",
        "1",
        "-c",
        "1",
        "raspberrypi"
      ],
      "pingSuccessRegex": null,
      "postBuild": null,
      "install": [
        "scp",
        "-r",
        "-o",
        "BatchMode=yes",
        "${localPath}",
        "pi@raspberrypi:/tmp/${appName}"
      ],
      "uninstall": [
        "ssh",
        "-o",
        "BatchMode=yes",
        "pi@raspberrypi",
        "rm -rf \"/tmp/${appName}\""
      ],
      "runDebug": [
        "ssh",
        "-o",
        "BatchMode=yes",
        "pi@raspberrypi",
        "flutter-pi \"/tmp/${appName}\""
      ],
      "forwardPort": [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ExitOnForwardFailure=yes",
        "-L",
        "127.0.0.1:${hostPort}:127.0.0.1:${devicePort}",
        "pi@raspberrypi",
        "echo 'Port forwarding success'; read"
      ],
      "forwardPortSuccessRegex": "Port forwarding success",
      "screenshot": [
        "ssh",
        "-o",
        "BatchMode=yes",
        "pi@raspberrypi",
        "fbgrab /tmp/screenshot.png && cat /tmp/screenshot.png | base64 | tr -d ' \\n\\t'"
      ]
    }
  ]
}
''';
const String defaultConfigLinux2 = r'''
{
  "custom-devices": [
    {
      "id": "pi",
      "label": "Raspberry Pi",
      "sdkNameAndVersion": "Raspberry Pi 4 Model B+",
      "platform": "linux-arm64",
      "enabled": false,
      "ping": [
        "ping",
        "-w",
        "1",
        "-c",
        "1",
        "raspberrypi"
      ],
      "pingSuccessRegex": null,
      "postBuild": null,
      "install": [
        "scp",
        "-r",
        "-o",
        "BatchMode=yes",
        "${localPath}",
        "pi@raspberrypi:/tmp/${appName}"
      ],
      "uninstall": [
        "ssh",
        "-o",
        "BatchMode=yes",
        "pi@raspberrypi",
        "rm -rf \"/tmp/${appName}\""
      ],
      "runDebug": [
        "ssh",
        "-o",
        "BatchMode=yes",
        "pi@raspberrypi",
        "flutter-pi \"/tmp/${appName}\""
      ],
      "forwardPort": [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ExitOnForwardFailure=yes",
        "-L",
        "127.0.0.1:${hostPort}:127.0.0.1:${devicePort}",
        "pi@raspberrypi",
        "echo 'Port forwarding success'; read"
      ],
      "forwardPortSuccessRegex": "Port forwarding success",
      "screenshot": [
        "ssh",
        "-o",
        "BatchMode=yes",
        "pi@raspberrypi",
        "fbgrab /tmp/screenshot.png && cat /tmp/screenshot.png | base64 | tr -d ' \\n\\t'"
      ]
    }
  ],
  "$schema": "file:///flutter/packages/flutter_tools/static/custom-devices.schema.json"
}
''';

final Platform windowsPlatform = FakePlatform(
  operatingSystem: 'windows',
  environment: <String, String>{'FLUTTER_ROOT': windowsFlutterRoot},
);

class FakeTerminal implements Terminal {
  factory FakeTerminal({required Platform platform}) {
    return FakeTerminal._private(stdio: FakeStdio(), platform: platform);
  }

  FakeTerminal._private({required this.stdio, required Platform platform})
    : terminal = AnsiTerminal(stdio: stdio, platform: platform);

  final FakeStdio stdio;
  final AnsiTerminal terminal;

  void simulateStdin(String line) {
    stdio.simulateStdin(line);
  }

  @override
  set usesTerminalUi(bool value) => terminal.usesTerminalUi = value;

  @override
  bool get usesTerminalUi => terminal.usesTerminalUi;

  @override
  String bolden(String message) => terminal.bolden(message);

  @override
  String clearScreen() => terminal.clearScreen();

  @override
  String color(String message, TerminalColor color) => terminal.color(message, color);

  @override
  Stream<String> get keystrokes => terminal.keystrokes;

  @override
  Future<String> promptForCharInput(
    List<String> acceptedCharacters, {
    required Logger logger,
    String? prompt,
    int? defaultChoiceIndex,
    bool displayAcceptedCharacters = true,
  }) => terminal.promptForCharInput(
    acceptedCharacters,
    logger: logger,
    prompt: prompt,
    defaultChoiceIndex: defaultChoiceIndex,
    displayAcceptedCharacters: displayAcceptedCharacters,
  );

  @override
  bool get singleCharMode => terminal.singleCharMode;
  @override
  set singleCharMode(bool value) => terminal.singleCharMode = value;

  @override
  bool get stdinHasTerminal => terminal.stdinHasTerminal;

  @override
  String get successMark => terminal.successMark;

  @override
  bool get supportsColor => terminal.supportsColor;

  @override
  bool get isCliAnimationEnabled => terminal.isCliAnimationEnabled;

  @override
  void applyFeatureFlags(FeatureFlags flags) {
    // ignored
  }

  @override
  bool get supportsEmoji => terminal.supportsEmoji;

  @override
  String get warningMark => terminal.warningMark;

  @override
  int get preferredStyle => terminal.preferredStyle;
}

class FakeCommandRunner extends FlutterCommandRunner {
  FakeCommandRunner({
    required Platform platform,
    required FileSystem fileSystem,
    required Logger logger,
    UserMessages? userMessages,
  }) : _platform = platform,
       _fileSystem = fileSystem,
       _logger = logger,
       _userMessages = userMessages ?? UserMessages();

  final Platform _platform;
  final FileSystem _fileSystem;
  final Logger _logger;
  final UserMessages _userMessages;

  @override
  Future<void> runCommand(ArgResults topLevelResults) async {
    final Logger logger = (topLevelResults['verbose'] as bool) ? VerboseLogger(_logger) : _logger;

    return context.run<void>(
      overrides: <Type, Generator>{Logger: () => logger},
      body: () {
        Cache.flutterRoot ??= Cache.defaultFlutterRoot(
          platform: _platform,
          fileSystem: _fileSystem,
          userMessages: _userMessages,
        );
        // For compatibility with tests that set this to a relative path.
        Cache.flutterRoot = _fileSystem.path.normalize(
          _fileSystem.path.absolute(Cache.flutterRoot!),
        );
        return super.runCommand(topLevelResults);
      },
    );
  }
}

/// May take platform, logger, processManager and fileSystem from context if
/// not explicitly specified.
CustomDevicesCommand createCustomDevicesCommand({
  CustomDevicesConfig Function(FileSystem, Logger)? config,
  Terminal Function(Platform)? terminal,
  Platform? platform,
  FileSystem? fileSystem,
  ProcessManager? processManager,
  Logger? logger,
  PrintFn? usagePrintFn,
  bool featureEnabled = false,
}) {
  platform ??= FakePlatform();
  processManager ??= FakeProcessManager.any();
  fileSystem ??= MemoryFileSystem.test();
  usagePrintFn ??= print;
  logger ??= BufferLogger.test();

  return CustomDevicesCommand.test(
    customDevicesConfig:
        config != null
            ? config(fileSystem, logger)
            : CustomDevicesConfig.test(
              platform: platform,
              fileSystem: fileSystem,
              directory: fileSystem.directory('/'),
              logger: logger,
            ),
    operatingSystemUtils: FakeOperatingSystemUtils(
      hostPlatform:
          platform.isLinux
              ? HostPlatform.linux_x64
              : platform.isWindows
              ? HostPlatform.windows_x64
              : platform.isMacOS
              ? HostPlatform.darwin_x64
              : throw UnsupportedError('Unsupported operating system'),
    ),
    terminal: terminal != null ? terminal(platform) : FakeTerminal(platform: platform),
    platform: platform,
    featureFlags: TestFeatureFlags(areCustomDevicesEnabled: featureEnabled),
    processManager: processManager,
    fileSystem: fileSystem,
    logger: logger,
    usagePrintFn: usagePrintFn,
  );
}

/// May take platform, logger, processManager and fileSystem from context if
/// not explicitly specified.
CommandRunner<void> createCustomDevicesCommandRunner({
  CustomDevicesConfig Function(FileSystem, Logger)? config,
  Terminal Function(Platform)? terminal,
  Platform? platform,
  FileSystem? fileSystem,
  ProcessManager? processManager,
  Logger? logger,
  PrintFn? usagePrintFn,
  bool featureEnabled = false,
}) {
  platform ??= FakePlatform();
  fileSystem ??= MemoryFileSystem.test();
  logger ??= BufferLogger.test();

  return FakeCommandRunner(platform: platform, fileSystem: fileSystem, logger: logger)..addCommand(
    createCustomDevicesCommand(
      config: config,
      terminal: terminal,
      platform: platform,
      fileSystem: fileSystem,
      processManager: processManager,
      logger: logger,
      usagePrintFn: usagePrintFn,
      featureEnabled: featureEnabled,
    ),
  );
}

FakeTerminal createFakeTerminalForAddingSshDevice({
  required Platform platform,
  required String id,
  required String label,
  required String sdkNameAndVersion,
  required String enabled,
  required String hostname,
  required String username,
  required String runDebug,
  required String usePortForwarding,
  required String screenshot,
  required String apply,
}) {
  return FakeTerminal(platform: platform)
    ..simulateStdin(id)
    ..simulateStdin(label)
    ..simulateStdin(sdkNameAndVersion)
    ..simulateStdin(enabled)
    ..simulateStdin(hostname)
    ..simulateStdin(username)
    ..simulateStdin(runDebug)
    ..simulateStdin(usePortForwarding)
    ..simulateStdin(screenshot)
    ..simulateStdin(apply);
}

void main() {
  const String featureNotEnabledMessage =
      'Custom devices feature must be enabled. Enable using `flutter config --enable-custom-devices`.';

  setUpAll(() {
    Cache.disableLocking();
  });

  group('linux', () {
    setUp(() {
      Cache.flutterRoot = linuxFlutterRoot;
    });

    testUsingContext(
      'custom-devices command shows config file in help when feature is enabled',
      () async {
        final BufferLogger logger = BufferLogger.test();

        final CommandRunner<void> runner = createCustomDevicesCommandRunner(
          logger: logger,
          usagePrintFn: (Object o) => logger.printStatus(o.toString()),
          featureEnabled: true,
        );
        await expectLater(runner.run(const <String>['custom-devices', '--help']), completes);
        expect(
          logger.statusText,
          contains('Makes changes to the config file at "/.flutter_custom_devices.json".'),
        );
      },
    );

    testUsingContext('running custom-devices command without arguments prints usage', () async {
      final BufferLogger logger = BufferLogger.test();

      final CommandRunner<void> runner = createCustomDevicesCommandRunner(
        logger: logger,
        usagePrintFn: (Object o) => logger.printStatus(o.toString()),
        featureEnabled: true,
      );

      await expectLater(runner.run(const <String>['custom-devices']), completes);
      expect(
        logger.statusText,
        contains('Makes changes to the config file at "/.flutter_custom_devices.json".'),
      );
    });

    // test behaviour with disabled feature
    testUsingContext('custom-devices add command fails when feature is not enabled', () async {
      final CommandRunner<void> runner = createCustomDevicesCommandRunner();
      expect(
        runner.run(const <String>['custom-devices', 'add']),
        throwsToolExit(message: featureNotEnabledMessage),
      );
    });

    testUsingContext('custom-devices delete command fails when feature is not enabled', () async {
      final CommandRunner<void> runner = createCustomDevicesCommandRunner();
      expect(
        runner.run(const <String>['custom-devices', 'delete', '-d', 'testid']),
        throwsToolExit(message: featureNotEnabledMessage),
      );
    });

    testUsingContext('custom-devices list command fails when feature is not enabled', () async {
      final CommandRunner<void> runner = createCustomDevicesCommandRunner();
      expect(
        runner.run(const <String>['custom-devices', 'list']),
        throwsToolExit(message: featureNotEnabledMessage),
      );
    });

    testUsingContext('custom-devices reset command fails when feature is not enabled', () async {
      final CommandRunner<void> runner = createCustomDevicesCommandRunner();
      expect(
        runner.run(const <String>['custom-devices', 'reset']),
        throwsToolExit(message: featureNotEnabledMessage),
      );
    });

    // test add command
    testUsingContext(
      'custom-devices add command correctly adds ssh device config on linux',
      () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();

        final CommandRunner<void> runner = createCustomDevicesCommandRunner(
          terminal:
              (Platform platform) => createFakeTerminalForAddingSshDevice(
                platform: platform,
                id: 'testid',
                label: 'testlabel',
                sdkNameAndVersion: 'testsdknameandversion',
                enabled: 'y',
                hostname: 'testhostname',
                username: 'testuser',
                runDebug: 'testrundebug',
                usePortForwarding: 'y',
                screenshot: 'testscreenshot',
                apply: 'y',
              ),
          fileSystem: fs,
          processManager: FakeProcessManager.any(),
          featureEnabled: true,
        );

        await expectLater(
          runner.run(const <String>['custom-devices', 'add', '--no-check']),
          completes,
        );

        final CustomDevicesConfig config = CustomDevicesConfig.test(
          fileSystem: fs,
          directory: fs.directory('/'),
          logger: BufferLogger.test(),
        );

        expect(
          config.devices,
          contains(
            CustomDeviceConfig(
              id: 'testid',
              label: 'testlabel',
              sdkNameAndVersion: 'testsdknameandversion',
              enabled: true,
              pingCommand: const <String>['ping', '-c', '1', '-w', '1', 'testhostname'],
              postBuildCommand: null,
              installCommand: const <String>[
                'scp',
                '-r',
                '-o',
                'BatchMode=yes',
                r'${localPath}',
                r'testuser@testhostname:/tmp/${appName}',
              ],
              uninstallCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                r'rm -rf "/tmp/${appName}"',
              ],
              runDebugCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                'testrundebug',
              ],
              forwardPortCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                '-o',
                'ExitOnForwardFailure=yes',
                '-L',
                r'127.0.0.1:${hostPort}:127.0.0.1:${devicePort}',
                'testuser@testhostname',
                "echo 'Port forwarding success'; read",
              ],
              forwardPortSuccessRegex: RegExp('Port forwarding success'),
              screenshotCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                'testscreenshot',
              ],
            ),
          ),
        );
      },
    );

    testUsingContext('custom-devices add command correctly adds ipv4 ssh device config', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();

      final CommandRunner<void> runner = createCustomDevicesCommandRunner(
        terminal:
            (Platform platform) => createFakeTerminalForAddingSshDevice(
              platform: platform,
              id: 'testid',
              label: 'testlabel',
              sdkNameAndVersion: 'testsdknameandversion',
              enabled: 'y',
              hostname: '192.168.178.1',
              username: 'testuser',
              runDebug: 'testrundebug',
              usePortForwarding: 'y',
              screenshot: 'testscreenshot',
              apply: 'y',
            ),
        processManager: FakeProcessManager.any(),
        fileSystem: fs,
        featureEnabled: true,
      );

      await expectLater(
        runner.run(const <String>['custom-devices', 'add', '--no-check']),
        completes,
      );

      final CustomDevicesConfig config = CustomDevicesConfig.test(
        fileSystem: fs,
        directory: fs.directory('/'),
        logger: BufferLogger.test(),
      );

      expect(
        config.devices,
        contains(
          CustomDeviceConfig(
            id: 'testid',
            label: 'testlabel',
            sdkNameAndVersion: 'testsdknameandversion',
            enabled: true,
            pingCommand: const <String>['ping', '-c', '1', '-w', '1', '192.168.178.1'],
            postBuildCommand: null,
            installCommand: const <String>[
              'scp',
              '-r',
              '-o',
              'BatchMode=yes',
              r'${localPath}',
              r'testuser@192.168.178.1:/tmp/${appName}',
            ],
            uninstallCommand: const <String>[
              'ssh',
              '-o',
              'BatchMode=yes',
              'testuser@192.168.178.1',
              r'rm -rf "/tmp/${appName}"',
            ],
            runDebugCommand: const <String>[
              'ssh',
              '-o',
              'BatchMode=yes',
              'testuser@192.168.178.1',
              'testrundebug',
            ],
            forwardPortCommand: const <String>[
              'ssh',
              '-o',
              'BatchMode=yes',
              '-o',
              'ExitOnForwardFailure=yes',
              '-L',
              r'127.0.0.1:${hostPort}:127.0.0.1:${devicePort}',
              'testuser@192.168.178.1',
              "echo 'Port forwarding success'; read",
            ],
            forwardPortSuccessRegex: RegExp('Port forwarding success'),
            screenshotCommand: const <String>[
              'ssh',
              '-o',
              'BatchMode=yes',
              'testuser@192.168.178.1',
              'testscreenshot',
            ],
          ),
        ),
      );
    });

    testUsingContext('custom-devices add command correctly adds ipv6 ssh device config', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();

      final CommandRunner<void> runner = createCustomDevicesCommandRunner(
        terminal:
            (Platform platform) => createFakeTerminalForAddingSshDevice(
              platform: platform,
              id: 'testid',
              label: 'testlabel',
              sdkNameAndVersion: 'testsdknameandversion',
              enabled: 'y',
              hostname: '::1',
              username: 'testuser',
              runDebug: 'testrundebug',
              usePortForwarding: 'y',
              screenshot: 'testscreenshot',
              apply: 'y',
            ),
        fileSystem: fs,
        featureEnabled: true,
      );

      await expectLater(
        runner.run(const <String>['custom-devices', 'add', '--no-check']),
        completes,
      );

      final CustomDevicesConfig config = CustomDevicesConfig.test(
        fileSystem: fs,
        directory: fs.directory('/'),
        logger: BufferLogger.test(),
      );

      expect(
        config.devices,
        contains(
          CustomDeviceConfig(
            id: 'testid',
            label: 'testlabel',
            sdkNameAndVersion: 'testsdknameandversion',
            enabled: true,
            pingCommand: const <String>['ping', '-6', '-c', '1', '-w', '1', '::1'],
            postBuildCommand: null,
            installCommand: const <String>[
              'scp',
              '-r',
              '-o',
              'BatchMode=yes',
              '-6',
              r'${localPath}',
              r'testuser@[::1]:/tmp/${appName}',
            ],
            uninstallCommand: const <String>[
              'ssh',
              '-o',
              'BatchMode=yes',
              '-6',
              'testuser@[::1]',
              r'rm -rf "/tmp/${appName}"',
            ],
            runDebugCommand: const <String>[
              'ssh',
              '-o',
              'BatchMode=yes',
              '-6',
              'testuser@[::1]',
              'testrundebug',
            ],
            forwardPortCommand: const <String>[
              'ssh',
              '-o',
              'BatchMode=yes',
              '-o',
              'ExitOnForwardFailure=yes',
              '-6',
              '-L',
              r'[::1]:${hostPort}:[::1]:${devicePort}',
              'testuser@[::1]',
              "echo 'Port forwarding success'; read",
            ],
            forwardPortSuccessRegex: RegExp('Port forwarding success'),
            screenshotCommand: const <String>[
              'ssh',
              '-o',
              'BatchMode=yes',
              '-6',
              'testuser@[::1]',
              'testscreenshot',
            ],
          ),
        ),
      );
    });

    testUsingContext(
      'custom-devices add command correctly adds non-forwarding ssh device config',
      () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();

        final CommandRunner<void> runner = createCustomDevicesCommandRunner(
          terminal:
              (Platform platform) => createFakeTerminalForAddingSshDevice(
                platform: platform,
                id: 'testid',
                label: 'testlabel',
                sdkNameAndVersion: 'testsdknameandversion',
                enabled: 'y',
                hostname: 'testhostname',
                username: 'testuser',
                runDebug: 'testrundebug',
                usePortForwarding: 'n',
                screenshot: 'testscreenshot',
                apply: 'y',
              ),
          fileSystem: fs,
          featureEnabled: true,
        );

        await expectLater(
          runner.run(const <String>['custom-devices', 'add', '--no-check']),
          completes,
        );

        final CustomDevicesConfig config = CustomDevicesConfig.test(
          fileSystem: fs,
          directory: fs.directory('/'),
          logger: BufferLogger.test(),
        );

        expect(
          config.devices,
          contains(
            const CustomDeviceConfig(
              id: 'testid',
              label: 'testlabel',
              sdkNameAndVersion: 'testsdknameandversion',
              enabled: true,
              pingCommand: <String>['ping', '-c', '1', '-w', '1', 'testhostname'],
              postBuildCommand: null,
              installCommand: <String>[
                'scp',
                '-r',
                '-o',
                'BatchMode=yes',
                r'${localPath}',
                r'testuser@testhostname:/tmp/${appName}',
              ],
              uninstallCommand: <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                r'rm -rf "/tmp/${appName}"',
              ],
              runDebugCommand: <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                'testrundebug',
              ],
              screenshotCommand: <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                'testscreenshot',
              ],
            ),
          ),
        );
      },
    );

    testUsingContext(
      'custom-devices add command correctly adds non-screenshotting ssh device config',
      () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();

        final CommandRunner<void> runner = createCustomDevicesCommandRunner(
          terminal:
              (Platform platform) => createFakeTerminalForAddingSshDevice(
                platform: platform,
                id: 'testid',
                label: 'testlabel',
                sdkNameAndVersion: 'testsdknameandversion',
                enabled: 'y',
                hostname: 'testhostname',
                username: 'testuser',
                runDebug: 'testrundebug',
                usePortForwarding: 'y',
                screenshot: '',
                apply: 'y',
              ),
          fileSystem: fs,
          featureEnabled: true,
        );

        await expectLater(
          runner.run(const <String>['custom-devices', 'add', '--no-check']),
          completes,
        );

        final CustomDevicesConfig config = CustomDevicesConfig.test(
          fileSystem: fs,
          directory: fs.directory('/'),
          logger: BufferLogger.test(),
        );

        expect(
          config.devices,
          contains(
            CustomDeviceConfig(
              id: 'testid',
              label: 'testlabel',
              sdkNameAndVersion: 'testsdknameandversion',
              enabled: true,
              pingCommand: const <String>['ping', '-c', '1', '-w', '1', 'testhostname'],
              postBuildCommand: null,
              installCommand: const <String>[
                'scp',
                '-r',
                '-o',
                'BatchMode=yes',
                r'${localPath}',
                r'testuser@testhostname:/tmp/${appName}',
              ],
              uninstallCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                r'rm -rf "/tmp/${appName}"',
              ],
              runDebugCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                'testrundebug',
              ],
              forwardPortCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                '-o',
                'ExitOnForwardFailure=yes',
                '-L',
                r'127.0.0.1:${hostPort}:127.0.0.1:${devicePort}',
                'testuser@testhostname',
                "echo 'Port forwarding success'; read",
              ],
              forwardPortSuccessRegex: RegExp('Port forwarding success'),
            ),
          ),
        );
      },
    );

    testUsingContext('custom-devices delete command deletes device and creates backup', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();

      final CustomDevicesConfig config = CustomDevicesConfig.test(
        fileSystem: fs,
        directory: fs.directory('/'),
        logger: BufferLogger.test(),
      );

      config.add(CustomDeviceConfig.exampleUnix.copyWith(id: 'testid'));

      final CommandRunner<void> runner = createCustomDevicesCommandRunner(
        config: (_, __) => config,
        fileSystem: fs,
        featureEnabled: true,
      );

      final Uint8List contentsBefore = fs.file('.flutter_custom_devices.json').readAsBytesSync();

      await expectLater(
        runner.run(const <String>['custom-devices', 'delete', '-d', 'testid']),
        completes,
      );
      expect(fs.file('/.flutter_custom_devices.json.bak'), exists);
      expect(config.devices, hasLength(0));

      final Uint8List backupContents =
          fs.file('.flutter_custom_devices.json.bak').readAsBytesSync();
      expect(contentsBefore, equals(backupContents));
    });

    testUsingContext(
      'custom-devices delete command without device argument throws tool exit',
      () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();

        final CustomDevicesConfig config = CustomDevicesConfig.test(
          fileSystem: fs,
          directory: fs.directory('/'),
          logger: BufferLogger.test(),
        );
        config.add(CustomDeviceConfig.exampleUnix.copyWith(id: 'testid2'));
        final Uint8List contentsBefore = fs.file('.flutter_custom_devices.json').readAsBytesSync();

        final CommandRunner<void> runner = createCustomDevicesCommandRunner(featureEnabled: true);
        await expectLater(runner.run(const <String>['custom-devices', 'delete']), throwsToolExit());

        final Uint8List contentsAfter = fs.file('.flutter_custom_devices.json').readAsBytesSync();
        expect(contentsBefore, equals(contentsAfter));
        expect(fs.file('.flutter_custom_devices.json.bak').existsSync(), isFalse);
      },
    );

    testUsingContext(
      'custom-devices delete command throws tool exit with invalid device id',
      () async {
        final CommandRunner<void> runner = createCustomDevicesCommandRunner(featureEnabled: true);
        await expectLater(
          runner.run(const <String>['custom-devices', 'delete', '-d', 'testid']),
          throwsToolExit(
            message:
                'Couldn\'t find device with id "testid" in config at "/.flutter_custom_devices.json"',
          ),
        );
      },
    );

    testUsingContext(
      'custom-devices list command throws tool exit when config contains errors',
      () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final BufferLogger logger = BufferLogger.test();

        fs.file('.flutter_custom_devices.json').writeAsStringSync('{"custom-devices": {}}');

        final CommandRunner<void> runner = createCustomDevicesCommandRunner(
          fileSystem: fs,
          logger: logger,
          featureEnabled: true,
        );

        await expectLater(
          runner.run(const <String>['custom-devices', 'list']),
          throwsToolExit(message: 'Could not list custom devices.'),
        );
        expect(
          logger.errorText,
          contains(
            "Could not load custom devices config. config['custom-devices'] is not a JSON array.",
          ),
        );
      },
    );

    testUsingContext('custom-devices list command prints message when no devices found', () async {
      final BufferLogger logger = BufferLogger.test();

      final CommandRunner<void> runner = createCustomDevicesCommandRunner(
        logger: logger,
        featureEnabled: true,
      );

      await expectLater(runner.run(const <String>['custom-devices', 'list']), completes);
      expect(
        logger.statusText,
        contains('No custom devices found in "/.flutter_custom_devices.json"'),
      );
    });

    testUsingContext('custom-devices list command lists all devices', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final BufferLogger logger = BufferLogger.test();

      CustomDevicesConfig.test(fileSystem: fs, directory: fs.directory('/'), logger: logger)
        ..add(
          CustomDeviceConfig.exampleUnix.copyWith(id: 'testid', label: 'testlabel', enabled: true),
        )
        ..add(
          CustomDeviceConfig.exampleUnix.copyWith(
            id: 'testid2',
            label: 'testlabel2',
            enabled: false,
          ),
        );

      final CommandRunner<void> runner = createCustomDevicesCommandRunner(
        logger: logger,
        fileSystem: fs,
        featureEnabled: true,
      );

      await expectLater(runner.run(const <String>['custom-devices', 'list']), completes);
      expect(
        logger.statusText,
        contains('List of custom devices in "/.flutter_custom_devices.json":'),
      );
      expect(logger.statusText, contains('id: testid, label: testlabel, enabled: true'));
      expect(logger.statusText, contains('id: testid2, label: testlabel2, enabled: false'));
    });

    testUsingContext('custom-devices reset correctly backs up the config file', () async {
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final BufferLogger logger = BufferLogger.test();

      CustomDevicesConfig.test(fileSystem: fs, directory: fs.directory('/'), logger: logger)
        ..add(
          CustomDeviceConfig.exampleUnix.copyWith(id: 'testid', label: 'testlabel', enabled: true),
        )
        ..add(
          CustomDeviceConfig.exampleUnix.copyWith(
            id: 'testid2',
            label: 'testlabel2',
            enabled: false,
          ),
        );

      final Uint8List contentsBefore = fs.file('.flutter_custom_devices.json').readAsBytesSync();

      final CommandRunner<void> runner = createCustomDevicesCommandRunner(
        logger: logger,
        fileSystem: fs,
        featureEnabled: true,
      );
      await expectLater(runner.run(const <String>['custom-devices', 'reset']), completes);
      expect(
        logger.statusText,
        contains(
          'Successfully reset the custom devices config file and created a '
          'backup at "/.flutter_custom_devices.json.bak".',
        ),
      );

      final Uint8List backupContents =
          fs.file('.flutter_custom_devices.json.bak').readAsBytesSync();
      expect(contentsBefore, equals(backupContents));
      expect(
        fs.file('.flutter_custom_devices.json').readAsStringSync(),
        anyOf(equals(defaultConfigLinux1), equals(defaultConfigLinux2)),
      );
    });

    testUsingContext(
      "custom-devices reset outputs correct msg when config file didn't exist",
      () async {
        final MemoryFileSystem fs = MemoryFileSystem.test();
        final BufferLogger logger = BufferLogger.test();

        final CommandRunner<void> runner = createCustomDevicesCommandRunner(
          logger: logger,
          fileSystem: fs,
          featureEnabled: true,
        );
        await expectLater(runner.run(const <String>['custom-devices', 'reset']), completes);
        expect(logger.statusText, contains('Successfully reset the custom devices config file.'));

        expect(fs.file('.flutter_custom_devices.json.bak'), isNot(exists));
        expect(
          fs.file('.flutter_custom_devices.json').readAsStringSync(),
          anyOf(equals(defaultConfigLinux1), equals(defaultConfigLinux2)),
        );
      },
    );
  });

  group('windows', () {
    setUp(() {
      Cache.flutterRoot = windowsFlutterRoot;
    });

    testUsingContext(
      'custom-devices add command correctly adds ssh device config on windows',
      () async {
        final MemoryFileSystem fs = MemoryFileSystem.test(style: FileSystemStyle.windows);

        final CommandRunner<void> runner = createCustomDevicesCommandRunner(
          terminal:
              (Platform platform) => createFakeTerminalForAddingSshDevice(
                platform: platform,
                id: 'testid',
                label: 'testlabel',
                sdkNameAndVersion: 'testsdknameandversion',
                enabled: 'y',
                hostname: 'testhostname',
                username: 'testuser',
                runDebug: 'testrundebug',
                usePortForwarding: 'y',
                screenshot: 'testscreenshot',
                apply: 'y',
              ),
          fileSystem: fs,
          platform: windowsPlatform,
          featureEnabled: true,
        );

        await expectLater(
          runner.run(const <String>['custom-devices', 'add', '--no-check']),
          completes,
        );

        final CustomDevicesConfig config = CustomDevicesConfig.test(
          fileSystem: fs,
          directory: fs.directory('/'),
          logger: BufferLogger.test(),
        );

        expect(
          config.devices,
          contains(
            CustomDeviceConfig(
              id: 'testid',
              label: 'testlabel',
              sdkNameAndVersion: 'testsdknameandversion',
              enabled: true,
              pingCommand: const <String>['ping', '-n', '1', '-w', '500', 'testhostname'],
              pingSuccessRegex: RegExp(r'[<=]\d+ms'),
              postBuildCommand: null,
              installCommand: const <String>[
                'scp',
                '-r',
                '-o',
                'BatchMode=yes',
                r'${localPath}',
                r'testuser@testhostname:/tmp/${appName}',
              ],
              uninstallCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                r'rm -rf "/tmp/${appName}"',
              ],
              runDebugCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                'testrundebug',
              ],
              forwardPortCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                '-o',
                'ExitOnForwardFailure=yes',
                '-L',
                r'127.0.0.1:${hostPort}:127.0.0.1:${devicePort}',
                'testuser@testhostname',
                "echo 'Port forwarding success'; read",
              ],
              forwardPortSuccessRegex: RegExp('Port forwarding success'),
              screenshotCommand: const <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                'testuser@testhostname',
                'testscreenshot',
              ],
            ),
          ),
        );
      },
    );
  });
}
