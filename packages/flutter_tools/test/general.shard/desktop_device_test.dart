// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:fake_async/fake_async.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/desktop_device.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/macos/macos_device.dart';
import 'package:flutter_tools/src/project.dart';

import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/context.dart';

void main() {
  group('Basic info', () {
    testWithoutContext('Category is desktop', () async {
      final FakeDesktopDevice device = setUpDesktopDevice();

      expect(device.category, Category.desktop);
    });

    testWithoutContext('Not an emulator', () async {
      final FakeDesktopDevice device = setUpDesktopDevice();

      expect(await device.isLocalEmulator, false);
      expect(await device.emulatorId, null);
    });

    testWithoutContext('Uses OS name as SDK name', () async {
      final FakeDesktopDevice device = setUpDesktopDevice();

      expect(await device.sdkNameAndVersion, 'Example');
    });
  });

  group('Install', () {
    testWithoutContext('Install checks always return true', () async {
      final FakeDesktopDevice device = setUpDesktopDevice();

      expect(await device.isAppInstalled(FakeApplicationPackage()), true);
      expect(await device.isLatestBuildInstalled(FakeApplicationPackage()), true);
      expect(device.category, Category.desktop);
    });

    testWithoutContext('Install and uninstall are no-ops that report success', () async {
      final FakeDesktopDevice device = setUpDesktopDevice();
      final FakeApplicationPackage package = FakeApplicationPackage();

      expect(await device.uninstallApp(package), true);
      expect(await device.isAppInstalled(package), true);
      expect(await device.isLatestBuildInstalled(package), true);

      expect(await device.installApp(package), true);
      expect(await device.isAppInstalled(package), true);
      expect(await device.isLatestBuildInstalled(package), true);
      expect(device.category, Category.desktop);
    });
  });

  group('Starting and stopping application', () {
    testWithoutContext('Stop without start is a successful no-op', () async {
      final FakeDesktopDevice device = setUpDesktopDevice();
      final FakeApplicationPackage package = FakeApplicationPackage();

      expect(await device.stopApp(package), true);
    });

    testWithoutContext('Can run from prebuilt application', () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Completer<void> completer = Completer<void>();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: const <String>['debug'],
          stdout: 'The Dart VM service is listening on http://127.0.0.1/0\n',
          completer: completer,
        ),
      ]);
      final FakeDesktopDevice device = setUpDesktopDevice(
        processManager: processManager,
        fileSystem: fileSystem,
      );
      final String? executableName = device.executablePathForDevice(
        FakeApplicationPackage(),
        BuildInfo.debug,
      );
      fileSystem.file(executableName).writeAsStringSync('\n');
      final FakeApplicationPackage package = FakeApplicationPackage();
      final LaunchResult result = await device.startApp(
        package,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
      );

      expect(result.started, true);
      expect(result.vmServiceUri, Uri.parse('http://127.0.0.1/0'));
    });

    testWithoutContext('Null executable path fails gracefully', () async {
      final BufferLogger logger = BufferLogger.test();
      final DesktopDevice device = setUpDesktopDevice(
        nullExecutablePathForDevice: true,
        logger: logger,
      );
      final FakeApplicationPackage package = FakeApplicationPackage();
      final LaunchResult result = await device.startApp(
        package,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
      );

      expect(result.started, false);
      expect(logger.errorText, contains('Unable to find executable to run'));
    });

    testWithoutContext('stopApp kills process started by startApp', () async {
      final Completer<void> completer = Completer<void>();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: const <String>['debug'],
          stdout: 'The Dart VM service is listening on http://127.0.0.1/0\n',
          completer: completer,
        ),
      ]);
      final FakeDesktopDevice device = setUpDesktopDevice(processManager: processManager);
      final FakeApplicationPackage package = FakeApplicationPackage();
      final LaunchResult result = await device.startApp(
        package,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
      );

      expect(result.started, true);
      expect(await device.stopApp(package), true);
    });
  });

  testWithoutContext(
    'startApp supports DebuggingOptions through FLUTTER_ENGINE_SWITCH environment variables',
    () async {
      final Completer<void> completer = Completer<void>();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: const <String>['debug'],
          stdout: 'The Dart VM service is listening on http://127.0.0.1/0\n',
          completer: completer,
          environment: const <String, String>{
            'FLUTTER_ENGINE_SWITCH_1': 'enable-dart-profiling=true',
            'FLUTTER_ENGINE_SWITCH_2': 'trace-startup=true',
            'FLUTTER_ENGINE_SWITCH_3': 'enable-software-rendering=true',
            'FLUTTER_ENGINE_SWITCH_4': 'skia-deterministic-rendering=true',
            'FLUTTER_ENGINE_SWITCH_5': 'trace-skia=true',
            'FLUTTER_ENGINE_SWITCH_6': 'trace-allowlist=foo,bar',
            'FLUTTER_ENGINE_SWITCH_7': 'trace-skia-allowlist=skia.a,skia.b',
            'FLUTTER_ENGINE_SWITCH_8': 'trace-systrace=true',
            'FLUTTER_ENGINE_SWITCH_9': 'trace-to-file=path/to/trace.binpb',
            'FLUTTER_ENGINE_SWITCH_10': 'endless-trace-buffer=true',
            'FLUTTER_ENGINE_SWITCH_11': 'dump-skp-on-shader-compilation=true',
            'FLUTTER_ENGINE_SWITCH_12': 'cache-sksl=true',
            'FLUTTER_ENGINE_SWITCH_13': 'purge-persistent-cache=true',
            'FLUTTER_ENGINE_SWITCH_14': 'enable-impeller=false',
            'FLUTTER_ENGINE_SWITCH_15': 'enable-checked-mode=true',
            'FLUTTER_ENGINE_SWITCH_16': 'verify-entry-points=true',
            'FLUTTER_ENGINE_SWITCH_17': 'start-paused=true',
            'FLUTTER_ENGINE_SWITCH_18': 'disable-service-auth-codes=true',
            'FLUTTER_ENGINE_SWITCH_19': 'dart-flags=--null_assertions',
            'FLUTTER_ENGINE_SWITCH_20': 'use-test-fonts=true',
            'FLUTTER_ENGINE_SWITCH_21': 'verbose-logging=true',
            'FLUTTER_ENGINE_SWITCHES': '21',
          },
        ),
      ]);
      final FakeDesktopDevice device = setUpDesktopDevice(processManager: processManager);
      final FakeApplicationPackage package = FakeApplicationPackage();
      final LaunchResult result = await device.startApp(
        package,
        prebuiltApplication: true,
        platformArgs: <String, Object>{'trace-startup': true},
        debuggingOptions: DebuggingOptions.enabled(
          BuildInfo.debug,
          startPaused: true,
          disableServiceAuthCodes: true,
          enableSoftwareRendering: true,
          skiaDeterministicRendering: true,
          traceSkia: true,
          traceAllowlist: 'foo,bar',
          traceSkiaAllowlist: 'skia.a,skia.b',
          traceSystrace: true,
          traceToFile: 'path/to/trace.binpb',
          endlessTraceBuffer: true,
          dumpSkpOnShaderCompilation: true,
          cacheSkSL: true,
          purgePersistentCache: true,
          useTestFonts: true,
          verboseSystemLogs: true,
          nullAssertions: true,
        ),
      );

      expect(result.started, true);
    },
  );

  testWithoutContext(
    'startApp supports DebuggingOptions through FLUTTER_ENGINE_SWITCH environment variables when debugging is disabled',
    () async {
      final Completer<void> completer = Completer<void>();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: const <String>['debug'],
          stdout: 'The Dart VM service is listening on http://127.0.0.1/0\n',
          completer: completer,
          environment: const <String, String>{
            'FLUTTER_ENGINE_SWITCH_1': 'enable-dart-profiling=true',
            'FLUTTER_ENGINE_SWITCH_2': 'trace-startup=true',
            'FLUTTER_ENGINE_SWITCH_3': 'trace-allowlist=foo,bar',
            'FLUTTER_ENGINE_SWITCH_4': 'cache-sksl=true',
            'FLUTTER_ENGINE_SWITCH_5': 'enable-impeller=false',
            'FLUTTER_ENGINE_SWITCHES': '5',
          },
        ),
      ]);
      final FakeDesktopDevice device = setUpDesktopDevice(processManager: processManager);
      final FakeApplicationPackage package = FakeApplicationPackage();
      final LaunchResult result = await device.startApp(
        package,
        prebuiltApplication: true,
        platformArgs: <String, Object>{'trace-startup': true},
        debuggingOptions: DebuggingOptions.disabled(
          BuildInfo.debug,
          traceAllowlist: 'foo,bar',
          cacheSkSL: true,
        ),
      );

      expect(result.started, true);
    },
  );

  testWithoutContext('Port forwarder is a no-op', () async {
    final FakeDesktopDevice device = setUpDesktopDevice();
    final DevicePortForwarder portForwarder = device.portForwarder;
    final int result = await portForwarder.forward(2);

    expect(result, 2);
    expect(portForwarder.forwardedPorts.isEmpty, true);
  });

  testWithoutContext('createDevFSWriter returns a LocalDevFSWriter', () {
    final FakeDesktopDevice device = setUpDesktopDevice();

    expect(device.createDevFSWriter(FakeApplicationPackage(), ''), isA<LocalDevFSWriter>());
  });

  testWithoutContext('startApp supports dartEntrypointArgs', () async {
    final Completer<void> completer = Completer<void>();
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: const <String>['debug', 'arg1', 'arg2'],
        stdout: 'The Dart VM service is listening on http://127.0.0.1/0\n',
        completer: completer,
      ),
    ]);
    final FakeDesktopDevice device = setUpDesktopDevice(processManager: processManager);
    final FakeApplicationPackage package = FakeApplicationPackage();
    final LaunchResult result = await device.startApp(
      package,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.enabled(
        BuildInfo.debug,
        dartEntrypointArgs: <String>['arg1', 'arg2'],
      ),
    );

    expect(result.started, true);
  });

  testWithoutContext('Device logger captures all output', () async {
    final Completer<void> exitCompleter = Completer<void>();
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: const <String>['debug', 'arg1', 'arg2'],
        exitCode: -1,
        stderr: 'Oops\n',
        completer: exitCompleter,
        outputFollowsExit: true,
      ),
    ]);
    final FakeDesktopDevice device = setUpDesktopDevice(processManager: processManager);
    unawaited(
      Future<void>(() {
        exitCompleter.complete();
      }),
    );

    // Start looking for 'Oops' in the stream before starting the app.
    expect(device.getLogReader().logLines, emits('Oops'));

    final FakeApplicationPackage package = FakeApplicationPackage();
    await device.startApp(
      package,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.enabled(
        BuildInfo.debug,
        dartEntrypointArgs: <String>['arg1', 'arg2'],
      ),
    );
  });

  testWithoutContext('Desktop devices pass through the enable-impeller flag', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['debug'],
        exitCode: -1,
        environment: <String, String>{
          'FLUTTER_ENGINE_SWITCH_1': 'enable-dart-profiling=true',
          'FLUTTER_ENGINE_SWITCH_2': 'enable-impeller=true',
          'FLUTTER_ENGINE_SWITCH_3': 'enable-checked-mode=true',
          'FLUTTER_ENGINE_SWITCH_4': 'verify-entry-points=true',
          'FLUTTER_ENGINE_SWITCHES': '4',
        },
      ),
    ]);
    final FakeDesktopDevice device = setUpDesktopDevice(processManager: processManager);

    final FakeApplicationPackage package = FakeApplicationPackage();
    await device.startApp(
      package,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.enabled(
        BuildInfo.debug,
        enableImpeller: ImpellerStatus.enabled,
        dartEntrypointArgs: <String>[],
      ),
    );
  });

  testWithoutContext('Desktop devices pass through the --no-enable-impeller flag', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['debug'],
        exitCode: -1,
        environment: <String, String>{
          'FLUTTER_ENGINE_SWITCH_1': 'enable-dart-profiling=true',
          'FLUTTER_ENGINE_SWITCH_2': 'enable-impeller=false',
          'FLUTTER_ENGINE_SWITCH_3': 'enable-checked-mode=true',
          'FLUTTER_ENGINE_SWITCH_4': 'verify-entry-points=true',
          'FLUTTER_ENGINE_SWITCHES': '4',
        },
      ),
    ]);
    final FakeDesktopDevice device = setUpDesktopDevice(processManager: processManager);

    final FakeApplicationPackage package = FakeApplicationPackage();
    await device.startApp(
      package,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.enabled(
        BuildInfo.debug,
        enableImpeller: ImpellerStatus.disabled,
        dartEntrypointArgs: <String>[],
      ),
    );
  });

  testUsingContext(
    'macOS devices print warning if Dart VM not found within timeframe in CI',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FakeMacOSDevice device = FakeMacOSDevice(
        fileSystem: MemoryFileSystem.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
        logger: logger,
      );

      final FakeApplicationPackage package = FakeApplicationPackage();

      FakeAsync().run((FakeAsync fakeAsync) {
        device.startApp(
          package,
          prebuiltApplication: true,
          debuggingOptions: DebuggingOptions.enabled(
            BuildInfo.debug,
            enableImpeller: ImpellerStatus.disabled,
            dartEntrypointArgs: <String>[],
            usingCISystem: true,
          ),
        );
        fakeAsync.flushTimers();
        expect(
          logger.errorText,
          contains('Ensure sandboxing is disabled by checking the set CODE_SIGN_ENTITLEMENTS'),
        );
      });
    },
  );
}

FakeDesktopDevice setUpDesktopDevice({
  FileSystem? fileSystem,
  Logger? logger,
  ProcessManager? processManager,
  OperatingSystemUtils? operatingSystemUtils,
  bool nullExecutablePathForDevice = false,
}) {
  return FakeDesktopDevice(
    fileSystem: fileSystem ?? MemoryFileSystem.test(),
    logger: logger ?? BufferLogger.test(),
    processManager: processManager ?? FakeProcessManager.any(),
    operatingSystemUtils: operatingSystemUtils ?? FakeOperatingSystemUtils(),
    nullExecutablePathForDevice: nullExecutablePathForDevice,
  );
}

/// A trivial subclass of DesktopDevice for testing the shared functionality.
class FakeDesktopDevice extends DesktopDevice {
  FakeDesktopDevice({
    required ProcessManager processManager,
    required Logger logger,
    required FileSystem fileSystem,
    required OperatingSystemUtils operatingSystemUtils,
    this.nullExecutablePathForDevice = false,
  }) : super(
         'dummy',
         platformType: PlatformType.linux,
         ephemeral: false,
         processManager: processManager,
         logger: logger,
         fileSystem: fileSystem,
         operatingSystemUtils: operatingSystemUtils,
       );

  /// The [mainPath] last passed to [buildForDevice].
  String? lastBuiltMainPath;

  /// The [buildInfo] last passed to [buildForDevice].
  BuildInfo? lastBuildInfo;

  final bool nullExecutablePathForDevice;

  @override
  String get name => 'dummy';

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.tester;

  @override
  bool isSupported() => true;

  @override
  bool isSupportedForProject(FlutterProject flutterProject) => true;

  @override
  Future<void> buildForDevice({
    String? mainPath,
    BuildInfo? buildInfo,
    bool usingCISystem = false,
  }) async {
    lastBuiltMainPath = mainPath;
    lastBuildInfo = buildInfo;
  }

  // Dummy implementation that just returns the build mode name.
  @override
  String? executablePathForDevice(ApplicationPackage package, BuildInfo buildInfo) {
    if (nullExecutablePathForDevice) {
      return null;
    }
    return buildInfo.mode.cliName;
  }
}

class FakeApplicationPackage extends Fake implements ApplicationPackage {}

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  @override
  String get name => 'Example';
}

class FakeMacOSDevice extends MacOSDevice {
  FakeMacOSDevice({
    required super.processManager,
    required super.logger,
    required super.fileSystem,
    required super.operatingSystemUtils,
  });

  @override
  String get name => 'dummy';

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.tester;

  @override
  bool isSupported() => true;

  @override
  bool isSupportedForProject(FlutterProject flutterProject) => true;

  @override
  Future<void> buildForDevice({
    String? mainPath,
    BuildInfo? buildInfo,
    bool usingCISystem = false,
  }) async {}

  // Dummy implementation that just returns the build mode name.
  @override
  String? executablePathForDevice(ApplicationPackage package, BuildInfo buildInfo) {
    return buildInfo.mode.cliName;
  }
}
