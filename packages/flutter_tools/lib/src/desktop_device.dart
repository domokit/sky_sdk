// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:process/process.dart';

import 'application_package.dart';
import 'base/file_system.dart';
import 'base/io.dart';
import 'base/logger.dart';
import 'base/os.dart';
import 'build_info.dart';
import 'convert.dart';
import 'devfs.dart';
import 'device.dart';
import 'device_port_forwarder.dart';
import 'globals.dart' as globals;
import 'macos/macos_device.dart';
import 'protocol_discovery.dart';
import 'vmservice.dart';

/// A partial implementation of Device for desktop-class devices to inherit
/// from, containing implementations that are common to all desktop devices.
abstract class DesktopDevice extends Device {
  DesktopDevice(
    super.id, {
    required PlatformType super.platformType,
    required super.ephemeral,
    required super.logger,
    required ProcessManager processManager,
    required FileSystem fileSystem,
    required OperatingSystemUtils operatingSystemUtils,
  }) : _logger = logger,
       _processManager = processManager,
       _fileSystem = fileSystem,
       _operatingSystemUtils = operatingSystemUtils,
       super(category: Category.desktop);

  final Logger _logger;
  final ProcessManager _processManager;
  final FileSystem _fileSystem;
  final OperatingSystemUtils _operatingSystemUtils;
  final Set<Process> _runningProcesses = <Process>{};
  final DesktopLogReader _deviceLogReader = DesktopLogReader();

  @override
  DevFSWriter createDevFSWriter(ApplicationPackage? app, String? userIdentifier) {
    return LocalDevFSWriter(fileSystem: _fileSystem);
  }

  // Since the host and target devices are the same, no work needs to be done
  // to install the application.
  @override
  Future<bool> isAppInstalled(ApplicationPackage app, {String? userIdentifier}) async => true;

  // Since the host and target devices are the same, no work needs to be done
  // to install the application.
  @override
  Future<bool> isLatestBuildInstalled(ApplicationPackage app) async => true;

  // Since the host and target devices are the same, no work needs to be done
  // to install the application.
  @override
  Future<bool> installApp(ApplicationPackage app, {String? userIdentifier}) async => true;

  // Since the host and target devices are the same, no work needs to be done
  // to uninstall the application.
  @override
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier}) async => true;

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  Future<String?> get emulatorId async => null;

  @override
  DevicePortForwarder get portForwarder => const NoOpDevicePortForwarder();

  @override
  Future<String> get sdkNameAndVersion async => _operatingSystemUtils.name;

  @override
  bool supportsRuntimeMode(BuildMode buildMode) => buildMode != BuildMode.jitRelease;

  @override
  DeviceLogReader getLogReader({ApplicationPackage? app, bool includePastLogs = false}) {
    assert(!includePastLogs, 'Past log reading not supported on desktop.');
    return _deviceLogReader;
  }

  @override
  void clearLogs() {}

  @override
  Future<LaunchResult> startApp(
    ApplicationPackage package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, dynamic> platformArgs = const <String, dynamic>{},
    bool prebuiltApplication = false,
    String? userIdentifier,
  }) async {
    if (!prebuiltApplication) {
      await buildForDevice(
        buildInfo: debuggingOptions.buildInfo,
        mainPath: mainPath,
        usingCISystem: debuggingOptions.usingCISystem,
      );
    }

    // Ensure that the executable is locatable.
    final BuildInfo buildInfo = debuggingOptions.buildInfo;
    final bool traceStartup = platformArgs['trace-startup'] as bool? ?? false;
    final String? executable = executablePathForDevice(package, buildInfo);
    if (executable == null) {
      _logger.printError('Unable to find executable to run');
      return LaunchResult.failed();
    }

    Process process;
    final List<String> command = <String>[executable, ...debuggingOptions.dartEntrypointArgs];
    try {
      process = await _processManager.start(
        command,
        environment: _computeEnvironment(debuggingOptions, traceStartup, route),
      );
    } on ProcessException catch (e) {
      _logger.printError('Unable to start executable "${command.join(' ')}": $e');
      rethrow;
    }
    _runningProcesses.add(process);
    unawaited(process.exitCode.then((_) => _runningProcesses.remove(process)));

    _deviceLogReader.initializeProcess(process);
    if (debuggingOptions.buildInfo.isRelease) {
      return LaunchResult.succeeded();
    }
    final ProtocolDiscovery vmServiceDiscovery = ProtocolDiscovery.vmService(
      _deviceLogReader,
      devicePort: debuggingOptions.deviceVmServicePort,
      hostPort: debuggingOptions.hostVmServicePort,
      ipv6: debuggingOptions.ipv6,
      logger: _logger,
    );
    try {
      Timer? timer;
      if (this is MacOSDevice) {
        if (await globals.isRunningOnBot) {
          const int defaultTimeout = 5;
          timer = Timer(const Duration(minutes: defaultTimeout), () {
            // As of macOS 14, if sandboxing is enabled and the app is not codesigned,
            // a dialog will prompt the user to allow the app to run. This will
            // cause tests in CI to hang. In CI, we workaround this by setting
            // the CODE_SIGN_ENTITLEMENTS build setting to a version with
            // sandboxing disabled.
            final String sandboxingMessage;
            if (debuggingOptions.usingCISystem) {
              sandboxingMessage =
                  'Ensure sandboxing is disabled by checking '
                  'the set CODE_SIGN_ENTITLEMENTS.';
            } else {
              sandboxingMessage =
                  'Consider codesigning your app or disabling '
                  'sandboxing. Flutter will attempt to disable sandboxing if '
                  'the `--ci` flag is provided.';
            }
            _logger.printError(
              'The Dart VM Service was not discovered after $defaultTimeout '
              'minutes. If the app has sandboxing enabled and is not '
              'codesigned or codesigning changed, this may be caused by a '
              'system prompt asking for access. $sandboxingMessage\n'
              'See https://developer.apple.com/documentation/security/app_sandbox/accessing_files_from_the_macos_app_sandbox '
              'for more information.',
            );
          });
        }
      }

      final Uri? vmServiceUri = await vmServiceDiscovery.uri;
      if (vmServiceUri != null) {
        timer?.cancel();
        onAttached(package, buildInfo, process);
        return LaunchResult.succeeded(vmServiceUri: vmServiceUri);
      }
      _logger.printError(
        'Error waiting for a debug connection: '
        'The log reader stopped unexpectedly, or never started.',
      );
    } on Exception catch (error) {
      _logger.printError('Error waiting for a debug connection: $error');
    } finally {
      await vmServiceDiscovery.cancel();
    }
    return LaunchResult.failed();
  }

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    bool succeeded = true;
    // Walk a copy of _runningProcesses, since the exit handler removes from the
    // set.
    for (final Process process in Set<Process>.of(_runningProcesses)) {
      succeeded &= _processManager.killPid(process.pid);
    }
    return succeeded;
  }

  @override
  Future<void> dispose() async {
    await portForwarder.dispose();
  }

  /// Builds the current project for this device, with the given options.
  Future<void> buildForDevice({
    required BuildInfo buildInfo,
    String? mainPath,
    bool usingCISystem = false,
  });

  /// Returns the path to the executable to run for [package] on this device for
  /// the given [buildMode].
  String? executablePathForDevice(ApplicationPackage package, BuildInfo buildInfo);

  /// Called after a process is attached, allowing any device-specific extra
  /// steps to be run.
  void onAttached(ApplicationPackage package, BuildInfo buildInfo, Process process) {}

  /// Computes a set of environment variables used to pass debugging information
  /// to the engine without interfering with application level command line
  /// arguments.
  ///
  /// The format of the environment variables is:
  ///   * FLUTTER_ENGINE_SWITCHES to the number of switches.
  ///   * FLUTTER_ENGINE_SWITCH_<N> (indexing from 1) to the individual switches.
  Map<String, String> _computeEnvironment(
    DebuggingOptions debuggingOptions,
    bool traceStartup,
    String? route,
  ) {
    int flags = 0;
    final Map<String, String> environment = <String, String>{};

    void addFlag(String value) {
      flags += 1;
      environment['FLUTTER_ENGINE_SWITCH_$flags'] = value;
    }

    void finish() {
      environment['FLUTTER_ENGINE_SWITCHES'] = flags.toString();
    }

    addFlag('enable-dart-profiling=true');

    if (traceStartup) {
      addFlag('trace-startup=true');
    }
    if (route != null) {
      addFlag('route=$route');
    }
    if (debuggingOptions.enableSoftwareRendering) {
      addFlag('enable-software-rendering=true');
    }
    if (debuggingOptions.skiaDeterministicRendering) {
      addFlag('skia-deterministic-rendering=true');
    }
    if (debuggingOptions.traceSkia) {
      addFlag('trace-skia=true');
    }
    if (debuggingOptions.traceAllowlist != null) {
      addFlag('trace-allowlist=${debuggingOptions.traceAllowlist}');
    }
    if (debuggingOptions.traceSkiaAllowlist != null) {
      addFlag('trace-skia-allowlist=${debuggingOptions.traceSkiaAllowlist}');
    }
    if (debuggingOptions.traceSystrace) {
      addFlag('trace-systrace=true');
    }
    if (debuggingOptions.traceToFile != null) {
      addFlag('trace-to-file=${debuggingOptions.traceToFile}');
    }
    if (debuggingOptions.endlessTraceBuffer) {
      addFlag('endless-trace-buffer=true');
    }
    if (debuggingOptions.dumpSkpOnShaderCompilation) {
      addFlag('dump-skp-on-shader-compilation=true');
    }
    if (debuggingOptions.cacheSkSL) {
      addFlag('cache-sksl=true');
    }
    if (debuggingOptions.purgePersistentCache) {
      addFlag('purge-persistent-cache=true');
    }
    switch (debuggingOptions.enableImpeller) {
      case ImpellerStatus.enabled:
        addFlag('enable-impeller=true');
      case ImpellerStatus.disabled:
      case ImpellerStatus.platformDefault:
        addFlag('enable-impeller=false');
    }
    // Options only supported when there is a VM Service connection between the
    // tool and the device, usually in debug or profile mode.
    if (debuggingOptions.debuggingEnabled) {
      if (debuggingOptions.deviceVmServicePort != null) {
        addFlag('vm-service-port=${debuggingOptions.deviceVmServicePort}');
      }
      if (debuggingOptions.buildInfo.isDebug) {
        addFlag('enable-checked-mode=true');
        addFlag('verify-entry-points=true');
      }
      if (debuggingOptions.startPaused) {
        addFlag('start-paused=true');
      }
      if (debuggingOptions.disableServiceAuthCodes) {
        addFlag('disable-service-auth-codes=true');
      }
      final String dartVmFlags = computeDartVmFlags(debuggingOptions);
      if (dartVmFlags.isNotEmpty) {
        addFlag('dart-flags=$dartVmFlags');
      }
      if (debuggingOptions.useTestFonts) {
        addFlag('use-test-fonts=true');
      }
      if (debuggingOptions.verboseSystemLogs) {
        addFlag('verbose-logging=true');
      }
    }
    finish();
    return environment;
  }
}

/// A log reader for desktop applications that delegates to a [Process] stdout
/// and stderr streams.
class DesktopLogReader extends DeviceLogReader {
  final StreamController<List<int>> _inputController = StreamController<List<int>>.broadcast();

  /// Begin listening to the stdout and stderr streams of the provided [process].
  void initializeProcess(Process process) {
    final StreamSubscription<List<int>> stdoutSub = process.stdout.listen(_inputController.add);
    final StreamSubscription<List<int>> stderrSub = process.stderr.listen(_inputController.add);
    final Future<void> stdioFuture = Future.wait<void>(<Future<void>>[
      stdoutSub.asFuture<void>(),
      stderrSub.asFuture<void>(),
    ]);
    process.exitCode.whenComplete(() async {
      // Wait for output to be fully processed.
      await stdioFuture;
      // The streams have already completed, so waiting for the stream
      // cancellation to complete is not needed.
      unawaited(stdoutSub.cancel());
      unawaited(stderrSub.cancel());
      await _inputController.close();
    });
  }

  @override
  Stream<String> get logLines {
    return _inputController.stream.transform(utf8.decoder).transform(const LineSplitter());
  }

  @override
  String get name => 'desktop';

  @override
  void dispose() {
    // Nothing to dispose.
  }

  @override
  Future<void> provideVmService(FlutterVmService connectedVmService) async {}
}
