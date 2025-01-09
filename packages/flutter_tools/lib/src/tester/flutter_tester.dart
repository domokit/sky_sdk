// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:process/process.dart';

import '../application_package.dart';
import '../artifacts.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../build_info.dart';
import '../bundle.dart';
import '../bundle_builder.dart';
import '../desktop_device.dart';
import '../devfs.dart';
import '../device.dart';
import '../device_port_forwarder.dart';
import '../globals.dart' as globals;
import '../native_assets.dart';
import '../project.dart';
import '../protocol_discovery.dart';
import '../version.dart';

class FlutterTesterApp extends ApplicationPackage {
  factory FlutterTesterApp.fromCurrentDirectory(FileSystem fileSystem) {
    return FlutterTesterApp._(fileSystem.currentDirectory);
  }

  FlutterTesterApp._(Directory directory) : _directory = directory, super(id: directory.path);

  final Directory _directory;

  @override
  String get name => _directory.basename;
}

/// The device interface for running on the flutter_tester shell.
///
/// Normally this is only used as the runner for `flutter test`, but it can
/// also be used as a regular device when `--show-test-device` is provided
/// to the flutter command.
class FlutterTesterDevice extends Device {
  FlutterTesterDevice(
    super.id, {
    required ProcessManager processManager,
    required FlutterVersion flutterVersion,
    required super.logger,
    required FileSystem fileSystem,
    required Artifacts artifacts,
    TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
  }) : _processManager = processManager,
       _flutterVersion = flutterVersion,
       _logger = logger,
       _fileSystem = fileSystem,
       _artifacts = artifacts,
       _nativeAssetsBuilder = nativeAssetsBuilder,
       super(platformType: null, category: null, ephemeral: false);

  final ProcessManager _processManager;
  final FlutterVersion _flutterVersion;
  final Logger _logger;
  final FileSystem _fileSystem;
  final Artifacts _artifacts;
  final TestCompilerNativeAssetsBuilder? _nativeAssetsBuilder;

  Process? _process;
  final DevicePortForwarder _portForwarder = const NoOpDevicePortForwarder();

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  Future<String?> get emulatorId async => null;

  @override
  String get name => 'Flutter test device';

  @override
  DevicePortForwarder get portForwarder => _portForwarder;

  @override
  Future<String> get sdkNameAndVersion async {
    final FlutterVersion flutterVersion = _flutterVersion;
    return 'Flutter ${flutterVersion.frameworkRevisionShort}';
  }

  @override
  bool supportsRuntimeMode(BuildMode buildMode) => buildMode == BuildMode.debug;

  @override
  bool get supportsFlavors => true;

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.tester;

  @override
  void clearLogs() {}

  final DesktopLogReader _logReader = DesktopLogReader();

  @override
  DeviceLogReader getLogReader({ApplicationPackage? app, bool includePastLogs = false}) {
    return _logReader;
  }

  @override
  Future<bool> installApp(ApplicationPackage app, {String? userIdentifier}) async => true;

  @override
  Future<bool> isAppInstalled(ApplicationPackage app, {String? userIdentifier}) async => false;

  @override
  Future<bool> isLatestBuildInstalled(ApplicationPackage app) async => false;

  @override
  bool isSupported() => true;

  @override
  Future<LaunchResult> startApp(
    ApplicationPackage? package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, Object?> platformArgs = const <String, Object>{},
    bool prebuiltApplication = false,
    String? userIdentifier,
  }) async {
    final BuildInfo buildInfo = debuggingOptions.buildInfo;
    if (!buildInfo.isDebug) {
      _logger.printError('This device only supports debug mode.');
      return LaunchResult.failed();
    }

    final Directory assetDirectory = _fileSystem.systemTempDirectory.createTempSync(
      'flutter_tester.',
    );
    final String applicationKernelFilePath = getKernelPathForTransformerOptions(
      _fileSystem.path.join(assetDirectory.path, 'flutter-tester-app.dill'),
      trackWidgetCreation: buildInfo.trackWidgetCreation,
    );

    // Build assets and perform initial compilation.
    final FlutterProject project = FlutterProject.current();
    await BundleBuilder().build(
      project: project,
      buildInfo: buildInfo,
      mainPath: mainPath,
      applicationKernelFilePath: applicationKernelFilePath,
      platform: TargetPlatform.tester,
      assetDirPath: assetDirectory.path,
    );

    final List<String> command = <String>[
      _artifacts.getArtifactPath(Artifact.flutterTester),
      '--run-forever',
      '--non-interactive',
      if (debuggingOptions.enableDartProfiling) '--enable-dart-profiling',
      '--packages=${debuggingOptions.buildInfo.packageConfigPath}',
      '--flutter-assets-dir=${assetDirectory.path}',
      if (debuggingOptions.startPaused) '--start-paused',
      if (debuggingOptions.disableServiceAuthCodes) '--disable-service-auth-codes',
      if (debuggingOptions.hostVmServicePort != null)
        '--vm-service-port=${debuggingOptions.hostVmServicePort}',
      applicationKernelFilePath,
    ];

    ProtocolDiscovery? vmServiceDiscovery;
    try {
      _logger.printTrace(command.join(' '));
      _process = await _processManager.start(
        command,
        environment: <String, String>{
          'FLUTTER_TEST': 'true',
          if (globals.platform.isWindows && _nativeAssetsBuilder != null)
            'PATH':
                '${_nativeAssetsBuilder.windowsBuildDirectory(project)};${globals.platform.environment['PATH']}',
        },
      );
      if (!debuggingOptions.debuggingEnabled) {
        return LaunchResult.succeeded();
      }

      vmServiceDiscovery = ProtocolDiscovery.vmService(
        getLogReader(),
        hostPort: debuggingOptions.hostVmServicePort,
        devicePort: debuggingOptions.deviceVmServicePort,
        ipv6: debuggingOptions.ipv6,
        logger: _logger,
      );
      _logReader.initializeProcess(_process!);

      final Uri? vmServiceUri = await vmServiceDiscovery.uri;
      if (vmServiceUri != null) {
        return LaunchResult.succeeded(vmServiceUri: vmServiceUri);
      }
      _logger.printError(
        'Failed to launch $package: '
        'The log reader failed unexpectedly.',
      );
    } on Exception catch (error) {
      _logger.printError('Failed to launch $package: $error');
    } finally {
      await vmServiceDiscovery?.cancel();
    }
    return LaunchResult.failed();
  }

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    _process?.kill();
    _process = null;
    return true;
  }

  @override
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier}) async => true;

  @override
  bool isSupportedForProject(FlutterProject flutterProject) => true;

  @override
  DevFSWriter createDevFSWriter(ApplicationPackage? app, String? userIdentifier) {
    return LocalDevFSWriter(fileSystem: _fileSystem);
  }

  @override
  Future<void> dispose() async {
    _logReader.dispose();
    await _portForwarder.dispose();
  }
}

class FlutterTesterDevices extends PollingDeviceDiscovery {
  FlutterTesterDevices({
    required FileSystem fileSystem,
    required Artifacts artifacts,
    required ProcessManager processManager,
    required Logger logger,
    required FlutterVersion flutterVersion,
    TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
  }) : _testerDevice = FlutterTesterDevice(
         kTesterDeviceId,
         fileSystem: fileSystem,
         artifacts: artifacts,
         processManager: processManager,
         logger: logger,
         flutterVersion: flutterVersion,
         nativeAssetsBuilder: nativeAssetsBuilder,
       ),
       super('Flutter tester');

  static const String kTesterDeviceId = 'flutter-tester';

  static bool showFlutterTesterDevice = false;

  final FlutterTesterDevice _testerDevice;

  @override
  bool get canListAnything => true;

  @override
  bool get supportsPlatform => true;

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    return showFlutterTesterDevice ? <Device>[_testerDevice] : <Device>[];
  }

  @override
  List<String> get wellKnownIds => const <String>[kTesterDeviceId];
}
