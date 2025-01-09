// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../application_package.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/process.dart';
import '../base/utils.dart';
import '../base/version.dart';
import '../build_info.dart';
import '../convert.dart';
import '../devfs.dart';
import '../device.dart';
import '../device_port_forwarder.dart';
import '../device_vm_service_discovery_for_attach.dart';
import '../globals.dart' as globals;
import '../macos/xcode.dart';
import '../project.dart';
import '../protocol_discovery.dart';
import '../vmservice.dart';
import 'application_package.dart';
import 'mac.dart';
import 'plist_parser.dart';

const String iosSimulatorId = 'apple_ios_simulator';

class IOSSimulators extends PollingDeviceDiscovery {
  IOSSimulators({required IOSSimulatorUtils iosSimulatorUtils})
    : _iosSimulatorUtils = iosSimulatorUtils,
      super('iOS simulators');

  final IOSSimulatorUtils _iosSimulatorUtils;

  @override
  bool get supportsPlatform => globals.platform.isMacOS;

  @override
  bool get canListAnything => globals.iosWorkflow?.canListDevices ?? false;

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async =>
      _iosSimulatorUtils.getAttachedDevices();

  @override
  List<String> get wellKnownIds => const <String>[];
}

class IOSSimulatorUtils {
  IOSSimulatorUtils({
    required Xcode xcode,
    required Logger logger,
    required ProcessManager processManager,
  }) : _simControl = SimControl(logger: logger, processManager: processManager, xcode: xcode),
       _xcode = xcode;

  final SimControl _simControl;
  final Xcode _xcode;

  Future<List<IOSSimulator>> getAttachedDevices() async {
    if (!_xcode.isInstalledAndMeetsVersionCheck) {
      return <IOSSimulator>[];
    }

    final List<BootedSimDevice> connected = await _simControl.getConnectedDevices();
    return connected
        .map<IOSSimulator?>((BootedSimDevice device) {
          final String? udid = device.udid;
          final String? name = device.name;
          if (udid == null) {
            globals.printTrace('Could not parse simulator udid');
            return null;
          }
          if (name == null) {
            globals.printTrace('Could not parse simulator name');
            return null;
          }
          return IOSSimulator(
            udid,
            name: name,
            simControl: _simControl,
            simulatorCategory: device.category,
            logger: _simControl._logger,
          );
        })
        .whereType<IOSSimulator>()
        .toList();
  }

  Future<List<IOSSimulatorRuntime>> getAvailableIOSRuntimes() async {
    if (!_xcode.isInstalledAndMeetsVersionCheck) {
      return <IOSSimulatorRuntime>[];
    }

    return _simControl.listAvailableIOSRuntimes();
  }
}

/// A wrapper around the `simctl` command line tool.
class SimControl {
  SimControl({required Logger logger, required ProcessManager processManager, required Xcode xcode})
    : _logger = logger,
      _xcode = xcode,
      _processUtils = ProcessUtils(processManager: processManager, logger: logger);

  final Logger _logger;
  final ProcessUtils _processUtils;
  final Xcode _xcode;

  /// Runs `simctl list --json` and returns the JSON of the corresponding
  /// [section].
  Future<Map<String, Object?>> _listBootedDevices() async {
    // Sample output from `simctl list available booted --json`:
    //
    // {
    //   "devices" : {
    //     "com.apple.CoreSimulator.SimRuntime.iOS-14-0" : [
    //       {
    //         "lastBootedAt" : "2022-07-26T01:46:23Z",
    //         "dataPath" : "\/Users\/magder\/Library\/Developer\/CoreSimulator\/Devices\/9EC90A99-6924-472D-8CDD-4D8234AB4779\/data",
    //         "dataPathSize" : 1620578304,
    //         "logPath" : "\/Users\/magder\/Library\/Logs\/CoreSimulator\/9EC90A99-6924-472D-8CDD-4D8234AB4779",
    //         "udid" : "9EC90A99-6924-472D-8CDD-4D8234AB4779",
    //         "isAvailable" : true,
    //         "logPathSize" : 9740288,
    //         "deviceTypeIdentifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-11",
    //         "state" : "Booted",
    //         "name" : "iPhone 11"
    //       }
    //     ],
    //     "com.apple.CoreSimulator.SimRuntime.iOS-13-0" : [
    //
    //     ],
    //     "com.apple.CoreSimulator.SimRuntime.iOS-12-4" : [
    //
    //     ],
    //     "com.apple.CoreSimulator.SimRuntime.iOS-16-0" : [
    //
    //     ]
    //   }
    // }

    final List<String> command = <String>[
      ..._xcode.xcrunCommand(),
      'simctl',
      'list',
      'devices',
      'booted',
      'iOS',
      '--json',
    ];
    _logger.printTrace(command.join(' '));
    final RunResult results = await _processUtils.run(command);
    if (results.exitCode != 0) {
      _logger.printError('Error executing simctl: ${results.exitCode}\n${results.stderr}');
      return <String, Map<String, Object?>>{};
    }
    try {
      final Object? decodeResult = (json.decode(results.stdout) as Map<String, Object?>)['devices'];
      if (decodeResult is Map<String, Object?>) {
        return decodeResult;
      }
      _logger.printError('simctl returned unexpected JSON response: ${results.stdout}');
      return <String, Object>{};
    } on FormatException {
      // We failed to parse the simctl output, or it returned junk.
      // One known message is "Install Started" isn't valid JSON but is
      // returned sometimes.
      _logger.printError('simctl returned non-JSON response: ${results.stdout}');
      return <String, Object>{};
    }
  }

  /// Returns all the connected simulator devices.
  Future<List<BootedSimDevice>> getConnectedDevices() async {
    final Map<String, Object?> devicesSection = await _listBootedDevices();

    return <BootedSimDevice>[
      for (final String deviceCategory in devicesSection.keys)
        if (devicesSection[deviceCategory] case final List<Object?> devicesData)
          for (final Object? data in devicesData.map<Map<String, Object?>?>(castStringKeyedMap))
            if (data is Map<String, Object?>) BootedSimDevice(deviceCategory, data),
    ];
  }

  Future<bool> isInstalled(String deviceId, String appId) {
    return _processUtils.exitsHappy(<String>[
      ..._xcode.xcrunCommand(),
      'simctl',
      'get_app_container',
      deviceId,
      appId,
    ]);
  }

  Future<RunResult> install(String deviceId, String appPath) async {
    RunResult result;
    try {
      result = await _processUtils.run(<String>[
        ..._xcode.xcrunCommand(),
        'simctl',
        'install',
        deviceId,
        appPath,
      ], throwOnError: true);
    } on ProcessException catch (exception) {
      throwToolExit(
        'Unable to install $appPath on $deviceId. This is sometimes caused by a malformed plist file:\n$exception',
      );
    }
    return result;
  }

  Future<RunResult> uninstall(String deviceId, String appId) async {
    RunResult result;
    try {
      result = await _processUtils.run(<String>[
        ..._xcode.xcrunCommand(),
        'simctl',
        'uninstall',
        deviceId,
        appId,
      ], throwOnError: true);
    } on ProcessException catch (exception) {
      throwToolExit('Unable to uninstall $appId from $deviceId:\n$exception');
    }
    return result;
  }

  Future<RunResult> launch(
    String deviceId,
    String appIdentifier, [
    List<String>? launchArgs,
  ]) async {
    RunResult result;
    try {
      result = await _processUtils.run(<String>[
        ..._xcode.xcrunCommand(),
        'simctl',
        'launch',
        deviceId,
        appIdentifier,
        ...?launchArgs,
      ], throwOnError: true);
    } on ProcessException catch (exception) {
      throwToolExit('Unable to launch $appIdentifier on $deviceId:\n$exception');
    }
    return result;
  }

  Future<RunResult> stopApp(String deviceId, String appIdentifier) async {
    RunResult result;
    try {
      result = await _processUtils.run(<String>[
        ..._xcode.xcrunCommand(),
        'simctl',
        'terminate',
        deviceId,
        appIdentifier,
      ], throwOnError: true);
    } on ProcessException catch (exception) {
      throwToolExit('Unable to terminate $appIdentifier on $deviceId:\n$exception');
    }
    return result;
  }

  Future<void> takeScreenshot(String deviceId, String outputPath) async {
    try {
      await _processUtils.run(<String>[
        ..._xcode.xcrunCommand(),
        'simctl',
        'io',
        deviceId,
        'screenshot',
        outputPath,
      ], throwOnError: true);
    } on ProcessException catch (exception) {
      _logger.printError('Unable to take screenshot of $deviceId:\n$exception');
    }
  }

  /// Runs `simctl list runtimes available iOS --json` and returns all available iOS simulator runtimes.
  Future<List<IOSSimulatorRuntime>> listAvailableIOSRuntimes() async {
    final List<IOSSimulatorRuntime> runtimes = <IOSSimulatorRuntime>[];
    final RunResult results = await _processUtils.run(<String>[
      ..._xcode.xcrunCommand(),
      'simctl',
      'list',
      'runtimes',
      'available',
      'iOS',
      '--json',
    ]);

    if (results.exitCode != 0) {
      _logger.printError('Error executing simctl: ${results.exitCode}\n${results.stderr}');
      return runtimes;
    }

    try {
      final Object? decodeResult =
          (json.decode(results.stdout) as Map<String, Object?>)['runtimes'];
      if (decodeResult is List<Object?>) {
        for (final Object? runtimeData in decodeResult) {
          if (runtimeData is Map<String, Object?>) {
            runtimes.add(IOSSimulatorRuntime.fromJson(runtimeData));
          }
        }
      }

      return runtimes;
    } on FormatException {
      // We failed to parse the simctl output, or it returned junk.
      // One known message is "Install Started" isn't valid JSON but is
      // returned sometimes.
      _logger.printError('simctl returned non-JSON response: ${results.stdout}');
      return runtimes;
    }
  }
}

class BootedSimDevice {
  BootedSimDevice(this.category, this.data);

  final String category;
  final Map<String, Object?> data;

  String? get name => data['name']?.toString();
  String? get udid => data['udid']?.toString();
}

class IOSSimulator extends Device {
  IOSSimulator(
    super.id, {
    required this.name,
    required this.simulatorCategory,
    required SimControl simControl,
    required super.logger,
  }) : _simControl = simControl,
       super(category: Category.mobile, platformType: PlatformType.ios, ephemeral: true);

  @override
  final String name;

  final String simulatorCategory;

  final SimControl _simControl;

  @override
  DevFSWriter createDevFSWriter(ApplicationPackage? app, String? userIdentifier) {
    return LocalDevFSWriter(fileSystem: globals.fs);
  }

  @override
  Future<bool> get isLocalEmulator async => true;

  @override
  Future<String> get emulatorId async => iosSimulatorId;

  @override
  bool get supportsHotReload => true;

  @override
  bool get supportsHotRestart => true;

  @override
  bool get supportsFlavors => true;

  @override
  Future<bool> get supportsHardwareRendering async => false;

  @override
  bool supportsRuntimeMode(BuildMode buildMode) => buildMode == BuildMode.debug;

  final Map<IOSApp?, DeviceLogReader> _logReaders = <IOSApp?, DeviceLogReader>{};
  _IOSSimulatorDevicePortForwarder? _portForwarder;

  @override
  Future<bool> isAppInstalled(ApplicationPackage app, {String? userIdentifier}) {
    return _simControl.isInstalled(id, app.id);
  }

  @override
  Future<bool> isLatestBuildInstalled(ApplicationPackage app) async => false;

  @override
  Future<bool> installApp(covariant IOSApp app, {String? userIdentifier}) async {
    try {
      await _simControl.install(id, app.simulatorBundlePath);
      return true;
    } on Exception {
      return false;
    }
  }

  @override
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier}) async {
    try {
      await _simControl.uninstall(id, app.id);
      return true;
    } on Exception {
      return false;
    }
  }

  @override
  bool isSupported() {
    if (!globals.platform.isMacOS) {
      _supportMessage = 'iOS devices require a Mac host machine.';
      return false;
    }

    // Check if the device is part of a blocked category.
    // We do not yet support WatchOS or tvOS devices.
    final RegExp blocklist = RegExp(r'Apple (TV|Watch)', caseSensitive: false);
    if (blocklist.hasMatch(name)) {
      _supportMessage = 'Flutter does not support Apple TV or Apple Watch.';
      return false;
    }
    return true;
  }

  String? _supportMessage;

  @override
  String supportMessage() {
    if (isSupported()) {
      return 'Supported';
    }

    return _supportMessage ?? 'Unknown';
  }

  @override
  Future<LaunchResult> startApp(
    IOSApp package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, Object?> platformArgs = const <String, Object?>{},
    bool prebuiltApplication = false,
    String? userIdentifier,
  }) async {
    if (!prebuiltApplication && package is BuildableIOSApp) {
      globals.printTrace('Building ${package.name} for $id.');

      try {
        await _setupUpdatedApplicationBundle(package, debuggingOptions.buildInfo, mainPath);
      } on ToolExit catch (e) {
        globals.printError('${e.message}');
        return LaunchResult.failed();
      }
    } else {
      if (!await installApp(package)) {
        return LaunchResult.failed();
      }
    }

    // Prepare launch arguments.
    final List<String> launchArguments = debuggingOptions.getIOSLaunchArguments(
      EnvironmentType.simulator,
      route,
      platformArgs,
    );

    ProtocolDiscovery? vmServiceDiscovery;
    if (debuggingOptions.debuggingEnabled) {
      vmServiceDiscovery = ProtocolDiscovery.vmService(
        getLogReader(app: package),
        ipv6: debuggingOptions.ipv6,
        hostPort: debuggingOptions.hostVmServicePort,
        devicePort: debuggingOptions.deviceVmServicePort,
        logger: globals.logger,
      );
    }

    // Launch the updated application in the simulator.
    try {
      // Use the built application's Info.plist to get the bundle identifier,
      // which should always yield the correct value and does not require
      // parsing the xcodeproj or configuration files.
      // See https://github.com/flutter/flutter/issues/31037 for more information.
      final String plistPath = globals.fs.path.join(package.simulatorBundlePath, 'Info.plist');
      final String? bundleIdentifier = globals.plistParser.getValueFromFile<String>(
        plistPath,
        PlistParser.kCFBundleIdentifierKey,
      );
      if (bundleIdentifier == null) {
        globals.printError(
          'Invalid prebuilt iOS app. Info.plist does not contain bundle identifier',
        );
        return LaunchResult.failed();
      }

      await _simControl.launch(id, bundleIdentifier, launchArguments);
    } on Exception catch (error) {
      globals.printError('$error');
      return LaunchResult.failed();
    }

    if (!debuggingOptions.debuggingEnabled) {
      return LaunchResult.succeeded();
    }

    // Wait for the service protocol port here. This will complete once the
    // device has printed "Dart VM Service is listening on..."
    globals.printTrace('Waiting for VM Service port to be available...');

    try {
      final Uri? deviceUri = await vmServiceDiscovery?.uri;
      if (deviceUri != null) {
        return LaunchResult.succeeded(vmServiceUri: deviceUri);
      }
      globals.printError(
        'Error waiting for a debug connection: '
        'The log reader failed unexpectedly',
      );
    } on Exception catch (error) {
      globals.printError('Error waiting for a debug connection: $error');
    } finally {
      await vmServiceDiscovery?.cancel();
    }
    return LaunchResult.failed();
  }

  Future<void> _setupUpdatedApplicationBundle(
    BuildableIOSApp app,
    BuildInfo buildInfo,
    String? mainPath,
  ) async {
    // Step 1: Build the Xcode project.
    // The build mode for the simulator is always debug.
    assert(buildInfo.isDebug);

    final XcodeBuildResult buildResult = await buildXcodeProject(
      app: app,
      buildInfo: buildInfo,
      targetOverride: mainPath,
      environmentType: EnvironmentType.simulator,
      deviceID: id,
    );
    if (!buildResult.success) {
      await diagnoseXcodeBuildFailure(
        buildResult,
        analytics: globals.analytics,
        fileSystem: globals.fs,
        logger: globals.logger,
        platform: SupportedPlatform.ios,
        project: app.project.parent,
      );
      throwToolExit('Could not build the application for the simulator.');
    }

    // Step 2: Assert that the Xcode project was successfully built.
    final Directory bundle = globals.fs.directory(app.simulatorBundlePath);
    final bool bundleExists = bundle.existsSync();
    if (!bundleExists) {
      throwToolExit('Could not find the built application bundle at ${bundle.path}.');
    }

    // Step 3: Install the updated bundle to the simulator.
    await _simControl.install(id, globals.fs.path.absolute(bundle.path));
  }

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    if (app == null) {
      return false;
    }
    return (await _simControl.stopApp(id, app.id)).exitCode == 0;
  }

  String get logFilePath {
    final String? logPath = globals.platform.environment['IOS_SIMULATOR_LOG_FILE_PATH'];
    return logPath != null
        ? logPath.replaceAll('%{id}', id)
        : globals.fs.path.join(
          globals.fsUtils.homeDirPath!,
          'Library',
          'Logs',
          'CoreSimulator',
          id,
          'system.log',
        );
  }

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.ios;

  @override
  Future<String> get sdkNameAndVersion async => simulatorCategory;

  final RegExp _iosSdkRegExp = RegExp(r'iOS( |-)(\d+)');

  Future<int> get sdkMajorVersion async {
    final Match? sdkMatch = _iosSdkRegExp.firstMatch(await sdkNameAndVersion);
    return int.parse(sdkMatch?.group(2) ?? '11');
  }

  @override
  DeviceLogReader getLogReader({covariant IOSApp? app, bool includePastLogs = false}) {
    assert(!includePastLogs, 'Past log reading not supported on iOS simulators.');
    return _logReaders.putIfAbsent(app, () => _IOSSimulatorLogReader(this, app));
  }

  @override
  DevicePortForwarder get portForwarder =>
      _portForwarder ??= _IOSSimulatorDevicePortForwarder(this);

  @override
  void clearLogs() {
    final File logFile = globals.fs.file(logFilePath);
    if (logFile.existsSync()) {
      final RandomAccessFile randomFile = logFile.openSync(mode: FileMode.write);
      randomFile.truncateSync(0);
      randomFile.closeSync();
    }
  }

  Future<void> ensureLogsExists() async {
    if (await sdkMajorVersion < 11) {
      final File logFile = globals.fs.file(logFilePath);
      if (!logFile.existsSync()) {
        logFile.writeAsBytesSync(<int>[]);
      }
    }
  }

  @override
  VMServiceDiscoveryForAttach getVMServiceDiscoveryForAttach({
    String? appId,
    String? fuchsiaModule,
    int? filterDevicePort,
    int? expectedHostPort,
    required bool ipv6,
    required Logger logger,
  }) {
    final MdnsVMServiceDiscoveryForAttach mdnsVMServiceDiscoveryForAttach =
        MdnsVMServiceDiscoveryForAttach(
          device: this,
          appId: appId,
          deviceVmservicePort: filterDevicePort,
          hostVmservicePort: expectedHostPort,
          usesIpv6: ipv6,
          useDeviceIPAsHost: false,
        );

    return DelegateVMServiceDiscoveryForAttach(<VMServiceDiscoveryForAttach>[
      mdnsVMServiceDiscoveryForAttach,
      super.getVMServiceDiscoveryForAttach(
        appId: appId,
        fuchsiaModule: fuchsiaModule,
        filterDevicePort: filterDevicePort,
        expectedHostPort: expectedHostPort,
        ipv6: ipv6,
        logger: logger,
      ),
    ]);
  }

  @override
  bool get supportsScreenshot => true;

  @override
  Future<void> takeScreenshot(File outputFile) {
    return _simControl.takeScreenshot(id, outputFile.path);
  }

  @override
  bool isSupportedForProject(FlutterProject flutterProject) {
    return flutterProject.ios.existsSync();
  }

  @override
  Future<void> dispose() async {
    for (final DeviceLogReader logReader in _logReaders.values) {
      logReader.dispose();
    }
    await _portForwarder?.dispose();
  }
}

class IOSSimulatorRuntime {
  IOSSimulatorRuntime._({
    this.bundlePath,
    this.buildVersion,
    this.platform,
    this.runtimeRoot,
    this.identifier,
    this.version,
    this.isInternal,
    this.isAvailable,
    this.name,
  });

  // Example:
  // {
  //   "bundlePath" : "\/Library\/Developer\/CoreSimulator\/Volumes\/iOS_21A5277g\/Library\/Developer\/CoreSimulator\/Profiles\/Runtimes\/iOS 17.0.simruntime",
  //   "buildversion" : "21A5277g",
  //   "platform" : "iOS",
  //   "runtimeRoot" : "\/Library\/Developer\/CoreSimulator\/Volumes\/iOS_21A5277g\/Library\/Developer\/CoreSimulator\/Profiles\/Runtimes\/iOS 17.0.simruntime\/Contents\/Resources\/RuntimeRoot",
  //   "identifier" : "com.apple.CoreSimulator.SimRuntime.iOS-17-0",
  //   "version" : "17.0",
  //   "isInternal" : false,
  //   "isAvailable" : true,
  //   "name" : "iOS 17.0",
  //   "supportedDeviceTypes" : [
  //     {
  //       "bundlePath" : "\/Applications\/Xcode.app\/Contents\/Developer\/Platforms\/iPhoneOS.platform\/Library\/Developer\/CoreSimulator\/Profiles\/DeviceTypes\/iPhone 8.simdevicetype",
  //       "name" : "iPhone 8",
  //       "identifier" : "com.apple.CoreSimulator.SimDeviceType.iPhone-8",
  //       "productFamily" : "iPhone"
  //     }
  //   ]
  // },
  factory IOSSimulatorRuntime.fromJson(Map<String, Object?> data) {
    return IOSSimulatorRuntime._(
      bundlePath: data['bundlePath']?.toString(),
      buildVersion: data['buildversion']?.toString(),
      platform: data['platform']?.toString(),
      runtimeRoot: data['runtimeRoot']?.toString(),
      identifier: data['identifier']?.toString(),
      version: Version.parse(data['version']?.toString()),
      isInternal: data['isInternal'] is bool? ? data['isInternal'] as bool? : null,
      isAvailable: data['isAvailable'] is bool? ? data['isAvailable'] as bool? : null,
      name: data['name']?.toString(),
    );
  }

  final String? bundlePath;
  final String? buildVersion;
  final String? platform;
  final String? runtimeRoot;
  final String? identifier;
  final Version? version;
  final bool? isInternal;
  final bool? isAvailable;
  final String? name;
}

/// Launches the device log reader process on the host and parses the syslog.
@visibleForTesting
Future<Process> launchDeviceSystemLogTool(IOSSimulator device) async {
  return globals.processUtils.start(<String>['tail', '-n', '0', '-F', device.logFilePath]);
}

/// Launches the device log reader process on the host and parses unified logging.
@visibleForTesting
Future<Process> launchDeviceUnifiedLogging(IOSSimulator device, String? appName) async {
  // Make NSPredicate concatenation easier to read.
  String orP(List<String> clauses) => '(${clauses.join(" OR ")})';
  String andP(List<String> clauses) => clauses.join(' AND ');
  String notP(String clause) => 'NOT($clause)';

  final String predicate = andP(<String>[
    'eventType = logEvent',
    if (appName != null) 'processImagePath ENDSWITH "$appName"',
    // Either from Flutter or Swift (maybe assertion or fatal error) or from the app itself.
    orP(<String>[
      'senderImagePath ENDSWITH "/Flutter"',
      'senderImagePath ENDSWITH "/libswiftCore.dylib"',
      'processImageUUID == senderImageUUID',
    ]),
    // Filter out some messages that clearly aren't related to Flutter.
    notP('eventMessage CONTAINS ": could not find icon for representation -> com.apple."'),
    notP('eventMessage BEGINSWITH "assertion failed: "'),
    notP('eventMessage CONTAINS " libxpc.dylib "'),
  ]);

  return globals.processUtils.start(<String>[
    ...globals.xcode!.xcrunCommand(),
    'simctl',
    'spawn',
    device.id,
    'log',
    'stream',
    '--style',
    'json',
    '--predicate',
    predicate,
  ]);
}

@visibleForTesting
Future<Process?> launchSystemLogTool(IOSSimulator device) async {
  // Versions of iOS prior to 11 tail the simulator syslog file.
  if (await device.sdkMajorVersion < 11) {
    return globals.processUtils.start(<String>[
      'tail',
      '-n',
      '0',
      '-F',
      '/private/var/log/system.log',
    ]);
  }

  // For iOS 11 and later, all relevant detail is in the device log.
  return null;
}

class _IOSSimulatorLogReader extends DeviceLogReader {
  _IOSSimulatorLogReader(this.device, IOSApp? app) : _appName = app?.name?.replaceAll('.app', '');

  final IOSSimulator device;

  final String? _appName;

  late final StreamController<String> _linesController = StreamController<String>.broadcast(
    onListen: _start,
    onCancel: _stop,
  );

  // We log from two files: the device and the system log.
  Process? _deviceProcess;
  Process? _systemProcess;

  @override
  Stream<String> get logLines => _linesController.stream;

  @override
  String get name => device.name;

  Future<void> _start() async {
    // Unified logging iOS 11 and greater (introduced in iOS 10).
    if (await device.sdkMajorVersion >= 11) {
      _deviceProcess = await launchDeviceUnifiedLogging(device, _appName);
      _deviceProcess?.stdout
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen(_onUnifiedLoggingLine);
      _deviceProcess?.stderr
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen(_onUnifiedLoggingLine);
    } else {
      // Fall back to syslog parsing.
      await device.ensureLogsExists();
      _deviceProcess = await launchDeviceSystemLogTool(device);
      _deviceProcess?.stdout
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen(_onSysLogDeviceLine);
      _deviceProcess?.stderr
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen(_onSysLogDeviceLine);
    }

    // Track system.log crashes.
    // ReportCrash[37965]: Saved crash report for FlutterRunner[37941]...
    _systemProcess = await launchSystemLogTool(device);
    if (_systemProcess != null) {
      _systemProcess?.stdout
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen(_onSystemLine);
      _systemProcess?.stderr
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen(_onSystemLine);
    }

    // We don't want to wait for the process or its callback. Best effort
    // cleanup in the callback.
    unawaited(
      _deviceProcess?.exitCode.whenComplete(() {
        if (_linesController.hasListener) {
          _linesController.close();
        }
      }),
    );
  }

  // Match the log prefix (in order to shorten it):
  // * Xcode 8: Sep 13 15:28:51 cbracken-macpro localhost Runner[37195]: (Flutter) The Dart VM service is listening on http://127.0.0.1:57701/
  // * Xcode 9: 2017-09-13 15:26:57.228948-0700  localhost Runner[37195]: (Flutter) The Dart VM service is listening on http://127.0.0.1:57701/
  static final RegExp _mapRegex = RegExp(
    r'\S+ +\S+ +(?:\S+) (.+?(?=\[))\[\d+\]\)?: (\(.*?\))? *(.*)$',
  );

  // Jan 31 19:23:28 --- last message repeated 1 time ---
  static final RegExp _lastMessageSingleRegex = RegExp(
    r'\S+ +\S+ +\S+ --- last message repeated 1 time ---$',
  );
  static final RegExp _lastMessageMultipleRegex = RegExp(
    r'\S+ +\S+ +\S+ --- last message repeated (\d+) times ---$',
  );

  static final RegExp _flutterRunnerRegex = RegExp(r' FlutterRunner\[\d+\] ');

  // Remember what we did with the last line, in case we need to process
  // a multiline record
  bool _lastLineMatched = false;

  String? _filterDeviceLine(String string) {
    final Match? match = _mapRegex.matchAsPrefix(string);
    if (match != null) {
      // The category contains the text between the date and the PID. Depending on which version of iOS being run,
      // it can contain "hostname App Name" or just "App Name".
      final String? category = match.group(1);
      final String? tag = match.group(2);
      final String? content = match.group(3);

      // Filter out log lines from an app other than this one (category doesn't match the app name).
      // If the hostname is included in the category, check that it doesn't end with the app name.
      final String? appName = _appName;
      if (appName != null && category != null && !category.endsWith(appName)) {
        return null;
      }

      if (tag != null && tag != '(Flutter)') {
        return null;
      }

      // Filter out some messages that clearly aren't related to Flutter.
      if (string.contains(': could not find icon for representation -> com.apple.')) {
        return null;
      }

      // assertion failed: 15G1212 13E230: libxpc.dylib + 57882 [66C28065-C9DB-3C8E-926F-5A40210A6D1B]: 0x7d
      if (content != null &&
          content.startsWith('assertion failed: ') &&
          content.contains(' libxpc.dylib ')) {
        return null;
      }

      if (appName == null) {
        return '$category: $content';
      } else if (category != null && (category == appName || category.endsWith(' $appName'))) {
        return content;
      }

      return null;
    }

    if (string.startsWith('Filtering the log data using ')) {
      return null;
    }

    if (string.startsWith('Timestamp                       (process)[PID]')) {
      return null;
    }

    if (_lastMessageSingleRegex.matchAsPrefix(string) != null) {
      return null;
    }

    if (RegExp(r'assertion failed: .* libxpc.dylib .* 0x7d$').matchAsPrefix(string) != null) {
      return null;
    }

    // Starts with space(s) - continuation of the multiline message
    if (RegExp(r'\s+').matchAsPrefix(string) != null && !_lastLineMatched) {
      return null;
    }

    return string;
  }

  String? _lastLine;

  void _onSysLogDeviceLine(String line) {
    globals.printTrace('[DEVICE LOG] $line');
    final Match? multi = _lastMessageMultipleRegex.matchAsPrefix(line);

    if (multi != null) {
      if (_lastLine != null) {
        int repeat = int.parse(multi.group(1)!);
        repeat = math.max(0, math.min(100, repeat));
        for (int i = 1; i < repeat; i++) {
          _linesController.add(_lastLine!);
        }
      }
    } else {
      _lastLine = _filterDeviceLine(line);
      if (_lastLine != null) {
        _linesController.add(_lastLine!);
        _lastLineMatched = true;
      } else {
        _lastLineMatched = false;
      }
    }
  }

  //   "eventMessage" : "flutter: 21",
  static final RegExp _unifiedLoggingEventMessageRegex = RegExp(r'.*"eventMessage" : (".*")');
  void _onUnifiedLoggingLine(String line) {
    // The log command predicate handles filtering, so every log eventMessage should be decoded and added.
    final Match? eventMessageMatch = _unifiedLoggingEventMessageRegex.firstMatch(line);
    if (eventMessageMatch != null) {
      final String message = eventMessageMatch.group(1)!;
      try {
        final Object? decodedJson = jsonDecode(message);
        if (decodedJson is String) {
          _linesController.add(decodedJson);
        }
      } on FormatException {
        globals.printError('Logger returned non-JSON response: $message');
      }
    }
  }

  String _filterSystemLog(String string) {
    final Match? match = _mapRegex.matchAsPrefix(string);
    return match == null ? string : '${match.group(1)}: ${match.group(2)}';
  }

  void _onSystemLine(String line) {
    globals.printTrace('[SYS LOG] $line');
    if (!_flutterRunnerRegex.hasMatch(line)) {
      return;
    }

    final String filteredLine = _filterSystemLog(line);

    _linesController.add(filteredLine);
  }

  void _stop() {
    _deviceProcess?.kill();
    _systemProcess?.kill();
  }

  @override
  void dispose() {
    _stop();
  }

  @override
  Future<void> provideVmService(FlutterVmService connectedVmService) async {}
}

class _IOSSimulatorDevicePortForwarder extends DevicePortForwarder {
  _IOSSimulatorDevicePortForwarder(this.device);

  final IOSSimulator device;

  final List<ForwardedPort> _ports = <ForwardedPort>[];

  @override
  List<ForwardedPort> get forwardedPorts => _ports;

  @override
  Future<int> forward(int devicePort, {int? hostPort}) async {
    if (hostPort == null || hostPort == 0) {
      hostPort = devicePort;
    }
    assert(devicePort == hostPort);
    _ports.add(ForwardedPort(devicePort, hostPort));
    return hostPort;
  }

  @override
  Future<void> unforward(ForwardedPort forwardedPort) async {
    _ports.remove(forwardedPort);
  }

  @override
  Future<void> dispose() async {
    final List<ForwardedPort> portsCopy = List<ForwardedPort>.of(_ports);
    for (final ForwardedPort port in portsCopy) {
      await unforward(port);
    }
  }
}
