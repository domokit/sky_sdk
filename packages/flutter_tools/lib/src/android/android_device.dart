// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:vm_service/vm_service.dart';

import '../application_package.dart';
import '../base/common.dart' show throwToolExit;
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../build_info.dart';
import '../convert.dart';
import '../device.dart';
import '../device_port_forwarder.dart';
import '../device_vm_service_discovery_for_attach.dart';
import '../project.dart';
import '../protocol_discovery.dart';
import '../vmservice.dart';
import 'android.dart';
import 'android_builder.dart';
import 'android_console.dart';
import 'android_sdk.dart';
import 'application_package.dart';

/// Whether the [AndroidDevice] is believed to be a physical device or an emulator.
enum HardwareType { emulator, physical }

/// Map to help our `isLocalEmulator` detection.
///
/// See [AndroidDevice] for more explanation of why this is needed.
const Map<String, HardwareType> kKnownHardware = <String, HardwareType>{
  'goldfish': HardwareType.emulator,
  'qcom': HardwareType.physical,
  'ranchu': HardwareType.emulator,
  'samsungexynos7420': HardwareType.physical,
  'samsungexynos7580': HardwareType.physical,
  'samsungexynos7870': HardwareType.physical,
  'samsungexynos7880': HardwareType.physical,
  'samsungexynos8890': HardwareType.physical,
  'samsungexynos8895': HardwareType.physical,
  'samsungexynos9810': HardwareType.physical,
  'samsungexynos7570': HardwareType.physical,
};

/// A physical Android device or emulator.
///
/// While [isEmulator] attempts to distinguish between the device categories,
/// this is a best effort process and not a guarantee; certain physical devices
/// identify as emulators. These device identifiers may be added to the [kKnownHardware]
/// map to specify that they are actually physical devices.
class AndroidDevice extends Device {
  AndroidDevice(
    super.id, {
    this.productID,
    required this.modelID,
    this.deviceCodeName,
    required super.logger,
    required ProcessManager processManager,
    required Platform platform,
    required AndroidSdk androidSdk,
    required FileSystem fileSystem,
    AndroidConsoleSocketFactory androidConsoleSocketFactory = kAndroidConsoleSocketFactory,
  }) : _logger = logger,
       _processManager = processManager,
       _androidSdk = androidSdk,
       _platform = platform,
       _fileSystem = fileSystem,
       _androidConsoleSocketFactory = androidConsoleSocketFactory,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager),
       super(category: Category.mobile, platformType: PlatformType.android, ephemeral: true);

  final Logger _logger;
  final ProcessManager _processManager;
  final AndroidSdk _androidSdk;
  final Platform _platform;
  final FileSystem _fileSystem;
  final ProcessUtils _processUtils;
  final AndroidConsoleSocketFactory _androidConsoleSocketFactory;

  final String? productID;
  final String modelID;
  final String? deviceCodeName;

  @override
  // Wirelessly paired Android devices should have `adb-tls-connect` in the id.
  // Source: https://android.googlesource.com/platform/packages/modules/adb/+/f4ba8d73079b99532069dbe888a58167b8723d6c/adb_mdns.h#30
  DeviceConnectionInterface get connectionInterface =>
      id.contains('adb-tls-connect')
          ? DeviceConnectionInterface.wireless
          : DeviceConnectionInterface.attached;

  late final Future<Map<String, String>> _properties = () async {
    Map<String, String> properties = <String, String>{};

    final List<String> propCommand = adbCommandForDevice(<String>['shell', 'getprop']);
    _logger.printTrace(propCommand.join(' '));

    try {
      // We pass an encoding of latin1 so that we don't try and interpret the
      // `adb shell getprop` result as UTF8.
      final ProcessResult result = await _processManager.run(
        propCommand,
        stdoutEncoding: latin1,
        stderrEncoding: latin1,
      );
      if (result.exitCode == 0 || _allowHeapCorruptionOnWindows(result.exitCode, _platform)) {
        properties = parseAdbDeviceProperties(result.stdout as String);
      } else {
        _logger.printError('Error ${result.exitCode} retrieving device properties for $name:');
        _logger.printError(result.stderr as String);
      }
    } on ProcessException catch (error) {
      _logger.printError('Error retrieving device properties for $name: $error');
    }
    return properties;
  }();

  Future<String?> _getProperty(String name) async {
    return (await _properties)[name];
  }

  @override
  late final Future<bool> isLocalEmulator = () async {
    final String? hardware = await _getProperty('ro.hardware');
    _logger.printTrace('ro.hardware = $hardware');
    if (kKnownHardware.containsKey(hardware)) {
      // Look for known hardware models.
      return kKnownHardware[hardware] == HardwareType.emulator;
    }
    // Fall back to a best-effort heuristic-based approach.
    final String? characteristics = await _getProperty('ro.build.characteristics');
    _logger.printTrace('ro.build.characteristics = $characteristics');
    return characteristics != null && characteristics.contains('emulator');
  }();

  /// The unique identifier for the emulator that corresponds to this device, or
  /// null if it is not an emulator.
  ///
  /// The ID returned matches that in the output of `flutter emulators`. Fetching
  /// this name may require connecting to the device and if an error occurs null
  /// will be returned.
  @override
  Future<String?> get emulatorId async {
    if (!(await isLocalEmulator)) {
      return null;
    }

    // Emulators always have IDs in the format emulator-(port) where port is the
    // Android Console port number.
    final RegExp emulatorPortRegex = RegExp(r'emulator-(\d+)');

    final Match? portMatch = emulatorPortRegex.firstMatch(id);
    if (portMatch == null || portMatch.groupCount < 1) {
      return null;
    }

    const String host = 'localhost';
    final int port = int.parse(portMatch.group(1)!);
    _logger.printTrace('Fetching avd name for $name via Android console on $host:$port');

    try {
      final Socket socket = await _androidConsoleSocketFactory(host, port);
      final AndroidConsole console = AndroidConsole(socket);

      try {
        await console.connect().timeout(
          const Duration(seconds: 2),
          onTimeout: () => throw TimeoutException('Connection timed out'),
        );

        return await console.getAvdName().timeout(
          const Duration(seconds: 2),
          onTimeout: () => throw TimeoutException('"avd name" timed out'),
        );
      } finally {
        console.destroy();
      }
    } on Exception catch (e) {
      _logger.printTrace('Failed to fetch avd name for emulator at $host:$port: $e');
      // If we fail to connect to the device, we should not fail so just return
      // an empty name. This data is best-effort.
      return null;
    }
  }

  @override
  late final Future<TargetPlatform> targetPlatform = () async {
    // http://developer.android.com/ndk/guides/abis.html (x86, armeabi-v7a, ...)
    switch (await _getProperty('ro.product.cpu.abi')) {
      case 'arm64-v8a':
        // Perform additional verification for 64 bit ABI. Some devices,
        // like the Kindle Fire 8, misreport the abilist. We might not
        // be able to retrieve this property, in which case we fall back
        // to assuming 64 bit.
        final String? abilist = await _getProperty('ro.product.cpu.abilist');
        if (abilist == null || abilist.contains('arm64-v8a')) {
          return TargetPlatform.android_arm64;
        } else {
          return TargetPlatform.android_arm;
        }
      case 'x86_64':
        return TargetPlatform.android_x64;
      case 'x86':
        return TargetPlatform.android_x86;
      default:
        return TargetPlatform.android_arm;
    }
  }();

  @override
  Future<bool> supportsRuntimeMode(BuildMode buildMode) async {
    switch (await targetPlatform) {
      case TargetPlatform.android_arm:
      case TargetPlatform.android_arm64:
      case TargetPlatform.android_x64:
        return buildMode != BuildMode.jitRelease;
      case TargetPlatform.android_x86:
        return buildMode == BuildMode.debug;
      case TargetPlatform.android:
      case TargetPlatform.darwin:
      case TargetPlatform.fuchsia_arm64:
      case TargetPlatform.fuchsia_x64:
      case TargetPlatform.ios:
      case TargetPlatform.linux_arm64:
      case TargetPlatform.linux_x64:
      case TargetPlatform.tester:
      case TargetPlatform.web_javascript:
      case TargetPlatform.windows_x64:
      case TargetPlatform.windows_arm64:
        throw UnsupportedError('Invalid target platform for Android');
    }
  }

  @override
  Future<String> get sdkNameAndVersion async =>
      'Android ${await _sdkVersion} (API ${await apiVersion})';

  Future<String?> get _sdkVersion => _getProperty('ro.build.version.release');

  @visibleForTesting
  Future<String?> get apiVersion => _getProperty('ro.build.version.sdk');

  AdbLogReader? _logReader;
  AdbLogReader? _pastLogReader;

  List<String> adbCommandForDevice(List<String> args) {
    return <String>[_androidSdk.adbPath!, '-s', id, ...args];
  }

  Future<RunResult> runAdbCheckedAsync(
    List<String> params, {
    String? workingDirectory,
    bool allowReentrantFlutter = false,
  }) async {
    return _processUtils.run(
      adbCommandForDevice(params),
      throwOnError: true,
      workingDirectory: workingDirectory,
      allowReentrantFlutter: allowReentrantFlutter,
      allowedFailures: (int value) => _allowHeapCorruptionOnWindows(value, _platform),
    );
  }

  bool _isValidAdbVersion(String adbVersion) {
    // Sample output: 'Android Debug Bridge version 1.0.31'
    final Match? versionFields = RegExp(r'(\d+)\.(\d+)\.(\d+)').firstMatch(adbVersion);
    if (versionFields != null) {
      final int majorVersion = int.parse(versionFields[1]!);
      final int minorVersion = int.parse(versionFields[2]!);
      final int patchVersion = int.parse(versionFields[3]!);
      if (majorVersion > 1) {
        return true;
      }
      if (majorVersion == 1 && minorVersion > 0) {
        return true;
      }
      if (majorVersion == 1 && minorVersion == 0 && patchVersion >= 39) {
        return true;
      }
      return false;
    }
    _logger.printError('Unrecognized adb version string $adbVersion. Skipping version check.');
    return true;
  }

  Future<bool> _checkForSupportedAdbVersion() async {
    final String? adbPath = _androidSdk.adbPath;
    if (adbPath == null) {
      return false;
    }

    try {
      final RunResult adbVersion = await _processUtils.run(<String>[
        adbPath,
        'version',
      ], throwOnError: true);
      if (_isValidAdbVersion(adbVersion.stdout)) {
        return true;
      }
      _logger.printError(
        'The ADB at "$adbPath" is too old; please install version 1.0.39 or later.',
      );
    } on Exception catch (error, trace) {
      _logger.printError('Error running ADB: $error', stackTrace: trace);
    }

    return false;
  }

  Future<bool> _checkForSupportedAndroidVersion() async {
    final String? adbPath = _androidSdk.adbPath;
    if (adbPath == null) {
      return false;
    }
    try {
      // If the server is automatically restarted, then we get irrelevant
      // output lines like this, which we want to ignore:
      //   adb server is out of date. killing..
      //   * daemon started successfully *
      await _processUtils.run(<String>[adbPath, 'start-server'], throwOnError: true);

      // This has been reported to return null on some devices. In this case,
      // assume the lowest supported API to still allow Flutter to run.
      // Sample output: '22'
      final String sdkVersion =
          await _getProperty('ro.build.version.sdk') ?? minApiLevel.toString();

      final int? sdkVersionParsed = int.tryParse(sdkVersion);
      if (sdkVersionParsed == null) {
        _logger.printError('Unexpected response from getprop: "$sdkVersion"');
        return false;
      }

      if (sdkVersionParsed < minApiLevel) {
        _logger.printError(
          'The Android version ($sdkVersion) on the target device is too old. Please '
          'use a $minVersionName (version $minApiLevel / $minVersionText) device or later.',
        );
        return false;
      }

      return true;
    } on Exception catch (e, stacktrace) {
      _logger.printError('Unexpected failure from adb: $e');
      _logger.printError('Stacktrace: $stacktrace');
      return false;
    }
  }

  String _getDeviceSha1Path(AndroidApk apk) {
    return '/data/local/tmp/sky.${apk.id}.sha1';
  }

  Future<String> _getDeviceApkSha1(AndroidApk apk) async {
    final RunResult result = await _processUtils.run(
      adbCommandForDevice(<String>['shell', 'cat', _getDeviceSha1Path(apk)]),
    );
    return result.stdout;
  }

  String _getSourceSha1(AndroidApk apk) {
    final File shaFile = _fileSystem.file('${apk.applicationPackage.path}.sha1');
    return shaFile.existsSync() ? shaFile.readAsStringSync() : '';
  }

  @override
  String get name => modelID;

  @override
  bool get supportsFlavors => true;

  @override
  Future<bool> isAppInstalled(ApplicationPackage app, {String? userIdentifier}) async {
    // This call takes 400ms - 600ms.
    try {
      final RunResult listOut = await runAdbCheckedAsync(<String>[
        'shell',
        'pm',
        'list',
        'packages',
        if (userIdentifier != null) ...<String>['--user', userIdentifier],
        app.id,
      ]);
      return LineSplitter.split(listOut.stdout).contains('package:${app.id}');
    } on Exception catch (error) {
      _logger.printTrace('$error');
      return false;
    }
  }

  @override
  Future<bool> isLatestBuildInstalled(covariant AndroidApk app) async {
    final String installedSha1 = await _getDeviceApkSha1(app);
    return installedSha1.isNotEmpty && installedSha1 == _getSourceSha1(app);
  }

  @override
  Future<bool> installApp(covariant AndroidApk app, {String? userIdentifier}) async {
    if (!await _adbIsValid) {
      return false;
    }
    final bool wasInstalled = await isAppInstalled(app, userIdentifier: userIdentifier);
    if (wasInstalled && await isLatestBuildInstalled(app)) {
      _logger.printTrace('Latest build already installed.');
      return true;
    }
    _logger.printTrace('Installing APK.');
    if (await _installApp(app, userIdentifier: userIdentifier)) {
      return true;
    }
    _logger.printTrace('Warning: Failed to install APK.');
    if (!wasInstalled) {
      return false;
    }
    _logger.printStatus('Uninstalling old version...');
    if (!await uninstallApp(app, userIdentifier: userIdentifier)) {
      _logger.printError('Error: Uninstalling old version failed.');
      return false;
    }
    if (!await _installApp(app, userIdentifier: userIdentifier)) {
      _logger.printError('Error: Failed to install APK again.');
      return false;
    }
    return true;
  }

  Future<bool> _installApp(AndroidApk app, {String? userIdentifier}) async {
    if (!app.applicationPackage.existsSync()) {
      _logger.printError(
        '"${_fileSystem.path.relative(app.applicationPackage.path)}" does not exist.',
      );
      return false;
    }

    final Status status = _logger.startProgress(
      'Installing ${_fileSystem.path.relative(app.applicationPackage.path)}...',
    );
    final RunResult installResult = await _processUtils.run(
      adbCommandForDevice(<String>[
        'install',
        '-t',
        '-r',
        if (userIdentifier != null) ...<String>['--user', userIdentifier],
        app.applicationPackage.path,
      ]),
    );
    status.stop();
    // Some versions of adb exit with exit code 0 even on failure :(
    // Parsing the output to check for failures.
    final RegExp failureExp = RegExp(r'^Failure.*$', multiLine: true);
    final String? failure = failureExp.stringMatch(installResult.stdout);
    if (failure != null) {
      _logger.printError('Package install error: $failure');
      return false;
    }
    if (installResult.exitCode != 0) {
      if (installResult.stderr.contains('Bad user number')) {
        _logger.printError(
          'Error: User "$userIdentifier" not found. Run "adb shell pm list users" to see list of available identifiers.',
        );
      } else {
        _logger.printError('Error: ADB exited with exit code ${installResult.exitCode}');
        _logger.printError('$installResult');
      }
      return false;
    }
    try {
      await runAdbCheckedAsync(<String>[
        'shell',
        'echo',
        '-n',
        _getSourceSha1(app),
        '>',
        _getDeviceSha1Path(app),
      ]);
    } on ProcessException catch (error) {
      _logger.printError('adb shell failed to write the SHA hash: $error.');
      return false;
    }
    return true;
  }

  @override
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier}) async {
    if (!await _adbIsValid) {
      return false;
    }

    String uninstallOut;
    try {
      final RunResult uninstallResult = await _processUtils.run(
        adbCommandForDevice(<String>[
          'uninstall',
          if (userIdentifier != null) ...<String>['--user', userIdentifier],
          app.id,
        ]),
        throwOnError: true,
      );
      uninstallOut = uninstallResult.stdout;
    } on Exception catch (error) {
      _logger.printError('adb uninstall failed: $error');
      return false;
    }
    final RegExp failureExp = RegExp(r'^Failure.*$', multiLine: true);
    final String? failure = failureExp.stringMatch(uninstallOut);
    if (failure != null) {
      _logger.printError('Package uninstall error: $failure');
      return false;
    }
    return true;
  }

  // Whether the adb and Android versions are aligned.
  late final Future<bool> _adbIsValid = () async {
    return await _checkForSupportedAdbVersion() && await _checkForSupportedAndroidVersion();
  }();

  AndroidApk? _package;

  @override
  Future<LaunchResult> startApp(
    AndroidApk? package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, Object?> platformArgs = const <String, Object>{},
    bool prebuiltApplication = false,
    bool ipv6 = false,
    String? userIdentifier,
  }) async {
    if (!await _adbIsValid) {
      return LaunchResult.failed();
    }

    final TargetPlatform devicePlatform = await targetPlatform;
    if (devicePlatform == TargetPlatform.android_x86 && !debuggingOptions.buildInfo.isDebug) {
      _logger.printError('Profile and release builds are only supported on ARM/x64 targets.');
      return LaunchResult.failed();
    }

    AndroidApk? builtPackage = package;
    AndroidArch androidArch;
    switch (devicePlatform) {
      case TargetPlatform.android_arm:
        androidArch = AndroidArch.armeabi_v7a;
      case TargetPlatform.android_arm64:
        androidArch = AndroidArch.arm64_v8a;
      case TargetPlatform.android_x64:
        androidArch = AndroidArch.x86_64;
      case TargetPlatform.android_x86:
        androidArch = AndroidArch.x86;
      case TargetPlatform.android:
      case TargetPlatform.darwin:
      case TargetPlatform.fuchsia_arm64:
      case TargetPlatform.fuchsia_x64:
      case TargetPlatform.ios:
      case TargetPlatform.linux_arm64:
      case TargetPlatform.linux_x64:
      case TargetPlatform.tester:
      case TargetPlatform.web_javascript:
      case TargetPlatform.windows_arm64:
      case TargetPlatform.windows_x64:
        _logger.printError('Android platforms are only supported.');
        return LaunchResult.failed();
    }

    if (!prebuiltApplication ||
        _androidSdk.licensesAvailable && _androidSdk.latestVersion == null) {
      _logger.printTrace('Building APK');
      final FlutterProject project = FlutterProject.current();
      await androidBuilder!.buildApk(
        project: project,
        target: mainPath ?? 'lib/main.dart',
        androidBuildInfo: AndroidBuildInfo(
          debuggingOptions.buildInfo,
          targetArchs: <AndroidArch>[androidArch],
          fastStart: debuggingOptions.fastStart,
        ),
      );
      // Package has been built, so we can get the updated application ID and
      // activity name from the .apk.
      builtPackage =
          await ApplicationPackageFactory.instance!.getPackageForPlatform(
                devicePlatform,
                buildInfo: debuggingOptions.buildInfo,
              )
              as AndroidApk?;
    }
    // There was a failure parsing the android project information.
    if (builtPackage == null) {
      throwToolExit('Problem building Android application: see above error(s).');
    }

    _logger.printTrace("Stopping app '${builtPackage.name}' on $name.");
    await stopApp(builtPackage, userIdentifier: userIdentifier);

    if (!await installApp(builtPackage, userIdentifier: userIdentifier)) {
      return LaunchResult.failed();
    }

    final bool traceStartup = platformArgs['trace-startup'] as bool? ?? false;
    ProtocolDiscovery? vmServiceDiscovery;

    if (debuggingOptions.debuggingEnabled) {
      vmServiceDiscovery = ProtocolDiscovery.vmService(
        // Avoid using getLogReader, which returns a singleton instance, because the
        // VM Service discovery will dispose at the end. creating a new logger here allows
        // logs to be surfaced normally during `flutter drive`.
        await AdbLogReader.createLogReader(this, _processManager, _logger),
        portForwarder: portForwarder,
        hostPort: debuggingOptions.hostVmServicePort,
        devicePort: debuggingOptions.deviceVmServicePort,
        ipv6: ipv6,
        logger: _logger,
      );
    }

    final String dartVmFlags = computeDartVmFlags(debuggingOptions);
    final String? traceAllowlist = debuggingOptions.traceAllowlist;
    final String? traceSkiaAllowlist = debuggingOptions.traceSkiaAllowlist;
    final String? traceToFile = debuggingOptions.traceToFile;
    final List<String> cmd = <String>[
      'shell', 'am', 'start',
      '-a', 'android.intent.action.MAIN',
      '-c', 'android.intent.category.LAUNCHER',
      '-f', '0x20000000', // FLAG_ACTIVITY_SINGLE_TOP
      if (debuggingOptions.enableDartProfiling) ...<String>[
        '--ez',
        'enable-dart-profiling',
        'true',
      ],
      if (traceStartup) ...<String>['--ez', 'trace-startup', 'true'],
      if (route != null) ...<String>['--es', 'route', route],
      if (debuggingOptions.enableSoftwareRendering) ...<String>[
        '--ez',
        'enable-software-rendering',
        'true',
      ],
      if (debuggingOptions.skiaDeterministicRendering) ...<String>[
        '--ez',
        'skia-deterministic-rendering',
        'true',
      ],
      if (debuggingOptions.traceSkia) ...<String>['--ez', 'trace-skia', 'true'],
      if (traceAllowlist != null) ...<String>['--es', 'trace-allowlist', traceAllowlist],
      if (traceSkiaAllowlist != null) ...<String>[
        '--es',
        'trace-skia-allowlist',
        traceSkiaAllowlist,
      ],
      if (debuggingOptions.traceSystrace) ...<String>['--ez', 'trace-systrace', 'true'],
      if (traceToFile != null) ...<String>['--es', 'trace-to-file', traceToFile],
      if (debuggingOptions.endlessTraceBuffer) ...<String>['--ez', 'endless-trace-buffer', 'true'],
      if (debuggingOptions.dumpSkpOnShaderCompilation) ...<String>[
        '--ez',
        'dump-skp-on-shader-compilation',
        'true',
      ],
      if (debuggingOptions.cacheSkSL) ...<String>['--ez', 'cache-sksl', 'true'],
      if (debuggingOptions.purgePersistentCache) ...<String>[
        '--ez',
        'purge-persistent-cache',
        'true',
      ],
      if (debuggingOptions.enableImpeller == ImpellerStatus.enabled) ...<String>[
        '--ez',
        'enable-impeller',
        'true',
      ],
      if (debuggingOptions.enableImpeller == ImpellerStatus.disabled) ...<String>[
        '--ez',
        'enable-impeller',
        'false',
      ],
      if (debuggingOptions.enableVulkanValidation) ...<String>[
        '--ez',
        'enable-vulkan-validation',
        'true',
      ],
      if (debuggingOptions.debuggingEnabled) ...<String>[
        if (debuggingOptions.buildInfo.isDebug) ...<String>[
          ...<String>['--ez', 'enable-checked-mode', 'true'],
          ...<String>['--ez', 'verify-entry-points', 'true'],
        ],
        if (debuggingOptions.startPaused) ...<String>['--ez', 'start-paused', 'true'],
        if (debuggingOptions.disableServiceAuthCodes) ...<String>[
          '--ez',
          'disable-service-auth-codes',
          'true',
        ],
        if (dartVmFlags.isNotEmpty) ...<String>['--es', 'dart-flags', dartVmFlags],
        if (debuggingOptions.useTestFonts) ...<String>['--ez', 'use-test-fonts', 'true'],
        if (debuggingOptions.verboseSystemLogs) ...<String>['--ez', 'verbose-logging', 'true'],
        if (userIdentifier != null) ...<String>['--user', userIdentifier],
      ],
      builtPackage.launchActivity,
    ];
    final String result = (await runAdbCheckedAsync(cmd)).stdout;
    // This invocation returns 0 even when it fails.
    if (result.contains('Error: ')) {
      _logger.printError(result.trim(), wrap: false);
      return LaunchResult.failed();
    }

    _package = builtPackage;
    if (!debuggingOptions.debuggingEnabled) {
      return LaunchResult.succeeded();
    }

    // Wait for the service protocol port here. This will complete once the
    // device has printed "VM Service is listening on...".
    _logger.printTrace('Waiting for VM Service port to be available...');
    try {
      Uri? vmServiceUri;
      if (debuggingOptions.buildInfo.isDebug || debuggingOptions.buildInfo.isProfile) {
        vmServiceUri = await vmServiceDiscovery?.uri;
        if (vmServiceUri == null) {
          _logger.printError(
            'Error waiting for a debug connection: '
            'The log reader stopped unexpectedly',
          );
          return LaunchResult.failed();
        }
      }
      return LaunchResult.succeeded(vmServiceUri: vmServiceUri);
    } on Exception catch (error) {
      _logger.printError('Error waiting for a debug connection: $error');
      return LaunchResult.failed();
    } finally {
      await vmServiceDiscovery?.cancel();
    }
  }

  @override
  bool get supportsHotReload => true;

  @override
  bool get supportsHotRestart => true;

  @override
  bool get supportsFastStart => true;

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    if (app == null) {
      return false;
    }
    final List<String> command = adbCommandForDevice(<String>[
      'shell',
      'am',
      'force-stop',
      if (userIdentifier != null) ...<String>['--user', userIdentifier],
      app.id,
    ]);
    return _processUtils
        .stream(command)
        .then<bool>(
          (int exitCode) => exitCode == 0 || _allowHeapCorruptionOnWindows(exitCode, _platform),
        );
  }

  @override
  Future<MemoryInfo> queryMemoryInfo() async {
    final AndroidApk? package = _package;
    if (package == null) {
      _logger.printError('Android package unknown, skipping dumpsys meminfo.');
      return const MemoryInfo.empty();
    }
    final RunResult runResult = await _processUtils.run(
      adbCommandForDevice(<String>['shell', 'dumpsys', 'meminfo', package.id, '-d']),
    );

    if (runResult.exitCode != 0) {
      return const MemoryInfo.empty();
    }
    return parseMeminfoDump(runResult.stdout);
  }

  @override
  void clearLogs() {
    final RunResult result = _processUtils.runSync(adbCommandForDevice(<String>['logcat', '-c']));
    // Do not log to standard error because that causes test to fail.
    if (result.exitCode != 0) {
      _logger.printTrace(
        '"adb logcat -c" failed: exitCode: ${result.exitCode}'
        ' stdout: ${result.stdout} stderr: ${result.stderr}',
      );
    }
  }

  @override
  FutureOr<DeviceLogReader> getLogReader({
    ApplicationPackage? app,
    bool includePastLogs = false,
  }) async {
    // The Android log reader isn't app-specific. The `app` parameter isn't used.
    if (includePastLogs) {
      return _pastLogReader ??= await AdbLogReader.createLogReader(
        this,
        _processManager,
        _logger,
        includePastLogs: true,
      );
    } else {
      return _logReader ??= await AdbLogReader.createLogReader(this, _processManager, _logger);
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
  }) => LogScanningVMServiceDiscoveryForAttach(
    // If it's an Android device, attaching relies on past log searching
    // to find the service protocol.
    Future<DeviceLogReader>.value(getLogReader(includePastLogs: true)),
    portForwarder: portForwarder,
    ipv6: ipv6,
    devicePort: filterDevicePort,
    hostPort: expectedHostPort,
    logger: logger,
  );

  @override
  late final DevicePortForwarder? portForwarder = () {
    final String? adbPath = _androidSdk.adbPath;
    if (adbPath == null) {
      return null;
    }
    return AndroidDevicePortForwarder(
      processManager: _processManager,
      logger: _logger,
      deviceId: id,
      adbPath: adbPath,
    );
  }();

  static final RegExp _timeRegExp = RegExp(
    r'^\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}',
    multiLine: true,
  );

  /// Return the most recent timestamp in the Android log or [null] if there is
  /// no available timestamp. The format can be passed to logcat's -T option.
  @visibleForTesting
  Future<String?> lastLogcatTimestamp() async {
    RunResult output;
    try {
      output = await runAdbCheckedAsync(<String>['shell', '-x', 'logcat', '-v', 'time', '-t', '1']);
    } on Exception catch (error) {
      _logger.printError(
        'Failed to extract the most recent timestamp from the Android log: $error.',
      );
      return null;
    }
    final Match? timeMatch = _timeRegExp.firstMatch(output.stdout);
    return timeMatch?.group(0);
  }

  @override
  bool isSupported() => true;

  @override
  bool get supportsScreenshot => true;

  @override
  Future<void> takeScreenshot(File outputFile) async {
    const String remotePath = '/data/local/tmp/flutter_screenshot.png';
    await runAdbCheckedAsync(<String>['shell', 'screencap', '-p', remotePath]);
    await _processUtils.run(
      adbCommandForDevice(<String>['pull', remotePath, outputFile.path]),
      throwOnError: true,
    );
    await runAdbCheckedAsync(<String>['shell', 'rm', remotePath]);
  }

  @override
  bool isSupportedForProject(FlutterProject flutterProject) {
    return flutterProject.android.existsSync();
  }

  @override
  Future<void> dispose() async {
    _logReader?._stop();
    _pastLogReader?._stop();
  }
}

Map<String, String> parseAdbDeviceProperties(String str) {
  final Map<String, String> properties = <String, String>{};
  final RegExp propertyExp = RegExp(r'\[(.*?)\]: \[(.*?)\]');
  for (final Match match in propertyExp.allMatches(str)) {
    properties[match.group(1)!] = match.group(2)!;
  }
  return properties;
}

/// Process the dumpsys info formatted in a table-like structure.
///
/// Currently this only pulls information from the  "App Summary" subsection.
///
/// Example output:
///
/// ```none
/// Applications Memory Usage (in Kilobytes):
/// Uptime: 441088659 Realtime: 521464097
///
/// ** MEMINFO in pid 16141 [io.flutter.demo.gallery] **
///                    Pss  Private  Private  SwapPss     Heap     Heap     Heap
///                  Total    Dirty    Clean    Dirty     Size    Alloc     Free
///                 ------   ------   ------   ------   ------   ------   ------
///   Native Heap     8648     8620        0       16    20480    12403     8076
///   Dalvik Heap      547      424       40       18     2628     1092     1536
///  Dalvik Other      464      464        0        0
///         Stack      496      496        0        0
///        Ashmem        2        0        0        0
///       Gfx dev      212      204        0        0
///     Other dev       48        0       48        0
///      .so mmap    10770      708     9372       25
///     .apk mmap      240        0        0        0
///     .ttf mmap       35        0       32        0
///     .dex mmap     2205        4     1172        0
///     .oat mmap       64        0        0        0
///     .art mmap     4228     3848       24        2
///    Other mmap    20713        4    20704        0
///     GL mtrack     2380     2380        0        0
///       Unknown    43971    43968        0        1
///         TOTAL    95085    61120    31392       62    23108    13495     9612
///
///  App Summary
///                        Pss(KB)
///                         ------
///            Java Heap:     4296
///          Native Heap:     8620
///                 Code:    11288
///                Stack:      496
///             Graphics:     2584
///        Private Other:    65228
///               System:     2573
///
///                TOTAL:    95085       TOTAL SWAP PSS:       62
///
///  Objects
///                Views:        9         ViewRootImpl:        1
///          AppContexts:        3           Activities:        1
///              Assets:        4        AssetManagers:        3
///        Local Binders:       10        Proxy Binders:       18
///        Parcel memory:        6         Parcel count:       24
///     Death Recipients:        0      OpenSSL Sockets:        0
///             WebViews:        0
///
///  SQL
///          MEMORY_USED:        0
///   PAGECACHE_OVERFLOW:        0          MALLOC_SIZE:        0
/// ...
/// ```
///
/// For more information, see https://developer.android.com/studio/command-line/dumpsys.
@visibleForTesting
AndroidMemoryInfo parseMeminfoDump(String input) {
  final AndroidMemoryInfo androidMemoryInfo = AndroidMemoryInfo();

  final List<String> lines = input.split('\n');

  final String timelineData = lines.firstWhere(
    (String line) => line.startsWith('${AndroidMemoryInfo._kUpTimeKey}: '),
  );
  final List<String> times = timelineData.trim().split('${AndroidMemoryInfo._kRealTimeKey}:');
  androidMemoryInfo.realTime = int.tryParse(times.last.trim()) ?? 0;

  lines
      .skipWhile((String line) => !line.contains('App Summary'))
      .takeWhile((String line) => !line.contains('TOTAL'))
      .where((String line) => line.contains(':'))
      .forEach((String line) {
        final List<String> sections = line.trim().split(':');
        final String key = sections.first.trim();
        final int value = int.tryParse(sections.last.trim()) ?? 0;
        switch (key) {
          case AndroidMemoryInfo._kJavaHeapKey:
            androidMemoryInfo.javaHeap = value;
          case AndroidMemoryInfo._kNativeHeapKey:
            androidMemoryInfo.nativeHeap = value;
          case AndroidMemoryInfo._kCodeKey:
            androidMemoryInfo.code = value;
          case AndroidMemoryInfo._kStackKey:
            androidMemoryInfo.stack = value;
          case AndroidMemoryInfo._kGraphicsKey:
            androidMemoryInfo.graphics = value;
          case AndroidMemoryInfo._kPrivateOtherKey:
            androidMemoryInfo.privateOther = value;
          case AndroidMemoryInfo._kSystemKey:
            androidMemoryInfo.system = value;
        }
      });
  return androidMemoryInfo;
}

/// Android specific implementation of memory info.
class AndroidMemoryInfo extends MemoryInfo {
  static const String _kUpTimeKey = 'Uptime';
  static const String _kRealTimeKey = 'Realtime';
  static const String _kJavaHeapKey = 'Java Heap';
  static const String _kNativeHeapKey = 'Native Heap';
  static const String _kCodeKey = 'Code';
  static const String _kStackKey = 'Stack';
  static const String _kGraphicsKey = 'Graphics';
  static const String _kPrivateOtherKey = 'Private Other';
  static const String _kSystemKey = 'System';
  static const String _kTotalKey = 'Total';

  // Realtime is time since the system was booted includes deep sleep. Clock
  // is monotonic, and ticks even when the CPU is in power saving modes.
  int realTime = 0;

  // Each measurement has KB as a unit.
  int javaHeap = 0;
  int nativeHeap = 0;
  int code = 0;
  int stack = 0;
  int graphics = 0;
  int privateOther = 0;
  int system = 0;

  @override
  Map<String, Object> toJson() {
    return <String, Object>{
      'platform': 'Android',
      _kRealTimeKey: realTime,
      _kJavaHeapKey: javaHeap,
      _kNativeHeapKey: nativeHeap,
      _kCodeKey: code,
      _kStackKey: stack,
      _kGraphicsKey: graphics,
      _kPrivateOtherKey: privateOther,
      _kSystemKey: system,
      _kTotalKey: javaHeap + nativeHeap + code + stack + graphics + privateOther + system,
    };
  }
}

/// A log reader that logs from `adb logcat`.
class AdbLogReader extends DeviceLogReader {
  AdbLogReader._(this._adbProcess, this.name, this._logger);

  @visibleForTesting
  factory AdbLogReader.test(Process adbProcess, String name, Logger logger) = AdbLogReader._;

  /// Create a new [AdbLogReader] from an [AndroidDevice] instance.
  static Future<AdbLogReader> createLogReader(
    AndroidDevice device,
    ProcessManager processManager,
    Logger logger, {
    bool includePastLogs = false,
  }) async {
    // logcat -T is not supported on Android releases before Lollipop.
    const int kLollipopVersionCode = 21;
    final int? apiVersion = (String? v) {
      // If the API version string isn't found, conservatively assume that the
      // version is less recent than the one we're looking for.
      return v == null ? kLollipopVersionCode - 1 : int.tryParse(v);
    }(await device.apiVersion);

    // Start the adb logcat process and filter the most recent logs since `lastTimestamp`.
    // Some devices (notably LG) will only output logcat via shell
    // https://github.com/flutter/flutter/issues/51853
    final List<String> args = <String>['shell', '-x', 'logcat', '-v', 'time'];

    // If past logs are included then filter for 'flutter' logs only.
    if (includePastLogs) {
      args.addAll(<String>['-s', 'flutter']);
    } else if (apiVersion != null && apiVersion >= kLollipopVersionCode) {
      // Otherwise, filter for logs appearing past the present.
      // '-T 0` means the timestamp of the logcat command invocation.
      final String? lastLogcatTimestamp = await device.lastLogcatTimestamp();
      args.addAll(<String>[
        '-T',
        if (lastLogcatTimestamp != null) "'$lastLogcatTimestamp'" else '0',
      ]);
    }
    final Process process = await processManager.start(device.adbCommandForDevice(args));
    return AdbLogReader._(process, device.name, logger);
  }

  int? _appPid;

  final Process _adbProcess;

  final Logger _logger;

  @override
  final String name;

  late final StreamController<String> _linesController = StreamController<String>.broadcast(
    onListen: _start,
    onCancel: _stop,
  );

  @override
  Stream<String> get logLines => _linesController.stream;

  @override
  Future<void> provideVmService(FlutterVmService connectedVmService) async {
    final VM? vm = await connectedVmService.getVmGuarded();
    if (vm == null) {
      _logger.printError(
        'An error occurred when setting up filtering for adb logs. '
        'Unable to communicate with the VM service.',
      );
    } else {
      _appPid = vm.pid;
    }
  }

  void _start() {
    // We expect logcat streams to occasionally contain invalid utf-8,
    // see: https://github.com/flutter/flutter/pull/8864.
    const Utf8Decoder decoder = Utf8Decoder(reportErrors: false);
    _adbProcess.stdout
        .transform<String>(decoder)
        .transform<String>(const LineSplitter())
        .listen(_onLine);
    _adbProcess.stderr
        .transform<String>(decoder)
        .transform<String>(const LineSplitter())
        .listen(_onLine);
    unawaited(
      _adbProcess.exitCode.whenComplete(() {
        if (_linesController.hasListener) {
          _linesController.close();
        }
      }),
    );
  }

  // 'W/ActivityManager(pid): '
  static final RegExp _logFormat = RegExp(r'^[VDIWEF]\/.*?\(\s*(\d+)\):\s');

  static final List<RegExp> _allowedTags = <RegExp>[
    RegExp(r'^[VDIWEF]\/flutter[^:]*:\s+', caseSensitive: false),
    RegExp(r'^[IE]\/DartVM[^:]*:\s+'),
    RegExp(r'^[WEF]\/AndroidRuntime:\s+'),
    RegExp(r'^[WEF]\/AndroidRuntime\([0-9]+\):\s+'),
    RegExp(r'^[WEF]\/ActivityManager:\s+.*(\bflutter\b|\bdomokit\b|\bsky\b)'),
    RegExp(r'^[WEF]\/System\.err:\s+'),
    RegExp(r'^[F]\/[\S^:]+:\s+'),
  ];

  // E/SurfaceSyncer(22636): Failed to find sync for id=9
  // Some versions of Android spew this out. It is inactionable to the end user
  // and causes no problems for the application.
  static final RegExp _surfaceSyncerSpam = RegExp(
    r'^E/SurfaceSyncer\(\s*\d+\): Failed to find sync for id=\d+',
  );

  // 'F/libc(pid): Fatal signal 11'
  static final RegExp _fatalLog = RegExp(r'^F\/libc\s*\(\s*\d+\):\sFatal signal (\d+)');

  // 'I/DEBUG(pid): ...'
  static final RegExp _tombstoneLine = RegExp(r'^[IF]\/DEBUG\s*\(\s*\d+\):\s(.+)$');

  // 'I/DEBUG(pid): Tombstone written to: '
  static final RegExp _tombstoneTerminator = RegExp(r'^Tombstone written to:\s');

  // we default to true in case none of the log lines match
  bool _acceptedLastLine = true;

  // Whether a fatal crash is happening or not.
  // During a fatal crash only lines from the crash are accepted, the rest are
  // dropped.
  bool _fatalCrash = false;

  // The format of the line is controlled by the '-v' parameter passed to
  // adb logcat. We are currently passing 'time', which has the format:
  // mm-dd hh:mm:ss.milliseconds Priority/Tag( PID): ....
  void _onLine(String line) {
    // This line might be processed after the subscription is closed but before
    // adb stops streaming logs.
    if (_linesController.isClosed) {
      return;
    }
    final Match? timeMatch = AndroidDevice._timeRegExp.firstMatch(line);
    if (timeMatch == null || line.length == timeMatch.end) {
      _acceptedLastLine = false;
      return;
    }
    // Chop off the time.
    line = line.substring(timeMatch.end + 1);
    final Match? logMatch = _logFormat.firstMatch(line);
    if (logMatch != null) {
      bool acceptLine = false;

      if (_fatalCrash) {
        // While a fatal crash is going on, only accept lines from the crash
        // Otherwise the crash log in the console may get interrupted

        final Match? fatalMatch = _tombstoneLine.firstMatch(line);

        if (fatalMatch != null) {
          acceptLine = true;

          line = fatalMatch[1]!;

          if (_tombstoneTerminator.hasMatch(line)) {
            // Hit crash terminator, stop logging the crash info
            _fatalCrash = false;
          }
        }
      } else if (_appPid != null && int.parse(logMatch.group(1)!) == _appPid) {
        acceptLine = !_surfaceSyncerSpam.hasMatch(line);

        if (_fatalLog.hasMatch(line)) {
          // Hit fatal signal, app is now crashing
          _fatalCrash = true;
        }
      } else {
        // Filter on approved names and levels.
        acceptLine = _allowedTags.any((RegExp re) => re.hasMatch(line));
      }

      if (acceptLine) {
        _acceptedLastLine = true;
        _linesController.add(line);
        return;
      }
      _acceptedLastLine = false;
    } else if (line == '--------- beginning of system' || line == '--------- beginning of main') {
      // hide the ugly adb logcat log boundaries at the start
      _acceptedLastLine = false;
    } else {
      // If it doesn't match the log pattern at all, then pass it through if we
      // passed the last matching line through. It might be a multiline message.
      if (_acceptedLastLine) {
        _linesController.add(line);
        return;
      }
    }
  }

  void _stop() {
    _linesController.close();
    _adbProcess.kill();
  }

  @override
  void dispose() {
    _stop();
  }
}

/// A [DevicePortForwarder] implemented for Android devices that uses adb.
class AndroidDevicePortForwarder extends DevicePortForwarder {
  AndroidDevicePortForwarder({
    required ProcessManager processManager,
    required Logger logger,
    required String deviceId,
    required String adbPath,
  }) : _deviceId = deviceId,
       _adbPath = adbPath,
       _logger = logger,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager);

  final String _deviceId;
  final String _adbPath;
  final Logger _logger;
  final ProcessUtils _processUtils;

  static int? _extractPort(String portString) {
    return int.tryParse(portString.trim());
  }

  @override
  List<ForwardedPort> get forwardedPorts {
    final List<ForwardedPort> ports = <ForwardedPort>[];

    String stdout;
    try {
      stdout =
          _processUtils
              .runSync(<String>[_adbPath, '-s', _deviceId, 'forward', '--list'], throwOnError: true)
              .stdout
              .trim();
    } on ProcessException catch (error) {
      _logger.printError('Failed to list forwarded ports: $error.');
      return ports;
    }

    final List<String> lines = LineSplitter.split(stdout).toList();
    for (final String line in lines) {
      if (!line.startsWith(_deviceId)) {
        continue;
      }
      final List<String> splitLine = line.split('tcp:');

      // Sanity check splitLine.
      if (splitLine.length != 3) {
        continue;
      }

      // Attempt to extract ports.
      final int? hostPort = _extractPort(splitLine[1]);
      final int? devicePort = _extractPort(splitLine[2]);

      // Failed, skip.
      if (hostPort == null || devicePort == null) {
        continue;
      }

      ports.add(ForwardedPort(hostPort, devicePort));
    }

    return ports;
  }

  @override
  Future<int> forward(int devicePort, {int? hostPort}) async {
    hostPort ??= 0;
    final RunResult process = await _processUtils.run(<String>[
      _adbPath,
      '-s',
      _deviceId,
      'forward',
      'tcp:$hostPort',
      'tcp:$devicePort',
    ], throwOnError: true);

    if (process.stderr.isNotEmpty) {
      process.throwException('adb returned error:\n${process.stderr}');
    }

    if (process.exitCode != 0) {
      if (process.stdout.isNotEmpty) {
        process.throwException('adb returned error:\n${process.stdout}');
      }
      process.throwException('adb failed without a message');
    }

    if (hostPort == 0) {
      if (process.stdout.isEmpty) {
        process.throwException('adb did not report forwarded port');
      }
      hostPort = int.tryParse(process.stdout);
      if (hostPort == null) {
        process.throwException('adb returned invalid port number:\n${process.stdout}');
      }
    } else {
      // stdout may be empty or the port we asked it to forward, though it's
      // not documented (or obvious) what triggers each case.
      //
      // Observations are:
      //   - On MacOS it's always empty when Flutter spawns the process, but
      //   - On MacOS it prints the port number when run from the terminal, unless
      //     the port is already forwarded, when it also prints nothing.
      //   - On ChromeOS, the port appears to be printed even when Flutter spawns
      //     the process
      //
      // To cover all cases, we accept the output being either empty or exactly
      // the port number, but treat any other output as probably being an error
      // message.
      if (process.stdout.isNotEmpty && process.stdout.trim() != '$hostPort') {
        process.throwException('adb returned error:\n${process.stdout}');
      }
    }

    return hostPort!;
  }

  @override
  Future<void> unforward(ForwardedPort forwardedPort) async {
    final String tcpLine = 'tcp:${forwardedPort.hostPort}';
    final RunResult runResult = await _processUtils.run(<String>[
      _adbPath,
      '-s',
      _deviceId,
      'forward',
      '--remove',
      tcpLine,
    ]);
    if (runResult.exitCode == 0) {
      return;
    }
    _logger.printError('Failed to unforward port: $runResult');
  }

  @override
  Future<void> dispose() async {
    for (final ForwardedPort port in forwardedPorts) {
      await unforward(port);
    }
  }
}

// In platform tools 29.0.0 adb.exe seems to be ending with this heap
// corruption error code on seemingly successful termination. Ignore
// this error on windows.
bool _allowHeapCorruptionOnWindows(int exitCode, Platform platform) {
  return exitCode == -1073740940 && platform.isWindows;
}
