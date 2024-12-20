// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:meta/meta.dart';
import 'package:package_config/package_config_types.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../application_package.dart';
import '../base/common.dart';
import '../base/dds.dart';
import '../base/logger.dart';
import '../base/process.dart';
import '../build_info.dart';
import '../device.dart';
import '../resident_runner.dart';
import '../sksl_writer.dart';
import '../vmservice.dart';
import 'web_driver_service.dart';

class FlutterDriverFactory {
  FlutterDriverFactory({
    required ApplicationPackageFactory applicationPackageFactory,
    required Logger logger,
    required ProcessUtils processUtils,
    required String dartSdkPath,
    required DevtoolsLauncher devtoolsLauncher,
  }) : _applicationPackageFactory = applicationPackageFactory,
       _logger = logger,
       _processUtils = processUtils,
       _dartSdkPath = dartSdkPath,
       _devtoolsLauncher = devtoolsLauncher;

  final ApplicationPackageFactory _applicationPackageFactory;
  final Logger _logger;
  final ProcessUtils _processUtils;
  final String _dartSdkPath;
  final DevtoolsLauncher _devtoolsLauncher;

  /// Create a driver service for running `flutter drive`.
  DriverService createDriverService(bool web) {
    if (web) {
      return WebDriverService(
        logger: _logger,
        processUtils: _processUtils,
        dartSdkPath: _dartSdkPath,
      );
    }
    return FlutterDriverService(
      logger: _logger,
      processUtils: _processUtils,
      dartSdkPath: _dartSdkPath,
      applicationPackageFactory: _applicationPackageFactory,
      devtoolsLauncher: _devtoolsLauncher,
    );
  }
}

/// An interface for the `flutter driver` integration test operations.
abstract class DriverService {
  /// Install and launch the application for the provided [device].
  Future<void> start(
    BuildInfo buildInfo,
    Device device,
    DebuggingOptions debuggingOptions, {
    File? applicationBinary,
    String? route,
    String? userIdentifier,
    String? mainPath,
    Map<String, Object> platformArgs = const <String, Object>{},
  });

  /// If --use-existing-app is provided, configured the correct VM Service URI.
  Future<void> reuseApplication(Uri vmServiceUri, Device device, DebuggingOptions debuggingOptions);

  /// Start the test file with the provided [arguments] and [environment], returning
  /// the test process exit code.
  ///
  /// if [profileMemory] is provided, it will be treated as a file path to write a
  /// devtools memory profile.
  Future<int> startTest(
    String testFile,
    List<String> arguments,
    Map<String, String> environment,
    PackageConfig packageConfig, {
    bool? headless,
    String? chromeBinary,
    String? browserName,
    bool? androidEmulator,
    int? driverPort,
    List<String> webBrowserFlags,
    List<String>? browserDimension,
    String? profileMemory,
  });

  /// Stop the running application and uninstall it from the device.
  ///
  /// If [writeSkslOnExit] is non-null, will connect to the VM Service
  /// and write SkSL to the file. This is only supported on mobile and
  /// desktop devices.
  Future<void> stop({File? writeSkslOnExit, String? userIdentifier});
}

/// An implementation of the driver service that connects to mobile and desktop
/// applications.
class FlutterDriverService extends DriverService {
  FlutterDriverService({
    required ApplicationPackageFactory applicationPackageFactory,
    required Logger logger,
    required ProcessUtils processUtils,
    required String dartSdkPath,
    required DevtoolsLauncher devtoolsLauncher,
    @visibleForTesting VMServiceConnector vmServiceConnector = connectToVmService,
  }) : _applicationPackageFactory = applicationPackageFactory,
       _logger = logger,
       _processUtils = processUtils,
       _dartSdkPath = dartSdkPath,
       _vmServiceConnector = vmServiceConnector,
       _devtoolsLauncher = devtoolsLauncher;

  static const int _kLaunchAttempts = 3;

  final ApplicationPackageFactory _applicationPackageFactory;
  final Logger _logger;
  final ProcessUtils _processUtils;
  final String _dartSdkPath;
  final VMServiceConnector _vmServiceConnector;
  final DevtoolsLauncher _devtoolsLauncher;

  Device? _device;
  ApplicationPackage? _applicationPackage;
  late String _vmServiceUri;
  late FlutterVmService _vmService;

  @override
  Future<void> start(
    BuildInfo buildInfo,
    Device device,
    DebuggingOptions debuggingOptions, {
    File? applicationBinary,
    String? route,
    String? userIdentifier,
    Map<String, Object> platformArgs = const <String, Object>{},
    String? mainPath,
  }) async {
    if (buildInfo.isRelease) {
      throwToolExit(
        'Flutter Driver (non-web) does not support running in release mode.\n'
        '\n'
        'Use --profile mode for testing application performance.\n'
        'Use --debug (default) mode for testing correctness (with assertions).',
      );
    }
    _device = device;
    final TargetPlatform targetPlatform = await device.targetPlatform;
    _applicationPackage = await _applicationPackageFactory.getPackageForPlatform(
      targetPlatform,
      buildInfo: buildInfo,
      applicationBinary: applicationBinary,
    );
    int attempt = 0;
    LaunchResult? result;
    bool prebuiltApplication = applicationBinary != null;
    while (attempt < _kLaunchAttempts) {
      result = await device.startApp(
        _applicationPackage,
        mainPath: mainPath,
        route: route,
        debuggingOptions: debuggingOptions,
        platformArgs: platformArgs,
        userIdentifier: userIdentifier,
        prebuiltApplication: prebuiltApplication,
      );
      if (result.started) {
        break;
      }
      // On attempts past 1, assume the application is built correctly and re-use it.
      attempt += 1;
      prebuiltApplication = true;
      _logger.printError('Application failed to start on attempt: $attempt');
    }
    if (result == null || !result.started) {
      throwToolExit('Application failed to start. Will not run test. Quitting.', exitCode: 1);
    }
    return reuseApplication(result.vmServiceUri!, device, debuggingOptions);
  }

  @override
  Future<void> reuseApplication(
    Uri vmServiceUri,
    Device device,
    DebuggingOptions debuggingOptions,
  ) async {
    Uri uri;
    if (vmServiceUri.scheme == 'ws') {
      final List<String> segments = vmServiceUri.pathSegments.toList();
      segments.remove('ws');
      uri = vmServiceUri.replace(scheme: 'http', path: segments.join('/'));
    } else {
      uri = vmServiceUri;
    }
    _vmServiceUri = uri.toString();
    _device = device;
    if (debuggingOptions.enableDds) {
      try {
        await device.dds.startDartDevelopmentServiceFromDebuggingOptions(
          uri,
          debuggingOptions: debuggingOptions,
        );
        _vmServiceUri = device.dds.uri.toString();
      } on DartDevelopmentServiceException {
        // If there's another flutter_tools instance still connected to the target
        // application, DDS will already be running remotely and this call will fail.
        // This can be ignored to continue to use the existing remote DDS instance.
      }
    }
    _vmService = await _vmServiceConnector(uri, device: _device, logger: _logger);
    final DeviceLogReader logReader = await device.getLogReader(app: _applicationPackage);
    logReader.logLines.listen(_logger.printStatus);
    await logReader.provideVmService(_vmService);
  }

  @override
  Future<int> startTest(
    String testFile,
    List<String> arguments,
    Map<String, String> environment,
    PackageConfig packageConfig, {
    bool? headless,
    String? chromeBinary,
    String? browserName,
    bool? androidEmulator,
    int? driverPort,
    List<String> webBrowserFlags = const <String>[],
    List<String>? browserDimension,
    String? profileMemory,
  }) async {
    if (profileMemory != null) {
      unawaited(
        _devtoolsLauncher.launch(
          Uri.parse(_vmServiceUri),
          additionalArguments: <String>['--record-memory-profile=$profileMemory'],
        ),
      );
      // When profiling memory the original launch future will never complete.
      await _devtoolsLauncher.processStart;
    }
    try {
      final int result = await _processUtils.stream(
        <String>[_dartSdkPath, ...arguments, testFile],
        environment: <String, String>{'VM_SERVICE_URL': _vmServiceUri, ...environment},
      );
      return result;
    } finally {
      if (profileMemory != null) {
        await _devtoolsLauncher.close();
      }
    }
  }

  @override
  Future<void> stop({File? writeSkslOnExit, String? userIdentifier}) async {
    if (writeSkslOnExit != null) {
      final FlutterView flutterView = (await _vmService.getFlutterViews()).first;
      final Map<String, Object?>? result = await _vmService.getSkSLs(viewId: flutterView.id);
      await sharedSkSlWriter(_device!, result, outputFile: writeSkslOnExit, logger: _logger);
    }
    // If the application package is available, stop and uninstall.
    final ApplicationPackage? package = _applicationPackage;
    if (package != null) {
      if (!await _device!.stopApp(package, userIdentifier: userIdentifier)) {
        _logger.printError('Failed to stop app');
      }
      if (!await _device!.uninstallApp(package, userIdentifier: userIdentifier)) {
        _logger.printError('Failed to uninstall app');
      }
    } else if (_device!.supportsFlutterExit) {
      // Otherwise use the VM Service URI to stop the app as a best effort approach.
      final vm_service.VM vm = await _vmService.service.getVM();
      final vm_service.IsolateRef isolateRef = vm.isolates!.firstWhere((
        vm_service.IsolateRef element,
      ) {
        return !element.isSystemIsolate!;
      });
      unawaited(_vmService.flutterExit(isolateId: isolateRef.id!));
    } else {
      _logger.printTrace('No application package for $_device, leaving app running');
    }
    await _device!.dispose();
  }
}
