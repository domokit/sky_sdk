// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import 'application_package.dart';
import 'artifacts.dart';
import 'asset.dart';
import 'base/command_help.dart';
import 'base/common.dart';
import 'base/context.dart';
import 'base/dds.dart';
import 'base/file_system.dart';
import 'base/io.dart' as io;
import 'base/logger.dart';
import 'base/platform.dart';
import 'base/signals.dart';
import 'base/terminal.dart';
import 'base/utils.dart';
import 'build_info.dart';
import 'build_system/build_system.dart';
import 'build_system/tools/shader_compiler.dart';
import 'bundle.dart';
import 'cache.dart';
import 'compile.dart';
import 'convert.dart';
import 'devfs.dart';
import 'device.dart';
import 'globals.dart' as globals;
import 'ios/application_package.dart';
import 'ios/devices.dart';
import 'project.dart';
import 'resident_devtools_handler.dart';
import 'run_cold.dart';
import 'run_hot.dart';
import 'vmservice.dart';
import 'web/chrome.dart';

class FlutterDevice {
  FlutterDevice(
    this.device, {
    required this.buildInfo,
    TargetModel targetModel = TargetModel.flutter,
    this.targetPlatform,
    ResidentCompiler? generator,
    this.userIdentifier,
    required this.developmentShaderCompiler,
  }) : generator =
           generator ??
           ResidentCompiler(
             globals.artifacts!.getArtifactPath(
               Artifact.flutterPatchedSdkPath,
               platform: targetPlatform,
               mode: buildInfo.mode,
             ),
             buildMode: buildInfo.mode,
             trackWidgetCreation: buildInfo.trackWidgetCreation,
             fileSystemRoots: buildInfo.fileSystemRoots,
             fileSystemScheme: buildInfo.fileSystemScheme,
             targetModel: targetModel,
             dartDefines: buildInfo.dartDefines,
             packagesPath: buildInfo.packageConfigPath,
             frontendServerStarterPath: buildInfo.frontendServerStarterPath,
             extraFrontEndOptions: buildInfo.extraFrontEndOptions,
             artifacts: globals.artifacts!,
             processManager: globals.processManager,
             logger: globals.logger,
             platform: globals.platform,
             fileSystem: globals.fs,
           );

  /// Create a [FlutterDevice] with optional code generation enabled.
  static Future<FlutterDevice> create(
    Device device, {
    required String? target,
    required BuildInfo buildInfo,
    required Platform platform,
    TargetModel targetModel = TargetModel.flutter,
    List<String>? experimentalFlags,
    String? userIdentifier,
  }) async {
    final TargetPlatform targetPlatform = await device.targetPlatform;
    if (device.platformType == PlatformType.fuchsia) {
      targetModel = TargetModel.flutterRunner;
    }
    final DevelopmentShaderCompiler shaderCompiler = DevelopmentShaderCompiler(
      shaderCompiler: ShaderCompiler(
        artifacts: globals.artifacts!,
        logger: globals.logger,
        processManager: globals.processManager,
        fileSystem: globals.fs,
      ),
      fileSystem: globals.fs,
    );

    final ResidentCompiler generator;

    // For both web and non-web platforms we initialize dill to/from
    // a shared location for faster bootstrapping. If the compiler fails
    // due to a kernel target or version mismatch, no error is reported
    // and the compiler starts up as normal. Unexpected errors will print
    // a warning message and dump some debug information which can be
    // used to file a bug, but the compiler will still start up correctly.
    if (targetPlatform == TargetPlatform.web_javascript) {
      // TODO(zanderso): consistently provide these flags across platforms.
      const String platformDillName = 'ddc_outline.dill';

      final String platformDillPath = globals.fs.path.join(
        globals.artifacts!.getHostArtifact(HostArtifact.webPlatformKernelFolder).path,
        platformDillName,
      );

      generator = ResidentCompiler(
        globals.artifacts!.getHostArtifact(HostArtifact.flutterWebSdk).path,
        buildMode: buildInfo.mode,
        trackWidgetCreation: buildInfo.trackWidgetCreation,
        fileSystemRoots: buildInfo.fileSystemRoots,
        // Override the filesystem scheme so that the frontend_server can find
        // the generated entrypoint code.
        fileSystemScheme: 'org-dartlang-app',
        initializeFromDill:
            buildInfo.initializeFromDill ??
            getDefaultCachedKernelPath(
              trackWidgetCreation: buildInfo.trackWidgetCreation,
              dartDefines: buildInfo.dartDefines,
              extraFrontEndOptions: buildInfo.extraFrontEndOptions,
            ),
        assumeInitializeFromDillUpToDate: buildInfo.assumeInitializeFromDillUpToDate,
        targetModel: TargetModel.dartdevc,
        frontendServerStarterPath: buildInfo.frontendServerStarterPath,
        extraFrontEndOptions: buildInfo.extraFrontEndOptions,
        platformDill: globals.fs.file(platformDillPath).absolute.uri.toString(),
        dartDefines: buildInfo.dartDefines,
        librariesSpec:
            globals.fs
                .file(globals.artifacts!.getHostArtifact(HostArtifact.flutterWebLibrariesJson))
                .uri
                .toString(),
        packagesPath: buildInfo.packageConfigPath,
        artifacts: globals.artifacts!,
        processManager: globals.processManager,
        logger: globals.logger,
        fileSystem: globals.fs,
        platform: platform,
      );
    } else {
      List<String> extraFrontEndOptions = buildInfo.extraFrontEndOptions;
      extraFrontEndOptions = <String>[
        '--enable-experiment=alternative-invalidation-strategy',
        ...extraFrontEndOptions,
      ];
      generator = ResidentCompiler(
        globals.artifacts!.getArtifactPath(
          Artifact.flutterPatchedSdkPath,
          platform: targetPlatform,
          mode: buildInfo.mode,
        ),
        buildMode: buildInfo.mode,
        trackWidgetCreation: buildInfo.trackWidgetCreation,
        fileSystemRoots: buildInfo.fileSystemRoots,
        fileSystemScheme: buildInfo.fileSystemScheme,
        targetModel: targetModel,
        dartDefines: buildInfo.dartDefines,
        frontendServerStarterPath: buildInfo.frontendServerStarterPath,
        extraFrontEndOptions: extraFrontEndOptions,
        initializeFromDill:
            buildInfo.initializeFromDill ??
            getDefaultCachedKernelPath(
              trackWidgetCreation: buildInfo.trackWidgetCreation,
              dartDefines: buildInfo.dartDefines,
              extraFrontEndOptions: extraFrontEndOptions,
            ),
        assumeInitializeFromDillUpToDate: buildInfo.assumeInitializeFromDillUpToDate,
        packagesPath: buildInfo.packageConfigPath,
        artifacts: globals.artifacts!,
        processManager: globals.processManager,
        logger: globals.logger,
        platform: platform,
        fileSystem: globals.fs,
      );
    }

    return FlutterDevice(
      device,
      targetModel: targetModel,
      targetPlatform: targetPlatform,
      generator: generator,
      buildInfo: buildInfo,
      userIdentifier: userIdentifier,
      developmentShaderCompiler: shaderCompiler,
    );
  }

  final TargetPlatform? targetPlatform;
  final Device? device;
  final ResidentCompiler? generator;
  final BuildInfo buildInfo;
  final String? userIdentifier;
  final DevelopmentShaderCompiler developmentShaderCompiler;

  DevFSWriter? devFSWriter;
  Stream<Uri?>? vmServiceUris;
  FlutterVmService? vmService;
  DevFS? devFS;
  ApplicationPackage? package;
  StreamSubscription<String>? _loggingSubscription;
  bool? _isListeningForVmServiceUri;

  /// Whether the stream [vmServiceUris] is still open.
  bool get isWaitingForVmService => _isListeningForVmServiceUri ?? false;

  /// If the [reloadSources] parameter is not null the 'reloadSources' service
  /// will be registered.
  /// The 'reloadSources' service can be used by other Service Protocol clients
  /// connected to the VM (e.g. VmService) to request a reload of the source
  /// code of the running application (a.k.a. HotReload).
  /// The 'compileExpression' service can be used to compile user-provided
  /// expressions requested during debugging of the application.
  /// This ensures that the reload process follows the normal orchestration of
  /// the Flutter Tools and not just the VM internal service.
  Future<void> connect({
    ReloadSources? reloadSources,
    Restart? restart,
    CompileExpression? compileExpression,
    PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
    required DebuggingOptions debuggingOptions,
    int? hostVmServicePort,
    required bool allowExistingDdsInstance,
  }) {
    final Completer<void> completer = Completer<void>();
    late StreamSubscription<void> subscription;
    bool isWaitingForVm = false;

    subscription = vmServiceUris!.listen(
      (Uri? vmServiceUri) async {
        // FYI, this message is used as a sentinel in tests.
        globals.printTrace('Connecting to service protocol: $vmServiceUri');
        isWaitingForVm = true;
        bool existingDds = false;
        FlutterVmService? service;
        if (debuggingOptions.enableDds) {
          void handleError(Exception e, StackTrace st) {
            globals.printTrace('Fail to connect to service protocol: $vmServiceUri: $e');
            if (!completer.isCompleted) {
              completer.completeError('failed to connect to $vmServiceUri $e', st);
            }
          }

          // First check if the VM service is actually listening on vmServiceUri as
          // this may not be the case when scraping logcat for URIs. If this URI is
          // from an old application instance, we shouldn't try and start DDS.
          try {
            service = await connectToVmService(vmServiceUri!, logger: globals.logger);
            await service.dispose();
          } on Exception catch (exception) {
            globals.printTrace('Fail to connect to service protocol: $vmServiceUri: $exception');
            if (!completer.isCompleted && !_isListeningForVmServiceUri!) {
              completer.completeError('failed to connect to $vmServiceUri $exception');
            }
            return;
          }

          // This first try block is meant to catch errors that occur during DDS startup
          // (e.g., failure to bind to a port, failure to connect to the VM service,
          // attaching to a VM service with existing clients, etc.).
          try {
            await device!.dds.startDartDevelopmentServiceFromDebuggingOptions(
              vmServiceUri,
              debuggingOptions: debuggingOptions,
            );
          } on DartDevelopmentServiceException catch (e, st) {
            if (!allowExistingDdsInstance ||
                (e.errorCode != DartDevelopmentServiceException.existingDdsInstanceError)) {
              handleError(e, st);
              return;
            } else {
              existingDds = true;
            }
          } on ToolExit {
            rethrow;
          } on Exception catch (e, st) {
            handleError(e, st);
            return;
          }
        }
        // This second try block handles cases where the VM service connection goes down
        // before flutter_tools connects to DDS. The DDS `done` future completes when DDS
        // shuts down, including after an error. If `done` completes before `connectToVmService`,
        // something went wrong that caused DDS to shutdown early.
        try {
          service =
              await Future.any<dynamic>(<Future<dynamic>>[
                    connectToVmService(
                      debuggingOptions.enableDds
                          ? (device!.dds.uri ?? vmServiceUri!)
                          : vmServiceUri!,
                      reloadSources: reloadSources,
                      restart: restart,
                      compileExpression: compileExpression,
                      flutterProject: FlutterProject.current(),
                      printStructuredErrorLogMethod: printStructuredErrorLogMethod,
                      device: device,
                      logger: globals.logger,
                    ),
                    if (!existingDds)
                      device!.dds.done.whenComplete(
                        () => throw Exception('DDS shut down too early'),
                      ),
                  ])
                  as FlutterVmService?;
        } on Exception catch (exception) {
          globals.printTrace('Fail to connect to service protocol: $vmServiceUri: $exception');
          if (!completer.isCompleted && !_isListeningForVmServiceUri!) {
            completer.completeError('failed to connect to $vmServiceUri $exception');
          }
          return;
        }
        if (completer.isCompleted) {
          return;
        }
        globals.printTrace('Successfully connected to service protocol: $vmServiceUri');

        vmService = service;
        await (await device!.getLogReader(app: package)).provideVmService(vmService!);
        completer.complete();
        await subscription.cancel();
      },
      onError: (dynamic error) {
        globals.printTrace('Fail to handle VM Service URI: $error');
      },
      onDone: () {
        _isListeningForVmServiceUri = false;
        if (!completer.isCompleted && !isWaitingForVm) {
          completer.completeError(Exception('connection to device ended too early'));
        }
      },
    );
    _isListeningForVmServiceUri = true;
    return completer.future;
  }

  Future<void> exitApps({
    @visibleForTesting Duration timeoutDelay = const Duration(seconds: 10),
  }) async {
    // TODO(zanderso): https://github.com/flutter/flutter/issues/83127
    // When updating `flutter attach` to support running without a device,
    // this will need to be changed to fall back to io exit.
    await device!.stopApp(package, userIdentifier: userIdentifier);
  }

  Future<Uri?> setupDevFS(String fsName, Directory rootDirectory) {
    // One devFS per device. Shared by all running instances.
    devFS = DevFS(
      vmService!,
      fsName,
      rootDirectory,
      osUtils: globals.os,
      fileSystem: globals.fs,
      logger: globals.logger,
      processManager: globals.processManager,
      artifacts: globals.artifacts!,
      buildMode: buildInfo.mode,
    );
    return devFS!.create();
  }

  Future<void> startEchoingDeviceLog(DebuggingOptions debuggingOptions) async {
    if (_loggingSubscription != null) {
      return;
    }
    final Stream<String> logStream;
    if (device is IOSDevice) {
      logStream =
          (device! as IOSDevice)
              .getLogReader(app: package as IOSApp?, usingCISystem: debuggingOptions.usingCISystem)
              .logLines;
    } else {
      logStream = (await device!.getLogReader(app: package)).logLines;
    }
    _loggingSubscription = logStream.listen((String line) {
      if (!line.contains(globals.kVMServiceMessageRegExp)) {
        globals.printStatus(line, wrap: false);
      }
    });
  }

  Future<void> stopEchoingDeviceLog() async {
    if (_loggingSubscription == null) {
      return;
    }
    await _loggingSubscription!.cancel();
    _loggingSubscription = null;
  }

  Future<int> runHot({required HotRunner hotRunner, String? route}) async {
    final bool prebuiltMode = hotRunner.applicationBinary != null;
    final String modeName = hotRunner.debuggingOptions.buildInfo.friendlyModeName;
    globals.printStatus(
      'Launching ${getDisplayPath(hotRunner.mainPath, globals.fs)} '
      'on ${device!.displayName} in $modeName mode...',
    );

    final TargetPlatform targetPlatform = await device!.targetPlatform;
    package = await ApplicationPackageFactory.instance!.getPackageForPlatform(
      targetPlatform,
      buildInfo: hotRunner.debuggingOptions.buildInfo,
      applicationBinary: hotRunner.applicationBinary,
    );
    final ApplicationPackage? applicationPackage = package;

    if (applicationPackage == null) {
      String message = 'No application found for $targetPlatform.';
      final String? hint = await getMissingPackageHintForPlatform(targetPlatform);
      if (hint != null) {
        message += '\n$hint';
      }
      globals.printError(message);
      return 1;
    }
    devFSWriter = device!.createDevFSWriter(applicationPackage, userIdentifier);

    final Map<String, dynamic> platformArgs = <String, dynamic>{};

    await startEchoingDeviceLog(hotRunner.debuggingOptions);

    // Start the application.
    final Future<LaunchResult> futureResult = device!.startApp(
      applicationPackage,
      mainPath: hotRunner.mainPath,
      debuggingOptions: hotRunner.debuggingOptions,
      platformArgs: platformArgs,
      route: route,
      prebuiltApplication: prebuiltMode,
      userIdentifier: userIdentifier,
    );

    final LaunchResult result = await futureResult;

    if (!result.started) {
      globals.printError('Error launching application on ${device!.displayName}.');
      await stopEchoingDeviceLog();
      return 2;
    }
    if (result.hasVmService) {
      vmServiceUris = Stream<Uri?>.value(result.vmServiceUri).asBroadcastStream();
    } else {
      vmServiceUris = const Stream<Uri>.empty().asBroadcastStream();
    }
    return 0;
  }

  Future<int> runCold({required ColdRunner coldRunner, String? route}) async {
    final TargetPlatform targetPlatform = await device!.targetPlatform;
    package = await ApplicationPackageFactory.instance!.getPackageForPlatform(
      targetPlatform,
      buildInfo: coldRunner.debuggingOptions.buildInfo,
      applicationBinary: coldRunner.applicationBinary,
    );
    final ApplicationPackage? applicationPackage = package;

    if (applicationPackage == null) {
      String message = 'No application found for $targetPlatform.';
      final String? hint = await getMissingPackageHintForPlatform(targetPlatform);
      if (hint != null) {
        message += '\n$hint';
      }
      globals.printError(message);
      return 1;
    }

    devFSWriter = device!.createDevFSWriter(applicationPackage, userIdentifier);

    final String modeName = coldRunner.debuggingOptions.buildInfo.friendlyModeName;
    final bool prebuiltMode = coldRunner.applicationBinary != null;
    globals.printStatus(
      'Launching ${getDisplayPath(coldRunner.mainPath, globals.fs)} '
      'on ${device!.displayName} in $modeName mode...',
    );

    final Map<String, dynamic> platformArgs = <String, dynamic>{};
    platformArgs['trace-startup'] = coldRunner.traceStartup;

    await startEchoingDeviceLog(coldRunner.debuggingOptions);

    final LaunchResult result = await device!.startApp(
      applicationPackage,
      mainPath: coldRunner.mainPath,
      debuggingOptions: coldRunner.debuggingOptions,
      platformArgs: platformArgs,
      route: route,
      prebuiltApplication: prebuiltMode,
      userIdentifier: userIdentifier,
    );

    if (!result.started) {
      globals.printError('Error running application on ${device!.displayName}.');
      await stopEchoingDeviceLog();
      return 2;
    }
    if (result.hasVmService) {
      vmServiceUris = Stream<Uri?>.value(result.vmServiceUri).asBroadcastStream();
    } else {
      vmServiceUris = const Stream<Uri>.empty().asBroadcastStream();
    }
    return 0;
  }

  Future<UpdateFSReport> updateDevFS({
    required Uri mainUri,
    String? target,
    AssetBundle? bundle,
    bool bundleFirstUpload = false,
    bool bundleDirty = false,
    bool fullRestart = false,
    required String pathToReload,
    required String dillOutputPath,
    required List<Uri> invalidatedFiles,
    required PackageConfig packageConfig,
  }) async {
    final Status devFSStatus = globals.logger.startProgress(
      'Syncing files to device ${device!.displayName}...',
      progressId: 'devFS.update',
    );
    UpdateFSReport report;
    try {
      report = await devFS!.update(
        mainUri: mainUri,
        target: target,
        bundle: bundle,
        bundleFirstUpload: bundleFirstUpload,
        generator: generator!,
        fullRestart: fullRestart,
        resetCompiler: fullRestart,
        dillOutputPath: dillOutputPath,
        trackWidgetCreation: buildInfo.trackWidgetCreation,
        pathToReload: pathToReload,
        invalidatedFiles: invalidatedFiles,
        packageConfig: packageConfig,
        devFSWriter: devFSWriter,
        shaderCompiler: developmentShaderCompiler,
        dartPluginRegistrant: FlutterProject.current().dartPluginRegistrant,
      );
    } on DevFSException {
      devFSStatus.cancel();
      return UpdateFSReport();
    }
    devFSStatus.stop();
    globals.printTrace('Synced ${getSizeAsPlatformMB(report.syncedBytes)}.');
    return report;
  }

  Future<void> updateReloadStatus(bool wasReloadSuccessful) async {
    if (wasReloadSuccessful) {
      generator?.accept();
    } else {
      await generator?.reject();
    }
  }
}

/// A subset of the [ResidentRunner] for delegating to attached flutter devices.
abstract class ResidentHandlers {
  List<FlutterDevice?> get flutterDevices;

  /// Whether the resident runner has hot reload and restart enabled.
  bool get hotMode;

  /// Whether the resident runner is connect to the device's VM Service.
  bool get supportsServiceProtocol;

  /// The application is running in debug mode.
  bool get isRunningDebug;

  /// The application is running in profile mode.
  bool get isRunningProfile;

  /// The application is running in release mode.
  bool get isRunningRelease;

  /// The resident runner should stay resident after establishing a connection with the
  /// application.
  bool get stayResident;

  /// Whether all of the connected devices support hot restart.
  ///
  /// To prevent scenarios where only a subset of devices are hot restarted,
  /// the runner requires that all attached devices can support hot restart
  /// before enabling it.
  bool get supportsRestart;

  /// Whether all of the connected devices support hot reload.
  bool get canHotReload;

  /// Whether an application can be detached without being stopped.
  bool get supportsDetach;

  // TODO(bkonyi): remove when ready to serve DevTools from DDS.
  ResidentDevtoolsHandler? get residentDevtoolsHandler;

  @protected
  Logger get logger;

  @protected
  FileSystem? get fileSystem;

  /// Called to print help to the terminal.
  ///
  /// If [details] is true, prints out extra help information.
  void printHelp({required bool details});

  /// Perform a hot reload or hot restart of all attached applications.
  ///
  /// If [fullRestart] is true, a hot restart is performed. Otherwise a hot reload
  /// is run instead. On web devices, this only performs a hot restart regardless of
  /// the value of [fullRestart].
  Future<OperationResult> restart({bool fullRestart = false, bool pause = false, String? reason}) {
    final String mode =
        isRunningProfile
            ? 'profile'
            : isRunningRelease
            ? 'release'
            : 'this';
    throw Exception('${fullRestart ? 'Restart' : 'Reload'} is not supported in $mode mode');
  }

  /// Dump the application's current widget tree to the terminal.
  Future<bool> debugDumpApp() async {
    if (!supportsServiceProtocol) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        final String data = await device.vmService!.flutterDebugDumpApp(
          isolateId: view.uiIsolate!.id!,
        );
        logger.printStatus(data);
      }
    }
    return true;
  }

  /// Dump the application's current render tree to the terminal.
  Future<bool> debugDumpRenderTree() async {
    if (!supportsServiceProtocol) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        final String data = await device.vmService!.flutterDebugDumpRenderTree(
          isolateId: view.uiIsolate!.id!,
        );
        logger.printStatus(data);
      }
    }
    return true;
  }

  /// Dump the application's current layer tree to the terminal.
  Future<bool> debugDumpLayerTree() async {
    if (!supportsServiceProtocol || !isRunningDebug) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        final String data = await device.vmService!.flutterDebugDumpLayerTree(
          isolateId: view.uiIsolate!.id!,
        );
        logger.printStatus(data);
      }
    }
    return true;
  }

  Future<bool> debugDumpFocusTree() async {
    if (!supportsServiceProtocol || !isRunningDebug) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        final String data = await device.vmService!.flutterDebugDumpFocusTree(
          isolateId: view.uiIsolate!.id!,
        );
        logger.printStatus(data);
      }
    }
    return true;
  }

  /// Dump the application's current semantics tree to the terminal.
  ///
  /// If semantics are not enabled, nothing is returned.
  Future<bool> debugDumpSemanticsTreeInTraversalOrder() async {
    if (!supportsServiceProtocol) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        final String data = await device.vmService!.flutterDebugDumpSemanticsTreeInTraversalOrder(
          isolateId: view.uiIsolate!.id!,
        );
        logger.printStatus(data);
      }
    }
    return true;
  }

  /// Dump the application's current semantics tree to the terminal.
  ///
  /// If semantics are not enabled, nothing is returned.
  Future<bool> debugDumpSemanticsTreeInInverseHitTestOrder() async {
    if (!supportsServiceProtocol) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        final String data = await device.vmService!
            .flutterDebugDumpSemanticsTreeInInverseHitTestOrder(isolateId: view.uiIsolate!.id!);
        logger.printStatus(data);
      }
    }
    return true;
  }

  /// Toggle the "paint size" debugging feature.
  Future<bool> debugToggleDebugPaintSizeEnabled() async {
    if (!supportsServiceProtocol || !isRunningDebug) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        await device.vmService!.flutterToggleDebugPaintSizeEnabled(isolateId: view.uiIsolate!.id!);
      }
    }
    return true;
  }

  /// Toggle the performance overlay.
  ///
  /// This is not supported in web mode.
  Future<bool> debugTogglePerformanceOverlayOverride() async {
    if (!supportsServiceProtocol) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      if (device!.targetPlatform == TargetPlatform.web_javascript) {
        continue;
      }
      final List<FlutterView> views = await device.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        await device.vmService!.flutterTogglePerformanceOverlayOverride(
          isolateId: view.uiIsolate!.id!,
        );
      }
    }
    return true;
  }

  /// Toggle the widget inspector.
  Future<bool> debugToggleWidgetInspector() async {
    if (!supportsServiceProtocol) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        await device.vmService!.flutterToggleWidgetInspector(isolateId: view.uiIsolate!.id!);
      }
    }
    return true;
  }

  /// Toggle the "invert images" debugging feature.
  Future<bool> debugToggleInvertOversizedImages() async {
    if (!supportsServiceProtocol || !isRunningDebug) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        await device.vmService!.flutterToggleInvertOversizedImages(isolateId: view.uiIsolate!.id!);
      }
    }
    return true;
  }

  /// Toggle the "profile widget builds" debugging feature.
  Future<bool> debugToggleProfileWidgetBuilds() async {
    if (!supportsServiceProtocol) {
      return false;
    }
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        await device.vmService!.flutterToggleProfileWidgetBuilds(isolateId: view.uiIsolate!.id!);
      }
    }
    return true;
  }

  /// Toggle the operating system brightness (light or dark).
  Future<bool> debugToggleBrightness() async {
    if (!supportsServiceProtocol) {
      return false;
    }
    final List<FlutterView> views = await flutterDevices.first!.vmService!.getFlutterViews();
    final Brightness? current = await flutterDevices.first!.vmService!.flutterBrightnessOverride(
      isolateId: views.first.uiIsolate!.id!,
    );
    final Brightness next = switch (current) {
      Brightness.light => Brightness.dark,
      Brightness.dark || null => Brightness.light,
    };
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        await device.vmService!.flutterBrightnessOverride(
          isolateId: view.uiIsolate!.id!,
          brightness: next,
        );
      }
      logger.printStatus('Changed brightness to $next.');
    }
    return true;
  }

  /// Rotate the application through different `defaultTargetPlatform` values.
  Future<bool> debugTogglePlatform() async {
    if (!supportsServiceProtocol || !isRunningDebug) {
      return false;
    }
    final List<FlutterView> views = await flutterDevices.first!.vmService!.getFlutterViews();
    final String from = await flutterDevices.first!.vmService!.flutterPlatformOverride(
      isolateId: views.first.uiIsolate!.id!,
    );
    final String to = nextPlatform(from);
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        await device.vmService!.flutterPlatformOverride(
          platform: to,
          isolateId: view.uiIsolate!.id!,
        );
      }
    }
    logger.printStatus('Switched operating system to $to');
    return true;
  }

  /// Take a screenshot on the provided [device].
  ///
  /// If the device has a connected vmservice, this method will attempt to hide
  /// and restore the debug banner before taking the screenshot.
  ///
  /// If the device type does not support a "native" screenshot, then this
  /// will fallback to a rasterizer screenshot from the engine. This has the
  /// downside of being unable to display the contents of platform views.
  ///
  /// This method will return without writing the screenshot file if any
  /// RPC errors are encountered, printing them to stderr. This is true even
  /// if an error occurs after the data has already been received, such as
  /// from restoring the debug banner.
  Future<void> screenshot(FlutterDevice device) async {
    if (!device.device!.supportsScreenshot && !supportsServiceProtocol) {
      return;
    }
    final Status status = logger.startProgress(
      'Taking screenshot for ${device.device!.displayName}...',
    );
    final File outputFile = getUniqueFile(fileSystem!.currentDirectory, 'flutter', 'png');

    try {
      bool result;
      if (device.device!.supportsScreenshot) {
        result = await _toggleDebugBanner(device, () => device.device!.takeScreenshot(outputFile));
      } else {
        result = await _takeVmServiceScreenshot(device, outputFile);
      }
      if (!result) {
        return;
      }
      final int sizeKB = outputFile.lengthSync() ~/ 1024;
      status.stop();
      logger.printStatus(
        'Screenshot written to ${fileSystem!.path.relative(outputFile.path)} (${sizeKB}kB).',
      );
    } on Exception catch (error) {
      status.cancel();
      logger.printError('Error taking screenshot: $error');
    }
  }

  Future<bool> _takeVmServiceScreenshot(FlutterDevice device, File outputFile) async {
    if (device.targetPlatform != TargetPlatform.web_javascript) {
      return false;
    }
    assert(supportsServiceProtocol);

    return _toggleDebugBanner(device, () async {
      final vm_service.Response? response = await device.vmService!.callMethodWrapper(
        'ext.dwds.screenshot',
      );
      if (response == null) {
        throw Exception('Failed to take screenshot');
      }
      final String data = response.json!['data'] as String;
      outputFile.writeAsBytesSync(base64.decode(data));
    });
  }

  Future<bool> _toggleDebugBanner(FlutterDevice device, Future<void> Function() cb) async {
    List<FlutterView> views = <FlutterView>[];
    if (supportsServiceProtocol) {
      views = await device.vmService!.getFlutterViews();
    }

    Future<bool> setDebugBanner(bool value) async {
      try {
        for (final FlutterView view in views) {
          await device.vmService!.flutterDebugAllowBanner(value, isolateId: view.uiIsolate!.id!);
        }
        return true;
      } on vm_service.RPCError catch (error) {
        logger.printError('Error communicating with Flutter on the device: $error');
        return false;
      }
    }

    if (!await setDebugBanner(false)) {
      return false;
    }
    bool succeeded = true;
    try {
      await cb();
    } finally {
      if (!await setDebugBanner(true)) {
        succeeded = false;
      }
    }
    return succeeded;
  }

  /// Remove sigusr signal handlers.
  Future<void> cleanupAfterSignal();

  /// Tear down the runner and leave the application running.
  Future<void> detach();

  /// Tear down the runner and exit the application.
  Future<void> exit();

  /// Run any source generators, such as localizations.
  ///
  /// These are automatically run during hot restart, but can be
  /// triggered manually to see the updated generated code.
  Future<void> runSourceGenerators();
}

// Shared code between different resident application runners.
abstract class ResidentRunner extends ResidentHandlers {
  ResidentRunner(
    this.flutterDevices, {
    required this.target,
    required this.debuggingOptions,
    String? projectRootPath,
    this.stayResident = true,
    this.hotMode = true,
    String? dillOutputPath,
    this.machine = false,
    ResidentDevtoolsHandlerFactory devtoolsHandler = createDefaultHandler,
    CommandHelp? commandHelp,
  }) : mainPath = globals.fs.file(target).absolute.path,
       packagesFilePath = debuggingOptions.buildInfo.packageConfigPath,
       projectRootPath = projectRootPath ?? globals.fs.currentDirectory.path,
       _dillOutputPath = dillOutputPath,
       artifactDirectory =
           dillOutputPath == null
               ? globals.fs.systemTempDirectory.createTempSync('flutter_tool.')
               : globals.fs.file(dillOutputPath).parent,
       assetBundle = AssetBundleFactory.instance.createBundle(),
       commandHelp =
           commandHelp ??
           CommandHelp(
             logger: globals.logger,
             terminal: globals.terminal,
             platform: globals.platform,
             outputPreferences: globals.outputPreferences,
           ) {
    if (!artifactDirectory.existsSync()) {
      artifactDirectory.createSync(recursive: true);
    }
    // TODO(bkonyi): remove when ready to serve DevTools from DDS.
    _residentDevtoolsHandler = devtoolsHandler(
      DevtoolsLauncher.instance,
      this,
      globals.logger,
      ChromiumLauncher(
        fileSystem: globals.fs,
        platform: globals.platform,
        processManager: globals.processManager,
        operatingSystemUtils: globals.os,
        browserFinder: findChromeExecutable,
        logger: globals.logger,
      ),
    );
  }

  @override
  Logger get logger => globals.logger;

  @override
  FileSystem get fileSystem => globals.fs;

  @override
  final List<FlutterDevice> flutterDevices;

  final String target;
  final DebuggingOptions debuggingOptions;

  @override
  final bool stayResident;
  final String? _dillOutputPath;

  /// The parent location of the incremental artifacts.
  final Directory artifactDirectory;
  final String packagesFilePath;
  final String projectRootPath;
  final String mainPath;
  final AssetBundle assetBundle;

  final CommandHelp commandHelp;
  final bool machine;

  // TODO(bkonyi): remove when ready to serve DevTools from DDS.
  @override
  ResidentDevtoolsHandler? get residentDevtoolsHandler => _residentDevtoolsHandler;
  ResidentDevtoolsHandler? _residentDevtoolsHandler;

  bool _exited = false;
  Completer<int> _finished = Completer<int>();
  BuildResult? _lastBuild;
  Environment? _environment;

  @override
  bool hotMode;

  /// Returns true if every device is streaming vmService URIs.
  bool get isWaitingForVmService {
    return flutterDevices.every((FlutterDevice? device) {
      return device!.isWaitingForVmService;
    });
  }

  String get dillOutputPath =>
      _dillOutputPath ?? globals.fs.path.join(artifactDirectory.path, 'app.dill');
  String getReloadPath({bool resetCompiler = false, required bool swap}) {
    if (!resetCompiler) {
      return 'main.dart.incremental.dill';
    }
    return 'main.dart${swap ? '.swap' : ''}.dill';
  }

  /// Whether the app being instrumented by the runner should be stopped during
  /// cleanup.
  ///
  /// A detached app can happen one of two ways:
  /// - [run] is used, and then the created application is manually [detach]ed;
  /// - [attach] is used to explicitly connect to an already running app.
  @protected
  @visibleForTesting
  bool stopAppDuringCleanup = true;

  bool get debuggingEnabled => debuggingOptions.debuggingEnabled;

  @override
  bool get isRunningDebug => !debuggingOptions.webUseWasm && debuggingOptions.buildInfo.isDebug;

  @override
  bool get isRunningProfile => !debuggingOptions.webUseWasm && debuggingOptions.buildInfo.isProfile;

  @override
  bool get isRunningRelease => debuggingOptions.buildInfo.isRelease;

  @override
  bool get supportsServiceProtocol => isRunningDebug || isRunningProfile;

  bool get trackWidgetCreation => debuggingOptions.buildInfo.trackWidgetCreation;

  /// True if the shared Dart plugin registry (which is different than the one
  /// used for web) should be generated during source generation.
  bool get generateDartPluginRegistry => true;

  // Returns the Uri of the first connected device for mobile,
  // and only connected device for web.
  //
  // Would be null if there is no device connected or
  // there is no devFS associated with the first device.
  Uri? get uri => flutterDevices.first.devFS?.baseUri;

  /// Returns [true] if the resident runner exited after invoking [exit()].
  bool get exited => _exited;

  @override
  bool get supportsRestart {
    return isRunningDebug &&
        flutterDevices.every((FlutterDevice? device) {
          return device!.device!.supportsHotRestart;
        });
  }

  @override
  bool get canHotReload => hotMode;

  /// Whether the hot reload support is implemented as hot restart.
  bool get reloadIsRestart => false;

  /// Start the app and keep the process running during its lifetime.
  ///
  /// Returns the exit code that we should use for the flutter tool process; 0
  /// for success, 1 for user error (e.g. bad arguments), 2 for other failures.
  Future<int?> run({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    String? route,
  });

  /// Connect to a flutter application.
  ///
  /// [needsFullRestart] defaults to `true`, and controls if the frontend server should
  /// compile a full dill. This should be set to `false` if this is called in [ResidentRunner.run], since that method already performs an initial compilation.
  Future<int?> attach({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    bool allowExistingDdsInstance = false,
    bool needsFullRestart = true,
  });

  @override
  Future<void> runSourceGenerators() async {
    _environment ??= Environment(
      artifacts: globals.artifacts!,
      logger: globals.logger,
      cacheDir: globals.cache.getRoot(),
      engineVersion: globals.flutterVersion.engineRevision,
      fileSystem: globals.fs,
      flutterRootDir: globals.fs.directory(Cache.flutterRoot),
      outputDir: globals.fs.directory(getBuildDirectory()),
      processManager: globals.processManager,
      platform: globals.platform,
      analytics: globals.analytics,
      projectDir: globals.fs.currentDirectory,
      packageConfigPath: debuggingOptions.buildInfo.packageConfigPath,
      generateDartPluginRegistry: generateDartPluginRegistry,
      defines: <String, String>{
        // Needed for Dart plugin registry generation.
        kTargetFile: mainPath,
      },
    );

    final CompositeTarget compositeTarget = CompositeTarget(<Target>[
      globals.buildTargets.generateLocalizationsTarget,
      globals.buildTargets.dartPluginRegistrantTarget,
    ]);

    _lastBuild = await globals.buildSystem.buildIncremental(
      compositeTarget,
      _environment!,
      _lastBuild,
    );
    if (!_lastBuild!.success) {
      for (final ExceptionMeasurement exceptionMeasurement in _lastBuild!.exceptions.values) {
        globals.printError(
          exceptionMeasurement.exception.toString(),
          stackTrace: globals.logger.isVerbose ? exceptionMeasurement.stackTrace : null,
        );
      }
    }
    globals.printTrace('complete');
  }

  @protected
  void writeVmServiceFile() {
    if (debuggingOptions.vmserviceOutFile != null) {
      try {
        final String address = flutterDevices.first.vmService!.wsAddress.toString();
        final File vmserviceOutFile = globals.fs.file(debuggingOptions.vmserviceOutFile);
        vmserviceOutFile.createSync(recursive: true);
        vmserviceOutFile.writeAsStringSync(address);
      } on FileSystemException {
        globals.printError(
          'Failed to write vmservice-out-file at ${debuggingOptions.vmserviceOutFile}',
        );
      }
    }
  }

  @override
  Future<void> exit() async {
    _exited = true;
    // TODO(bkonyi): remove when ready to serve DevTools from DDS.
    await residentDevtoolsHandler!.shutdown();
    await stopEchoingDeviceLog();
    await preExit();
    await exitApp(); // calls appFinished
    shutdownDartDevelopmentService();
  }

  @override
  @mustCallSuper
  Future<void> detach() async {
    stopAppDuringCleanup = false;

    // TODO(bkonyi): remove when ready to serve DevTools from DDS.
    await residentDevtoolsHandler!.shutdown();
    await stopEchoingDeviceLog();
    await preExit();
    shutdownDartDevelopmentService();
    appFinished();
  }

  Future<void> stopEchoingDeviceLog() async {
    await Future.wait<void>(
      flutterDevices.map<Future<void>>((FlutterDevice? device) => device!.stopEchoingDeviceLog()),
    );
  }

  void shutdownDartDevelopmentService() {
    for (final FlutterDevice device in flutterDevices) {
      device.device?.dds.shutdown();
    }
  }

  @protected
  void cacheInitialDillCompilation() {
    if (_dillOutputPath != null) {
      return;
    }
    globals.printTrace('Caching compiled dill');
    final File outputDill = globals.fs.file(dillOutputPath);
    if (outputDill.existsSync()) {
      final String copyPath = getDefaultCachedKernelPath(
        trackWidgetCreation: trackWidgetCreation,
        dartDefines: debuggingOptions.buildInfo.dartDefines,
        extraFrontEndOptions: debuggingOptions.buildInfo.extraFrontEndOptions,
      );
      globals.fs.file(copyPath).parent.createSync(recursive: true);
      outputDill.copySync(copyPath);
    }
  }

  void printStructuredErrorLog(vm_service.Event event) {
    if (event.extensionKind == 'Flutter.Error' && !machine) {
      final Map<String, Object?>? json = event.extensionData?.data;
      if (json != null && json.containsKey('renderedErrorText')) {
        final int errorsSinceReload;
        if (json.containsKey('errorsSinceReload') && json['errorsSinceReload'] is int) {
          errorsSinceReload = json['errorsSinceReload']! as int;
        } else {
          errorsSinceReload = 0;
        }
        if (errorsSinceReload == 0) {
          // We print a blank line around the first error, to more clearly emphasize it
          // in the output. (Other errors don't get this.)
          globals.printStatus('');
        }
        globals.printStatus('${json['renderedErrorText']}');
        if (errorsSinceReload == 0) {
          globals.printStatus('');
        }
      } else {
        globals.printError(
          'Received an invalid ${globals.logger.terminal.bolden("Flutter.Error")} message from app: $json',
        );
      }
    }
  }

  /// If the [reloadSources] parameter is not null the 'reloadSources' service
  /// will be registered.
  //
  // Failures should be indicated by completing the future with an error, using
  // a string as the error object, which will be used by the caller (attach())
  // to display an error message.
  Future<void> connectToServiceProtocol({
    ReloadSources? reloadSources,
    Restart? restart,
    CompileExpression? compileExpression,
    required bool allowExistingDdsInstance,
  }) async {
    if (!debuggingOptions.debuggingEnabled) {
      throw Exception('The service protocol is not enabled.');
    }
    _finished = Completer<int>();
    // Listen for service protocol connection to close.
    for (final FlutterDevice? device in flutterDevices) {
      await device!.connect(
        debuggingOptions: debuggingOptions,
        reloadSources: reloadSources,
        restart: restart,
        compileExpression: compileExpression,
        allowExistingDdsInstance: allowExistingDdsInstance,
        hostVmServicePort: debuggingOptions.hostVmServicePort,
        printStructuredErrorLogMethod: printStructuredErrorLog,
      );
      await device.vmService!.getFlutterViews();

      // This hooks up callbacks for when the connection stops in the future.
      // We don't want to wait for them. We don't handle errors in those callbacks'
      // futures either because they just print to logger and is not critical.
      unawaited(
        device.vmService!.service.onDone
            .then<void>(_serviceProtocolDone, onError: _serviceProtocolError)
            .whenComplete(_serviceDisconnected),
      );
    }
  }

  Future<void> _serviceProtocolDone(dynamic object) async {
    globals.printTrace('Service protocol connection closed.');
  }

  Future<void> _serviceProtocolError(Object error, StackTrace stack) {
    globals.printTrace('Service protocol connection closed with an error: $error\n$stack');
    return Future<void>.error(error, stack);
  }

  void _serviceDisconnected() {
    if (_exited) {
      // User requested the application exit.
      return;
    }
    if (_finished.isCompleted) {
      return;
    }
    globals.printStatus('Lost connection to device.');
    _finished.complete(0);
  }

  Future<void> enableObservatory() async {
    assert(debuggingOptions.serveObservatory);
    final List<Future<vm_service.Response?>> serveObservatoryRequests =
        <Future<vm_service.Response?>>[
          for (final FlutterDevice? device in flutterDevices)
            if (device != null)
              // Notify the VM service if the user wants Observatory to be served.
              device.vmService?.callMethodWrapper('_serveObservatory') ??
                  Future<vm_service.Response?>.value(),
        ];
    try {
      await Future.wait(serveObservatoryRequests);
    } on vm_service.RPCError catch (e) {
      globals.printWarning('Unable to enable Observatory: $e');
    }
  }

  @protected
  void appFinished() {
    if (_finished.isCompleted) {
      return;
    }
    globals.printStatus('Application finished.');
    _finished.complete(0);
  }

  void appFailedToStart() {
    if (!_finished.isCompleted) {
      _finished.complete(1);
    }
  }

  Future<int> waitForAppToFinish() async {
    final int exitCode = await _finished.future;
    await cleanupAtFinish();
    return exitCode;
  }

  @mustCallSuper
  Future<void> preExit() async {
    // If _dillOutputPath is null, the tool created a temporary directory for
    // the dill.
    if (_dillOutputPath == null && artifactDirectory.existsSync()) {
      artifactDirectory.deleteSync(recursive: true);
    }
  }

  Future<void> exitApp() async {
    final List<Future<void>> futures = <Future<void>>[
      for (final FlutterDevice? device in flutterDevices) device!.exitApps(),
    ];
    await Future.wait(futures);
    appFinished();
  }

  bool get reportedDebuggers => _reportedDebuggers;
  bool _reportedDebuggers = false;

  void printDebuggerList({bool includeVmService = true, bool includeDevtools = true}) {
    // TODO(bkonyi): update this logic when ready to serve DevTools from DDS.
    final DevToolsServerAddress? devToolsServerAddress =
        residentDevtoolsHandler!.activeDevToolsServer;
    if (!residentDevtoolsHandler!.readyToAnnounce) {
      includeDevtools = false;
    }
    assert(!includeDevtools || devToolsServerAddress != null);
    for (final FlutterDevice? device in flutterDevices) {
      if (device!.vmService == null) {
        continue;
      }
      if (includeVmService) {
        // Caution: This log line is parsed by device lab tests.
        logger.printStatus(
          'A Dart VM Service on ${device.device!.displayName} '
          'is available at: ${device.vmService!.httpAddress}',
        );
      }
      if (includeDevtools) {
        if (_residentDevtoolsHandler!.printDtdUri) {
          final Uri? dtdUri = residentDevtoolsHandler!.dtdUri;
          if (dtdUri != null) {
            logger.printStatus('The Dart Tooling Daemon is available at: $dtdUri\n');
          }
        }
        final Uri? uri = devToolsServerAddress!.uri?.replace(
          queryParameters: <String, dynamic>{'uri': '${device.vmService!.httpAddress}'},
        );
        if (uri != null) {
          logger.printStatus(
            'The Flutter DevTools debugger and profiler '
            'on ${device.device!.displayName} '
            'is available at: ${urlToDisplayString(uri)}',
          );
        }
      }
    }
    _reportedDebuggers = true;
  }

  @override
  void printHelp({required bool details}) {
    logger.printStatus('Flutter run key commands.');
    // Don't print the command in the case where the runner implements reload as
    // restart since it's misleading.
    if (canHotReload && !reloadIsRestart) {
      commandHelp.r.print();
    }
    if (supportsRestart) {
      commandHelp.R.print();
    }
    if (details) {
      printHelpDetails();
      commandHelp.hWithDetails.print();
    } else {
      commandHelp.hWithoutDetails.print();
    }
    if (supportsDetach) {
      commandHelp.d.print();
    }
    commandHelp.c.print();
    commandHelp.q.print();
    logger.printStatus('');
    printDebuggerList();
  }

  void printHelpDetails() {
    commandHelp.v.print();
    if (flutterDevices.any((FlutterDevice? d) => d!.device!.supportsScreenshot)) {
      commandHelp.s.print();
    }
    if (supportsServiceProtocol) {
      commandHelp.w.print();
      commandHelp.t.print();
      if (isRunningDebug) {
        commandHelp.L.print();
        commandHelp.f.print();
        commandHelp.S.print();
        commandHelp.U.print();
        commandHelp.i.print();
        commandHelp.p.print();
        commandHelp.I.print();
        commandHelp.o.print();
        commandHelp.b.print();
      } else {
        commandHelp.S.print();
        commandHelp.U.print();
      }
      // Performance related features: `P` should precede `a`, which should precede `M`.
      commandHelp.P.print();
      commandHelp.a.print();
      if (isRunningDebug) {
        commandHelp.g.print();
      }
    }
  }

  @override
  Future<void> cleanupAfterSignal();

  /// Called right before we exit.
  Future<void> cleanupAtFinish();
}

class OperationResult {
  OperationResult(
    this.code,
    this.message, {
    this.fatal = false,
    this.updateFSReport,
    this.extraTimings = const <OperationResultExtraTiming>[],
  });

  /// The result of the operation; a non-zero code indicates a failure.
  final int code;

  /// A user facing message about the results of the operation.
  final String message;

  /// User facing extra timing information about the operation.
  final List<OperationResultExtraTiming> extraTimings;

  /// Whether this error should cause the runner to exit.
  final bool fatal;

  final UpdateFSReport? updateFSReport;

  bool get isOk => code == 0;

  static final OperationResult ok = OperationResult(0, '');
}

class OperationResultExtraTiming {
  const OperationResultExtraTiming(this.description, this.timeInMs);

  /// A user facing short description of this timing.
  final String description;

  /// The time this operation took in milliseconds.
  final int timeInMs;
}

Future<String?> getMissingPackageHintForPlatform(TargetPlatform platform) async {
  switch (platform) {
    case TargetPlatform.android_arm:
    case TargetPlatform.android_arm64:
    case TargetPlatform.android_x64:
    case TargetPlatform.android_x86:
      final FlutterProject project = FlutterProject.current();
      final String manifestPath = globals.fs.path.relative(project.android.appManifestFile.path);
      return 'Is your project missing an $manifestPath?\nConsider running "flutter create ." to create one.';
    case TargetPlatform.ios:
      return 'Is your project missing an ios/Runner/Info.plist?\nConsider running "flutter create ." to create one.';
    case TargetPlatform.android:
    case TargetPlatform.darwin:
    case TargetPlatform.fuchsia_arm64:
    case TargetPlatform.fuchsia_x64:
    case TargetPlatform.linux_arm64:
    case TargetPlatform.linux_x64:
    case TargetPlatform.tester:
    case TargetPlatform.web_javascript:
    case TargetPlatform.windows_x64:
    case TargetPlatform.windows_arm64:
      return null;
  }
}

/// Redirects terminal commands to the correct resident runner methods.
class TerminalHandler {
  TerminalHandler(
    this.residentRunner, {
    required Logger logger,
    required Terminal terminal,
    required Signals signals,
    required io.ProcessInfo processInfo,
    required bool reportReady,
    String? pidFile,
  }) : _logger = logger,
       _terminal = terminal,
       _signals = signals,
       _processInfo = processInfo,
       _reportReady = reportReady,
       _pidFile = pidFile;

  final Logger _logger;
  final Terminal _terminal;
  final Signals _signals;
  final io.ProcessInfo _processInfo;
  final bool _reportReady;
  final String? _pidFile;

  final ResidentHandlers residentRunner;
  bool _processingUserRequest = false;
  StreamSubscription<void>? subscription;
  File? _actualPidFile;

  @visibleForTesting
  String? lastReceivedCommand;

  /// This is only a buffer logger in unit tests
  @visibleForTesting
  BufferLogger get logger => _logger as BufferLogger;

  void setupTerminal() {
    if (!_logger.quiet) {
      _logger.printStatus('');
      residentRunner.printHelp(details: false);
    }
    _terminal.singleCharMode = true;
    subscription = _terminal.keystrokes.listen(processTerminalInput);
  }

  final Map<io.ProcessSignal, Object> _signalTokens = <io.ProcessSignal, Object>{};

  void _addSignalHandler(io.ProcessSignal signal, SignalHandler handler) {
    _signalTokens[signal] = _signals.addHandler(signal, handler);
  }

  void registerSignalHandlers() {
    assert(residentRunner.stayResident);
    _addSignalHandler(io.ProcessSignal.sigint, _cleanUp);
    _addSignalHandler(io.ProcessSignal.sigterm, _cleanUp);
    if (residentRunner.supportsServiceProtocol && residentRunner.supportsRestart) {
      _addSignalHandler(io.ProcessSignal.sigusr1, _handleSignal);
      _addSignalHandler(io.ProcessSignal.sigusr2, _handleSignal);
      if (_pidFile != null) {
        _logger.printTrace('Writing pid to: $_pidFile');
        _actualPidFile = _processInfo.writePidFile(_pidFile);
      }
    }
  }

  /// Unregisters terminal signal and keystroke handlers.
  void stop() {
    assert(residentRunner.stayResident);
    if (_actualPidFile != null) {
      try {
        _logger.printTrace('Deleting pid file (${_actualPidFile!.path}).');
        _actualPidFile!.deleteSync();
      } on FileSystemException catch (error) {
        _logger.printWarning(
          'Failed to delete pid file (${_actualPidFile!.path}): ${error.message}',
        );
      }
      _actualPidFile = null;
    }
    for (final MapEntry<io.ProcessSignal, Object> entry in _signalTokens.entries) {
      _signals.removeHandler(entry.key, entry.value);
    }
    _signalTokens.clear();
    subscription?.cancel();
  }

  /// Returns [true] if the input has been handled by this function.
  Future<bool> _commonTerminalInputHandler(String character) async {
    _logger.printStatus(''); // the key the user tapped might be on this line
    switch (character) {
      case 'a':
        return residentRunner.debugToggleProfileWidgetBuilds();
      case 'b':
        return residentRunner.debugToggleBrightness();
      case 'c':
        _logger.clear();
        return true;
      case 'd':
      case 'D':
        await residentRunner.detach();
        return true;
      case 'f':
        return residentRunner.debugDumpFocusTree();
      case 'g':
        await residentRunner.runSourceGenerators();
        return true;
      case 'h':
      case 'H':
      case '?':
        // help
        residentRunner.printHelp(details: true);
        return true;
      case 'i':
        return residentRunner.debugToggleWidgetInspector();
      case 'I':
        return residentRunner.debugToggleInvertOversizedImages();
      case 'L':
        return residentRunner.debugDumpLayerTree();
      case 'o':
      case 'O':
        return residentRunner.debugTogglePlatform();
      case 'p':
        return residentRunner.debugToggleDebugPaintSizeEnabled();
      case 'P':
        return residentRunner.debugTogglePerformanceOverlayOverride();
      case 'q':
      case 'Q':
        // exit
        await residentRunner.exit();
        return true;
      case 'r':
        if (!residentRunner.canHotReload) {
          return false;
        }
        final OperationResult result = await residentRunner.restart();
        if (result.fatal) {
          throwToolExit(result.message);
        }
        if (!result.isOk) {
          _logger.printStatus('Try again after fixing the above error(s).', emphasis: true);
        }
        return true;
      case 'R':
        // If hot restart is not supported for all devices, ignore the command.
        if (!residentRunner.supportsRestart || !residentRunner.hotMode) {
          return false;
        }
        final OperationResult result = await residentRunner.restart(fullRestart: true);
        if (result.fatal) {
          throwToolExit(result.message);
        }
        if (!result.isOk) {
          _logger.printStatus('Try again after fixing the above error(s).', emphasis: true);
        }
        return true;
      case 's':
        for (final FlutterDevice? device in residentRunner.flutterDevices) {
          await residentRunner.screenshot(device!);
        }
        return true;
      case 'S':
        return residentRunner.debugDumpSemanticsTreeInTraversalOrder();
      case 't':
      case 'T':
        return residentRunner.debugDumpRenderTree();
      case 'U':
        return residentRunner.debugDumpSemanticsTreeInInverseHitTestOrder();
      case 'v':
      case 'V':
        return residentRunner.residentDevtoolsHandler!.launchDevToolsInBrowser(
          flutterDevices: residentRunner.flutterDevices,
        );
      case 'w':
      case 'W':
        return residentRunner.debugDumpApp();
    }
    return false;
  }

  Future<void> processTerminalInput(String command) async {
    // When terminal doesn't support line mode, '\n' can sneak into the input.
    command = command.trim();
    if (_processingUserRequest) {
      _logger.printTrace('Ignoring terminal input: "$command" because we are busy.');
      return;
    }
    _processingUserRequest = true;
    try {
      lastReceivedCommand = command;
      await _commonTerminalInputHandler(command);
      // Catch all exception since this is doing cleanup and rethrowing.
    } catch (error, st) {
      // Don't print stack traces for known error types.
      if (error is! ToolExit) {
        _logger.printError('$error\n$st');
      }
      await _cleanUp(null);
      rethrow;
    } finally {
      _processingUserRequest = false;
      if (_reportReady) {
        _logger.printStatus('ready');
      }
    }
  }

  Future<void> _handleSignal(io.ProcessSignal signal) async {
    if (_processingUserRequest) {
      _logger.printTrace('Ignoring signal: "$signal" because we are busy.');
      return;
    }
    _processingUserRequest = true;

    final bool fullRestart = signal == io.ProcessSignal.sigusr2;

    try {
      await residentRunner.restart(fullRestart: fullRestart);
    } finally {
      _processingUserRequest = false;
    }
  }

  Future<void> _cleanUp(io.ProcessSignal? signal) async {
    _terminal.singleCharMode = false;
    await subscription?.cancel();
    await residentRunner.cleanupAfterSignal();
  }
}

class DebugConnectionInfo {
  DebugConnectionInfo({this.httpUri, this.wsUri, this.baseUri});

  final Uri? httpUri;
  final Uri? wsUri;
  final String? baseUri;
}

/// Returns the next platform value for the switcher.
///
/// These values must match what is available in
/// `packages/flutter/lib/src/foundation/platform.dart`.
String nextPlatform(String currentPlatform) {
  // The following lines are read by a script, which expects a certain format.
  // dart format off
  const List<String> platforms = <String>[
    'android',
    'iOS',
    'windows',
    'macOS',
    'linux',
    'fuchsia',
  ];
  // dart format on
  final int index = platforms.indexOf(currentPlatform);
  assert(index >= 0, 'unknown platform "$currentPlatform"');
  return platforms[(index + 1) % platforms.length];
}

/// A launcher for the devtools debugger and analysis tool.
abstract class DevtoolsLauncher {
  static DevtoolsLauncher? get instance => context.get<DevtoolsLauncher>();

  /// Serve Dart DevTools and return the host and port they are available on.
  ///
  /// This method must return a future that is guaranteed not to fail, because it
  /// will be used in unawaited contexts. It may, however, return null.
  Future<DevToolsServerAddress?> serve();

  /// Launch a Dart DevTools process, optionally targeting a specific VM Service
  /// URI if [vmServiceUri] is non-null.
  ///
  /// [additionalArguments] may be optionally specified and are passed directly
  /// to the devtools run command.
  ///
  /// This method must return a future that is guaranteed not to fail, because it
  /// will be used in unawaited contexts.
  Future<void> launch(Uri vmServiceUri, {List<String>? additionalArguments});

  Future<void> close();

  /// When measuring devtools memory via additional arguments, the launch process
  /// will technically never complete.
  ///
  /// Us this as an indicator that the process has started.
  Future<void>? processStart;

  /// Returns a future that completes when the DevTools server is ready.
  ///
  /// Completes when [devToolsUrl] is set. That can be set either directly, or
  /// by calling [serve].
  Future<void> get ready => _readyCompleter.future;
  Completer<void> _readyCompleter = Completer<void>();

  Uri? get devToolsUrl => _devToolsUrl;
  Uri? _devToolsUrl;
  set devToolsUrl(Uri? value) {
    assert((_devToolsUrl == null) != (value == null));
    _devToolsUrl = value;
    if (_devToolsUrl != null) {
      _readyCompleter.complete();
    } else {
      _readyCompleter = Completer<void>();
    }
  }

  /// The Dart Tooling Daemon (DTD) URI for the DTD instance being hosted by
  /// DevTools server.
  ///
  /// This will be null if the DevTools server is not served through Flutter
  /// tools (e.g. if it is served from an IDE).
  Uri? get dtdUri => _dtdUri;
  Uri? _dtdUri;
  @protected
  set dtdUri(Uri? value) => _dtdUri = value;

  // TODO(bkonyi): remove when ready to serve DevTools from DDS.
  /// Whether to print the Dart Tooling Daemon URI.
  ///
  /// This will always return false when there is not a DTD instance being
  /// served from the DevTools server.
  bool get printDtdUri => _printDtdUri ?? false;
  bool? _printDtdUri;
  set printDtdUri(bool value) {
    _printDtdUri = value;
  }

  /// The URL of the current DevTools server.
  ///
  /// Returns null if [ready] is not complete.
  DevToolsServerAddress? get activeDevToolsServer {
    if (_devToolsUrl == null) {
      return null;
    }
    return DevToolsServerAddress(devToolsUrl!.host, devToolsUrl!.port);
  }
}

class DevToolsServerAddress {
  DevToolsServerAddress(this.host, this.port);

  final String host;
  final int port;

  Uri? get uri {
    return Uri(scheme: 'http', host: host, port: port);
  }
}
