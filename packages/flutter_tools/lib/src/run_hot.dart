// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';
import 'package:pool/pool.dart';
import 'package:unified_analytics/unified_analytics.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import 'base/context.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/platform.dart';
import 'base/utils.dart';
import 'build_info.dart';
import 'compile.dart';
import 'convert.dart';
import 'devfs.dart';
import 'device.dart';
import 'globals.dart' as globals;
import 'project.dart';
import 'reporting/reporting.dart';
import 'resident_runner.dart';
import 'vmservice.dart';

ProjectFileInvalidator get projectFileInvalidator =>
    context.get<ProjectFileInvalidator>() ??
    ProjectFileInvalidator(
      fileSystem: globals.fs,
      platform: globals.platform,
      logger: globals.logger,
    );

HotRunnerConfig? get hotRunnerConfig => context.get<HotRunnerConfig>();

class HotRunnerConfig {
  /// Should the hot runner assume that the minimal Dart dependencies do not change?
  bool stableDartDependencies = false;

  /// Whether the hot runner should scan for modified files asynchronously.
  bool asyncScanning = false;

  /// A hook for implementations to perform any necessary initialization prior
  /// to a hot restart. Should return true if the hot restart should continue.
  Future<bool?> setupHotRestart() async {
    return true;
  }

  /// A hook for implementations to perform any necessary initialization prior
  /// to a hot reload. Should return true if the hot restart should continue.
  Future<bool?> setupHotReload() async {
    return true;
  }

  /// A hook for implementations to perform any necessary cleanup after the
  /// devfs sync is complete. At this point the flutter_tools no longer needs to
  /// access the source files and assets.
  void updateDevFSComplete() {}

  /// A hook for implementations to perform any necessary operations right
  /// before the runner is about to be shut down.
  Future<void> runPreShutdownOperations() async {
    return;
  }
}

const bool kHotReloadDefault = true;

class DeviceReloadReport {
  DeviceReloadReport(this.device, this.reports);

  FlutterDevice? device;
  List<vm_service.ReloadReport> reports; // List has one report per Flutter view.
}

class HotRunner extends ResidentRunner {
  HotRunner(
    super.flutterDevices, {
    required super.target,
    required super.debuggingOptions,
    this.benchmarkMode = false,
    this.applicationBinary,
    this.hostIsIde = false,
    super.projectRootPath,
    super.dillOutputPath,
    super.stayResident,
    super.machine,
    super.devtoolsHandler,
    StopwatchFactory stopwatchFactory = const StopwatchFactory(),
    ReloadSourcesHelper reloadSourcesHelper = defaultReloadSourcesHelper,
    ReassembleHelper reassembleHelper = _defaultReassembleHelper,
    String? nativeAssetsYamlFile,
    required Analytics analytics,
  }) : _stopwatchFactory = stopwatchFactory,
       _reloadSourcesHelper = reloadSourcesHelper,
       _reassembleHelper = reassembleHelper,
       _nativeAssetsYamlFile = nativeAssetsYamlFile,
       _analytics = analytics,
       super(hotMode: true);

  final StopwatchFactory _stopwatchFactory;
  final ReloadSourcesHelper _reloadSourcesHelper;
  final ReassembleHelper _reassembleHelper;
  final Analytics _analytics;

  final bool benchmarkMode;
  final File? applicationBinary;
  final bool hostIsIde;

  /// When performing a hot restart, the tool needs to upload a new main.dart.dill to
  /// each attached device's devfs. Replacing the existing file is not safe and does
  /// not work at all on the windows embedder, because the old dill file will still be
  /// memory-mapped by the embedder. To work around this issue, the tool will alternate
  /// names for the uploaded dill, sometimes inserting `.swap`. Since the active dill will
  /// never be replaced, there is no risk of writing the file while the embedder is attempting
  /// to read from it. This also avoids filling up the devfs, if a incrementing counter was
  /// used instead.
  ///
  /// This is only used for hot restart, incremental dills uploaded as part of the hot
  /// reload process do not have this issue.
  bool _swap = false;

  final Map<String, List<int>> benchmarkData = <String, List<int>>{};

  String? _targetPlatform;
  String? _sdkName;
  bool? _emulator;

  final String? _nativeAssetsYamlFile;

  String? flavor;

  Future<void> _calculateTargetPlatform() async {
    if (_targetPlatform != null) {
      return;
    }

    switch (flutterDevices.length) {
      case 1:
        final Device device = flutterDevices.first.device!;
        _targetPlatform = getNameForTargetPlatform(await device.targetPlatform);
        _sdkName = await device.sdkNameAndVersion;
        _emulator = await device.isLocalEmulator;
      case > 1:
        _targetPlatform = 'multiple';
        _sdkName = 'multiple';
        _emulator = false;
      default:
        _targetPlatform = 'unknown';
        _sdkName = 'unknown';
        _emulator = false;
    }
  }

  void _addBenchmarkData(String name, int value) {
    benchmarkData[name] ??= <int>[];
    benchmarkData[name]!.add(value);
  }

  Future<void> _reloadSourcesService(
    String isolateId, {
    bool force = false,
    bool pause = false,
  }) async {
    final OperationResult result = await restart(pause: pause);
    if (!result.isOk) {
      throw vm_service.RPCError(
        'Unable to reload sources',
        vm_service.RPCErrorKind.kInternalError.code,
        '',
      );
    }
  }

  Future<void> _restartService({bool pause = false}) async {
    final OperationResult result = await restart(fullRestart: true, pause: pause);
    if (!result.isOk) {
      throw vm_service.RPCError(
        'Unable to restart',
        vm_service.RPCErrorKind.kInternalError.code,
        '',
      );
    }
  }

  Future<String> _compileExpressionService(
    String isolateId,
    String expression,
    List<String> definitions,
    List<String> definitionTypes,
    List<String> typeDefinitions,
    List<String> typeBounds,
    List<String> typeDefaults,
    String libraryUri,
    String? klass,
    String? method,
    bool isStatic,
  ) async {
    for (final FlutterDevice? device in flutterDevices) {
      if (device!.generator != null) {
        final CompilerOutput? compilerOutput = await device.generator!.compileExpression(
          expression,
          definitions,
          definitionTypes,
          typeDefinitions,
          typeBounds,
          typeDefaults,
          libraryUri,
          klass,
          method,
          isStatic,
        );
        if (compilerOutput != null) {
          if (compilerOutput.errorCount == 0 && compilerOutput.expressionData != null) {
            return base64.encode(compilerOutput.expressionData!);
          } else if (compilerOutput.errorCount > 0 && compilerOutput.errorMessage != null) {
            throw VmServiceExpressionCompilationException(compilerOutput.errorMessage!);
          }
        }
      }
    }
    throw Exception('Failed to compile $expression');
  }

  @override
  @nonVirtual
  Future<int> attach({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    bool allowExistingDdsInstance = false,
    bool needsFullRestart = true,
  }) async {
    stopAppDuringCleanup = false;
    return _attach(
      connectionInfoCompleter: connectionInfoCompleter,
      appStartedCompleter: appStartedCompleter,
      allowExistingDdsInstance: allowExistingDdsInstance,
      needsFullRestart: needsFullRestart,
    );
  }

  Future<int> _attach({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    bool allowExistingDdsInstance = false,
    bool needsFullRestart = true,
  }) async {
    try {
      await connectToServiceProtocol(
        reloadSources: _reloadSourcesService,
        restart: _restartService,
        compileExpression: _compileExpressionService,
        getSkSLMethod: writeSkSL,
        allowExistingDdsInstance: allowExistingDdsInstance,
      );
      // Catches all exceptions, non-Exception objects are rethrown.
    } catch (error) {
      // ignore: avoid_catches_without_on_clauses
      if (error is! Exception && error is! String) {
        rethrow;
      }
      globals.printError('Error connecting to the service protocol: $error');
      return 2;
    }

    if (debuggingOptions.serveObservatory) {
      await enableObservatory();
    }

    // TODO(bkonyi): remove when ready to serve DevTools from DDS.
    if (debuggingOptions.enableDevTools) {
      // The method below is guaranteed never to return a failing future.
      unawaited(
        residentDevtoolsHandler!.serveAndAnnounceDevTools(
          devToolsServerAddress: debuggingOptions.devToolsServerAddress,
          flutterDevices: flutterDevices,
          isStartPaused: debuggingOptions.startPaused,
        ),
      );
    }

    for (final FlutterDevice? device in flutterDevices) {
      device!.developmentShaderCompiler.configureCompiler(device.targetPlatform);
    }
    try {
      final List<Uri?> baseUris = await _initDevFS();
      if (connectionInfoCompleter != null) {
        // Only handle one debugger connection.
        connectionInfoCompleter.complete(
          DebugConnectionInfo(
            httpUri: flutterDevices.first.vmService!.httpAddress,
            wsUri: flutterDevices.first.vmService!.wsAddress,
            baseUri: baseUris.first.toString(),
          ),
        );
      }
    } on DevFSException catch (error) {
      globals.printError('Error initializing DevFS: $error');
      return 3;
    }

    final Stopwatch initialUpdateDevFSsTimer = Stopwatch()..start();
    final UpdateFSReport devfsResult = await _updateDevFS(fullRestart: needsFullRestart);
    _addBenchmarkData(
      'hotReloadInitialDevFSSyncMilliseconds',
      initialUpdateDevFSsTimer.elapsed.inMilliseconds,
    );
    if (!devfsResult.success) {
      return 3;
    }

    for (final FlutterDevice? device in flutterDevices) {
      // VM must have accepted the kernel binary, there will be no reload
      // report, so we let incremental compiler know that source code was accepted.
      if (device!.generator != null) {
        device.generator!.accept();
      }
      final List<FlutterView> views = await device.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        globals.printTrace('Connected to $view.');
      }
    }

    // In fast-start mode, apps are initialized from a placeholder splashscreen
    // app. We must do a restart here to load the program and assets for the
    // real app.
    if (debuggingOptions.fastStart) {
      await restart(fullRestart: true, reason: 'restart', silent: true);
    }

    appStartedCompleter?.complete();

    if (benchmarkMode) {
      // Wait multiple seconds for the isolate to have fully started.
      await Future<void>.delayed(const Duration(seconds: 10));
      // We are running in benchmark mode.
      globals.printStatus('Running in benchmark mode.');
      // Measure time to perform a hot restart.
      globals.printStatus('Benchmarking hot restart');
      await restart(fullRestart: true);
      // Wait multiple seconds to stabilize benchmark on slower device lab hardware.
      // Hot restart finishes when the new isolate is started, not when the new isolate
      // is ready. This process can actually take multiple seconds.
      await Future<void>.delayed(const Duration(seconds: 10));

      globals.printStatus('Benchmarking hot reload');
      // Measure time to perform a hot reload.
      await restart();
      if (stayResident) {
        await waitForAppToFinish();
      } else {
        globals.printStatus('Benchmark completed. Exiting application.');
        await _cleanupDevFS();
        await stopEchoingDeviceLog();
        await exitApp();
      }
      final File benchmarkOutput = globals.fs.file('hot_benchmark.json');
      benchmarkOutput.writeAsStringSync(toPrettyJson(benchmarkData));
      return 0;
    }
    writeVmServiceFile();

    int result = 0;
    if (stayResident) {
      result = await waitForAppToFinish();
    }
    await cleanupAtFinish();
    return result;
  }

  @override
  Future<int> run({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    String? route,
  }) async {
    await _calculateTargetPlatform();

    final Uri? nativeAssetsYaml =
        _nativeAssetsYamlFile != null ? globals.fs.path.toUri(_nativeAssetsYamlFile) : null;

    final Stopwatch appStartedTimer = Stopwatch()..start();
    final File mainFile = globals.fs.file(mainPath);

    Duration totalCompileTime = Duration.zero;
    Duration totalLaunchAppTime = Duration.zero;

    final List<Future<bool>> startupTasks = <Future<bool>>[];
    for (final FlutterDevice? device in flutterDevices) {
      // Here we initialize the frontend_server concurrently with the platform
      // build, reducing overall initialization time. This is safe because the first
      // invocation of the frontend server produces a full dill file that the
      // subsequent invocation in devfs will not overwrite.
      await runSourceGenerators();
      if (device!.generator != null) {
        final Stopwatch compileTimer = Stopwatch()..start();
        startupTasks.add(
          device.generator!
              .recompile(
                mainFile.uri,
                <Uri>[],
                // When running without a provided applicationBinary, the tool will
                // simultaneously run the initial frontend_server compilation and
                // the native build step. If there is a Dart compilation error, it
                // should only be displayed once.
                suppressErrors: applicationBinary == null,
                checkDartPluginRegistry: true,
                dartPluginRegistrant: FlutterProject.current().dartPluginRegistrant,
                outputPath: dillOutputPath,
                packageConfig: debuggingOptions.buildInfo.packageConfig,
                projectRootPath: FlutterProject.current().directory.absolute.path,
                fs: globals.fs,
                nativeAssetsYaml: nativeAssetsYaml,
              )
              .then((CompilerOutput? output) {
                compileTimer.stop();
                totalCompileTime += compileTimer.elapsed;
                return output?.errorCount == 0;
              }),
        );
      }

      final Stopwatch launchAppTimer = Stopwatch()..start();
      startupTasks.add(
        device.runHot(hotRunner: this, route: route).then((int result) {
          totalLaunchAppTime += launchAppTimer.elapsed;
          return result == 0;
        }),
      );
    }

    unawaited(
      appStartedCompleter?.future.then((_) {
        HotEvent(
          'reload-ready',
          targetPlatform: _targetPlatform!,
          sdkName: _sdkName!,
          emulator: _emulator!,
          fullRestart: false,
          overallTimeInMs: appStartedTimer.elapsed.inMilliseconds,
          compileTimeInMs: totalCompileTime.inMilliseconds,
          transferTimeInMs: totalLaunchAppTime.inMilliseconds,
        ).send();

        _analytics.send(
          Event.hotRunnerInfo(
            label: 'reload-ready',
            targetPlatform: _targetPlatform!,
            sdkName: _sdkName!,
            emulator: _emulator!,
            fullRestart: false,
            overallTimeInMs: appStartedTimer.elapsed.inMilliseconds,
            compileTimeInMs: totalCompileTime.inMilliseconds,
            transferTimeInMs: totalLaunchAppTime.inMilliseconds,
          ),
        );
      }),
    );

    try {
      final List<bool> results = await Future.wait(startupTasks);
      if (!results.every((bool passed) => passed)) {
        appFailedToStart();
        return 1;
      }
      cacheInitialDillCompilation();
    } on Exception catch (err) {
      globals.printError(err.toString());
      appFailedToStart();
      return 1;
    }

    return _attach(
      connectionInfoCompleter: connectionInfoCompleter,
      appStartedCompleter: appStartedCompleter,
      needsFullRestart: false,
    );
  }

  Future<List<Uri?>> _initDevFS() async {
    final String fsName = globals.fs.path.basename(projectRootPath);
    return <Uri?>[
      for (final FlutterDevice? device in flutterDevices)
        await device!.setupDevFS(fsName, globals.fs.directory(projectRootPath)),
    ];
  }

  Future<UpdateFSReport> _updateDevFS({bool fullRestart = false}) async {
    final bool isFirstUpload = !assetBundle.wasBuiltOnce();
    final bool rebuildBundle = assetBundle.needsBuild();
    if (rebuildBundle) {
      globals.printTrace('Updating assets');
      final int result = await assetBundle.build(
        packageConfigPath: debuggingOptions.buildInfo.packageConfigPath,
        flavor: debuggingOptions.buildInfo.flavor,
      );
      if (result != 0) {
        return UpdateFSReport();
      }
    }

    final Stopwatch findInvalidationTimer = _stopwatchFactory.createStopwatch('updateDevFS')
      ..start();
    final DevFS devFS = flutterDevices[0].devFS!;
    final InvalidationResult invalidationResult = await projectFileInvalidator.findInvalidated(
      lastCompiled: devFS.lastCompiled,
      urisToMonitor: devFS.sources,
      packagesPath: packagesFilePath,
      asyncScanning: hotRunnerConfig!.asyncScanning,
      packageConfig: devFS.lastPackageConfig ?? debuggingOptions.buildInfo.packageConfig,
    );
    findInvalidationTimer.stop();
    final File entrypointFile = globals.fs.file(mainPath);
    if (!entrypointFile.existsSync()) {
      globals.printError(
        'The entrypoint file (i.e. the file with main()) ${entrypointFile.path} '
        'cannot be found. Moving or renaming this file will prevent changes to '
        'its contents from being discovered during hot reload/restart until '
        'flutter is restarted or the file is restored.',
      );
    }
    final UpdateFSReport results = UpdateFSReport(
      success: true,
      scannedSourcesCount: devFS.sources.length,
      findInvalidatedDuration: findInvalidationTimer.elapsed,
    );
    for (final FlutterDevice? device in flutterDevices) {
      results.incorporateResults(
        await device!.updateDevFS(
          mainUri: entrypointFile.absolute.uri,
          target: target,
          bundle: assetBundle,
          bundleFirstUpload: isFirstUpload,
          bundleDirty: !isFirstUpload && rebuildBundle,
          fullRestart: fullRestart,
          pathToReload: getReloadPath(fullRestart: fullRestart, swap: _swap),
          invalidatedFiles: invalidationResult.uris!,
          packageConfig: invalidationResult.packageConfig!,
          dillOutputPath: dillOutputPath,
        ),
      );
    }
    return results;
  }

  void _resetDirtyAssets() {
    for (final FlutterDevice device in flutterDevices) {
      final DevFS? devFS = device.devFS;
      if (devFS == null) {
        // This is sometimes null, however we don't know why and have not been
        // able to reproduce, https://github.com/flutter/flutter/issues/108653
        continue;
      }
      devFS.assetPathsToEvict.clear();
      devFS.shaderPathsToEvict.clear();
      devFS.scenePathsToEvict.clear();
    }
  }

  Future<void> _cleanupDevFS() async {
    final List<Future<void>> futures = <Future<void>>[];
    for (final FlutterDevice device in flutterDevices) {
      if (device.devFS != null) {
        // Cleanup the devFS, but don't wait indefinitely.
        // We ignore any errors, because it's not clear what we would do anyway.
        futures.add(
          device.devFS!
              .destroy()
              .timeout(const Duration(milliseconds: 250))
              .then<void>(
                (Object? _) {},
                onError: (Object? error, StackTrace stackTrace) {
                  globals.printTrace('Ignored error while cleaning up DevFS: $error\n$stackTrace');
                },
              ),
        );
      }
      device.devFS = null;
    }
    await Future.wait(futures);
  }

  Future<void> _launchInView(FlutterDevice device, Uri main, Uri assetsDirectory) async {
    final List<FlutterView> views = await device.vmService!.getFlutterViews();
    await Future.wait(<Future<void>>[
      for (final FlutterView view in views)
        device.vmService!.runInView(viewId: view.id, main: main, assetsDirectory: assetsDirectory),
    ]);
  }

  Future<void> _launchFromDevFS() async {
    final List<Future<void>> futures = <Future<void>>[];
    for (final FlutterDevice? device in flutterDevices) {
      final Uri deviceEntryUri = device!.devFS!.baseUri!.resolve(
        _swap ? 'main.dart.swap.dill' : 'main.dart.dill',
      );
      final Uri deviceAssetsDirectoryUri = device.devFS!.baseUri!.resolveUri(
        globals.fs.path.toUri(getAssetBuildDirectory()),
      );
      futures.add(_launchInView(device, deviceEntryUri, deviceAssetsDirectoryUri));
    }
    await Future.wait(futures);
  }

  Future<OperationResult> _restartFromSources({String? reason}) async {
    final Stopwatch restartTimer = Stopwatch()..start();
    UpdateFSReport updatedDevFS;
    try {
      updatedDevFS = await _updateDevFS(fullRestart: true);
    } finally {
      hotRunnerConfig!.updateDevFSComplete();
    }
    if (!updatedDevFS.success) {
      for (final FlutterDevice? device in flutterDevices) {
        if (device!.generator != null) {
          await device.generator!.reject();
        }
      }
      return OperationResult(1, 'DevFS synchronization failed');
    }
    _resetDirtyAssets();
    for (final FlutterDevice? device in flutterDevices) {
      // VM must have accepted the kernel binary, there will be no reload
      // report, so we let incremental compiler know that source code was accepted.
      if (device!.generator != null) {
        device.generator!.accept();
      }
    }
    // Check if the isolate is paused and resume it.
    final List<Future<void>> operations = <Future<void>>[];
    for (final FlutterDevice? device in flutterDevices) {
      final Set<String?> uiIsolatesIds = <String?>{};
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      for (final FlutterView view in views) {
        if (view.uiIsolate == null) {
          continue;
        }
        uiIsolatesIds.add(view.uiIsolate!.id);
        // Reload the isolate.
        final Future<vm_service.Isolate?> reloadIsolate = device.vmService!.getIsolateOrNull(
          view.uiIsolate!.id!,
        );
        operations.add(
          reloadIsolate.then((vm_service.Isolate? isolate) async {
            if ((isolate != null) && isPauseEvent(isolate.pauseEvent!.kind!)) {
              // The embedder requires that the isolate is unpaused, because the
              // runInView method requires interaction with dart engine APIs that
              // are not thread-safe, and thus must be run on the same thread that
              // would be blocked by the pause. Simply un-pausing is not sufficient,
              // because this does not prevent the isolate from immediately hitting
              // a breakpoint (for example if the breakpoint was placed in a loop
              // or in a frequently called method) or an exception. Instead, all
              // breakpoints are first disabled and exception pause mode set to
              // None, and then the isolate resumed.
              // These settings to not need restoring as Hot Restart results in
              // new isolates, which will be configured by the editor as they are
              // started.
              final List<Future<void>> breakpointAndExceptionRemoval = <Future<void>>[
                device.vmService!.service.setIsolatePauseMode(
                  isolate.id!,
                  exceptionPauseMode: vm_service.ExceptionPauseMode.kNone,
                ),
                for (final vm_service.Breakpoint breakpoint in isolate.breakpoints!)
                  device.vmService!.service.removeBreakpoint(isolate.id!, breakpoint.id!),
              ];
              await Future.wait(breakpointAndExceptionRemoval);
              await device.vmService!.service.resume(view.uiIsolate!.id!);
            }
          }),
        );
      }

      // The engine handles killing and recreating isolates that it has spawned
      // ("uiIsolates"). The isolates that were spawned from these uiIsolates
      // will not be restarted, and so they must be manually killed.
      final vm_service.VM vm = await device.vmService!.service.getVM();
      for (final vm_service.IsolateRef isolateRef in vm.isolates!) {
        if (uiIsolatesIds.contains(isolateRef.id)) {
          continue;
        }
        operations.add(
          device.vmService!.service
              .kill(isolateRef.id!)
              // Since we never check the value of this Future, only await its
              // completion, make its type nullable so we can return null when
              // catching errors.
              .then<vm_service.Success?>(
                (vm_service.Success success) => success,
                onError: (Object error, StackTrace stackTrace) {
                  if (error is vm_service.SentinelException ||
                      (error is vm_service.RPCError && error.code == 105)) {
                    // Do nothing on a SentinelException since it means the isolate
                    // has already been killed.
                    // Error code 105 indicates the isolate is not yet runnable, and might
                    // be triggered if the tool is attempting to kill the asset parsing
                    // isolate before it has finished starting up.
                    return null;
                  }
                  return Future<vm_service.Success?>.error(error, stackTrace);
                },
              ),
        );
      }
    }
    await Future.wait(operations);
    globals.printTrace('Finished waiting on operations.');
    await _launchFromDevFS();
    restartTimer.stop();
    globals.printTrace(
      'Hot restart performed in ${getElapsedAsMilliseconds(restartTimer.elapsed)}.',
    );
    _addBenchmarkData('hotRestartMillisecondsToFrame', restartTimer.elapsed.inMilliseconds);

    // Send timing analytics.
    final Duration elapsedDuration = restartTimer.elapsed;
    globals.flutterUsage.sendTiming('hot', 'restart', elapsedDuration);
    _analytics.send(
      Event.timing(
        workflow: 'hot',
        variableName: 'restart',
        elapsedMilliseconds: elapsedDuration.inMilliseconds,
      ),
    );

    // Toggle the main dill name after successfully uploading.
    _swap = !_swap;

    return OperationResult(
      OperationResult.ok.code,
      OperationResult.ok.message,
      updateFSReport: updatedDevFS,
    );
  }

  /// Returns [true] if the reload was successful.
  /// Prints errors if [printErrors] is [true].
  static bool validateReloadReport(
    vm_service.ReloadReport? reloadReport, {
    bool printErrors = true,
  }) {
    if (reloadReport == null) {
      if (printErrors) {
        globals.printError('Hot reload did not receive reload report.');
      }
      return false;
    }
    final ReloadReportContents contents = ReloadReportContents.fromReloadReport(reloadReport);
    if (!reloadReport.success!) {
      if (printErrors) {
        globals.printError('Hot reload was rejected:');
        for (final ReasonForCancelling reason in contents.notices) {
          globals.printError(reason.toString());
        }
      }
      return false;
    }
    return true;
  }

  @override
  Future<OperationResult> restart({
    bool fullRestart = false,
    String? reason,
    bool silent = false,
    bool pause = false,
  }) async {
    if (flutterDevices.any((FlutterDevice? device) => device!.devFS == null)) {
      return OperationResult(1, 'Device initialization has not completed.');
    }
    await _calculateTargetPlatform();
    final Stopwatch timer = Stopwatch()..start();

    // Run source generation if needed.
    await runSourceGenerators();

    if (fullRestart) {
      final OperationResult result = await _fullRestartHelper(
        targetPlatform: _targetPlatform,
        sdkName: _sdkName,
        emulator: _emulator,
        reason: reason,
        silent: silent,
      );
      if (!silent) {
        globals.printStatus('Restarted application in ${getElapsedAsMilliseconds(timer.elapsed)}.');
      }
      // TODO(bkonyi): remove when ready to serve DevTools from DDS.
      unawaited(residentDevtoolsHandler!.hotRestart(flutterDevices));
      // for (final FlutterDevice? device in flutterDevices) {
      //   unawaited(device?.handleHotRestart());
      // }
      return result;
    }
    final OperationResult result = await _hotReloadHelper(
      targetPlatform: _targetPlatform,
      sdkName: _sdkName,
      emulator: _emulator,
      reason: reason,
      pause: pause,
    );
    if (result.isOk) {
      final String elapsed = getElapsedAsMilliseconds(timer.elapsed);
      if (!silent) {
        if (result.extraTimings.isNotEmpty) {
          final String extraTimingsString = result.extraTimings
              .map((OperationResultExtraTiming e) => '${e.description}: ${e.timeInMs} ms')
              .join(', ');
          globals.printStatus('${result.message} in $elapsed ($extraTimingsString).');
        } else {
          globals.printStatus('${result.message} in $elapsed.');
        }
      }
    }
    return result;
  }

  Future<OperationResult> _fullRestartHelper({
    String? targetPlatform,
    String? sdkName,
    bool? emulator,
    String? reason,
    bool? silent,
  }) async {
    if (!supportsRestart) {
      return OperationResult(1, 'hotRestart not supported');
    }
    Status? status;
    if (!silent!) {
      status = globals.logger.startProgress('Performing hot restart...', progressId: 'hot.restart');
    }
    OperationResult result;
    String? restartEvent;
    try {
      final Stopwatch restartTimer = _stopwatchFactory.createStopwatch('fullRestartHelper')
        ..start();
      if ((await hotRunnerConfig!.setupHotRestart()) != true) {
        return OperationResult(1, 'setupHotRestart failed');
      }
      result = await _restartFromSources(reason: reason);
      restartTimer.stop();
      if (!result.isOk) {
        restartEvent = 'restart-failed';
      } else {
        HotEvent(
          'restart',
          targetPlatform: targetPlatform!,
          sdkName: sdkName!,
          emulator: emulator!,
          fullRestart: true,
          reason: reason,
          overallTimeInMs: restartTimer.elapsed.inMilliseconds,
          syncedBytes: result.updateFSReport?.syncedBytes,
          invalidatedSourcesCount: result.updateFSReport?.invalidatedSourcesCount,
          transferTimeInMs: result.updateFSReport?.transferDuration.inMilliseconds,
          compileTimeInMs: result.updateFSReport?.compileDuration.inMilliseconds,
          findInvalidatedTimeInMs: result.updateFSReport?.findInvalidatedDuration.inMilliseconds,
          scannedSourcesCount: result.updateFSReport?.scannedSourcesCount,
        ).send();
        _analytics.send(
          Event.hotRunnerInfo(
            label: 'restart',
            targetPlatform: targetPlatform,
            sdkName: sdkName,
            emulator: emulator,
            fullRestart: true,
            reason: reason,
            overallTimeInMs: restartTimer.elapsed.inMilliseconds,
            syncedBytes: result.updateFSReport?.syncedBytes,
            invalidatedSourcesCount: result.updateFSReport?.invalidatedSourcesCount,
            transferTimeInMs: result.updateFSReport?.transferDuration.inMilliseconds,
            compileTimeInMs: result.updateFSReport?.compileDuration.inMilliseconds,
            findInvalidatedTimeInMs: result.updateFSReport?.findInvalidatedDuration.inMilliseconds,
            scannedSourcesCount: result.updateFSReport?.scannedSourcesCount,
          ),
        );
      }
    } on vm_service.SentinelException catch (err, st) {
      restartEvent = 'exception';
      return OperationResult(1, 'hot restart failed to complete: $err\n$st', fatal: true);
    } on vm_service.RPCError catch (err, st) {
      restartEvent = 'exception';
      return OperationResult(1, 'hot restart failed to complete: $err\n$st', fatal: true);
    } finally {
      // The `restartEvent` variable will be null if restart succeeded. We will
      // only handle the case when it failed here.
      if (restartEvent != null) {
        HotEvent(
          restartEvent,
          targetPlatform: targetPlatform!,
          sdkName: sdkName!,
          emulator: emulator!,
          fullRestart: true,
          reason: reason,
        ).send();
        _analytics.send(
          Event.hotRunnerInfo(
            label: restartEvent,
            targetPlatform: targetPlatform,
            sdkName: sdkName,
            emulator: emulator,
            fullRestart: true,
            reason: reason,
          ),
        );
      }
      status?.cancel();
    }
    return result;
  }

  Future<OperationResult> _hotReloadHelper({
    String? targetPlatform,
    String? sdkName,
    bool? emulator,
    String? reason,
    bool? pause,
  }) async {
    Status status = globals.logger.startProgress(
      'Performing hot reload...',
      progressId: 'hot.reload',
    );
    OperationResult result;
    try {
      result = await _reloadSources(
        targetPlatform: targetPlatform,
        sdkName: sdkName,
        emulator: emulator,
        reason: reason,
        pause: pause,
        onSlow: (String message) {
          status.cancel();
          status = globals.logger.startProgress(message, progressId: 'hot.reload');
        },
      );
    } on vm_service.RPCError catch (error) {
      String errorMessage = 'hot reload failed to complete';
      int errorCode = 1;
      if (error.code == kIsolateReloadBarred) {
        errorCode = error.code;
        errorMessage =
            'Unable to hot reload application due to an unrecoverable error in '
            'the source code. Please address the error and then use "R" to '
            'restart the app.\n'
            '${error.message} (error code: ${error.code})';
        HotEvent(
          'reload-barred',
          targetPlatform: targetPlatform!,
          sdkName: sdkName!,
          emulator: emulator!,
          fullRestart: false,
          reason: reason,
        ).send();
        _analytics.send(
          Event.hotRunnerInfo(
            label: 'reload-barred',
            targetPlatform: targetPlatform,
            sdkName: sdkName,
            emulator: emulator,
            fullRestart: false,
            reason: reason,
          ),
        );
      } else {
        HotEvent(
          'exception',
          targetPlatform: targetPlatform!,
          sdkName: sdkName!,
          emulator: emulator!,
          fullRestart: false,
          reason: reason,
        ).send();
        _analytics.send(
          Event.hotRunnerInfo(
            label: 'exception',
            targetPlatform: targetPlatform,
            sdkName: sdkName,
            emulator: emulator,
            fullRestart: false,
            reason: reason,
          ),
        );
      }
      return OperationResult(errorCode, errorMessage, fatal: true);
    } finally {
      status.cancel();
    }
    return result;
  }

  Future<OperationResult> _reloadSources({
    String? targetPlatform,
    String? sdkName,
    bool? emulator,
    bool? pause = false,
    String? reason,
    void Function(String message)? onSlow,
  }) async {
    final Map<FlutterDevice?, List<FlutterView>> viewCache = <FlutterDevice?, List<FlutterView>>{};
    for (final FlutterDevice? device in flutterDevices) {
      final List<FlutterView> views = await device!.vmService!.getFlutterViews();
      viewCache[device] = views;
      for (final FlutterView view in views) {
        if (view.uiIsolate == null) {
          return OperationResult(2, 'Application isolate not found', fatal: true);
        }
      }
    }

    final Stopwatch reloadTimer = _stopwatchFactory.createStopwatch('reloadSources:reload')
      ..start();
    if ((await hotRunnerConfig!.setupHotReload()) != true) {
      return OperationResult(1, 'setupHotReload failed');
    }
    final Stopwatch devFSTimer = Stopwatch()..start();
    UpdateFSReport updatedDevFS;
    try {
      updatedDevFS = await _updateDevFS();
    } finally {
      hotRunnerConfig!.updateDevFSComplete();
    }
    // Record time it took to synchronize to DevFS.
    bool shouldReportReloadTime = true;
    _addBenchmarkData('hotReloadDevFSSyncMilliseconds', devFSTimer.elapsed.inMilliseconds);
    if (!updatedDevFS.success) {
      return OperationResult(1, 'DevFS synchronization failed');
    }

    final List<OperationResultExtraTiming> extraTimings = <OperationResultExtraTiming>[];
    extraTimings.add(
      OperationResultExtraTiming('compile', updatedDevFS.compileDuration.inMilliseconds),
    );

    String reloadMessage = 'Reloaded 0 libraries';
    final Stopwatch reloadVMTimer = _stopwatchFactory.createStopwatch('reloadSources:vm')..start();
    final Map<String, Object?> firstReloadDetails = <String, Object?>{};
    if (updatedDevFS.invalidatedSourcesCount > 0) {
      final OperationResult result = await _reloadSourcesHelper(
        this,
        flutterDevices,
        pause,
        firstReloadDetails,
        targetPlatform,
        sdkName,
        emulator,
        reason,
        globals.flutterUsage,
        globals.analytics,
      );
      if (result.code != 0) {
        return result;
      }
      reloadMessage = result.message;
    } else {
      _addBenchmarkData('hotReloadVMReloadMilliseconds', 0);
    }
    reloadVMTimer.stop();
    extraTimings.add(OperationResultExtraTiming('reload', reloadVMTimer.elapsedMilliseconds));

    await evictDirtyAssets();

    final Stopwatch reassembleTimer = _stopwatchFactory.createStopwatch('reloadSources:reassemble')
      ..start();

    final ReassembleResult reassembleResult = await _reassembleHelper(
      flutterDevices,
      viewCache,
      onSlow,
      reloadMessage,
    );
    shouldReportReloadTime = reassembleResult.shouldReportReloadTime;
    if (reassembleResult.reassembleViews.isEmpty) {
      return OperationResult(OperationResult.ok.code, reloadMessage);
    }
    // Record time it took for Flutter to reassemble the application.
    reassembleTimer.stop();
    _addBenchmarkData(
      'hotReloadFlutterReassembleMilliseconds',
      reassembleTimer.elapsed.inMilliseconds,
    );
    extraTimings.add(OperationResultExtraTiming('reassemble', reassembleTimer.elapsedMilliseconds));

    reloadTimer.stop();
    final Duration reloadDuration = reloadTimer.elapsed;
    final int reloadInMs = reloadDuration.inMilliseconds;

    // Collect stats that help understand scale of update for this hot reload request.
    // For example, [syncedLibraryCount]/[finalLibraryCount] indicates how
    // many libraries were affected by the hot reload request.
    // Relation of [invalidatedSourcesCount] to [syncedLibraryCount] should help
    // understand sync/transfer "overhead" of updating this number of source files.
    HotEvent(
      'reload',
      targetPlatform: targetPlatform!,
      sdkName: sdkName!,
      emulator: emulator!,
      fullRestart: false,
      reason: reason,
      overallTimeInMs: reloadInMs,
      finalLibraryCount: firstReloadDetails['finalLibraryCount'] as int? ?? 0,
      syncedLibraryCount: firstReloadDetails['receivedLibraryCount'] as int? ?? 0,
      syncedClassesCount: firstReloadDetails['receivedClassesCount'] as int? ?? 0,
      syncedProceduresCount: firstReloadDetails['receivedProceduresCount'] as int? ?? 0,
      syncedBytes: updatedDevFS.syncedBytes,
      invalidatedSourcesCount: updatedDevFS.invalidatedSourcesCount,
      transferTimeInMs: updatedDevFS.transferDuration.inMilliseconds,
      compileTimeInMs: updatedDevFS.compileDuration.inMilliseconds,
      findInvalidatedTimeInMs: updatedDevFS.findInvalidatedDuration.inMilliseconds,
      scannedSourcesCount: updatedDevFS.scannedSourcesCount,
      reassembleTimeInMs: reassembleTimer.elapsed.inMilliseconds,
      reloadVMTimeInMs: reloadVMTimer.elapsed.inMilliseconds,
    ).send();
    _analytics.send(
      Event.hotRunnerInfo(
        label: 'reload',
        targetPlatform: targetPlatform,
        sdkName: sdkName,
        emulator: emulator,
        fullRestart: false,
        reason: reason,
        overallTimeInMs: reloadInMs,
        finalLibraryCount: firstReloadDetails['finalLibraryCount'] as int? ?? 0,
        syncedLibraryCount: firstReloadDetails['receivedLibraryCount'] as int? ?? 0,
        syncedClassesCount: firstReloadDetails['receivedClassesCount'] as int? ?? 0,
        syncedProceduresCount: firstReloadDetails['receivedProceduresCount'] as int? ?? 0,
        syncedBytes: updatedDevFS.syncedBytes,
        invalidatedSourcesCount: updatedDevFS.invalidatedSourcesCount,
        transferTimeInMs: updatedDevFS.transferDuration.inMilliseconds,
        compileTimeInMs: updatedDevFS.compileDuration.inMilliseconds,
        findInvalidatedTimeInMs: updatedDevFS.findInvalidatedDuration.inMilliseconds,
        scannedSourcesCount: updatedDevFS.scannedSourcesCount,
        reassembleTimeInMs: reassembleTimer.elapsed.inMilliseconds,
        reloadVMTimeInMs: reloadVMTimer.elapsed.inMilliseconds,
      ),
    );

    if (shouldReportReloadTime) {
      globals.printTrace('Hot reload performed in ${getElapsedAsMilliseconds(reloadDuration)}.');
      // Record complete time it took for the reload.
      _addBenchmarkData('hotReloadMillisecondsToFrame', reloadInMs);
    }
    // Only report timings if we reloaded a single view without any errors.
    if ((reassembleResult.reassembleViews.length == 1) &&
        !reassembleResult.failedReassemble &&
        shouldReportReloadTime) {
      globals.flutterUsage.sendTiming('hot', 'reload', reloadDuration);
      _analytics.send(
        Event.timing(
          workflow: 'hot',
          variableName: 'reload',
          elapsedMilliseconds: reloadDuration.inMilliseconds,
        ),
      );
    }
    return OperationResult(
      reassembleResult.failedReassemble ? 1 : OperationResult.ok.code,
      reloadMessage,
      extraTimings: extraTimings,
    );
  }

  @override
  void printHelp({required bool details}) {
    globals.printStatus('Flutter run key commands.');
    commandHelp.r.print();
    if (supportsRestart) {
      commandHelp.R.print();
    }
    if (details) {
      printHelpDetails();
      commandHelp.hWithDetails.print();
    } else {
      commandHelp.hWithoutDetails.print();
    }
    if (stopAppDuringCleanup) {
      commandHelp.d.print();
    }
    commandHelp.c.print();
    commandHelp.q.print();
    if (debuggingOptions.buildInfo.nullSafetyMode != NullSafetyMode.sound) {
      globals.printStatus('');
      globals.printStatus('Running without sound null safety ⚠️', emphasis: true);
      globals.printStatus(
        'Dart 3 will only support sound null safety, see https://dart.dev/null-safety',
      );
    }
    globals.printStatus('');
    printDebuggerList();
  }

  @visibleForTesting
  Future<void> evictDirtyAssets() async {
    final List<Future<void>> futures = <Future<void>>[];
    for (final FlutterDevice? device in flutterDevices) {
      if (device!.devFS!.assetPathsToEvict.isEmpty &&
          device.devFS!.shaderPathsToEvict.isEmpty &&
          device.devFS!.scenePathsToEvict.isEmpty) {
        continue;
      }
      final List<FlutterView> views = await device.vmService!.getFlutterViews();

      // If this is the first time we update the assets, make sure to call the setAssetDirectory
      if (!device.devFS!.hasSetAssetDirectory) {
        final Uri deviceAssetsDirectoryUri = device.devFS!.baseUri!.resolveUri(
          globals.fs.path.toUri(getAssetBuildDirectory()),
        );
        await Future.wait<void>(
          views.map<Future<void>>(
            (FlutterView view) => device.vmService!.setAssetDirectory(
              assetsDirectory: deviceAssetsDirectoryUri,
              uiIsolateId: view.uiIsolate!.id,
              viewId: view.id,
              windows: device.targetPlatform == TargetPlatform.windows_x64,
            ),
          ),
        );
        for (final FlutterView view in views) {
          globals.printTrace('Set asset directory in $view.');
        }
        device.devFS!.hasSetAssetDirectory = true;
      }

      if (views.first.uiIsolate == null) {
        globals.printError('Application isolate not found for $device');
        continue;
      }

      if (device.devFS!.didUpdateFontManifest) {
        futures.add(
          device.vmService!.reloadAssetFonts(
            isolateId: views.first.uiIsolate!.id!,
            viewId: views.first.id,
          ),
        );
      }

      for (final String assetPath in device.devFS!.assetPathsToEvict) {
        futures.add(
          device.vmService!.flutterEvictAsset(assetPath, isolateId: views.first.uiIsolate!.id!),
        );
      }
      for (final String assetPath in device.devFS!.shaderPathsToEvict) {
        futures.add(
          device.vmService!.flutterEvictShader(assetPath, isolateId: views.first.uiIsolate!.id!),
        );
      }
      for (final String assetPath in device.devFS!.scenePathsToEvict) {
        futures.add(
          device.vmService!.flutterEvictScene(assetPath, isolateId: views.first.uiIsolate!.id!),
        );
      }
      device.devFS!.assetPathsToEvict.clear();
      device.devFS!.shaderPathsToEvict.clear();
      device.devFS!.scenePathsToEvict.clear();
    }
    await Future.wait<void>(futures);
  }

  @override
  Future<void> cleanupAfterSignal() async {
    await residentDevtoolsHandler!.shutdown();
    await stopEchoingDeviceLog();
    await hotRunnerConfig!.runPreShutdownOperations();
    shutdownDartDevelopmentService();
    if (stopAppDuringCleanup) {
      return exitApp();
    }
    appFinished();
  }

  @override
  Future<void> preExit() async {
    await _cleanupDevFS();
    await hotRunnerConfig!.runPreShutdownOperations();
    await super.preExit();
  }

  @override
  Future<void> cleanupAtFinish() async {
    for (final FlutterDevice? flutterDevice in flutterDevices) {
      await flutterDevice!.device!.dispose();
    }
    await _cleanupDevFS();
    await residentDevtoolsHandler!.shutdown();
    await stopEchoingDeviceLog();
  }
}

typedef ReloadSourcesHelper =
    Future<OperationResult> Function(
      HotRunner hotRunner,
      List<FlutterDevice?> flutterDevices,
      bool? pause,
      Map<String, dynamic> firstReloadDetails,
      String? targetPlatform,
      String? sdkName,
      bool? emulator,
      String? reason,
      Usage usage,
      Analytics analytics,
    );

@visibleForTesting
Future<OperationResult> defaultReloadSourcesHelper(
  HotRunner hotRunner,
  List<FlutterDevice?> flutterDevices,
  bool? pause,
  Map<String, dynamic> firstReloadDetails,
  String? targetPlatform,
  String? sdkName,
  bool? emulator,
  String? reason,
  Usage usage,
  Analytics analytics,
) async {
  final Stopwatch vmReloadTimer = Stopwatch()..start();
  const String entryPath = 'main.dart.incremental.dill';
  final List<Future<DeviceReloadReport?>> allReportsFutures = <Future<DeviceReloadReport?>>[];

  for (final FlutterDevice? device in flutterDevices) {
    final List<Future<vm_service.ReloadReport>> reportFutures = await _reloadDeviceSources(
      device!,
      entryPath,
      pause: pause,
    );
    allReportsFutures.add(
      Future.wait(reportFutures).then<DeviceReloadReport?>((
        List<vm_service.ReloadReport> reports,
      ) async {
        // TODO(aam): Investigate why we are validating only first reload report,
        // which seems to be current behavior
        if (reports.isEmpty) {
          return null;
        }
        final vm_service.ReloadReport firstReport = reports.first;
        // Don't print errors because they will be printed further down when
        // `validateReloadReport` is called again.
        await device.updateReloadStatus(
          HotRunner.validateReloadReport(firstReport, printErrors: false),
        );
        return DeviceReloadReport(device, reports);
      }),
    );
  }
  final Iterable<DeviceReloadReport> reports =
      (await Future.wait(allReportsFutures)).whereType<DeviceReloadReport>();
  final vm_service.ReloadReport? reloadReport = reports.isEmpty ? null : reports.first.reports[0];
  if (reloadReport == null || !HotRunner.validateReloadReport(reloadReport)) {
    // Reload failed.
    HotEvent(
      'reload-reject',
      targetPlatform: targetPlatform!,
      sdkName: sdkName!,
      emulator: emulator!,
      fullRestart: false,
      reason: reason,
      usage: usage,
    ).send();
    analytics.send(
      Event.hotRunnerInfo(
        label: 'reload-reject',
        targetPlatform: targetPlatform,
        sdkName: sdkName,
        emulator: emulator,
        fullRestart: false,
        reason: reason,
      ),
    );
    // Reset devFS lastCompileTime to ensure the file will still be marked
    // as dirty on subsequent reloads.
    _resetDevFSCompileTime(flutterDevices);
    if (reloadReport == null) {
      return OperationResult(1, 'No Dart isolates found');
    }
    final ReloadReportContents contents = ReloadReportContents.fromReloadReport(reloadReport);
    return OperationResult(1, 'Reload rejected: ${contents.notices.join("\n")}');
  }
  // Collect stats only from the first device. If/when run -d all is
  // refactored, we'll probably need to send one hot reload/restart event
  // per device to analytics.
  firstReloadDetails.addAll(castStringKeyedMap(reloadReport.json!['details'])!);
  final Map<String, dynamic> details = reloadReport.json!['details'] as Map<String, dynamic>;
  final int? loadedLibraryCount = details['loadedLibraryCount'] as int?;
  final int? finalLibraryCount = details['finalLibraryCount'] as int?;
  globals.printTrace('reloaded $loadedLibraryCount of $finalLibraryCount libraries');
  // reloadMessage = 'Reloaded $loadedLibraryCount of $finalLibraryCount libraries';
  // Record time it took for the VM to reload the sources.
  hotRunner._addBenchmarkData(
    'hotReloadVMReloadMilliseconds',
    vmReloadTimer.elapsed.inMilliseconds,
  );
  return OperationResult(0, 'Reloaded $loadedLibraryCount of $finalLibraryCount libraries');
}

Future<List<Future<vm_service.ReloadReport>>> _reloadDeviceSources(
  FlutterDevice device,
  String entryPath, {
  bool? pause = false,
}) async {
  final String deviceEntryUri = device.devFS!.baseUri!.resolve(entryPath).toString();
  final vm_service.VM vm = await device.vmService!.service.getVM();
  return <Future<vm_service.ReloadReport>>[
    for (final vm_service.IsolateRef isolateRef in vm.isolates!)
      device.vmService!.service.reloadSources(
        isolateRef.id!,
        pause: pause,
        rootLibUri: deviceEntryUri,
      ),
  ];
}

void _resetDevFSCompileTime(List<FlutterDevice?> flutterDevices) {
  for (final FlutterDevice? device in flutterDevices) {
    device!.devFS!.resetLastCompiled();
  }
}

@visibleForTesting
class ReassembleResult {
  ReassembleResult(this.reassembleViews, this.failedReassemble, this.shouldReportReloadTime);
  final Map<FlutterView?, FlutterVmService?> reassembleViews;
  final bool failedReassemble;
  final bool shouldReportReloadTime;
}

typedef ReassembleHelper =
    Future<ReassembleResult> Function(
      List<FlutterDevice?> flutterDevices,
      Map<FlutterDevice?, List<FlutterView>> viewCache,
      void Function(String message)? onSlow,
      String reloadMessage,
    );

Future<ReassembleResult> _defaultReassembleHelper(
  List<FlutterDevice?> flutterDevices,
  Map<FlutterDevice?, List<FlutterView>> viewCache,
  void Function(String message)? onSlow,
  String reloadMessage,
) async {
  // Check if any isolates are paused and reassemble those that aren't.
  final Map<FlutterView, FlutterVmService?> reassembleViews = <FlutterView, FlutterVmService?>{};
  final List<Future<void>> reassembleFutures = <Future<void>>[];
  String? serviceEventKind;
  int pausedIsolatesFound = 0;
  bool failedReassemble = false;
  bool shouldReportReloadTime = true;
  for (final FlutterDevice? device in flutterDevices) {
    final List<FlutterView> views = viewCache[device]!;
    for (final FlutterView view in views) {
      // Check if the isolate is paused, and if so, don't reassemble. Ignore the
      // PostPauseEvent event - the client requesting the pause will resume the app.
      final vm_service.Event? pauseEvent = await device!.vmService!.getIsolatePauseEventOrNull(
        view.uiIsolate!.id!,
      );
      if (pauseEvent != null &&
          isPauseEvent(pauseEvent.kind!) &&
          pauseEvent.kind != vm_service.EventKind.kPausePostRequest) {
        pausedIsolatesFound += 1;
        if (serviceEventKind == null) {
          serviceEventKind = pauseEvent.kind;
        } else if (serviceEventKind != pauseEvent.kind) {
          serviceEventKind = ''; // many kinds
        }
      } else {
        reassembleViews[view] = device.vmService;
        // If the tool identified a change in a single widget, do a fast instead
        // of a full reassemble.
        final Future<void> reassembleWork = device.vmService!.flutterReassemble(
          isolateId: view.uiIsolate!.id!,
        );
        reassembleFutures.add(
          reassembleWork.then(
            (Object? obj) => obj,
            onError: (Object error, StackTrace stackTrace) {
              if (error is! Exception) {
                return Future<Object?>.error(error, stackTrace);
              }
              failedReassemble = true;
              globals.printError(
                'Reassembling ${view.uiIsolate!.name} failed: $error\n$stackTrace',
              );
            },
          ),
        );
      }
    }
  }
  if (pausedIsolatesFound > 0) {
    if (onSlow != null) {
      onSlow(
        '${_describePausedIsolates(pausedIsolatesFound, serviceEventKind!)}; interface might not update.',
      );
    }
    if (reassembleViews.isEmpty) {
      globals.printTrace('Skipping reassemble because all isolates are paused.');
      return ReassembleResult(reassembleViews, failedReassemble, shouldReportReloadTime);
    }
  }
  assert(reassembleViews.isNotEmpty);

  globals.printTrace('Reassembling application');

  final Future<void> reassembleFuture = Future.wait<void>(reassembleFutures).then((void _) => null);
  await reassembleFuture.timeout(
    const Duration(seconds: 2),
    onTimeout: () async {
      if (pausedIsolatesFound > 0) {
        shouldReportReloadTime = false;
        return; // probably no point waiting, they're probably deadlocked and we've already warned.
      }
      // Check if any isolate is newly paused.
      globals.printTrace('This is taking a long time; will now check for paused isolates.');
      int postReloadPausedIsolatesFound = 0;
      String? serviceEventKind;
      for (final FlutterView view in reassembleViews.keys) {
        final vm_service.Event? pauseEvent = await reassembleViews[view]!
            .getIsolatePauseEventOrNull(view.uiIsolate!.id!);
        if (pauseEvent != null && isPauseEvent(pauseEvent.kind!)) {
          postReloadPausedIsolatesFound += 1;
          if (serviceEventKind == null) {
            serviceEventKind = pauseEvent.kind;
          } else if (serviceEventKind != pauseEvent.kind) {
            serviceEventKind = ''; // many kinds
          }
        }
      }
      globals.printTrace('Found $postReloadPausedIsolatesFound newly paused isolate(s).');
      if (postReloadPausedIsolatesFound == 0) {
        await reassembleFuture; // must just be taking a long time... keep waiting!
        return;
      }
      shouldReportReloadTime = false;
      if (onSlow != null) {
        onSlow('${_describePausedIsolates(postReloadPausedIsolatesFound, serviceEventKind!)}.');
      }
      return;
    },
  );
  return ReassembleResult(reassembleViews, failedReassemble, shouldReportReloadTime);
}

String _describePausedIsolates(int pausedIsolatesFound, String serviceEventKind) {
  assert(pausedIsolatesFound > 0);
  final StringBuffer message = StringBuffer();
  bool plural;
  if (pausedIsolatesFound == 1) {
    message.write('The application is ');
    plural = false;
  } else {
    message.write('$pausedIsolatesFound isolates are ');
    plural = true;
  }
  message.write(switch (serviceEventKind) {
    vm_service.EventKind.kPauseStart => 'paused (probably due to --start-paused)',
    vm_service.EventKind.kPauseExit =>
      'paused because ${plural ? 'they have' : 'it has'} terminated',
    vm_service.EventKind.kPauseBreakpoint => 'paused in the debugger on a breakpoint',
    vm_service.EventKind.kPauseInterrupted => 'paused due in the debugger',
    vm_service.EventKind.kPauseException => 'paused in the debugger after an exception was thrown',
    vm_service.EventKind.kPausePostRequest => 'paused',
    '' => 'paused for various reasons',
    _ => 'paused',
  });
  return message.toString();
}

/// The result of an invalidation check from [ProjectFileInvalidator].
class InvalidationResult {
  const InvalidationResult({this.uris, this.packageConfig});

  final List<Uri>? uris;
  final PackageConfig? packageConfig;
}

/// The [ProjectFileInvalidator] track the dependencies for a running
/// application to determine when they are dirty.
class ProjectFileInvalidator {
  ProjectFileInvalidator({
    required FileSystem fileSystem,
    required Platform platform,
    required Logger logger,
  }) : _fileSystem = fileSystem,
       _platform = platform,
       _logger = logger;

  final FileSystem _fileSystem;
  final Platform _platform;
  final Logger _logger;

  static const String _pubCachePathLinuxAndMac = '.pub-cache';
  static const String _pubCachePathWindows = 'Pub/Cache';

  // As of writing, Dart supports up to 32 asynchronous I/O threads per
  // isolate. We also want to avoid hitting platform limits on open file
  // handles/descriptors.
  //
  // This value was chosen based on empirical tests scanning a set of
  // ~2000 files.
  static const int _kMaxPendingStats = 8;

  Future<InvalidationResult> findInvalidated({
    required DateTime? lastCompiled,
    required List<Uri> urisToMonitor,
    required String packagesPath,
    required PackageConfig packageConfig,
    bool asyncScanning = false,
  }) async {
    if (lastCompiled == null) {
      // Initial load.
      assert(urisToMonitor.isEmpty);
      return InvalidationResult(packageConfig: packageConfig, uris: <Uri>[]);
    }

    final Stopwatch stopwatch = Stopwatch()..start();
    final List<Uri> urisToScan = <Uri>[
      // Don't watch pub cache directories to speed things up a little.
      for (final Uri uri in urisToMonitor)
        if (_isNotInPubCache(uri)) uri,
    ];
    final List<Uri> invalidatedFiles = <Uri>[];
    if (asyncScanning) {
      final Pool pool = Pool(_kMaxPendingStats);
      final List<Future<void>> waitList = <Future<void>>[];
      for (final Uri uri in urisToScan) {
        waitList.add(
          pool.withResource<void>(
            // Calling fs.stat() is more performant than fs.file().stat(), but
            // uri.toFilePath() does not work with MultiRootFileSystem.
            () => (uri.hasScheme && uri.scheme != 'file'
                    ? _fileSystem.file(uri).stat()
                    : _fileSystem.stat(uri.toFilePath(windows: _platform.isWindows)))
                .then((FileStat stat) {
                  final DateTime updatedAt = stat.modified;
                  if (updatedAt.isAfter(lastCompiled)) {
                    invalidatedFiles.add(uri);
                  }
                }),
          ),
        );
      }
      await Future.wait<void>(waitList);
    } else {
      for (final Uri uri in urisToScan) {
        // Calling fs.statSync() is more performant than fs.file().statSync(), but
        // uri.toFilePath() does not work with MultiRootFileSystem.
        final DateTime updatedAt =
            uri.hasScheme && uri.scheme != 'file'
                ? _fileSystem.file(uri).statSync().modified
                : _fileSystem.statSync(uri.toFilePath(windows: _platform.isWindows)).modified;
        if (updatedAt.isAfter(lastCompiled)) {
          invalidatedFiles.add(uri);
        }
      }
    }
    // We need to check the .dart_tool/package_config.json file too since it is
    // not used in compilation.
    final File packageFile = _fileSystem.file(packagesPath);
    final Uri packageUri = packageFile.uri;
    final DateTime updatedAt = packageFile.statSync().modified;
    if (updatedAt.isAfter(lastCompiled)) {
      invalidatedFiles.add(packageUri);
    }

    _logger.printTrace(
      'Scanned through ${urisToScan.length} files in '
      '${stopwatch.elapsedMilliseconds}ms'
      '${asyncScanning ? " (async)" : ""}',
    );
    return InvalidationResult(packageConfig: packageConfig, uris: invalidatedFiles);
  }

  bool _isNotInPubCache(Uri uri) {
    return !(_platform.isWindows && uri.path.contains(_pubCachePathWindows)) &&
        !uri.path.contains(_pubCachePathLinuxAndMac);
  }
}

/// Additional serialization logic for a hot reload response.
class ReloadReportContents {
  factory ReloadReportContents.fromReloadReport(vm_service.ReloadReport report) {
    final List<ReasonForCancelling> reasons = <ReasonForCancelling>[];
    final Object? notices = report.json!['notices'];
    if (notices is! List<dynamic>) {
      return ReloadReportContents._(report.success, reasons, report);
    }
    for (final Object? obj in notices) {
      if (obj is! Map<String, dynamic>) {
        continue;
      }
      final Map<String, dynamic> notice = obj;
      reasons.add(
        ReasonForCancelling(
          message: notice['message'] is String ? notice['message'] as String? : 'Unknown Error',
        ),
      );
    }

    return ReloadReportContents._(report.success, reasons, report);
  }

  ReloadReportContents._(this.success, this.notices, this.report);

  final bool? success;
  final List<ReasonForCancelling> notices;
  final vm_service.ReloadReport report;
}

/// A serialization class for hot reload rejection reasons.
///
/// Injects an additional error message that a hot restart will
/// resolve the issue.
class ReasonForCancelling {
  ReasonForCancelling({this.message});

  final String? message;

  @override
  String toString() {
    return '$message.\nTry performing a hot restart instead.';
  }
}
