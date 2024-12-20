// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io; // flutter_ignore: dart_io_import;

import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../base/dds.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../convert.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../native_assets.dart';
import '../project.dart';
import '../resident_runner.dart';
import '../vmservice.dart';
import 'font_config_manager.dart';
import 'test_device.dart';

/// Implementation of [TestDevice] with the Flutter Tester over a [Process].
class FlutterTesterTestDevice extends TestDevice {
  FlutterTesterTestDevice({
    required this.id,
    required this.platform,
    required this.fileSystem,
    required this.processManager,
    required this.logger,
    required this.shellPath,
    required this.debuggingOptions,
    required this.enableVmService,
    required this.machine,
    required this.host,
    required this.testAssetDirectory,
    required this.flutterProject,
    required this.icudtlPath,
    required this.compileExpression,
    required this.fontConfigManager,
    required this.nativeAssetsBuilder,
  }) : assert(!debuggingOptions.startPaused || enableVmService),
       _gotProcessVmServiceUri =
           enableVmService ? Completer<Uri?>() : (Completer<Uri?>()..complete());

  /// Used for logging to identify the test that is currently being executed.
  final int id;
  final Platform platform;
  final FileSystem fileSystem;
  final ProcessManager processManager;
  final Logger logger;
  final String shellPath;
  final DebuggingOptions debuggingOptions;
  final bool enableVmService;
  final bool? machine;
  final InternetAddress? host;
  final String? testAssetDirectory;
  final FlutterProject? flutterProject;
  final String? icudtlPath;
  final CompileExpression? compileExpression;
  final FontConfigManager fontConfigManager;
  final TestCompilerNativeAssetsBuilder? nativeAssetsBuilder;

  late final DartDevelopmentService _ddsLauncher = DartDevelopmentService(logger: logger);
  final Completer<Uri?> _gotProcessVmServiceUri;
  final Completer<int> _exitCode = Completer<int>();

  Process? _process;
  HttpServer? _server;
  DevtoolsLauncher? _devToolsLauncher;

  /// Starts the device.
  ///
  /// [entrypointPath] is the path to the entrypoint file which must be compiled
  /// as a dill.
  @override
  Future<StreamChannel<String>> start(String entrypointPath) async {
    assert(!_exitCode.isCompleted);
    assert(_process == null);
    assert(_server == null);

    // Prepare our WebSocket server to talk to the engine subprocess.
    // Let the server choose an unused port.
    _server = await bind(host, /*port*/ 0);
    logger.printTrace('test $id: test harness socket server is running at port:${_server!.port}');
    final List<String> command = <String>[
      shellPath,
      if (enableVmService) ...<String>[
        // Some systems drive the _FlutterPlatform class in an unusual way, where
        // only one test file is processed at a time, and the operating
        // environment hands out specific ports ahead of time in a cooperative
        // manner, where we're only allowed to open ports that were given to us in
        // advance like this. For those esoteric systems, we have this feature
        // whereby you can create _FlutterPlatform with a pair of ports.
        //
        // I mention this only so that you won't be tempted, as I was, to apply
        // the obvious simplification to this code and remove this entire feature.
        '--vm-service-port=${debuggingOptions.enableDds ? 0 : debuggingOptions.hostVmServicePort}',
        if (debuggingOptions.startPaused) '--start-paused',
        if (debuggingOptions.disableServiceAuthCodes) '--disable-service-auth-codes',
      ] else
        '--disable-vm-service',
      if (host!.type == InternetAddressType.IPv6) '--ipv6',
      if (icudtlPath != null) '--icu-data-file-path=$icudtlPath',
      '--enable-checked-mode',
      '--verify-entry-points',
      if (debuggingOptions.enableImpeller == ImpellerStatus.enabled)
        '--enable-impeller'
      else ...<String>['--enable-software-rendering', '--skia-deterministic-rendering'],
      if (debuggingOptions.enableDartProfiling) '--enable-dart-profiling',
      '--non-interactive',
      '--use-test-fonts',
      '--disable-asset-fonts',
      '--packages=${debuggingOptions.buildInfo.packageConfigPath}',
      if (testAssetDirectory != null) '--flutter-assets-dir=$testAssetDirectory',
      if (debuggingOptions.nullAssertions) '--dart-flags=--null_assertions',
      ...debuggingOptions.dartEntrypointArgs,
      entrypointPath,
    ];

    // If the FLUTTER_TEST environment variable has been set, then pass it on
    // for package:flutter_test to handle the value.
    //
    // If FLUTTER_TEST has not been set, assume from this context that this
    // call was invoked by the command 'flutter test'.
    final String flutterTest =
        platform.environment.containsKey('FLUTTER_TEST')
            ? platform.environment['FLUTTER_TEST']!
            : 'true';
    final Map<String, String> environment = <String, String>{
      'FLUTTER_TEST': flutterTest,
      'FONTCONFIG_FILE': fontConfigManager.fontConfigFile.path,
      'SERVER_PORT': _server!.port.toString(),
      'APP_NAME': flutterProject?.manifest.appName ?? '',
      if (debuggingOptions.enableImpeller == ImpellerStatus.enabled)
        'FLUTTER_TEST_IMPELLER': 'true',
      if (testAssetDirectory != null) 'UNIT_TEST_ASSETS': testAssetDirectory!,
      if (platform.isWindows && nativeAssetsBuilder != null && flutterProject != null)
        'PATH':
            '${nativeAssetsBuilder!.windowsBuildDirectory(flutterProject!)};${platform.environment['PATH']}',
    };

    logger.printTrace(
      'test $id: Starting flutter_tester process with command=$command, environment=$environment',
    );
    _process = await processManager.start(command, environment: environment);

    // Unawaited to update state.
    unawaited(
      _process!.exitCode.then((int exitCode) {
        logger.printTrace(
          'test $id: flutter_tester process at pid ${_process!.pid} exited with code=$exitCode',
        );
        _exitCode.complete(exitCode);
      }),
    );

    logger.printTrace('test $id: Started flutter_tester process at pid ${_process!.pid}');

    // Pipe stdout and stderr from the subprocess to our printStatus console.
    // We also keep track of what VM Service port the engine used, if any.
    _pipeStandardStreamsToConsole(
      process: _process!,
      reportVmServiceUri: (Uri detectedUri) async {
        assert(!_gotProcessVmServiceUri.isCompleted);
        assert(
          debuggingOptions.hostVmServicePort == null ||
              debuggingOptions.hostVmServicePort == detectedUri.port,
        );

        Uri? forwardingUri;

        if (debuggingOptions.enableDds) {
          logger.printTrace('test $id: Starting Dart Development Service');
          await _ddsLauncher.startDartDevelopmentServiceFromDebuggingOptions(
            detectedUri,
            debuggingOptions: debuggingOptions,
          );
          forwardingUri = _ddsLauncher.uri;
          logger.printTrace(
            'test $id: Dart Development Service started at $forwardingUri, forwarding to VM service at $detectedUri.',
          );
        } else {
          forwardingUri = detectedUri;
        }

        logger.printTrace('Connecting to service protocol: $forwardingUri');
        final FlutterVmService vmService = await connectToVmServiceImpl(
          forwardingUri!,
          compileExpression: compileExpression,
          logger: logger,
        );
        logger.printTrace('test $id: Successfully connected to service protocol: $forwardingUri');
        if (debuggingOptions.serveObservatory) {
          try {
            await vmService.callMethodWrapper('_serveObservatory');
          } on vm_service.RPCError {
            logger.printWarning('Unable to enable Observatory');
          }
        }

        if (debuggingOptions.startPaused && !machine!) {
          logger.printStatus('The Dart VM service is listening on $forwardingUri');
          await _startDevTools(forwardingUri, _ddsLauncher);
          logger.printStatus('');
          logger.printStatus(
            'The test process has been started. Set any relevant breakpoints and then resume the test in the debugger.',
          );
        }
        _gotProcessVmServiceUri.complete(forwardingUri);
      },
    );

    return remoteChannel;
  }

  @override
  Future<Uri?> get vmServiceUri {
    return _gotProcessVmServiceUri.future;
  }

  @override
  Future<void> kill() async {
    logger.printTrace('test $id: Terminating flutter_tester process');
    _process?.kill(io.ProcessSignal.sigkill);

    logger.printTrace('test $id: Shutting down DevTools server');
    await _devToolsLauncher?.close();

    logger.printTrace('test $id: Shutting down test harness socket server');
    await _server?.close(force: true);
    await finished;
  }

  @override
  Future<void> get finished async {
    final int exitCode = await _exitCode.future;

    // On Windows, the [exitCode] and the terminating signal have no correlation.
    if (platform.isWindows) {
      return;
    }

    // ProcessSignal.SIGKILL. Negative because signals are returned as negative
    // exit codes.
    if (exitCode == -9) {
      // We expect SIGKILL (9) because we could have tried to [kill] it.
      return;
    }
    throw TestDeviceException(_getExitCodeMessage(exitCode), StackTrace.current);
  }

  @visibleForTesting
  @protected
  Future<FlutterVmService> connectToVmServiceImpl(
    Uri httpUri, {
    CompileExpression? compileExpression,
    required Logger logger,
  }) {
    return connectToVmService(httpUri, compileExpression: compileExpression, logger: logger);
  }

  // TODO(bkonyi): remove when ready to serve DevTools from DDS.
  Future<void> _startDevTools(Uri forwardingUri, DartDevelopmentService? dds) async {
    _devToolsLauncher = DevtoolsLauncher.instance;
    logger.printTrace('test $id: Serving DevTools...');
    final DevToolsServerAddress? devToolsServerAddress = await _devToolsLauncher?.serve();

    if (devToolsServerAddress == null) {
      logger.printTrace('test $id: Failed to start DevTools');
      return;
    }
    await _devToolsLauncher?.ready;
    logger.printTrace('test $id: DevTools is being served at ${devToolsServerAddress.uri}');

    final Uri devToolsUri = devToolsServerAddress.uri!.replace(
      // Use query instead of queryParameters to avoid unnecessary encoding.
      query: 'uri=$forwardingUri',
    );
    logger.printStatus('The Flutter DevTools debugger and profiler is available at: $devToolsUri');
  }

  /// Binds an [HttpServer] serving from `host` on `port`.
  ///
  /// Only intended to be overridden in tests.
  @protected
  @visibleForTesting
  Future<HttpServer> bind(InternetAddress? host, int port) => HttpServer.bind(host, port);

  @protected
  @visibleForTesting
  Future<StreamChannel<String>> get remoteChannel async {
    assert(_server != null);

    try {
      final HttpRequest firstRequest = await _server!.first;
      final WebSocket webSocket = await WebSocketTransformer.upgrade(firstRequest);
      return _webSocketToStreamChannel(webSocket);
    } on Exception catch (error, stackTrace) {
      throw TestDeviceException('Unable to connect to flutter_tester process: $error', stackTrace);
    }
  }

  @override
  String toString() {
    final String status =
        _process != null
            ? 'pid: ${_process!.pid}, ${_exitCode.isCompleted ? 'exited' : 'running'}'
            : 'not started';
    return 'Flutter Tester ($status) for test $id';
  }

  void _pipeStandardStreamsToConsole({
    required Process process,
    required Future<void> Function(Uri uri) reportVmServiceUri,
  }) {
    for (final Stream<List<int>> stream in <Stream<List<int>>>[process.stderr, process.stdout]) {
      stream
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen(
            (String line) async {
              logger.printTrace('test $id: Shell: $line');

              final Match? match = globals.kVMServiceMessageRegExp.firstMatch(line);
              if (match != null) {
                try {
                  final Uri uri = Uri.parse(match[1]!);
                  await reportVmServiceUri(uri);
                } on Exception catch (error) {
                  logger.printError('Could not parse shell VM Service port message: $error');
                }
              } else {
                logger.printStatus('Shell: $line');
              }
            },
            onError: (dynamic error) {
              logger.printError(
                'shell console stream for process pid ${process.pid} experienced an unexpected error: $error',
              );
            },
            cancelOnError: true,
          );
    }
  }
}

String _getExitCodeMessage(int exitCode) {
  return switch (exitCode) {
    1 => 'Shell subprocess cleanly reported an error. Check the logs above for an error message.',
    0 => 'Shell subprocess ended cleanly. Did main() call exit()?',
    -0x0f => 'Shell subprocess crashed with SIGTERM ($exitCode).', // ProcessSignal.SIGTERM
    -0x0b => 'Shell subprocess crashed with segmentation fault.', // ProcessSignal.SIGSEGV
    -0x06 => 'Shell subprocess crashed with SIGABRT ($exitCode).', // ProcessSignal.SIGABRT
    -0x02 => 'Shell subprocess terminated by ^C (SIGINT, $exitCode).', // ProcessSignal.SIGINT
    _ => 'Shell subprocess crashed with unexpected exit code $exitCode.',
  };
}

StreamChannel<String> _webSocketToStreamChannel(WebSocket webSocket) {
  final StreamChannelController<String> controller = StreamChannelController<String>();

  controller.local.stream.map<dynamic>((String message) => message as dynamic).pipe(webSocket);
  webSocket
      // We're only communicating with string encoded JSON.
      .map<String>((dynamic message) => message as String)
      .pipe(controller.local.sink);

  return controller.foreign;
}
