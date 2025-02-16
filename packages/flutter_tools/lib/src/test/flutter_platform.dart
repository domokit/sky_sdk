// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';
import 'package:process/process.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:test_core/src/platform.dart'; // ignore: implementation_imports
import 'package:vm_service/vm_service.dart';

import '../base/async_guard.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/process.dart';
import '../build_info.dart';
import '../cache.dart';
import '../compile.dart';
import '../convert.dart';
import '../dart/language_version.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../native_assets.dart';
import '../project.dart';
import '../test/test_wrapper.dart';

import '../vmservice.dart';
import 'flutter_tester_device.dart';
import 'font_config_manager.dart';
import 'integration_test_device.dart';
import 'test_compiler.dart';
import 'test_config.dart';
import 'test_device.dart';
import 'test_golden_comparator.dart';
import 'test_time_recorder.dart';
import 'watcher.dart';

/// The address at which our WebSocket server resides and at which the sky_shell
/// processes will host the VmService server.
final Map<InternetAddressType, InternetAddress> _kHosts = <InternetAddressType, InternetAddress>{
  InternetAddressType.IPv4: InternetAddress.loopbackIPv4,
  InternetAddressType.IPv6: InternetAddress.loopbackIPv6,
};

typedef PlatformPluginRegistration = void Function(FlutterPlatform platform);

/// Configure the `test` package to work with Flutter.
///
/// On systems where each [FlutterPlatform] is only used to run one test suite
/// (that is, one Dart file with a `*_test.dart` file name and a single `void
/// main()`), you can set a VM Service port explicitly.
FlutterPlatform installHook({
  TestWrapper testWrapper = const TestWrapper(),
  required String flutterTesterBinPath,
  required DebuggingOptions debuggingOptions,
  required BuildInfo buildInfo,
  required FileSystem fileSystem,
  required Logger logger,
  required ProcessManager processManager,
  TestWatcher? watcher,
  bool enableVmService = false,
  bool machine = false,
  String? precompiledDillPath,
  Map<String, String>? precompiledDillFiles,
  bool updateGoldens = false,
  String? testAssetDirectory,
  InternetAddressType serverType = InternetAddressType.IPv4,
  Uri? projectRootDirectory,
  FlutterProject? flutterProject,
  String? icudtlPath,
  PlatformPluginRegistration? platformPluginRegistration,
  Device? integrationTestDevice,
  String? integrationTestUserIdentifier,
  TestTimeRecorder? testTimeRecorder,
  TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
}) {
  assert(
    enableVmService ||
        (!debuggingOptions.startPaused && debuggingOptions.hostVmServicePort == null),
  );

  // registerPlatformPlugin can be injected for testing since it's not very mock-friendly.
  platformPluginRegistration ??= (FlutterPlatform platform) {
    testWrapper.registerPlatformPlugin(<Runtime>[Runtime.vm], () {
      return platform;
    });
  };
  final FlutterPlatform platform = FlutterPlatform(
    flutterTesterBinPath: flutterTesterBinPath,
    debuggingOptions: debuggingOptions,
    watcher: watcher,
    machine: machine,
    enableVmService: enableVmService,
    host: _kHosts[serverType],
    precompiledDillPath: precompiledDillPath,
    precompiledDillFiles: precompiledDillFiles,
    updateGoldens: updateGoldens,
    testAssetDirectory: testAssetDirectory,
    projectRootDirectory: projectRootDirectory,
    flutterProject: flutterProject,
    icudtlPath: icudtlPath,
    integrationTestDevice: integrationTestDevice,
    integrationTestUserIdentifier: integrationTestUserIdentifier,
    testTimeRecorder: testTimeRecorder,
    nativeAssetsBuilder: nativeAssetsBuilder,
    buildInfo: buildInfo,
    fileSystem: fileSystem,
    logger: logger,
    processManager: processManager,
  );
  platformPluginRegistration(platform);
  return platform;
}

/// Generates the bootstrap entry point script that will be used to launch an
/// individual test file.
///
/// The [testUrl] argument specifies the path to the test file that is being
/// launched.
///
/// The [host] argument specifies the address at which the test harness is
/// running.
///
/// If [testConfigFile] is specified, it must follow the conventions of test
/// configuration files as outlined in the [flutter_test] library. By default,
/// the test file will be launched directly.
///
/// The [packageConfigUri] argument specifies the package config location for
/// the test file being launched. This is expected to be a file URI.
///
/// The [updateGoldens] argument will set the [autoUpdateGoldens] global
/// variable in the [flutter_test] package before invoking the test.
///
/// The [integrationTest] argument can be specified to generate the bootstrap
/// for integration tests.
///
// This API is used by the Fuchsia source tree, do not add new
// required or position parameters.
String generateTestBootstrap({
  required Uri testUrl,
  required InternetAddress host,
  File? testConfigFile,
  Uri? packageConfigUri,
  bool updateGoldens = false,
  String languageVersionHeader = '',
  bool flutterTestDep = true,
  bool integrationTest = false,
}) {
  final String websocketUrl =
      host.type == InternetAddressType.IPv4 ? 'ws://${host.address}' : 'ws://[${host.address}]';

  final StringBuffer buffer = StringBuffer();
  buffer.write('''
$languageVersionHeader
import 'dart:async';
import 'dart:convert';  // flutter_ignore: dart_convert_import
import 'dart:io';  // flutter_ignore: dart_io_import
import 'dart:isolate';
''');
  if (flutterTestDep) {
    buffer.write('''
import 'package:flutter_test/flutter_test.dart';
''');
  }
  if (integrationTest) {
    buffer.write('''
import 'package:integration_test/integration_test.dart';
import 'dart:developer' as developer;
''');
  }
  buffer.write('''
import 'package:test_api/backend.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:stack_trace/stack_trace.dart';

import '$testUrl' as test;
''');
  if (testConfigFile != null) {
    buffer.write('''
import '${Uri.file(testConfigFile.path)}' as test_config;
''');
  }

  // IMPORTANT: DO NOT RENAME, REMOVE, OR MODIFY THE
  // 'const packageConfigLocation' VARIABLE.
  // Dash tooling like Dart DevTools performs an evaluation on this variable at
  // runtime to get the package config location for Flutter test targets.
  buffer.write('''

const packageConfigLocation = '$packageConfigUri';
''');
  buffer.write('''

/// Returns a serialized test suite.
StreamChannel<dynamic> serializeSuite(Function getMain()) {
  return RemoteListener.start(getMain);
}

Future<void> _testMain() async {
''');
  if (integrationTest) {
    buffer.write('''
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();
''');
  }
  // Don't propagate the return value of `test.main` here. If the `main`
  // function on users` test is annotated with `@doNotStore`, it will cause an
  // analyzer error otherwise.
  buffer.write('''
  await Future(test.main);
}

/// Capture any top-level errors (mostly lazy syntax errors, since other are
/// caught below) and report them to the parent isolate.
void catchIsolateErrors() {
  final ReceivePort errorPort = ReceivePort();
  // Treat errors non-fatal because otherwise they'll be double-printed.
  Isolate.current.setErrorsFatal(false);
  Isolate.current.addErrorListener(errorPort.sendPort);
  errorPort.listen((dynamic message) {
    // Masquerade as an IsolateSpawnException because that's what this would
    // be if the error had been detected statically.
    final IsolateSpawnException error = IsolateSpawnException(
        message[0] as String);
    final Trace stackTrace = message[1] == null ?
        Trace(const <Frame>[]) : Trace.parse(message[1] as String);
    Zone.current.handleUncaughtError(error, stackTrace);
  });
}

void main() {
  final String serverPort = Platform.environment['SERVER_PORT'] ?? '';
  final String server = '$websocketUrl:\$serverPort';
  StreamChannel<dynamic> testChannel = serializeSuite(() {
    catchIsolateErrors();
''');
  if (flutterTestDep) {
    buffer.write('''
    goldenFileComparator = LocalFileComparator(Uri.parse('$testUrl'));
    autoUpdateGoldenFiles = $updateGoldens;
''');
  }
  if (integrationTest) {
    buffer.write('''
    VmServiceProxyGoldenFileComparator.useIfRunningOnDevice();
''');
  }
  if (testConfigFile != null) {
    buffer.write('''
    return () => test_config.testExecutable(_testMain);
''');
  } else {
    buffer.write('''
    return _testMain;
''');
  }
  buffer.write('''
  });
''');
  if (integrationTest) {
    buffer.write('''
  final callback = (method, params) async {
    testChannel.sink.add(json.decode(params['$kIntegrationTestData'] as String));

    // Result is ignored but null is not accepted here.
    return developer.ServiceExtensionResponse.result('{}');
  };

  developer.registerExtension('$kIntegrationTestMethod', callback);

  testChannel.stream.listen((x) {
    developer.postEvent(
      '$kIntegrationTestExtension',
      {'$kIntegrationTestData': json.encode(x)},
    );
  });
  ''');
  } else {
    buffer.write('''
  WebSocket.connect(server).then((WebSocket socket) {
    socket.map((dynamic message) {
      // We're only communicating with string encoded JSON.
      return json.decode(message as String);
    }).pipe(testChannel.sink);
    socket.addStream(testChannel.stream.map(json.encode));
  });
''');
  }
  buffer.write('''
}
  ''');
  return buffer.toString();
}

typedef Finalizer = Future<void> Function();

/// The flutter test platform used to integrate with package:test.
class FlutterPlatform extends PlatformPlugin {
  FlutterPlatform({
    required this.flutterTesterBinPath,
    required this.debuggingOptions,
    required this.buildInfo,
    required this.logger,
    required FileSystem fileSystem,
    required ProcessManager processManager,
    this.watcher,
    this.enableVmService,
    this.machine,
    this.host,
    this.precompiledDillPath,
    this.precompiledDillFiles,
    this.updateGoldens,
    this.testAssetDirectory,
    this.projectRootDirectory,
    this.flutterProject,
    this.icudtlPath,
    this.integrationTestDevice,
    this.integrationTestUserIdentifier,
    this.testTimeRecorder,
    this.nativeAssetsBuilder,
    this.shutdownHooks,
  }) {
    _testGoldenComparator = TestGoldenComparator(
      flutterTesterBinPath: flutterTesterBinPath,
      compilerFactory:
          () =>
              compiler ??
              TestCompiler(buildInfo, flutterProject, testTimeRecorder: testTimeRecorder),
      fileSystem: fileSystem,
      logger: logger,
      processManager: processManager,
    );
  }

  final String flutterTesterBinPath;
  final DebuggingOptions debuggingOptions;
  final TestWatcher? watcher;
  final bool? enableVmService;
  final bool? machine;
  final InternetAddress? host;
  final String? precompiledDillPath;
  final Map<String, String>? precompiledDillFiles;
  final bool? updateGoldens;
  final String? testAssetDirectory;
  final Uri? projectRootDirectory;
  final FlutterProject? flutterProject;
  final String? icudtlPath;
  final TestTimeRecorder? testTimeRecorder;
  final TestCompilerNativeAssetsBuilder? nativeAssetsBuilder;
  final BuildInfo buildInfo;
  final Logger logger;
  final ShutdownHooks? shutdownHooks;

  /// The device to run the test on for Integration Tests.
  ///
  /// If this is null, the test will run as a regular test with the Flutter
  /// Tester; otherwise it will run as a Integration Test on this device.
  final Device? integrationTestDevice;
  bool get _isIntegrationTest => integrationTestDevice != null;
  late final TestGoldenComparator _testGoldenComparator;

  final String? integrationTestUserIdentifier;

  final FontConfigManager _fontConfigManager = FontConfigManager();

  /// The test compiler produces dill files for each test main.
  ///
  /// To speed up compilation, each compile is initialized from an existing
  /// dill file from previous runs, if possible.
  TestCompiler? compiler;

  // Each time loadChannel() is called, we spin up a local WebSocket server,
  // then spin up the engine in a subprocess. We pass the engine a Dart file
  // that connects to our WebSocket server, then we proxy JSON messages from
  // the test harness to the engine and back again. If at any time the engine
  // crashes, we inject an error into that stream. When the process closes,
  // we clean everything up.

  int _testCount = 0;

  @override
  Future<RunnerSuite> load(
    String path,
    SuitePlatform platform,
    SuiteConfiguration suiteConfig,
    Object message,
  ) async {
    // loadChannel may throw an exception. That's fine; it will cause the
    // LoadSuite to emit an error, which will be presented to the user.
    // Except for the Declarer error, which is a specific test incompatibility
    // error we need to catch.
    final StreamChannel<dynamic> channel = loadChannel(path, platform);
    final RunnerSuiteController controller = deserializeSuite(
      path,
      platform,
      suiteConfig,
      const PluginEnvironment(),
      channel,
      message,
    );
    return controller.suite;
  }

  /// Used as an implementation detail for [load]ing a test suite.
  @visibleForTesting
  StreamChannel<Object?> loadChannel(String path, SuitePlatform platform) {
    if (_testCount > 0) {
      // Fail if there will be a port conflict.
      if (debuggingOptions.hostVmServicePort != null) {
        throwToolExit(
          'installHook() was called with a VM Service port or debugger mode enabled, but then more than one test suite was run.',
        );
      }
      // Fail if we're passing in a precompiled entry-point.
      if (precompiledDillPath != null) {
        throwToolExit(
          'installHook() was called with a precompiled test entry-point, but then more than one test suite was run.',
        );
      }
    }

    final int ourTestCount = _testCount;
    _testCount += 1;
    final StreamController<dynamic> localController = StreamController<dynamic>();
    final StreamController<dynamic> remoteController = StreamController<dynamic>();
    final Completer<_AsyncError?> testCompleteCompleter = Completer<_AsyncError?>();
    final _FlutterPlatformStreamSinkWrapper<dynamic> remoteSink =
        _FlutterPlatformStreamSinkWrapper<dynamic>(
          remoteController.sink,
          testCompleteCompleter.future,
        );
    final StreamChannel<dynamic> localChannel = StreamChannel<dynamic>.withGuarantees(
      remoteController.stream,
      localController.sink,
    );
    final StreamChannel<dynamic> remoteChannel = StreamChannel<dynamic>.withGuarantees(
      localController.stream,
      remoteSink,
    );
    testCompleteCompleter.complete(_startTest(path, localChannel, ourTestCount));
    return remoteChannel;
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
    if (compiler == null || compiler!.compiler == null) {
      throw Exception('Compiler is not set up properly to compile $expression');
    }
    final CompilerOutput? compilerOutput = await compiler!.compiler!.compileExpression(
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
    throw Exception('Failed to compile $expression');
  }

  TestDevice _createTestDevice(int ourTestCount) {
    if (_isIntegrationTest) {
      return IntegrationTestTestDevice(
        id: ourTestCount,
        debuggingOptions: debuggingOptions,
        device: integrationTestDevice!,
        userIdentifier: integrationTestUserIdentifier,
        compileExpression: _compileExpressionService,
      );
    }
    return FlutterTesterTestDevice(
      id: ourTestCount,
      platform: globals.platform,
      fileSystem: globals.fs,
      processManager: globals.processManager,
      logger: globals.logger,
      flutterTesterBinPath: flutterTesterBinPath,
      enableVmService: enableVmService!,
      machine: machine,
      debuggingOptions: debuggingOptions,
      host: host,
      testAssetDirectory: testAssetDirectory,
      flutterProject: flutterProject,
      icudtlPath: icudtlPath,
      compileExpression: _compileExpressionService,
      fontConfigManager: _fontConfigManager,
      nativeAssetsBuilder: nativeAssetsBuilder,
    );
  }

  void _handleStartedDevice({required Uri? uri, required int testCount, required String testPath}) {
    if (uri != null) {
      globals.printTrace('test $testCount: VM Service uri is available at $uri');
      if (_isIntegrationTest) {
        _listenToVmServiceForGoldens(uri: uri, testPath: testPath);
      }
    } else {
      globals.printTrace('test $testCount: VM Service uri is not available');
    }
    watcher?.handleStartedDevice(uri);
  }

  static const String _kEventName = 'integration_test.VmServiceProxyGoldenFileComparator';
  static const String _kExtension = 'ext.$_kEventName';

  Future<void> _listenToVmServiceForGoldens({required Uri uri, required String testPath}) async {
    final Uri goldensBaseUri = Uri.parse(testPath);
    final FlutterVmService vmService = await connectToVmService(uri, logger: logger);
    final IsolateRef testAppIsolate = await vmService.findExtensionIsolate(_kExtension);
    await vmService.service.streamListen(_kEventName);
    vmService.service.onEvent(_kEventName).listen((Event e) async {
      if (!const <String>['compare', 'update'].contains(e.extensionKind)) {
        throw StateError('Unexpected command: "${e.extensionKind}".');
      }

      final Map<String, Object?>? data = e.extensionData?.data;
      if (data == null) {
        throw StateError('Expected VM service data, but got null.');
      }
      final int id = data['id']! as int;
      final Uri relativePath = Uri.parse(data['path']! as String);
      final Uint8List bytes = base64.decode(data['bytes']! as String);

      final Map<String, Object?> args;
      if (e.extensionKind == 'update') {
        switch (await _testGoldenComparator.update(goldensBaseUri, bytes, relativePath)) {
          case TestGoldenUpdateDone():
            args = <String, Object?>{'result': true};
          case TestGoldenUpdateError(error: final String error):
            args = <String, Object?>{'error': error};
        }
      } else {
        switch (await _testGoldenComparator.compare(goldensBaseUri, bytes, relativePath)) {
          case TestGoldenComparisonDone(matched: final bool matched):
            args = <String, Object?>{'result': matched};
          case TestGoldenComparisonError(error: final String error):
            args = <String, Object?>{'error': error};
        }
      }

      await vmService.callMethodWrapper(
        _kExtension,
        isolateId: testAppIsolate.id,
        args: <String, Object?>{'id': id, ...args},
      );
    });
  }

  Future<_AsyncError?> _startTest(
    String testPath,
    StreamChannel<dynamic> testHarnessChannel,
    int ourTestCount,
  ) async {
    globals.printTrace('test $ourTestCount: starting test $testPath');

    _AsyncError?
    outOfBandError; // error that we couldn't send to the harness that we need to send via our future

    // Will be run in reverse order.
    final List<Finalizer> finalizers = <Finalizer>[];
    bool ranFinalizers = false;
    bool controllerSinkClosed = false;
    Future<void> finalize() async {
      if (ranFinalizers) {
        return;
      }
      ranFinalizers = true;
      globals.printTrace('test $ourTestCount: cleaning up...');
      for (final Finalizer finalizer in finalizers.reversed) {
        try {
          await finalizer();
        } on Exception catch (error, stack) {
          globals.printTrace(
            'test $ourTestCount: error while cleaning up; ${controllerSinkClosed ? "reporting to console" : "sending to test framework"}',
          );
          if (!controllerSinkClosed) {
            testHarnessChannel.sink.addError(error, stack);
          } else {
            globals.printError(
              'unhandled error during finalization of test:\n$testPath\n$error\n$stack',
            );
            outOfBandError ??= _AsyncError(error, stack);
          }
        }
      }
    }

    // If the flutter CLI is forcibly terminated, cleanup processes.
    final ShutdownHooks shutdownHooks = this.shutdownHooks ?? globals.shutdownHooks;
    shutdownHooks.addShutdownHook(finalize);

    try {
      // Callback can't throw since it's just setting a variable.
      unawaited(
        testHarnessChannel.sink.done.whenComplete(() {
          controllerSinkClosed = true;
        }),
      );

      void initializeExpressionCompiler(String path) {
        // When start paused is specified, it means that the user is likely
        // running this with a debugger attached. Initialize the resident
        // compiler in this case.
        if (debuggingOptions.startPaused) {
          compiler ??= TestCompiler(
            debuggingOptions.buildInfo,
            flutterProject,
            precompiledDillPath: precompiledDillPath,
            testTimeRecorder: testTimeRecorder,
          );
          final Uri uri = globals.fs.file(path).uri;
          // Trigger a compilation to initialize the resident compiler.
          unawaited(compiler!.compile(uri));
        }
      }

      // If a kernel file is given, then use that to launch the test.
      // If mapping is provided, look kernel file from mapping.
      // If all fails, create a "listener" dart that invokes actual test.
      String? mainDart;
      if (precompiledDillPath != null) {
        mainDart = precompiledDillPath;
        initializeExpressionCompiler(testPath);
      } else if (precompiledDillFiles != null) {
        mainDart = precompiledDillFiles![testPath];
      } else {
        mainDart = _createListenerDart(finalizers, ourTestCount, testPath);

        // Integration test device takes care of the compilation.
        if (integrationTestDevice == null) {
          // Lazily instantiate compiler so it is built only if it is actually used.
          compiler ??= TestCompiler(
            debuggingOptions.buildInfo,
            flutterProject,
            testTimeRecorder: testTimeRecorder,
          );
          switch (await compiler!.compile(globals.fs.file(mainDart).uri)) {
            case TestCompilerComplete(:final String outputPath):
              mainDart = outputPath;
            case TestCompilerFailure(:final String? error):
              testHarnessChannel.sink.addError(
                'Compilation failed for testPath=$testPath: $error.',
              );
              return null;
          }
        } else {
          // For integration tests, we may still need to set up expression compilation service.
          initializeExpressionCompiler(mainDart);
        }
      }

      globals.printTrace('test $ourTestCount: starting test device');
      final TestDevice testDevice = _createTestDevice(ourTestCount);
      final Stopwatch? testTimeRecorderStopwatch = testTimeRecorder?.start(TestTimePhases.Run);
      final Completer<StreamChannel<String>> remoteChannelCompleter =
          Completer<StreamChannel<String>>();
      unawaited(
        asyncGuard(
          () async {
            final StreamChannel<String> channel = await testDevice.start(mainDart!);
            remoteChannelCompleter.complete(channel);
          },
          onError: (Object err, StackTrace stackTrace) {
            remoteChannelCompleter.completeError(err, stackTrace);
          },
        ),
      );
      finalizers.add(() async {
        globals.printTrace('test $ourTestCount: ensuring test device is terminated.');
        await testDevice.kill();
      });

      // At this point, these things can happen:
      // A. The test device could crash, in which case [testDevice.finished]
      // will complete.
      // B. The test device could connect to us, in which case
      // [remoteChannelFuture] will complete.
      globals.printTrace('test $ourTestCount: awaiting connection to test device');
      await Future.any<void>(<Future<void>>[
        testDevice.finished,
        () async {
          final [Object? first, Object? _] = await Future.wait<Object?>(
            <Future<Object?>>[
              // This future may depend on [_handleStartedDevice] having been called
              remoteChannelCompleter.future,
              testDevice.vmServiceUri.then<void>((Uri? processVmServiceUri) {
                _handleStartedDevice(
                  uri: processVmServiceUri,
                  testCount: ourTestCount,
                  testPath: testPath,
                );
              }),
            ],
            // If [remoteChannelCompleter.future] errors, we may never get the
            // VM service URI, so erroring eagerly is necessary to avoid a
            // deadlock.
            eagerError: true,
          );
          final StreamChannel<String> remoteChannel = first! as StreamChannel<String>;

          globals.printTrace(
            'test $ourTestCount: connected to test device, now awaiting test result',
          );

          await _pipeHarnessToRemote(
            id: ourTestCount,
            harnessChannel: testHarnessChannel,
            remoteChannel: remoteChannel,
          );

          globals.printTrace('test $ourTestCount: finished');
          testTimeRecorder?.stop(TestTimePhases.Run, testTimeRecorderStopwatch!);
          final Stopwatch? watchTestTimeRecorderStopwatch = testTimeRecorder?.start(
            TestTimePhases.WatcherFinishedTest,
          );
          await watcher?.handleFinishedTest(testDevice);
          testTimeRecorder?.stop(
            TestTimePhases.WatcherFinishedTest,
            watchTestTimeRecorderStopwatch!,
          );
        }(),
      ]);
    } on Exception catch (error, stackTrace) {
      Object reportedError = error;
      StackTrace reportedStackTrace = stackTrace;
      if (error is TestDeviceException) {
        reportedError = error.message;
        reportedStackTrace = error.stackTrace;
      }

      globals.printTrace(
        'test $ourTestCount: error caught during test; ${controllerSinkClosed ? "reporting to console" : "sending to test framework"}',
      );
      if (!controllerSinkClosed) {
        testHarnessChannel.sink.addError(reportedError, reportedStackTrace);
      } else {
        globals.printError(
          'unhandled error during test:\n$testPath\n$reportedError\n$reportedStackTrace',
        );
        outOfBandError ??= _AsyncError(reportedError, reportedStackTrace);
      }
    } finally {
      await finalize();
      if (!controllerSinkClosed) {
        // Waiting below with await.
        unawaited(testHarnessChannel.sink.close());
        globals.printTrace('test $ourTestCount: waiting for controller sink to close');
        await testHarnessChannel.sink.done;
      }
    }
    assert(controllerSinkClosed);
    if (outOfBandError != null) {
      globals.printTrace('test $ourTestCount: finished with out-of-band failure');
    } else {
      globals.printTrace('test $ourTestCount: finished');
    }
    return outOfBandError;
  }

  String _createListenerDart(List<Finalizer> finalizers, int ourTestCount, String testPath) {
    // Prepare a temporary directory to store the Dart file that will talk to us.
    final Directory tempDir = globals.fs.systemTempDirectory.createTempSync(
      'flutter_test_listener.',
    );
    finalizers.add(() async {
      globals.printTrace('test $ourTestCount: deleting temporary directory');
      tempDir.deleteSync(recursive: true);
    });

    // Prepare the Dart file that will talk to us and start the test.
    final File listenerFile = globals.fs.file('${tempDir.path}/listener.dart');
    listenerFile.createSync();
    listenerFile.writeAsStringSync(
      _generateTestMain(testUrl: globals.fs.path.toUri(globals.fs.path.absolute(testPath))),
    );
    return listenerFile.path;
  }

  String _generateTestMain({required Uri testUrl}) {
    assert(testUrl.scheme == 'file');
    final File file = globals.fs.file(testUrl);
    final PackageConfig packageConfig = debuggingOptions.buildInfo.packageConfig;

    final LanguageVersion languageVersion = determineLanguageVersion(
      file,
      packageConfig[flutterProject!.manifest.appName],
      Cache.flutterRoot!,
    );
    return generateTestBootstrap(
      testUrl: testUrl,
      testConfigFile: findTestConfigFile(globals.fs.file(testUrl), globals.logger),
      // This MUST be a file URI.
      packageConfigUri: globals.fs.path.toUri(buildInfo.packageConfigPath),
      host: host!,
      updateGoldens: updateGoldens!,
      flutterTestDep: packageConfig['flutter_test'] != null,
      languageVersionHeader: '// @dart=${languageVersion.major}.${languageVersion.minor}',
      integrationTest: _isIntegrationTest,
    );
  }

  @override
  Future<dynamic> close() async {
    if (compiler != null) {
      await compiler!.dispose();
      compiler = null;
    }
    await _fontConfigManager.dispose();
  }
}

// The [_shellProcessClosed] future can't have errors thrown on it because it
// crosses zones (it's fed in a zone created by the test package, but listened
// to by a parent zone, the same zone that calls [close] below).
//
// This is because Dart won't let errors that were fed into a Future in one zone
// propagate to listeners in another zone. (Specifically, the zone in which the
// future was completed with the error, and the zone in which the listener was
// registered, are what matters.)
//
// Because of this, the [_shellProcessClosed] future takes an [_AsyncError]
// object as a result. If it's null, it's as if it had completed correctly; if
// it's non-null, it contains the error and stack trace of the actual error, as
// if it had completed with that error.
class _FlutterPlatformStreamSinkWrapper<S> implements StreamSink<S> {
  _FlutterPlatformStreamSinkWrapper(this._parent, this._shellProcessClosed);

  final StreamSink<S> _parent;
  final Future<_AsyncError?> _shellProcessClosed;

  @override
  Future<void> get done => _done.future;
  final Completer<void> _done = Completer<void>();

  @override
  Future<dynamic> close() {
    Future.wait<dynamic>(<Future<dynamic>>[_parent.close(), _shellProcessClosed]).then<void>((
      List<dynamic> futureResults,
    ) {
      assert(futureResults.length == 2);
      assert(futureResults.first == null);
      final dynamic lastResult = futureResults.last;
      if (lastResult is _AsyncError) {
        _done.completeError(lastResult.error as Object, lastResult.stack);
      } else {
        assert(lastResult == null);
        _done.complete();
      }
    }, onError: _done.completeError);
    return done;
  }

  @override
  void add(S event) => _parent.add(event);
  @override
  void addError(Object errorEvent, [StackTrace? stackTrace]) =>
      _parent.addError(errorEvent, stackTrace);
  @override
  Future<dynamic> addStream(Stream<S> stream) => _parent.addStream(stream);
}

@immutable
class _AsyncError {
  const _AsyncError(this.error, this.stack);
  final dynamic error;
  final StackTrace stack;
}

/// Bridges the package:test harness and the remote device.
///
/// The returned future completes when either side is closed, which also
/// indicates when the tests have finished.
Future<void> _pipeHarnessToRemote({
  required int id,
  required StreamChannel<dynamic> harnessChannel,
  required StreamChannel<String> remoteChannel,
}) async {
  globals.printTrace('test $id: Waiting for test harness or tests to finish');

  await Future.any<void>(<Future<void>>[
    harnessChannel.stream.map<String>(json.encode).pipe(remoteChannel.sink).then<void>((
      void value,
    ) {
      globals.printTrace('test $id: Test process is no longer needed by test harness');
    }),
    remoteChannel.stream.map<dynamic>(json.decode).pipe(harnessChannel.sink).then<void>((
      void value,
    ) {
      globals.printTrace('test $id: Test harness is no longer needed by test process');
    }),
  ]);
}
