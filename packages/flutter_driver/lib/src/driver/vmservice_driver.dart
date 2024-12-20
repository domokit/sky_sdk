// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:developer';
library;

import 'dart:async';
import 'dart:io';

import 'package:file/file.dart' as f;
import 'package:fuchsia_remote_debug_protocol/fuchsia_remote_debug_protocol.dart' as fuchsia;
import 'package:path/path.dart' as p;
import 'package:vm_service/vm_service.dart' as vms;
import 'package:webdriver/async_io.dart' as async_io;

import '../../flutter_driver.dart';

/// An implementation of the Flutter Driver over the vmservice protocol.
class VMServiceFlutterDriver extends FlutterDriver {
  /// Creates a driver that uses a connection provided by the given
  /// [serviceClient] and [appIsolate].
  VMServiceFlutterDriver.connectedTo(
    this._serviceClient,
    this._appIsolate, {
    bool printCommunication = false,
    bool logCommunicationToFile = true,
  }) : _printCommunication = printCommunication,
       _logCommunicationToFile = logCommunicationToFile,
       _driverId = _nextDriverId++ {
    _logFilePathName = p.join(testOutputsDirectory, 'flutter_driver_commands_$_driverId.log');
  }

  /// Connects to a Flutter application.
  ///
  /// See [FlutterDriver.connect] for more documentation.
  static Future<FlutterDriver> connect({
    String? dartVmServiceUrl,
    bool printCommunication = false,
    bool logCommunicationToFile = true,
    int? isolateNumber,
    Pattern? fuchsiaModuleTarget,
    Map<String, dynamic>? headers,
  }) async {
    // If running on a Fuchsia device, connect to the first isolate whose name
    // matches FUCHSIA_MODULE_TARGET.
    //
    // If the user has already supplied an isolate number/URL to the Dart VM
    // service, then this won't be run as it is unnecessary.
    if (Platform.isFuchsia && isolateNumber == null) {
      // TODO(awdavies): Use something other than print. On fuchsia
      // `stderr`/`stdout` appear to have issues working correctly.
      driverLog = (String source, String message) {
        print('$source: $message'); // ignore: avoid_print
      };
      fuchsiaModuleTarget ??= Platform.environment['FUCHSIA_MODULE_TARGET'];
      if (fuchsiaModuleTarget == null) {
        throw DriverError(
          'No Fuchsia module target has been specified.\n'
          'Please make sure to specify the FUCHSIA_MODULE_TARGET '
          'environment variable.',
        );
      }
      final fuchsia.FuchsiaRemoteConnection fuchsiaConnection = await FuchsiaCompat.connect();
      final List<fuchsia.IsolateRef> refs = await fuchsiaConnection.getMainIsolatesByPattern(
        fuchsiaModuleTarget,
      );
      if (refs.isEmpty) {
        throw DriverError('Failed to get any isolate refs!');
      }
      final fuchsia.IsolateRef ref = refs.first;
      isolateNumber = ref.number;
      dartVmServiceUrl = ref.dartVm.uri.toString();
      await fuchsiaConnection.stop();
      FuchsiaCompat.cleanup();
    }

    dartVmServiceUrl ??= Platform.environment['VM_SERVICE_URL'];

    if (dartVmServiceUrl == null) {
      throw DriverError(
        'Could not determine URL to connect to application.\n'
        'Either the VM_SERVICE_URL environment variable should be set, or an explicit '
        'URL should be provided to the FlutterDriver.connect() method.',
      );
    }

    // Connect to Dart VM services
    _log('Connecting to Flutter application at $dartVmServiceUrl');
    final vms.VmService client = await vmServiceConnectFunction(dartVmServiceUrl, headers);

    Future<vms.IsolateRef?> waitForRootIsolate() async {
      bool checkIsolate(vms.IsolateRef ref) => ref.number == isolateNumber.toString();
      while (true) {
        final vms.VM vm = await client.getVM();
        if (vm.isolates!.isEmpty || (isolateNumber != null && !vm.isolates!.any(checkIsolate))) {
          await Future<void>.delayed(_kPauseBetweenReconnectAttempts);
          continue;
        }
        return isolateNumber == null ? vm.isolates!.first : vm.isolates!.firstWhere(checkIsolate);
      }
    }

    // Refreshes the isolate state periodically until the isolate reports as
    // being runnable.
    Future<vms.Isolate> waitForIsolateToBeRunnable(vms.IsolateRef ref) async {
      while (true) {
        final vms.Isolate isolate = await client.getIsolate(ref.id!);
        if (isolate.pauseEvent!.kind == vms.EventKind.kNone) {
          _log('Waiting for isolate ${ref.number} to be runnable.');
          await Future<void>.delayed(_kPauseBetweenIsolateRefresh);
        } else {
          _log('Isolate ${ref.number} is runnable.');
          return isolate;
        }
      }
    }

    final vms.IsolateRef isolateRef =
        (await _warnIfSlow<vms.IsolateRef?>(
          future: waitForRootIsolate(),
          timeout: kUnusuallyLongTimeout,
          message:
              isolateNumber == null
                  ? 'The root isolate is taking an unusually long time to start.'
                  : 'Isolate $isolateNumber is taking an unusually long time to start.',
        ))!;
    _log('Isolate found with number: ${isolateRef.number}');
    final vms.Isolate isolate = await _warnIfSlow<vms.Isolate>(
      future: waitForIsolateToBeRunnable(isolateRef),
      timeout: kUnusuallyLongTimeout,
      message:
          'The isolate ${isolateRef.number} is taking unusually long time '
          'to initialize. It still reports ${vms.EventKind.kNone} as pause '
          'event which is incorrect.',
    );

    final VMServiceFlutterDriver driver = VMServiceFlutterDriver.connectedTo(
      client,
      isolate,
      printCommunication: printCommunication,
      logCommunicationToFile: logCommunicationToFile,
    );

    // Attempts to resume the isolate, but does not crash if it fails because
    // the isolate is already resumed. There could be a race with other tools,
    // such as a debugger, any of which could have resumed the isolate.
    Future<vms.Success> resumeLeniently() async {
      _log('Attempting to resume isolate');
      // Let subsequent isolates start automatically.
      try {
        final vms.Response result = await client.setFlag('pause_isolates_on_start', 'false');
        if (result.type != 'Success') {
          _log('setFlag failure: $result');
        }
      } catch (e) {
        _log('Failed to set pause_isolates_on_start=false, proceeding. Error: $e');
      }

      return client.resume(isolate.id!).catchError((Object e) {
        const int vmMustBePausedCode = 101;
        if (e is vms.RPCError && e.code == vmMustBePausedCode) {
          // No biggie; something else must have resumed the isolate
          _log(
            'Attempted to resume an already resumed isolate. This may happen '
            'when another tool (usually a debugger) resumed the isolate '
            'before the flutter_driver did.',
          );
          return vms.Success();
        } else {
          // Failed to resume due to another reason. Fail hard.
          throw e; // ignore: only_throw_errors, proxying the error from upstream.
        }
      });
    }

    /// Waits for a signal from the VM service that the extension is registered.
    ///
    /// Looks at the list of loaded extensions for the current [isolateRef], as
    /// well as the stream of added extensions.
    Future<void> waitForServiceExtension() async {
      await client.streamListen(vms.EventStreams.kIsolate);

      final Future<void> extensionAlreadyAdded = client.getIsolate(isolateRef.id!).then((
        vms.Isolate isolate,
      ) async {
        if (isolate.extensionRPCs!.contains(_flutterExtensionMethodName)) {
          return;
        }
        // Never complete. Rely on the stream listener to find the service
        // extension instead.
        return Completer<void>().future;
      });

      final Completer<void> extensionAdded = Completer<void>();
      late StreamSubscription<vms.Event> isolateAddedSubscription;

      isolateAddedSubscription = client.onIsolateEvent.listen(
        (vms.Event data) {
          if (data.kind == vms.EventKind.kServiceExtensionAdded &&
              data.extensionRPC == _flutterExtensionMethodName) {
            extensionAdded.complete();
            isolateAddedSubscription.cancel();
          }
        },
        onError: extensionAdded.completeError,
        cancelOnError: true,
      );

      await Future.any(<Future<void>>[extensionAlreadyAdded, extensionAdded.future]);
      await isolateAddedSubscription.cancel();
      await client.streamCancel(vms.EventStreams.kIsolate);
    }

    // Attempt to resume isolate if it was paused
    if (isolate.pauseEvent!.kind == vms.EventKind.kPauseStart) {
      _log('Isolate is paused at start.');

      await resumeLeniently();
    } else if (isolate.pauseEvent!.kind == vms.EventKind.kPauseExit ||
        isolate.pauseEvent!.kind == vms.EventKind.kPauseBreakpoint ||
        isolate.pauseEvent!.kind == vms.EventKind.kPauseException ||
        isolate.pauseEvent!.kind == vms.EventKind.kPauseInterrupted ||
        isolate.pauseEvent!.kind == vms.EventKind.kPausePostRequest) {
      // If the isolate is paused for any other reason, assume the extension is
      // already there.
      _log('Isolate is paused mid-flight.');
      await resumeLeniently();
    } else if (isolate.pauseEvent!.kind == vms.EventKind.kResume) {
      _log('Isolate is not paused. Assuming application is ready.');
    } else {
      _log(
        'Unknown pause event type ${isolate.pauseEvent.runtimeType}. '
        'Assuming application is ready.',
      );
    }

    // We will never receive the extension event if the user does not register
    // it. If that happens, show a message but continue waiting.
    await _warnIfSlow<void>(
      future: waitForServiceExtension(),
      timeout: kUnusuallyLongTimeout,
      message:
          'Flutter Driver extension is taking a long time to become available. '
          'Ensure your test app (often "lib/main.dart") imports '
          '"package:flutter_driver/driver_extension.dart" and '
          'calls enableFlutterDriverExtension() as the first call in main().',
    );

    final Health health = await driver.checkHealth();
    if (health.status != HealthStatus.ok) {
      await client.dispose();
      await client.onDone;
      throw DriverError('Flutter application health check failed.');
    }

    _log('Connected to Flutter application.');
    return driver;
  }

  static int _nextDriverId = 0;

  static const String _flutterExtensionMethodName = 'ext.flutter.driver';
  static const String _collectAllGarbageMethodName = '_collectAllGarbage';

  // The additional blank line in the beginning is for _log.
  static const String _kDebugWarning = '''

┏╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍┓
┇ ⚠    THIS BENCHMARK IS BEING RUN IN DEBUG MODE     ⚠  ┇
┡╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍╍┦
│                                                       │
│  Numbers obtained from a benchmark while asserts are  │
│  enabled will not accurately reflect the performance  │
│  that will be experienced by end users using release  ╎
│  builds. Benchmarks should be run using this command  ┆
│  line:  flutter drive --profile test_perf.dart        ┊
│                                                       ┊
└─────────────────────────────────────────────────╌┄┈  🐢
''';

  /// The unique ID of this driver instance.
  final int _driverId;

  @override
  vms.Isolate get appIsolate => _appIsolate;

  /// Client connected to the Dart VM running the Flutter application.
  ///
  /// You can use [vms.VmService] to check VM version, flags and get
  /// notified when a new isolate has been instantiated. That could be
  /// useful if your application spawns multiple isolates that you
  /// would like to instrument.
  final vms.VmService _serviceClient;

  @override
  vms.VmService get serviceClient => _serviceClient;

  @override
  async_io.WebDriver get webDriver =>
      throw UnsupportedError('VMServiceFlutterDriver does not support webDriver');

  /// The main isolate hosting the Flutter application.
  ///
  /// If you used the [registerExtension] API to instrument your application,
  /// you can use this [vms.Isolate] to call these extension methods via
  /// [vms.Isolate.invokeExtension].
  final vms.Isolate _appIsolate;

  /// Whether to print communication between host and app to `stdout`.
  final bool _printCommunication;

  /// Whether to log communication between host and app to `flutter_driver_commands.log`.
  final bool _logCommunicationToFile;

  /// Logs are written here when _logCommunicationToFile is true.
  late final String _logFilePathName;

  /// Getter for file pathname where logs are written when _logCommunicationToFile is true.
  String get logFilePathName => _logFilePathName;

  @override
  Future<Map<String, dynamic>> sendCommand(Command command) async {
    late Map<String, dynamic> response;
    try {
      final Map<String, String> serialized = command.serialize();
      _logCommunication('>>> $serialized');
      final Future<Map<String, dynamic>> future = _serviceClient
          .callServiceExtension(
            _flutterExtensionMethodName,
            isolateId: _appIsolate.id,
            args: serialized,
          )
          .then<Map<String, dynamic>>((vms.Response value) => value.json!);
      response = await _warnIfSlow<Map<String, dynamic>>(
        future: future,
        timeout: command.timeout ?? kUnusuallyLongTimeout,
        message: '${command.kind} message is taking a long time to complete...',
      );
      _logCommunication('<<< $response');
    } catch (error, stackTrace) {
      throw DriverError(
        'Failed to fulfill ${command.runtimeType} due to remote error',
        error,
        stackTrace,
      );
    }
    if ((response['isError'] as bool?) ?? false) {
      throw DriverError('Error in Flutter application: ${response['response']}');
    }
    return response['response'] as Map<String, dynamic>;
  }

  void _logCommunication(String message) {
    if (_printCommunication) {
      _log(message);
    }
    if (_logCommunicationToFile) {
      final f.File file = fs.file(_logFilePathName);
      file.createSync(recursive: true); // no-op if file exists
      file.writeAsStringSync('${DateTime.now()} $message\n', mode: f.FileMode.append, flush: true);
    }
  }

  @override
  Future<List<Map<String, dynamic>>> getVmFlags() async {
    final vms.FlagList result = await _serviceClient.getFlagList();
    return result.flags != null
        ? result.flags!.map((vms.Flag flag) => flag.toJson()).toList()
        : const <Map<String, dynamic>>[];
  }

  Future<vms.Timestamp> _getVMTimelineMicros() async {
    return _serviceClient.getVMTimelineMicros();
  }

  @override
  Future<void> startTracing({
    List<TimelineStream> streams = const <TimelineStream>[TimelineStream.all],
    Duration timeout = kUnusuallyLongTimeout,
  }) async {
    assert(streams.isNotEmpty);
    try {
      await _warnIfSlow<vms.Success>(
        future: _serviceClient.setVMTimelineFlags(_timelineStreamsToString(streams)),
        timeout: timeout,
        message: 'VM is taking an unusually long time to respond to being told to start tracing...',
      );
    } catch (error, stackTrace) {
      throw DriverError('Failed to start tracing due to remote error', error, stackTrace);
    }
  }

  @override
  Future<Timeline> stopTracingAndDownloadTimeline({
    Duration timeout = kUnusuallyLongTimeout,
    int? startTime,
    int? endTime,
  }) async {
    assert((startTime == null && endTime == null) || (startTime != null && endTime != null));

    try {
      await _warnIfSlow<vms.Success>(
        future: _serviceClient.setVMTimelineFlags(const <String>[]),
        timeout: timeout,
        message: 'VM is taking an unusually long time to respond to being told to stop tracing...',
      );
      if (startTime == null) {
        final vms.Timeline timeline = await _serviceClient.getVMTimeline();
        return Timeline.fromJson(timeline.json!);
      }
      const int kSecondInMicros = 1000000;
      int currentStart = startTime;
      int currentEnd = startTime + kSecondInMicros; // 1 second of timeline
      final List<Map<String, Object?>?> chunks = <Map<String, Object?>?>[];
      do {
        final vms.Timeline chunk = await _serviceClient.getVMTimeline(
          timeOriginMicros: currentStart,
          // The range is inclusive, avoid double counting on the chance something
          // aligns on the boundary.
          timeExtentMicros: kSecondInMicros - 1,
        );
        chunks.add(chunk.json);
        currentStart = currentEnd;
        currentEnd += kSecondInMicros;
      } while (currentStart < endTime!);
      return Timeline.fromJson(<String, Object>{
        'traceEvents': <Object?>[
          for (final Map<String, Object?>? chunk in chunks)
            ...chunk!['traceEvents']! as List<Object?>,
        ],
      });
    } catch (error, stackTrace) {
      throw DriverError('Failed to stop tracing due to remote error', error, stackTrace);
    }
  }

  Future<bool> _isPrecompiledMode() async {
    final List<Map<String, dynamic>> flags = await getVmFlags();
    for (final Map<String, dynamic> flag in flags) {
      if (flag['name'] == 'precompiled_mode') {
        return flag['valueAsString'] == 'true';
      }
    }
    return false;
  }

  @override
  Future<Timeline> traceAction(
    Future<dynamic> Function() action, {
    List<TimelineStream> streams = const <TimelineStream>[TimelineStream.all],
    bool retainPriorEvents = false,
  }) async {
    if (retainPriorEvents) {
      await startTracing(streams: streams);
      await action();

      if (!(await _isPrecompiledMode())) {
        _log(_kDebugWarning);
      }

      return stopTracingAndDownloadTimeline();
    }

    await clearTimeline();

    final vms.Timestamp startTimestamp = await _getVMTimelineMicros();
    await startTracing(streams: streams);
    await action();
    final vms.Timestamp endTimestamp = await _getVMTimelineMicros();

    if (!(await _isPrecompiledMode())) {
      _log(_kDebugWarning);
    }

    return stopTracingAndDownloadTimeline(
      startTime: startTimestamp.timestamp,
      endTime: endTimestamp.timestamp,
    );
  }

  @override
  Future<void> clearTimeline({Duration timeout = kUnusuallyLongTimeout}) async {
    try {
      await _warnIfSlow<vms.Success>(
        future: _serviceClient.clearVMTimeline(),
        timeout: timeout,
        message:
            'VM is taking an unusually long time to respond to being told to clear its timeline buffer...',
      );
    } catch (error, stackTrace) {
      throw DriverError('Failed to clear event timeline due to remote error', error, stackTrace);
    }
  }

  @override
  Future<void> forceGC() async {
    try {
      await _serviceClient.callMethod(_collectAllGarbageMethodName, isolateId: _appIsolate.id);
    } catch (error, stackTrace) {
      throw DriverError('Failed to force a GC due to remote error', error, stackTrace);
    }
  }

  @override
  Future<void> close() async {
    await _serviceClient.dispose();
    await _serviceClient.onDone;
  }
}

/// The connection function used by [FlutterDriver.connect].
///
/// Overwrite this function if you require a custom method for connecting to
/// the VM service.
VMServiceConnectFunction vmServiceConnectFunction = _waitAndConnect;

/// Restores [vmServiceConnectFunction] to its default value.
void restoreVmServiceConnectFunction() {
  vmServiceConnectFunction = _waitAndConnect;
}

String _getWebSocketUrl(String url) {
  Uri uri = Uri.parse(url);
  final List<String> pathSegments = <String>[
    // If there's an authentication code (default), we need to add it to our path.
    if (uri.pathSegments.isNotEmpty) uri.pathSegments.first,
    'ws',
  ];
  if (uri.scheme == 'http') {
    uri = uri.replace(scheme: 'ws', pathSegments: pathSegments);
  }
  return uri.toString();
}

/// Waits for a real Dart VM service to become available, then connects using
/// the [vms.VmService].
Future<vms.VmService> _waitAndConnect(String url, Map<String, dynamic>? headers) async {
  final String webSocketUrl = _getWebSocketUrl(url);
  int attempts = 0;
  WebSocket? socket;
  while (true) {
    try {
      socket = await WebSocket.connect(webSocketUrl, headers: headers);
      final StreamController<dynamic> controller = StreamController<dynamic>();
      final Completer<void> streamClosedCompleter = Completer<void>();
      socket.listen(
        (dynamic data) => controller.add(data),
        onDone: () => streamClosedCompleter.complete(),
      );
      final vms.VmService service = vms.VmService(
        controller.stream,
        socket.add,
        disposeHandler: () => socket!.close(),
        streamClosed: streamClosedCompleter.future,
      );
      // This call is to ensure we are able to establish a connection instead of
      // keeping on trucking and failing farther down the process.
      await service.getVersion();
      return service;
    } catch (e) {
      // We should not be catching all errors arbitrarily here, this might hide real errors.
      // TODO(ianh): Determine which exceptions to catch here.
      await socket?.close();
      if (attempts > 5) {
        _log('It is taking an unusually long time to connect to the VM...');
      }
      attempts += 1;
      await Future<void>.delayed(_kPauseBetweenReconnectAttempts);
    }
  }
}

/// The amount of time we wait prior to making the next attempt to connect to
/// the VM service.
const Duration _kPauseBetweenReconnectAttempts = Duration(seconds: 1);

/// The amount of time we wait prior to refreshing the isolate state.
const Duration _kPauseBetweenIsolateRefresh = Duration(milliseconds: 100);

// See `timeline_streams` in
// https://github.com/dart-lang/sdk/blob/main/runtime/vm/timeline.cc
List<String> _timelineStreamsToString(List<TimelineStream> streams) {
  return streams
      .map<String>(
        (TimelineStream stream) => switch (stream) {
          TimelineStream.all => 'all',
          TimelineStream.api => 'API',
          TimelineStream.dart => 'Dart',
          TimelineStream.debugger => 'Debugger',
          TimelineStream.embedder => 'Embedder',
          TimelineStream.gc => 'GC',
          TimelineStream.isolate => 'Isolate',
          TimelineStream.vm => 'VM',
          TimelineStream.compiler => 'Compiler',
          TimelineStream.compilerVerbose => 'CompilerVerbose',
        },
      )
      .toList();
}

void _log(String message) {
  driverLog('VMServiceFlutterDriver', message);
}

Future<T> _warnIfSlow<T>({
  required Future<T> future,
  required Duration timeout,
  required String message,
}) async {
  final Completer<void> completer = Completer<void>();
  completer.future.timeout(
    timeout,
    onTimeout: () {
      _log(message);
      return null;
    },
  );
  try {
    await future.whenComplete(() {
      completer.complete();
    });
  } catch (e) {
    // Don't duplicate errors if [future] completes with an error.
  }
  return future;
}

/// A function that connects to a Dart VM service given the `url` and `headers`.
typedef VMServiceConnectFunction =
    Future<vms.VmService> Function(String url, Map<String, dynamic>? headers);
