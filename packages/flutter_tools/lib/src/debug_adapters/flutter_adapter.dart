// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:dds/dap.dart' hide PidTracker;
import 'package:vm_service/vm_service.dart' as vm;

import '../base/io.dart';
import '../base/process.dart';
import '../cache.dart';
import '../convert.dart';
import '../globals.dart' as globals show fs;
import 'error_formatter.dart';
import 'flutter_adapter_args.dart';
import 'flutter_base_adapter.dart';

/// A DAP Debug Adapter for running and debugging Flutter applications.
class FlutterDebugAdapter extends FlutterBaseDebugAdapter with VmServiceInfoFileUtils {
  FlutterDebugAdapter(
    super.channel, {
    required super.fileSystem,
    required super.platform,
    super.ipv6,
    super.enableFlutterDds = true,
    super.enableAuthCodes,
    super.logger,
    super.onError,
  });

  /// A completer that completes when the app.started event has been received.
  final Completer<void> _appStartedCompleter = Completer<void>();

  /// Whether or not the app.started event has been received.
  bool get _receivedAppStarted => _appStartedCompleter.isCompleted;

  /// The appId of the current running Flutter app.
  String? _appId;

  /// A progress reporter for the applications launch progress.
  ///
  /// `null` if a launch is not in progress (or has completed).
  DapProgressReporter? launchProgress;

  /// The ID to use for the next request sent to the Flutter run daemon.
  int _flutterRequestId = 1;

  /// Outstanding requests that have been sent to the Flutter run daemon and
  /// their handlers.
  final Map<int, Completer<Object?>> _flutterRequestCompleters = <int, Completer<Object?>>{};

  /// A list of reverse-requests from `flutter run --machine` that should be forwarded to the client.
  static const Set<String> _requestsToForwardToClient = <String>{
    // The 'app.exposeUrl' request is sent by Flutter to request the client
    // exposes a URL to the user and return the public version of that URL.
    //
    // This supports some web scenarios where the `flutter` tool may be running
    // on a different machine to the user (for example a cloud IDE or in VS Code
    // remote workspace) so we cannot just use the raw URL because the hostname
    // and/or port might not be available to the machine the user is using.
    // Instead, the IDE/infrastructure can set up port forwarding/proxying and
    // return a user-facing URL that will map to the original (localhost) URL
    // Flutter provided.
    'app.exposeUrl',
  };

  /// A list of events from `flutter run --machine` that should be forwarded to the client.
  static const Set<String> _eventsToForwardToClient = <String>{
    // The 'app.webLaunchUrl' event is sent to the client to tell it about a URL
    // that should be launched (including a flag for whether it has been
    // launched by the tool or needs launching by the editor).
    'app.webLaunchUrl',
  };

  /// Completers for reverse requests from Flutter that may need to be handled by the client.
  final Map<Object, Completer<Object?>> _reverseRequestCompleters = <Object, Completer<Object?>>{};

  /// Whether or not the user requested debugging be enabled.
  ///
  /// For debugging to be enabled, the user must have chosen "Debug" (and not
  /// "Run") in the editor (which maps to the DAP `noDebug` field) _and_ must
  /// not have requested to run in Profile or Release mode. Profile/Release
  /// modes will always disable debugging.
  ///
  /// This is always `true` for attach requests.
  ///
  /// When not debugging, we will not connect to the VM Service so some
  /// functionality (breakpoints, evaluation, etc.) will not be available.
  /// Functionality provided via the daemon (hot reload/restart) will still be
  /// available.
  @override
  bool get enableDebugger => super.enableDebugger && !profileMode && !releaseMode;

  /// Whether the launch configuration arguments specify `--profile`.
  ///
  /// Always `false` for attach requests.
  bool get profileMode {
    final DartCommonLaunchAttachRequestArguments args = this.args;
    if (args is FlutterLaunchRequestArguments) {
      return args.toolArgs?.contains('--profile') ?? false;
    }

    // Otherwise (attach), always false.
    return false;
  }

  /// Whether the launch configuration arguments specify `--release`.
  ///
  /// Always `false` for attach requests.
  bool get releaseMode {
    final DartCommonLaunchAttachRequestArguments args = this.args;
    if (args is FlutterLaunchRequestArguments) {
      return args.toolArgs?.contains('--release') ?? false;
    }

    // Otherwise (attach), always false.
    return false;
  }

  /// Called by [attachRequest] to request that we actually connect to the app to be debugged.
  @override
  Future<void> attachImpl() async {
    final FlutterAttachRequestArguments args = this.args as FlutterAttachRequestArguments;
    String? vmServiceUri = args.vmServiceUri;
    final String? vmServiceInfoFile = args.vmServiceInfoFile;

    if (vmServiceUri != null && vmServiceInfoFile != null) {
      sendConsoleOutput(
        'To attach, provide only one (or neither) of vmServiceUri/vmServiceInfoFile',
      );
      handleSessionTerminate();
      return;
    }

    launchProgress = startProgressNotification('launch', 'Flutter', message: 'Attaching…');

    if (vmServiceUri == null && vmServiceInfoFile != null) {
      final Uri uriFromFile = await waitForVmServiceInfoFile(
        logger,
        globals.fs.file(vmServiceInfoFile),
      );
      vmServiceUri = uriFromFile.toString();
    }

    final List<String> toolArgs = <String>[
      'attach',
      '--machine',
      if (!enableFlutterDds) '--no-dds',
      if (vmServiceUri != null) ...<String>['--debug-uri', vmServiceUri],
    ];

    await _startProcess(
      toolArgs: toolArgs,
      customTool: args.customTool,
      customToolReplacesArgs: args.customToolReplacesArgs,
      userToolArgs: args.toolArgs,
      targetProgram: args.program,
    );
  }

  /// [customRequest] handles any messages that do not match standard messages in the spec.
  ///
  /// This is used to allow a client/DA to have custom methods outside of the
  /// spec. It is up to the client/DA to negotiate which custom messages are
  /// allowed.
  ///
  /// [sendResponse] must be called when handling a message, even if it is with
  /// a null response. Otherwise the client will never be informed that the
  /// request has completed.
  ///
  /// Any requests not handled must call super which will respond with an error
  /// that the message was not supported.
  ///
  /// Unless they start with _ to indicate they are private, custom messages
  /// should not change in breaking ways if client IDEs/editors may be calling
  /// them.
  @override
  Future<void> customRequest(
    Request request,
    RawRequestArguments? args,
    void Function(Object?) sendResponse,
  ) async {
    switch (request.command) {
      case 'hotRestart':
      case 'hotReload':
      // This convention is for the internal IDE client.
      case r'$/hotReload':
        final bool isFullRestart = request.command == 'hotRestart';
        await _performRestart(isFullRestart, args?.args['reason'] as String?);
        sendResponse(null);

      // Handle requests (from the client) that provide responses to reverse-requests
      // that we forwarded from `flutter run --machine`.
      case 'flutter.sendForwardedRequestResponse':
        _handleForwardedResponse(args);
        sendResponse(null);

      default:
        await super.customRequest(request, args, sendResponse);
    }
  }

  @override
  Future<void> handleExtensionEvent(vm.Event event) async {
    await super.handleExtensionEvent(event);

    switch (event.kind) {
      case vm.EventKind.kExtension:
        switch (event.extensionKind) {
          case 'Flutter.ServiceExtensionStateChanged':
            _sendServiceExtensionStateChanged(event.extensionData);
          case 'Flutter.Error':
            _handleFlutterErrorEvent(event.extensionData);
        }
    }
  }

  /// Sends OutputEvents to the client for a Flutter.Error event.
  void _handleFlutterErrorEvent(vm.ExtensionData? data) {
    final Map<String, Object?>? errorData = data?.data;
    if (errorData == null) {
      return;
    }

    FlutterErrorFormatter()
      ..formatError(errorData)
      ..sendOutput(sendOutput);
  }

  /// Called by [launchRequest] to request that we actually start the app to be run/debugged.
  ///
  /// For debugging, this should start paused, connect to the VM Service, set
  /// breakpoints, and resume.
  @override
  Future<void> launchImpl() async {
    final FlutterLaunchRequestArguments args = this.args as FlutterLaunchRequestArguments;

    launchProgress = startProgressNotification('launch', 'Flutter', message: 'Launching…');

    final List<String> toolArgs = <String>[
      'run',
      '--machine',
      if (!enableFlutterDds) '--no-dds',
      if (enableDebugger) '--start-paused',
      // Structured errors are enabled by default, but since we don't connect
      // the VM Service for noDebug, we need to disable them so that error text
      // is sent to stderr. Otherwise the user will not see any exception text
      // (because nobody is listening for Flutter.Error events).
      if (!enableDebugger) '--dart-define=flutter.inspector.structuredErrors=false',
    ];

    await _startProcess(
      toolArgs: toolArgs,
      customTool: args.customTool,
      customToolReplacesArgs: args.customToolReplacesArgs,
      targetProgram: args.program,
      userToolArgs: args.toolArgs,
      userArgs: args.args,
    );
  }

  /// Starts the `flutter` process to run/attach to the required app.
  Future<void> _startProcess({
    required String? customTool,
    required int? customToolReplacesArgs,
    required List<String> toolArgs,
    required List<String>? userToolArgs,
    String? targetProgram,
    List<String>? userArgs,
  }) async {
    // Handle customTool and deletion of any arguments for it.
    final String executable =
        customTool ??
        fileSystem.path.join(
          Cache.flutterRoot!,
          'bin',
          platform.isWindows ? 'flutter.bat' : 'flutter',
        );
    final int? removeArgs = customToolReplacesArgs;
    if (customTool != null && removeArgs != null) {
      toolArgs.removeRange(0, math.min(removeArgs, toolArgs.length));
    }

    final List<String> processArgs = <String>[
      ...toolArgs,
      ...?userToolArgs,
      if (targetProgram != null) ...<String>['--target', targetProgram],
      ...?userArgs,
    ];

    await launchAsProcess(executable: executable, processArgs: processArgs, env: args.env);
  }

  /// restart is called by the client when the user invokes a restart (for example with the button on the debug toolbar).
  ///
  /// For Flutter, we handle this ourselves be sending a Hot Restart request
  /// to the running app.
  @override
  Future<void> restartRequest(
    Request request,
    RestartArguments? args,
    void Function() sendResponse,
  ) async {
    await _performRestart(true);

    sendResponse();
  }

  /// Sends a request to the Flutter run daemon that is running/attaching to the app and waits for a response.
  ///
  /// If there is no process, the message will be silently ignored (this is
  /// common during the application being stopped, where async messages may be
  /// processed).
  Future<Object?> sendFlutterRequest(String method, Map<String, Object?>? params) async {
    final Completer<Object?> completer = Completer<Object?>();
    final int id = _flutterRequestId++;
    _flutterRequestCompleters[id] = completer;

    await sendFlutterMessage(<String, Object?>{'id': id, 'method': method, 'params': params});

    return completer.future;
  }

  /// Sends a message to the Flutter run daemon.
  ///
  /// Throws `DebugAdapterException` if a Flutter process is not yet running.
  Future<void> sendFlutterMessage(Map<String, Object?> message) async {
    final Process? process = this.process;
    if (process == null) {
      throw DebugAdapterException('Flutter process has not yet started');
    }

    final String messageString = jsonEncode(message);
    // Flutter requests are always wrapped in brackets as an array.
    final String payload = '[$messageString]\n';
    _logTraffic('==> [Flutter] $payload');
    await ProcessUtils.writelnToStdinUnsafe(stdin: process.stdin, line: payload);
  }

  /// Called by [terminateRequest] to request that we gracefully shut down the app being run (or in the case of an attach, disconnect).
  @override
  Future<void> terminateImpl() async {
    if (isAttach) {
      await handleDetach();
    }

    // Send a request to stop/detach to give Flutter chance to do some cleanup.
    // It's possible the Flutter process will terminate before we process the
    // response, so accept either a response or the process exiting.
    if (_appId != null) {
      final String method = isAttach ? 'app.detach' : 'app.stop';
      await Future.any<void>(<Future<void>>[
        sendFlutterRequest(method, <String, Object?>{'appId': _appId}),
        process?.exitCode ?? Future<void>.value(),
      ]);
    }

    terminatePids(ProcessSignal.sigterm);
    await process?.exitCode;
  }

  /// Connects to the VM Service if the app.started event has fired, and a VM Service URI is available.
  Future<void> _connectDebugger(Uri vmServiceUri) async {
    if (enableDebugger) {
      await connectDebugger(vmServiceUri);
    } else {
      // Usually, `connectDebugger` (in the base Dart adapter) will send this
      // event when it connects a debugger. Since we're not connecting a
      // debugger we send this ourselves, to allow clients to connect to the
      // VM Service for things like starting DevTools, even if debugging is
      // not available.
      sendDebuggerUris(vmServiceUri);
    }
  }

  /// Handles the app.start event from Flutter.
  void _handleAppStart(Map<String, Object?> params) {
    _appId = params['appId'] as String?;
    if (_appId == null) {
      throw DebugAdapterException('Unexpected null `appId` in app.start event');
    }

    // Notify the client whether it can call 'restartRequest' when the user
    // clicks restart, instead of terminating and re-starting its own debug
    // session (which is much slower, but required for profile/release mode).
    final bool supportsRestart = (params['supportsRestart'] as bool?) ?? false;
    sendEvent(
      CapabilitiesEventBody(capabilities: Capabilities(supportsRestartRequest: supportsRestart)),
    );

    // Send a custom event so the editor has info about the app starting.
    //
    // This message contains things like the `deviceId` and `mode` that the
    // client might not know about if they were inferred or set by users custom
    // args.
    sendEvent(RawEventBody(params), eventType: 'flutter.appStart');
  }

  /// Handles any app.progress event from Flutter.
  void _handleAppProgress(Map<String, Object?> params) {
    // If this is a new progress starting (and we're still launching), update
    // the progress notification.
    //
    // We ignore finished status because we have a limited API - the next
    // item will replace it (or the launch progress will be completed by
    // _handleAppStarted).
    if (params case {'message': final String message, 'finished': false}) {
      launchProgress?.update(message: message);
    }
  }

  /// Handles the app.started event from Flutter.
  Future<void> _handleAppStarted() async {
    launchProgress?.end();
    launchProgress = null;
    _appStartedCompleter.complete();

    // Send a custom event so the editor knows the app has started.
    //
    // This may be useful when there's no VM Service (for example Profile mode)
    // but the editor still wants to know that startup has finished.
    if (enableDebugger) {
      await debuggerInitialized; // Ensure we're fully initialized before sending.
    }
    sendEvent(RawEventBody(<String, Object?>{}), eventType: 'flutter.appStarted');
  }

  /// Handles the app.stop event from Flutter.
  Future<void> _handleAppStop(Map<String, Object?> params) async {
    // It's possible to get an app.stop without ever having an app.start in the
    // case of an error, so we may need to clean up the launch progress.
    // https://github.com/Dart-Code/Dart-Code/issues/5124
    // https://github.com/flutter/flutter/issues/149258
    launchProgress?.end();
    launchProgress = null;

    // If the stop had an error attached, be sure to pass it to the client.
    final Object? error = params['error'];
    if (error is String) {
      sendConsoleOutput(error);
    }
  }

  /// Handles the daemon.connected event, recording the pid of the flutter_tools process.
  void _handleDaemonConnected(Map<String, Object?> params) {
    // On Windows, the pid from the process we spawn is the shell running
    // flutter.bat and terminating it may not be reliable, so we also take the
    // pid provided from the VM running flutter_tools.
    final int? pid = params['pid'] as int?;
    if (pid != null) {
      pidsToTerminate.add(pid);
    }
  }

  /// Handles the app.debugPort event from Flutter, connecting to the VM Service if everything else is ready.
  Future<void> _handleDebugPort(Map<String, Object?> params) async {
    // Capture the VM Service URL which we'll connect to when we get app.started.
    final String? wsUri = params['wsUri'] as String?;
    if (wsUri != null) {
      final Uri vmServiceUri = Uri.parse(wsUri);
      // Also wait for app.started before we connect, to ensure Flutter's
      // initialization is all complete.
      await _appStartedCompleter.future;
      await _connectDebugger(vmServiceUri);
    }
  }

  /// Handles the Flutter process exiting, terminating the debug session if it has not already begun terminating.
  @override
  void handleExitCode(int code) {
    final String codeSuffix = code == 0 ? '' : ' ($code)';
    _logTraffic('<== [Flutter] Process exited ($code)');
    handleSessionTerminate(codeSuffix);
  }

  /// Handles incoming JSON events from `flutter run --machine`.
  void handleJsonEvent(String event, Map<String, Object?>? params) {
    params ??= <String, Object?>{};
    switch (event) {
      case 'daemon.connected':
        _handleDaemonConnected(params);
      case 'app.debugPort':
        _handleDebugPort(params);
      case 'app.start':
        _handleAppStart(params);
      case 'app.progress':
        _handleAppProgress(params);
      case 'app.started':
        _handleAppStarted();
      case 'app.stop':
        _handleAppStop(params);
    }

    if (_eventsToForwardToClient.contains(event)) {
      // Forward the event to the client.
      sendEvent(
        RawEventBody(<String, Object?>{'event': event, 'params': params}),
        eventType: 'flutter.forwardedEvent',
      );
    }
  }

  /// Handles incoming reverse requests from `flutter run --machine`.
  ///
  /// These requests are usually just forwarded to the client via an event
  /// (`flutter.forwardedRequest`) and responses are provided by the client in a
  /// custom event (`flutter.forwardedRequestResponse`).
  void _handleJsonRequest(Object id, String method, Map<String, Object?>? params) {
    /// A helper to send a client response to Flutter.
    Future<void> sendResponseToFlutter(Object? id, Object? value, {bool error = false}) async {
      await sendFlutterMessage(<String, Object?>{
        'id': id,
        if (error) 'error': value else 'result': value,
      });
    }

    // Set up a completer to forward the response back to `flutter` when it arrives.
    final Completer<Object?> completer = Completer<Object?>();
    _reverseRequestCompleters[id] = completer;
    completer.future.then(
      (Object? value) => sendResponseToFlutter(id, value),
      onError: (Object? e) => sendResponseToFlutter(id, e.toString(), error: true),
    );

    if (_requestsToForwardToClient.contains(method)) {
      // Forward the request to the client in an event.
      sendEvent(
        RawEventBody(<String, Object?>{'id': id, 'method': method, 'params': params}),
        eventType: 'flutter.forwardedRequest',
      );
    } else {
      completer.completeError(ArgumentError.value(method, 'Unknown request method.'));
    }
  }

  /// Handles client responses to reverse-requests that were forwarded from Flutter.
  void _handleForwardedResponse(RawRequestArguments? args) {
    final Object? id = args?.args['id'];
    final Object? result = args?.args['result'];
    final Object? error = args?.args['error'];
    final Completer<Object?>? completer = _reverseRequestCompleters[id];
    if (error != null) {
      completer?.completeError(
        DebugAdapterException('Client reported an error handling reverse-request $error'),
      );
    } else {
      completer?.complete(result);
    }
  }

  /// Handles incoming JSON messages from `flutter run --machine` that are responses to requests that we sent.
  void _handleJsonResponse(int id, Map<String, Object?> response) {
    final Completer<Object?>? handler = _flutterRequestCompleters.remove(id);
    if (handler == null) {
      logger?.call(
        'Received response from Flutter run daemon with ID $id '
        'but had not matching handler',
      );
      return;
    }

    final Object? error = response['error'];
    final Object? result = response['result'];
    if (error != null) {
      handler.completeError(DebugAdapterException('$error'));
    } else {
      handler.complete(result);
    }
  }

  @override
  void handleStderr(String data) {
    _logTraffic('<== [Flutter] [stderr] $data');
    sendOutput('stderr', data);
  }

  /// Handles stdout from the `flutter run --machine` process, decoding the JSON and calling the appropriate handlers.
  @override
  void handleStdout(String data) {
    // Output intended for us to parse is JSON wrapped in brackets:
    // [{"event":"app.foo","params":{"bar":"baz"}}]
    // However, it's also possible a user printed things that look a little like
    // this so try to detect only things we're interested in:
    // - parses as JSON
    // - is a List of only a single item that is a Map<String, Object?>
    // - the item has an "event" field that is a String
    // - the item has a "params" field that is a Map<String, Object?>?

    _logTraffic('<== [Flutter] $data');

    // Output is sent as console (eg. output from tooling) until the app has
    // started, then stdout (users output). This is so info like
    // "Launching lib/main.dart on Device foo" is formatted differently to
    // general output printed by the user.
    final String outputCategory = _receivedAppStarted ? 'stdout' : 'console';

    // Output in stdout can include both user output (eg. print) and Flutter
    // daemon output. Since it's not uncommon for users to print JSON while
    // debugging, we must try to detect which messages are likely Flutter
    // messages as reliably as possible, as trying to process users output
    // as a Flutter message may result in an unhandled error that will
    // terminate the debug adapter in a way that does not provide feedback
    // because the standard crash violates the DAP protocol.
    Object? jsonData;
    try {
      jsonData = jsonDecode(data);
    } on FormatException {
      // If the output wasn't valid JSON, it was standard stdout that should
      // be passed through to the user.
      sendOutput(outputCategory, data);

      // Detect if the output contains a prompt about using the Dart Debug
      // extension and also update the progress notification to make it clearer
      // we're waiting for the user to do something.
      if (data.contains('Waiting for connection from Dart debug extension')) {
        launchProgress?.update(
          message: 'Please click the Dart Debug extension button in the spawned browser window',
        );
      }

      return;
    }

    final Map<String, Object?>? payload =
        jsonData is List && jsonData.length == 1 && jsonData.first is Map<String, Object?>
            ? jsonData.first as Map<String, Object?>
            : null;

    if (payload == null) {
      // JSON didn't match expected format for Flutter responses, so treat as
      // standard user output.
      sendOutput(outputCategory, data);
      return;
    }

    final Object? event = payload['event'];
    final Object? method = payload['method'];
    final Object? params = payload['params'];
    final Object? id = payload['id'];
    if (event is String && params is Map<String, Object?>?) {
      handleJsonEvent(event, params);
    } else if (id != null && method is String && params is Map<String, Object?>?) {
      _handleJsonRequest(id, method, params);
    } else if (id is int && _flutterRequestCompleters.containsKey(id)) {
      _handleJsonResponse(id, payload);
    } else {
      // If it wasn't processed above,
      sendOutput(outputCategory, data);
    }
  }

  /// Logs JSON traffic to aid debugging.
  ///
  /// If `sendLogsToClient` was `true` in the launch/attach config, logs will
  /// also be sent back to the client in a "dart.log" event to simplify
  /// capturing logs from the IDE (such as using the **Dart: Capture Logs**
  /// command in VS Code).
  void _logTraffic(String message) {
    logger?.call(message);
    if (sendLogsToClient) {
      sendEvent(RawEventBody(<String, String>{'message': message}), eventType: 'dart.log');
    }
  }

  /// Performs a restart/reload by sending the `app.restart` message to the `flutter run --machine` process.
  Future<void> _performRestart(bool fullRestart, [String? reason]) async {
    // Don't do anything if the app hasn't started yet, as restarts and reloads
    // can only operate on a running app.
    if (_appId == null) {
      return;
    }

    final String progressId = fullRestart ? 'hotRestart' : 'hotReload';
    final String progressMessage = fullRestart ? 'Hot restarting…' : 'Hot reloading…';
    final DapProgressReporter progress = startProgressNotification(
      progressId,
      'Flutter',
      message: progressMessage,
    );

    try {
      await sendFlutterRequest('app.restart', <String, Object?>{
        'appId': _appId,
        'fullRestart': fullRestart,
        'pause': enableDebugger,
        'reason': reason,
        'debounce': true,
      });
    } on DebugAdapterException catch (error) {
      final String action = fullRestart ? 'Hot Restart' : 'Hot Reload';
      sendOutput('console', 'Failed to $action: $error');
    } finally {
      progress.end();
    }
  }

  void _sendServiceExtensionStateChanged(vm.ExtensionData? extensionData) {
    final Map<String, dynamic>? data = extensionData?.data;
    if (data != null) {
      sendEvent(RawEventBody(data), eventType: 'flutter.serviceExtensionStateChanged');
    }
  }
}
