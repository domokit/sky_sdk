// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:dds/dap.dart' hide PidTracker;

import '../base/io.dart';
import '../cache.dart';
import '../convert.dart';
import 'flutter_adapter_args.dart';
import 'flutter_base_adapter.dart';

/// A DAP Debug Adapter for running and debugging Flutter tests.
class FlutterTestDebugAdapter extends FlutterBaseDebugAdapter with TestAdapter {
  FlutterTestDebugAdapter(
    super.channel, {
    required super.fileSystem,
    required super.platform,
    super.ipv6,
    super.enableFlutterDds = true,
    super.enableAuthCodes,
    super.logger,
    super.onError,
  });

  /// Called by [attachRequest] to request that we actually connect to the app to be debugged.
  @override
  Future<void> attachImpl() async {
    sendOutput('console', '\nAttach is not currently supported');
    handleSessionTerminate();
  }

  /// Called by [launchRequest] to request that we actually start the tests to be run/debugged.
  ///
  /// For debugging, this should start paused, connect to the VM Service, set
  /// breakpoints, and resume.
  @override
  Future<void> launchImpl() async {
    final FlutterLaunchRequestArguments args = this.args as FlutterLaunchRequestArguments;

    final bool debug = enableDebugger;
    final String? program = args.program;

    final List<String> toolArgs = <String>[
      'test',
      '--machine',
      if (!enableFlutterDds) '--no-dds',
      if (debug) '--start-paused',
    ];

    // Handle customTool and deletion of any arguments for it.
    final String executable =
        args.customTool ??
        fileSystem.path.join(
          Cache.flutterRoot!,
          'bin',
          platform.isWindows ? 'flutter.bat' : 'flutter',
        );
    final int? removeArgs = args.customToolReplacesArgs;
    if (args.customTool != null && removeArgs != null) {
      toolArgs.removeRange(0, math.min(removeArgs, toolArgs.length));
    }

    final List<String> processArgs = <String>[
      ...toolArgs,
      ...?args.toolArgs,
      if (program != null) program,
      ...?args.args,
    ];

    await launchAsProcess(executable: executable, processArgs: processArgs, env: args.env);

    // Delay responding until the debugger is connected.
    if (debug) {
      await debuggerInitialized;
    }
  }

  /// Called by [terminateRequest] to request that we gracefully shut down the app being run (or in the case of an attach, disconnect).
  @override
  Future<void> terminateImpl() async {
    terminatePids(ProcessSignal.sigterm);
    await process?.exitCode;
  }

  /// Handles the Flutter process exiting, terminating the debug session if it has not already begun terminating.
  @override
  void handleExitCode(int code) {
    final String codeSuffix = code == 0 ? '' : ' ($code)';
    logger?.call('Process exited ($code)');
    handleSessionTerminate(codeSuffix);
  }

  /// Handles incoming JSON events from `flutter test --machine`.
  bool _handleJsonEvent(String event, Map<String, Object?>? params) {
    params ??= <String, Object?>{};
    switch (event) {
      case 'test.startedProcess':
        _handleTestStartedProcess(params);
        return true;
    }

    return false;
  }

  @override
  void handleStderr(String data) {
    logger?.call('stderr: $data');
    sendOutput('stderr', data);
  }

  /// Handles stdout from the `flutter test --machine` process, decoding the JSON and calling the appropriate handlers.
  @override
  void handleStdout(String data) {
    // Output to stdout from `flutter test --machine` is either:
    //   1. JSON output from flutter_tools (eg. "test.startedProcess") which is
    //      wrapped in [] brackets and has an event/params.
    //   2. JSON output from package:test (not wrapped in brackets).
    //   3. Non-JSON output (user messages, or flutter_tools printing things like
    //      call stacks/error information).
    logger?.call('stdout: $data');

    Object? jsonData;
    try {
      jsonData = jsonDecode(data);
    } on FormatException {
      // If the output wasn't valid JSON, it was standard stdout that should
      // be passed through to the user.
      sendOutput('stdout', data);
      return;
    }

    // Check for valid flutter_tools JSON output (1) first.
    final Map<String, Object?>? flutterPayload =
        jsonData is List && jsonData.length == 1 && jsonData.first is Map<String, Object?>
            ? jsonData.first as Map<String, Object?>
            : null;
    final Object? event = flutterPayload?['event'];
    final Object? params = flutterPayload?['params'];

    if (event is String && params is Map<String, Object?>?) {
      _handleJsonEvent(event, params);
    } else if (jsonData != null) {
      // Handle package:test output (2).
      sendTestEvents(jsonData);
    } else {
      // Other output should just be passed straight through.
      sendOutput('stdout', data);
    }
  }

  /// Handles the test.processStarted event from Flutter that provides the VM Service URL.
  void _handleTestStartedProcess(Map<String, Object?> params) {
    final String? vmServiceUriString = params['vmServiceUri'] as String?;
    // For no-debug mode, this event may be still sent so ignore it if we know
    // we're not debugging, or its URI is null.
    if (!enableDebugger || vmServiceUriString == null) {
      return;
    }
    final Uri vmServiceUri = Uri.parse(vmServiceUriString);
    connectDebugger(vmServiceUri);
  }
}
