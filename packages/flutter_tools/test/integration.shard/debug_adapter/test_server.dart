// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:dds/dap.dart' show Logger;
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/debug_adapters/server.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

/// Enable to run from local source when running out-of-process (useful in
/// development to avoid having to keep rebuilding the flutter tool).
const bool _runFromSource = false;

abstract class DapTestServer {
  Future<void> stop();
  StreamSink<List<int>> get sink;
  Stream<List<int>> get stream;
  void Function(String message)? onStderrOutput;
}

/// An instance of a DAP server running in-process (to aid debugging).
///
/// All communication still goes over the socket to ensure all messages are
/// serialized and deserialized but it's not quite the same running out of
/// process.
class InProcessDapTestServer extends DapTestServer {
  InProcessDapTestServer._(List<String> args) {
    _server = DapServer(
      stdinController.stream,
      stdoutController.sink,
      fileSystem: globals.fs,
      platform: globals.platform,
      // Simulate flags based on the args to aid testing.
      enableDds: !args.contains('--no-dds'),
      ipv6: args.contains('--ipv6'),
      test: args.contains('--test'),
    );
  }

  late final DapServer _server;
  final StreamController<List<int>> stdinController = StreamController<List<int>>();
  final StreamController<List<int>> stdoutController = StreamController<List<int>>();

  @override
  StreamSink<List<int>> get sink => stdinController.sink;

  @override
  Stream<List<int>> get stream => stdoutController.stream;

  @override
  Future<void> stop() async {
    _server.stop();
  }

  static Future<InProcessDapTestServer> create({
    Logger? logger,
    List<String>? additionalArgs,
  }) async {
    return InProcessDapTestServer._(additionalArgs ?? <String>[]);
  }
}

/// An instance of a DAP server running out-of-process.
///
/// This is how an editor will usually consume DAP so is a more accurate test
/// but will be a little more difficult to debug tests as the debugger will not
/// be attached to the process.
class OutOfProcessDapTestServer extends DapTestServer {
  OutOfProcessDapTestServer._(this._process, Logger? logger) {
    // Unless we're given an error handler, treat anything written to stderr as
    // the DAP crashing and fail the test unless it's "Waiting for another
    // flutter command to release the startup lock" or we're tearing down.
    _process.stderr
        .transform(utf8.decoder)
        .where(
          (String error) =>
              !error.contains('Waiting for another flutter command to release the startup lock'),
        )
        .listen((String error) {
          logger?.call(error);
          if (!_isShuttingDown) {
            final void Function(String message)? stderrHandler = onStderrOutput;
            if (stderrHandler != null) {
              stderrHandler(error);
            } else {
              throw Exception(error);
            }
          }
        });
    unawaited(
      _process.exitCode.then((int code) {
        final String message = 'Out-of-process DAP server terminated with code $code';
        logger?.call(message);
        if (!_isShuttingDown && code != 0 && onStderrOutput == null) {
          throw Exception(message);
        }
      }),
    );
  }

  bool _isShuttingDown = false;
  final Process _process;

  Future<int> get exitCode => _process.exitCode;

  @override
  StreamSink<List<int>> get sink => _process.stdin;

  @override
  Stream<List<int>> get stream => _process.stdout;

  @override
  Future<void> stop() async {
    _isShuttingDown = true;
    _process.kill();
    await _process.exitCode;
  }

  static Future<OutOfProcessDapTestServer> create({
    Logger? logger,
    List<String>? additionalArgs,
  }) async {
    // runFromSource=true will run "dart bin/flutter_tools.dart ..." to avoid
    // having to rebuild the flutter_tools snapshot.
    // runFromSource=false will run "flutter ..."

    final String flutterToolPath = globals.fs.path.join(
      Cache.flutterRoot!,
      'bin',
      Platform.isWindows ? 'flutter.bat' : 'flutter',
    );
    final String flutterToolsEntryScript = globals.fs.path.join(
      Cache.flutterRoot!,
      'packages',
      'flutter_tools',
      'bin',
      'flutter_tools.dart',
    );

    // When running from source, run "dart bin/flutter_tools.dart debug_adapter"
    // instead of directly using "flutter debug_adapter".
    final String executable = _runFromSource ? Platform.resolvedExecutable : flutterToolPath;
    final List<String> args = <String>[
      if (_runFromSource) flutterToolsEntryScript,
      'debug-adapter',
      ...?additionalArgs,
    ];

    final Process process = await Process.start(executable, args);

    return OutOfProcessDapTestServer._(process, logger);
  }
}
