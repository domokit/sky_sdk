// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:dds/dap.dart';
import 'package:file/file.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:test/test.dart';

import 'test_client.dart';
import 'test_server.dart';

/// Whether to run the DAP server in-process with the tests, or externally in
/// another process.
///
/// By default tests will run the DAP server out-of-process to match the real
/// use from editors, but this complicates debugging the adapter. Set this env
/// variables to run the server in-process for easier debugging (this can be
/// simplified in VS Code by using a launch config with custom CodeLens links).
final bool useInProcessDap = Platform.environment['DAP_TEST_INTERNAL'] == 'true';

/// Whether to print all protocol traffic to stdout while running tests.
///
/// This is useful for debugging locally or on the bots and will include both
/// DAP traffic (between the test DAP client and the DAP server) and the VM
/// Service traffic (wrapped in a custom 'dart.log' event).
final bool verboseLogging = Platform.environment['DAP_TEST_VERBOSE'] == 'true';

const String endOfErrorOutputMarker =
    '════════════════════════════════════════════════════════════════════════════════';

/// Expects the lines in [actual] to match the relevant matcher in [expected],
/// ignoring differences in line endings and trailing whitespace.
void expectLines(String actual, List<Object> expected, {bool allowExtras = false}) {
  if (allowExtras) {
    expect(actual.replaceAll('\r\n', '\n').trim().split('\n'), containsAllInOrder(expected));
  } else {
    expect(actual.replaceAll('\r\n', '\n').trim().split('\n'), equals(expected));
  }
}

/// Manages running a simple Flutter app to be used in tests that need to attach
/// to an existing process.
class SimpleFlutterRunner {
  SimpleFlutterRunner(this.process) {
    process.stdout.transform(ByteToLineTransformer()).listen(_handleStdout);
    process.stderr.transform(utf8.decoder).listen(_handleStderr);
    unawaited(process.exitCode.then(_handleExitCode));
  }

  final StreamController<String> _output = StreamController<String>.broadcast();

  /// A broadcast stream of any non-JSON output from the process.
  Stream<String> get output => _output.stream;

  void _handleExitCode(int code) {
    if (!_vmServiceUriCompleter.isCompleted) {
      _vmServiceUriCompleter.completeError(
        'Flutter process ended without producing a VM Service URI',
      );
    }
  }

  void _handleStderr(String err) {
    if (!_vmServiceUriCompleter.isCompleted) {
      _vmServiceUriCompleter.completeError(err);
    }
  }

  void _handleStdout(String outputLine) {
    try {
      final Object? json = jsonDecode(outputLine);
      // Flutter --machine output is wrapped in [brackets] so will deserialize
      // as a list with one item.
      if (json is List && json.length == 1) {
        final Object? message = json.single;
        // Parse the add.debugPort event which contains our VM Service URI.
        if (message is Map<String, Object?> && message['event'] == 'app.debugPort') {
          final String vmServiceUri =
              (message['params']! as Map<String, Object?>)['wsUri']! as String;
          if (!_vmServiceUriCompleter.isCompleted) {
            _vmServiceUriCompleter.complete(Uri.parse(vmServiceUri));
          }
        }
      }
    } on FormatException {
      // `flutter run` writes a lot of text to stdout that isn't daemon messages
      //  (not valid JSON), so just pass that one for tests that may want it.
      _output.add(outputLine);
    }
  }

  final Process process;
  final Completer<Uri> _vmServiceUriCompleter = Completer<Uri>();
  Future<Uri> get vmServiceUri => _vmServiceUriCompleter.future;

  static Future<SimpleFlutterRunner> start(Directory projectDirectory) async {
    final String flutterToolPath = globals.fs.path.join(
      Cache.flutterRoot!,
      'bin',
      globals.platform.isWindows ? 'flutter.bat' : 'flutter',
    );

    final List<String> args = <String>['run', '--machine', '-d', 'flutter-tester'];

    final Process process = await Process.start(
      flutterToolPath,
      args,
      workingDirectory: projectDirectory.path,
    );

    return SimpleFlutterRunner(process);
  }
}

/// A helper class containing the DAP server/client for DAP integration tests.
class DapTestSession {
  DapTestSession._(this.server, this.client);

  DapTestServer server;
  DapTestClient client;

  Future<void> tearDown() async {
    await client.stop();
    await server.stop();
  }

  static Future<DapTestSession> setUp({List<String>? additionalArgs}) async {
    final DapTestServer server = await _startServer(additionalArgs: additionalArgs);
    final DapTestClient client = await DapTestClient.connect(
      server,
      captureVmServiceTraffic: verboseLogging,
      logger: verboseLogging ? print : null,
    );
    return DapTestSession._(server, client);
  }

  /// Starts a DAP server that can be shared across tests.
  static Future<DapTestServer> _startServer({Logger? logger, List<String>? additionalArgs}) async {
    return useInProcessDap
        ? await InProcessDapTestServer.create(logger: logger, additionalArgs: additionalArgs)
        : await OutOfProcessDapTestServer.create(logger: logger, additionalArgs: additionalArgs);
  }
}
