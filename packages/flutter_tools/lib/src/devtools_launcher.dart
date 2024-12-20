// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import 'artifacts.dart';
import 'base/bot_detector.dart';
import 'base/common.dart';
import 'base/io.dart' as io;
import 'base/logger.dart';
import 'convert.dart';
import 'resident_runner.dart';

/// An implementation of the devtools launcher that uses `dart devtools` to
/// start a DevTools server instance.
class DevtoolsServerLauncher extends DevtoolsLauncher {
  DevtoolsServerLauncher({
    required ProcessManager processManager,
    required Logger logger,
    required BotDetector botDetector,
    required Artifacts artifacts,
  }) : _processManager = processManager,
       _logger = logger,
       _botDetector = botDetector,
       _artifacts = artifacts;

  final ProcessManager _processManager;
  final Artifacts _artifacts;
  late final String _dartExecutable = _artifacts.getArtifactPath(Artifact.engineDartBinary);
  final Logger _logger;
  final BotDetector _botDetector;
  final Completer<void> _processStartCompleter = Completer<void>();

  io.Process? _devToolsProcess;
  bool _devToolsProcessKilled = false;
  @visibleForTesting
  Future<void>? devToolsProcessExit;

  static final RegExp _serveDevToolsPattern = RegExp(
    r'Serving DevTools at ((http|//)[a-zA-Z0-9:/=_\-\.\[\]]+?)\.?$',
  );
  static final RegExp _serveDtdPattern = RegExp(
    r'Serving the Dart Tooling Daemon at (ws:\/\/[a-zA-Z0-9:/=_\-\.\[\]]+?)\.?$',
  );

  @override
  Future<void> get processStart => _processStartCompleter.future;

  @override
  Future<void> launch(Uri? vmServiceUri, {List<String>? additionalArguments}) async {
    // Place this entire method in a try/catch that swallows exceptions because
    // this method is guaranteed not to return a Future that throws.
    try {
      _devToolsProcess = await _processManager.start(<String>[
        _dartExecutable,
        'devtools',
        '--no-launch-browser',
        if (printDtdUri) '--print-dtd',
        if (vmServiceUri != null) '--vm-uri=$vmServiceUri',
        ...?additionalArguments,
      ]);
      _processStartCompleter.complete();

      final Completer<Uri> devToolsCompleter = Completer<Uri>();
      _devToolsProcess!.stdout.transform(utf8.decoder).transform(const LineSplitter()).listen((
        String line,
      ) {
        final Match? dtdMatch = _serveDtdPattern.firstMatch(line);
        if (dtdMatch != null) {
          final String uri = dtdMatch[1]!;
          dtdUri = Uri.parse(uri);
        }
        final Match? devToolsMatch = _serveDevToolsPattern.firstMatch(line);
        if (devToolsMatch != null) {
          final String url = devToolsMatch[1]!;
          devToolsCompleter.complete(Uri.parse(url));
        }
      });
      _devToolsProcess!.stderr
          .transform(utf8.decoder)
          .transform(const LineSplitter())
          .listen(_logger.printError);

      final bool runningOnBot = await _botDetector.isRunningOnBot;
      devToolsProcessExit = _devToolsProcess!.exitCode.then((int exitCode) {
        if (!_devToolsProcessKilled && runningOnBot) {
          throwToolExit('DevTools process failed: exitCode=$exitCode');
        }
      });

      // We do not need to wait for a [Completer] holding the DTD URI because
      // the DTD URI will be output to stdout before the DevTools URI. Awaiting
      // a [Completer] for the DevTools URI ensures both values will be
      // populated before returning.
      devToolsUrl = await devToolsCompleter.future;
    } on Exception catch (e, st) {
      _logger.printError('Failed to launch DevTools: $e', stackTrace: st);
    }
  }

  @override
  Future<DevToolsServerAddress?> serve() async {
    if (activeDevToolsServer == null) {
      await launch(null);
    }
    return activeDevToolsServer;
  }

  @override
  Future<void> close() async {
    if (devToolsUrl != null) {
      devToolsUrl = null;
    }
    if (_devToolsProcess != null) {
      _devToolsProcessKilled = true;
      _devToolsProcess!.kill();
    }
  }
}
