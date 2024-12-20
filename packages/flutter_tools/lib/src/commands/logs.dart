// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import '../application_package.dart';
import '../base/common.dart';
import '../base/io.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../runner/flutter_command.dart';

class LogsCommand extends FlutterCommand {
  LogsCommand({required this.sigint, required this.sigterm}) {
    argParser.addFlag(
      'clear',
      negatable: false,
      abbr: 'c',
      help: 'Clear log history before reading from logs.',
    );
    usesDeviceTimeoutOption();
    usesDeviceConnectionOption();
  }

  @override
  final String name = 'logs';

  @override
  final String description = 'Show log output for running Flutter apps.';

  @override
  final String category = FlutterCommandCategory.tools;

  @override
  bool get refreshWirelessDevices => true;

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => const <DevelopmentArtifact>{};

  Device? device;
  final ProcessSignal sigint;
  final ProcessSignal sigterm;

  @override
  Future<FlutterCommandResult> verifyThenRunCommand(String? commandPath) async {
    device = await findTargetDevice(includeDevicesUnsupportedByProject: true);
    if (device == null) {
      throwToolExit(null);
    }
    return super.verifyThenRunCommand(commandPath);
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    final Device cachedDevice = device!;
    if (boolArg('clear')) {
      cachedDevice.clearLogs();
    }

    final ApplicationPackage? app = await applicationPackages?.getPackageForPlatform(
      await cachedDevice.targetPlatform,
    );

    final DeviceLogReader logReader = await cachedDevice.getLogReader(app: app);

    globals.printStatus('Showing $logReader logs:');

    final Completer<int> exitCompleter = Completer<int>();

    // First check if we already completed by another branch before completing
    // with [exitCode].
    void maybeComplete([int exitCode = 0]) {
      if (exitCompleter.isCompleted) {
        return;
      }
      exitCompleter.complete(exitCode);
    }

    // Start reading.
    final StreamSubscription<String> subscription = logReader.logLines.listen(
      (String message) => globals.printStatus(message, wrap: false),
      onDone: () => maybeComplete(),
      onError: (dynamic error) => maybeComplete(error is int ? error : 1),
    );

    // When terminating, close down the log reader.
    sigint.watch().listen((ProcessSignal signal) {
      subscription.cancel();
      maybeComplete();
      globals.printStatus('');
    });
    sigterm.watch().listen((ProcessSignal signal) {
      subscription.cancel();
      maybeComplete();
    });

    // Wait for the log reader to be finished.
    final int result = await exitCompleter.future;
    await subscription.cancel();
    if (result != 0) {
      throwToolExit('Error listening to $logReader logs.');
    }

    return FlutterCommandResult.success();
  }
}
