// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ffi' as ffi;
import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;

void main(List<String> args) async {
  if (!io.Platform.isMacOS) {
    io.stderr.writeln('This script is only supported on macOS.');
    io.exitCode = 1;
    return;
  }

  final engine = Engine.tryFindWithin();
  if (engine == null) {
    io.stderr.writeln('Must be run from within the engine repository.');
    io.exitCode = 1;
    return;
  }

  if (args.length > 1 || args.contains('-h') || args.contains('--help')) {
    io.stderr.writeln('Usage: run_ios_tests.dart [ios_engine_variant]');
    io.stderr.writeln(_args.usage);
    io.exitCode = 1;
    return;
  }

  // Collect cleanup tasks to run when the script terminates.
  final cleanup = <FutureOr<void> Function()>{};

  // Parse the command-line arguments.
  final results = _args.parse(args);
  final String iosEngineVariant;
  if (results.rest case [final variant]) {
    iosEngineVariant = variant;
  } else if (ffi.Abi.current() == ffi.Abi.macosArm64) {
    iosEngineVariant = 'ios_debug_sim_unopt_arm64';
  } else {
    iosEngineVariant = 'ios_debug_sim_unopt';
  }

  // Null if the tests should create and dispose their own temporary directory.
  String? dumpXcresultOnFailurePath;
  if (results.option('dump-xcresult-on-failure') case final String path) {
    dumpXcresultOnFailurePath = path;
  }

  // Run the actual script.
  final completer = Completer<void>();
  runZonedGuarded(
    () async {
      await _run(
        cleanup,
        engine,
        iosEngineVariant: iosEngineVariant,
        deviceName: results.option('device-name')!,
        deviceIdentifier: results.option('device-identifier')!,
        osRuntime: results.option('os-runtime')!,
        osVersion: results.option('os-version')!,
        withImpeller: results.flag('with-impeller'),
        dumpXcresultOnFailure: dumpXcresultOnFailurePath,
      );
      completer.complete();
    },
    (e, s) {
      if (e is _ToolFailure) {
        io.stderr.writeln(e);
        io.exitCode = 1;
      } else {
        io.stderr.writeln('Uncaught exception: $e\n$s');
        io.exitCode = 255;
      }
      completer.complete();
    },
  );

  // We can't await the result of runZonedGuarded becauase async errors in futures never cross different errorZone boundaries.
  await completer.future;

  // Run cleanup tasks.
  for (final task in cleanup) {
    await task();
  }
}

void _deleteIfPresent(io.FileSystemEntity entity) {
  if (entity.existsSync()) {
    entity.deleteSync(recursive: true);
  }
}

/// Runs the script.
///
/// The [cleanup] set contains cleanup tasks to run when the script is either
/// completed normally or terminated early. For example, deleting a temporary
/// directory or killing a process.
///
/// Each named argument cooresponds to a flag or option in the `ArgParser`.
Future<void> _run(
  Set<FutureOr<void> Function()> cleanup,
  Engine engine, {
  required String iosEngineVariant,
  required String deviceName,
  required String deviceIdentifier,
  required String osRuntime,
  required String osVersion,
  required bool withImpeller,
  required String? dumpXcresultOnFailure,
}) async {
  // Terminate early on SIGINT.
  late final StreamSubscription<void> sigint;
  sigint = io.ProcessSignal.sigint.watch().listen((_) {
    throw _ToolFailure('Received SIGINT');
  });
  cleanup.add(sigint.cancel);

  _ensureSimulatorsRotateAutomaticallyForPlatformViewRotationTest();
  _deleteAnyExistingDevices(deviceName: deviceName);
  _createDevice(deviceName: deviceName, deviceIdentifier: deviceIdentifier, osRuntime: osRuntime);

  final (scenarioPath, resultBundle) = _buildResultBundlePath(
    engine: engine,
    iosEngineVariant: iosEngineVariant,
  );

  cleanup.add(() => _deleteIfPresent(resultBundle));

  if (withImpeller) {
    final process = await _runTests(
      outScenariosPath: scenarioPath,
      resultBundlePath: resultBundle.path,
      osVersion: osVersion,
      deviceName: deviceName,
      iosEngineVariant: iosEngineVariant,
    );
    cleanup.add(process.kill);

    // Create a temporary directory, if needed.
    var storePath = dumpXcresultOnFailure;
    if (storePath == null) {
      final dumpDir = io.Directory.systemTemp.createTempSync();
      storePath = dumpDir.path;
      cleanup.add(() => dumpDir.delete(recursive: true));
    }

    if (await process.exitCode != 0) {
      final String outputPath = _zipAndStoreFailedTestResults(
        iosEngineVariant: iosEngineVariant,
        resultBundle: resultBundle,
        storePath: storePath,
      );
      io.stderr.writeln('Failed test results are stored at $outputPath');
      throw _ToolFailure('test failed.');
    } else {
      io.stderr.writeln('test succcess.');
    }
    _deleteIfPresent(resultBundle);
  }
}

/// Exception thrown when the tool should halt execution intentionally.
final class _ToolFailure implements Exception {
  _ToolFailure(this.message);

  final String message;

  @override
  String toString() => message;
}

final _args =
    ArgParser()
      ..addFlag('help', abbr: 'h', help: 'Prints usage information.', negatable: false)
      ..addOption(
        'device-name',
        help: 'The name of the iOS simulator device to use.',
        defaultsTo: 'iPhone SE (3rd generation)',
      )
      ..addOption(
        'device-identifier',
        help: 'The identifier of the iOS simulator device to use.',
        defaultsTo: 'com.apple.CoreSimulator.SimDeviceType.iPhone-SE-3rd-generation',
      )
      ..addOption(
        'os-runtime',
        help: 'The OS runtime of the iOS simulator device to use.',
        defaultsTo: 'com.apple.CoreSimulator.SimRuntime.iOS-17-0',
      )
      ..addOption(
        'os-version',
        help: 'The OS version of the iOS simulator device to use.',
        defaultsTo: '17.0',
      )
      ..addFlag(
        'with-impeller',
        help: 'Whether to use the Impeller backend to run the tests.',
        defaultsTo: true,
      )
      ..addOption(
        'dump-xcresult-on-failure',
        help:
            'The path to dump the xcresult bundle to if the test fails.\n\n'
            'Defaults to the environment variable FLUTTER_TEST_OUTPUTS_DIR, '
            'otherwise to a randomly generated temporary directory.',
        defaultsTo: io.Platform.environment['FLUTTER_TEST_OUTPUTS_DIR'],
      );

void _ensureSimulatorsRotateAutomaticallyForPlatformViewRotationTest() {
  // Can also be set via Simulator Device > Rotate Device Automatically.
  final result = io.Process.runSync('defaults', const [
    'write',
    'com.apple.iphonesimulator',
    'RotateWindowWhenSignaledByGuest',
    '-int 1',
  ]);
  if (result.exitCode != 0) {
    throw Exception('Failed to enable automatic rotation for iOS simulator: ${result.stderr}');
  }
}

void _deleteAnyExistingDevices({required String deviceName}) {
  io.stderr.writeln('Deleting any existing simulator devices named $deviceName...');

  bool deleteSimulator() {
    final result = io.Process.runSync('xcrun', ['simctl', 'delete', deviceName]);
    if (result.exitCode == 0) {
      io.stderr.writeln('Deleted $deviceName');
      return true;
    } else {
      return false;
    }
  }

  while (deleteSimulator()) {}
}

void _createDevice({
  required String deviceName,
  required String deviceIdentifier,
  required String osRuntime,
}) {
  io.stderr.writeln('Creating $deviceName $deviceIdentifier $osRuntime...');
  final result = io.Process.runSync('xcrun', [
    'simctl',
    'create',
    deviceName,
    deviceIdentifier,
    osRuntime,
  ]);
  if (result.exitCode != 0) {
    throw Exception('Failed to create simulator device: ${result.stderr}');
  }
}

@useResult
(String scenarios, io.Directory resultBundle) _buildResultBundlePath({
  required Engine engine,
  required String iosEngineVariant,
}) {
  final scenarioPath = path.normalize(
    path.join(engine.outDir.path, iosEngineVariant, 'scenario_app', 'Scenarios'),
  );

  // Create a temporary directory to store the test results.
  final result = io.Directory(scenarioPath).createTempSync('ios_scenario_xcresult');
  return (scenarioPath, result);
}

@useResult
Future<io.Process> _runTests({
  required String resultBundlePath,
  required String outScenariosPath,
  required String osVersion,
  required String deviceName,
  required String iosEngineVariant,
  List<String> xcodeBuildExtraArgs = const [],
}) async {
  return io.Process.start('xcodebuild', [
    '-project',
    path.join(outScenariosPath, 'Scenarios.xcodeproj'),
    '-sdk',
    'iphonesimulator',
    '-scheme',
    'Scenarios',
    '-resultBundlePath',
    path.join(resultBundlePath, 'ios_scenario.xcresult'),
    '-destination',
    'platform=iOS Simulator,OS=$osVersion,name=$deviceName',
    'clean',
    'test',
    'FLUTTER_ENGINE=$iosEngineVariant',
    ...xcodeBuildExtraArgs,
  ], mode: io.ProcessStartMode.inheritStdio);
}

@useResult
String _zipAndStoreFailedTestResults({
  required String iosEngineVariant,
  required io.Directory resultBundle,
  required String storePath,
}) {
  final outputPath = path.join(storePath, '${iosEngineVariant.replaceAll('/', '_')}.zip');
  final result = io.Process.runSync('zip', ['-q', '-r', outputPath, resultBundle.path]);
  if (result.exitCode != 0) {
    throw Exception(
      'Failed to zip the test results (exit code = ${result.exitCode}).\n\n'
      'Stderr: ${result.stderr}\n\n'
      'Stdout: ${result.stdout}',
    );
  }
  return outputPath;
}
