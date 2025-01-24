// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:path/path.dart' as p;

final ArgParser _argParser =
    ArgParser()
      ..addFlag('help', abbr: 'h', help: 'Display usage information.', negatable: false)
      ..addFlag('verbose', abbr: 'v', help: 'Show noisy output while running', negatable: false)
      ..addFlag(
        'generate-initial-golden',
        help:
            'Whether an initial run (not part of "runs") should generate the '
            'base golden file. If false, it is assumed the golden file wasl already generated.',
        defaultsTo: true,
      )
      ..addFlag(
        'build-app-once',
        help:
            'Whether to use flutter build and --use-application-binary instead of rebuilding every iteration.',
        defaultsTo: true,
      )
      ..addOption('runs', abbr: 'n', help: 'How many times to run the test.', defaultsTo: '10');

/// Builds, establishes a baseline, and runs a golden-file test N number of times.
///
/// Example use:
/// ```sh
/// dart ./tool/deflake.dart lib/external_texture/surface_texture_smiley_face_main.dart
/// ```
///
/// By default it will:
/// - Build the app once (and reuse the APK);
/// - Generate a baseline (local) golden-file, overwriting your local file system;
/// - Run N (by default, 10) subsequent tests, asserting the generated golden exactly matches.
///
/// For advanced usage, see `dart ./tool/deflake.dart --help`.
void main(List<String> args) async {
  final ArgResults argResults = _argParser.parse(args);
  if (argResults.flag('help')) {
    return _printUsage();
  }

  final List<String> testFiles = argResults.rest;
  if (testFiles.length != 1) {
    io.stderr.writeln('Exactly one test-file must be specified');
    _printUsage();
    io.exitCode = 1;
    return;
  }

  final io.File testFile = io.File(testFiles.single);
  if (!testFile.existsSync()) {
    io.stderr.writeln('Not a file: ${testFile.path}');
    _printUsage();
    io.exitCode = 1;
    return;
  }

  final bool generateInitialGolden = argResults.flag('generate-initial-golden');
  final bool buildAppOnce = argResults.flag('build-app-once');
  final bool verbose = argResults.flag('verbose');
  final int runs;
  {
    final String rawRuns = argResults.option('runs')!;
    final int? parsedRuns = int.tryParse(rawRuns);
    if (parsedRuns == null || parsedRuns < 1) {
      io.stderr.writeln('--runs must be a positive number: "$rawRuns".');
      io.exitCode = 1;
      return;
    }
    runs = parsedRuns;
  }

  final List<String> driverArgs;
  if (buildAppOnce) {
    io.stderr.writeln('Building initial app with "flutter build apk --debug...');
    final io.Process proccess = await io.Process.start('flutter', <String>[
      'build',
      'apk',
      '--debug',
      testFile.path,
    ], mode: verbose ? io.ProcessStartMode.inheritStdio : io.ProcessStartMode.normal);
    if (await proccess.exitCode case final int exitCode when exitCode != 0) {
      io.stderr.writeln('Failed to build (exit code = $exitCode).');
      io.stderr.writeln(_collectStdOut(proccess));
      io.exitCode = 1;
      return;
    }

    // Strictly speaking, it would be better to parse stdout for:
    // "✓ Built build/app/outputs/flutter-apk/app-debug.apk"
    //
    // ... _or_ specify the expected out ourselves and rely on that.
    driverArgs = <String>[
      'drive',
      '--use-application-binary',
      p.join('build', 'app', 'outputs', 'flutter-apk', 'app-debug.apk'),
      testFile.path,
    ];
  } else {
    // I can't imagine wanting to do this, but here is the option anyway!
    driverArgs = <String>['drive', testFile.path];
  }

  Future<bool> runDriverTest({Map<String, String>? environment}) async {
    final io.Process proccess = await io.Process.start(
      'flutter',
      driverArgs,
      mode: verbose ? io.ProcessStartMode.inheritStdio : io.ProcessStartMode.normal,
      environment: environment,
    );
    if (await proccess.exitCode case final int exitCode when exitCode != 0) {
      io.stderr.writeln('Failed to build (exit code = $exitCode).');
      io.stderr.writeln(_collectStdOut(proccess));
      return false;
    }
    return true;
  }

  // Do an initial baseline run.
  if (generateInitialGolden) {
    io.stderr.writeln('Generating a baseline set of golden-files...');
    await runDriverTest(environment: <String, String>{'UPDATE_GOLDENS': '1'});
  }

  // Now run.
  int totalFailed = 0;
  for (int i = 0; i < runs; i++) {
    io.stderr.writeln('RUN ${i + 1} of $runs');
    final bool result = await runDriverTest();
    if (!result) {
      totalFailed++;
      io.stderr.writeln('FAIL');
    } else {
      io.stderr.writeln('PASS');
    }
  }

  io.stderr.writeln('PASSED: ${runs - totalFailed} / $runs');
  if (totalFailed != 0) {
    io.exitCode = 1;
  }
}

void _printUsage() {
  io.stdout.writeln('Usage: dart tool/deflake.dart lib/<path-to-main>.dart');
  io.stdout.writeln(_argParser.usage);
}

Future<String> _collectStdOut(io.Process process) async {
  final StringBuffer buffer = StringBuffer();
  buffer.writeln('stdout:');
  buffer.writeln(await utf8.decodeStream(process.stdout));
  buffer.writeln('stderr:');
  buffer.writeln(await utf8.decodeStream(process.stderr));
  return buffer.toString();
}
