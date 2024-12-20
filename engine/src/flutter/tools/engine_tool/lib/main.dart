// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' as ffi show Abi;
import 'dart:io' as io show Directory, Platform, exitCode, stderr;

import 'package:engine_build_configs/engine_build_configs.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as p;
import 'package:platform/platform.dart';
import 'package:process_runner/process_runner.dart';

import 'src/commands/command_runner.dart';
import 'src/environment.dart';
import 'src/logger.dart';
import 'src/phone_home.dart';

void main(List<String> args) async {
  if (phoneHome(args)) {
    return;
  }

  final bool verbose = args.contains('--verbose') || args.contains('-v');
  final bool help = args.contains('help') || args.contains('--help') || args.contains('-h');

  // Find the engine repo.
  final Engine engine;
  try {
    engine = Engine.findWithin(p.dirname(io.Platform.script.toFilePath()));
  } catch (e) {
    io.stderr.writeln(e);
    io.exitCode = 1;
    return;
  }

  // Find and parse the engine build configs.
  final io.Directory buildConfigsDir = io.Directory(
    p.join(engine.flutterDir.path, 'ci', 'builders'),
  );
  final BuildConfigLoader loader = BuildConfigLoader(buildConfigsDir: buildConfigsDir);

  // Treat it as an error if no build configs were found. The caller likely
  // expected to find some.
  final Map<String, BuilderConfig> configs = loader.configs;
  if (configs.isEmpty) {
    io.stderr.writeln('Error: No build configs found under ${buildConfigsDir.path}');
    io.exitCode = 1;
    return;
  }
  if (loader.errors.isNotEmpty) {
    loader.errors.forEach(io.stderr.writeln);
    io.exitCode = 1;
  }

  final Logger logger;
  if (verbose) {
    logger = Logger(level: Logger.infoLevel);
  } else {
    logger = Logger();
  }
  final Environment environment = Environment(
    abi: ffi.Abi.current(),
    engine: engine,
    platform: const LocalPlatform(),
    processRunner: ProcessRunner(),
    logger: logger,
    verbose: verbose,
  );

  // Use the Engine and BuildConfig collection to build the CommandRunner.
  final ToolCommandRunner runner = ToolCommandRunner(
    environment: environment,
    configs: configs,
    help: help,
  );

  try {
    io.exitCode = await runner.run(args);
  } on FatalError catch (e, st) {
    environment.logger.error(
      'FatalError caught in main. Please file a bug\n'
      'error: $e\n'
      'stack: $st',
    );
    io.exitCode = 1;
  }
  return;
}
