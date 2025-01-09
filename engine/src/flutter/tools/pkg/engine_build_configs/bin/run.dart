// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' as ffi;
import 'dart:io' as io;

import 'package:engine_build_configs/engine_build_configs.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as p;
import 'package:platform/platform.dart';
import 'package:process_runner/process_runner.dart';

// This is an example of how to use the APIs of this library to parse and
// execute the build configurations json files under ci/builders.
//
// Usage:
// $ dart bin/run.dart [build config name] [build name]
// For example:
// $ dart bin/run.dart mac_unopt host_debug_unopt
//
// The build config names are the names of the json files under ci/builders
// The build names are the "name" fields of the maps in the list of "builds".

void main(List<String> args) async {
  final String? configName;
  final String? buildName;
  if (args.length >= 2) {
    configName = args[0];
    buildName = args[1];
  } else {
    io.stderr.writeln(r'''
Usage:
$ dart bin/run.dart [build config name] [build name]

For example:

$ dart bin/run.dart mac_unopt host_debug_unopt

The build config names are the names of the json files under ci/builders.
The build names are the "name" fields of the maps in the list of "builds".
''');
    io.exitCode = 1;
    return;
  }

  // Find the engine repo.
  final Engine engine;
  try {
    engine = Engine.findWithin();
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

  // Check the parsed build configs for validity.
  final BuilderConfig? targetConfig = configs[configName];
  if (targetConfig == null) {
    io.stderr.writeln('Build config "$configName" not found.');
    io.exitCode = 1;
    return;
  }
  final List<String> buildConfigErrors = targetConfig.check(configName);
  if (buildConfigErrors.isNotEmpty) {
    io.stderr.writeln('Errors in "$configName":');
    for (final String error in buildConfigErrors) {
      io.stderr.writeln('    $error');
    }
    io.exitCode = 1;
    return;
  }

  Build? targetBuild;
  for (int i = 0; i < targetConfig.builds.length; i++) {
    final Build build = targetConfig.builds[i];
    if (build.name == buildName) {
      targetBuild = build;
    }
  }
  if (targetBuild == null) {
    io.stderr.writeln('Target build not found. No build called $buildName in $configName');
    io.exitCode = 1;
    return;
  }

  // If RBE config files aren't in the tree, then disable RBE.
  final String rbeConfigPath = p.join(engine.srcDir.path, 'flutter', 'build', 'rbe');
  final List<String> extraGnArgs = <String>[
    if (!io.Directory(rbeConfigPath).existsSync()) '--no-rbe',
  ];

  final BuildRunner buildRunner = BuildRunner(
    platform: const LocalPlatform(),
    processRunner: ProcessRunner(),
    abi: ffi.Abi.current(),
    engineSrcDir: engine.srcDir,
    build: targetBuild,
    extraGnArgs: extraGnArgs,
    runGenerators: false,
    runTests: false,
  );
  void handler(RunnerEvent event) {
    switch (event) {
      case RunnerStart():
        io.stdout.writeln('$event: ${event.command.join(' ')}');
      case RunnerProgress(done: true):
        io.stdout.writeln(event);
      case RunnerProgress(done: false):
        {
          final int width = io.stdout.terminalColumns;
          final String percent = '${event.percent.toStringAsFixed(1)}%';
          final String fraction = '(${event.completed}/${event.total})';
          final String prefix = '[${event.name}] $percent $fraction ';
          final int remainingSpace = width - prefix.length;
          final String what;
          if (remainingSpace >= event.what.length) {
            what = event.what;
          } else {
            what = event.what.substring(event.what.length - remainingSpace + 1);
          }
          final String spaces = ' ' * width;
          io.stdout.write('$spaces\r'); // Erase the old line.
          io.stdout.write('$prefix$what\r'); // Print the new line.
        }
      default:
        io.stdout.writeln(event);
    }
  }

  final bool buildResult = await buildRunner.run(handler);
  io.exitCode = buildResult ? 0 : 1;
}
