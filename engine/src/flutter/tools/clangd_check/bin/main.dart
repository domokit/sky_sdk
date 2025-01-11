// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as p;

void main(List<String> args) {
  final Engine? engine = Engine.tryFindWithin();
  final ArgParser parser =
      ArgParser()
        ..addFlag('help', abbr: 'h', help: 'Print this usage information.', negatable: false)
        ..addOption(
          'clangd',
          help: 'Path to clangd. Defaults to deriving the path from compile_commands.json.',
        )
        ..addOption(
          'compile-commands-dir',
          help: 'Path to a directory containing compile_commands.json.',
          defaultsTo: engine?.latestOutput()?.compileCommandsJson.parent.path,
        );
  final ArgResults results = parser.parse(args);
  if (results['help'] as bool) {
    io.stdout.writeln(parser.usage);
    return;
  }

  final String? compileCommandsDir = results['compile-commands-dir'] as String?;
  if (compileCommandsDir == null) {
    io.stderr.writeln('Must provide a path to compile_commands.json');
    io.exitCode = 1;
    return;
  }
  final io.File compileCommandsFile = io.File(p.join(compileCommandsDir, 'compile_commands.json'));
  if (!compileCommandsFile.existsSync()) {
    io.stderr.writeln('No compile_commands.json found in $compileCommandsDir');
    io.exitCode = 1;
    return;
  }

  final List<Object?> compileCommands =
      json.decode(compileCommandsFile.readAsStringSync()) as List<Object?>;
  if (compileCommands.isEmpty) {
    io.stderr.writeln('Unexpected: compile_commands.json is empty');
    io.exitCode = 1;
    return;
  }

  String? clangd = results['clangd'] as String?;
  // To improve determinism, check the first clangd item that matches the asset fixture file.
  Map<String, Object?>? selectedEntry;
  for (final Object? entry in compileCommands) {
    if (entry is Map<String, Object?>) {
      final String? file = entry['file'] as String?;
      if (file != null && file.endsWith('_fl__fl_assets_fixtures.cc')) {
        selectedEntry = entry;
        break;
      }
    } else {
      io.stderr.writeln('Unexpected: compile_commands.json has an unexpected format');
      io.stderr.writeln('First entry: ${const JsonEncoder.withIndent('  ').convert(entry)}');
      io.exitCode = 1;
      return;
    }
  }
  if (selectedEntry == null) {
    io.stderr.writeln('No compile_commands.json entry found for _fl__fl_assets_fixtures.cc');
    io.exitCode = 1;
    return;
  }

  final String checkFile;
  if (selectedEntry case {
    'command': final String command,
    'directory': final String directory,
    'file': final String file,
  }) {
    // Given a path like ../../flutter/foo.cc, we want to check foo.cc.
    checkFile = p.join(directory, file);
    // On CI, the command path is different from the local path.
    // Find the engine root and derive the clangd path from there.
    if (clangd == null) {
      // Strip the command to find the path to the engine root.
      // i.e. "command": "/path/to/engine/src/... arg1 arg2 ..."
      //
      // This now looks like "../../flutter/buildtools/{platform}/{...}"
      final String commandPath = p.dirname(command.split(' ').first);

      // Find the canonical path to the command (i.e. resolve "../" and ".")
      //
      // This now looks like "/path/to/engine/src/flutter/buildtools/{platform}/{...}"
      final String path = p.canonicalize(p.join(compileCommandsDir, commandPath));

      // Extract which platform we're building for (e.g. linux-x64, mac-arm64, mac-x64).
      final String platform = RegExp(r'buildtools/([^/]+)/').firstMatch(path)!.group(1)!;

      // Find the engine root and derive the clangd path from there.
      final Engine compileCommandsEngineRoot = Engine.findWithin(path);
      clangd = p.join(
        // engine/src/flutter
        compileCommandsEngineRoot.flutterDir.path,
        // buildtools
        'buildtools',
        // {platform}
        platform,
        // clangd
        'clang',
        'bin',
        'clangd',
      );
    }
  } else {
    io.stderr.writeln('Unexpected: compile_commands.json has an unexpected format');
    io.stderr.writeln('First entry: ${const JsonEncoder.withIndent('  ').convert(selectedEntry)}');
    io.exitCode = 1;
    return;
  }

  final Engine engineRoot = Engine.findWithin(p.canonicalize(compileCommandsDir));
  final io.File clangdConfig = io.File(p.join(engineRoot.flutterDir.path, '.clangd'));
  try {
    // Write a .clangd file to the engine root directory.
    //
    // See:
    // - https://clangd.llvm.org/config#compileflags
    // - https://github.com/clangd/clangd/issues/662
    clangdConfig.writeAsStringSync(
      'CompileFlags:\n'
      '  Add: -Wno-unknown-warning-option\n'
      '  Remove: [-m*, -f*]\n',
    );

    // Run clangd.
    final io.ProcessResult result = io.Process.runSync(clangd, <String>[
      '--compile-commands-dir',
      compileCommandsDir,
      '--check=$checkFile',
    ]);
    io.stdout.write(result.stdout);
    io.stderr.write(result.stderr);
    if ((result.stderr as String).contains(
      'Path specified by --compile-commands-dir does not exist',
    )) {
      io.stdout.writeln('clangd_check failed: --compile-commands-dir does not exist');
      io.exitCode = 1;
    } else if ((result.stderr as String).contains('Failed to resolve path')) {
      io.stdout.writeln('clangd_check failed: --check file does not exist');
      io.exitCode = 1;
    } else {
      io.exitCode = result.exitCode;
    }
  } on io.ProcessException catch (e) {
    io.stderr.writeln('Failed to run clangd: $e');
    io.stderr.writeln(const JsonEncoder.withIndent('  ').convert(selectedEntry));
    io.exitCode = 1;
  } finally {
    // Remove the copied .clangd file from the engine root directory.
    clangdConfig.deleteSync();
  }
}
