// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';
import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as p;

import 'src/header_file.dart';

/// Checks C++ header files for header guards.
@immutable
final class HeaderGuardCheck {
  /// Creates a new header guard checker.
  HeaderGuardCheck({
    required this.source,
    required this.exclude,
    this.include = const <String>[],
    this.fix = false,
    StringSink? stdOut,
    StringSink? stdErr,
  }) : _stdOut = stdOut ?? io.stdout,
       _stdErr = stdErr ?? io.stderr;

  /// Parses the command line arguments and creates a new header guard checker.
  factory HeaderGuardCheck.fromCommandLine(List<String> arguments) {
    final ArgResults argResults = _parser.parse(arguments);
    return HeaderGuardCheck(
      source: Engine.fromSrcPath(argResults['root'] as String),
      include: argResults['include'] as List<String>,
      exclude: argResults['exclude'] as List<String>,
      fix: argResults['fix'] as bool,
    );
  }

  /// Engine source root.
  final Engine source;

  /// Whether to automatically fix most header guards.
  final bool fix;

  /// Path directories to include in the check.
  final List<String> include;

  /// Path directories to exclude from the check.
  final List<String> exclude;

  /// Stdout.
  final StringSink _stdOut;

  /// Stderr.
  final StringSink _stdErr;

  /// Runs the header guard check.
  Future<int> run() async {
    final List<HeaderFile> badFiles = _checkFiles(_findIncludedHeaderFiles()).toList();

    if (badFiles.isNotEmpty) {
      _stdOut.writeln('The following ${badFiles.length} files have invalid header guards:');
      for (final HeaderFile headerFile in badFiles) {
        _stdOut.writeln('  ${headerFile.path}');
      }

      // If we're fixing, fix the files.
      if (fix) {
        for (final HeaderFile headerFile in badFiles) {
          headerFile.fix(engineRoot: source.flutterDir.path);
        }

        _stdOut.writeln('Fixed ${badFiles.length} files.');
        return 0;
      }

      return 1;
    }

    return 0;
  }

  Iterable<io.File> _findIncludedHeaderFiles() sync* {
    final Queue<String> queue = Queue<String>();
    final Set<String> yielded = <String>{};
    if (include.isEmpty) {
      queue.add(source.flutterDir.path);
    } else {
      queue.addAll(include);
    }
    while (queue.isNotEmpty) {
      final String path = queue.removeFirst();
      if (path.endsWith('.h')) {
        if (!_isExcluded(path) && yielded.add(path)) {
          yield io.File(path);
        }
      } else if (io.FileSystemEntity.isDirectorySync(path)) {
        if (_isExcluded(path)) {
          continue;
        }
        final io.Directory directory = io.Directory(path);
        for (final io.FileSystemEntity entity in directory.listSync(recursive: true)) {
          if (entity is io.File && entity.path.endsWith('.h')) {
            queue.add(entity.path);
          }
        }
      } else {
        // Neither a header file nor a directory that might contain header files.
      }
    }
  }

  bool _isExcluded(String path) {
    final rootPath = source.flutterDir.parent.parent.parent.path;
    for (final String excludePath in exclude) {
      final String relativePath = p.relative(excludePath, from: rootPath);
      if (p.isWithin(relativePath, path) || p.equals(relativePath, path)) {
        return true;
      }
    }
    return false;
  }

  Iterable<HeaderFile> _checkFiles(Iterable<io.File> headers) sync* {
    for (final io.File header in headers) {
      final HeaderFile headerFile = HeaderFile.parse(header.path);
      if (headerFile.pragmaOnce != null) {
        _stdErr.writeln(headerFile.pragmaOnce!.message('Unexpected #pragma once'));
        yield headerFile;
        continue;
      }

      if (headerFile.guard == null) {
        _stdErr.writeln('Missing header guard in ${headerFile.path}');
        yield headerFile;
        continue;
      }

      final String expectedGuard = headerFile.computeExpectedName(
        engineRoot: source.flutterDir.path,
      );
      if (headerFile.guard!.ifndefValue != expectedGuard) {
        _stdErr.writeln(headerFile.guard!.ifndefSpan!.message('Expected #ifndef $expectedGuard'));
        yield headerFile;
        continue;
      }
      if (headerFile.guard!.defineValue != expectedGuard) {
        _stdErr.writeln(headerFile.guard!.defineSpan!.message('Expected #define $expectedGuard'));
        yield headerFile;
        continue;
      }
      if (headerFile.guard!.endifValue != expectedGuard) {
        _stdErr.writeln(headerFile.guard!.endifSpan!.message('Expected #endif // $expectedGuard'));
        yield headerFile;
        continue;
      }
    }
  }
}

final Engine? _engine = Engine.tryFindWithin(p.dirname(p.fromUri(io.Platform.script)));

final ArgParser _parser =
    ArgParser()
      ..addFlag('fix', help: 'Automatically fixes most header guards.')
      ..addOption(
        'root',
        abbr: 'r',
        help: 'Path to the engine source root.',
        valueHelp: 'path/to/engine/src',
        defaultsTo: _engine?.srcDir.path,
      )
      ..addMultiOption(
        'include',
        abbr: 'i',
        help: 'Paths to include in the check.',
        valueHelp: 'path/to/dir/or/file (relative to the engine root)',
        defaultsTo: <String>[],
      )
      ..addMultiOption(
        'exclude',
        abbr: 'e',
        help: 'Paths to exclude from the check.',
        valueHelp: 'path/to/dir/or/file (relative to the engine root)',
        defaultsTo:
            _engine != null
                ? <String>[
                  'engine/src/build',
                  'engine/src/flutter/build',
                  'engine/src/flutter/buildtools',
                  'engine/src/flutter/impeller/compiler/code_gen_template.h',
                  'engine/src/flutter/prebuilts',
                  'engine/src/flutter/third_party',
                ]
                : null,
      );
