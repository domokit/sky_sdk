// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show Directory;

import 'package:path/path.dart' as p;

import '../dart_utils.dart';

import '../proc_utils.dart';
import '../worker_pool.dart';
import 'command.dart';
import 'flags.dart';

/// The different kind of linters we support.
enum Linter {
  /// Dart linter
  dart,

  /// Java linter
  java,

  /// C/C++ linter
  c,

  /// Python linter
  python,
}

class _LinterDescription {
  _LinterDescription(this.linter, this.cwd, this.command);

  final Linter linter;
  final Directory cwd;
  final List<String> command;
}

/// The root 'lint' command.
final class LintCommand extends CommandBase {
  /// Constructs the 'lint' command.
  LintCommand({required super.environment, super.usageLineLength}) {
    final String engineFlutterPath = environment.engine.flutterDir.path;
    _linters[Linter.dart] = _LinterDescription(Linter.dart, environment.engine.flutterDir, <String>[
      p.join(engineFlutterPath, 'ci', 'analyze.sh'),
      findDartBinDirectory(environment),
    ]);
    _linters[Linter.java] = _LinterDescription(Linter.java, environment.engine.flutterDir, <String>[
      findDartBinary(environment),
      p.join(engineFlutterPath, 'tools', 'android_lint', 'bin', 'main.dart'),
    ]);
    _linters[Linter.c] = _LinterDescription(Linter.c, environment.engine.flutterDir, <String>[
      p.join(engineFlutterPath, 'ci', 'clang_tidy.sh'),
    ]);
    _linters[Linter.python] = _LinterDescription(
      Linter.python,
      environment.engine.flutterDir,
      <String>[p.join(engineFlutterPath, 'ci', 'pylint.sh')],
    );
    argParser.addFlag(quietFlag, abbr: 'q', help: 'Prints minimal output');
  }

  final Map<Linter, _LinterDescription> _linters = <Linter, _LinterDescription>{};

  @override
  String get name => 'lint';

  @override
  String get description => 'Lint the engine repository.';

  @override
  Future<int> run() async {
    // TODO(loic-sharma): Relax this restriction.
    if (environment.platform.isWindows) {
      environment.logger.fatal('lint command is not supported on Windows (for now).');
    }
    final WorkerPool wp = WorkerPool(environment, ProcessTaskProgressReporter(environment));

    final Set<ProcessTask> tasks = <ProcessTask>{};
    for (final MapEntry<Linter, _LinterDescription> entry in _linters.entries) {
      tasks.add(ProcessTask(entry.key.name, environment, entry.value.cwd, entry.value.command));
    }
    final bool r = await wp.run(tasks);

    final bool quiet = argResults![quietFlag] as bool;
    if (!quiet) {
      environment.logger.status('\nDumping failure logs\n');
      for (final ProcessTask pt in tasks) {
        final ProcessArtifacts pa = pt.processArtifacts;
        if (pa.exitCode == 0) {
          continue;
        }
        environment.logger.status('Linter ${pt.name} found issues:');
        environment.logger.status('${pa.stdout}\n');
        environment.logger.status('${pa.stderr}\n');
      }
    }
    return r ? 0 : 1;
  }
}
