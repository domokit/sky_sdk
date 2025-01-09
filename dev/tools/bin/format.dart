// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:args/args.dart';
import 'package:path/path.dart' as path;
import 'package:process_runner/process_runner.dart';

Future<int> main(List<String> arguments) async {
  final ArgParser parser = ArgParser();
  parser.addFlag('help', help: 'Print help.', abbr: 'h');
  parser.addFlag(
    'fix',
    abbr: 'f',
    help: 'Instead of just checking for formatting errors, fix them in place.',
  );
  parser.addFlag(
    'all-files',
    abbr: 'a',
    help:
        'Instead of just checking for formatting errors in changed files, '
        'check for them in all files.',
  );

  late final ArgResults options;
  try {
    options = parser.parse(arguments);
  } on FormatException catch (e) {
    stderr.writeln('ERROR: $e');
    _usage(parser, exitCode: 0);
  }

  if (options['help'] as bool) {
    _usage(parser, exitCode: 0);
  }

  final File script = File.fromUri(Platform.script).absolute;
  final Directory flutterRoot = script.parent.parent.parent.parent;

  final bool result =
      (await DartFormatChecker(
        flutterRoot: flutterRoot,
        allFiles: options['all-files'] as bool,
      ).check(fix: options['fix'] as bool)) ==
      0;

  exit(result ? 0 : 1);
}

void _usage(ArgParser parser, {int exitCode = 1}) {
  stderr.writeln('format.dart [--help] [--fix] [--all-files]');
  stderr.writeln(parser.usage);
  exit(exitCode);
}

class DartFormatChecker {
  DartFormatChecker({required this.flutterRoot, required this.allFiles})
    : processRunner = ProcessRunner(defaultWorkingDirectory: flutterRoot);

  final Directory flutterRoot;
  final bool allFiles;
  final ProcessRunner processRunner;

  Future<int> check({required bool fix}) async {
    final String baseGitRef = await _getDiffBaseRevision();
    final List<String> filesToCheck = await _getFileList(
      types: <String>['*.dart'],
      allFiles: allFiles,
      baseGitRef: baseGitRef,
    );
    return _checkFormat(filesToCheck: filesToCheck, fix: fix);
  }

  Future<String> _getDiffBaseRevision() async {
    String upstream = 'upstream';
    final String upstreamUrl = await _runGit(
      <String>['remote', 'get-url', upstream],
      processRunner,
      failOk: true,
    );
    if (upstreamUrl.isEmpty) {
      upstream = 'origin';
    }
    await _runGit(<String>['fetch', upstream, 'main'], processRunner);
    String result = '';
    try {
      // This is the preferred command to use, but developer checkouts often do
      // not have a clear fork point, so we fall back to just the regular
      // merge-base in that case.
      result = await _runGit(<String>[
        'merge-base',
        '--fork-point',
        'FETCH_HEAD',
        'HEAD',
      ], processRunner);
    } on ProcessRunnerException {
      result = await _runGit(<String>['merge-base', 'FETCH_HEAD', 'HEAD'], processRunner);
    }
    return result.trim();
  }

  Future<String> _runGit(
    List<String> args,
    ProcessRunner processRunner, {
    bool failOk = false,
  }) async {
    final ProcessRunnerResult result = await processRunner.runProcess(<String>[
      'git',
      ...args,
    ], failOk: failOk);
    return result.stdout;
  }

  Future<List<String>> _getFileList({
    required List<String> types,
    required bool allFiles,
    required String baseGitRef,
  }) async {
    String output;
    if (allFiles) {
      output = await _runGit(<String>['ls-files', '--', ...types], processRunner);
    } else {
      output = await _runGit(<String>[
        'diff',
        '-U0',
        '--no-color',
        '--diff-filter=d',
        '--name-only',
        baseGitRef,
        '--',
        ...types,
      ], processRunner);
    }
    return output
        .split('\n')
        .where((String line) => line.isNotEmpty && !line.startsWith('engine'))
        .toList();
  }

  Future<int> _checkFormat({required List<String> filesToCheck, required bool fix}) async {
    final List<String> cmd = <String>[
      path.join(flutterRoot.path, 'bin', 'dart'),
      'format',
      '--set-exit-if-changed',
      '--show=none',
      if (!fix) '--output=show',
      if (fix) '--output=write',
    ];
    final List<WorkerJob> jobs = <WorkerJob>[];
    for (final String file in filesToCheck) {
      jobs.add(WorkerJob(<String>[...cmd, file]));
    }
    final ProcessPool dartFmt = ProcessPool(
      processRunner: processRunner,
      printReport: _namedReport('dart format'),
    );

    Iterable<WorkerJob> incorrect;
    final List<WorkerJob> errorJobs = <WorkerJob>[];
    if (!fix) {
      final Stream<WorkerJob> completedJobs = dartFmt.startWorkers(jobs);
      final List<WorkerJob> diffJobs = <WorkerJob>[];
      await for (final WorkerJob completedJob in completedJobs) {
        if (completedJob.result.exitCode != 0 && completedJob.result.exitCode != 1) {
          // The formatter had a problem formatting the file.
          errorJobs.add(completedJob);
        } else if (completedJob.result.exitCode == 1) {
          diffJobs.add(
            WorkerJob(<String>[
              'git',
              'diff',
              '--no-index',
              '--no-color',
              '--ignore-cr-at-eol',
              '--',
              completedJob.command.last,
              '-',
            ], stdinRaw: _codeUnitsAsStream(completedJob.result.stdoutRaw)),
          );
        }
      }
      final ProcessPool diffPool = ProcessPool(
        processRunner: processRunner,
        printReport: _namedReport('diff'),
      );
      final List<WorkerJob> completedDiffs = await diffPool.runToCompletion(diffJobs);
      incorrect = completedDiffs.where((WorkerJob job) => job.result.exitCode != 0);
    } else {
      final List<WorkerJob> completedJobs = await dartFmt.runToCompletion(jobs);
      final List<WorkerJob> incorrectJobs = incorrect = <WorkerJob>[];
      for (final WorkerJob job in completedJobs) {
        if (job.result.exitCode != 0 && job.result.exitCode != 1) {
          // The formatter had a problem formatting the file.
          errorJobs.add(job);
        } else if (job.result.exitCode == 1) {
          incorrectJobs.add(job);
        }
      }
    }

    _clearOutput();

    if (incorrect.isNotEmpty) {
      final bool plural = incorrect.length > 1;
      if (fix) {
        stdout.writeln(
          'Fixing ${incorrect.length} dart file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
      } else {
        stderr.writeln(
          'Found ${incorrect.length} Dart file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
        final String fileList = incorrect
            .map((WorkerJob job) => job.command[job.command.length - 2])
            .join(' ');
        stdout.writeln();
        stdout.writeln('To fix, run `dart format $fileList` or:');
        stdout.writeln();
        stdout.writeln('git apply <<DONE');
        for (final WorkerJob job in incorrect) {
          stdout.write(
            job.result.stdout
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
                .replaceFirst(
                  RegExp('\\+Formatted \\d+ files? \\(\\d+ changed\\) in \\d+.\\d+ seconds.\n'),
                  '',
                ),
          );
        }
        stdout.writeln('DONE');
        stdout.writeln();
      }
      _printErrorJobs(errorJobs);
    } else if (errorJobs.isNotEmpty) {
      _printErrorJobs(errorJobs);
    } else {
      stdout.writeln('All dart files formatted correctly.');
    }
    return fix ? errorJobs.length : (incorrect.length + errorJobs.length);
  }

  void _printErrorJobs(List<WorkerJob> errorJobs) {
    if (errorJobs.isNotEmpty) {
      final bool plural = errorJobs.length > 1;
      stderr.writeln(
        'The formatter failed to run on ${errorJobs.length} Dart file${plural ? 's' : ''}.',
      );
      stdout.writeln();
      for (final WorkerJob job in errorJobs) {
        stdout.writeln('--> ${job.command.last} produced the following error:');
        stdout.write(job.result.stderr);
        stdout.writeln();
      }
    }
  }
}

ProcessPoolProgressReporter _namedReport(String name) {
  return (int total, int completed, int inProgress, int pending, int failed) {
    final String percent = total == 0 ? '100' : ((100 * completed) ~/ total).toString().padLeft(3);
    final String completedStr = completed.toString().padLeft(3);
    final String totalStr = total.toString().padRight(3);
    final String inProgressStr = inProgress.toString().padLeft(2);
    final String pendingStr = pending.toString().padLeft(3);
    final String failedStr = failed.toString().padLeft(3);

    stdout.write(
      '$name Jobs: $percent% done, '
      '$completedStr/$totalStr completed, '
      '$inProgressStr in progress, '
      '$pendingStr pending, '
      '$failedStr failed.${' ' * 20}\r',
    );
  };
}

void _clearOutput() {
  stdout.write('\r${' ' * 100}\r');
}

Stream<List<int>> _codeUnitsAsStream(List<int>? input) async* {
  if (input != null) {
    yield input;
  }
}
