// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io;

import 'package:path/path.dart' as p;

import '../logger.dart';
import 'command.dart';
import 'flags.dart';

/// The 'format' command.
///
/// The format command implementation below works by spawning another Dart VM to
/// run the program under ci/bin/format.dart.
///
// TODO(team-engine): Part of https://github.com/flutter/flutter/issues/132807.
// Instead, format.dart should be moved under the engine_tool package and
// invoked by a function call. The file ci/bin/format.dart should be split up so
// that each of its `FormatCheckers` is in a separate file under src/formatters,
// and they should be unit-tested.
final class FormatCommand extends CommandBase {
  // ignore: public_member_api_docs
  FormatCommand({required super.environment, super.usageLineLength}) {
    argParser
      ..addFlag(
        allFlag,
        abbr: 'a',
        help:
            'By default only dirty files are checked. This flag causes all '
            'files to be checked. (Slow)',
        negatable: false,
      )
      ..addFlag(
        dryRunFlag,
        abbr: 'd',
        help:
            'Do not fix formatting in-place. Instead, print file diffs to '
            'the logs.',
        negatable: false,
      )
      ..addFlag(
        quietFlag,
        abbr: 'q',
        help: 'Silences all log messages except for errors and warnings',
        negatable: false,
      );
  }

  @override
  String get name => 'format';

  @override
  String get description => 'Formats files using standard formatters and styles.';

  @override
  Future<int> run() async {
    final bool all = argResults![allFlag]! as bool;
    final bool dryRun = argResults![dryRunFlag]! as bool;
    final bool quiet = argResults![quietFlag]! as bool;
    final bool verbose = globalResults![verboseFlag] as bool;
    final String formatPath = p.join(
      environment.engine.flutterDir.path,
      'ci',
      'bin',
      'format.dart',
    );

    final io.Process process = await environment.processRunner.processManager.start(<String>[
      environment.platform.resolvedExecutable,
      formatPath,
      if (all) '--all-files',
      if (!dryRun) '--fix',
      if (verbose) '--verbose',
    ], workingDirectory: environment.engine.flutterDir.path);
    final Completer<void> stdoutComplete = Completer<void>();
    final Completer<void> stderrComplete = Completer<void>();

    final _FormatStreamer streamer = _FormatStreamer(environment.logger, dryRun, quiet);
    process.stdout
        .transform<String>(const Utf8Decoder())
        .transform(const LineSplitter())
        .listen(streamer.nextStdout, onDone: () async => stdoutComplete.complete());
    process.stderr
        .transform<String>(const Utf8Decoder())
        .transform(const LineSplitter())
        .listen(streamer.nextStderr, onDone: () async => stderrComplete.complete());

    await Future.wait<void>(<Future<void>>[stdoutComplete.future, stderrComplete.future]);
    final int exitCode = await process.exitCode;

    return exitCode;
  }
}

class _FormatStreamer {
  _FormatStreamer(this.logger, this.dryRun, this.quiet);

  final Logger logger;
  final bool dryRun;
  final bool quiet;

  bool inADiff = false;
  bool inProgress = false;

  void nextStdout(String line) {
    if (quiet) {
      return;
    }
    final String l = line.trim();
    if (l == 'To fix, run `et format --all` or:') {
      inADiff = true;
    }
    if (l.isNotEmpty && (!inADiff || dryRun)) {
      if (_isProgressLine(l)) {
        inProgress = true;
        logger.clearLine();
        logger.status('$l\r', newline: false);
      } else {
        if (inProgress) {
          logger.clearLine();
          inProgress = false;
        }
        logger.status(l);
      }
    }
    if (l == 'DONE') {
      inADiff = false;
    }
  }

  void nextStderr(String line) {
    final String l = line.trim();
    if (l.isEmpty) {
      return;
    }
    logger.error(l);
  }

  bool _isProgressLine(String l) {
    final List<String> words = l.split(',');
    return words.isNotEmpty && words[0].endsWith('% done');
  }
}
