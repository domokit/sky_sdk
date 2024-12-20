// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show LineSplitter, utf8;
import 'dart:io' as io;

import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;
import 'package:process_runner/process_runner.dart';

import 'options.dart';

/// The url prefix for issues that must be attached to the directive in files
/// that disables linting.
const String issueUrlPrefix = 'https://github.com/flutter/flutter/issues';

/// Lint actions to apply to a file.
enum LintAction {
  /// Run the linter over the file.
  lint,

  /// Ignore files under third_party/.
  skipThirdParty,

  /// Ignore due to a well-formed FLUTTER_NOLINT comment.
  skipNoLint,

  /// Fail due to a malformed FLUTTER_NOLINT comment.
  failMalformedNoLint,

  /// Ignore because the file doesn't exist locally.
  skipMissing,
}

/// A compilation command and methods to generate the lint command and job for
/// it.
class Command {
  /// Generate a [Command] from a [Map].
  Command.fromMap(Map<String, dynamic> map)
    : directory = io.Directory(map['directory'] as String).absolute,
      command = map['command'] as String {
    filePath = path.normalize(path.join(directory.path, map['file'] as String));
  }

  /// The working directory of the command.
  final io.Directory directory;

  /// The compilation command.
  final String command;

  /// The file on which the command operates.
  late final String filePath;

  static final RegExp _pathRegex = RegExp(r'\S*clang/bin/clang(\+\+)?');
  static final RegExp _argRegex = RegExp(r'-MF\s+\S+\s+');

  // Filter out any extra commands that were appended to the compile command.
  static final RegExp _extraCommandRegex = RegExp(r'&&.*$');

  String? _tidyArgs;

  /// The command line arguments of the command.
  String get tidyArgs {
    return _tidyArgs ??=
        (() {
          String result = command;
          result = result.replaceAll(r'\s+', ' ');
          // Remove everything that comes before the compiler command.
          result = result.split(' ').skipWhile((String s) => !_pathRegex.hasMatch(s)).join(' ');
          result = result.replaceAll(_pathRegex, '');
          result = result.replaceAll(_argRegex, '');
          result = result.replaceAll(_extraCommandRegex, '');
          return result;
        })();
  }

  String? _tidyPath;

  /// The command but with clang-tidy instead of clang.
  String get tidyPath {
    return _tidyPath ??=
        _pathRegex
            .stringMatch(command)
            ?.replaceAll('clang/bin/clang', 'clang/bin/clang-tidy')
            .replaceAll('clang-tidy++', 'clang-tidy') ??
        '';
  }

  /// Whether this command operates on any of the files in `queries`.
  bool containsAny(List<io.File> queries) {
    return queries.indexWhere((io.File query) => path.equals(query.path, filePath)) != -1;
  }

  static final RegExp _nolintRegex = RegExp(
    r'//\s*FLUTTER_NOLINT(: https://github.com/flutter/flutter/issues/\d+)?',
  );

  /// The type of lint that is appropriate for this command.
  late final Future<LintAction> lintAction = getLintAction(filePath);

  /// Determine the lint action for the file at `path`.
  @visibleForTesting
  static Future<LintAction> getLintAction(String filePath) async {
    if (path.split(filePath).contains('third_party')) {
      return LintAction.skipThirdParty;
    }

    final io.File file = io.File(filePath);
    if (!file.existsSync()) {
      return LintAction.skipMissing;
    }
    final Stream<String> lines = file
        .openRead()
        .transform(utf8.decoder)
        .transform(const LineSplitter());
    return lintActionFromContents(lines);
  }

  /// Determine the lint action for the file with contents `lines`.
  @visibleForTesting
  static Future<LintAction> lintActionFromContents(Stream<String> lines) async {
    // Check for FlUTTER_NOLINT at top of file.
    await for (final String line in lines) {
      final RegExpMatch? match = _nolintRegex.firstMatch(line);
      if (match != null) {
        return match.group(1) != null ? LintAction.skipNoLint : LintAction.failMalformedNoLint;
      } else if (line.isNotEmpty && line[0] != '\n' && line[0] != '/') {
        // Quick out once we find a line that isn't empty or a comment.  The
        // FLUTTER_NOLINT must show up before the first real code.
        return LintAction.lint;
      }
    }
    return LintAction.lint;
  }

  /// The job for the process runner for the lint needed for this command.
  WorkerJob createLintJob(Options options) {
    final List<String> args = <String>[
      filePath,
      '--warnings-as-errors=${options.warningsAsErrors ?? '*'}',
      if (options.checks != null) options.checks!,
      if (options.fix) ...<String>['--fix', '--format-style=file'],
      if (options.enableCheckProfile) '--enable-check-profile',
      '--',
    ];
    args.addAll(tidyArgs.split(' '));
    final String clangTidyPath = options.clangTidyPath?.path ?? tidyPath;
    return WorkerJob(
      <String>[clangTidyPath, ...args],
      workingDirectory: directory,
      name: 'clang-tidy on $filePath',
      printOutput: options.verbose,
    );
  }
}
