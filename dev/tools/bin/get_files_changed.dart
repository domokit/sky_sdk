// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:args/args.dart';

final ArgParser _argParser =
    ArgParser()
      ..addOption(
        'since',
        help: 'What previous SHA to compare the current git state to.',
        defaultsTo: 'HEAD^',
      )
      ..addOption(
        'output',
        help: 'What format to output in.',
        defaultsTo: io.stdout.hasTerminal ? 'text' : 'json',
        allowed: <String>['text', 'json'],
      );

void main(List<String> args) async {
  final ArgResults argResults = _argParser.parse(args);

  // Get a list of files changed between this commit and the base SHA.
  final List<String> filesChanged;
  {
    final List<String> args = <String>[
      'diff',
      '--name-only',
      '--full-index',
      argResults.option('since')!,
    ];
    final io.ProcessResult git = await io.Process.run('git', args);
    if (git.exitCode != 0) {
      io.stderr.writeln('$args failed (exit code: ${git.exitCode}):');
      io.stderr.writeln(git.stdout);
      io.stderr.writeln(git.stderr);
      io.exitCode = 1;
      return;
    }

    final String stdout = git.stdout as String;
    filesChanged = const LineSplitter().convert(stdout);
  }

  // Output to stdout.
  final _Output output = _Output.values.byName(argResults.option('output')!);
  io.stdout.writeln(switch (output) {
    _Output.text => filesChanged.join('\n'),
    _Output.json => jsonEncode(filesChanged),
  });
}

enum _Output { text, json }
