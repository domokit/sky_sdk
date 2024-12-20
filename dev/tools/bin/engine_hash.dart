// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ---------------------------------- NOTE ----------------------------------
//
// We must keep the logic in this file consistent with the logic in the
// `engine_hash.sh` script in the same directory to ensure that Flutter
// continues to work across all platforms!
//
// --------------------------------------------------------------------------

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:args/args.dart';
import 'package:crypto/crypto.dart';

enum GitRevisionStrategy { mergeBase, head }

final RegExp _hashRegex = RegExp(r'^([a-fA-F0-9]+)');

final ArgParser parser =
    ArgParser()
      ..addOption(
        'strategy',
        abbr: 's',
        allowed: <String>['head', 'mergeBase'],
        defaultsTo: 'head',
        allowedHelp: <String, String>{
          'head': 'hash from git HEAD',
          'mergeBase': 'hash from the merge-base of HEAD and upstream/master',
        },
      )
      ..addFlag('help', abbr: 'h', negatable: false);

Never printHelp({String? error}) {
  final Stdout out = error != null ? stderr : stdout;
  if (error != null) {
    out.writeln(error);
    out.writeln();
  }
  out.writeln('''
Calculate the hash signature for the Flutter Engine
${parser.usage}
''');
  exit(error != null ? 1 : 0);
}

Future<int> main(List<String> args) async {
  final ArgResults arguments;
  try {
    arguments = parser.parse(args);
  } catch (e) {
    printHelp(error: '$e');
  }

  if (arguments.wasParsed('help')) {
    printHelp();
  }

  final String result;
  try {
    result = await engineHash(
      (List<String> command) =>
          Process.run(command.first, command.sublist(1), stdoutEncoding: utf8),
      revisionStrategy: GitRevisionStrategy.values.byName(arguments.option('strategy')!),
    );
  } catch (e) {
    stderr.writeln('Error calculating engine hash: $e');
    return 1;
  }

  stdout.writeln(result);

  return 0;
}

/// Returns the hash signature for the engine source code.
Future<String> engineHash(
  Future<ProcessResult> Function(List<String> command) runProcess, {
  GitRevisionStrategy revisionStrategy = GitRevisionStrategy.mergeBase,
}) async {
  // First figure out the hash we're working with
  final String base;
  switch (revisionStrategy) {
    case GitRevisionStrategy.head:
      base = 'HEAD';
    case GitRevisionStrategy.mergeBase:
      final ProcessResult processResult = await runProcess(<String>[
        'git',
        'merge-base',
        'upstream/master',
        'HEAD',
      ]);

      if (processResult.exitCode != 0) {
        throw '''
Unable to find merge-base hash of the repository:
${processResult.stderr}''';
      }

      final Match? baseHash = _hashRegex.matchAsPrefix(processResult.stdout as String);
      if (baseHash?.groupCount != 1) {
        throw '''
Unable to parse merge-base hash of the repository
${processResult.stdout}''';
      }
      base = baseHash![1]!;
  }

  // List the tree (not the working tree) recursively for the merge-base.
  // This is important for future filtering of files, but also do not include
  // the developer's changes / in flight PRs.
  // The presence `engine` and `DEPS` are signals that you live in a monorepo world.
  final ProcessResult processResult = await runProcess(<String>[
    'git',
    'ls-tree',
    '-r',
    base,
    'engine',
    'DEPS',
  ]);

  if (processResult.exitCode != 0) {
    throw '''
Unable to list tree
${processResult.stderr}''';
  }

  // Ensure stable line endings so our hash calculation is stable
  final String lsTree = processResult.stdout as String;
  if (lsTree.trim().isEmpty) {
    throw 'Not in a monorepo';
  }

  final Iterable<String> treeLines = LineSplitter.split(processResult.stdout as String);

  // We could call `git hash-object --stdin` which would just take the input, calculate the size,
  // and then sha1sum it like: `blob $size\0$string'. However, that can have different line endings.
  // Instead this is equivalent to:
  //     git ls-tree -r $(git merge-base upstream/main HEAD) | <only newlines> | sha1sum
  final StreamController<Digest> output = StreamController<Digest>();
  final ByteConversionSink sink = sha1.startChunkedConversion(output);
  for (final String line in treeLines) {
    sink.add(utf8.encode(line));
    sink.add(<int>[0x0a]);
  }
  sink.close();
  final Digest digest = await output.stream.first;

  return '$digest';
}
