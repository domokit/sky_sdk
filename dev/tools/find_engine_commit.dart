// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

void _validate(List<String> args) {
  bool errors = false;
  if (!File('bin/internal/engine.version').existsSync()) {
    errors = true;
    print('This program must be run from the root of your flutter repository.');
  }
  if (!File('../engine/src/flutter/DEPS').existsSync()) {
    errors = true;
    print(
      'This program assumes the engine directory is a sibling to the flutter repository directory.',
    );
  }
  if (args.length != 1) {
    errors = true;
    print('This program takes the engine revision as a single argument.');
  }
  if (errors) {
    exit(-1);
  }
}

const String engineRepo = '../engine/src/flutter';

Future<void> main(List<String> args) async {
  _validate(args);
  await _fetchUpstream();
  await _fetchUpstream(engineRepo);
  String? flutterRevision;
  await for (final FlutterEngineRevision revision in _logEngineVersions()) {
    if (!await containsRevision(args[0], revision.engineRevision)) {
      if (flutterRevision == null) {
        print('Revision not found.');
        exit(-1);
      }
      print('earliest revision: $flutterRevision');
      print('Tags that contain this engine revision:');
      print(await _tagsForRevision(flutterRevision));
      exit(0);
    }
    flutterRevision = revision.flutterRevision;
  }
}

Future<void> _fetchUpstream([String workingDirectory = '.']) async {
  print(
    'Fetching remotes for "$workingDirectory" - you may be prompted for SSH credentials by git.',
  );
  final ProcessResult fetchResult = await Process.run('git', <String>[
    'fetch',
    '--all',
  ], workingDirectory: workingDirectory);
  if (fetchResult.exitCode != 0) {
    throw Exception('Failed to fetch upstream in repository $workingDirectory');
  }
}

Future<String> _tagsForRevision(String flutterRevision) async {
  final ProcessResult tagResult = await Process.run('git', <String>[
    'tag',
    '--contains',
    flutterRevision,
  ]);
  return tagResult.stdout as String;
}

Future<bool> containsRevision(String ancestorRevision, String revision) async {
  final ProcessResult result = await Process.run('git', <String>[
    'merge-base',
    '--is-ancestor',
    ancestorRevision,
    revision,
  ], workingDirectory: engineRepo);
  return result.exitCode == 0;
}

Stream<FlutterEngineRevision> _logEngineVersions() async* {
  final ProcessResult result = await Process.run('git', <String>[
    'log',
    '--oneline',
    '-p',
    '--',
    'bin/internal/engine.version',
  ]);
  if (result.exitCode != 0) {
    print(result.stderr);
    throw Exception('Failed to log bin/internal/engine.version');
  }

  final List<String> lines = (result.stdout as String).split('\n');
  int index = 0;
  while (index < lines.length - 1) {
    final String flutterRevision = lines[index].split(' ').first;
    index += 1;
    while (!lines[index].startsWith('+') || lines[index].startsWith('+++')) {
      index += 1;
    }
    if (index >= lines.length) {
      break;
    }
    final String engineRevision = lines[index].substring(1);
    yield FlutterEngineRevision(flutterRevision, engineRevision);
    index += lines[index + 1].startsWith(r'\ ') ? 2 : 1;
  }
}

class FlutterEngineRevision {
  const FlutterEngineRevision(this.flutterRevision, this.engineRevision);

  final String flutterRevision;
  final String engineRevision;

  @override
  String toString() => '$flutterRevision: $engineRevision';
}
