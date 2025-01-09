// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart';
import 'package:process_runner/process_runner.dart';
import 'package:test/test.dart';

void main() {
  final Engine engine;
  try {
    engine = Engine.findWithin();
  } catch (e) {
    io.stderr.writeln(e);
    io.exitCode = 1;
    return;
  }

  test('The entry points under bin/ work', () async {
    const Platform platform = LocalPlatform();
    final ProcessRunner runner = ProcessRunner();
    final String exe = platform.isWindows ? '.bat' : '';
    final String entrypointPath = path.join(engine.flutterDir.path, 'bin', 'et$exe');
    final ProcessRunnerResult processResult = await runner.runProcess(<String>[
      entrypointPath,
      'help',
    ], failOk: true);
    if (processResult.exitCode != 0) {
      io.stdout.writeln(processResult.stdout);
      io.stderr.writeln(processResult.stderr);
    }
    expect(processResult.exitCode, equals(0));
  });
}
