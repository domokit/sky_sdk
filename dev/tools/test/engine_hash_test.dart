// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('posix')
library;

import 'dart:io' as io;

import 'package:path/path.dart' as p;
import 'package:test/test.dart';

/// Tests that `/dev/tools/bin/engine_hash.sh` _appears_ to work.
void main() {
  late final io.File engineHashSh;

  setUpAll(() {
    engineHashSh = io.File(p.join(p.current, 'bin', 'engine_hash.sh'));
    if (!engineHashSh.existsSync()) {
      fail('No engine_hash.sh at "${p.absolute(engineHashSh.path)}".');
    }
  });

  late io.Directory tmpFlutterRoot;

  setUp(() {
    tmpFlutterRoot = io.Directory.systemTemp.createTempSync('engine_hash_test.');

    // Create engine_hash.sh at the same component it would be in the real root.
    io.Directory(p.join(tmpFlutterRoot.path, 'dev', 'tools', 'bin')).createSync(recursive: true);
    engineHashSh.copySync(p.join(tmpFlutterRoot.path, 'dev', 'tools', 'bin', 'engine_hash.sh'));

    // Create FLUTTER_ROOT/DEPS.
    io.File(p.join(tmpFlutterRoot.path, 'DEPS')).createSync();
  });

  tearDown(() {
    tmpFlutterRoot.deleteSync(recursive: true);
  });

  test('omission of FLUTTER_ROOT/DEPS falls back to engine.version', () {
    io.File(p.join(tmpFlutterRoot.path, 'bin', 'internal', 'engine.version'))
      ..createSync(recursive: true)
      ..writeAsStringSync('12345');
    io.File(p.join(tmpFlutterRoot.path, 'DEPS')).deleteSync();

    final io.ProcessResult result = io.Process.runSync(
      p.join(tmpFlutterRoot.path, 'dev', 'tools', 'bin', 'engine_hash.sh'),
      <String>[],
    );
    expect(result.exitCode, 0, reason: result.stderr.toString());
    expect(result.stdout, '12345\n');
  });

  test('uses git -C merge-base HEAD origin/master', () {
    final io.ProcessResult result = io.Process.runSync(
      p.join(tmpFlutterRoot.path, 'dev', 'tools', 'bin', 'engine_hash.sh'),
      <String>[],
      environment: <String, String>{'GIT': p.join(p.current, 'test', 'mock_git.sh')},
    );
    expect(result.exitCode, 0, reason: result.stderr.toString());
    expect(
      result.stdout,
      stringContainsInOrder(<String>[
        'Mock Git: -C',
        'engine_hash_test',
        // This needs to be origin/master if the google3 script is running from a fresh checkout.
        'merge-base HEAD origin/master',
      ]),
    );
  });
}
