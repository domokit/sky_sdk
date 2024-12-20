// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
@TestOn('vm')
library;

import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:header_guard_check/header_guard_check.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart';

Future<int> main() async {
  void withTestRepository(String path, void Function(io.Directory) fn) {
    // Create a temporary directory and delete it when we're done.
    final io.Directory tempDir = io.Directory.systemTemp.createTempSync('header_guard_check_test');
    final io.Directory repoDir = io.Directory(p.join(tempDir.path, path));
    repoDir.createSync(recursive: true);
    try {
      fn(repoDir);
    } finally {
      tempDir.deleteSync(recursive: true);
    }
  }

  group('HeaderGuardCheck', () {
    test('by default checks all files', () {
      withTestRepository('engine/src', (io.Directory repoDir) {
        final io.Directory flutterDir = io.Directory(p.join(repoDir.path, 'flutter'));
        flutterDir.createSync(recursive: true);
        final io.File file1 = io.File(p.join(flutterDir.path, 'foo.h'));
        file1.createSync(recursive: true);
        final io.File file2 = io.File(p.join(flutterDir.path, 'bar.h'));
        file2.createSync(recursive: true);
        final io.File file3 = io.File(p.join(flutterDir.path, 'baz.h'));
        file3.createSync(recursive: true);

        final StringBuffer stdOut = StringBuffer();
        final StringBuffer stdErr = StringBuffer();
        final HeaderGuardCheck check = HeaderGuardCheck(
          source: Engine.fromSrcPath(repoDir.path),
          exclude: const <String>[],
          stdOut: stdOut,
          stdErr: stdErr,
        );

        expect(check.run(), completion(1));
        expect(stdOut.toString(), contains('foo.h'));
        expect(stdOut.toString(), contains('bar.h'));
        expect(stdOut.toString(), contains('baz.h'));
      });
    });

    test('if --include is provided, checks specific files', () {
      withTestRepository('engine/src', (io.Directory repoDir) {
        final io.Directory flutterDir = io.Directory(p.join(repoDir.path, 'flutter'));
        flutterDir.createSync(recursive: true);
        final io.File file1 = io.File(p.join(flutterDir.path, 'foo.h'));
        file1.createSync(recursive: true);
        final io.File file2 = io.File(p.join(flutterDir.path, 'bar.h'));
        file2.createSync(recursive: true);
        final io.File file3 = io.File(p.join(flutterDir.path, 'baz.h'));
        file3.createSync(recursive: true);

        final StringBuffer stdOut = StringBuffer();
        final StringBuffer stdErr = StringBuffer();
        final HeaderGuardCheck check = HeaderGuardCheck(
          source: Engine.fromSrcPath(repoDir.path),
          include: <String>[file1.path, file3.path],
          exclude: const <String>[],
          stdOut: stdOut,
          stdErr: stdErr,
        );

        expect(check.run(), completion(1));
        expect(stdOut.toString(), contains('foo.h'));
        expect(stdOut.toString(), contains('baz.h'));

        // TODO(matanlurey): https://github.com/flutter/flutter/issues/133569).
        if (stdOut.toString().contains('bar.h')) {
          // There is no not(contains(...)) matcher.
          fail('bar.h should not be checked. Output: $stdOut');
        }
      });
    });

    test('if --include is provided, checks specific directories', () {
      withTestRepository('engine/src', (io.Directory repoDir) {
        final io.Directory flutterDir = io.Directory(p.join(repoDir.path, 'flutter'));
        flutterDir.createSync(recursive: true);

        // Create a sub-directory called "impeller".
        final io.Directory impellerDir = io.Directory(p.join(flutterDir.path, 'impeller'));
        impellerDir.createSync(recursive: true);

        // Create one file in both the root and in impeller.
        final io.File file1 = io.File(p.join(flutterDir.path, 'foo.h'));
        file1.createSync(recursive: true);
        final io.File file2 = io.File(p.join(impellerDir.path, 'bar.h'));
        file2.createSync(recursive: true);

        final StringBuffer stdOut = StringBuffer();
        final StringBuffer stdErr = StringBuffer();
        final HeaderGuardCheck check = HeaderGuardCheck(
          source: Engine.fromSrcPath(repoDir.path),
          include: <String>[impellerDir.path],
          exclude: const <String>[],
          stdOut: stdOut,
          stdErr: stdErr,
        );

        expect(check.run(), completion(1));
        expect(stdOut.toString(), contains('bar.h'));

        // TODO(matanlurey): https://github.com/flutter/flutter/issues/133569).
        if (stdOut.toString().contains('foo.h')) {
          // There is no not(contains(...)) matcher.
          fail('foo.h should not be checked. Output: $stdOut');
        }
      });
    });
  });

  return 0;
}
