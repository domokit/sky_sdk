// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart';

void main() {
  // Find a path to `dir_contents_diff.dart` from the working directory.
  final String pkgPath = p.join(Engine.findWithin().flutterDir.path, 'tools', 'dir_contents_diff');
  final String binPath = p.join(pkgPath, 'bin', 'dir_contents_diff.dart');

  // As a sanity check, ensure that the file exists.
  if (!io.File(binPath).existsSync()) {
    io.stderr.writeln('Unable to find $binPath');
    io.exitCode = 1;
    return;
  }

  // Runs `../bin/dir_contents_diff.dart` with the given arguments.
  (int, String) runSync(String goldenPath, String dirPath) {
    final io.ProcessResult result = io.Process.runSync(io.Platform.resolvedExecutable, <String>[
      binPath,
      goldenPath,
      dirPath,
    ]);
    return (result.exitCode, result.stdout ?? result.stderr);
  }

  test('lists files and diffs successfully', () {
    final String goldenPath = p.join(pkgPath, 'test', 'file_ok.txt');
    final String dirPath = p.join(pkgPath, 'test', 'fixtures');
    final (int exitCode, String output) = runSync(goldenPath, dirPath);
    if (exitCode != 0) {
      io.stderr.writeln('Expected exit code 0, got $exitCode');
      io.stderr.writeln(output);
    }
    expect(exitCode, 0);
  });

  test('lists files and diffs successfully, even with an EOF newline', () {
    final String goldenPath = p.join(pkgPath, 'test', 'file_ok_eof_newline.txt');
    final String dirPath = p.join(pkgPath, 'test', 'fixtures');
    final (int exitCode, String output) = runSync(goldenPath, dirPath);
    if (exitCode != 0) {
      io.stderr.writeln('Expected exit code 0, got $exitCode');
      io.stderr.writeln(output);
    }
    expect(exitCode, 0);
  });

  test('diff fails when an expected file is missing', () {
    final String goldenPath = p.join(pkgPath, 'test', 'file_bad_missing.txt');
    final String dirPath = p.join(pkgPath, 'test', 'fixtures');
    final (int exitCode, String output) = runSync(goldenPath, dirPath);
    if (exitCode == 0) {
      io.stderr.writeln('Expected non-zero exit code, got $exitCode');
      io.stderr.writeln(output);
    }
    expect(exitCode, 1);
    expect(output, contains('+a.txt'));
  });

  test('diff fails when an unexpected file is present', () {
    final String goldenPath = p.join(pkgPath, 'test', 'file_bad_unexpected.txt');
    final String dirPath = p.join(pkgPath, 'test', 'fixtures');
    final (int exitCode, String output) = runSync(goldenPath, dirPath);
    if (exitCode == 0) {
      io.stderr.writeln('Expected non-zero exit code, got $exitCode');
      io.stderr.writeln(output);
    }
    expect(exitCode, 1);
    expect(output, contains('-c.txt'));
  });
}
