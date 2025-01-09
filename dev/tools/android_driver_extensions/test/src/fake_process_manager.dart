// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'dart:io';
import 'dart:typed_data';

import 'package:process/process.dart';

/// A stub of [ProcessManager] that allows mocking the [run] method only.
final class FakeProcessManager extends ProcessManager {
  /// Creates a new [FakeProcessManager] with the given delegate callback.
  FakeProcessManager(this._runDelegate);
  final Future<ProcessResult> Function(String executable, List<String> arguments) _runDelegate;

  @override
  Never noSuchMethod(_) => throw UnimplementedError();

  @override
  Future<ProcessResult> run(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding? stdoutEncoding = systemEncoding,
    Encoding? stderrEncoding = systemEncoding,
  }) {
    final List<String> strings = command.map((Object e) => '$e').toList();
    final [String executable, ...List<String> arguments] = strings;
    return _runDelegate(executable, arguments);
  }

  /// Returns a [ProcessResult] with the given [stdout].
  static ProcessResult ok([String stdout = '']) {
    return ProcessResult(0, 0, stdout, '');
  }

  /// Returns a [ProcessResult] with the given [stderr].
  static ProcessResult error([String stderr = '']) {
    return ProcessResult(0, 1, '', stderr);
  }

  /// Returns a [ProcessResult] with the given [stdout] binary data.
  static ProcessResult okBinary(List<int> stdout) {
    return ProcessResult(0, 0, Uint8List.fromList(stdout), '');
  }
}
