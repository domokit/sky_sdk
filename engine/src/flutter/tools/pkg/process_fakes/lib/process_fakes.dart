// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io;

import 'package:process/process.dart';

/// A fake implementation of [ProcessManager] that allows control for testing.
final class FakeProcessManager implements ProcessManager {
  /// Creates a fake process manager delegates to [onRun] and [onStart].
  ///
  /// If either is not provided, it throws an [UnsupportedError] when called.
  FakeProcessManager({
    io.ProcessResult Function(List<String> command) onRun = unhandledRun,
    io.Process Function(List<String> command) onStart = unhandledStart,
    bool Function(Object?, {String? workingDirectory}) canRun = unhandledCanRun,
  }) : _onRun = onRun,
       _onStart = onStart,
       _canRun = canRun;

  /// A default implementation of [onRun] that throws an [UnsupportedError].
  static io.ProcessResult unhandledRun(List<String> command) {
    throw UnsupportedError('Unhandled run: ${command.join(' ')}');
  }

  /// A default implementation of [onStart] that throws an [UnsupportedError].
  static io.Process unhandledStart(List<String> command) {
    throw UnsupportedError('Unhandled start: ${command.join(' ')}');
  }

  /// A default implementation of [canRun] that returns `true`.
  static bool unhandledCanRun(Object? executable, {String? workingDirectory}) {
    return true;
  }

  final io.ProcessResult Function(List<String> command) _onRun;
  final io.Process Function(List<String> command) _onStart;
  final bool Function(Object?, {String? workingDirectory}) _canRun;

  @override
  bool canRun(Object? executable, {String? workingDirectory}) {
    return _canRun(executable, workingDirectory: workingDirectory);
  }

  @override
  bool killPid(int pid, [io.ProcessSignal signal = io.ProcessSignal.sigterm]) => true;

  @override
  Future<io.ProcessResult> run(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding stdoutEncoding = io.systemEncoding,
    Encoding stderrEncoding = io.systemEncoding,
  }) async {
    return runSync(
      command,
      workingDirectory: workingDirectory,
      environment: environment,
      includeParentEnvironment: includeParentEnvironment,
      runInShell: runInShell,
      stdoutEncoding: stdoutEncoding,
      stderrEncoding: stderrEncoding,
    );
  }

  @override
  io.ProcessResult runSync(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding stdoutEncoding = io.systemEncoding,
    Encoding stderrEncoding = io.systemEncoding,
  }) {
    return _onRun(command.map((Object o) => '$o').toList());
  }

  @override
  Future<io.Process> start(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    io.ProcessStartMode mode = io.ProcessStartMode.normal,
  }) async {
    return _onStart(command.map((Object o) => '$o').toList());
  }
}

/// An incomplete fake of [io.Process] that allows control for testing.
final class FakeProcess implements io.Process {
  /// Creates a fake process that returns the given [exitCode] and out/err.
  FakeProcess({int exitCode = 0, String stdout = '', String stderr = ''})
    : _exitCode = exitCode,
      _stdout = stdout,
      _stderr = stderr,
      _stdin = io.IOSink(StreamController<List<int>>.broadcast().sink);

  final int _exitCode;
  final String _stdout;
  final String _stderr;
  final io.IOSink _stdin;

  @override
  Future<int> get exitCode async => _exitCode;

  @override
  bool kill([io.ProcessSignal signal = io.ProcessSignal.sigterm]) => true;

  @override
  int get pid => 0;

  @override
  Stream<List<int>> get stderr {
    return Stream<List<int>>.fromIterable(<List<int>>[io.systemEncoding.encoder.convert(_stderr)]);
  }

  @override
  io.IOSink get stdin => _stdin;

  @override
  Stream<List<int>> get stdout {
    return Stream<List<int>>.fromIterable(<List<int>>[io.systemEncoding.encoder.convert(_stdout)]);
  }
}
