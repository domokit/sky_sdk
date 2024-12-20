// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:process/process.dart';

(Future<int> exitCode, Stream<String> output) getProcessStreams(Process process) {
  final Completer<void> stdoutCompleter = Completer<void>();
  final Completer<void> stderrCompleter = Completer<void>();
  final StreamController<String> outputController = StreamController<String>();

  final StreamSubscription<void> stdoutSub = process.stdout
      .transform(utf8.decoder)
      .transform<String>(const LineSplitter())
      .listen(outputController.add, onDone: stdoutCompleter.complete);

  // From looking at historic logs, it seems that the stderr output is rare.
  // Instead of prefacing every line with [stdout] unnecessarily, we'll just
  // use [stderr] to indicate that it's from the stderr stream.
  //
  // For example, a historic log which has 0 occurrences of stderr:
  // https://gist.github.com/matanlurey/84cf9c903ef6d507dcb63d4c303ca45f
  final StreamSubscription<void> stderrSub = process.stderr
      .transform(utf8.decoder)
      .transform<String>(const LineSplitter())
      .map((String line) => '[stderr] $line')
      .listen(outputController.add, onDone: stderrCompleter.complete);

  final Future<int> exitCode = process.exitCode.then<int>((int code) async {
    await (stdoutSub.cancel(), stderrSub.cancel()).wait;
    outputController.close();
    return code;
  });

  return (exitCode, outputController.stream);
}

/// Pipes the [process] streams and writes them to [out] sink.
///
/// If [out] is null, then the current [Process.stdout] is used as the sink.
Future<int> pipeProcessStreams(Process process, {StringSink? out}) async {
  out ??= stdout;

  final (Future<int> exitCode, Stream<String> output) = getProcessStreams(process);
  output.listen(out.writeln);
  return exitCode;
}

extension RunAndForward on ProcessManager {
  /// Runs [cmd], and forwards the stdout and stderr pipes to the current process stdout pipe.
  Future<int> runAndForward(List<String> cmd) async {
    return pipeProcessStreams(await start(cmd), out: stdout);
  }

  /// Runs [cmd], and captures the stdout and stderr pipes.
  Future<(int, StringBuffer)> runAndCapture(List<String> cmd) async {
    final StringBuffer buffer = StringBuffer();
    final int exitCode = await pipeProcessStreams(await start(cmd), out: buffer);
    return (exitCode, buffer);
  }
}
