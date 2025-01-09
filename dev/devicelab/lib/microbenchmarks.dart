// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

/// Reads through the print commands from [process] waiting for the magic phase
/// that contains microbenchmarks results as defined in
/// `dev/benchmarks/microbenchmarks/lib/common.dart`.
///
/// If you are using this outside of microbenchmarks, ensure you print a single
/// line with `╡ ••• Done ••• ╞` to signal the end of collection.
Future<Map<String, double>> readJsonResults(Process process) {
  // IMPORTANT: keep these values in sync with dev/benchmarks/microbenchmarks/lib/common.dart
  const String jsonStart = '================ RESULTS ================';
  const String jsonEnd = '================ FORMATTED ==============';
  const String jsonPrefix = ':::JSON:::';
  const String testComplete = '╡ ••• Done ••• ╞';

  bool jsonStarted = false;
  final StringBuffer jsonBuf = StringBuffer();
  final Completer<Map<String, double>> completer = Completer<Map<String, double>>();

  final StreamSubscription<String> stderrSub = process.stderr
      .transform<String>(const Utf8Decoder())
      .transform<String>(const LineSplitter())
      .listen((String line) {
        stderr.writeln('[STDERR] $line');
      });

  final List<String> collectedJson = <String>[];

  bool processWasKilledIntentionally = false;
  final StreamSubscription<String> stdoutSub = process.stdout
      .transform<String>(const Utf8Decoder())
      .transform<String>(const LineSplitter())
      .listen((String line) async {
        print('[STDOUT] $line');

        if (line.contains(jsonStart)) {
          jsonStarted = true;
          return;
        }

        if (line.contains(testComplete)) {
          processWasKilledIntentionally = true;
          // Sending a SIGINT/SIGTERM to the process here isn't reliable because [process] is
          // the shell (flutter is a shell script) and doesn't pass the signal on.
          // Sending a `q` is an instruction to quit using the console runner.
          // See https://github.com/flutter/flutter/issues/19208
          process.stdin.write('q');
          await process.stdin.flush();
          // Give the process a couple of seconds to exit and run shutdown hooks
          // before sending kill signal.
          // TODO(fujino): https://github.com/flutter/flutter/issues/134566
          await Future<void>.delayed(const Duration(seconds: 2));
          // Also send a kill signal in case the `q` above didn't work.
          process.kill(ProcessSignal.sigint);
          try {
            final Map<String, double> results = Map<String, double>.from(<String, dynamic>{
              for (final String data in collectedJson) ...json.decode(data) as Map<String, dynamic>,
            });
            completer.complete(results);
          } catch (ex) {
            completer.completeError(
              'Decoding JSON failed ($ex). JSON strings where: $collectedJson',
            );
          }
          return;
        }

        if (jsonStarted && line.contains(jsonEnd)) {
          collectedJson.add(jsonBuf.toString().trim());
          jsonBuf.clear();
          jsonStarted = false;
        }

        if (jsonStarted && line.contains(jsonPrefix)) {
          jsonBuf.writeln(line.substring(line.indexOf(jsonPrefix) + jsonPrefix.length));
        }
      });

  process.exitCode.then<void>((int code) async {
    await Future.wait<void>(<Future<void>>[stdoutSub.cancel(), stderrSub.cancel()]);
    if (!processWasKilledIntentionally && code != 0) {
      completer.completeError('flutter run failed: exit code=$code');
    }
  });

  return completer.future;
}
