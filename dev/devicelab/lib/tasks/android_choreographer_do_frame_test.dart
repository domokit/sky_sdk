// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import '../framework/framework.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

const List<String> kSentinelStr = <String>[
  '==== sentinel #1 ====',
  '==== sentinel #2 ====',
  '==== sentinel #3 ====',
];

/// Tests that Choreographer#doFrame finishes during application startup.
/// This test fails if the application hangs during this period.
/// https://ui.perfetto.dev/#!/?s=da6628c3a92456ae8fa3f345d0186e781da77e90fc8a64d073e9fee11d1e65
/// Regression test for https://github.com/flutter/flutter/issues/98973
TaskFunction androidChoreographerDoFrameTest({Map<String, String>? environment}) {
  final Directory tempDir = Directory.systemTemp.createTempSync(
    'flutter_devicelab_android_surface_recreation.',
  );
  return () async {
    try {
      section('Create app');
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>['--platforms', 'android', 'app'],
          environment: environment,
        );
      });

      final File mainDart = File(path.join(tempDir.absolute.path, 'app', 'lib', 'main.dart'));
      if (!mainDart.existsSync()) {
        return TaskResult.failure('${mainDart.path} does not exist');
      }

      section('Patch lib/main.dart');
      await mainDart.writeAsString('''
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  print('${kSentinelStr[0]}');
  await SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);

  print('${kSentinelStr[1]}');
  // If the Android UI thread is blocked, then this Future won't resolve.
  await SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);

  print('${kSentinelStr[2]}');
  runApp(
    Container(
      decoration: BoxDecoration(
        color: const Color(0xff7c94b6),
      ),
    ),
  );
}
''', flush: true);

      Future<TaskResult> runTestFor(String mode) async {
        int nextCompleterIdx = 0;
        final Map<String, Completer<void>> sentinelCompleters = <String, Completer<void>>{};
        for (final String sentinel in kSentinelStr) {
          sentinelCompleters[sentinel] = Completer<void>();
        }

        section('Flutter run (mode: $mode)');
        late Process run;
        await inDirectory(path.join(tempDir.path, 'app'), () async {
          run = await startFlutter('run', options: <String>['--$mode', '--verbose']);
        });

        int currSentinelIdx = 0;
        final StreamSubscription<void> stdout = run.stdout
            .transform<String>(utf8.decoder)
            .transform<String>(const LineSplitter())
            .listen((String line) {
              if (currSentinelIdx < sentinelCompleters.keys.length &&
                  line.contains(sentinelCompleters.keys.elementAt(currSentinelIdx))) {
                sentinelCompleters.values.elementAt(currSentinelIdx).complete();
                currSentinelIdx++;
                print('stdout(MATCHED): $line');
              } else {
                print('stdout: $line');
              }
            });

        final StreamSubscription<void> stderr = run.stderr
            .transform<String>(utf8.decoder)
            .transform<String>(const LineSplitter())
            .listen((String line) {
              print('stderr: $line');
            });

        final Completer<void> exitCompleter = Completer<void>();

        unawaited(
          run.exitCode.then((int exitCode) {
            exitCompleter.complete();
          }),
        );

        section('Wait for sentinels (mode: $mode)');
        for (final Completer<void> completer in sentinelCompleters.values) {
          if (nextCompleterIdx == 0) {
            // Don't time out because we don't know how long it would take to get the first log.
            await Future.any<dynamic>(<Future<dynamic>>[completer.future, exitCompleter.future]);
          } else {
            try {
              // Time out since this should not take 1s after the first log was received.
              await Future.any<dynamic>(<Future<dynamic>>[
                completer.future.timeout(const Duration(seconds: 1)),
                exitCompleter.future,
              ]);
            } on TimeoutException {
              break;
            }
          }
          if (exitCompleter.isCompleted) {
            // The process exited.
            break;
          }
          nextCompleterIdx++;
        }

        section('Quit app (mode: $mode)');
        run.stdin.write('q');
        await exitCompleter.future;

        section('Stop listening to stdout and stderr (mode: $mode)');
        await stdout.cancel();
        await stderr.cancel();
        run.kill();

        if (nextCompleterIdx == sentinelCompleters.values.length) {
          return TaskResult.success(null);
        }
        final String nextSentinel = sentinelCompleters.keys.elementAt(nextCompleterIdx);
        return TaskResult.failure('Expected sentinel `$nextSentinel` in mode $mode');
      }

      final TaskResult debugResult = await runTestFor('debug');
      if (debugResult.failed) {
        return debugResult;
      }

      final TaskResult profileResult = await runTestFor('profile');
      if (profileResult.failed) {
        return profileResult;
      }

      final TaskResult releaseResult = await runTestFor('release');
      if (releaseResult.failed) {
        return releaseResult;
      }

      return TaskResult.success(null);
    } finally {
      rmTree(tempDir);
    }
  };
}
