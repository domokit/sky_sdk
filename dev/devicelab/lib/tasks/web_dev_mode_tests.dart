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

final Directory _editedFlutterGalleryDir = dir(
  path.join(Directory.systemTemp.path, 'edited_flutter_gallery'),
);
final Directory flutterGalleryDir = dir(
  path.join(flutterDirectory.path, 'dev/integration_tests/flutter_gallery'),
);

const String kInitialStartupTime = 'InitialStartupTime';
const String kFirstRestartTime = 'FistRestartTime';
const String kFirstRecompileTime = 'FirstRecompileTime';
const String kSecondStartupTime = 'SecondStartupTime';
const String kSecondRestartTime = 'SecondRestartTime';

abstract class WebDevice {
  static const String chrome = 'chrome';
  static const String webServer = 'web-server';
}

TaskFunction createWebDevModeTest(String webDevice, bool enableIncrementalCompiler) {
  return () async {
    final List<String> options = <String>[
      '--hot',
      '-d',
      webDevice,
      '--verbose',
      '--resident',
      '--target=lib/main.dart',
    ];
    int hotRestartCount = 0;
    final String expectedMessage =
        webDevice == WebDevice.webServer ? 'Recompile complete' : 'Reloaded application';
    final Map<String, int> measurements = <String, int>{};
    await inDirectory<void>(flutterDirectory, () async {
      rmTree(_editedFlutterGalleryDir);
      mkdirs(_editedFlutterGalleryDir);
      recursiveCopy(flutterGalleryDir, _editedFlutterGalleryDir);
      await inDirectory<void>(_editedFlutterGalleryDir, () async {
        {
          await flutter('packages', options: <String>['get']);
          final Process process = await startFlutter('run', options: options);

          final Completer<void> stdoutDone = Completer<void>();
          final Completer<void> stderrDone = Completer<void>();
          final Stopwatch sw = Stopwatch()..start();
          bool restarted = false;
          process.stdout
              .transform<String>(utf8.decoder)
              .transform<String>(const LineSplitter())
              .listen(
                (String line) {
                  // non-dwds builds do not know when the browser is loaded so keep trying
                  // until this succeeds.
                  if (line.contains('Ignoring terminal input')) {
                    Future<void>.delayed(const Duration(seconds: 1)).then((void _) {
                      process.stdin.write(restarted ? 'q' : 'r');
                    });
                    return;
                  }
                  if (line.contains('To hot restart')) {
                    // measure clean start-up time.
                    sw.stop();
                    measurements[kInitialStartupTime] = sw.elapsedMilliseconds;
                    sw
                      ..reset()
                      ..start();
                    process.stdin.write('r');
                    return;
                  }
                  if (line.contains(expectedMessage)) {
                    if (hotRestartCount == 0) {
                      measurements[kFirstRestartTime] = sw.elapsedMilliseconds;
                      // Update the file and reload again.
                      final File appDartSource = file(
                        path.join(_editedFlutterGalleryDir.path, 'lib/gallery/app.dart'),
                      );
                      appDartSource.writeAsStringSync(
                        appDartSource.readAsStringSync().replaceFirst(
                          "'Flutter Gallery'",
                          "'Updated Flutter Gallery'",
                        ),
                      );
                      sw
                        ..reset()
                        ..start();
                      process.stdin.writeln('r');
                      ++hotRestartCount;
                    } else {
                      restarted = true;
                      measurements[kFirstRecompileTime] = sw.elapsedMilliseconds;
                      // Quit after second hot restart.
                      process.stdin.writeln('q');
                    }
                  }
                  print('stdout: $line');
                },
                onDone: () {
                  stdoutDone.complete();
                },
              );
          process.stderr
              .transform<String>(utf8.decoder)
              .transform<String>(const LineSplitter())
              .listen(
                (String line) {
                  print('stderr: $line');
                },
                onDone: () {
                  stderrDone.complete();
                },
              );

          await Future.wait<void>(<Future<void>>[stdoutDone.future, stderrDone.future]);
          await process.exitCode;
        }

        // Start `flutter run` again to make sure it loads from the previous
        // state. dev compilers loads up from previously compiled JavaScript.
        {
          final Stopwatch sw = Stopwatch()..start();
          final Process process = await startFlutter('run', options: options);
          final Completer<void> stdoutDone = Completer<void>();
          final Completer<void> stderrDone = Completer<void>();
          bool restarted = false;
          process.stdout
              .transform<String>(utf8.decoder)
              .transform<String>(const LineSplitter())
              .listen(
                (String line) {
                  // non-dwds builds do not know when the browser is loaded so keep trying
                  // until this succeeds.
                  if (line.contains('Ignoring terminal input')) {
                    Future<void>.delayed(const Duration(seconds: 1)).then((void _) {
                      process.stdin.write(restarted ? 'q' : 'r');
                    });
                    return;
                  }
                  if (line.contains('To hot restart')) {
                    measurements[kSecondStartupTime] = sw.elapsedMilliseconds;
                    sw
                      ..reset()
                      ..start();
                    process.stdin.write('r');
                    return;
                  }
                  if (line.contains(expectedMessage)) {
                    restarted = true;
                    measurements[kSecondRestartTime] = sw.elapsedMilliseconds;
                    process.stdin.writeln('q');
                  }
                  print('stdout: $line');
                },
                onDone: () {
                  stdoutDone.complete();
                },
              );
          process.stderr
              .transform<String>(utf8.decoder)
              .transform<String>(const LineSplitter())
              .listen(
                (String line) {
                  print('stderr: $line');
                },
                onDone: () {
                  stderrDone.complete();
                },
              );

          await Future.wait<void>(<Future<void>>[stdoutDone.future, stderrDone.future]);
          await process.exitCode;
        }
      });
    });
    if (hotRestartCount != 1) {
      return TaskResult.failure(null);
    }
    return TaskResult.success(
      measurements,
      benchmarkScoreKeys: <String>[
        kInitialStartupTime,
        kFirstRestartTime,
        kFirstRecompileTime,
        kSecondStartupTime,
        kSecondRestartTime,
      ],
    );
  };
}
