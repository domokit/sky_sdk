// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;
import 'package:process/process.dart';

import '../framework/devices.dart';
import '../framework/framework.dart';
import '../framework/running_processes.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

final Directory _editedFlutterGalleryDir = dir(
  path.join(Directory.systemTemp.path, 'edited_flutter_gallery'),
);
final Directory flutterGalleryDir = dir(
  path.join(flutterDirectory.path, 'dev/integration_tests/flutter_gallery'),
);
const String kSourceLine = 'fontSize: (orientation == Orientation.portrait) ? 32.0 : 24.0';
const String kReplacementLine = 'fontSize: (orientation == Orientation.portrait) ? 34.0 : 24.0';

TaskFunction createHotModeTest({
  String? deviceIdOverride,
  bool checkAppRunningOnLocalDevice = false,
  List<String>? additionalOptions,
}) {
  // This file is modified during the test and needs to be restored at the end.
  final File flutterFrameworkSource = file(
    path.join(flutterDirectory.path, 'packages/flutter/lib/src/widgets/framework.dart'),
  );
  final String oldContents = flutterFrameworkSource.readAsStringSync();
  return () async {
    if (deviceIdOverride == null) {
      final Device device = await devices.workingDevice;
      await device.unlock();
      deviceIdOverride = device.deviceId;
    }
    final File benchmarkFile = file(path.join(_editedFlutterGalleryDir.path, 'hot_benchmark.json'));
    rm(benchmarkFile);
    final List<String> options = <String>[
      '--hot',
      '-d',
      deviceIdOverride!,
      '--benchmark',
      '--resident',
      '--no-android-gradle-daemon',
      '--no-publish-port',
      '--verbose',
      '--uninstall-first',
      if (additionalOptions != null) ...additionalOptions,
    ];
    int hotReloadCount = 0;
    late Map<String, dynamic> smallReloadData;
    late Map<String, dynamic> mediumReloadData;
    late Map<String, dynamic> largeReloadData;
    late Map<String, dynamic> freshRestartReloadsData;

    await inDirectory<void>(flutterDirectory, () async {
      rmTree(_editedFlutterGalleryDir);
      mkdirs(_editedFlutterGalleryDir);
      recursiveCopy(flutterGalleryDir, _editedFlutterGalleryDir);

      try {
        await inDirectory<void>(_editedFlutterGalleryDir, () async {
          smallReloadData = await captureReloadData(
            options: options,
            benchmarkFile: benchmarkFile,
            onLine: (String line, Process process) {
              if (!line.contains('Reloaded ')) {
                return;
              }
              if (hotReloadCount == 0) {
                // Update a file for 2 library invalidation.
                final File appDartSource = file(
                  path.join(_editedFlutterGalleryDir.path, 'lib/gallery/app.dart'),
                );
                appDartSource.writeAsStringSync(
                  appDartSource.readAsStringSync().replaceFirst(
                    "'Flutter Gallery'",
                    "'Updated Flutter Gallery'",
                  ),
                );
                process.stdin.writeln('r');
                hotReloadCount += 1;
              } else {
                process.stdin.writeln('q');
              }
            },
          );

          mediumReloadData = await captureReloadData(
            options: options,
            benchmarkFile: benchmarkFile,
            onLine: (String line, Process process) {
              if (!line.contains('Reloaded ')) {
                return;
              }
              if (hotReloadCount == 1) {
                // Update a file for ~50 library invalidation.
                final File appDartSource = file(
                  path.join(_editedFlutterGalleryDir.path, 'lib/demo/calculator/home.dart'),
                );
                appDartSource.writeAsStringSync(
                  appDartSource.readAsStringSync().replaceFirst(kSourceLine, kReplacementLine),
                );
                process.stdin.writeln('r');
                hotReloadCount += 1;
              } else {
                process.stdin.writeln('q');
              }
            },
          );

          largeReloadData = await captureReloadData(
            options: options,
            benchmarkFile: benchmarkFile,
            onLine: (String line, Process process) async {
              if (!line.contains('Reloaded ')) {
                return;
              }
              if (hotReloadCount == 2) {
                // Trigger a framework invalidation (370 libraries) without modifying the source
                flutterFrameworkSource.writeAsStringSync(
                  '${flutterFrameworkSource.readAsStringSync()}\n',
                );
                process.stdin.writeln('r');
                hotReloadCount += 1;
              } else {
                if (checkAppRunningOnLocalDevice) {
                  await _checkAppRunning(true);
                }
                process.stdin.writeln('q');
              }
            },
          );

          // Start `flutter run` again to make sure it loads from the previous
          // state. Frontend loads up from previously generated kernel files.
          {
            final Process process = await startFlutter('run', options: options);
            final Completer<void> stdoutDone = Completer<void>();
            final Completer<void> stderrDone = Completer<void>();
            process.stdout
                .transform<String>(utf8.decoder)
                .transform<String>(const LineSplitter())
                .listen(
                  (String line) {
                    if (line.contains('Reloaded ')) {
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

            freshRestartReloadsData =
                json.decode(benchmarkFile.readAsStringSync()) as Map<String, dynamic>;
          }
        });
        if (checkAppRunningOnLocalDevice) {
          await _checkAppRunning(false);
        }
      } finally {
        flutterFrameworkSource.writeAsStringSync(oldContents);
      }
    });

    return TaskResult.success(
      <String, dynamic>{
        'hotReloadInitialDevFSSyncMilliseconds':
            // ignore: avoid_dynamic_calls
            smallReloadData['hotReloadInitialDevFSSyncMilliseconds'][0],
        // ignore: avoid_dynamic_calls
        'hotRestartMillisecondsToFrame': smallReloadData['hotRestartMillisecondsToFrame'][0],
        // ignore: avoid_dynamic_calls
        'hotReloadMillisecondsToFrame': smallReloadData['hotReloadMillisecondsToFrame'][0],
        // ignore: avoid_dynamic_calls
        'hotReloadDevFSSyncMilliseconds': smallReloadData['hotReloadDevFSSyncMilliseconds'][0],
        'hotReloadFlutterReassembleMilliseconds':
            // ignore: avoid_dynamic_calls
            smallReloadData['hotReloadFlutterReassembleMilliseconds'][0],
        // ignore: avoid_dynamic_calls
        'hotReloadVMReloadMilliseconds': smallReloadData['hotReloadVMReloadMilliseconds'][0],
        'hotReloadMillisecondsToFrameAfterChange':
            // ignore: avoid_dynamic_calls
            smallReloadData['hotReloadMillisecondsToFrame'][1],
        'hotReloadDevFSSyncMillisecondsAfterChange':
            // ignore: avoid_dynamic_calls
            smallReloadData['hotReloadDevFSSyncMilliseconds'][1],
        'hotReloadFlutterReassembleMillisecondsAfterChange':
            // ignore: avoid_dynamic_calls
            smallReloadData['hotReloadFlutterReassembleMilliseconds'][1],
        'hotReloadVMReloadMillisecondsAfterChange':
            // ignore: avoid_dynamic_calls
            smallReloadData['hotReloadVMReloadMilliseconds'][1],
        'hotReloadInitialDevFSSyncAfterRelaunchMilliseconds':
            // ignore: avoid_dynamic_calls
            freshRestartReloadsData['hotReloadInitialDevFSSyncMilliseconds'][0],

        'hotReloadMillisecondsToFrameAfterMediumChange':
            // ignore: avoid_dynamic_calls
            mediumReloadData['hotReloadMillisecondsToFrame'][1],

        'hotReloadDevFSSyncMillisecondsAfterMediumChange':
            // ignore: avoid_dynamic_calls
            mediumReloadData['hotReloadDevFSSyncMilliseconds'][1],

        'hotReloadFlutterReassembleMillisecondsAfterMediumChange':
            // ignore: avoid_dynamic_calls
            mediumReloadData['hotReloadFlutterReassembleMilliseconds'][1],

        'hotReloadVMReloadMillisecondsAfterMediumChange':
            // ignore: avoid_dynamic_calls
            mediumReloadData['hotReloadVMReloadMilliseconds'][1],

        'hotReloadMillisecondsToFrameAfterLargeChange':
            // ignore: avoid_dynamic_calls
            largeReloadData['hotReloadMillisecondsToFrame'][1],

        'hotReloadDevFSSyncMillisecondsAfterLargeChange':
            // ignore: avoid_dynamic_calls
            largeReloadData['hotReloadDevFSSyncMilliseconds'][1],

        'hotReloadFlutterReassembleMillisecondsAfterLargeChange':
            // ignore: avoid_dynamic_calls
            largeReloadData['hotReloadFlutterReassembleMilliseconds'][1],

        'hotReloadVMReloadMillisecondsAfterLargeChange':
            // ignore: avoid_dynamic_calls
            largeReloadData['hotReloadVMReloadMilliseconds'][1],
      },
      benchmarkScoreKeys: <String>[
        'hotReloadInitialDevFSSyncMilliseconds',
        'hotRestartMillisecondsToFrame',
        'hotReloadMillisecondsToFrame',
        'hotReloadDevFSSyncMilliseconds',
        'hotReloadFlutterReassembleMilliseconds',
        'hotReloadVMReloadMilliseconds',
        'hotReloadMillisecondsToFrameAfterChange',
        'hotReloadDevFSSyncMillisecondsAfterChange',
        'hotReloadFlutterReassembleMillisecondsAfterChange',
        'hotReloadVMReloadMillisecondsAfterChange',
        'hotReloadInitialDevFSSyncAfterRelaunchMilliseconds',
        'hotReloadMillisecondsToFrameAfterMediumChange',
        'hotReloadDevFSSyncMillisecondsAfterMediumChange',
        'hotReloadFlutterReassembleMillisecondsAfterMediumChange',
        'hotReloadVMReloadMillisecondsAfterMediumChange',
        'hotReloadMillisecondsToFrameAfterLargeChange',
        'hotReloadDevFSSyncMillisecondsAfterLargeChange',
        'hotReloadFlutterReassembleMillisecondsAfterLargeChange',
        'hotReloadVMReloadMillisecondsAfterLargeChange',
      ],
    );
  };
}

Future<Map<String, dynamic>> captureReloadData({
  required List<String> options,
  required File benchmarkFile,
  required void Function(String, Process) onLine,
}) async {
  final Process process = await startFlutter('run', options: options);

  final Completer<void> stdoutDone = Completer<void>();
  final Completer<void> stderrDone = Completer<void>();
  process.stdout.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen((
    String line,
  ) {
    onLine(line, process);
    print('stdout: $line');
  }, onDone: stdoutDone.complete);

  process.stderr
      .transform<String>(utf8.decoder)
      .transform<String>(const LineSplitter())
      .listen((String line) => print('stderr: $line'), onDone: stderrDone.complete);

  await Future.wait<void>(<Future<void>>[stdoutDone.future, stderrDone.future]);
  await process.exitCode;
  final Map<String, dynamic> result =
      json.decode(benchmarkFile.readAsStringSync()) as Map<String, dynamic>;
  benchmarkFile.deleteSync();
  return result;
}

Future<void> _checkAppRunning(bool shouldBeRunning) async {
  late Set<RunningProcessInfo> galleryProcesses;
  for (int i = 0; i < 10; i++) {
    final String exe = Platform.isWindows ? '.exe' : '';
    galleryProcesses = await getRunningProcesses(
      processName: 'Flutter Gallery$exe',
      processManager: const LocalProcessManager(),
    );

    if (galleryProcesses.isNotEmpty == shouldBeRunning) {
      return;
    }

    // Give the app time to shut down.
    sleep(const Duration(seconds: 1));
  }
  print(galleryProcesses.join('\n'));
  throw TaskResult.failure('Flutter Gallery app is ${shouldBeRunning ? 'not' : 'still'} running');
}
