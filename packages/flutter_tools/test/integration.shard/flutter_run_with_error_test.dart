// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'dart:async';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:vm_service/vm_service.dart';
import 'package:vm_service/vm_service_io.dart';

import '../src/common.dart';
import 'test_data/project_with_early_error.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;
  final ProjectWithEarlyError project = ProjectWithEarlyError();
  const String exceptionStart = '══╡ EXCEPTION CAUGHT BY WIDGETS LIBRARY ╞══════════════════';
  late FlutterRunTestDriver flutter;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterRunTestDriver(tempDir);
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext(
    'flutter run in non-machine mode reports an early error in an application',
    () async {
      final StringBuffer stdout = StringBuffer();

      final Process process = await processManager.start(<String>[
        flutterBin,
        'run',
        '--disable-service-auth-codes',
        '--show-test-device',
        '-dflutter-tester',
        '--start-paused',
        '--dart-define=flutter.inspector.structuredErrors=true',
      ], workingDirectory: tempDir.path);

      transformToLines(process.stdout).listen((String line) async {
        stdout.writeln(line);

        if (line.startsWith('A Dart VM Service on')) {
          final RegExp exp = RegExp(r'http://127.0.0.1:(\d+)/');
          final RegExpMatch match = exp.firstMatch(line)!;
          final String port = match.group(1)!;
          final VmService vmService = await vmServiceConnectUri('ws://localhost:$port/ws');
          final VM vm = await vmService.getVM();
          for (final IsolateRef isolate in vm.isolates!) {
            await vmService.resume(isolate.id!);
          }
        }

        if (line.startsWith('Another exception was thrown')) {
          process.kill();
        }
      });

      await process.exitCode;

      expect(stdout.toString(), contains(exceptionStart));
    },
  );

  testWithoutContext('flutter run in machine mode does not print an error', () async {
    final StringBuffer stdout = StringBuffer();

    await flutter.run(startPaused: true, withDebugger: true, structuredErrors: true);
    await flutter.resume();

    final Completer<void> completer = Completer<void>();

    await Future<void>(() async {
      flutter.stdout.listen((String line) {
        stdout.writeln(line);
      });
      await completer.future;
    }).timeout(
      const Duration(seconds: 5),
      onTimeout: () {
        // We don't expect to see any output but want to write to stdout anyway.
        completer.complete();
      },
    );
    await flutter.stop();

    expect(stdout.toString(), isNot(contains(exceptionStart)));
  });
}
