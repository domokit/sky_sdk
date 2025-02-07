// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:process/process.dart';

import '../src/common.dart';
import 'test_data/basic_project.dart';
import 'test_utils.dart';

const List<String> firstLaunchMessages = <String>[
  'Creating widget preview scaffolding at:',
  'Performing initial build of the Widget Preview Scaffold...',
  'Widget Preview Scaffold initial build complete.',
  'Loading previews into the Widget Preview Scaffold...',
  'Done loading previews.',
];

const List<String> subsequentLaunchMessages = <String>[
  'Loading previews into the Widget Preview Scaffold...',
  'Done loading previews.',
];

void main() {
  late Directory tempDir;
  Process? process;
  final BasicProject project = BasicProject();
  const ProcessManager processManager = LocalProcessManager();

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('widget_preview_test.');
    await project.setUpIn(tempDir);
  });

  tearDown(() async {
    process?.kill();
    process = null;
    tryToDelete(tempDir);
  });

  Future<void> runWidgetPreview({required List<String> expectedMessages}) async {
    expect(expectedMessages, isNotEmpty);
    int i = 0;
    process = await processManager.start(<String>[
      flutterBin,
      'widget-preview',
      'start',
    ], workingDirectory: tempDir.path);

    final Completer<void> completer = Completer<void>();
    process!.stdout.transform(utf8.decoder).transform(const LineSplitter()).listen((String msg) {
      printOnFailure('STDOUT: $msg');
      if (completer.isCompleted) {
        return;
      }
      if (msg.contains(expectedMessages[i])) {
        ++i;
      }
      if (i == expectedMessages.length) {
        completer.complete();
      }
    });

    process!.stderr.transform(utf8.decoder).transform(const LineSplitter()).listen((String msg) {
      printOnFailure('STDERR: $msg');
    });

    await completer.future;
    process!.kill();
    process = null;
  }

  group('flutter widget-preview start', () {
    testWithoutContext('smoke test', () async {
      await runWidgetPreview(expectedMessages: firstLaunchMessages);
    });

    testWithoutContext('does not rebuild project on subsequent runs', () async {
      // The first run of 'flutter widget-preview start' should generate a new preview scaffold and
      // pre-build the application.
      await runWidgetPreview(expectedMessages: firstLaunchMessages);

      // We shouldn't regenerate the scaffold after the initial run.
      await runWidgetPreview(expectedMessages: subsequentLaunchMessages);
    });
  });
}
