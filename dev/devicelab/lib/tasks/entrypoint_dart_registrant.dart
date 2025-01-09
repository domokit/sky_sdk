// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' show Directory, File, Process, ProcessSignal;

import '../framework/devices.dart';
import '../framework/framework.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

const String _messagePrefix = 'entrypoint:';
const String _entrypointName = 'entrypoint';

const String _dartCode = '''
import 'package:flutter/widgets.dart';

@pragma('vm:entry-point')
void main() {
  print('$_messagePrefix main');
  runApp(const ColoredBox(color: Color(0xffcc0000)));
}

@pragma('vm:entry-point')
void $_entrypointName() {
  print('$_messagePrefix $_entrypointName');
  runApp(const ColoredBox(color: Color(0xff00cc00)));
}
''';

const String _kotlinCode = '''
package com.example.entrypoint_dart_registrant

import io.flutter.embedding.android.FlutterActivity

class MainActivity: FlutterActivity() {
  override fun getDartEntrypointFunctionName(): String {
    return "$_entrypointName"
  }
}
''';

Future<TaskResult> _runWithTempDir(Directory tempDir) async {
  const String testDirName = 'entrypoint_dart_registrant';
  final String testPath = '${tempDir.path}/$testDirName';
  await inDirectory(tempDir, () async {
    await flutter('create', options: <String>['--platforms', 'android', testDirName]);
  });
  final String mainPath = '${tempDir.path}/$testDirName/lib/main.dart';
  print(mainPath);
  File(mainPath).writeAsStringSync(_dartCode);
  final String activityPath =
      '${tempDir.path}/$testDirName/android/app/src/main/kotlin/com/example/entrypoint_dart_registrant/MainActivity.kt';
  File(activityPath).writeAsStringSync(_kotlinCode);
  final Device device = await devices.workingDevice;
  await device.unlock();
  final String entrypoint = await inDirectory(testPath, () async {
    // The problem only manifested when the dart plugin registrant was used
    // (which path_provider has).
    await flutter('pub', options: <String>['add', 'path_provider:2.0.9']);
    // The problem only manifested on release builds, so we test release.
    final Process process = await startFlutter('run', options: <String>['--release']);
    final Completer<String> completer = Completer<String>();
    final StreamSubscription<String> stdoutSub = process.stdout
        .transform<String>(const Utf8Decoder())
        .transform<String>(const LineSplitter())
        .listen((String line) async {
          print(line);
          if (line.contains(_messagePrefix)) {
            completer.complete(line);
          }
        });
    final String entrypoint = await completer.future;
    await stdoutSub.cancel();
    process.stdin.write('q');
    await process.stdin.flush();
    process.kill(ProcessSignal.sigint);
    return entrypoint;
  });
  if (entrypoint.contains('$_messagePrefix $_entrypointName')) {
    return TaskResult.success(null);
  } else {
    return TaskResult.failure('expected entrypoint:"$_entrypointName" but found:"$entrypoint"');
  }
}

/// Asserts that the custom entrypoint works in the presence of the dart plugin
/// registrant.
TaskFunction entrypointDartRegistrant() {
  return () async {
    final Directory tempDir = Directory.systemTemp.createTempSync('entrypoint_dart_registrant.');
    try {
      return await _runWithTempDir(tempDir);
    } finally {
      rmTree(tempDir);
    }
  };
}
