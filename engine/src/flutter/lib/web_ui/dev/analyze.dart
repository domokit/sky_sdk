// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:args/command_runner.dart';

import 'environment.dart';
import 'pipeline.dart';
import 'utils.dart';

class AnalyzeCommand extends Command<bool> with ArgUtils<bool> {
  @override
  String get name => 'analyze';

  @override
  String get description => 'Analyze the Flutter web engine.';

  @override
  FutureOr<bool> run() async {
    final Pipeline buildPipeline = Pipeline(steps: <PipelineStep>[PubGetStep(), AnalyzeStep()]);
    await buildPipeline.run();
    return true;
  }
}

/// Runs `dart pub get`.
class PubGetStep extends ProcessStep {
  @override
  String get description => 'pub get';

  @override
  bool get isSafeToInterrupt => true;

  @override
  Future<ProcessManager> createProcess() {
    print('Running `dart pub get`...');
    return startProcess(environment.dartExecutable, <String>[
      'pub',
      'get',
    ], workingDirectory: environment.webUiRootDir.path);
  }
}

/// Runs `dart analyze --fatal-infos`.
class AnalyzeStep extends ProcessStep {
  @override
  String get description => 'analyze';

  @override
  bool get isSafeToInterrupt => true;

  @override
  Future<ProcessManager> createProcess() {
    print('Running `dart analyze`...');
    return startProcess(environment.dartExecutable, <String>[
      'analyze',
      '--fatal-infos',
    ], workingDirectory: environment.webUiRootDir.path);
  }
}
