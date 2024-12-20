// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:args/command_runner.dart';
import 'package:path/path.dart' as path;
import 'package:watcher/watcher.dart';

import 'environment.dart';
import 'exceptions.dart';
import 'pipeline.dart';
import 'utils.dart';

const Map<String, String> targetAliases = <String, String>{
  'sdk': 'flutter/web_sdk',
  'web_sdk': 'flutter/web_sdk',
  'canvaskit': 'flutter/third_party/canvaskit:canvaskit_group',
  'canvaskit_chromium': 'flutter/third_party/canvaskit:canvaskit_chromium_group',
  'skwasm': 'flutter/third_party/canvaskit:skwasm_group',
  'skwasm_st': 'flutter/third_party/canvaskit:skwasm_st_group',
  'archive': 'flutter/web_sdk:flutter_web_sdk_archive',
};

class BuildCommand extends Command<bool> with ArgUtils<bool> {
  BuildCommand() {
    argParser.addFlag(
      'watch',
      abbr: 'w',
      help:
          'Run the build in watch mode so it rebuilds whenever a change is '
          'made. Disabled by default.',
    );
    argParser.addFlag(
      'host',
      help:
          'Build the host build instead of the wasm build, which is '
          'currently needed for `flutter run --local-engine` to work.',
    );
    argParser.addFlag(
      'profile',
      help:
          'Build in profile mode instead of release mode. In this mode, the '
          'output will be located at "out/wasm_profile".\nThis only applies to '
          'the wasm build. The host build is always built in release mode.',
    );
    argParser.addFlag(
      'debug',
      help:
          'Build in debug mode instead of release mode. In this mode, the '
          'output will be located at "out/wasm_debug".\nThis only applies to '
          'the wasm build. The host build is always built in release mode.',
    );
    argParser.addFlag(
      'dwarf',
      help:
          'Embed DWARF debugging info into the output wasm modules. This is '
          'only valid in debug mode.',
    );
  }

  @override
  String get name => 'build';

  @override
  String get description => 'Build the Flutter web engine.';

  bool get isWatchMode => boolArg('watch');

  bool get host => boolArg('host');

  List<String> get targets => argResults?.rest ?? <String>[];
  bool get embedDwarf => boolArg('dwarf');

  RuntimeMode get runtimeMode {
    final bool isProfile = boolArg('profile');
    final bool isDebug = boolArg('debug');
    if (isProfile && isDebug) {
      throw ToolExit('Cannot specify both --profile and --debug at the same time.');
    }
    if (isProfile) {
      return RuntimeMode.profile;
    } else if (isDebug) {
      return RuntimeMode.debug;
    } else {
      return RuntimeMode.release;
    }
  }

  @override
  FutureOr<bool> run() async {
    if (embedDwarf && runtimeMode != RuntimeMode.debug) {
      throw ToolExit('Embedding DWARF data requires debug runtime mode.');
    }
    final FilePath libPath = FilePath.fromWebUi('lib');
    final List<PipelineStep> steps = <PipelineStep>[
      GnPipelineStep(host: host, runtimeMode: runtimeMode, embedDwarf: embedDwarf),
      NinjaPipelineStep(
        host: host,
        runtimeMode: runtimeMode,
        targets: targets.map((String target) => targetAliases[target] ?? target),
      ),
    ];
    final Pipeline buildPipeline = Pipeline(steps: steps);
    await buildPipeline.run();

    if (isWatchMode) {
      print('Initial build done!');
      print('Watching directory: ${libPath.relativeToCwd}/');
      await PipelineWatcher(
        dir: libPath.absolute,
        pipeline: buildPipeline,
        // Ignore font files that are copied whenever tests run.
        ignore: (WatchEvent event) => event.path.endsWith('.ttf'),
      ).start();
    }
    return true;
  }
}

/// Runs `gn`.
///
/// Not safe to interrupt as it may leave the `out/` directory in a corrupted
/// state. GN is pretty quick though, so it's OK to not support interruption.
class GnPipelineStep extends ProcessStep {
  GnPipelineStep({required this.host, required this.runtimeMode, required this.embedDwarf});

  final bool host;
  final RuntimeMode runtimeMode;
  final bool embedDwarf;

  @override
  String get description => 'gn';

  @override
  bool get isSafeToInterrupt => false;

  List<String> get _gnArgs {
    if (host) {
      return <String>['--unoptimized', '--full-dart-sdk'];
    } else {
      return <String>[
        '--web',
        '--runtime-mode=${runtimeMode.name}',
        if (runtimeMode == RuntimeMode.debug) '--unoptimized',
        if (embedDwarf) '--wasm-use-dwarf',
      ];
    }
  }

  @override
  Future<ProcessManager> createProcess() {
    print('Running gn...');
    return startProcess(path.join(environment.flutterDirectory.path, 'tools', 'gn'), _gnArgs);
  }
}

/// Runs `autoninja`.
///
/// Can be safely interrupted.
class NinjaPipelineStep extends ProcessStep {
  NinjaPipelineStep({required this.host, required this.runtimeMode, required this.targets});

  @override
  String get description => 'ninja';

  @override
  bool get isSafeToInterrupt => true;

  final bool host;
  final Iterable<String> targets;
  final RuntimeMode runtimeMode;

  String get buildDirectory {
    if (host) {
      return environment.hostDebugUnoptDir.path;
    }
    return getBuildDirectoryForRuntimeMode(runtimeMode).path;
  }

  @override
  Future<ProcessManager> createProcess() {
    print('Running autoninja...');
    return startProcess('autoninja', <String>['-C', buildDirectory, ...targets]);
  }
}
