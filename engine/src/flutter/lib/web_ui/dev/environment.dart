// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' as ffi;
import 'dart:io' as io;

import 'package:path/path.dart' as pathlib;

import 'exceptions.dart';

/// Contains various environment variables, such as common file paths and command-line options.
Environment get environment {
  return _environment ??= Environment();
}

Environment? _environment;

/// Contains various environment variables, such as common file paths and command-line options.
class Environment {
  factory Environment() {
    final bool isMacosArm = ffi.Abi.current() == ffi.Abi.macosArm64;
    final io.File dartExecutable = io.File(io.Platform.resolvedExecutable);
    final io.File self = io.File.fromUri(io.Platform.script);

    final io.Directory engineSrcDir = self.parent.parent.parent.parent.parent;
    final io.Directory engineToolsDir = io.Directory(
      pathlib.join(engineSrcDir.path, 'flutter', 'tools'),
    );
    final io.Directory outDir = io.Directory(pathlib.join(engineSrcDir.path, 'out'));
    final io.Directory wasmReleaseOutDir = io.Directory(pathlib.join(outDir.path, 'wasm_release'));
    final io.Directory wasmProfileOutDir = io.Directory(pathlib.join(outDir.path, 'wasm_profile'));
    final io.Directory wasmDebugUnoptOutDir = io.Directory(
      pathlib.join(outDir.path, 'wasm_debug_unopt'),
    );
    final io.Directory hostDebugUnoptDir = io.Directory(
      pathlib.join(outDir.path, 'host_debug_unopt'),
    );
    final io.Directory dartSdkDir = dartExecutable.parent.parent;
    final io.Directory webUiRootDir = io.Directory(
      pathlib.join(engineSrcDir.path, 'flutter', 'lib', 'web_ui'),
    );

    for (final io.Directory expectedDirectory in <io.Directory>[engineSrcDir, webUiRootDir]) {
      if (!expectedDirectory.existsSync()) {
        throw ToolExit('$expectedDirectory does not exist.');
      }
    }

    return Environment._(
      self: self,
      isMacosArm: isMacosArm,
      webUiRootDir: webUiRootDir,
      engineSrcDir: engineSrcDir,
      engineToolsDir: engineToolsDir,
      outDir: outDir,
      wasmReleaseOutDir: wasmReleaseOutDir,
      wasmProfileOutDir: wasmProfileOutDir,
      wasmDebugUnoptOutDir: wasmDebugUnoptOutDir,
      hostDebugUnoptDir: hostDebugUnoptDir,
      dartSdkDir: dartSdkDir,
    );
  }

  Environment._({
    required this.self,
    required this.isMacosArm,
    required this.webUiRootDir,
    required this.engineSrcDir,
    required this.engineToolsDir,
    required this.outDir,
    required this.wasmReleaseOutDir,
    required this.wasmProfileOutDir,
    required this.wasmDebugUnoptOutDir,
    required this.hostDebugUnoptDir,
    required this.dartSdkDir,
  });

  /// The Dart script that's currently running.
  final io.File self;

  /// Whether the environment is a macOS arm environment.
  final bool isMacosArm;

  /// Path to the "web_ui" package sources.
  final io.Directory webUiRootDir;

  /// Path to the engine's "src" directory.
  final io.Directory engineSrcDir;

  /// Path to the engine's "tools" directory.
  final io.Directory engineToolsDir;

  /// Path to the engine's "out" directory.
  ///
  /// This is where you'll find the ninja output, such as the Dart SDK.
  final io.Directory outDir;

  /// The output directory for the wasm_release build.
  ///
  /// We build CanvasKit in release mode to reduce code size.
  final io.Directory wasmReleaseOutDir;

  /// The output directory for the wasm_profile build.
  final io.Directory wasmProfileOutDir;

  /// The output directory for the wasm_debug build.
  final io.Directory wasmDebugUnoptOutDir;

  /// The output directory for the host_debug_unopt build.
  final io.Directory hostDebugUnoptDir;

  /// The root of the Dart SDK.
  final io.Directory dartSdkDir;

  /// The "dart" executable file.
  String get dartExecutable => pathlib.join(dartSdkDir.path, 'bin', 'dart');

  /// Path to dartaotruntime for running aot snapshots
  String get dartAotRuntimePath => pathlib.join(dartSdkDir.path, 'bin', 'dartaotruntime');

  /// The "pub" executable file.
  String get pubExecutable => pathlib.join(dartSdkDir.path, 'bin', 'pub');

  /// The path to dart2wasm pre-compiled snapshot
  String get dart2wasmSnapshotPath =>
      pathlib.join(dartSdkDir.path, 'bin', 'snapshots', 'dart2wasm_product.snapshot');

  /// The path to dart2wasm.dart file
  String get dart2wasmScriptPath => pathlib.join(
    engineSrcDir.path,
    'third_party',
    'dart',
    'pkg',
    'dart2wasm',
    'bin',
    'dart2wasm.dart',
  );

  /// Path to where github.com/flutter/engine is checked out inside the engine workspace.
  io.Directory get flutterDirectory => io.Directory(pathlib.join(engineSrcDir.path, 'flutter'));
  io.Directory get webSdkRootDir => io.Directory(pathlib.join(flutterDirectory.path, 'web_sdk'));

  /// Path to the "web_engine_tester" package.
  io.Directory get webEngineTesterRootDir =>
      io.Directory(pathlib.join(webSdkRootDir.path, 'web_engine_tester'));

  /// Path to the "build" directory, generated by "package:build_runner".
  ///
  /// This is where compiled test output goes.
  io.Directory get webUiBuildDir => io.Directory(pathlib.join(outDir.path, 'web_tests'));

  io.Directory get webTestsArtifactsDir =>
      io.Directory(pathlib.join(webUiBuildDir.path, 'artifacts'));

  /// Path to the ".dart_tool" directory, generated by various Dart tools.
  io.Directory get webUiDartToolDir => io.Directory(pathlib.join(webUiRootDir.path, '.dart_tool'));

  /// Path to the ".dart_tool" directory living under `engine/src/flutter`.
  ///
  /// This is a designated area for tool downloads which can be used by
  /// multiple platforms. For exampe: Flutter repo for e2e tests.
  io.Directory get engineDartToolDir =>
      io.Directory(pathlib.join(engineSrcDir.path, 'flutter', '.dart_tool'));

  /// Path to the "dev" directory containing engine developer tools and
  /// configuration files.
  io.Directory get webUiDevDir => io.Directory(pathlib.join(webUiRootDir.path, 'dev'));

  /// Path to the "test" directory containing web engine tests.
  io.Directory get webUiTestDir => io.Directory(pathlib.join(webUiRootDir.path, 'test'));

  /// Path to the "lib" directory containing web engine code.
  io.Directory get webUiLibDir => io.Directory(pathlib.join(webUiRootDir.path, 'lib'));

  /// Path to the base directory to be used by Skia Gold.
  io.Directory get webUiSkiaGoldDirectory =>
      io.Directory(pathlib.join(webUiDartToolDir.path, 'skia_gold'));

  /// Directory to add test results which would later be uploaded to a gcs
  /// bucket by LUCI.
  io.Directory get webUiTestResultsDirectory =>
      io.Directory(pathlib.join(webUiDartToolDir.path, 'test_results'));
}
