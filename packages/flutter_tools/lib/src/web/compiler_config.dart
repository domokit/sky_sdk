// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../build_info.dart' show BuildMode;
import '../convert.dart';
import 'compile.dart';

enum CompileTarget { js, wasm }

sealed class WebCompilerConfig {
  const WebCompilerConfig({
    required this.renderer,
    this.optimizationLevel,
    required this.sourceMaps,
  });

  /// Build environment flag for [optimizationLevel].
  static const String kOptimizationLevel = 'OptimizationLevel';

  /// Build environment flag for [sourceMaps].
  static const String kSourceMapsEnabled = 'SourceMaps';

  /// Calculates the optimization level for the compiler for the given
  /// build mode.
  int optimizationLevelForBuildMode(BuildMode mode);

  /// The compiler optimization level specified by the user.
  ///
  /// Valid values are O0 (lowest, debug default) to O4 (highest, release default).
  /// If the value is null, the user hasn't specified an optimization level and an
  /// appropriate default for the build mode will be used instead.
  final int? optimizationLevel;

  /// `true` if the compiler build should output source maps.
  final bool sourceMaps;

  /// Returns which target this compiler outputs (js or wasm)
  CompileTarget get compileTarget;
  final WebRendererMode renderer;

  String get buildKey;

  Map<String, Object> get buildEventAnalyticsValues => <String, Object>{
    if (optimizationLevel != null) 'optimizationLevel': optimizationLevel!,
  };

  Map<String, dynamic> get _buildKeyMap => <String, dynamic>{
    'optimizationLevel': optimizationLevel,
    'webRenderer': renderer.name,
  };
}

/// Configuration for the Dart-to-Javascript compiler (dart2js).
class JsCompilerConfig extends WebCompilerConfig {
  const JsCompilerConfig({
    this.csp = false,
    this.dumpInfo = false,
    this.nativeNullAssertions = false,
    super.optimizationLevel,
    this.noFrequencyBasedMinification = false,
    super.sourceMaps = true,
    super.renderer = WebRendererMode.defaultForJs,
  });

  /// Instantiates [JsCompilerConfig] suitable for the `flutter run` command.
  const JsCompilerConfig.run({
    required bool nativeNullAssertions,
    required WebRendererMode renderer,
  }) : this(nativeNullAssertions: nativeNullAssertions, renderer: renderer);

  /// Build environment flag for [dumpInfo].
  static const String kDart2jsDumpInfo = 'Dart2jsDumpInfo';

  /// Build environment flag for [noFrequencyBasedMinification].
  static const String kDart2jsNoFrequencyBasedMinification = 'Dart2jsNoFrequencyBasedMinification';

  /// Build environment flag for [csp].
  static const String kCspMode = 'cspMode';

  /// Build environment flag for [nativeNullAssertions].
  static const String kNativeNullAssertions = 'NativeNullAssertions';

  /// Whether to disable dynamic generation code to satisfy CSP policies.
  final bool csp;

  /// If `--dump-info` should be passed to the compiler.
  final bool dumpInfo;

  /// Whether native null assertions are enabled.
  final bool nativeNullAssertions;

  // If `--no-frequency-based-minification` should be passed to dart2js
  // TODO(kevmoo): consider renaming this to be "positive". Double negatives are confusing.
  final bool noFrequencyBasedMinification;

  @override
  CompileTarget get compileTarget => CompileTarget.js;

  /// Arguments to use in both phases: full JS compile and CFE-only.
  List<String> toSharedCommandOptions(BuildMode buildMode) => <String>[
    if (nativeNullAssertions) '--native-null-assertions',
    if (!sourceMaps) '--no-source-maps',
    if (buildMode == BuildMode.debug) '--enable-asserts',
  ];

  @override
  int optimizationLevelForBuildMode(BuildMode mode) =>
      optimizationLevel ??
      switch (mode) {
        // dart2js optimization level 0 is not well supported. Use
        // 1 instead.
        BuildMode.debug => 1,
        BuildMode.profile || BuildMode.release => 4,
        BuildMode.jitRelease => throw ArgumentError('Invalid build mode for web'),
      };

  /// Arguments to use in the full JS compile, but not CFE-only.
  ///
  /// Includes the contents of [toSharedCommandOptions].
  List<String> toCommandOptions(BuildMode buildMode) => <String>[
    if (buildMode != BuildMode.release) '--no-minify',
    ...toSharedCommandOptions(buildMode),
    '-O${optimizationLevelForBuildMode(buildMode)}',
    if (dumpInfo) '--stage=dump-info-all',
    if (noFrequencyBasedMinification) '--no-frequency-based-minification',
    if (csp) '--csp',
  ];

  @override
  String get buildKey {
    final Map<String, dynamic> settings = <String, dynamic>{
      ...super._buildKeyMap,
      'csp': csp,
      'dumpInfo': dumpInfo,
      'nativeNullAssertions': nativeNullAssertions,
      'noFrequencyBasedMinification': noFrequencyBasedMinification,
      'sourceMaps': sourceMaps,
    };
    return jsonEncode(settings);
  }
}

/// Configuration for the Wasm compiler.
class WasmCompilerConfig extends WebCompilerConfig {
  const WasmCompilerConfig({
    super.optimizationLevel,
    this.stripWasm = true,
    super.sourceMaps = true,
    super.renderer = WebRendererMode.defaultForWasm,
  });

  /// Build environment for [stripWasm].
  static const String kStripWasm = 'StripWasm';

  /// Whether to strip the wasm file of static symbols.
  final bool stripWasm;

  @override
  CompileTarget get compileTarget => CompileTarget.wasm;

  @override
  int optimizationLevelForBuildMode(BuildMode mode) =>
      optimizationLevel ??
      switch (mode) {
        BuildMode.debug => 0,

        // The optimization level of O2 uses only sound optimizations. We default
        // to this level because our web benchmarks have shown that the difference
        // between O2 and O4 is marginal enough that we would prefer soundness here.
        BuildMode.profile || BuildMode.release => 2,
        BuildMode.jitRelease => throw ArgumentError('Invalid build mode for web'),
      };

  List<String> toCommandOptions(BuildMode buildMode) {
    final bool stripSymbols = buildMode == BuildMode.release && stripWasm;
    return <String>[
      '-O${optimizationLevelForBuildMode(buildMode)}',
      '--${stripSymbols ? '' : 'no-'}strip-wasm',
      if (!sourceMaps) '--no-source-maps',
      if (buildMode == BuildMode.debug) '--extra-compiler-option=--enable-asserts',
    ];
  }

  @override
  String get buildKey {
    final Map<String, dynamic> settings = <String, dynamic>{
      ...super._buildKeyMap,
      'stripWasm': stripWasm,
      'sourceMaps': sourceMaps,
    };
    return jsonEncode(settings);
  }
}
