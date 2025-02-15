// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:async/async.dart';
import 'package:convert/convert.dart';
import 'package:crypto/crypto.dart';
import 'package:meta/meta.dart';
import 'package:pool/pool.dart';
import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../artifacts.dart';
import '../base/error_handling_io.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../cache.dart';
import '../convert.dart';
import 'depfile.dart';
import 'exceptions.dart';
import 'file_store.dart';
import 'source.dart';

export 'source.dart';

/// A reasonable amount of files to open at the same time.
///
/// This number is somewhat arbitrary - it is difficult to detect whether
/// or not we'll run out of file descriptors when using async dart:io
/// APIs.
const int kMaxOpenFiles = 64;

/// Configuration for the build system itself.
class BuildSystemConfig {
  /// Create a new [BuildSystemConfig].
  const BuildSystemConfig({this.resourcePoolSize});

  /// The maximum number of concurrent tasks the build system will run.
  ///
  /// If not provided, defaults to [platform.numberOfProcessors].
  final int? resourcePoolSize;
}

/// A Target describes a single step during a flutter build.
///
/// The target inputs are required to be files discoverable via a combination
/// of at least one of the environment values and zero or more local values.
///
/// To determine if the action for a target needs to be executed, the
/// [BuildSystem] computes a key of the file contents for both inputs and
/// outputs. This is tracked separately in the [FileStore]. The key may
/// be either an md5 hash of the file contents or a timestamp.
///
/// A Target has both implicit and explicit inputs and outputs. Only the
/// later are safe to evaluate before invoking the [buildAction]. For example,
/// a wildcard output pattern requires the outputs to exist before it can
/// glob files correctly.
///
/// - All listed inputs are considered explicit inputs.
/// - Outputs which are provided as [Source.pattern].
///   without wildcards are considered explicit.
/// - The remaining outputs are considered implicit.
///
/// For each target, executing its action creates a corresponding stamp file
/// which records both the input and output files. This file is read by
/// subsequent builds to determine which file hashes need to be checked. If the
/// stamp file is missing, the target's action is always rerun.
///
///  file: `example_target.stamp`
///
/// {
///   "inputs": [
///      "absolute/path/foo",
///      "absolute/path/bar",
///      ...
///    ],
///    "outputs": [
///      "absolute/path/fizz"
///    ]
/// }
///
/// ## Code review
///
/// ### Targets should only depend on files that are provided as inputs
///
/// Example: gen_snapshot must be provided as an input to the aot_elf
/// build steps, even though it isn't a source file. This ensures that changes
/// to the gen_snapshot binary (during a local engine build) correctly
/// trigger a corresponding build update.
///
/// Example: aot_elf has a dependency on the dill and packages file
/// produced by the kernel_snapshot step.
///
/// ### Targets should declare all outputs produced
///
/// If a target produces an output it should be listed, even if it is not
/// intended to be consumed by another target.
///
/// ## Unit testing
///
/// Most targets will invoke an external binary which makes unit testing
/// trickier. It is recommend that for unit testing that a Fake is used and
/// provided via the dependency injection system. a [Testbed] may be used to
/// set up the environment before the test is run. Unit tests should fully
/// exercise the rule, ensuring that the existing input and output verification
/// logic can run, as well as verifying it correctly handles provided defines
/// and meets any additional contracts present in the target.
abstract class Target {
  const Target();

  /// The user-readable name of the target.
  ///
  /// This information is surfaced in the assemble commands and used as an
  /// argument to build a particular target.
  String get name;

  /// A name that measurements can be categorized under for this [Target].
  ///
  /// Unlike [name], this is not expected to be unique, so multiple targets
  /// that are conceptually the same can share an analytics name.
  ///
  /// If not provided, defaults to [name]
  String get analyticsName => name;

  /// The dependencies of this target.
  List<Target> get dependencies;

  /// The input [Source]s which are diffed to determine if a target should run.
  List<Source> get inputs;

  /// The output [Source]s which we attempt to verify are correctly produced.
  List<Source> get outputs;

  /// A list of zero or more depfiles, located directly under {BUILD_DIR}.
  List<String> get depfiles => const <String>[];

  /// A string that differentiates different build variants from each other
  /// with regards to build flags or settings on the target. This string should
  /// represent each build variant as a different unique value. If this value
  /// changes between builds, the target will be invalidated and rebuilt.
  ///
  /// By default, this returns null, which indicates there is only one build
  /// variant, and the target won't invalidate or rebuild due to this property.
  String? get buildKey => null;

  /// Whether this target can be executed with the given [environment].
  ///
  /// Returning `true` will cause [build] to be skipped. This is equivalent
  /// to a build that produces no outputs.
  bool canSkip(Environment environment) => false;

  /// The action which performs this build step.
  Future<void> build(Environment environment);

  /// Create a [Node] with resolved inputs and outputs.
  Node _toNode(Environment environment) {
    final ResolvedFiles inputsFiles = resolveInputs(environment);
    final ResolvedFiles outputFiles = resolveOutputs(environment);
    return Node(
      this,
      inputsFiles.sources,
      outputFiles.sources,
      <Node>[for (final Target target in dependencies) target._toNode(environment)],
      buildKey,
      environment,
      inputsFiles.containsNewDepfile,
    );
  }

  /// Invoke to remove the stamp file if the [buildAction] threw an exception.
  void clearStamp(Environment environment) {
    final File stamp = _findStampFile(environment);
    ErrorHandlingFileSystem.deleteIfExists(stamp);
  }

  void _writeStamp(List<File> inputs, List<File> outputs, Environment environment) {
    String getPath(File file) => file.path;
    final Map<String, Object> result = <String, Object>{
      'inputs': inputs.map(getPath).toList(),
      'outputs': outputs.map(getPath).toList(),
      if (buildKey case final String key) 'buildKey': key,
    };
    final File stamp = _findStampFile(environment);
    if (!stamp.existsSync()) {
      stamp.createSync();
    }
    stamp.writeAsStringSync(json.encode(result));
  }

  /// Resolve the set of input patterns and functions into a concrete list of
  /// files.
  ResolvedFiles resolveInputs(Environment environment) {
    return _resolveConfiguration(inputs, depfiles, environment);
  }

  /// Find the current set of declared outputs, including wildcard directories.
  ///
  /// The [implicit] flag controls whether it is safe to evaluate [Source]s
  /// which uses functions, behaviors, or patterns.
  ResolvedFiles resolveOutputs(Environment environment) {
    return _resolveConfiguration(outputs, depfiles, environment, inputs: false);
  }

  /// Performs a fold across this target and its dependencies.
  T fold<T>(T initialValue, T Function(T previousValue, Target target) combine) {
    final T dependencyResult = dependencies.fold(
      initialValue,
      (T prev, Target t) => t.fold(prev, combine),
    );
    return combine(dependencyResult, this);
  }

  /// Convert the target to a JSON structure appropriate for consumption by
  /// external systems.
  ///
  /// This requires constants from the [Environment] to resolve the paths of
  /// inputs and the output stamp.
  Map<String, Object> toJson(Environment environment) {
    final String? key = buildKey;
    return <String, Object>{
      'name': name,
      'dependencies': <String>[for (final Target target in dependencies) target.name],
      'inputs': <String>[for (final File file in resolveInputs(environment).sources) file.path],
      'outputs': <String>[for (final File file in resolveOutputs(environment).sources) file.path],
      if (key != null) 'buildKey': key,
      'stamp': _findStampFile(environment).absolute.path,
    };
  }

  /// Locate the stamp file for a particular target name and environment.
  File _findStampFile(Environment environment) {
    final String fileName = '$name.stamp';
    return environment.buildDir.childFile(fileName);
  }

  static ResolvedFiles _resolveConfiguration(
    List<Source> config,
    List<String> depfiles,
    Environment environment, {
    bool inputs = true,
  }) {
    final SourceVisitor collector = SourceVisitor(environment, inputs);
    for (final Source source in config) {
      source.accept(collector);
    }
    depfiles.forEach(collector.visitDepfile);
    return collector;
  }
}

/// Target that contains multiple other targets.
///
/// This target does not do anything in its own [build]
/// and acts as a wrapper around multiple other targets.
class CompositeTarget extends Target {
  CompositeTarget(this.dependencies);

  @override
  final List<Target> dependencies;

  @override
  String get name => '_composite';

  @override
  Future<void> build(Environment environment) async {}

  @override
  List<Source> get inputs => <Source>[];

  @override
  List<Source> get outputs => <Source>[];
}

/// The [Environment] defines several constants for use during the build.
///
/// The environment contains configuration and file paths that are safe to
/// depend on and reference during the build.
///
/// Example (Good):
///
/// Use the environment to determine where to write an output file.
///
/// ```dart
///    environment.buildDir.childFile('output')
///      ..createSync()
///      ..writeAsStringSync('output data');
/// ```
///
/// Example (Bad):
///
/// Use a hard-coded path or directory relative to the current working
/// directory to write an output file.
///
/// ```dart
///   globals.fs.file('build/linux/out')
///     ..createSync()
///     ..writeAsStringSync('output data');
/// ```
///
/// Example (Good):
///
/// Using the build mode to produce different output. The action
/// is still responsible for outputting a different file, as defined by the
/// corresponding output [Source].
///
/// ```dart
///    final BuildMode buildMode = getBuildModeFromDefines(environment.defines);
///    if (buildMode == BuildMode.debug) {
///      environment.buildDir.childFile('debug.output')
///        ..createSync()
///        ..writeAsStringSync('debug');
///    } else {
///      environment.buildDir.childFile('non_debug.output')
///        ..createSync()
///        ..writeAsStringSync('non_debug');
///    }
/// ```
class Environment {
  /// Create a new [Environment] object.
  ///
  /// [engineVersion] should be set to null for local engine builds.
  factory Environment({
    required Directory projectDir,
    required String packageConfigPath,
    required Directory outputDir,
    required Directory cacheDir,
    required Directory flutterRootDir,
    required FileSystem fileSystem,
    required Logger logger,
    required Artifacts artifacts,
    required ProcessManager processManager,
    required Platform platform,
    required Analytics analytics,
    String? engineVersion,
    required bool generateDartPluginRegistry,
    Directory? buildDir,
    Map<String, String> defines = const <String, String>{},
    Map<String, String> inputs = const <String, String>{},
  }) {
    // Compute a unique hash of this build's particular environment.
    // Sort the keys by key so that the result is stable. We always
    // include the engine and dart versions.
    String buildPrefix;
    final List<String> keys = defines.keys.toList()..sort();
    final StringBuffer buffer = StringBuffer();
    // The engine revision is `null` for local or custom engines.
    if (engineVersion != null) {
      buffer.write(engineVersion);
    }
    for (final String key in keys) {
      buffer.write(key);
      buffer.write(defines[key]);
    }
    buffer.write(outputDir.path);
    final String output = buffer.toString();
    final Digest digest = md5.convert(utf8.encode(output));
    buildPrefix = hex.encode(digest.bytes);

    final Directory rootBuildDir = buildDir ?? projectDir.childDirectory('build');
    final Directory buildDirectory = rootBuildDir.childDirectory(buildPrefix);
    return Environment._(
      outputDir: outputDir,
      projectDir: projectDir,
      packageConfigPath: packageConfigPath,
      buildDir: buildDirectory,
      rootBuildDir: rootBuildDir,
      cacheDir: cacheDir,
      defines: defines,
      flutterRootDir: flutterRootDir,
      fileSystem: fileSystem,
      logger: logger,
      artifacts: artifacts,
      processManager: processManager,
      platform: platform,
      analytics: analytics,
      engineVersion: engineVersion,
      inputs: inputs,
      generateDartPluginRegistry: generateDartPluginRegistry,
    );
  }

  /// Create a new [Environment] object for unit testing.
  ///
  /// Any directories not provided will fallback to a [testDirectory]
  @visibleForTesting
  factory Environment.test(
    Directory testDirectory, {
    Directory? projectDir,
    String? packageConfigPath,
    Directory? outputDir,
    Directory? cacheDir,
    Directory? flutterRootDir,
    Directory? buildDir,
    Map<String, String> defines = const <String, String>{},
    Map<String, String> inputs = const <String, String>{},
    String? engineVersion,
    Platform? platform,
    Analytics? analytics,
    bool generateDartPluginRegistry = false,
    required FileSystem fileSystem,
    required Logger logger,
    required Artifacts artifacts,
    required ProcessManager processManager,
  }) {
    return Environment(
      projectDir: projectDir ?? testDirectory,
      packageConfigPath: packageConfigPath ?? '.dart_tool/package_config.json',
      outputDir: outputDir ?? testDirectory,
      cacheDir: cacheDir ?? testDirectory,
      flutterRootDir: flutterRootDir ?? testDirectory,
      buildDir: buildDir,
      defines: defines,
      inputs: inputs,
      fileSystem: fileSystem,
      logger: logger,
      artifacts: artifacts,
      processManager: processManager,
      platform: platform ?? FakePlatform(),
      analytics: analytics ?? const NoOpAnalytics(),
      engineVersion: engineVersion,
      generateDartPluginRegistry: generateDartPluginRegistry,
    );
  }

  Environment._({
    required this.outputDir,
    required this.projectDir,
    required this.packageConfigPath,
    required this.buildDir,
    required this.rootBuildDir,
    required this.cacheDir,
    required this.defines,
    required this.flutterRootDir,
    required this.processManager,
    required this.platform,
    required this.logger,
    required this.fileSystem,
    required this.artifacts,
    required this.analytics,
    this.engineVersion,
    required this.inputs,
    required this.generateDartPluginRegistry,
  });

  /// The [Source] value which is substituted with the path to [projectDir].
  static const String kProjectDirectory = '{PROJECT_DIR}';

  /// The [Source] value which is substituted with the path to the directory
  /// that contains `.dart_tool/package_config.json1`.
  /// That is the grand-parent of [BuildInfo.packageConfigPath].
  static const String kWorkspaceDirectory = '{WORKSPACE_DIR}';

  /// The [Source] value which is substituted with the path to [buildDir].
  static const String kBuildDirectory = '{BUILD_DIR}';

  /// The [Source] value which is substituted with the path to [cacheDir].
  static const String kCacheDirectory = '{CACHE_DIR}';

  /// The [Source] value which is substituted with a path to the flutter root.
  static const String kFlutterRootDirectory = '{FLUTTER_ROOT}';

  /// The [Source] value which is substituted with a path to [outputDir].
  static const String kOutputDirectory = '{OUTPUT_DIR}';

  /// The `PROJECT_DIR` environment variable.
  ///
  /// This should be root of the flutter project where a pubspec and dart files
  /// can be located.
  final Directory projectDir;

  /// The path to the package configuration file to use for compilation.
  ///
  /// This is used by package:package_config to locate the actual package_config.json
  /// file. If not provided in tests, defaults to `.dart_tool/package_config.json`.
  final String packageConfigPath;

  /// The `BUILD_DIR` environment variable.
  ///
  /// The root of the output directory where build step intermediates and
  /// outputs are written. Current usages of assemble configure this to be
  /// a unique directory under `.dart_tool/flutter_build`, though it can
  /// be placed anywhere. The uniqueness is only enforced by callers, and
  /// is currently done by hashing the build configuration.
  final Directory buildDir;

  /// The `CACHE_DIR` environment variable.
  ///
  /// Defaults to `{FLUTTER_ROOT}/bin/cache`. The root of the artifact cache for
  /// the flutter tool.
  final Directory cacheDir;

  /// The `FLUTTER_ROOT` environment variable.
  ///
  /// Defaults to the value of [Cache.flutterRoot].
  final Directory flutterRootDir;

  /// The `OUTPUT_DIR` environment variable.
  ///
  /// Must be provided to configure the output location for the final artifacts.
  final Directory outputDir;

  /// Additional configuration passed to the build targets.
  ///
  /// Setting values here forces a unique build directory to be chosen
  /// which prevents the config from leaking into different builds.
  final Map<String, String> defines;

  /// Additional input files passed to the build targets.
  ///
  /// Unlike [defines], values set here do not force a new build configuration.
  /// This is useful for passing file inputs that may have changing paths
  /// without running builds from scratch.
  ///
  /// It is the responsibility of the [Target] to declare that an input was
  /// used in an output depfile.
  final Map<String, String> inputs;

  /// The root build directory shared by all builds.
  final Directory rootBuildDir;

  final ProcessManager processManager;

  final Platform platform;

  final Logger logger;

  final Artifacts artifacts;

  final FileSystem fileSystem;

  final Analytics analytics;

  /// The version of the current engine, or `null` if built with a local engine.
  final String? engineVersion;

  /// Whether to generate the Dart plugin registry.
  /// When [true], the main entrypoint is wrapped and the wrapper becomes
  /// the new entrypoint.
  final bool generateDartPluginRegistry;

  late final DepfileService depFileService = DepfileService(logger: logger, fileSystem: fileSystem);
}

/// The result information from the build system.
class BuildResult {
  BuildResult({
    required this.success,
    this.exceptions = const <String, ExceptionMeasurement>{},
    this.performance = const <String, PerformanceMeasurement>{},
    this.inputFiles = const <File>[],
    this.outputFiles = const <File>[],
  });

  final bool success;
  final Map<String, ExceptionMeasurement> exceptions;
  final Map<String, PerformanceMeasurement> performance;
  final List<File> inputFiles;
  final List<File> outputFiles;

  bool get hasException => exceptions.isNotEmpty;
}

/// The build system is responsible for invoking and ordering [Target]s.
abstract class BuildSystem {
  /// Const constructor to allow subclasses to be const.
  const BuildSystem();

  /// Build [target] and all of its dependencies.
  Future<BuildResult> build(
    Target target,
    Environment environment, {
    BuildSystemConfig buildSystemConfig = const BuildSystemConfig(),
  });

  /// Perform an incremental build of [target] and all of its dependencies.
  ///
  /// If [previousBuild] is not provided, a new incremental build is
  /// initialized.
  Future<BuildResult> buildIncremental(
    Target target,
    Environment environment,
    BuildResult? previousBuild,
  );
}

class FlutterBuildSystem extends BuildSystem {
  const FlutterBuildSystem({
    required FileSystem fileSystem,
    required Platform platform,
    required Logger logger,
  }) : _fileSystem = fileSystem,
       _platform = platform,
       _logger = logger;

  final FileSystem _fileSystem;
  final Platform _platform;
  final Logger _logger;

  @override
  Future<BuildResult> build(
    Target target,
    Environment environment, {
    BuildSystemConfig buildSystemConfig = const BuildSystemConfig(),
  }) async {
    environment.buildDir.createSync(recursive: true);
    environment.outputDir.createSync(recursive: true);

    // Load file store from previous builds.
    final File cacheFile = environment.buildDir.childFile(FileStore.kFileCache);
    final FileStore fileCache = FileStore(cacheFile: cacheFile, logger: _logger)..initialize();

    // Perform sanity checks on build.
    checkCycles(target);

    final Node node = target._toNode(environment);
    final _BuildInstance buildInstance = _BuildInstance(
      environment: environment,
      fileCache: fileCache,
      buildSystemConfig: buildSystemConfig,
      logger: _logger,
      fileSystem: _fileSystem,
      platform: _platform,
    );
    bool passed = true;
    try {
      passed = await buildInstance.invokeTarget(node);
    } finally {
      // Always persist the file cache to disk.
      fileCache.persist();
    }
    // This is a bit of a hack, due to various parts of
    // the flutter tool writing these files unconditionally. Since Xcode uses
    // timestamps to track files, this leads to unnecessary rebuilds if they
    // are included. Once all the places that write these files have been
    // tracked down and moved into assemble, these checks should be removable.
    // We also remove files under .dart_tool, since these are intermediaries
    // and don't need to be tracked by external systems.
    {
      bool isUnconditionalFile(String path) {
        return switch (_fileSystem.path.basename(path)) {
          '.flutter-plugins' || '.flutter-plugins-dependencies' => true,
          _ when _fileSystem.path.extension(path) == '.xcconfig' => true,
          _ when _fileSystem.path.split(path).contains('.dart_tool') => true,
          _ => false,
        };
      }

      buildInstance.inputFiles.removeWhere((String path, File file) {
        return isUnconditionalFile(path);
      });
      buildInstance.outputFiles.removeWhere((String path, File file) {
        return isUnconditionalFile(path);
      });
    }
    trackSharedBuildDirectory(environment, _fileSystem, buildInstance.outputFiles);
    environment.buildDir
        .childFile('outputs.json')
        .writeAsStringSync(json.encode(buildInstance.outputFiles.keys.toList()));

    return BuildResult(
      success: passed,
      exceptions: buildInstance.exceptionMeasurements,
      performance: buildInstance.stepTimings,
      inputFiles:
          buildInstance.inputFiles.values.toList()
            ..sort((File a, File b) => a.path.compareTo(b.path)),
      outputFiles:
          buildInstance.outputFiles.values.toList()
            ..sort((File a, File b) => a.path.compareTo(b.path)),
    );
  }

  static final Expando<FileStore> _incrementalFileStore = Expando<FileStore>();

  @override
  Future<BuildResult> buildIncremental(
    Target target,
    Environment environment,
    BuildResult? previousBuild,
  ) async {
    environment.buildDir.createSync(recursive: true);
    environment.outputDir.createSync(recursive: true);

    FileStore? fileCache;
    if (previousBuild == null || _incrementalFileStore[previousBuild] == null) {
      final File cacheFile = environment.buildDir.childFile(FileStore.kFileCache);
      fileCache = FileStore(
        cacheFile: cacheFile,
        logger: _logger,
        strategy: FileStoreStrategy.timestamp,
      )..initialize();
    } else {
      fileCache = _incrementalFileStore[previousBuild];
    }
    final Node node = target._toNode(environment);
    final _BuildInstance buildInstance = _BuildInstance(
      environment: environment,
      fileCache: fileCache!,
      buildSystemConfig: const BuildSystemConfig(),
      logger: _logger,
      fileSystem: _fileSystem,
      platform: _platform,
    );
    bool passed = true;
    try {
      passed = await buildInstance.invokeTarget(node);
    } finally {
      fileCache.persistIncremental();
    }
    final BuildResult result = BuildResult(
      success: passed,
      exceptions: buildInstance.exceptionMeasurements,
      performance: buildInstance.stepTimings,
    );
    _incrementalFileStore[result] = fileCache;
    return result;
  }

  /// Write the identifier of the last build into the output directory and
  /// remove the previous build's output.
  ///
  /// The build identifier is the basename of the build directory where
  /// outputs and intermediaries are written, under `.dart_tool/flutter_build`.
  /// This is computed from a hash of the build's configuration.
  ///
  /// This identifier is used to perform a targeted cleanup of the last output
  /// files, if these were not already covered by the built-in cleanup. This
  /// cleanup is only necessary when multiple different build configurations
  /// output to the same directory.
  @visibleForTesting
  void trackSharedBuildDirectory(
    Environment environment,
    FileSystem fileSystem,
    Map<String, File> currentOutputs,
  ) {
    if (environment.defines[kXcodePreAction] == 'PrepareFramework') {
      // If the current build is the PrepareFramework Xcode pre-action, skip
      // updating the last build identifier and cleaning up the previous build
      // since this build is not a complete build.
      return;
    }

    final String currentBuildId = fileSystem.path.basename(environment.buildDir.path);
    final File lastBuildIdFile = environment.outputDir.childFile('.last_build_id');
    if (!lastBuildIdFile.existsSync()) {
      lastBuildIdFile.parent.createSync(recursive: true);
      lastBuildIdFile.writeAsStringSync(currentBuildId);
      // No config file, either output was cleaned or this is the first build.
      return;
    }
    final String lastBuildId = lastBuildIdFile.readAsStringSync().trim();
    if (lastBuildId == currentBuildId) {
      // The last build was the same configuration as the current build
      return;
    }
    // Update the output dir with the latest config.
    lastBuildIdFile
      ..createSync()
      ..writeAsStringSync(currentBuildId);
    final File outputsFile = environment.buildDir.parent
        .childDirectory(lastBuildId)
        .childFile('outputs.json');

    if (!outputsFile.existsSync()) {
      // There is no output list. This could happen if the user manually
      // edited .last_config or deleted .dart_tool.
      return;
    }
    final List<String> lastOutputs =
        (json.decode(outputsFile.readAsStringSync()) as List<Object?>).cast<String>();
    for (final String lastOutput in lastOutputs) {
      if (!currentOutputs.containsKey(lastOutput)) {
        final File lastOutputFile = fileSystem.file(lastOutput);
        ErrorHandlingFileSystem.deleteIfExists(lastOutputFile);
      }
    }
  }
}

/// An active instance of a build.
class _BuildInstance {
  _BuildInstance({
    required this.environment,
    required this.fileCache,
    required this.buildSystemConfig,
    required this.logger,
    required this.fileSystem,
    Platform? platform,
  }) : resourcePool = Pool(buildSystemConfig.resourcePoolSize ?? platform?.numberOfProcessors ?? 1);

  final Logger logger;
  final FileSystem fileSystem;
  final BuildSystemConfig buildSystemConfig;
  final Pool resourcePool;
  final Map<String, AsyncMemoizer<bool>> pending = <String, AsyncMemoizer<bool>>{};
  final Environment environment;
  final FileStore fileCache;
  final Map<String, File> inputFiles = <String, File>{};
  final Map<String, File> outputFiles = <String, File>{};

  // Timings collected during target invocation.
  final Map<String, PerformanceMeasurement> stepTimings = <String, PerformanceMeasurement>{};

  // Exceptions caught during the build process.
  final Map<String, ExceptionMeasurement> exceptionMeasurements = <String, ExceptionMeasurement>{};

  Future<bool> invokeTarget(Node node) async {
    final List<bool> results = await Future.wait(node.dependencies.map(invokeTarget));
    if (results.any((bool result) => !result)) {
      return false;
    }
    final AsyncMemoizer<bool> memoizer = pending[node.target.name] ??= AsyncMemoizer<bool>();
    return memoizer.runOnce(() => _invokeInternal(node));
  }

  Future<bool> _invokeInternal(Node node) async {
    final PoolResource resource = await resourcePool.request();
    final Stopwatch stopwatch = Stopwatch()..start();
    bool succeeded = true;
    bool skipped = false;

    // The build system produces a list of aggregate input and output
    // files for the overall build. This list is provided to a hosting build
    // system, such as Xcode, to configure logic for when to skip the
    // rule/phase which contains the flutter build.
    //
    // When looking at the inputs and outputs for the individual rules, we need
    // to be careful to remove inputs that were actually output from previous
    // build steps. This indicates that the file is an intermediary. If
    // these files are included as both inputs and outputs then it isn't
    // possible to construct a DAG describing the build.
    void updateGraph() {
      for (final File output in node.outputs) {
        outputFiles[output.path] = output;
      }
      for (final File input in node.inputs) {
        final String resolvedPath = input.absolute.path;
        if (outputFiles.containsKey(resolvedPath)) {
          continue;
        }
        inputFiles[resolvedPath] = input;
      }
    }

    try {
      // If we're missing a depfile, wait until after evaluating the target to
      // compute changes.
      final bool canSkip =
          !node.missingDepfile && node.computeChanges(environment, fileCache, fileSystem, logger);

      if (canSkip) {
        skipped = true;
        logger.printTrace('Skipping target: ${node.target.name}');
        updateGraph();
        return succeeded;
      }
      // Clear old inputs. These will be replaced with new inputs/outputs
      // after the target is run. In the case of a runtime skip, each list
      // must be empty to ensure the previous outputs are purged.
      node.inputs.clear();
      node.outputs.clear();

      // Check if we can skip via runtime dependencies.
      final bool runtimeSkip = node.target.canSkip(environment);
      if (runtimeSkip) {
        logger.printTrace('Skipping target: ${node.target.name}');
        skipped = true;
      } else {
        logger.printTrace('${node.target.name}: Starting due to ${node.invalidatedReasons}');
        await node.target.build(environment);
        logger.printTrace('${node.target.name}: Complete');
        node.inputs.addAll(node.target.resolveInputs(environment).sources);
        node.outputs.addAll(node.target.resolveOutputs(environment).sources);
      }

      // If we were missing the depfile, resolve input files after executing the
      // target so that all file hashes are up to date on the next run.
      if (node.missingDepfile) {
        fileCache.diffFileList(node.inputs);
      }

      // Always update hashes for output files.
      fileCache.diffFileList(node.outputs);
      node.target._writeStamp(node.inputs, node.outputs, environment);
      updateGraph();

      // Delete outputs from previous stages that are no longer a part of the
      // build.
      for (final String previousOutput in node.previousOutputs) {
        if (outputFiles.containsKey(previousOutput)) {
          continue;
        }
        final File previousFile = fileSystem.file(previousOutput);
        ErrorHandlingFileSystem.deleteIfExists(previousFile);
      }
    } on Exception catch (exception, stackTrace) {
      node.target.clearStamp(environment);
      succeeded = false;
      skipped = false;
      exceptionMeasurements[node.target.name] = ExceptionMeasurement(
        node.target.name,
        exception,
        stackTrace,
        fatal: true,
      );
    } finally {
      resource.release();
      stopwatch.stop();
      stepTimings[node.target.name] = PerformanceMeasurement(
        target: node.target.name,
        elapsedMilliseconds: stopwatch.elapsedMilliseconds,
        skipped: skipped,
        succeeded: succeeded,
        analyticsName: node.target.analyticsName,
      );
    }
    return succeeded;
  }
}

/// Helper class to collect exceptions.
class ExceptionMeasurement {
  ExceptionMeasurement(this.target, this.exception, this.stackTrace, {this.fatal = false});

  final String target;
  final Object? exception;
  final StackTrace stackTrace;

  /// Whether this exception was a fatal build system error.
  final bool fatal;

  @override
  String toString() => 'target: $target\nexception:$exception\n$stackTrace';
}

/// Helper class to collect measurement data.
class PerformanceMeasurement {
  PerformanceMeasurement({
    required this.target,
    required this.elapsedMilliseconds,
    required this.skipped,
    required this.succeeded,
    required this.analyticsName,
  });

  final int elapsedMilliseconds;
  final String target;
  final bool skipped;
  final bool succeeded;
  final String analyticsName;
}

/// Check if there are any dependency cycles in the target.
///
/// Throws a [CycleException] if one is encountered.
void checkCycles(Target initial) {
  void checkInternal(Target target, Set<Target> visited, Set<Target> stack) {
    if (stack.contains(target)) {
      throw CycleException(stack..add(target));
    }
    if (visited.contains(target)) {
      return;
    }
    visited.add(target);
    stack.add(target);
    for (final Target dependency in target.dependencies) {
      checkInternal(dependency, visited, stack);
    }
    stack.remove(target);
  }

  checkInternal(initial, <Target>{}, <Target>{});
}

/// Verifies that all files exist and are in a subdirectory of [Environment.buildDir].
void verifyOutputDirectories(List<File> outputs, Environment environment, Target target) {
  final String buildDirectory = environment.buildDir.resolveSymbolicLinksSync();
  final String projectDirectory = environment.projectDir.resolveSymbolicLinksSync();
  final List<File> missingOutputs = <File>[];
  for (final File sourceFile in outputs) {
    if (!sourceFile.existsSync()) {
      missingOutputs.add(sourceFile);
      continue;
    }
    final String path = sourceFile.path;
    if (!path.startsWith(buildDirectory) && !path.startsWith(projectDirectory)) {
      throw MisplacedOutputException(path, target.name);
    }
  }
  if (missingOutputs.isNotEmpty) {
    throw MissingOutputException(missingOutputs, target.name);
  }
}

/// A node in the build graph.
class Node {
  factory Node(
    Target target,
    List<File> inputs,
    List<File> outputs,
    List<Node> dependencies,
    String? buildKey,
    Environment environment,
    bool missingDepfile,
  ) {
    final File stamp = target._findStampFile(environment);
    Map<String, Object?>? stampValues;

    // If the stamp file doesn't exist, we haven't run this step before and
    // all inputs were added.
    if (stamp.existsSync()) {
      final String content = stamp.readAsStringSync();
      if (content.isEmpty) {
        stamp.deleteSync();
      } else {
        try {
          stampValues = castStringKeyedMap(json.decode(content));
        } on FormatException {
          // The json is malformed in some way.
        }
      }
    }
    if (stampValues != null) {
      final String? previousBuildKey = stampValues['buildKey'] as String?;
      final Object? stampInputs = stampValues['inputs'];
      final Object? stampOutputs = stampValues['outputs'];
      if (stampInputs is List<Object?> && stampOutputs is List<Object?>) {
        final Set<String> previousInputs = stampInputs.whereType<String>().toSet();
        final Set<String> previousOutputs = stampOutputs.whereType<String>().toSet();
        return Node.withStamp(
          target,
          inputs,
          previousInputs,
          outputs,
          previousOutputs,
          dependencies,
          buildKey,
          previousBuildKey,
          missingDepfile,
        );
      }
    }
    return Node.withNoStamp(target, inputs, outputs, dependencies, buildKey, missingDepfile);
  }

  Node.withNoStamp(
    this.target,
    this.inputs,
    this.outputs,
    this.dependencies,
    this.buildKey,
    this.missingDepfile,
  ) : previousInputs = <String>{},
      previousOutputs = <String>{},
      previousBuildKey = null,
      _dirty = true;

  Node.withStamp(
    this.target,
    this.inputs,
    this.previousInputs,
    this.outputs,
    this.previousOutputs,
    this.dependencies,
    this.buildKey,
    this.previousBuildKey,
    this.missingDepfile,
  ) : _dirty = false;

  /// The resolved input files.
  ///
  /// These files may not yet exist if they are produced by previous steps.
  final List<File> inputs;

  /// The resolved output files.
  ///
  /// These files may not yet exist if the target hasn't run yet.
  final List<File> outputs;

  /// The current build key of the target
  ///
  /// See `buildKey` in the `Target` class for more information.
  final String? buildKey;

  /// Whether this node is missing a depfile.
  ///
  /// This requires an additional pass of source resolution after the target
  /// has been executed.
  final bool missingDepfile;

  /// The target definition which contains the build action to invoke.
  final Target target;

  /// All of the nodes that this one depends on.
  final List<Node> dependencies;

  /// Output file paths from the previous invocation of this build node.
  final Set<String> previousOutputs;

  /// Input file paths from the previous invocation of this build node.
  final Set<String> previousInputs;

  /// The buildKey from the previous invocation of this build node.
  ///
  /// See `buildKey` in the `Target` class for more information.
  final String? previousBuildKey;

  /// One or more reasons why a task was invalidated.
  ///
  /// May be empty if the task was skipped.
  final Map<InvalidatedReasonKind, InvalidatedReason> invalidatedReasons =
      <InvalidatedReasonKind, InvalidatedReason>{};

  /// Whether this node needs an action performed.
  bool get dirty => _dirty;
  bool _dirty = false;

  InvalidatedReason _invalidate(InvalidatedReasonKind kind) {
    return invalidatedReasons[kind] ??= InvalidatedReason(kind);
  }

  /// Collect hashes for all inputs to determine if any have changed.
  ///
  /// Returns whether this target can be skipped.
  bool computeChanges(
    Environment environment,
    FileStore fileStore,
    FileSystem fileSystem,
    Logger logger,
  ) {
    if (buildKey != previousBuildKey) {
      _invalidate(InvalidatedReasonKind.buildKeyChanged);
      _dirty = true;
    }
    final Set<String> currentOutputPaths = <String>{for (final File file in outputs) file.path};
    // For each input, first determine if we've already computed the key
    // for it. Then collect it to be sent off for diffing as a group.
    final List<File> sourcesToDiff = <File>[];
    final List<File> missingInputs = <File>[];
    for (final File file in inputs) {
      if (!file.existsSync()) {
        missingInputs.add(file);
        continue;
      }

      final String absolutePath = file.path;
      final String? previousAssetKey = fileStore.previousAssetKeys[absolutePath];
      if (fileStore.currentAssetKeys.containsKey(absolutePath)) {
        final String? currentHash = fileStore.currentAssetKeys[absolutePath];
        if (currentHash != previousAssetKey) {
          final InvalidatedReason reason = _invalidate(InvalidatedReasonKind.inputChanged);
          reason.data.add(absolutePath);
          _dirty = true;
        }
      } else {
        sourcesToDiff.add(file);
      }
    }

    // For each output, first determine if we've already computed the key
    // for it. Then collect it to be sent off for hashing as a group.
    for (final String previousOutput in previousOutputs) {
      // output paths changed.
      if (!currentOutputPaths.contains(previousOutput)) {
        _dirty = true;
        final InvalidatedReason reason = _invalidate(InvalidatedReasonKind.outputSetChanged);
        reason.data.add(previousOutput);
        // if this isn't a current output file there is no reason to compute the key.
        continue;
      }
      final File file = fileSystem.file(previousOutput);
      if (!file.existsSync()) {
        final InvalidatedReason reason = _invalidate(InvalidatedReasonKind.outputMissing);
        reason.data.add(file.path);
        _dirty = true;
        continue;
      }
      final String absolutePath = file.path;
      final String? previousHash = fileStore.previousAssetKeys[absolutePath];
      if (fileStore.currentAssetKeys.containsKey(absolutePath)) {
        final String? currentHash = fileStore.currentAssetKeys[absolutePath];
        if (currentHash != previousHash) {
          final InvalidatedReason reason = _invalidate(InvalidatedReasonKind.outputChanged);
          reason.data.add(absolutePath);
          _dirty = true;
        }
      } else {
        sourcesToDiff.add(file);
      }
    }

    // If we depend on a file that doesn't exist on disk, mark the build as
    // dirty. if the rule is not correctly specified, this will result in it
    // always being rerun.
    if (missingInputs.isNotEmpty) {
      _dirty = true;
      final String missingMessage = missingInputs.map((File file) => file.path).join(', ');
      logger.printTrace('invalidated build due to missing files: $missingMessage');
      final InvalidatedReason reason = _invalidate(InvalidatedReasonKind.inputMissing);
      reason.data.addAll(missingInputs.map((File file) => file.path));
    }

    // If we have files to diff, compute them asynchronously and then
    // update the result.
    if (sourcesToDiff.isNotEmpty) {
      final List<File> dirty = fileStore.diffFileList(sourcesToDiff);
      if (dirty.isNotEmpty) {
        final InvalidatedReason reason = _invalidate(InvalidatedReasonKind.inputChanged);
        reason.data.addAll(dirty.map((File file) => file.path));
        _dirty = true;
      }
    }
    return !_dirty;
  }
}

/// Data about why a target was re-run.
class InvalidatedReason {
  InvalidatedReason(this.kind);

  final InvalidatedReasonKind kind;

  /// Absolute file paths of inputs or outputs, depending on [kind].
  final List<String> data = <String>[];

  @override
  String toString() {
    return switch (kind) {
      InvalidatedReasonKind.inputMissing => 'The following inputs were missing: ${data.join(',')}',
      InvalidatedReasonKind.inputChanged =>
        'The following inputs have updated contents: ${data.join(',')}',
      InvalidatedReasonKind.outputChanged =>
        'The following outputs have updated contents: ${data.join(',')}',
      InvalidatedReasonKind.outputMissing =>
        'The following outputs were missing: ${data.join(',')}',
      InvalidatedReasonKind.outputSetChanged =>
        'The following outputs were removed from the output set: ${data.join(',')}',
      InvalidatedReasonKind.buildKeyChanged => 'The target build key changed.',
    };
  }
}

/// A description of why a target was rerun.
enum InvalidatedReasonKind {
  /// An input file that was expected is missing. This can occur when using
  /// depfile dependencies, or if a target is incorrectly specified.
  inputMissing,

  /// An input file has an updated key.
  inputChanged,

  /// An output file has an updated key.
  outputChanged,

  /// An output file that is expected is missing.
  outputMissing,

  /// The set of expected output files changed.
  outputSetChanged,

  /// The build key changed
  buildKeyChanged,
}
