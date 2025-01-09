// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// A minimal library for discovering and probing a local engine repository.
///
/// This library is intended to be used by tools that need to interact with a
/// local engine repository, such as `clang_tidy` or `githooks`. For example,
/// finding the `compile_commands.json` file for the most recently built output:
///
/// ```dart
/// final Engine engine = Engine.findWithin();
/// final Output? output = engine.latestOutput();
/// if (output == null) {
///   print('No output targets found.');
/// } else {
///   final io.File? compileCommandsJson = output.compileCommandsJson;
///   if (compileCommandsJson == null) {
///     print('No compile_commands.json file found.');
///   } else {
///     print('Found compile_commands.json file at ${compileCommandsJson.path}');
///   }
/// }
/// ```
library;

import 'dart:io' as io;

import 'package:path/path.dart' as p;

/// Represents the `$ENGINE` directory (i.e. a checked-out Flutter engine).
///
/// If you have a path to the `$ENGINE/src` directory, use [Engine.fromSrcPath].
///
/// If you have a path to a directory within the `$ENGINE/src` directory, or
/// want to use the current working directory, use [Engine.findWithin].
final class Engine {
  const Engine._({required this.srcDir, required this.flutterDir, required this.outDir});

  /// Creates an [Engine] from a path such as `/Users/.../flutter/engine/src`.
  ///
  /// ```dart
  /// final Engine engine = Engine.findWithin('/Users/.../engine/src');
  /// print(engine.srcDir.path); // /Users/.../engine/src
  /// ```
  ///
  /// Throws a [InvalidEngineException] if the path is not a valid engine root.
  factory Engine.fromSrcPath(String srcPath) {
    // If the path does not end in `/src`, fail.
    if (p.basename(srcPath) != 'src') {
      throw InvalidEngineException.doesNotEndWithSrc(srcPath);
    }

    // If the directory does not exist, or is not a directory, fail.
    final io.Directory srcDir = io.Directory(srcPath);
    if (!srcDir.existsSync()) {
      throw InvalidEngineException.notADirectory(srcPath);
    }

    // Check for the existence of a `flutter` directory within `src`.
    final io.Directory flutterDir = io.Directory(p.join(srcPath, 'flutter'));
    if (!flutterDir.existsSync()) {
      throw InvalidEngineException.missingFlutterDirectory(srcPath);
    }

    // We do **NOT** check for the existence of a `out` directory within `src`,
    // it's not required to exist (i.e. a new checkout of the engine), and we
    // don't want to fail if it doesn't exist.
    final io.Directory outDir = io.Directory(p.join(srcPath, 'out'));

    return Engine._(srcDir: srcDir, flutterDir: flutterDir, outDir: outDir);
  }

  /// Creates an [Engine] by looking for a `src/` directory in the given path.
  ///
  /// Similar to [tryFindWithin], but throws a [StateError] if the path is not
  /// within a valid engine. This is useful for tools that require an engine
  /// and do not have a reasonable fallback or recovery path.
  factory Engine.findWithin([String? path]) {
    final Engine? engine = tryFindWithin(path);
    if (engine == null) {
      throw StateError('The path "$path" is not within a valid engine.');
    }
    return engine;
  }

  /// Creates an [Engine] by looking for a `src/` directory in the given [path].
  ///
  /// ```dart
  /// // Use the current working directory.
  /// final Engine engine = Engine.findWithin();
  /// print(engine.srcDir.path); // /Users/.../engine/src
  ///
  /// // Use a specific directory.
  /// final Engine engine = Engine.findWithin('/Users/.../engine/src/foo/bar');
  /// print(engine.srcDir.path); // /Users/.../engine/src
  /// ```
  ///
  /// If a [path] is not provided, the current working directory is used.
  ///
  /// If path does not exist, or is not a directory, an error is thrown.
  ///
  /// Returns `null` if the path is not within a valid engine.
  static Engine? tryFindWithin([String? path]) {
    path ??= p.current;

    // Search parent directories for a `src` directory.
    io.Directory maybeSrcDir = io.Directory(path);

    if (!maybeSrcDir.existsSync()) {
      throw StateError('The path "$path" does not exist or is not a directory.');
    }

    do {
      try {
        return Engine.fromSrcPath(maybeSrcDir.path);
      } on InvalidEngineException {
        // Ignore, we'll keep searching.
      }
      maybeSrcDir = maybeSrcDir.parent;
    } while (maybeSrcDir.parent.path != maybeSrcDir.path /* at root */ );

    return null;
  }

  /// The path to the `$ENGINE/src` directory.
  final io.Directory srcDir;

  /// The path to the `$ENGINE/src/flutter` directory.
  final io.Directory flutterDir;

  /// The path to the `$ENGINE/src/out` directory.
  ///
  /// **NOTE**: This directory may not exist.
  final io.Directory outDir;

  /// Returns a list of all output targets in [outDir].
  List<Output> outputs() {
    return outDir.listSync().whereType<io.Directory>().map<Output>(Output._).toList();
  }

  /// Returns the most recently modified output target in [outDir].
  ///
  /// If there are no output targets, returns `null`.
  Output? latestOutput() {
    final List<Output> outputs = this.outputs();
    if (outputs.isEmpty) {
      return null;
    }
    outputs.sort((Output a, Output b) {
      return b.path.statSync().modified.compareTo(a.path.statSync().modified);
    });
    return outputs.first;
  }
}

/// An implementation of [Engine] that has pre-defined outputs for testing.
final class TestEngine extends Engine {
  /// Creates a [TestEngine] with pre-defined paths.
  ///
  /// The [srcDir] and [flutterDir] must exist, but the [outDir] is optional.
  ///
  /// Optionally, provide a list of [outputs] to use, otherwise it is empty.
  TestEngine.withPaths({
    required super.srcDir,
    required super.flutterDir,
    required super.outDir,
    List<TestOutput> outputs = const <TestOutput>[],
  }) : _outputs = outputs,
       super._() {
    if (!srcDir.existsSync()) {
      throw ArgumentError.value(srcDir, 'srcDir', 'does not exist');
    }
    if (!flutterDir.existsSync()) {
      throw ArgumentError.value(flutterDir, 'flutterDir', 'does not exist');
    }
  }

  /// Creates a [TestEngine] within a temporary directory.
  ///
  /// The [rootDir] is the temporary directory that will contain the engine.
  ///
  /// Optionally, provide a list of [outputs] to use, otherwise it is empty.
  factory TestEngine.createTemp({
    required io.Directory rootDir,
    List<TestOutput> outputs = const <TestOutput>[],
  }) {
    final io.Directory srcDir = io.Directory(p.join(rootDir.path, 'src'));
    final io.Directory flutterDir = io.Directory(p.join(srcDir.path, 'flutter'));
    final io.Directory outDir = io.Directory(p.join(srcDir.path, 'out'));
    srcDir.createSync(recursive: true);
    flutterDir.createSync(recursive: true);
    outDir.createSync(recursive: true);
    return TestEngine.withPaths(
      srcDir: srcDir,
      flutterDir: flutterDir,
      outDir: outDir,
      outputs: outputs,
    );
  }

  final List<TestOutput> _outputs;

  @override
  List<Output> outputs() => List<Output>.unmodifiable(_outputs);

  @override
  Output? latestOutput() {
    if (_outputs.isEmpty) {
      return null;
    }
    _outputs.sort((TestOutput a, TestOutput b) {
      return b.lastModified.compareTo(a.lastModified);
    });
    return _outputs.first;
  }
}

/// An implementation of [Output] that has a pre-defined path for testing.
final class TestOutput extends Output {
  /// Creates a [TestOutput] with a pre-defined path.
  ///
  /// Optionally, provide a [lastModified] date.
  TestOutput(super.path, {DateTime? lastModified})
    : lastModified = lastModified ?? _defaultLastModified,
      super._();

  static final DateTime _defaultLastModified = DateTime.now();

  /// The last modified date of the output target.
  final DateTime lastModified;
}

/// Thrown when an [Engine] could not be created from a path.
sealed class InvalidEngineException implements Exception {
  /// Thrown when an [Engine] was created from a path not ending in `src`.
  factory InvalidEngineException.doesNotEndWithSrc(String path) {
    return InvalidEngineSrcPathException._(path);
  }

  /// Thrown when an [Engine] was created from a directory that does not exist.
  factory InvalidEngineException.notADirectory(String path) {
    return InvalidEngineNotADirectoryException._(path);
  }

  /// Thrown when an [Engine] was created from a path not containing `flutter/`.
  factory InvalidEngineException.missingFlutterDirectory(String path) {
    return InvalidEngineMissingFlutterDirectoryException._(path);
  }
}

/// Thrown when an [Engine] was created from a path not ending in `src`.
final class InvalidEngineSrcPathException implements InvalidEngineException {
  InvalidEngineSrcPathException._(this.path);

  /// The path that was used to create the [Engine].
  final String path;

  @override
  String toString() {
    return 'The path $path does not end in `${p.separator}src`.';
  }
}

/// Thrown when an [Engine] was created from a path that is not a directory.
final class InvalidEngineNotADirectoryException implements InvalidEngineException {
  InvalidEngineNotADirectoryException._(this.path);

  /// The path that was used to create the [Engine].
  final String path;

  @override
  String toString() {
    return 'The path "$path" does not exist or is not a directory.';
  }
}

/// Thrown when an [Engine] was created from a path not containing `flutter/`.
final class InvalidEngineMissingFlutterDirectoryException implements InvalidEngineException {
  InvalidEngineMissingFlutterDirectoryException._(this.path);

  /// The path that was used to create the [Engine].
  final String path;

  @override
  String toString() {
    return 'The path "$path" does not contain a "flutter" directory.';
  }
}

/// Represents a single output target in the `$ENGINE/src/out` directory.
final class Output {
  const Output._(this.path);

  /// The directory containing the output target.
  final io.Directory path;

  /// The `compile_commands.json` file that should exist for this output target.
  ///
  /// The file may not exist.
  io.File get compileCommandsJson {
    return io.File(p.join(path.path, 'compile_commands.json'));
  }
}
