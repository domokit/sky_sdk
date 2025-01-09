// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:engine_build_configs/engine_build_configs.dart';

/// Builds a test [BuilderConfig].
///
/// Many tests will involve exactly one build configuration, or a small number
/// of build configurations. Instead of constructing these configurations ahead
/// of time, this builder is used to create them on-the-fly, with convenient
/// methods for setting up and cloning configurations.
///
/// This builder exists in order to avoid global fixtures in tests that do not
/// isolate elements of the test environment relevant to their test. Prior to
/// the builder, 100s of lines of static configuration were used to setup and
/// instrument tests across multiple files; instead, this builder is used to
/// precisely configure the test environment for each test.
///
/// See <https://github.com/flutter/flutter/issues/148420> for more information.
final class TestBuilderConfig {
  final _builds = <Map<String, Object?>>[];

  /// Appends a build to the configuration.
  void addBuild({
    required String name,
    required TestDroneDimension dimension,
    bool enableRbe = false,
    bool? enableLto,
    String description = 'A default description.',
    String? targetDir,
    (String, List<String>)? generatorTask,
    (String, List<String>)? testTask,
  }) {
    _builds.add({
      'archives': [],
      'drone_dimensions': [dimension._dimension],
      'gclient_variables': <String, Object?>{},
      'gn': [if (enableRbe) '--rbe', if (enableLto == false) '--no-lto'],
      'name': name,
      'description': description,
      'ninja': <String, Object?>{
        if (targetDir case final targetDir?) ...{
          'config': targetDir,
          'targets': ['ninja_target'],
        },
      },
      'tests': _testTask(testTask),
      'generators': _generatorTask(generatorTask),
    });
  }

  static List<Object?> _testTask((String, List<String>)? task) {
    if (task == null) {
      return [];
    }
    final (script, args) = task;
    return [
      {
        'name': 'test_task',
        'language': 'python',
        'scripts': [script],
        'parameters': args,
        'contexts': ['context'],
      },
    ];
  }

  static Map<String, Object?> _generatorTask((String, List<String>)? task) {
    if (task == null) {
      return {};
    }
    final (script, args) = task;
    return {
      'tasks': [
        {
          'name': 'generator_task',
          'language': 'python',
          'scripts': [script],
          'parameters': args,
        },
      ],
    };
  }

  /// Copies the state of `this` as a new [TestBuilderConfig].
  TestBuilderConfig clone() {
    final clone = TestBuilderConfig();
    clone._builds.addAll(_builds);
    return clone;
  }

  /// Creates and returns a [BuilderConfig] capturing the current builder state.
  ///
  /// [path] is the path to the configuration file that would be read from disk.
  ///
  /// After creation, the builder state remains, and changes can be made to the
  /// builder to create a new configuration without affecting the previous one
  /// created.
  BuilderConfig buildConfig({required String path}) {
    final config = BuilderConfig.fromJson(map: buildJson(), path: path);
    if (config.check(path) case final errors when errors.isNotEmpty) {
      throw StateError('Invalid configuration:\n${errors.join('\n')}');
    }
    return config;
  }

  /// Creates and returns the JSON serialized format of a [BuilderConfig].
  ///
  /// Most of the time, use [build] instead of this method.
  ///
  /// It is undefined behavior to mutate the returned map.
  Map<String, Object?> buildJson() {
    return {'builds': _builds};
  }
}

/// Fixed set of dimensions for [TestBuilderConfig.addBuild].
enum TestDroneDimension {
  /// Runs on Linux.
  linux('os=Linux'),

  /// Runs on macOS.
  mac('os=Mac-12'),

  /// Runs on Windows.
  win('os=Windows-11');

  const TestDroneDimension(this._dimension);
  final String _dimension;
}
