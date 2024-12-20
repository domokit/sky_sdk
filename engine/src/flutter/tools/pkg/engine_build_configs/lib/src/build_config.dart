// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:platform/platform.dart';

// This library parses Engine builder config data out of the "Engine v2" build
// config JSON files with the format described at:
//   https://github.com/flutter/engine/blob/main/ci/builders/README.md

/// Base class for all nodes in the build config.
sealed class BuildConfigBase {
  BuildConfigBase(this.errors);

  /// Accumulated errors. Non-null and non-empty when a node is invalid.
  final List<String>? errors;

  /// Whether there were errors when loading the data for this node.
  late final bool valid = errors == null;

  /// Returns an empty list when the object is valid, and errors when it is not.
  /// Subclasses with more data to check for validity should override this
  /// method and add `super.check(path)` to the returned list.
  @mustCallSuper
  List<String> check(String path) {
    if (valid) {
      return <String>[];
    }
    return errors!.map((String s) => '$path: $s').toList();
  }
}

/// The builder configuration is a json file containing a list of builds, tests,
/// generators and archives.
///
/// Each builder config file contains a top-level json map with the following
/// fields:
/// {
///    "builds": [],
///    "tests": [],
///    "generators": {
///      "tasks": []
///    },
///    "archives": []
/// }
final class BuilderConfig extends BuildConfigBase {
  /// Load build configuration data into an instance of this class.
  ///
  /// [path] should be the file system path to the file that the JSON data comes
  /// from. [map] must be the JSON data returned by e.g. `JsonDecoder.convert`.
  factory BuilderConfig.fromJson({required String path, required Map<String, Object?> map}) {
    final List<String> errors = <String>[];

    // Parse the "builds" field.
    final List<Build>? builds = objListOfJson<Build>(map, 'builds', errors, Build.fromJson);

    // Parse the "tests" field.
    final List<GlobalTest>? tests = objListOfJson<GlobalTest>(
      map,
      'tests',
      errors,
      GlobalTest.fromJson,
    );

    // Parse the "generators" field.
    final List<TestTask>? generators;
    if (map['generators'] == null) {
      generators = <TestTask>[];
    } else if (map['generators'] is! Map<String, Object?>) {
      appendTypeError(map, 'generators', 'map', errors);
      generators = null;
    } else {
      generators = objListOfJson(
        map['generators']! as Map<String, Object?>,
        'tasks',
        errors,
        TestTask.fromJson,
      );
    }

    // Parse the "archives" field.
    final List<GlobalArchive>? archives = objListOfJson<GlobalArchive>(
      map,
      'archives',
      errors,
      GlobalArchive.fromJson,
    );

    if (builds == null || tests == null || generators == null || archives == null) {
      return BuilderConfig._invalid(path, errors);
    }
    return BuilderConfig._(path, builds, tests, generators, archives);
  }

  BuilderConfig._(this.path, this.builds, this.tests, this.generators, this.archives) : super(null);

  BuilderConfig._invalid(this.path, super.errors)
    : builds = <Build>[],
      tests = <GlobalTest>[],
      generators = <TestTask>[],
      archives = <GlobalArchive>[];

  /// The path to the JSON file.
  final String path;

  /// A list of independent builds that have no dependencies among them. They
  /// can run in parallel if need be.
  final List<Build> builds;

  /// A list of tests. The tests may have dependencies on one or more of the
  /// builds.
  final List<GlobalTest> tests;

  /// A list of generator tasks that produce additional artifacts, which may
  /// depend on the output of one or more builds.
  final List<TestTask> generators;

  /// A description of the upload instructions for the artifacts produced by
  /// the global generators.
  final List<GlobalArchive> archives;

  @override
  List<String> check(String path) {
    final List<String> errors = <String>[];
    errors.addAll(super.check(path));
    for (int i = 0; i < builds.length; i++) {
      final Build build = builds[i];
      errors.addAll(build.check('$path/builds[$i]'));
    }
    for (int i = 0; i < tests.length; i++) {
      final GlobalTest test = tests[i];
      errors.addAll(test.check('$path/tests[$i]'));
    }
    for (int i = 0; i < generators.length; i++) {
      final TestTask task = generators[i];
      errors.addAll(task.check('$path/generators/tasks[$i]'));
    }
    for (int i = 0; i < archives.length; i++) {
      final GlobalArchive archive = archives[i];
      errors.addAll(archive.check('$path/archives[$i]'));
    }
    return errors;
  }

  /// Returns true if any of the [Build]s it contains can run on
  /// `platform`, and false otherwise.
  bool canRunOn(Platform platform) {
    return builds.any((Build b) => b.canRunOn(platform));
  }
}

/// A "build" is a dictionary with a gn command, a ninja command, zero or more
/// generator commands, zero or more local tests, zero or more local generators
/// and zero or more output artifacts.
///
/// "builds" contains a list of maps with fields like:
/// {
///   "name": "",
///   "description": "",
///   "gn": [""],
///   "ninja": {},
///   "tests": [],
///   "generators": {
///     "tasks": []
///   }, (optional)
///   "archives": [],
///   "drone_dimensions": [""],
///   "gclient_variables": {}
/// }
final class Build extends BuildConfigBase {
  factory Build.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? name = stringOfJson(map, 'name', errors);
    final String? description = stringOfJson(map, 'description', errors);
    final List<String>? gn = stringListOfJson(map, 'gn', errors);
    final List<BuildTest>? tests = objListOfJson(map, 'tests', errors, BuildTest.fromJson);
    final List<BuildArchive>? archives = objListOfJson(
      map,
      'archives',
      errors,
      BuildArchive.fromJson,
    );
    final List<String>? droneDimensions = stringListOfJson(map, 'drone_dimensions', errors);

    final BuildNinja? ninja;
    if (map['ninja'] == null) {
      ninja = BuildNinja.nop();
    } else if (map['ninja'] is! Map<String, Object?>) {
      ninja = null;
    } else {
      ninja = BuildNinja.fromJson(map['ninja']! as Map<String, Object?>);
    }
    if (ninja == null) {
      appendTypeError(map, 'ninja', 'map', errors);
    }

    final List<BuildTask>? generators;
    if (map['generators'] == null) {
      generators = <BuildTask>[];
    } else if (map['generators'] is! Map<String, Object?>) {
      appendTypeError(map, 'generators', 'map', errors);
      generators = null;
    } else {
      generators = objListOfJson(
        map['generators']! as Map<String, Object?>,
        'tasks',
        errors,
        BuildTask.fromJson,
      );
    }

    final Map<String, Object?>? gclientVariables;
    if (map['gclient_variables'] == null) {
      gclientVariables = <String, Object?>{};
    } else if (map['gclient_variables'] is! Map<String, Object?>) {
      gclientVariables = null;
    } else {
      gclientVariables = map['gclient_variables']! as Map<String, Object?>;
    }
    if (gclientVariables == null) {
      appendTypeError(map, 'gclient_variables', 'map', errors);
    }

    if (name == null ||
        description == null ||
        gn == null ||
        ninja == null ||
        archives == null ||
        tests == null ||
        generators == null ||
        droneDimensions == null ||
        gclientVariables == null) {
      return Build._invalid(errors);
    }
    return Build._(
      name,
      description,
      gn,
      ninja,
      tests,
      generators,
      archives,
      droneDimensions,
      gclientVariables,
    );
  }

  Build._(
    this.name,
    this.description,
    this.gn,
    this.ninja,
    this.tests,
    this.generators,
    this.archives,
    this.droneDimensions,
    this.gclientVariables,
  ) : super(null);

  Build._invalid(super.errors)
    : name = '',
      description = '',
      gn = <String>[],
      ninja = BuildNinja.nop(),
      tests = <BuildTest>[],
      generators = <BuildTask>[],
      archives = <BuildArchive>[],
      droneDimensions = <String>[],
      gclientVariables = <String, Object?>{};

  /// The name of the build which may also be used to reference it as a
  /// depdendency of a global test.
  final String name;

  /// The human-readable description of what a build accomplishes.
  final String description;

  /// The parameters to pass to `flutter/tools/gn` to configure the build.
  final List<String> gn;

  /// The data to form the ninja command to perform the build.
  final BuildNinja ninja;

  /// The list of tests that can be run after the ninja build is finished.
  final List<BuildTest> tests;

  /// A list of other tasks that may generate new artifacts after the ninja
  /// build is finished.
  final List<BuildTask> generators;

  /// Upload instructions for the artifacts produced by the build.
  final List<BuildArchive> archives;

  /// A list 'key=value' strings that are used to select the bot where this
  /// build will be running.
  final List<String> droneDimensions;

  /// A dictionary with variables included in the `custom_vars` section of the
  /// .gclient file before `gclient sync` is run.
  final Map<String, Object?> gclientVariables;

  /// Returns true if platform is capable of executing this build and false
  /// otherwise.
  bool canRunOn(Platform platform) => _canRunOn(droneDimensions, platform);

  @override
  List<String> check(String path) {
    final List<String> errors = <String>[];
    errors.addAll(super.check(path));
    errors.addAll(ninja.check('$path/ninja'));
    for (int i = 0; i < tests.length; i++) {
      final BuildTest test = tests[i];
      errors.addAll(test.check('$path/tests[$i]'));
    }
    for (int i = 0; i < generators.length; i++) {
      final BuildTask task = generators[i];
      errors.addAll(task.check('$path/generators/tasks[$i]'));
    }
    for (int i = 0; i < archives.length; i++) {
      final BuildArchive archive = archives[i];
      errors.addAll(archive.check('$path/archives[$i]'));
    }
    return errors;
  }
}

/// "builds" -> "ninja" contains a map with fields like:
/// {
///   "config": "",
///   "targets": [""]
/// },
final class BuildNinja extends BuildConfigBase {
  factory BuildNinja.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? config = stringOfJson(map, 'config', errors);
    final List<String>? targets = stringListOfJson(map, 'targets', errors);
    if (config == null || targets == null) {
      return BuildNinja._invalid(errors);
    }
    return BuildNinja._(config, targets);
  }

  BuildNinja._(this.config, this.targets) : super(null);

  BuildNinja._invalid(super.errors) : config = '', targets = <String>[];

  BuildNinja.nop() : config = '', targets = <String>[], super(null);

  /// The name of the configuration created by gn.
  ///
  /// This is also the subdirectory of the `out/` directory where the build
  /// output will go.
  final String config;

  /// The ninja targets to build.
  final List<String> targets;
}

/// "builds" -> "tests" contains a list of maps with fields like:
/// {
///  "language": "",
///  "name": "",
///  "parameters": [""],
///  "script": "",
///  "contexts": [""]
/// }
final class BuildTest extends BuildConfigBase {
  factory BuildTest.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? name = stringOfJson(map, 'name', errors);
    final String? language = stringOfJson(map, 'language', errors);
    final String? script = stringOfJson(map, 'script', errors);
    final List<String>? parameters = stringListOfJson(map, 'parameters', errors);
    final List<String>? contexts = stringListOfJson(map, 'contexts', errors);
    if (name == null ||
        language == null ||
        script == null ||
        parameters == null ||
        contexts == null) {
      return BuildTest._invalid(errors);
    }
    return BuildTest._(name, language, script, parameters, contexts);
  }

  BuildTest._(this.name, this.language, this.script, this.parameters, this.contexts) : super(null);

  BuildTest._invalid(super.errors)
    : name = '',
      language = '',
      script = '',
      parameters = <String>[],
      contexts = <String>[];

  /// The human readable description of the test.
  final String name;

  /// The executable used to run the script.
  final String language;

  /// The path to the script to execute relative to the checkout directory.
  final String script;

  /// Flags or parameters passed to the script.
  ///
  /// Parameters accept magic environment variables (placeholders replaced
  /// before executing the test). Magic environment variables have the following
  /// limitations: only ${FLUTTER_LOGS_DIR} is currently supported and it needs
  /// to be used alone within the parameter string(e.g. ["${FLUTTER_LOGS_DIR}"]
  /// is OK but ["path=${FLUTTER_LOGS_DIR}"] is not).
  final List<String> parameters;

  /// A list of available contexts to add to the text execution step.
  ///
  /// Two contexts are supported: "android_virtual_device" and
  /// "metric_center_token".
  final List<String> contexts;
}

/// "builds" -> "generators" is a map containing a single property "tasks",
/// which is a list of maps with fields like:
/// {
///   "name": "",
///   "parameters": [""],
///   "scripts": [""],
///   "language": ""
/// }
///
/// The semantics of this task are that each script in the list of scripts is
/// run in sequence by appending the same parameter list to each one.
final class BuildTask extends BuildConfigBase {
  factory BuildTask.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? name = stringOfJson(map, 'name', errors);
    final String? language = stringOfJson(map, 'language', errors);
    final List<String>? scripts = stringListOfJson(map, 'scripts', errors);
    final List<String>? parameters = stringListOfJson(map, 'parameters', errors);
    if (name == null || language == null || scripts == null || parameters == null) {
      return BuildTask._invalid(errors);
    }
    return BuildTask._(name, language, scripts, parameters);
  }

  BuildTask._invalid(super.errors)
    : name = '',
      language = '',
      scripts = <String>[],
      parameters = <String>[];

  BuildTask._(this.name, this.language, this.scripts, this.parameters) : super(null);

  /// The human readable name of the step running the script.
  final String name;

  /// The script language executable to run the script. If empty it is assumed
  /// to be bash.
  final String language;

  /// A list of paths of scripts relative to the checkout directory. Each
  /// script is run in turn by appending the list of parameters to it.
  final List<String> scripts;

  /// The flags passed to the script. Paths referenced in the list are relative
  /// to the checkout directory.
  final List<String> parameters;
}

/// "builds" -> "archives" contains a list of maps with fields like:
/// {
///   "name": "",
///   "base_path": "",
///   "type": "",
///   "include_paths": [""],
///   "realm": ""
/// }
final class BuildArchive extends BuildConfigBase {
  factory BuildArchive.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? name = stringOfJson(map, 'name', errors);
    final String? type = stringOfJson(map, 'type', errors);
    final String? basePath = stringOfJson(map, 'base_path', errors);
    final String? realm = stringOfJson(map, 'realm', errors);
    final List<String>? includePaths = stringListOfJson(map, 'include_paths', errors);
    if (name == null || type == null || basePath == null || realm == null || includePaths == null) {
      return BuildArchive._invalid(errors);
    }
    return BuildArchive._(name, type, basePath, realm, includePaths);
  }

  BuildArchive._invalid(super.error)
    : name = '',
      type = '',
      basePath = '',
      realm = '',
      includePaths = <String>[];

  BuildArchive._(this.name, this.type, this.basePath, this.realm, this.includePaths) : super(null);

  /// The name which may be referenced later as a dependency of global tests.
  final String name;

  /// The type of storage to use. Currently only “gcs” and “cas” are supported.
  final String type;

  /// The portion of the path to remove from the full path before uploading
  final String basePath;

  /// Either "production" or "experimental".
  final String realm;

  /// A list of strings with the paths to be uploaded to a given destination.
  final List<String> includePaths;
}

/// Global "tests" is a list of maps containing fields like:
/// {
///   "name": "",
///   "recipe": "",
///   "drone_dimensions": [""],
///   "dependencies": [""],
///   "tasks": [] (same format as above)
/// }
final class GlobalTest extends BuildConfigBase {
  factory GlobalTest.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? name = stringOfJson(map, 'name', errors);
    final String? recipe = stringOfJson(map, 'recipe', errors);
    final List<String>? droneDimensions = stringListOfJson(map, 'drone_dimensions', errors);
    final List<String>? dependencies = stringListOfJson(map, 'dependencies', errors);
    final List<TestDependency>? testDependencies = objListOfJson(
      map,
      'test_dependencies',
      errors,
      TestDependency.fromJson,
    );
    final List<TestTask>? tasks = objListOfJson(map, 'tasks', errors, TestTask.fromJson);
    if (name == null ||
        recipe == null ||
        droneDimensions == null ||
        dependencies == null ||
        testDependencies == null ||
        tasks == null) {
      return GlobalTest._invalid(errors);
    }
    return GlobalTest._(name, recipe, droneDimensions, dependencies, testDependencies, tasks);
  }

  GlobalTest._invalid(super.errors)
    : name = '',
      recipe = '',
      droneDimensions = <String>[],
      dependencies = <String>[],
      testDependencies = <TestDependency>[],
      tasks = <TestTask>[];

  GlobalTest._(
    this.name,
    this.recipe,
    this.droneDimensions,
    this.dependencies,
    this.testDependencies,
    this.tasks,
  ) : super(null);

  /// The name that will be assigned to the sub-build.
  final String name;

  /// The recipe name to use if different than tester.
  final String recipe;

  /// A list of strings with key values to select the bot where the test will
  /// run.
  final List<String> droneDimensions;

  /// A list of build outputs required by the test.
  final List<String> dependencies;

  /// A list of dependencies required for the test to run.
  final List<TestDependency> testDependencies;

  /// A list of dictionaries representing scripts and parameters to run them
  final List<TestTask> tasks;

  /// Returns true if platform is capable of executing this build and false
  /// otherwise.
  bool canRunOn(Platform platform) => _canRunOn(droneDimensions, platform);

  @override
  List<String> check(String path) {
    final List<String> errors = <String>[];
    errors.addAll(super.check(path));
    for (int i = 0; i < testDependencies.length; i++) {
      final TestDependency testDependency = testDependencies[i];
      errors.addAll(testDependency.check('$path/test_dependencies[$i]'));
    }
    for (int i = 0; i < tasks.length; i++) {
      final TestTask task = tasks[i];
      errors.addAll(task.check('$path/tasks[$i]'));
    }
    return errors;
  }
}

/// A test dependency for a global test has fields like:
/// {
///   "dependency": "",
///   "version": ""
/// }
final class TestDependency extends BuildConfigBase {
  factory TestDependency.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? dependency = stringOfJson(map, 'dependency', errors);
    final String? version = stringOfJson(map, 'version', errors);
    if (dependency == null || version == null) {
      return TestDependency._invalid(errors);
    }
    return TestDependency._(dependency, version);
  }

  TestDependency._invalid(super.error) : dependency = '', version = '';

  TestDependency._(this.dependency, this.version) : super(null);

  /// A dependency from the list at:
  /// https://flutter.googlesource.com/recipes/+/refs/heads/main/recipe_modules/flutter_deps/api.py#75
  final String dependency;

  /// The CIPD version string of the dependency.
  final String version;
}

/// Task for a global generator and a global test.
/// {
///   "name": "",
///   "parameters": [""],
///   "script": "",
///   "language": ""
/// }
final class TestTask extends BuildConfigBase {
  factory TestTask.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? name = stringOfJson(map, 'name', errors);
    final String? language = stringOfJson(map, 'language', errors);
    final String? script = stringOfJson(map, 'script', errors);
    final int? maxAttempts = intOfJson(map, 'max_attempts', fallback: 1, errors);
    final List<String>? parameters = stringListOfJson(map, 'parameters', errors);
    if (name == null ||
        language == null ||
        script == null ||
        maxAttempts == null ||
        parameters == null) {
      return TestTask._invalid(errors);
    }
    return TestTask._(name, language, script, maxAttempts, parameters);
  }

  TestTask._invalid(super.error)
    : name = '',
      language = '',
      script = '',
      maxAttempts = 0,
      parameters = <String>[];

  TestTask._(this.name, this.language, this.script, this.maxAttempts, this.parameters)
    : super(null);

  /// The human readable name of the step running the script.
  final String name;

  /// The script language executable to run the script. If empty it is assumed
  /// to be bash.
  final String language;

  /// The script path relative to the checkout repository.
  final String script;

  /// The maximum number of failures to tolerate. The default is 1.
  final int maxAttempts;

  /// The flags passed to the script. Paths referenced in the list are relative
  /// to the checkout directory.
  final List<String> parameters;
}

/// The objects that populate the list of global archives have fields like:
/// {
///     "source": "out/debug/artifacts.zip",
///     "destination": "ios/artifacts.zip",
///     "realm": "production"
/// },
final class GlobalArchive extends BuildConfigBase {
  factory GlobalArchive.fromJson(Map<String, Object?> map) {
    final List<String> errors = <String>[];
    final String? source = stringOfJson(map, 'source', errors);
    final String? destination = stringOfJson(map, 'destination', errors);
    final String? realm = stringOfJson(map, 'realm', errors);
    if (source == null || destination == null || realm == null) {
      return GlobalArchive._invalid(errors);
    }
    return GlobalArchive._(source, destination, realm);
  }

  GlobalArchive._invalid(super.error) : source = '', destination = '', realm = '';

  GlobalArchive._(this.source, this.destination, this.realm) : super(null);

  /// The path of the artifact relative to the engine checkout.
  final String source;

  /// The destination folder in the storage bucket.
  final String destination;

  /// Which storage bucket the destination path is relative to.
  /// Either "production" or "experimental".
  final String realm;
}

bool _canRunOn(List<String> droneDimensions, Platform platform) {
  String? os;
  for (final String dimension in droneDimensions) {
    os ??= switch (dimension.split('=')) {
      ['os', 'Linux'] => Platform.linux,
      ['os', final String win] when win.startsWith('Windows') => Platform.windows,
      ['os', final String mac] when mac.startsWith('Mac') => Platform.macOS,
      _ => null,
    };
  }
  return os == platform.operatingSystem;
}

void appendTypeError(
  Map<String, Object?> map,
  String field,
  String expected,
  List<String> errors, {
  Object? element,
}) {
  if (element == null) {
    final Type actual = map[field]!.runtimeType;
    errors.add('For field "$field", expected type: $expected, actual type: $actual.');
  } else {
    final Type actual = element.runtimeType;
    errors.add(
      'For element "$element" of "$field", '
      'expected type: $expected, actual type: $actual',
    );
  }
}

List<T>? objListOfJson<T>(
  Map<String, Object?> map,
  String field,
  List<String> errors,
  T Function(Map<String, Object?>) fn,
) {
  if (map[field] == null) {
    return <T>[];
  }
  if (map[field]! is! List<Object?>) {
    appendTypeError(map, field, 'list', errors);
    return null;
  }
  for (final Object? obj in map[field]! as List<Object?>) {
    if (obj is! Map<String, Object?>) {
      appendTypeError(map, field, 'map', errors);
      return null;
    }
  }
  return (map[field]! as List<Object?>).cast<Map<String, Object?>>().map<T>(fn).toList();
}

List<String>? stringListOfJson(Map<String, Object?> map, String field, List<String> errors) {
  if (map[field] == null) {
    return <String>[];
  }
  if (map[field]! is! List<Object?>) {
    appendTypeError(map, field, 'list', errors);
    return null;
  }
  for (final Object? obj in map[field]! as List<Object?>) {
    if (obj is! String) {
      appendTypeError(map, field, element: obj, 'string', errors);
      return null;
    }
  }
  return (map[field]! as List<Object?>).cast<String>();
}

String? stringOfJson(Map<String, Object?> map, String field, List<String> errors) {
  if (map[field] == null) {
    return '<undef>';
  }
  if (map[field]! is! String) {
    appendTypeError(map, field, 'string', errors);
    return null;
  }
  return map[field]! as String;
}

int? intOfJson(Map<String, Object?> map, String field, List<String> errors, {int fallback = 0}) {
  if (map[field] == null) {
    return fallback;
  }
  if (map[field]! is! int) {
    appendTypeError(map, field, 'int', errors);
    return null;
  }
  return map[field]! as int;
}
