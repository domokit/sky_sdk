// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:collection/collection.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as p;

import 'environment.dart';
import 'label.dart';
import 'typed_json.dart';

/// An interface to the [GN](https://gn.googlesource.com/gn) build system.
///
/// See also: <https://gn.googlesource.com/gn/+/main/docs/reference.md>.
interface class Gn {
  /// Create a new GN interface using the given [environment].
  ///
  /// A GN binary must exist in `{engine.srcDir}/flutter/third_party/gn/gn`[^1].
  ///
  /// [^1]: On Windows, the binary is named `gn.exe`.
  const Gn.fromEnvironment(this._environment);

  final Environment _environment;

  String get _gnPath {
    return p.join(
      _environment.engine.srcDir.path,
      'flutter',
      'third_party',
      'gn',
      _environment.platform.isWindows ? 'gn.exe' : 'gn',
    );
  }

  /// Returns a list of build targets that match the given [pattern].
  ///
  /// This is equivalent to running:
  /// ```sh
  /// gn desc {outDir} {labelOrPattern}
  /// ```
  ///
  /// The [outDir] is the output directory of the build, e.g. `out/Release`.
  ///
  /// See also: <https://gn.googlesource.com/gn/+/main/docs/reference.md#cmd_desc>.
  Future<List<BuildTarget>> desc(String outDir, TargetPattern pattern) async {
    final command = [_gnPath, 'desc', '--format=json', outDir, pattern.toGnPattern()];
    final process = await _environment.processRunner.runProcess(
      command,
      workingDirectory: _environment.engine.srcDir,
      failOk: true,
    );
    if (process.exitCode != 0) {
      // If the error was in the format:
      // "The input testing/foo:foo matches no targets, configs or files."
      //
      // Then report a nicer error, versus a fatal error.
      final stdout = process.stdout;
      if (stdout.contains('matches no targets, configs or files')) {
        final gnPattern = pattern.toGnPattern();
        if (!gnPattern.startsWith('//flutter')) {
          _environment.logger.warning(
            'No targets matched the pattern `$gnPattern`.'
            'Did you mean `//flutter/$gnPattern`?',
          );
        } else {
          _environment.logger.warning('No targets matched the pattern `${pattern.toGnPattern()}`');
        }
        return [];
      }

      _environment.logger.fatal(
        'Failed to run `${command.join(' ')}` (exit code ${process.exitCode})'
        '\n\n'
        'STDOUT:\n${process.stdout}\n'
        'STDERR:\n${process.stderr}\n',
      );
    }

    final JsonObject result;
    try {
      result = JsonObject.parse(process.stdout);
    } on FormatException catch (e) {
      _environment.logger.fatal(
        'Failed to parse JSON output from `gn desc`:\n$e\n${process.stdout}',
      );
    }

    return result
        .asMap()
        .entries
        .map((entry) {
          final label = entry.key;
          final properties = entry.value;
          if (properties is! Map<String, Object?>) {
            return null;
          }
          final target = BuildTarget._fromJson(label, JsonObject(properties));
          if (target == null) {
            _environment.logger.warning(
              'Unknown target type for $label: type=${properties['type']}',
            );
            return null;
          }
          return target;
        })
        .whereType<BuildTarget>()
        .toList();
  }
}

/// Information about a build target.
@immutable
sealed class BuildTarget {
  const BuildTarget({required this.label, required this.testOnly});

  factory BuildTarget._parseFromAction(
    String label, {
    required bool testOnly,
    required JsonObject json,
  }) {
    final metadata = json.objectOrNull('metadata');
    if (metadata != null) {
      final actionTypes = metadata.stringListOrNull('action_type');
      if (actionTypes != null && actionTypes.contains('dart_test')) {
        final String executable;
        final outputs = json.stringListOrNull('outputs');
        if (outputs == null || outputs.isEmpty) {
          throw StateError('Expected at least one output for $label');
        }

        // Remove the leading // from the path.
        executable = outputs.first.substring(2);
        return ExecutableBuildTarget(
          label: Label.parseGn(label),
          testOnly: testOnly,
          executable: executable,
        );
      }
    }
    return ActionBuildTarget(label: Label.parseGn(label), testOnly: testOnly);
  }

  /// Returns a build target from JSON originating from `gn desc --format=json`.
  ///
  /// If the JSON is not a supported build target, returns `null`.
  static BuildTarget? _fromJson(String label, JsonObject json) {
    final (String type, bool testOnly) = json.map(
      (json) => (json.string('type'), json.boolean('testonly')),
    );
    return switch (type) {
      'executable' => ExecutableBuildTarget(
        label: Label.parseGn(label),
        testOnly: testOnly,
        // Remove the leading // from the path.
        executable: json.stringList('outputs').first.substring(2),
      ),
      'shared_library' ||
      'static_library' => LibraryBuildTarget(label: Label.parseGn(label), testOnly: testOnly),
      'action' => BuildTarget._parseFromAction(label, testOnly: testOnly, json: json),
      'group' => GroupBuildTarget(
        label: Label.parseGn(label),
        testOnly: testOnly,
        deps: json.stringList('deps').map(Label.parseGn).toList(),
      ),
      _ => null,
    };
  }

  /// Build target label, e.g. `//flutter/fml:fml_unittests`.
  final Label label;

  /// Whether a target is only used for testing.
  final bool testOnly;

  @mustBeOverridden
  @override
  bool operator ==(Object other);

  @mustBeOverridden
  @override
  int get hashCode;

  @mustBeOverridden
  @override
  String toString();
}

/// A build target that performs some [action][].
///
/// [action]: https://gn.googlesource.com/gn/+/main/docs/reference.md#func_action
final class ActionBuildTarget extends BuildTarget {
  /// Construct an action build target.
  const ActionBuildTarget({required super.label, required super.testOnly});

  @override
  bool operator ==(Object other) {
    return other is LibraryBuildTarget && label == other.label && testOnly == other.testOnly;
  }

  @override
  int get hashCode => Object.hash(label, testOnly);

  @override
  String toString() => 'ActionBuildTarget($label, testOnly=$testOnly)';
}

/// A build target that produces a [shared library][] or [static library][].
///
/// [shared library]: https://gn.googlesource.com/gn/+/main/docs/reference.md#func_shared_library
/// [static library]: https://gn.googlesource.com/gn/+/main/docs/reference.md#func_static_library
final class LibraryBuildTarget extends BuildTarget {
  /// Construct a library build target.
  const LibraryBuildTarget({required super.label, required super.testOnly});

  @override
  bool operator ==(Object other) {
    return other is LibraryBuildTarget && label == other.label && testOnly == other.testOnly;
  }

  @override
  int get hashCode => Object.hash(label, testOnly);

  @override
  String toString() => 'LibraryBuildTarget($label, testOnly=$testOnly)';
}

/// A build target that produces an [executable][] program.
///
/// [executable]: https://gn.googlesource.com/gn/+/main/docs/reference.md#func_executable
final class ExecutableBuildTarget extends BuildTarget {
  /// Construct an executable build target.
  const ExecutableBuildTarget({
    required super.label,
    required super.testOnly,
    required this.executable,
  });

  /// The path to the executable program.
  final String executable;

  @override
  bool operator ==(Object other) {
    return other is ExecutableBuildTarget &&
        label == other.label &&
        testOnly == other.testOnly &&
        executable == other.executable;
  }

  @override
  int get hashCode => Object.hash(label, testOnly, executable);

  @override
  String toString() => 'ExecutableBuildTarget($label, testOnly=$testOnly, executable=$executable)';
}

/// A build target that [group]s a meta-target of other build targets.
///
/// [group]: https://gn.googlesource.com/gn/+/main/docs/reference.md#func_group
final class GroupBuildTarget extends BuildTarget {
  /// Construct a group build target.
  const GroupBuildTarget({required super.label, required super.testOnly, required this.deps});

  /// The list of dependencies for this group target.
  final List<Label> deps;

  @override
  bool operator ==(Object other) {
    if (other is! GroupBuildTarget) {
      return false;
    }
    if (label != other.label || testOnly != other.testOnly) {
      return false;
    }
    return const ListEquality<Object>().equals(deps, other.deps);
  }

  @override
  int get hashCode => Object.hash(label, testOnly, Object.hashAll(deps));

  @override
  String toString() => 'GroupBuildTarget($label, testOnly=$testOnly, deps=$deps)';
}
