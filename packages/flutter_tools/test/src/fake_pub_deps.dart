// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/project.dart';

/// A fake implementation of [Pub] with a pre-primed [deps] response.
final class FakePubWithPrimedDeps implements Pub {
  /// Creates an implementation of [Pub] with a pre-primed [Pub.deps] response.
  ///
  /// It is expected that every [FlutterProject] that is provided to the
  /// [Pub.deps] call is represented by [rootPackageName].
  ///
  /// Optionally, provide [devDependencies] (of [rootPackageName]), and non
  /// dev-dependencies ([dependencies]) of any package to a set of any other
  /// packages. A resulting valid `dart pub deps --json` response is implicitly
  /// created.
  factory FakePubWithPrimedDeps({
    String rootPackageName = 'app_name',
    Set<String> devDependencies = const <String>{},
    Map<String, Set<String>> dependencies = const <String, Set<String>>{},
  }) {
    // Start the packages: [ ... ] list with the root package.
    final List<Object?> packages = <Object?>[
      <String, Object?>{
        'name': rootPackageName,
        'kind': 'root',
        'dependencies': <String>[...dependencies.keys, ...devDependencies]..sort(),
        'directDependencies': <String>[...?dependencies[rootPackageName], ...devDependencies]
          ..sort(),
        'devDependencies': <String>[...devDependencies],
      },
    ];

    // Add all non-dev dependencies.
    for (final String packageName in dependencies.keys) {
      final bool direct = dependencies[rootPackageName]!.contains(packageName);
      packages.add(<String, Object?>{
        'name': packageName,
        'kind': direct ? 'direct' : 'transitive',
        'dependencies': <String>[...?dependencies[packageName]],
        'directDependencies': <String>[...?dependencies[packageName]],
      });
    }

    // Add all dev-dependencies.
    for (final String packageName in devDependencies) {
      packages.add(<String, Object?>{
        'name': packageName,
        'kind': 'dev',
        'dependencies': <String>[],
        'directDependencies': <String>[],
      });
    }

    return FakePubWithPrimedDeps._(<String, Object?>{
      'root': rootPackageName,
      'packages': packages,
    });
  }

  const FakePubWithPrimedDeps._(this._deps);
  final Map<String, Object?> _deps;

  @override
  Future<Map<String, Object?>> deps(FlutterProject project) async => _deps;

  @override
  Never noSuchMethod(Invocation invocation) {
    throw UnsupportedError('Only <Pub>.deps is expected to be called');
  }
}
