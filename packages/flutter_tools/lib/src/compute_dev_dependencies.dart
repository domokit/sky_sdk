// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:package_config/package_config.dart';
import 'package:yaml/yaml.dart';

import 'base/file_system.dart';
import 'project.dart';

/// Computes a representation of the transitive dependency graph rooted at
/// [project].
///
/// Does a search rooted in `project`, following the `dependencies` and
/// `dev_dependencies` of the pubspec to find the transitive dependencies of the
/// app (including the project itself). Will follow the `dev_dependencies` of
/// the [project] package.
///
/// Will load each of the dependencies' `pubspec.yaml` using [fileSystem]. Using
/// [packageConfig] to locate the files.
///
/// Avoids loading the [project] manifest again.
///
/// If a pubspec cannot be read, or is malformed, that pubspec is skipped. If
/// nothing has changed since a succesful `pub get` that should never happen.
/// In case of `flutter run --no-pub` this will at most ignore new dependencies.
Map<String, Dependency> computeTransitiveDependencies(
  FlutterProject project,
  PackageConfig packageConfig,
  FileSystem fileSystem, {
  bool followDevDependencies = false,
}) {
  final Map<String, Dependency> result = <String, Dependency>{};

  final List<String> packageNamesToVisit = <String>[project.manifest.appName];
  while (packageNamesToVisit.isNotEmpty) {
    final String current = packageNamesToVisit.removeLast();
    if (result.containsKey(current)) {
      continue;
    }
    final YamlNode pubspec;
    final List<String> dependencies = <String>[];
    final Uri rootUri;
    if (current == project.manifest.appName) {
      rootUri = project.directory.uri;
      pubspec = project.manifest.toYaml();
      dependencies.addAll(project.manifest.dependencies);
    } else {
      final Package? package = packageConfig[current];
      if (package == null) {
        continue;
      }
      rootUri = package.root;
      if (rootUri.scheme != 'file') {
        continue;
      }

      final String pubspecPath = fileSystem.path.fromUri(rootUri.resolve('pubspec.yaml'));
      try {
        pubspec = loadYamlNode(
          fileSystem.file(pubspecPath).readAsStringSync(),
          sourceUrl: Uri.file(pubspecPath),
        );
      } on IOException {
        continue;
      } on FormatException {
        continue;
      }
      if (pubspec is! YamlMap) {
        continue;
      }
      final Object dependenciesMap = pubspec['dependencies'] as Object? ?? <String, String>{};
      if (dependenciesMap is! Map) {
        continue;
      }

      for (final dynamic name in dependenciesMap.keys) {
        if (name is! String) {
          continue;
        }
        dependencies.add(name);
      }
    }
    packageNamesToVisit.addAll(dependencies);
    if (current == project.manifest.appName) {
      packageNamesToVisit.addAll(project.manifest.devDependencies);
    }
    result[current] = Dependency(pubspec, dependencies, rootUri, isExclusiveDevDependency: true);
  }

  // Do a second traversal of only the non-dev-dependencies, to patch up the
  // `isExclusiveDevDependency` property.
  final Set<String> visitedDependencies = <String>{};
  packageNamesToVisit.add(project.manifest.appName);
  while (packageNamesToVisit.isNotEmpty) {
    final String current = packageNamesToVisit.removeLast();
    if (!visitedDependencies.add(current)) {
      continue;
    }
    final Dependency? currentDependency = result[current];
    if (currentDependency == null) {
      continue;
    }
    result[current] = Dependency(
      currentDependency.pubspec,
      currentDependency.dependencies,
      currentDependency.rootUri,
      isExclusiveDevDependency: false,
    );
    packageNamesToVisit.addAll(currentDependency.dependencies);
  }
  return result;
}

/// Represents a node in a dependency graph.
class Dependency {
  Dependency(
    this.pubspec,
    this.dependencies,
    this.rootUri, {
    required this.isExclusiveDevDependency,
  });

  /// The names of the direct, non-dev dependencies.
  List<String> dependencies;

  /// True if this dependency is in the transitive closure of the main app's
  /// `dev_dependencies`, and **not** in the transitive closure of the regular
  /// dependencies.
  final bool isExclusiveDevDependency;

  /// The pubspec of this dependency as parsed (but otherwise unvalidated) yaml.
  final YamlNode pubspec;

  /// The location of the package. (Same level as its pubspec.yaml).
  final Uri rootUri;
}
