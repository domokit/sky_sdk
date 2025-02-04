// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:package_config/package_config.dart';
import 'package:yaml/yaml.dart';

import 'base/file_system.dart';
import 'base/logger.dart';
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
  FileSystem fileSystem,
  Logger logger,
) {
  final Map<String, Dependency> result = <String, Dependency>{};
  result[project.manifest.appName] = Dependency(
    project.manifest.toYaml(),
    project.manifest.dependencies.toList(),
    project.directory.uri,

    // While building the dependency graph we mark everything as
    // isExclusiveDevDependency. Afterwards we traverse the non-dev-dependency
    // part of the graph removing the marker.
    isExclusiveDevDependency: true,
  );

  final List<String> packageNamesToVisit = <String>[
    ...project.manifest.dependencies,
    ...project.manifest.devDependencies,
  ];
  while (packageNamesToVisit.isNotEmpty) {
    final String current = packageNamesToVisit.removeLast();
    if (result.containsKey(current)) {
      continue;
    }
    final _PackageDetails? details = _loadPackageDetails(
      current,
      packageConfig,
      project,
      fileSystem,
      logger,
    );
    if (details == null) {
      continue;
    }

    packageNamesToVisit.addAll(details.dependencies);
    result[current] = Dependency(
      details.pubspec,
      details.dependencies,
      details.rootUri,
      isExclusiveDevDependency: true,
    );
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

typedef _PackageDetails = ({List<String> dependencies, YamlMap pubspec, Uri rootUri});

/// Attempts to load the pubspec.yaml for [packageName] and extract the list of
/// dependencies.
///
/// Returns `null` on failure (malformed pubspec etc.).
_PackageDetails? _loadPackageDetails(
  String packageName,
  PackageConfig packageConfig,
  FlutterProject project,
  FileSystem fileSystem,
  Logger logger,
) {
  final Package? package = packageConfig[packageName];
  if (package == null) {
    logger.printTrace('''
Could not load details of package `$packageName`.
It was not present in the package config.''');
    return null;
  }
  final Uri rootUri = package.root;
  if (rootUri.scheme != 'file') {
    logger.printTrace('''
Could not load details of package `$packageName`.
It did not have a file rootUri: `$rootUri`.''');
    return null;
  }
  final String pubspecPath = fileSystem.path.fromUri(rootUri.resolve('pubspec.yaml'));
  try {
    final YamlNode pubspec = loadYamlNode(
      fileSystem.file(pubspecPath).readAsStringSync(),
      sourceUrl: Uri.file(pubspecPath),
    );
    if (pubspec is! YamlMap) {
      logger.printTrace('''
Could not load details of package `$packageName`.
It has malformed $pubspecPath: toplevel is not a map.''');
      return null;
    }
    final List<String> dependencies = <String>[];
    final Object dependenciesMap = pubspec['dependencies'] as Object? ?? <String, String>{};
    if (dependenciesMap is! Map) {
      logger.printTrace('''
Could not load details of package `$packageName`.
It has malformed $pubspecPath: `dependencies` is not a map.''');
      return null;
    }

    for (final Object? name in dependenciesMap.keys) {
      if (name is! String) {
        continue;
      }
      dependencies.add(name);
    }
    return (rootUri: rootUri, pubspec: pubspec, dependencies: dependencies);
  } on IOException catch (e) {
    logger.printTrace('''
Could not load details of package `$packageName`.
Could not load $pubspecPath: $e.''');
    return null;
  } on FormatException catch (e) {
    logger.printTrace('''
Could not load details of package `$packageName`.
Could not parse $pubspecPath: $e.''');
    return null;
  }
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
  final List<String> dependencies;

  /// True if this dependency is in the transitive closure of the main app's
  /// `dev_dependencies`, and **not** in the transitive closure of the regular
  /// dependencies.
  final bool isExclusiveDevDependency;

  /// The pubspec of this dependency as parsed (but otherwise unvalidated) yaml.
  final YamlNode pubspec;

  /// The location of the package. (Same level as its pubspec.yaml).
  final Uri rootUri;
}
