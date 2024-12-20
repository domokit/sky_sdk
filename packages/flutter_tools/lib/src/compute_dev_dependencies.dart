// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:package_config/package_config.dart';

import 'base/logger.dart';
import 'flutter_manifest.dart';
import 'project.dart';

/// Computes a representation of the transitive dependency graph rooted at
/// [project].
///
/// Does a search rooted in `project`, following the `dependencies` and
/// `dev_dependencies` of the pubspec to find the transitive dependencies of the
/// app. (including the project itself). Will follow the `dev_dependencies` of
/// the [project] package.
///
/// Will load each of the dependencies' `pubspec.yaml` using [fileSystem]. Using
/// [packageConfig] to locate the files.
///
/// Does not load the [project] manifest again.
///
/// If a pubspec cannot be read, or is malformed, a warning is issued on
/// [logger] and that pubspec is skipped. If nothing has changed since a
/// succesful `pub get` that should never happen.
Map<String, Dependency> computeTransitiveDependencies(
  FlutterProject project,
  PackageConfig packageConfig,
  FileSystem fileSystem,
  Logger logger, {
  bool followDevDependencies = false,
}) {
  final Map<String, Dependency> result = <String, Dependency>{};

  final List<String> packageNamesToVisit = <String>[project.manifest.appName];
  while (packageNamesToVisit.isNotEmpty) {
    final String current = packageNamesToVisit.removeLast();
    if (result.containsKey(current)) {
      continue;
    }
    final FlutterManifest? packageManifest;
    if (current == project.manifest.appName) {
      packageManifest = project.manifest;
    } else {
      final Package? package = packageConfig[current];
      if (package == null) {
        continue;
      }
      final Uri packageUri = package.root;
      if (packageUri.scheme != 'file') {
        continue;
      }
      final String pubspecPath = fileSystem.path.fromUri(packageUri.resolve('pubspec.yaml'));
      packageManifest = FlutterManifest.createFromPath(
        pubspecPath,
        fileSystem: fileSystem,
        logger: logger,
      );
      if (packageManifest == null) {
        continue;
      }
    }

    packageNamesToVisit.addAll(packageManifest.dependencies);
    if (current == project.manifest.appName) {
      packageNamesToVisit.addAll(packageManifest.devDependencies);
    }
    result[current] = Dependency(packageManifest, isExclusiveDevDependency: true);
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
    final FlutterManifest? manifest = result[current]?.manifest;
    if (manifest == null) {
      continue;
    }
    result[current] = Dependency(manifest, isExclusiveDevDependency: false);
    packageNamesToVisit.addAll(manifest.dependencies);
  }
  return result;
}

/// Represents a node in a dependency graph.
class Dependency {
  Dependency(this.manifest, {required this.isExclusiveDevDependency});

  /// True if this dependency is in the transitive closure of the main app's
  /// `dev_dependencies`, and **not** in the transitive closure of the regular
  /// dependencies.
  final bool isExclusiveDevDependency;

  final FlutterManifest manifest;
}
