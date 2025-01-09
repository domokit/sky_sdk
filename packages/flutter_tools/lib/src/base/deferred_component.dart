// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/file_system.dart';
import '../base/logger.dart';
import '../convert.dart';
import '../flutter_manifest.dart';

/// Represents a configured deferred component as defined in
/// the app's pubspec.yaml.
class DeferredComponent {
  DeferredComponent({
    required this.name,
    this.libraries = const <String>[],
    this.assets = const <AssetsEntry>[],
  }) : _assigned = false;

  /// The name of the deferred component. There should be a matching
  /// android dynamic feature module with the same name.
  final String name;

  /// The dart libraries this component includes as listed in pubspec.yaml.
  ///
  /// This list is only of dart libraries manually configured to be in this component.
  /// Valid libraries that are listed here will always be guaranteed to be
  /// packaged in this component. However, libraries that are not listed here
  /// may also be included if the loading units that are needed also contain
  /// libraries that are not listed here.
  final List<String> libraries;

  /// Assets that are part of this component.
  final List<AssetsEntry> assets;

  /// The minimal set of [LoadingUnit]s needed that contain all of the dart libraries in
  /// [libraries].
  ///
  /// Each [LoadingUnit] contains the compiled code for a set of dart libraries. Each
  /// [DeferredComponent] contains a list of dart libraries that must be included in the
  /// component. The set [loadingUnits] is all of the [LoadingUnit]s needed such that
  /// all required dart libs in [libraries] are in the union of the [LoadingUnit.libraries]
  /// included by the loading units in [loadingUnits].
  ///
  /// When [loadingUnits] is non-null, then the component is considered [assigned] and the
  /// field [assigned] will be true. When [loadingUnits] is null, then the component is
  /// unassigned and should not be used for any tasks that require loading unit information.
  /// When using [loadingUnits], [assigned] should be checked first. Loading units can be
  /// assigned with [assignLoadingUnits].
  Set<LoadingUnit>? get loadingUnits => _loadingUnits;
  Set<LoadingUnit>? _loadingUnits;

  /// Indicates if the component has loading units assigned.
  ///
  /// Unassigned components reflect the pubspec.yaml configuration directly,
  /// contain no loading unit data, and [loadingUnits] is null. Once assigned, the component
  /// will contain a set of [loadingUnits] which contains the [LoadingUnit]s that the
  /// component needs to include. Loading units can be assigned with the [assignLoadingUnits]
  /// call.
  bool get assigned => _assigned;
  bool _assigned;

  /// Selects the [LoadingUnit]s that contain this component's dart libraries.
  ///
  /// After calling this method, this [DeferredComponent] will be considered [assigned],
  /// and [loadingUnits] will return a non-null result.
  ///
  /// [LoadingUnit]s in `allLoadingUnits` that contain libraries that are in [libraries]
  /// are added to the set [loadingUnits].
  ///
  /// Providing null or empty list of `allLoadingUnits` will still change the assigned
  /// status, but will result in [loadingUnits] returning an empty set.
  void assignLoadingUnits(List<LoadingUnit> allLoadingUnits) {
    _assigned = true;
    _loadingUnits = <LoadingUnit>{
      for (final String lib in libraries)
        for (final LoadingUnit loadingUnit in allLoadingUnits)
          if (loadingUnit.libraries.contains(lib)) loadingUnit,
    };
  }

  /// Returns a descriptor of the component to be used when modifying a
  /// pubspec.yaml.
  Map<String, Object?> get descriptor {
    return <String, Object?>{
      'name': name,
      if (libraries.isNotEmpty) 'libraries': libraries.toList(),
      if (assets.isNotEmpty) 'assets': assets.map((AssetsEntry e) => e.descriptor).toList(),
    };
  }

  /// Provides a human readable string representation of the
  /// configuration.
  @override
  String toString() {
    final StringBuffer out = StringBuffer('\nDeferredComponent: $name\n  Libraries:');
    for (final String lib in libraries) {
      out.write('\n    - $lib');
    }
    if (loadingUnits != null && _assigned) {
      out.write('\n  LoadingUnits:');
      for (final LoadingUnit loadingUnit in loadingUnits!) {
        out.write('\n    - ${loadingUnit.id}');
      }
    }
    out.write('\n  Assets:');
    for (final AssetsEntry asset in assets) {
      out.write('\n    - ${asset.uri.path}');
      if (asset.flavors.isNotEmpty) {
        out.write(' (flavors: ${asset.flavors.join(', ')})');
      }
    }
    return out.toString();
  }
}

/// Represents a single loading unit and holds information regarding it's id,
/// shared library path, and dart libraries in it.
class LoadingUnit {
  /// Constructs a [LoadingUnit].
  ///
  /// Loading units must include an [id] and [libraries]. The [path] is only present when
  /// parsing the loading unit from a loading unit manifest produced by gen_snapshot.
  LoadingUnit({required this.id, required this.libraries, this.path});

  /// The unique loading unit id that is used to identify the loading unit within dart.
  final int id;

  /// A list of dart libraries that the loading unit contains.
  final List<String> libraries;

  /// The output path of the shared library .so file created by gen_snapshot.
  ///
  /// This value may be null when the loading unit is parsed from a
  /// `deferred_components_golden.yaml` file, which does not store the path.
  final String? path;

  /// Returns a human readable string representation of this LoadingUnit, ignoring
  /// the [path] field. The [path] is not included as it is not relevant when the
  @override
  String toString() {
    final StringBuffer out = StringBuffer('\nLoadingUnit $id\n  Libraries:');
    for (final String lib in libraries) {
      out.write('\n  - $lib');
    }
    return out.toString();
  }

  /// Returns true if the other loading unit has the same [id] and the same set of [libraries],
  /// ignoring order.
  bool equalsIgnoringPath(LoadingUnit other) {
    return other.id == id && other.libraries.toSet().containsAll(libraries);
  }

  /// Parses the loading unit manifests from the [outputDir] of the latest
  /// gen_snapshot/assemble run.
  ///
  /// This will read all existing loading units for every provided abi. If no abis are
  /// provided, loading units for all abis will be parsed.
  static List<LoadingUnit> parseGeneratedLoadingUnits(
    Directory outputDir,
    Logger logger, {
    List<String>? abis,
  }) {
    final List<LoadingUnit> loadingUnits = <LoadingUnit>[];
    final List<FileSystemEntity> files = outputDir.listSync(recursive: true);
    for (final FileSystemEntity fileEntity in files) {
      if (fileEntity is File) {
        final File file = fileEntity;
        // Determine if the abi is one we build.
        bool matchingAbi = abis == null;
        if (abis != null) {
          for (final String abi in abis) {
            if (file.parent.path.endsWith(abi)) {
              matchingAbi = true;
              break;
            }
          }
        }
        if (!file.path.endsWith('manifest.json') || !matchingAbi) {
          continue;
        }
        loadingUnits.addAll(parseLoadingUnitManifest(file, logger));
      }
    }
    return loadingUnits;
  }

  /// Parses loading units from a single loading unit manifest json file.
  ///
  /// Returns an empty list if the manifestFile does not exist or is invalid.
  static List<LoadingUnit> parseLoadingUnitManifest(File manifestFile, Logger logger) {
    if (!manifestFile.existsSync()) {
      return <LoadingUnit>[];
    }
    // Read gen_snapshot manifest
    final String fileString = manifestFile.readAsStringSync();
    Map<String, dynamic>? manifest;
    try {
      manifest = jsonDecode(fileString) as Map<String, dynamic>;
    } on FormatException catch (e) {
      logger.printError('Loading unit manifest at `${manifestFile.path}` was invalid JSON:\n$e');
    }
    // Set up android source directory
    return <LoadingUnit>[
      if (manifest?['loadingUnits'] case final List<dynamic> loadingUnits)
        for (final Map<String, dynamic> loadingUnitMap in loadingUnits.cast<Map<String, dynamic>>())
          if (loadingUnitMap['id'] != 1) // skip base unit
            LoadingUnit(
              id: loadingUnitMap['id'] as int,
              path: loadingUnitMap['path'] as String,
              libraries: List<String>.from(loadingUnitMap['libraries'] as List<dynamic>),
            ),
    ];
  }
}
