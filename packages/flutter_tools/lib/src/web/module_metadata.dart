// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Taken from https://github.com/dart-lang/webdev/blob/616da45582e008efa114728927eabb498c71f1b7/dwds/lib/src/debugging/metadata/module_metadata.dart.
// Prefer to keep the implementations consistent.

/// Module metadata format version
///
/// Module reader always creates the current version but is able to read
/// metadata files with later versions as long as the changes are backward
/// compatible, i.e. only minor or patch versions have changed.
class ModuleMetadataVersion {
  const ModuleMetadataVersion(this.majorVersion, this.minorVersion, this.patchVersion);

  final int majorVersion;
  final int minorVersion;
  final int patchVersion;

  /// Current metadata version
  ///
  /// Version follows simple semantic versioning format 'major.minor.patch'
  /// See https://semver.org
  static const ModuleMetadataVersion current = ModuleMetadataVersion(2, 0, 0);

  /// Previous version supported by the metadata reader
  static const ModuleMetadataVersion previous = ModuleMetadataVersion(1, 0, 0);

  /// Current metadata version created by the reader
  String get version => '$majorVersion.$minorVersion.$patchVersion';

  /// Is this metadata version compatible with the given version
  ///
  /// The minor and patch version changes never remove any fields that current
  /// version supports, so the reader can create current metadata version from
  /// any file created with a later writer, as long as the major version does
  /// not change.
  bool isCompatibleWith(String version) {
    final List<String> parts = version.split('.');
    if (parts.length != 3) {
      throw FormatException(
        'Version: $version'
        'does not follow simple semantic versioning format',
      );
    }
    final int major = int.parse(parts[0]);
    final int minor = int.parse(parts[1]);
    final int patch = int.parse(parts[2]);
    return major == majorVersion && minor >= minorVersion && patch >= patchVersion;
  }
}

/// Library metadata
///
/// Represents library metadata used in the debugger,
/// supports reading from and writing to json.
class LibraryMetadata {
  LibraryMetadata(this.name, this.importUri, this.partUris);

  LibraryMetadata.fromJson(Map<String, Object?> json)
    : name = _readRequiredField(json, nameField),
      importUri = _readRequiredField(json, importUriField),
      partUris = _readOptionalList(json, partUrisField) ?? <String>[];

  static const String nameField = 'name';
  static const String importUriField = 'importUri';
  static const String partUrisField = 'partUris';

  /// Library name as defined in pubspec.yaml
  final String name;

  /// Library importUri
  ///
  /// Example package:path/path.dart
  final String importUri;

  /// All file uris from the library
  ///
  /// Can be relative paths to the directory of the fileUri
  final List<String> partUris;

  Map<String, Object?> toJson() {
    return <String, Object?>{
      nameField: name,
      importUriField: importUri,
      partUrisField: <String>[...partUris],
    };
  }
}

/// Module metadata
///
/// Represents module metadata used in the debugger,
/// supports reading from and writing to json.
class ModuleMetadata {
  ModuleMetadata(this.name, this.closureName, this.sourceMapUri, this.moduleUri, {String? ver}) {
    version = ver ?? ModuleMetadataVersion.current.version;
  }

  ModuleMetadata.fromJson(Map<String, Object?> json)
    : version = _readRequiredField(json, versionField),
      name = _readRequiredField(json, nameField),
      closureName = _readRequiredField(json, closureNameField),
      sourceMapUri = _readRequiredField(json, sourceMapUriField),
      moduleUri = _readRequiredField(json, moduleUriField) {
    if (!ModuleMetadataVersion.current.isCompatibleWith(version) &&
        !ModuleMetadataVersion.previous.isCompatibleWith(version)) {
      throw Exception(
        'Unsupported metadata version $version. '
        '\n  Supported versions: '
        '\n    ${ModuleMetadataVersion.current.version} '
        '\n    ${ModuleMetadataVersion.previous.version}',
      );
    }

    for (final Map<String, Object?> l in _readRequiredList<Map<String, Object?>>(
      json,
      librariesField,
    )) {
      addLibrary(LibraryMetadata.fromJson(l));
    }
  }

  static const String versionField = 'version';
  static const String nameField = 'name';
  static const String closureNameField = 'closureName';
  static const String sourceMapUriField = 'sourceMapUri';
  static const String moduleUriField = 'moduleUri';
  static const String librariesField = 'libraries';

  /// Metadata format version
  late final String version;

  /// Module name
  ///
  /// Used as a name of the js module created by the compiler and
  /// as key to store and load modules in the debugger and the browser
  // TODO(srujzs): Remove once https://github.com/dart-lang/sdk/issues/59618 is
  // resolved.
  final String name;

  /// Name of the function enclosing the module
  ///
  /// Used by debugger to determine the top dart scope
  final String closureName;

  /// Source map uri
  final String sourceMapUri;

  /// Module uri
  final String moduleUri;

  final Map<String, LibraryMetadata> libraries = <String, LibraryMetadata>{};

  /// Add [library] to metadata
  ///
  /// Used for filling the metadata in the compiler or for reading from
  /// stored metadata files.
  void addLibrary(LibraryMetadata library) {
    if (!libraries.containsKey(library.importUri)) {
      libraries[library.importUri] = library;
    } else {
      throw Exception(
        'Metadata creation error: '
        'Cannot add library $library with uri ${library.importUri}: '
        'another library "${libraries[library.importUri]}" is found '
        'with the same uri',
      );
    }
  }

  Map<String, Object?> toJson() {
    return <String, Object?>{
      versionField: version,
      nameField: name,
      closureNameField: closureName,
      sourceMapUriField: sourceMapUri,
      moduleUriField: moduleUri,
      librariesField: <Map<String, Object?>>[
        for (final LibraryMetadata lib in libraries.values) lib.toJson(),
      ],
    };
  }
}

T _readRequiredField<T>(Map<String, Object?> json, String field) {
  if (!json.containsKey(field)) {
    throw FormatException('Required field $field is not set in $json');
  }
  return json[field]! as T;
}

T? _readOptionalField<T>(Map<String, Object?> json, String field) => json[field] as T?;

List<T> _readRequiredList<T>(Map<String, Object?> json, String field) {
  final List<Object?> list = _readRequiredField<List<Object?>>(json, field);
  return List.castFrom<Object?, T>(list);
}

List<T>? _readOptionalList<T>(Map<String, Object?> json, String field) {
  final List<Object?>? list = _readOptionalField<List<Object?>>(json, field);
  return list == null ? null : List.castFrom<Object?, T>(list);
}
