// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:crypto/crypto.dart' show md5;
import 'package:meta/meta.dart';

import '../convert.dart' show json;
import 'file_system.dart';
import 'logger.dart';
import 'utils.dart';

/// A tool that can be used to compute, compare, and write [Fingerprint]s for a
/// set of input files and associated build settings.
///
/// This class should only be used in situations where `assemble` is not appropriate,
/// such as checking if Cocoapods should be run.
class Fingerprinter {
  Fingerprinter({
    required this.fingerprintPath,
    required Iterable<String> paths,
    required FileSystem fileSystem,
    required Logger logger,
  }) : _paths = paths.toList(),
       _logger = logger,
       _fileSystem = fileSystem;

  final String fingerprintPath;
  final List<String> _paths;
  final Logger _logger;
  final FileSystem _fileSystem;

  Fingerprint buildFingerprint() {
    final List<String> paths = _getPaths();
    return Fingerprint.fromBuildInputs(paths, _fileSystem);
  }

  bool doesFingerprintMatch() {
    try {
      final File fingerprintFile = _fileSystem.file(fingerprintPath);
      if (!fingerprintFile.existsSync()) {
        return false;
      }

      final List<String> paths = _getPaths();
      if (!paths.every(_fileSystem.isFileSync)) {
        return false;
      }

      final Fingerprint oldFingerprint = Fingerprint.fromJson(fingerprintFile.readAsStringSync());
      final Fingerprint newFingerprint = buildFingerprint();
      return oldFingerprint == newFingerprint;
    } on Exception catch (e) {
      // Log exception and continue, fingerprinting is only a performance improvement.
      _logger.printTrace('Fingerprint check error: $e');
    }
    return false;
  }

  void writeFingerprint() {
    try {
      final Fingerprint fingerprint = buildFingerprint();
      final File fingerprintFile = _fileSystem.file(fingerprintPath);
      fingerprintFile.createSync(recursive: true);
      fingerprintFile.writeAsStringSync(fingerprint.toJson());
    } on Exception catch (e) {
      // Log exception and continue, fingerprinting is only a performance improvement.
      _logger.printTrace('Fingerprint write error: $e');
    }
  }

  List<String> _getPaths() => _paths;
}

/// A fingerprint that uniquely identifies a set of build input files and
/// properties.
///
/// See [Fingerprinter].
@immutable
class Fingerprint {
  const Fingerprint._({Map<String, String>? checksums})
    : _checksums = checksums ?? const <String, String>{};

  factory Fingerprint.fromBuildInputs(Iterable<String> inputPaths, FileSystem fileSystem) {
    final Iterable<File> files = inputPaths.map<File>(fileSystem.file);
    final Iterable<File> missingInputs = files.where((File file) => !file.existsSync());
    if (missingInputs.isNotEmpty) {
      throw Exception('Missing input files:\n${missingInputs.join('\n')}');
    }
    return Fingerprint._(
      checksums: <String, String>{
        for (final File file in files) file.path: md5.convert(file.readAsBytesSync()).toString(),
      },
    );
  }

  /// Creates a Fingerprint from serialized JSON.
  ///
  /// Throws [Exception], if there is a version mismatch between the
  /// serializing framework and this framework.
  factory Fingerprint.fromJson(String jsonData) {
    final Map<String, dynamic>? content = castStringKeyedMap(json.decode(jsonData));
    final Map<String, String>? files =
        content == null ? null : castStringKeyedMap(content['files'])?.cast<String, String>();
    return Fingerprint._(checksums: files ?? <String, String>{});
  }

  final Map<String, String> _checksums;

  String toJson() => json.encode(<String, dynamic>{'files': _checksums});

  @override
  bool operator ==(Object other) {
    return other is Fingerprint && _equalMaps(other._checksums, _checksums);
  }

  bool _equalMaps(Map<String, String> a, Map<String, String> b) {
    return a.length == b.length && a.keys.every((String key) => a[key] == b[key]);
  }

  @override
  int get hashCode => Object.hash(
    Object.hashAllUnordered(_checksums.keys),
    Object.hashAllUnordered(_checksums.values),
  );

  @override
  String toString() => '{checksums: $_checksums}';
}
