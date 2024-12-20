// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import '../base/file_system.dart';
import '../base/utils.dart';
import '../convert.dart';

/// A pseudo-filesystem stored in memory.
///
/// To support output to arbitrary multi-root file schemes, the frontend server
/// will output web sources, sourcemaps, and metadata to concatenated single files
/// with an additional manifest file containing the correct offsets.
class WebMemoryFS {
  final Map<String, Uint8List> metadataFiles = <String, Uint8List>{};
  final Map<String, Uint8List> files = <String, Uint8List>{};
  final Map<String, Uint8List> sourcemaps = <String, Uint8List>{};

  String? get mergedMetadata => _mergedMetadata;
  String? _mergedMetadata;

  /// Update the filesystem with the provided source and manifest files.
  ///
  /// Returns the list of updated files.
  List<String> write(File codeFile, File manifestFile, File sourcemapFile, File metadataFile) {
    final List<String> modules = <String>[];
    final Uint8List codeBytes = codeFile.readAsBytesSync();
    final Uint8List sourcemapBytes = sourcemapFile.readAsBytesSync();
    final Uint8List metadataBytes = metadataFile.readAsBytesSync();
    final Map<String, dynamic> manifest =
        castStringKeyedMap(json.decode(manifestFile.readAsStringSync()))!;
    for (final String filePath in manifest.keys) {
      final Map<String, dynamic> offsets = castStringKeyedMap(manifest[filePath])!;
      final List<int> codeOffsets = (offsets['code'] as List<dynamic>).cast<int>();
      final List<int> sourcemapOffsets = (offsets['sourcemap'] as List<dynamic>).cast<int>();
      final List<int> metadataOffsets = (offsets['metadata'] as List<dynamic>).cast<int>();
      if (codeOffsets.length != 2 || sourcemapOffsets.length != 2 || metadataOffsets.length != 2) {
        continue;
      }

      final int codeStart = codeOffsets[0];
      final int codeEnd = codeOffsets[1];
      if (codeStart < 0 || codeEnd > codeBytes.lengthInBytes) {
        continue;
      }
      final Uint8List byteView = Uint8List.view(codeBytes.buffer, codeStart, codeEnd - codeStart);
      final String fileName = filePath.startsWith('/') ? filePath.substring(1) : filePath;
      files[fileName] = byteView;

      final int sourcemapStart = sourcemapOffsets[0];
      final int sourcemapEnd = sourcemapOffsets[1];
      if (sourcemapStart < 0 || sourcemapEnd > sourcemapBytes.lengthInBytes) {
        continue;
      }
      final Uint8List sourcemapView = Uint8List.view(
        sourcemapBytes.buffer,
        sourcemapStart,
        sourcemapEnd - sourcemapStart,
      );
      final String sourcemapName = '$fileName.map';
      sourcemaps[sourcemapName] = sourcemapView;

      final int metadataStart = metadataOffsets[0];
      final int metadataEnd = metadataOffsets[1];
      if (metadataStart < 0 || metadataEnd > metadataBytes.lengthInBytes) {
        continue;
      }
      final Uint8List metadataView = Uint8List.view(
        metadataBytes.buffer,
        metadataStart,
        metadataEnd - metadataStart,
      );
      final String metadataName = '$fileName.metadata';
      metadataFiles[metadataName] = metadataView;

      modules.add(fileName);
    }

    _mergedMetadata = metadataFiles.values
        .map((Uint8List encoded) => utf8.decode(encoded))
        .join('\n');

    return modules;
  }
}
