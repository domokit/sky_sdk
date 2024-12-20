// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' as convert;

/// A Dart representation of a `digest.json` file.
///
/// A single file is typically used to represent the results of a test suite,
/// or a series of tests that have been run with the same [dimensions]. For
/// example, "Impeller Unittests on Vulkan running on MacOS" might generate
/// a `digest.json` file.
///
/// Other tools (perhaps implemented in other languages, like C++) can use this
/// format to communicate with the `golden_tests_harvester` tool without
/// relying on the tool directly (or the Dart SDK).
final class Digests {
  /// Creates a new instance of [Digests].
  ///
  /// In practice, [Digests.parse] is typically used to create a new instance
  /// from an existing `digest.json` file read into memory as string contents.
  const Digests({required this.dimensions, required this.entries});

  /// Parses a `digest.json` file from a string.
  factory Digests.parse(String json) {
    final Object? decoded = convert.json.decode(json);
    if (decoded is! Map<String, Object?>) {
      throw FormatException('Expected a JSON object as the root, but got $decoded', json);
    }

    final Object? dimensions = decoded['dimensions'];
    if (dimensions is! Map<String, Object?>) {
      throw FormatException(
        'Expected a JSON object "dimensions", but got ${dimensions.runtimeType}',
        dimensions,
      );
    }

    final Object? entries = decoded['entries'];
    if (entries is! List<Object?>) {
      throw FormatException(
        'Expected a JSON list "entries", but got ${entries.runtimeType}',
        entries,
      );
    }

    // Now parse the entries.
    return Digests(
      dimensions: dimensions.map((String key, Object? value) {
        if (value is! String) {
          throw FormatException(
            'Expected a JSON string for dimension "$key", but got ${value.runtimeType}',
            value,
          );
        }
        return MapEntry<String, String>(key, value);
      }),
      entries: List<DigestEntry>.unmodifiable(
        entries.map((Object? entry) {
          if (entry is! Map<String, Object?>) {
            throw FormatException(
              'Expected a JSON object for an entry, but got ${entry.runtimeType}',
              entry,
            );
          }
          return DigestEntry(
            filename: entry['filename']! as String,
            width: entry['width']! as int,
            height: entry['height']! as int,
            maxDiffPixelsPercent: entry['maxDiffPixelsPercent']! as double,
            maxColorDelta: entry['maxColorDelta']! as int,
          );
        }),
      ),
    );
  }

  /// A key-value map of dimensions to provide to Skia Gold.
  final Map<String, String> dimensions;

  /// A list of test-run entries.
  final List<DigestEntry> entries;
}

/// A single entry in a `digest.json` file.
///
/// Each entry is a test-run (or part of a test-run).
final class DigestEntry {
  /// Creates a new instance of [DigestEntry].
  const DigestEntry({
    required this.filename,
    required this.width,
    required this.height,
    required this.maxDiffPixelsPercent,
    required this.maxColorDelta,
  });

  /// File path that is a direct sibling of the parsed `digest.json`.
  final String filename;

  /// Width of the image.
  final int width;

  /// Height of the image.
  final int height;

  /// Maximum percentage of different pixels.
  ///
  /// Within Skia Gold, this is called `differentPixelsRate`.
  final double maxDiffPixelsPercent;

  /// Maximum color delta.
  ///
  /// Within Skia Gold, this is called `pixelColorDelta`.
  final int maxColorDelta;
}
