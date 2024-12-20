// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

// It's bad to import a file from `bin` into `tool`.
// However this tool is not very important, so delete it if necessary.
import '../bin/utils/adb_logcat_filtering.dart';

/// A tiny tool to read saved `adb logcat` output and perform some analysis.
///
/// This tool is not meant to be a full-fledged logcat reader. It's just a
/// simple tool that uses the [AdbLogLine] extension type to parse results of
/// `adb logcat` and explain what log tag names are most common.
void main(List<String> args) {
  if (args case [final String path]) {
    final List<AdbLogLine> parsed =
        io.File(path)
            .readAsLinesSync()
            .map(AdbLogLine.tryParse)
            .whereType<AdbLogLine>()
            // Filter out all debug logs.
            .where((AdbLogLine line) => line.severity != 'D')
            .toList();

    final Map<String, int> tagCounts = <String, int>{};
    for (final AdbLogLine line in parsed) {
      tagCounts[line.name] = (tagCounts[line.name] ?? 0) + 1;
    }

    // Print in order of most common to least common.
    final List<MapEntry<String, int>> sorted =
        tagCounts.entries.toList()
          ..sort((MapEntry<String, int> a, MapEntry<String, int> b) => b.value.compareTo(a.value));
    for (final MapEntry<String, int> entry in sorted) {
      print("'${entry.key}', // ${entry.value}");
    }

    return;
  }

  print('Usage: logcat_reader.dart <path-to-logcat-output>');
  io.exitCode = 1;
}
