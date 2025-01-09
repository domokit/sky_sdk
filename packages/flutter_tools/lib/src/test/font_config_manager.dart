// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import '../base/file_system.dart';
import '../globals.dart' as globals;

/// Manages a Font configuration that can be shared across multiple tests.
class FontConfigManager {
  Directory? _fontsDirectory;

  /// Returns a Font configuration that limits font fallback to the artifact
  /// cache directory.
  late final File fontConfigFile = () {
    final StringBuffer sb = StringBuffer();
    sb.writeln('<fontconfig>');
    sb.writeln('  <dir>${globals.cache.getCacheArtifacts().path}</dir>');
    sb.writeln('  <cachedir>/var/cache/fontconfig</cachedir>');
    sb.writeln('</fontconfig>');

    if (_fontsDirectory == null) {
      _fontsDirectory = globals.fs.systemTempDirectory.createTempSync('flutter_test_fonts.');
      globals.printTrace('Using this directory for fonts configuration: ${_fontsDirectory!.path}');
    }

    final File cachedFontConfig = globals.fs.file('${_fontsDirectory!.path}/fonts.conf');
    cachedFontConfig.createSync();
    cachedFontConfig.writeAsStringSync(sb.toString());
    return cachedFontConfig;
  }();

  Future<void> dispose() async {
    if (_fontsDirectory != null) {
      globals.printTrace('Deleting ${_fontsDirectory!.path}...');
      try {
        await _fontsDirectory!.delete(recursive: true);
      } on FileSystemException {
        // Silently exit
      }
      _fontsDirectory = null;
    }
  }
}
