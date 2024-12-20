// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// The utility function `encodeKnArbFiles` replaces the material_kn.arb
// and cupertino_kn.arb files in flutter_localizations/packages/lib/src/l10n
// with versions where the contents of the localized strings have been
// replaced by JSON escapes. This is done because some of those strings
// contain characters that can crash Emacs on Linux. There is more information
// here: https://github.com/flutter/flutter/issues/36704 and in the README
// in flutter_localizations/packages/lib/src/l10n.
//
// This utility is run by `gen_localizations.dart` if --overwrite is passed
// in as an option.

import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import '../localizations_utils.dart';

Map<String, dynamic> _loadBundle(File file) {
  if (!FileSystemEntity.isFileSync(file.path)) {
    exitWithError('Unable to find input file: ${file.path}');
  }
  return json.decode(file.readAsStringSync()) as Map<String, dynamic>;
}

void _encodeBundleTranslations(Map<String, dynamic> bundle) {
  for (final String key in bundle.keys) {
    // The ARB file resource "attributes" for foo are called @foo. Don't need
    // to encode them.
    if (key.startsWith('@')) {
      continue;
    }
    final String translation = bundle[key] as String;
    // Rewrite the string as a series of unicode characters in JSON format.
    // Like "\u0012\u0123\u1234".
    bundle[key] =
        translation.runes.map((int code) {
          final String codeString = '00${code.toRadixString(16)}';
          return '\\u${codeString.substring(codeString.length - 4)}';
        }).join();
  }
}

void _checkEncodedTranslations(Map<String, dynamic> encodedBundle, Map<String, dynamic> bundle) {
  bool errorFound = false;
  const JsonDecoder decoder = JsonDecoder();
  for (final String key in bundle.keys) {
    if (decoder.convert('"${encodedBundle[key]}"') != bundle[key]) {
      stderr.writeln(
        '  encodedTranslation for $key does not match original value "${bundle[key]}"',
      );
      errorFound = true;
    }
  }
  if (errorFound) {
    exitWithError('JSON unicode translation encoding failed');
  }
}

void _rewriteBundle(File file, Map<String, dynamic> bundle) {
  final StringBuffer contents = StringBuffer();
  contents.writeln('{');
  for (final String key in bundle.keys) {
    contents.writeln('  "$key": "${bundle[key]}"${key == bundle.keys.last ? '' : ','}');
  }
  contents.writeln('}');
  file.writeAsStringSync(contents.toString());
}

void encodeKnArbFiles(Directory directory) {
  final File widgetsArbFile = File(path.join(directory.path, 'widgets_kn.arb'));
  final File materialArbFile = File(path.join(directory.path, 'material_kn.arb'));
  final File cupertinoArbFile = File(path.join(directory.path, 'cupertino_kn.arb'));

  final Map<String, dynamic> widgetsBundle = _loadBundle(widgetsArbFile);
  final Map<String, dynamic> materialBundle = _loadBundle(materialArbFile);
  final Map<String, dynamic> cupertinoBundle = _loadBundle(cupertinoArbFile);

  _encodeBundleTranslations(widgetsBundle);
  _encodeBundleTranslations(materialBundle);
  _encodeBundleTranslations(cupertinoBundle);

  _checkEncodedTranslations(widgetsBundle, _loadBundle(widgetsArbFile));
  _checkEncodedTranslations(materialBundle, _loadBundle(materialArbFile));
  _checkEncodedTranslations(cupertinoBundle, _loadBundle(cupertinoArbFile));

  _rewriteBundle(widgetsArbFile, widgetsBundle);
  _rewriteBundle(materialArbFile, materialBundle);
  _rewriteBundle(cupertinoArbFile, cupertinoBundle);
}
