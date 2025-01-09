// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Encodes ARB file resource values with Unicode escapes.
void encodeBundleTranslations(Map<String, dynamic> bundle) {
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

String generateArbString(Map<String, dynamic> bundle) {
  final StringBuffer contents = StringBuffer();
  contents.writeln('{');
  for (final String key in bundle.keys) {
    contents.writeln('  "$key": "${bundle[key]}"${key == bundle.keys.last ? '' : ','}');
  }
  contents.writeln('}');
  return contents.toString();
}
