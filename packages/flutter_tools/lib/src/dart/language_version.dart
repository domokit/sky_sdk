// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:package_config/package_config.dart';

import '../base/version.dart';

final RegExp _languageVersion = RegExp(r'\/\/\s*@dart\s*=\s*([0-9])\.([0-9]+)');
final RegExp _declarationEnd = RegExp('(import)|(library)|(part)');
const String _blockCommentStart = '/*';
const String _blockCommentEnd = '*/';

/// The first language version where null safety was available by default.
final LanguageVersion nullSafeVersion = LanguageVersion(2, 12);

LanguageVersion? _currentLanguageVersion;

/// Lookup the current Dart language version.
LanguageVersion currentLanguageVersion(FileSystem fileSystem, String flutterRoot) {
  if (_currentLanguageVersion != null) {
    return _currentLanguageVersion!;
  }
  // Either reading the file or parsing the version could fail on a corrupt Dart SDK.
  // let it crash so it shows up in crash logging.
  final File versionFile = fileSystem.file(
    fileSystem.path.join(flutterRoot, 'bin', 'cache', 'dart-sdk', 'version'),
  );
  if (!versionFile.existsSync() && _inUnitTest()) {
    return LanguageVersion(2, 12);
  }
  final Version version = Version.parse(versionFile.readAsStringSync())!;
  return _currentLanguageVersion = LanguageVersion(version.major, version.minor);
}

// Whether the tool is executing in a unit test.
bool _inUnitTest() {
  return Zone.current[#test.declarer] != null;
}

/// Attempts to read the language version of a dart [file].
///
/// If this is not present, falls back to the language version defined in
/// [package]. If [package] is not provided and there is no
/// language version header, returns 2.12. This does not specifically check
/// for language declarations other than library, part, or import.
///
/// The specification for the language version tag is defined at:
/// https://github.com/dart-lang/language/blob/main/accepted/2.8/language-versioning/feature-specification.md#individual-library-language-version-override
LanguageVersion determineLanguageVersion(File file, Package? package, String flutterRoot) {
  int blockCommentDepth = 0;
  // If reading the file fails, default to a null-safe version. The
  // command will likely fail later in the process with a better error
  // message.
  List<String> lines;
  try {
    lines = file.readAsLinesSync();
  } on FileSystemException {
    return currentLanguageVersion(file.fileSystem, flutterRoot);
  }

  for (final String line in lines) {
    final String trimmedLine = line.trim();
    if (trimmedLine.isEmpty) {
      continue;
    }
    // Check for the start or end of a block comment. Within a block
    // comment, all language version declarations are ignored. Block
    // comments can be nested, and the start or end may occur on
    // the same line. This does not handle the case of invalid
    // block comment combinations like `*/ /*` since that will cause
    // a compilation error anyway.
    bool sawBlockComment = false;
    final int startMatches = _blockCommentStart.allMatches(trimmedLine).length;
    final int endMatches = _blockCommentEnd.allMatches(trimmedLine).length;
    if (startMatches > 0) {
      blockCommentDepth += startMatches;
      sawBlockComment = true;
    }
    if (endMatches > 0) {
      blockCommentDepth -= endMatches;
      sawBlockComment = true;
    }
    if (blockCommentDepth != 0 || sawBlockComment) {
      continue;
    }
    // Check for a match with the language version.
    final Match? match = _languageVersion.matchAsPrefix(trimmedLine);
    if (match != null) {
      final String rawMajor = match.group(1) ?? '';
      final String rawMinor = match.group(2) ?? '';
      try {
        final int major = int.parse(rawMajor);
        final int minor = int.parse(rawMinor);
        return LanguageVersion(major, minor);
      } on FormatException {
        // Language comment was invalid in a way that the regexp did not
        // anticipate.
        break;
      }
    }

    // Check for a declaration which ends the search for a language
    // version.
    if (_declarationEnd.matchAsPrefix(trimmedLine) != null) {
      break;
    }
  }

  // If the language version cannot be found, use the package version.
  if (package != null) {
    return package.languageVersion ?? currentLanguageVersion(file.fileSystem, flutterRoot);
  }
  // Default to current version.
  return currentLanguageVersion(file.fileSystem, flutterRoot);
}
