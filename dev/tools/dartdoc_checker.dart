// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;

/// Makes sure that the path we were given contains some of the expected
/// libraries.
@visibleForTesting
const List<String> dartdocDirectiveCanaryLibraries = <String>[
  'animation',
  'cupertino',
  'material',
  'widgets',
  'rendering',
  'flutter_driver',
];

/// Makes sure that the path we were given contains some of the expected
/// HTML files.
@visibleForTesting
const List<String> dartdocDirectiveCanaryFiles = <String>[
  'Widget-class.html',
  'Material-class.html',
  'Canvas-class.html',
];

/// Scans the dartdoc HTML output in the provided `dartDocDir` for
/// unresolved dartdoc directives (`{@foo x y}`).
///
/// Dartdoc usually replaces those directives with other content. However,
/// if the directive is misspelled (or contains other errors) it is placed
/// verbatim into the HTML output. That's not desirable and this check verifies
/// that no directives appear verbatim in the output by checking that the
/// string `{@` does not appear in the HTML output outside of <code> sections.
///
/// The string `{@` is allowed in <code> sections, because those may contain
/// sample code where the sequence is perfectly legal, e.g. for required named
/// parameters of a method:
///
/// ```dart
/// void foo({@required int bar});
/// ```
void checkForUnresolvedDirectives(Directory dartDocDir) {
  if (!dartDocDir.existsSync()) {
    throw Exception('Directory with dartdoc output (${dartDocDir.path}) does not exist.');
  }

  // Make a copy since this will be mutated
  final List<String> canaryLibraries = dartdocDirectiveCanaryLibraries.toList();
  final List<String> canaryFiles = dartdocDirectiveCanaryFiles.toList();

  print('Scanning for unresolved dartdoc directives...');

  final List<FileSystemEntity> toScan = dartDocDir.listSync();
  int count = 0;

  while (toScan.isNotEmpty) {
    final FileSystemEntity entity = toScan.removeLast();
    if (entity is File) {
      if (path.extension(entity.path) != '.html') {
        continue;
      }
      canaryFiles.remove(path.basename(entity.path));
      count += _scanFile(entity);
    } else if (entity is Directory) {
      canaryLibraries.remove(path.basename(entity.path));
      toScan.addAll(entity.listSync());
    } else {
      throw Exception('$entity is neither file nor directory.');
    }
  }

  if (canaryLibraries.isNotEmpty) {
    throw Exception(
      'Did not find docs for the following libraries: ${canaryLibraries.join(', ')}.',
    );
  }
  if (canaryFiles.isNotEmpty) {
    throw Exception('Did not find docs for the following files: ${canaryFiles.join(', ')}.');
  }
  if (count > 0) {
    throw Exception('Found $count unresolved dartdoc directives (see log above).');
  }
  print('No unresolved dartdoc directives detected.');
}

int _scanFile(File file) {
  assert(path.extension(file.path) == '.html');
  final Iterable<String> matches = _pattern
      .allMatches(file.readAsStringSync())
      .map((RegExpMatch m) => m.group(0)!);

  if (matches.isNotEmpty) {
    stderr.writeln('Found unresolved dartdoc directives in ${file.path}:');
    for (final String match in matches) {
      stderr.writeln('  $match');
    }
  }
  return matches.length;
}

// Matches all `{@` that are not within `<code></code>` sections.
//
// This regex may lead to false positives if the docs ever contain nested tags
// inside <code> sections. Since we currently don't do that, doing the matching
// with a regex is a lot faster than using an HTML parser to strip out the
// <code> sections.
final RegExp _pattern = RegExp(r'({@[^}\n]*}?)(?![^<>]*</code)');

// Usually, the checker is invoked directly from `dartdoc.dart`. Main method
// is included for convenient local runs without having to regenerate
// the dartdocs every time.
//
// Provide the path to the dartdoc HTML output as an argument when running the
// program.
void main(List<String> args) {
  if (args.length != 1) {
    throw Exception('Must provide the path to the dartdoc HTML output as argument.');
  }
  if (!Directory(args.single).existsSync()) {
    throw Exception('The dartdoc HTML output directory ${args.single} does not exist.');
  }
  checkForUnresolvedDirectives(Directory(args.single));
}
