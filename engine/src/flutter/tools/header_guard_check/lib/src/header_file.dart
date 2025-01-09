// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:meta/meta.dart';
import 'package:path/path.dart' as p;
import 'package:source_span/source_span.dart';

/// Represents a C++ header file, i.e. a file on disk that ends in `.h`.
@immutable
final class HeaderFile {
  /// Creates a new header file from the given [path].
  const HeaderFile.from(this.path, {required this.guard, required this.pragmaOnce});

  /// Parses the given [path] as a header file.
  ///
  /// Throws an [ArgumentError] if the file does not exist.
  factory HeaderFile.parse(String path) {
    final io.File file = io.File(path);
    if (!file.existsSync()) {
      throw ArgumentError.value(path, 'path', 'File does not exist.');
    }

    final String contents = file.readAsStringSync();
    final SourceFile sourceFile = SourceFile.fromString(contents, url: p.toUri(path));
    return HeaderFile.from(
      path,
      guard: _parseGuard(sourceFile),
      pragmaOnce: _parsePragmaOnce(sourceFile),
    );
  }

  static ({int start, int end, String line}) _getLine(SourceFile sourceFile, int index) {
    final int start = sourceFile.getOffset(index);
    int end =
        index == sourceFile.lines - 1 ? sourceFile.length : sourceFile.getOffset(index + 1) - 1;
    String line = sourceFile.getText(start, end);

    // On Windows, it's common for files to have CRLF line endings, and for
    // developers to use git's `core.autocrlf` setting to convert them to LF
    // line endings.
    //
    // However, our scripts expect LF line endings, so we need to remove the
    // CR characters from the line endings when computing the line so that
    // properly formatted files are not considered malformed.
    if (line.isNotEmpty && sourceFile.getText(end - 1, end) == '\r') {
      end--;
      line = line.substring(0, line.length - 1);
    }

    return (start: start, end: end, line: line);
  }

  /// Parses the header guard of the given [sourceFile].
  static HeaderGuardSpans? _parseGuard(SourceFile sourceFile) {
    SourceSpan? ifndefSpan;
    SourceSpan? defineSpan;
    SourceSpan? endifSpan;

    // Iterate over the lines in the file.
    for (int i = 0; i < sourceFile.lines; i++) {
      final (:int start, :int end, :String line) = _getLine(sourceFile, i);

      // Check if the line is a header guard directive.
      if (line.startsWith('#ifndef')) {
        ifndefSpan = sourceFile.span(start, end);
      } else if (line.startsWith('#define')) {
        // If we find a define preceding an ifndef, it is not a header guard.
        if (ifndefSpan == null) {
          continue;
        }
        defineSpan = sourceFile.span(start, end);
        break;
      }
    }

    // If we found no header guard, return null.
    if (ifndefSpan == null) {
      return null;
    }

    // Now iterate backwards to find the (last) #endif directive.
    for (int i = sourceFile.lines - 1; i > 0; i--) {
      final (:int start, :int end, :String line) = _getLine(sourceFile, i);

      // Check if the line is a header guard directive.
      if (line.startsWith('#endif')) {
        endifSpan = sourceFile.span(start, end);
        break;
      }
    }

    return HeaderGuardSpans(ifndefSpan: ifndefSpan, defineSpan: defineSpan, endifSpan: endifSpan);
  }

  /// Parses the `#pragma once` directive of the given [sourceFile].
  static SourceSpan? _parsePragmaOnce(SourceFile sourceFile) {
    // Iterate over the lines in the file.
    for (int i = 0; i < sourceFile.lines; i++) {
      final (:int start, :int end, :String line) = _getLine(sourceFile, i);

      // Check if the line is a header guard directive.
      if (line.startsWith('#pragma once')) {
        return sourceFile.span(start, end);
      }
    }

    return null;
  }

  /// Path to the file on disk.
  final String path;

  /// The header guard span, if any.
  ///
  /// This is `null` if the file does not have a header guard.
  final HeaderGuardSpans? guard;

  /// The `#pragma once` directive, if any.
  ///
  /// This is `null` if the file does not have a `#pragma once` directive.
  final SourceSpan? pragmaOnce;

  static final RegExp _nonAlphaNumeric = RegExp(r'[^a-zA-Z0-9]');

  /// Returns the expected header guard for this file, relative to [engineRoot].
  ///
  /// For example, if the file is `foo/bar/baz.h`, this will return `FLUTTER_FOO_BAR_BAZ_H_`.
  String computeExpectedName({required String engineRoot}) {
    final String relativePath = p.relative(path, from: engineRoot);
    final String underscoredRelativePath = p
        .withoutExtension(relativePath)
        .replaceAll(_nonAlphaNumeric, '_');
    return 'FLUTTER_${underscoredRelativePath.toUpperCase()}_H_';
  }

  /// Updates the file at [path] to have the expected header guard.
  ///
  /// Returns `true` if the file was modified, `false` otherwise.
  bool fix({required String engineRoot}) {
    final String expectedGuard = computeExpectedName(engineRoot: engineRoot);

    // Check if the file already has a valid header guard.
    if (guard != null) {
      if (guard!.ifndefValue == expectedGuard &&
          guard!.defineValue == expectedGuard &&
          guard!.endifValue == expectedGuard) {
        return false;
      }
    }

    // Get the contents of the file.
    final String oldContents = io.File(path).readAsStringSync();

    // If we're using pragma once, replace it with an ifndef/define, and
    // append an endif and a newline at the end of the file.
    if (pragmaOnce != null) {
      // Append the endif and newline.
      String newContents = '$oldContents\n#endif  // $expectedGuard\n';

      // Replace the span with the ifndef/define.
      newContents = newContents.replaceRange(
        pragmaOnce!.start.offset,
        pragmaOnce!.end.offset,
        '#ifndef $expectedGuard\n'
        '#define $expectedGuard',
      );

      // Write the new contents to the file.
      io.File(path).writeAsStringSync(newContents);
      return true;
    }

    // If we're not using pragma once, replace the header guard with the
    // expected header guard.
    if (guard != null) {
      // Replace endif:
      String newContents = oldContents.replaceRange(
        guard!.endifSpan!.start.offset,
        guard!.endifSpan!.end.offset,
        '#endif  // $expectedGuard',
      );

      // Replace define:
      newContents = newContents.replaceRange(
        guard!.defineSpan!.start.offset,
        guard!.defineSpan!.end.offset,
        '#define $expectedGuard',
      );

      // Replace ifndef:
      newContents = newContents.replaceRange(
        guard!.ifndefSpan!.start.offset,
        guard!.ifndefSpan!.end.offset,
        '#ifndef $expectedGuard',
      );

      // Write the new contents to the file.
      io.File(path).writeAsStringSync('$newContents\n');
      return true;
    }

    // If we're missing a guard entirely, add one. The rules are:
    // 1. Add a newline, #endif at the end of the file.
    // 2. Add a newline, #ifndef, #define after the first non-comment line.
    String newContents = oldContents;
    newContents += '\n#endif  // $expectedGuard\n';
    newContents = newContents.replaceFirst(
      RegExp(r'^(?!//)', multiLine: true),
      '\n#ifndef $expectedGuard\n'
      '#define $expectedGuard\n',
    );

    // Write the new contents to the file.
    io.File(path).writeAsStringSync(newContents);
    return true;
  }

  @override
  bool operator ==(Object other) {
    return other is HeaderFile &&
        path == other.path &&
        guard == other.guard &&
        pragmaOnce == other.pragmaOnce;
  }

  @override
  int get hashCode => Object.hash(path, guard, pragmaOnce);

  @override
  String toString() {
    return 'HeaderFile(\n'
        '  path:       $path\n'
        '  guard:      $guard\n'
        '  pragmaOnce: $pragmaOnce\n'
        ')';
  }
}

/// Source elements that are part of a header guard.
@immutable
final class HeaderGuardSpans {
  /// Collects the source spans of the header guard directives.
  const HeaderGuardSpans({
    required this.ifndefSpan,
    required this.defineSpan,
    required this.endifSpan,
  });

  /// Location of the `#ifndef` directive.
  final SourceSpan? ifndefSpan;

  /// Location of the `#define` directive.
  final SourceSpan? defineSpan;

  /// Location of the `#endif` directive.
  final SourceSpan? endifSpan;

  @override
  bool operator ==(Object other) {
    return other is HeaderGuardSpans &&
        ifndefSpan == other.ifndefSpan &&
        defineSpan == other.defineSpan &&
        endifSpan == other.endifSpan;
  }

  @override
  int get hashCode => Object.hash(ifndefSpan, defineSpan, endifSpan);

  @override
  String toString() {
    return 'HeaderGuardSpans(\n'
        '  #ifndef: $ifndefSpan\n'
        '  #define: $defineSpan\n'
        '  #endif:  $endifSpan\n'
        ')';
  }

  /// Returns the value of the `#ifndef` directive.
  ///
  /// For example, `#ifndef FOO_H_`, this will return `FOO_H_`.
  ///
  /// If the span is not a valid `#ifndef` directive, `null` is returned.
  String? get ifndefValue {
    final String? value = ifndefSpan?.text;
    if (value == null) {
      return null;
    }
    if (!value.startsWith('#ifndef ')) {
      return null;
    }
    return value.substring('#ifndef '.length);
  }

  /// Returns the value of the `#define` directive.
  ///
  /// For example, `#define FOO_H_`, this will return `FOO_H_`.
  ///
  /// If the span is not a valid `#define` directive, `null` is returned.
  String? get defineValue {
    final String? value = defineSpan?.text;
    if (value == null) {
      return null;
    }
    if (!value.startsWith('#define ')) {
      return null;
    }
    return value.substring('#define '.length);
  }

  /// Returns the value of the `#endif` directive.
  ///
  /// For example, `#endif  // FOO_H_`, this will return `FOO_H_`.
  ///
  /// If the span is not a valid `#endif` directive, `null` is returned.
  String? get endifValue {
    final String? value = endifSpan?.text;
    if (value == null) {
      return null;
    }
    if (!value.startsWith('#endif  // ')) {
      return null;
    }
    return value.substring('#endif  // '.length);
  }
}
