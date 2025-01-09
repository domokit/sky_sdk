// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:math' as math;

import 'package:collection/collection.dart';
import 'package:test/test.dart';

import '../utils.dart';

export 'package:test/test.dart' hide isInstanceOf;

/// A matcher that compares the type of the actual value to the type argument T.
TypeMatcher<T> isInstanceOf<T>() => isA<T>();

void tryToDelete(Directory directory) {
  // This should not be necessary, but it turns out that
  // on Windows it's common for deletions to fail due to
  // bogus (we think) "access denied" errors.
  try {
    directory.deleteSync(recursive: true);
  } on FileSystemException catch (error) {
    print('Failed to delete ${directory.path}: $error');
  }
}

Matcher throwsExceptionWith(String messageSubString) {
  return throwsA(
    isA<Exception>().having(
      (Exception e) => e.toString(),
      'description',
      contains(messageSubString),
    ),
  );
}

/// A matcher that matches error messages specified in the given `fixture` [File].
///
/// This matcher allows analyzer tests to specify the expected error messages
/// in the test fixture file, eliminating the need to hard code line numbers in
/// the test.
///
/// The error messages must be printed using the [foundError] function. Each
/// error must start with the path to the file where the error resides, line
/// number (1-based instead of 0-based) of the error, and a short description,
/// delimited by colons (`:`). In the test fixture one could add the following
/// comment on the line that would produce the error, to tell matcher what to
/// expect:
/// `// ERROR: <error message without the leading path and line number>`.
Matcher matchesErrorsInFile(File fixture, {List<String> endsWith = const <Never>[]}) =>
    _ErrorMatcher(fixture, endsWith);

class _ErrorMatcher extends Matcher {
  _ErrorMatcher(this.file, this.endsWith) : bodyMatcher = _ErrorsInFileMatcher(file);

  static const String mismatchDescriptionKey = 'mismatchDescription';
  static final int _errorBoxWidth = math.max(15, (hasColor ? stdout.terminalColumns : 80) - 1);
  static const String _title = 'ERROR #1';
  static final String _firstLine = '╔═╡$_title╞═${"═" * (_errorBoxWidth - 4 - _title.length)}';

  static final String _lastLine = '╚${"═" * _errorBoxWidth}';
  static const String _linePrefix = '║ ';

  static bool mismatch(String mismatchDescription, Map<dynamic, dynamic> matchState) {
    matchState[mismatchDescriptionKey] = mismatchDescription;
    return false;
  }

  final List<String> endsWith;
  final File file;
  final _ErrorsInFileMatcher bodyMatcher;

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    if (item is! String) {
      return mismatch('expected a String, got $item', matchState);
    }
    final List<String> lines = item.split('\n');
    if (lines.isEmpty) {
      return mismatch('the actual error message is empty', matchState);
    }
    if (lines.first != _firstLine) {
      return mismatch(
        'the first line of the error message must be $_firstLine, got ${lines.first}',
        matchState,
      );
    }
    if (lines.last.isNotEmpty) {
      return mismatch(
        'missing newline at the end of the error message, got ${lines.last}',
        matchState,
      );
    }
    if (lines[lines.length - 2] != _lastLine) {
      return mismatch(
        'the last line of the error message must be $_lastLine, got ${lines[lines.length - 2]}',
        matchState,
      );
    }
    final List<String> body = lines.sublist(1, lines.length - 2);
    final String? noprefix = body.firstWhereOrNull((String line) => !line.startsWith(_linePrefix));
    if (noprefix != null) {
      return mismatch(
        'Line "$noprefix" should start with a prefix $_linePrefix..\n$lines',
        matchState,
      );
    }

    final List<String> bodyWithoutPrefix = body
        .map((String s) => s.substring(_linePrefix.length))
        .toList(growable: false);
    final bool hasTailMismatch = IterableZip<String>(<Iterable<String>>[
      bodyWithoutPrefix.reversed,
      endsWith.reversed,
    ]).any((List<String> ss) => ss[0] != ss[1]);
    if (bodyWithoutPrefix.length < endsWith.length || hasTailMismatch) {
      return mismatch(
        'The error message should end with $endsWith.\n'
        'Actual error(s): $item',
        matchState,
      );
    }
    return bodyMatcher.matches(
      bodyWithoutPrefix.sublist(0, bodyWithoutPrefix.length - endsWith.length),
      matchState,
    );
  }

  @override
  Description describe(Description description) {
    return description.add('file ${file.path} contains the expected analyze errors.');
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    final String? description = matchState[mismatchDescriptionKey] as String?;
    return description != null
        ? mismatchDescription.add(description)
        : mismatchDescription.add('$matchState');
  }
}

class _ErrorsInFileMatcher extends Matcher {
  _ErrorsInFileMatcher(this.file);

  final File file;

  static final RegExp expectationMatcher = RegExp(r'// ERROR: (?<expectations>.+)$');

  static bool mismatch(String mismatchDescription, Map<dynamic, dynamic> matchState) {
    return _ErrorMatcher.mismatch(mismatchDescription, matchState);
  }

  List<(int, String)> _expectedErrorMessagesFromFile(Map<dynamic, dynamic> matchState) {
    final List<(int, String)> returnValue = <(int, String)>[];
    for (final (int index, String line) in file.readAsLinesSync().indexed) {
      final List<String> expectations =
          expectationMatcher.firstMatch(line)?.namedGroup('expectations')?.split(' // ERROR: ') ??
          <String>[];
      for (final String expectation in expectations) {
        returnValue.add((index + 1, expectation));
      }
    }
    return returnValue;
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    final List<String> actualErrors = item as List<String>;
    final List<(int, String)> expectedErrors = _expectedErrorMessagesFromFile(matchState);
    if (expectedErrors.length != actualErrors.length) {
      return mismatch(
        'expected ${expectedErrors.length} error(s), got ${actualErrors.length}.\n'
        'expected lines with errors: ${expectedErrors.map(((int, String) x) => x.$1).toList()}\n'
        'actual error(s): \n>${actualErrors.join('\n>')}',
        matchState,
      );
    }
    for (int i = 0; i < actualErrors.length; ++i) {
      final String actualError = actualErrors[i];
      final (int lineNumber, String expectedError) = expectedErrors[i];
      switch (actualError.split(':')) {
        case [final String _]:
          return mismatch('No colons (":") found in the error message "$actualError".', matchState);
        case [final String path, final String line, ...final List<String> rest]:
          if (!path.endsWith(file.uri.pathSegments.last)) {
            return mismatch('"$path" does not match the file name of the source file.', matchState);
          }
          if (lineNumber.toString() != line) {
            return mismatch(
              'could not find the expected error "$expectedError" at line $lineNumber',
              matchState,
            );
          }
          final String actualMessage = rest.join(':').trimLeft();
          if (actualMessage != expectedError) {
            return mismatch(
              'expected \n"$expectedError"\n at line $lineNumber, got \n"$actualMessage"',
              matchState,
            );
          }

        case _:
          return mismatch(
            'failed to recognize a valid path from the error message "$actualError".',
            matchState,
          );
      }
    }
    return true;
  }

  @override
  Description describe(Description description) => description;
}
