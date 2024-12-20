// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:core' as core show RegExp;
import 'dart:core' hide RegExp;
import 'dart:io';

class RegExp implements core.RegExp {
  RegExp(
    String source, {
    bool multiLine = false,
    bool caseSensitive = true,
    bool unicode = false,
    bool dotAll = false,
    this.expectNoMatch = false,
  }) : _pattern = core.RegExp(
         source,
         multiLine: multiLine,
         caseSensitive: caseSensitive,
         unicode: unicode,
         dotAll: dotAll,
       ),
       source = _stripFrameNumber(
         StackTrace.current.toString().split('\n').skip(1).take(1).single,
       ) {
    _allPatterns.add(this);
  }

  static String _stripFrameNumber(String frame) {
    return frame.substring(frame.indexOf(' ')).trim();
  }

  final core.RegExp _pattern;
  final String source;

  static final List<RegExp> _allPatterns = <RegExp>[];

  final bool expectNoMatch;

  int _matchCount = 0;
  int get matchCount => _matchCount;

  int _testCount = 0;
  int get testCount => _testCount;

  final Stopwatch _stopwatch = Stopwatch();

  static void printDiagnostics() {
    final List<RegExp> patterns = _allPatterns.toList();
    stderr.writeln('Top ten patterns:');
    patterns.sort((RegExp a, RegExp b) => b._stopwatch.elapsed.compareTo(a._stopwatch.elapsed));
    for (final RegExp pattern in patterns.take(10)) {
      stderr.writeln(
        '${pattern._stopwatch.elapsedMicroseconds.toString().padLeft(10)}μs tests -- /${pattern.pattern}/ (${pattern.testCount} tests, ${pattern.matchCount} matches, ${pattern.source})',
      );
    }
    stderr.writeln();
    stderr.writeln('Unmatched patterns:');
    patterns.sort((RegExp a, RegExp b) => a.pattern.compareTo(b.pattern));
    for (final RegExp pattern in patterns) {
      if (pattern.matchCount == 0 && !pattern.expectNoMatch && pattern.testCount > 0) {
        stderr.writeln(
          '/${pattern.pattern}/ (${pattern.testCount} tests, ${pattern.matchCount} matches, ${pattern.source})',
        );
      }
    }
    stderr.writeln();
    stderr.writeln('Unexpectedly matched patterns:');
    for (final RegExp pattern in patterns) {
      if (pattern.matchCount > 0 && pattern.expectNoMatch) {
        stderr.writeln(
          '/${pattern.pattern}/ (${pattern.testCount} tests, ${pattern.matchCount} matches, ${pattern.source})',
        );
      }
    }
    stderr.writeln();
    stderr.writeln('Unused patterns:');
    for (final RegExp pattern in patterns) {
      if (pattern.testCount == 0) {
        stderr.writeln(
          '/${pattern.pattern}/ (${pattern.testCount} tests, ${pattern.matchCount} matches, ${pattern.source})',
        );
      }
    }
  }

  @override
  bool get isCaseSensitive => _pattern.isCaseSensitive;

  @override
  bool get isDotAll => _pattern.isDotAll;

  @override
  bool get isMultiLine => _pattern.isMultiLine;

  @override
  bool get isUnicode => _pattern.isUnicode;

  @override
  String get pattern => _pattern.pattern;

  @override
  Iterable<RegExpMatch> allMatches(String input, [int start = 0]) {
    _stopwatch.start();
    final List<RegExpMatch> result = _pattern.allMatches(input, start).toList();
    _stopwatch.stop();
    _testCount += 1;
    if (result.isNotEmpty) {
      _matchCount += 1;
    }
    return result;
  }

  @override
  RegExpMatch? firstMatch(String input) {
    _stopwatch.start();
    final RegExpMatch? result = _pattern.firstMatch(input);
    _stopwatch.stop();
    _testCount += 1;
    if (result != null) {
      _matchCount += 1;
    }
    return result;
  }

  @override
  bool hasMatch(String input) {
    _stopwatch.start();
    final bool result = _pattern.hasMatch(input);
    _stopwatch.stop();
    _stopwatch.stop();
    _testCount += 1;
    if (result) {
      _matchCount += 1;
    }
    return result;
  }

  @override
  Match? matchAsPrefix(String string, [int start = 0]) {
    _stopwatch.start();
    final Match? result = _pattern.matchAsPrefix(string, start);
    _stopwatch.stop();
    _testCount += 1;
    if (result != null) {
      _matchCount += 1;
    }
    return result;
  }

  @override
  String? stringMatch(String input) {
    _stopwatch.start();
    final String? result = _pattern.stringMatch(input);
    _stopwatch.stop();
    _testCount += 1;
    if (result != null) {
      _matchCount += 1;
    }
    return result;
  }

  @override
  String toString() => _pattern.toString();
}
