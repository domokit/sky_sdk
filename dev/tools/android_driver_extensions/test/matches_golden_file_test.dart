// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:android_driver_extensions/native_driver.dart';
// Similar to `flutter_test`, we ignore the implementation import.
// ignore: implementation_imports
import 'package:matcher/src/expect/async_matcher.dart';
import 'package:test/fake.dart';
import 'package:test/test.dart';

void main() {
  test('passes when the comparator passes', () async {
    goldenFileComparator = _FakeGoldenFileComparator(_CannedComparisonMode.alwaysPass);
    final AsyncMatcher matcher = matchesGoldenFile('test');
    await expectLater(matcher.matchAsync(Uint8List(0)), completion(isNull));
  });

  test('fails with a default message when the comparator fails', () async {
    goldenFileComparator = _FakeGoldenFileComparator(_CannedComparisonMode.alwaysFail);
    final AsyncMatcher matcher = matchesGoldenFile('test');
    await expectLater(matcher.matchAsync(Uint8List(0)), completion(contains('does not match')));
  });

  test('fails when the comparator throws a TestFailure', () async {
    goldenFileComparator = _FakeGoldenFileComparator(_CannedComparisonMode.alwaysThrowTestFailure);
    final AsyncMatcher matcher = matchesGoldenFile('test');
    await expectLater(matcher.matchAsync(Uint8List(0)), completion(contains('An expected error')));
  });

  test('unhandled exception when the comparator throws anything but TestFailure', () async {
    goldenFileComparator = _FakeGoldenFileComparator(_CannedComparisonMode.alwaysThrowStateError);
    final AsyncMatcher matcher = matchesGoldenFile('test');
    await expectLater(matcher.matchAsync(Uint8List(0)), throwsStateError);
  });
}

final class _FakeGoldenFileComparator extends Fake implements GoldenFileComparator {
  _FakeGoldenFileComparator(this._mode);
  final _CannedComparisonMode _mode;

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    return switch (_mode) {
      _CannedComparisonMode.alwaysPass => true,
      _CannedComparisonMode.alwaysFail => false,
      _CannedComparisonMode.alwaysThrowTestFailure => throw TestFailure('An expected error'),
      _CannedComparisonMode.alwaysThrowStateError => throw StateError('An unexpected error'),
    };
  }

  @override
  Uri getTestUri(Uri key, int? version) => key;
}

enum _CannedComparisonMode { alwaysPass, alwaysFail, alwaysThrowTestFailure, alwaysThrowStateError }
