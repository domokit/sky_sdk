// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:skia_gold_client/src/release_version.dart';
import 'package:test/test.dart';

void main() {
  test('should accept a major and minor version', () {
    final ReleaseVersion version = ReleaseVersion(major: 3, minor: 21);
    expect(version.major, equals(3));
    expect(version.minor, equals(21));
  });

  test('should accept the string "none"', () {
    final ReleaseVersion? version = ReleaseVersion.parse('none');
    expect(version, isNull);
  });

  test('should fail on a negative major version', () {
    expect(() => ReleaseVersion(major: -1, minor: 0), throwsRangeError);
  });

  test('should fail on a negative minor version', () {
    expect(() => ReleaseVersion(major: 0, minor: -1), throwsRangeError);
  });

  test('should parse a release version from a file', () {
    final ReleaseVersion version = ReleaseVersion.parse('3.21')!;
    expect(version.major, equals(3));
    expect(version.minor, equals(21));
  });

  test('should ignore comments and empty lines', () {
    final ReleaseVersion version =
        ReleaseVersion.parse(<String>['# This is a comment', '', '3.21', ''].join('\n'))!;
    expect(version.major, equals(3));
    expect(version.minor, equals(21));
  });

  test('should fail on a missing version line', () {
    expect(() => ReleaseVersion.parse(''), throwsFormatException);
  });

  test('should fail on a malformed version line', () {
    expect(() => ReleaseVersion.parse('3.21.0'), throwsFormatException);
  });

  test('should fail on a non-integer major version', () {
    expect(() => ReleaseVersion.parse('a.21'), throwsFormatException);
  });

  test('should fail on a non-integer minor version', () {
    expect(() => ReleaseVersion.parse('3.b'), throwsFormatException);
  });

  test('should fail on multiple version lines', () {
    expect(() => ReleaseVersion.parse('3.21\n3.22'), throwsFormatException);
  });

  test('should fail if both "none" and a version are present', () {
    expect(() => ReleaseVersion.parse('none\n3.21'), throwsFormatException);
  });
}
