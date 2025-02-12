// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:android_driver_extensions/native_driver.dart';
import 'package:android_driver_extensions/skia_gold.dart';
import 'package:file/src/interface/file.dart';
import 'package:flutter_goldens/skia_client.dart';
import 'package:test/fake.dart';
import 'package:test/test.dart';

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/163051.
  test('converts SkiaException to TestFailure in postsubmit', () async {
    final SkiaGoldClient skiaGold = _ThrowsSkiaException();
    await enableSkiaGoldComparatorForTesting(skiaGold, presubmit: false);

    await expectLater(
      goldenFileComparator.compare(Uint8List(0), Uri(path: 'test.png')),
      throwsA(
        isA<TestFailure>().having(
          (Object e) => '$e',
          'description',
          contains('Skia Gold received an unapproved image in post'),
        ),
      ),
    );
  });
}

final class _ThrowsSkiaException extends Fake implements SkiaGoldClient {
  @override
  Future<void> auth() async {}

  @override
  Future<void> imgtestInit() async {}

  @override
  Future<bool> imgtestAdd(String testName, File goldenFile) async {
    throw const SkiaException('Skia Gold received an unapproved image in post');
  }
}
