// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import '../rendering/rendering_tester.dart';
import 'mocks_for_image_cache.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  tearDown(() {
    imageCache
      ..clear()
      ..maximumSize = 1000
      ..maximumSizeBytes = 10485760;
  });

  test('Image cache resizing based on count', () async {
    imageCache.maximumSize = 2;

    final TestImageInfo a =
        await extractOneFrame(
              TestImageProvider(
                1,
                1,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    final TestImageInfo b =
        await extractOneFrame(
              TestImageProvider(
                2,
                2,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    final TestImageInfo c =
        await extractOneFrame(
              TestImageProvider(
                3,
                3,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    final TestImageInfo d =
        await extractOneFrame(
              TestImageProvider(
                1,
                4,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(a.value, equals(1));
    expect(b.value, equals(2));
    expect(c.value, equals(3));
    expect(d.value, equals(4));

    imageCache.maximumSize = 0;

    final TestImageInfo e =
        await extractOneFrame(
              TestImageProvider(
                1,
                5,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(e.value, equals(5));

    final TestImageInfo f =
        await extractOneFrame(
              TestImageProvider(
                1,
                6,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(f.value, equals(6));

    imageCache.maximumSize = 3;

    final TestImageInfo g =
        await extractOneFrame(
              TestImageProvider(
                1,
                7,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(g.value, equals(7));

    final TestImageInfo h =
        await extractOneFrame(
              TestImageProvider(
                1,
                8,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(h.value, equals(7));
  });

  test('Image cache resizing based on size', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8); // 256 B.
    imageCache.maximumSizeBytes = 256 * 2;

    final TestImageInfo a =
        await extractOneFrame(
              TestImageProvider(1, 1, image: testImage).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    final TestImageInfo b =
        await extractOneFrame(
              TestImageProvider(2, 2, image: testImage).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    final TestImageInfo c =
        await extractOneFrame(
              TestImageProvider(3, 3, image: testImage).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    final TestImageInfo d =
        await extractOneFrame(
              TestImageProvider(1, 4, image: testImage).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(a.value, equals(1));
    expect(b.value, equals(2));
    expect(c.value, equals(3));
    expect(d.value, equals(4));

    imageCache.maximumSizeBytes = 0;

    final TestImageInfo e =
        await extractOneFrame(
              TestImageProvider(1, 5, image: testImage).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(e.value, equals(5));

    final TestImageInfo f =
        await extractOneFrame(
              TestImageProvider(1, 6, image: testImage).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(f.value, equals(6));

    imageCache.maximumSizeBytes = 256 * 3;

    final TestImageInfo g =
        await extractOneFrame(
              TestImageProvider(1, 7, image: testImage).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(g.value, equals(7));

    final TestImageInfo h =
        await extractOneFrame(
              TestImageProvider(1, 8, image: testImage).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(h.value, equals(7));
  });
}
