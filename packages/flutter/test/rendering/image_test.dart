// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show Image;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

Future<void> main() async {
  TestRenderingFlutterBinding.ensureInitialized();

  final ui.Image squareImage = await createTestImage(width: 10, height: 10);
  final ui.Image wideImage = await createTestImage(width: 20, height: 10);
  final ui.Image tallImage = await createTestImage(width: 10, height: 20);
  test('Image sizing', () {
    RenderImage image;

    image = RenderImage(image: squareImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 25.0,
        minHeight: 25.0,
        maxWidth: 100.0,
        maxHeight: 100.0,
      ),
    );
    expect(image.size.width, equals(25.0));
    expect(image.size.height, equals(25.0));

    expect(image, hasAGoodToStringDeep);
    expect(
      image.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderImage#00000 relayoutBoundary=up2 NEEDS-PAINT\n'
        '   parentData: <none> (can use size)\n'
        '   constraints: BoxConstraints(25.0<=w<=100.0, 25.0<=h<=100.0)\n'
        '   size: Size(25.0, 25.0)\n'
        '   image: $squareImage\n'
        '   alignment: Alignment.center\n'
        '   invertColors: false\n'
        '   filterQuality: medium\n',
      ),
    );

    image = RenderImage(image: wideImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 5.0,
        minHeight: 30.0,
        maxWidth: 100.0,
        maxHeight: 100.0,
      ),
    );
    expect(image.size.width, equals(60.0));
    expect(image.size.height, equals(30.0));

    image = RenderImage(image: tallImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 50.0,
        minHeight: 5.0,
        maxWidth: 75.0,
        maxHeight: 75.0,
      ),
    );
    expect(image.size.width, equals(50.0));
    expect(image.size.height, equals(75.0));

    image = RenderImage(image: wideImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 5.0,
        minHeight: 5.0,
        maxWidth: 100.0,
        maxHeight: 100.0,
      ),
    );
    expect(image.size.width, equals(20.0));
    expect(image.size.height, equals(10.0));

    image = RenderImage(image: wideImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 5.0,
        minHeight: 5.0,
        maxWidth: 16.0,
        maxHeight: 16.0,
      ),
    );
    expect(image.size.width, equals(16.0));
    expect(image.size.height, equals(8.0));

    image = RenderImage(image: tallImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 5.0,
        minHeight: 5.0,
        maxWidth: 16.0,
        maxHeight: 16.0,
      ),
    );
    expect(image.size.width, equals(8.0));
    expect(image.size.height, equals(16.0));

    image = RenderImage(image: squareImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 4.0,
        minHeight: 4.0,
        maxWidth: 8.0,
        maxHeight: 8.0,
      ),
    );
    expect(image.size.width, equals(8.0));
    expect(image.size.height, equals(8.0));

    image = RenderImage(image: wideImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 20.0,
        minHeight: 20.0,
        maxWidth: 30.0,
        maxHeight: 30.0,
      ),
    );
    expect(image.size.width, equals(30.0));
    expect(image.size.height, equals(20.0));

    image = RenderImage(image: tallImage);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 20.0,
        minHeight: 20.0,
        maxWidth: 30.0,
        maxHeight: 30.0,
      ),
    );
    expect(image.size.width, equals(20.0));
    expect(image.size.height, equals(30.0));
  });

  test('Null image sizing', () {
    RenderImage image;

    image = RenderImage();
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 25.0,
        minHeight: 25.0,
        maxWidth: 100.0,
        maxHeight: 100.0,
      ),
    );
    expect(image.size.width, equals(25.0));
    expect(image.size.height, equals(25.0));

    image = RenderImage(width: 50.0);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 25.0,
        minHeight: 25.0,
        maxWidth: 100.0,
        maxHeight: 100.0,
      ),
    );
    expect(image.size.width, equals(50.0));
    expect(image.size.height, equals(25.0));

    image = RenderImage(height: 50.0);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 25.0,
        minHeight: 25.0,
        maxWidth: 100.0,
        maxHeight: 100.0,
      ),
    );
    expect(image.size.width, equals(25.0));
    expect(image.size.height, equals(50.0));

    image = RenderImage(width: 100.0, height: 100.0);
    layout(
      image,
      constraints: const BoxConstraints(
        minWidth: 25.0,
        minHeight: 25.0,
        maxWidth: 75.0,
        maxHeight: 75.0,
      ),
    );
    expect(image.size.width, equals(75.0));
    expect(image.size.height, equals(75.0));
  });

  test('update image colorBlendMode', () {
    final RenderImage image = RenderImage();
    expect(image.colorBlendMode, isNull);
    image.colorBlendMode = BlendMode.color;
    expect(image.colorBlendMode, BlendMode.color);
  });

  test('RenderImage disposes its image', () async {
    final ui.Image image = await createTestImage(width: 10, height: 10, cache: false);
    expect(image.debugGetOpenHandleStackTraces()!.length, 1);

    final RenderImage renderImage = RenderImage(image: image.clone());
    expect(image.debugGetOpenHandleStackTraces()!.length, 2);

    renderImage.image = image.clone();
    expect(image.debugGetOpenHandleStackTraces()!.length, 2);

    renderImage.image = null;
    expect(image.debugGetOpenHandleStackTraces()!.length, 1);

    image.dispose();
    expect(image.debugGetOpenHandleStackTraces()!.length, 0);
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87442

  test('RenderImage does not dispose its image if setting the same image twice', () async {
    final ui.Image image = await createTestImage(width: 10, height: 10, cache: false);
    expect(image.debugGetOpenHandleStackTraces()!.length, 1);

    final RenderImage renderImage = RenderImage(image: image.clone());
    expect(image.debugGetOpenHandleStackTraces()!.length, 2);

    // Testing short-circuit logic of setter.
    renderImage.image = renderImage.image; // ignore: no_self_assignments
    expect(image.debugGetOpenHandleStackTraces()!.length, 2);

    renderImage.image = null;
    expect(image.debugGetOpenHandleStackTraces()!.length, 1);

    image.dispose();
    expect(image.debugGetOpenHandleStackTraces()!.length, 0);
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87442

  test('Render image disposes its image when it is disposed', () async {
    final ui.Image image = await createTestImage(width: 10, height: 10, cache: false);
    expect(image.debugGetOpenHandleStackTraces()!.length, 1);

    final RenderImage renderImage = RenderImage(image: image.clone());
    expect(image.debugGetOpenHandleStackTraces()!.length, 2);

    renderImage.dispose();
    expect(image.debugGetOpenHandleStackTraces()!.length, 1);
    expect(renderImage.image, null);

    image.dispose();
    expect(image.debugGetOpenHandleStackTraces()!.length, 0);
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87442
}
