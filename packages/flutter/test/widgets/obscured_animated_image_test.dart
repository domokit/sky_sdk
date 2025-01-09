// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui' as ui show Image;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';
import '../painting/fake_codec.dart';
import '../painting/fake_image_provider.dart';

Future<void> main() async {
  final FakeCodec fakeCodec = await FakeCodec.fromData(Uint8List.fromList(kAnimatedGif));
  final FakeImageProvider fakeImageProvider = FakeImageProvider(fakeCodec);

  testWidgets('Obscured image does not animate', (WidgetTester tester) async {
    final GlobalKey imageKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Image(image: fakeImageProvider, excludeFromSemantics: true, key: imageKey),
        routes: <String, WidgetBuilder>{'/page': (BuildContext context) => Container()},
      ),
    );
    final RenderImage renderImage = tester.renderObject(find.byType(Image));
    final ui.Image? image1 = renderImage.image;
    await tester.pump(const Duration(milliseconds: 100));
    final ui.Image? image2 = renderImage.image;
    expect(image1, isNot(same(image2)));

    Navigator.pushNamed(imageKey.currentContext!, '/page');
    await tester.pump(); // Starts the page animation.
    await tester.pump(const Duration(seconds: 1)); // Let the page animation complete.

    // The image is now obscured by another page, it should not be changing
    // frames.
    final ui.Image? image3 = renderImage.image;
    await tester.pump(const Duration(milliseconds: 100));
    final ui.Image? image4 = renderImage.image;
    expect(image3, same(image4));
    imageCache.clear();
  });
}
