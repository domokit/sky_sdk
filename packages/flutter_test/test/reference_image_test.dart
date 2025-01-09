// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

Future<ui.Image> createTestImage(int width, int height, ui.Color color) async {
  final ui.Paint paint =
      ui.Paint()
        ..style = ui.PaintingStyle.stroke
        ..strokeWidth = 1.0
        ..color = color;
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas pictureCanvas = ui.Canvas(recorder);
  pictureCanvas.drawCircle(Offset.zero, 20.0, paint);
  final ui.Picture picture = recorder.endRecording();
  final ui.Image image = await picture.toImage(width, height);
  picture.dispose();
  return image;
}

void main() {
  const ui.Color red = ui.Color.fromARGB(255, 255, 0, 0);
  const ui.Color green = ui.Color.fromARGB(255, 0, 255, 0);
  const ui.Color transparentRed = ui.Color.fromARGB(128, 255, 0, 0);

  group('succeeds', () {
    testWidgets('when images have the same content', (WidgetTester tester) async {
      final ui.Image image1 = await createTestImage(100, 100, red);
      addTearDown(image1.dispose);
      final ui.Image referenceImage1 = await createTestImage(100, 100, red);
      addTearDown(referenceImage1.dispose);

      await expectLater(image1, matchesReferenceImage(referenceImage1));

      final ui.Image image2 = await createTestImage(100, 100, green);
      addTearDown(image2.dispose);
      final ui.Image referenceImage2 = await createTestImage(100, 100, green);
      addTearDown(referenceImage2.dispose);

      await expectLater(image2, matchesReferenceImage(referenceImage2));

      final ui.Image image3 = await createTestImage(100, 100, transparentRed);
      addTearDown(image3.dispose);
      final ui.Image referenceImage3 = await createTestImage(100, 100, transparentRed);
      addTearDown(referenceImage3.dispose);

      await expectLater(image3, matchesReferenceImage(referenceImage3));
    });

    testWidgets('when images are identical', (WidgetTester tester) async {
      final ui.Image image = await createTestImage(100, 100, red);
      addTearDown(image.dispose);
      await expectLater(image, matchesReferenceImage(image));
    });

    testWidgets('when widget looks the same', (WidgetTester tester) async {
      addTearDown(tester.view.reset);
      tester.view
        ..physicalSize = const Size(10, 10)
        ..devicePixelRatio = 1;

      const ValueKey<String> repaintBoundaryKey = ValueKey<String>('boundary');

      await tester.pumpWidget(
        const RepaintBoundary(key: repaintBoundaryKey, child: ColoredBox(color: red)),
      );

      final ui.Image referenceImage =
          (tester.renderObject(find.byKey(repaintBoundaryKey)) as RenderRepaintBoundary)
              .toImageSync();
      addTearDown(referenceImage.dispose);

      await expectLater(find.byKey(repaintBoundaryKey), matchesReferenceImage(referenceImage));
    });
  });

  group('fails', () {
    testWidgets('when image sizes do not match', (WidgetTester tester) async {
      final ui.Image red50 = await createTestImage(50, 50, red);
      addTearDown(red50.dispose);
      final ui.Image red100 = await createTestImage(100, 100, red);
      addTearDown(red100.dispose);

      expect(
        await matchesReferenceImage(red50).matchAsync(red100),
        equals('does not match as width or height do not match. [100×100] != [50×50]'),
      );
    });

    testWidgets('when image pixels do not match', (WidgetTester tester) async {
      final ui.Image red100 = await createTestImage(100, 100, red);
      addTearDown(red100.dispose);
      final ui.Image transparentRed100 = await createTestImage(100, 100, transparentRed);
      addTearDown(transparentRed100.dispose);

      expect(
        await matchesReferenceImage(red100).matchAsync(transparentRed100),
        equals('does not match on 57 pixels'),
      );

      final ui.Image green100 = await createTestImage(100, 100, green);
      addTearDown(green100.dispose);

      expect(
        await matchesReferenceImage(red100).matchAsync(green100),
        equals('does not match on 57 pixels'),
      );
    });

    testWidgets('when widget does not look the same', (WidgetTester tester) async {
      addTearDown(tester.view.reset);
      tester.view
        ..physicalSize = const Size(10, 10)
        ..devicePixelRatio = 1;

      const ValueKey<String> repaintBoundaryKey = ValueKey<String>('boundary');

      await tester.pumpWidget(
        const RepaintBoundary(key: repaintBoundaryKey, child: ColoredBox(color: red)),
      );

      final ui.Image referenceImage =
          (tester.renderObject(find.byKey(repaintBoundaryKey)) as RenderRepaintBoundary)
              .toImageSync();
      addTearDown(referenceImage.dispose);

      await tester.pumpWidget(
        const RepaintBoundary(key: repaintBoundaryKey, child: ColoredBox(color: green)),
      );

      expect(
        await matchesReferenceImage(referenceImage).matchAsync(find.byKey(repaintBoundaryKey)),
        equals('does not match on 100 pixels'),
      );
    });
  });
}
