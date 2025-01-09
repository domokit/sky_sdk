// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  List<CkColorFilter> createColorFilters() {
    return <CkColorFilter>[
      createCkColorFilter(
        const EngineColorFilter.mode(ui.Color(0x12345678), ui.BlendMode.srcOver),
      )!,
      createCkColorFilter(
        const EngineColorFilter.mode(ui.Color(0x12345678), ui.BlendMode.dstOver),
      )!,
      createCkColorFilter(
        const EngineColorFilter.mode(ui.Color(0x87654321), ui.BlendMode.dstOver),
      )!,
      createCkColorFilter(
        const EngineColorFilter.matrix(<double>[
          1,
          0,
          0,
          0,
          0,
          0,
          1,
          0,
          0,
          0,
          0,
          0,
          1,
          0,
          0,
          0,
          0,
          0,
          1,
          0,
        ]),
      )!,
      createCkColorFilter(
        EngineColorFilter.matrix(
          Float32List.fromList(<double>[
            2,
            0,
            0,
            0,
            0,
            0,
            2,
            0,
            0,
            0,
            0,
            0,
            2,
            0,
            0,
            0,
            0,
            0,
            2,
            0,
          ]),
        ),
      )!,
      createCkColorFilter(const EngineColorFilter.linearToSrgbGamma())!,
      createCkColorFilter(const EngineColorFilter.srgbToLinearGamma())!,
    ];
  }

  List<CkImageFilter> createImageFilters() {
    final List<CkImageFilter> filters = <CkImageFilter>[
      CkImageFilter.blur(sigmaX: 5, sigmaY: 6, tileMode: ui.TileMode.clamp),
      CkImageFilter.blur(sigmaX: 6, sigmaY: 5, tileMode: ui.TileMode.clamp),
      CkImageFilter.blur(sigmaX: 6, sigmaY: 5, tileMode: ui.TileMode.decal),
      CkImageFilter.dilate(radiusX: 5, radiusY: 6),
      CkImageFilter.erode(radiusX: 7, radiusY: 8),
      for (final CkColorFilter colorFilter in createColorFilters())
        CkImageFilter.color(colorFilter: colorFilter),
    ];
    filters.add(CkImageFilter.compose(outer: filters[0], inner: filters[1]));
    filters.add(CkImageFilter.compose(outer: filters[1], inner: filters[3]));
    return filters;
  }

  setUpCanvasKitTest(withImplicitView: true);

  group('ImageFilters', () {
    {
      final testFilters = createImageFilters();
      for (final imageFilter in testFilters) {
        test('${imageFilter.runtimeType}.withSkImageFilter creates temp SkImageFilter', () {
          expect(imageFilter, isA<CkImageFilter>());
          SkImageFilter? skFilter;
          imageFilter.withSkImageFilter((value) {
            expect(value.isDeleted(), isFalse);
            skFilter = value;
          });
          expect(skFilter, isNotNull);
          expect(
            reason: 'Because the SkImageFilter instance is temporary',
            skFilter!.isDeleted(),
            isTrue,
          );
        });
      }
    }

    test('== operator', () {
      final List<ui.ImageFilter> filters1 = <ui.ImageFilter>[
        ...createImageFilters(),
        ...createColorFilters(),
      ];
      final List<ui.ImageFilter> filters2 = <ui.ImageFilter>[
        ...createImageFilters(),
        ...createColorFilters(),
      ];

      for (int index1 = 0; index1 < filters1.length; index1 += 1) {
        final ui.ImageFilter imageFilter1 = filters1[index1];
        expect(imageFilter1 == imageFilter1, isTrue);
        for (int index2 = 0; index2 < filters2.length; index2 += 1) {
          final ui.ImageFilter imageFilter2 = filters2[index2];
          expect(imageFilter1 == imageFilter2, imageFilter2 == imageFilter1);
          expect(imageFilter1 == imageFilter2, index1 == index2);
        }
      }
    });

    test('reuses the Skia filter', () {
      final CkPaint paint = CkPaint();
      paint.imageFilter = CkImageFilter.blur(sigmaX: 5, sigmaY: 10, tileMode: ui.TileMode.clamp);

      final CkManagedSkImageFilterConvertible managedFilter1 =
          paint.imageFilter! as CkManagedSkImageFilterConvertible;

      paint.imageFilter = CkImageFilter.blur(sigmaX: 5, sigmaY: 10, tileMode: ui.TileMode.clamp);
      final CkManagedSkImageFilterConvertible managedFilter2 =
          paint.imageFilter! as CkManagedSkImageFilterConvertible;

      expect(managedFilter1, same(managedFilter2));
    });

    test('does not throw for both sigmaX and sigmaY set to 0', () async {
      final CkImageFilter imageFilter = CkImageFilter.blur(
        sigmaX: 0,
        sigmaY: 0,
        tileMode: ui.TileMode.clamp,
      );
      expect(imageFilter, isNotNull);

      const ui.Rect region = ui.Rect.fromLTRB(0, 0, 500, 250);

      final LayerSceneBuilder builder = LayerSceneBuilder();
      builder.pushOffset(0, 0);
      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle1 = recorder.endRecording();
      builder.addPicture(ui.Offset.zero, redCircle1);

      builder.pushImageFilter(imageFilter);

      // Draw another red circle and apply it to the scene.
      // This one should also be red with the image filter doing nothing
      final CkPictureRecorder recorder2 = CkPictureRecorder();
      final CkCanvas canvas2 = recorder2.beginRecording(region);
      canvas2.drawCircle(
        const ui.Offset(425, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle2 = recorder2.endRecording();

      builder.addPicture(ui.Offset.zero, redCircle2);

      await matchSceneGolden('canvaskit_zero_sigma_blur.png', builder.build(), region: region);
    });

    test('using a colorFilter', () async {
      final CkColorFilter colorFilter =
          createCkColorFilter(
            const EngineColorFilter.mode(ui.Color.fromARGB(255, 0, 255, 0), ui.BlendMode.srcIn),
          )!;

      const ui.Rect region = ui.Rect.fromLTRB(0, 0, 500, 250);

      final LayerSceneBuilder builder = LayerSceneBuilder();
      builder.pushOffset(0, 0);

      builder.pushImageFilter(colorFilter);

      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle1 = recorder.endRecording();
      builder.addPicture(ui.Offset.zero, redCircle1);
      // The drawn red circle should actually be green with the colorFilter.

      await matchSceneGolden(
        'canvaskit_imageFilter_using_colorFilter.png',
        builder.build(),
        region: region,
      );
    });

    test('using a compose filter', () async {
      final CkImageFilter blurFilter = CkImageFilter.blur(
        sigmaX: 5,
        sigmaY: 5,
        tileMode: ui.TileMode.clamp,
      );
      final CkColorFilter colorFilter =
          createCkColorFilter(
            const EngineColorFilter.mode(ui.Color.fromARGB(255, 0, 255, 0), ui.BlendMode.srcIn),
          )!;
      final CkImageFilter colorImageFilter = CkImageFilter.color(colorFilter: colorFilter);
      final CkImageFilter composeFilter = CkImageFilter.compose(
        outer: blurFilter,
        inner: colorImageFilter,
      );

      const ui.Rect region = ui.Rect.fromLTRB(0, 0, 500, 250);

      final LayerSceneBuilder builder = LayerSceneBuilder();
      builder.pushOffset(0, 0);

      builder.pushImageFilter(composeFilter);

      final CkPictureRecorder recorder = CkPictureRecorder();
      final CkCanvas canvas = recorder.beginRecording(region);

      canvas.drawCircle(
        const ui.Offset(75, 125),
        50,
        CkPaint()..color = const ui.Color.fromARGB(255, 255, 0, 0),
      );
      final CkPicture redCircle1 = recorder.endRecording();
      builder.addPicture(ui.Offset.zero, redCircle1);
      // The drawn red circle should actually be green and blurred.

      await matchSceneGolden('canvaskit_composeImageFilter.png', builder.build(), region: region);
    });
  });

  group('MaskFilter', () {
    test('with 0 sigma can be set on a Paint', () {
      final ui.Paint paint = ui.Paint();
      const ui.MaskFilter filter = ui.MaskFilter.blur(ui.BlurStyle.normal, 0);

      expect(() => paint.maskFilter = filter, isNot(throwsException));
    });
  });
}
