// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';
import 'dart:ui';

import 'package:test/test.dart';

import 'goldens.dart';
import 'impeller_enabled.dart';

const Color red = Color(0xFFAA0000);
const Color green = Color(0xFF00AA00);

const int greenCenterBlurred = 0x29001B00;
const int greenSideBlurred = 0x19001000;
const int greenCornerBlurred = 0x0F000A00;

const int greenCenterScaled = 0xFF00AA00;
const int greenSideScaled = 0x80005500;
const int greenCornerScaled = 0x40002B00;

const List<double> grayscaleColorMatrix = <double>[
  0.2126,
  0.7152,
  0.0722,
  0,
  0,
  0.2126,
  0.7152,
  0.0722,
  0,
  0,
  0.2126,
  0.7152,
  0.0722,
  0,
  0,
  0,
  0,
  0,
  1,
  0,
];

const List<double> constValueColorMatrix = <double>[
  0,
  0,
  0,
  0,
  2,
  0,
  0,
  0,
  0,
  2,
  0,
  0,
  0,
  0,
  2,
  0,
  0,
  0,
  0,
  255,
];

const List<double> halvesBrightnessColorMatrix = <double>[
  0.5,
  0,
  0,
  0,
  0,
  0,
  0.5,
  0,
  0,
  0,
  0,
  0,
  0.5,
  0,
  0,
  0,
  0,
  0,
  1,
  0,
];

void main() async {
  Future<Uint32List> getBytesForPaint(Paint paint, {int width = 3, int height = 3}) async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas recorderCanvas = Canvas(recorder);
    recorderCanvas.drawRect(const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0), paint);
    final Picture picture = recorder.endRecording();
    final Image image = await picture.toImage(width, height);
    final ByteData bytes = (await image.toByteData())!;

    expect(bytes.lengthInBytes, equals(width * height * 4));
    return bytes.buffer.asUint32List();
  }

  Future<Uint32List> getBytesForColorPaint(Paint paint, {int width = 1, int height = 1}) async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas recorderCanvas = Canvas(recorder);
    recorderCanvas.drawPaint(paint);
    final Picture picture = recorder.endRecording();
    final Image image = await picture.toImage(width, height);
    final ByteData bytes = (await image.toByteData())!;

    expect(bytes.lengthInBytes, width * height * 4);
    return bytes.buffer.asUint32List();
  }

  ImageFilter makeBlur(double sigmaX, double sigmaY, [TileMode? tileMode]) =>
      ImageFilter.blur(sigmaX: sigmaX, sigmaY: sigmaY, tileMode: tileMode);

  ImageFilter makeDilate(double radiusX, double radiusY) =>
      ImageFilter.dilate(radiusX: radiusX, radiusY: radiusY);

  ImageFilter makeErode(double radiusX, double radiusY) =>
      ImageFilter.erode(radiusX: radiusX, radiusY: radiusY);

  ImageFilter makeScale(
    double scX,
    double scY, [
    double trX = 0.0,
    double trY = 0.0,
    FilterQuality quality = FilterQuality.low,
  ]) {
    trX *= 1.0 - scX;
    trY *= 1.0 - scY;
    return ImageFilter.matrix(
      Float64List.fromList(<double>[
        scX,
        0.0,
        0.0,
        0.0,
        0.0,
        scY,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        trX,
        trY,
        0.0,
        1.0,
      ]),
      filterQuality: quality,
    );
  }

  List<ColorFilter> colorFilters() {
    // Create new color filter instances on each invocation.
    // ignore: prefer_const_constructors
    return <ColorFilter>[
      ColorFilter.mode(green, BlendMode.color), // ignore: prefer_const_constructors
      ColorFilter.mode(red, BlendMode.color), // ignore: prefer_const_constructors
      ColorFilter.mode(red, BlendMode.screen), // ignore: prefer_const_constructors
      ColorFilter.matrix(grayscaleColorMatrix), // ignore: prefer_const_constructors
      ColorFilter.linearToSrgbGamma(), // ignore: prefer_const_constructors
      ColorFilter.srgbToLinearGamma(), // ignore: prefer_const_constructors
    ];
  }

  List<ImageFilter> makeList() {
    return <ImageFilter>[
      makeBlur(10.0, 10.0),
      makeBlur(10.0, 10.0, TileMode.decal),
      makeBlur(10.0, 10.0, TileMode.clamp),
      makeBlur(10.0, 10.0, TileMode.mirror),
      makeBlur(10.0, 10.0, TileMode.repeated),
      makeBlur(10.0, 20.0),
      makeBlur(20.0, 20.0),
      makeDilate(10.0, 20.0),
      makeDilate(20.0, 20.0),
      makeDilate(20.0, 10.0),
      makeErode(10.0, 20.0),
      makeErode(20.0, 20.0),
      makeErode(20.0, 10.0),
      makeScale(10.0, 10.0),
      makeScale(10.0, 20.0),
      makeScale(20.0, 10.0),
      makeScale(10.0, 10.0, 1.0, 1.0),
      makeScale(10.0, 10.0, 0.0, 0.0, FilterQuality.medium),
      makeScale(10.0, 10.0, 0.0, 0.0, FilterQuality.high),
      makeScale(10.0, 10.0, 0.0, 0.0, FilterQuality.none),
      ...colorFilters(),
    ];
  }

  void checkEquality(List<ImageFilter> a, List<ImageFilter> b) {
    for (int i = 0; i < a.length; i++) {
      for (int j = 0; j < a.length; j++) {
        if (i == j) {
          expect(a[i], equals(b[j]));
          expect(a[i].hashCode, equals(b[j].hashCode));
          expect(a[i].toString(), equals(b[j].toString()));
        } else {
          expect(a[i], isNot(b[j]));
          // No expectations on hashCode if objects are not equal
          expect(a[i].toString(), isNot(b[j].toString()));
        }
      }
    }
  }

  List<ImageFilter> composed(List<ImageFilter> a, List<ImageFilter> b) {
    return <ImageFilter>[
      for (final ImageFilter x in a)
        for (final ImageFilter y in b) ImageFilter.compose(outer: x, inner: y),
    ];
  }

  test('ImageFilter - equals', () async {
    final List<ImageFilter> A = makeList();
    final List<ImageFilter> B = makeList();
    checkEquality(A, A);
    checkEquality(A, B);
    checkEquality(B, B);
    checkEquality(composed(A, A), composed(A, A));
    checkEquality(composed(A, B), composed(B, A));
    checkEquality(composed(B, B), composed(B, B));
  });

  void checkBytes(Uint32List bytes, int center, int side, int corner) {
    expect(bytes[0], equals(corner));
    expect(bytes[1], equals(side));
    expect(bytes[2], equals(corner));

    expect(bytes[3], equals(side));
    expect(bytes[4], equals(center));
    expect(bytes[5], equals(side));

    expect(bytes[6], equals(corner));
    expect(bytes[7], equals(side));
    expect(bytes[8], equals(corner));
  }

  test('ImageFilter - blur', () async {
    if (impellerEnabled) {
      print('Disabled - see https://github.com/flutter/flutter/issues/135712');
      return;
    }
    final Paint paint =
        Paint()
          ..color = green
          ..imageFilter = makeBlur(1.0, 1.0, TileMode.decal);

    final Uint32List bytes = await getBytesForPaint(paint);
    checkBytes(bytes, greenCenterBlurred, greenSideBlurred, greenCornerBlurred);
  });

  test('ImageFilter - blur toString', () async {
    var filter = makeBlur(1.9, 2.1);
    expect(filter.toString(), 'ImageFilter.blur(1.9, 2.1, unspecified)');

    filter = makeBlur(1.9, 2.1, TileMode.decal);
    expect(filter.toString(), 'ImageFilter.blur(1.9, 2.1, decal)');

    filter = makeBlur(1.9, 2.1, TileMode.clamp);
    expect(filter.toString(), 'ImageFilter.blur(1.9, 2.1, clamp)');

    filter = makeBlur(1.9, 2.1, TileMode.mirror);
    expect(filter.toString(), 'ImageFilter.blur(1.9, 2.1, mirror)');

    filter = makeBlur(1.9, 2.1, TileMode.repeated);
    expect(filter.toString(), 'ImageFilter.blur(1.9, 2.1, repeated)');
  });

  test('ImageFilter - dilate', () async {
    final Paint paint =
        Paint()
          ..color = green
          ..imageFilter = makeDilate(1.0, 1.0);

    final Uint32List bytes = await getBytesForPaint(paint);
    checkBytes(bytes, green.value, green.value, green.value);
  });

  test('ImageFilter - erode', () async {
    final Paint paint =
        Paint()
          ..color = green
          ..imageFilter = makeErode(1.0, 1.0);

    final Uint32List bytes = await getBytesForPaint(paint);
    checkBytes(bytes, 0, 0, 0);
  });

  test('ImageFilter - matrix', () async {
    if (impellerEnabled) {
      print('Disabled - see https://github.com/flutter/flutter/issues/135712');
      return;
    }

    final Paint paint =
        Paint()
          ..color = green
          ..imageFilter = makeScale(2.0, 2.0, 1.5, 1.5);

    final Uint32List bytes = await getBytesForPaint(paint);
    checkBytes(bytes, greenCenterScaled, greenSideScaled, greenCornerScaled);
  });

  test('ImageFilter - matrix: copies the list', () async {
    final Float64List matrix = Float64List.fromList(<double>[
      1.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
    ]);

    final ImageFilter filter = ImageFilter.matrix(matrix);
    final String originalDescription = filter.toString();

    // Modify the matrix.
    matrix[0] = 12345;
    expect(
      filter.toString(),
      contains('[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]'),
    );
    expect(filter.toString(), originalDescription);
  });

  test('ImageFilter - from color filters', () async {
    if (impellerEnabled) {
      print('Disabled - see https://github.com/flutter/flutter/issues/135712');
      return;
    }

    final Paint paint =
        Paint()
          ..color = green
          ..imageFilter = const ColorFilter.matrix(constValueColorMatrix);

    final Uint32List bytes = await getBytesForColorPaint(paint);
    expect(bytes[0], 0xFF020202);
  });

  test('ImageFilter - color filter composition', () async {
    if (impellerEnabled) {
      print('Disabled - see https://github.com/flutter/flutter/issues/135712');
      return;
    }

    final ImageFilter compOrder1 = ImageFilter.compose(
      outer: const ColorFilter.matrix(halvesBrightnessColorMatrix),
      inner: const ColorFilter.matrix(constValueColorMatrix),
    );

    final ImageFilter compOrder2 = ImageFilter.compose(
      outer: const ColorFilter.matrix(constValueColorMatrix),
      inner: const ColorFilter.matrix(halvesBrightnessColorMatrix),
    );

    final Paint paint =
        Paint()
          ..color = green
          ..imageFilter = compOrder1;

    Uint32List bytes = await getBytesForColorPaint(paint);
    expect(bytes[0], 0xFF010101);

    paint
      ..color = green
      ..imageFilter = compOrder2;
    bytes = await getBytesForColorPaint(paint);
    expect(bytes[0], 0xFF020202);
  });

  test('Composite ImageFilter toString', () {
    expect(
      ImageFilter.compose(
        outer: makeBlur(20.0, 20.0, TileMode.decal),
        inner: makeBlur(10.0, 10.0),
      ).toString(),
      contains('blur(10.0, 10.0, unspecified) -> blur(20.0, 20.0, decal)'),
    );

    // Produces a flat list of filters
    expect(
      ImageFilter.compose(
        outer: ImageFilter.compose(
          outer: makeBlur(30.0, 30.0, TileMode.mirror),
          inner: makeBlur(20.0, 20.0, TileMode.repeated),
        ),
        inner: ImageFilter.compose(
          outer: const ColorFilter.mode(Color(0xFFABCDEF), BlendMode.color),
          inner: makeScale(10.0, 10.0),
        ),
      ).toString(),
      contains(
        'matrix([10.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, 0.0, 1.0], FilterQuality.low) -> '
        'ColorFilter.mode(${const Color(0xFFABCDEF)}, BlendMode.color) -> '
        'blur(20.0, 20.0, repeated) -> '
        'blur(30.0, 30.0, mirror)',
      ),
    );
  });

  // Tests that FilterQuality.<value> produces the expected golden file.
  group('ImageFilter|FilterQuality', () {
    /// Draw a red-green checkerboard pattern with 1x1 squares (pixels).
    Future<Image> drawCheckerboard({int width = 100, int height = 100}) async {
      final Completer<Image> completer = Completer<Image>();
      final Uint32List pixels = Uint32List.fromList(
        List<int>.generate(width * height, (int index) {
          final int x = index % width;
          final int y = index ~/ width;
          return (x % 2 == y % 2) ? red.value : green.value;
        }),
      );
      decodeImageFromPixels(
        Uint8List.view(pixels.buffer),
        width,
        height,
        PixelFormat.rgba8888,
        completer.complete,
      );
      return completer.future;
    }

    final Future<Image> redGreenCheckerboard = drawCheckerboard();

    /// Return the [image] shrunk and then scaled.
    Future<Image> shrinkAndScaleImage(
      Image image,
      FilterQuality quality, {
      double factorDown = 0.25,
      double factorUp = 10,
    }) async {
      Future<Image> scale(Image image, double factor) async {
        final Paint paint = Paint()..filterQuality = quality;
        final PictureRecorder recorder = PictureRecorder();
        final Canvas canvas = Canvas(recorder);

        final Rect input = Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble());
        final Rect output = Rect.fromLTWH(0, 0, input.width * factor, input.height * factor);

        canvas.drawImageRect(image, input, output, paint);
        final Picture picture = recorder.endRecording();
        return picture.toImage(output.width.toInt(), output.height.toInt());
      }

      final Image shrunk = await scale(image, factorDown);
      return scale(shrunk, factorUp);
    }

    test('Scaling a checkerboard of 1x1 red-green pixels with FilterQuality.none', () async {
      final ImageComparer comparer = await ImageComparer.create();
      final Image base = await redGreenCheckerboard;
      final Image scaled = await shrinkAndScaleImage(base, FilterQuality.none);
      await comparer.addGoldenImage(
        scaled,
        'dart_ui_filter_quality_none_scale_1x1_red_green_checkerboard.png',
      );
    });
  });
}
