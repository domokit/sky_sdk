// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';
import 'dart:math';
import 'dart:typed_data';
import 'dart:ui';

import 'package:path/path.dart' as path;
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

import 'goldens.dart';
import 'impeller_enabled.dart';

typedef CanvasCallback = void Function(Canvas canvas);

Future<Image> createImage(int width, int height) {
  final Completer<Image> completer = Completer<Image>();
  decodeImageFromPixels(
    Uint8List.fromList(List<int>.generate(width * height * 4, (int pixel) => pixel % 255)),
    width,
    height,
    PixelFormat.rgba8888,
    (Image image) {
      completer.complete(image);
    },
  );

  return completer.future;
}

void testCanvas(CanvasCallback callback) {
  try {
    callback(Canvas(PictureRecorder(), Rect.zero));
  } catch (error) {} // ignore: empty_catches
}

Future<Image> toImage(CanvasCallback callback, int width, int height) {
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder, Rect.fromLTRB(0, 0, width.toDouble(), height.toDouble()));
  callback(canvas);
  final Picture picture = recorder.endRecording();
  return picture.toImage(width, height);
}

void testNoCrashes() {
  test('canvas APIs should not crash', () async {
    final Paint paint = Paint();
    const Rect rect = Rect.fromLTRB(double.nan, double.nan, double.nan, double.nan);
    final RRect rrect = RRect.fromRectAndCorners(rect);
    const Offset offset = Offset(double.nan, double.nan);
    final Path path = Path();
    const Color color = Color(0x00000000);
    final Paragraph paragraph = ParagraphBuilder(ParagraphStyle()).build();

    final PictureRecorder recorder = PictureRecorder();
    final Canvas recorderCanvas = Canvas(recorder);
    recorderCanvas.scale(1.0, 1.0);
    final Picture picture = recorder.endRecording();
    final Image image = await picture.toImage(1, 1);

    try {
      Canvas(PictureRecorder());
    } catch (error) {} // ignore: empty_catches
    try {
      Canvas(PictureRecorder(), rect);
    } catch (error) {} // ignore: empty_catches

    try {
      PictureRecorder()
        ..endRecording()
        ..endRecording()
        ..endRecording();
    } catch (error) {} // ignore: empty_catches

    testCanvas((Canvas canvas) => canvas.clipPath(path));
    testCanvas((Canvas canvas) => canvas.clipRect(rect));
    testCanvas((Canvas canvas) => canvas.clipRRect(rrect));
    testCanvas((Canvas canvas) => canvas.drawArc(rect, 0.0, 0.0, false, paint));
    testCanvas(
      (Canvas canvas) =>
          canvas.drawAtlas(image, <RSTransform>[], <Rect>[], <Color>[], BlendMode.src, rect, paint),
    );
    testCanvas((Canvas canvas) => canvas.drawCircle(offset, double.nan, paint));
    testCanvas((Canvas canvas) => canvas.drawColor(color, BlendMode.src));
    testCanvas((Canvas canvas) => canvas.drawDRRect(rrect, rrect, paint));
    testCanvas((Canvas canvas) => canvas.drawImage(image, offset, paint));
    testCanvas((Canvas canvas) => canvas.drawImageNine(image, rect, rect, paint));
    testCanvas((Canvas canvas) => canvas.drawImageRect(image, rect, rect, paint));
    testCanvas((Canvas canvas) => canvas.drawLine(offset, offset, paint));
    testCanvas((Canvas canvas) => canvas.drawOval(rect, paint));
    testCanvas((Canvas canvas) => canvas.drawPaint(paint));
    testCanvas((Canvas canvas) => canvas.drawParagraph(paragraph, offset));
    testCanvas((Canvas canvas) => canvas.drawPath(path, paint));
    testCanvas((Canvas canvas) => canvas.drawPicture(picture));
    testCanvas((Canvas canvas) => canvas.drawPoints(PointMode.points, <Offset>[], paint));
    testCanvas(
      (Canvas canvas) => canvas.drawRawAtlas(
        image,
        Float32List(0),
        Float32List(0),
        Int32List(0),
        BlendMode.src,
        rect,
        paint,
      ),
    );
    testCanvas((Canvas canvas) => canvas.drawRawPoints(PointMode.points, Float32List(0), paint));
    testCanvas((Canvas canvas) => canvas.drawRect(rect, paint));
    testCanvas((Canvas canvas) => canvas.drawRRect(rrect, paint));
    testCanvas((Canvas canvas) => canvas.drawShadow(path, color, double.nan, false));
    testCanvas((Canvas canvas) => canvas.drawShadow(path, color, double.nan, true));
    testCanvas(
      (Canvas canvas) =>
          canvas.drawVertices(Vertices(VertexMode.triangles, <Offset>[]), BlendMode.screen, paint),
    );
    testCanvas((Canvas canvas) => canvas.getSaveCount());
    testCanvas((Canvas canvas) => canvas.restore());
    testCanvas((Canvas canvas) => canvas.rotate(double.nan));
    testCanvas((Canvas canvas) => canvas.save());
    testCanvas((Canvas canvas) => canvas.saveLayer(rect, paint));
    testCanvas((Canvas canvas) => canvas.saveLayer(null, paint));
    testCanvas((Canvas canvas) => canvas.scale(double.nan, double.nan));
    testCanvas((Canvas canvas) => canvas.skew(double.nan, double.nan));
    testCanvas((Canvas canvas) => canvas.transform(Float64List(16)));
    testCanvas((Canvas canvas) => canvas.translate(double.nan, double.nan));
    testCanvas(
      (Canvas canvas) => canvas.drawVertices(
        Vertices(VertexMode.triangles, <Offset>[], indices: <int>[]),
        BlendMode.screen,
        paint,
      ),
    );
    testCanvas(
      (Canvas canvas) => canvas.drawVertices(
        Vertices(VertexMode.triangles, <Offset>[])..dispose(),
        BlendMode.screen,
        paint,
      ),
    );

    // Regression test for https://github.com/flutter/flutter/issues/115143
    testCanvas(
      (Canvas canvas) => canvas.drawPaint(
        Paint()..imageFilter = const ColorFilter.mode(Color(0x00000000), BlendMode.xor),
      ),
    );

    // Regression test for https://github.com/flutter/flutter/issues/120278
    testCanvas(
      (Canvas canvas) => canvas.drawPaint(
        Paint()
          ..imageFilter = ImageFilter.compose(
            outer: ImageFilter.matrix(Matrix4.identity().storage),
            inner: ImageFilter.blur(),
          ),
      ),
    );
  });
}

const String kFlutterBuildDirectory = 'kFlutterBuildDirectory';

String get _flutterBuildPath {
  const String buildPath = String.fromEnvironment(kFlutterBuildDirectory);
  if (buildPath.isEmpty) {
    throw StateError('kFlutterBuildDirectory -D variable is not set.');
  }
  return buildPath;
}

void main() async {
  final ImageComparer comparer = await ImageComparer.create();

  testNoCrashes();

  test('Simple .toImage', () async {
    final Image image = await toImage(
      (Canvas canvas) {
        final Path circlePath =
            Path()..addOval(Rect.fromCircle(center: const Offset(40.0, 40.0), radius: 20.0));
        final Paint paint =
            Paint()
              ..isAntiAlias = false
              ..style = PaintingStyle.fill;
        canvas.drawPath(circlePath, paint);
      },
      100,
      100,
    );
    expect(image.width, equals(100));
    expect(image.height, equals(100));
    await comparer.addGoldenImage(image, 'canvas_test_toImage.png');
  });

  Gradient makeGradient() {
    return Gradient.linear(Offset.zero, const Offset(100, 100), const <Color>[
      Color(0xFF4C4D52),
      Color(0xFF202124),
    ]);
  }

  test('Simple gradient, which is implicitly dithered', () async {
    final Image image = await toImage(
      (Canvas canvas) {
        final Paint paint = Paint()..shader = makeGradient();
        canvas.drawPaint(paint);
      },
      100,
      100,
    );
    expect(image.width, equals(100));
    expect(image.height, equals(100));

    await comparer.addGoldenImage(image, 'canvas_test_dithered_gradient.png');
  });

  test('Null values allowed for drawAtlas methods', () async {
    final Image image = await createImage(100, 100);
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect rect = Rect.fromLTWH(0, 0, 100, 100);
    final RSTransform transform = RSTransform(1, 0, 0, 0);
    const Color color = Color(0x00000000);
    final Paint paint = Paint();
    canvas.drawAtlas(
      image,
      <RSTransform>[transform],
      <Rect>[rect],
      <Color>[color],
      BlendMode.src,
      rect,
      paint,
    );
    canvas.drawAtlas(
      image,
      <RSTransform>[transform],
      <Rect>[rect],
      <Color>[color],
      BlendMode.src,
      null,
      paint,
    );
    canvas.drawAtlas(image, <RSTransform>[transform], <Rect>[rect], <Color>[], null, rect, paint);
    canvas.drawAtlas(image, <RSTransform>[transform], <Rect>[rect], null, null, rect, paint);
    canvas.drawRawAtlas(
      image,
      Float32List(0),
      Float32List(0),
      Int32List(0),
      BlendMode.src,
      rect,
      paint,
    );
    canvas.drawRawAtlas(
      image,
      Float32List(0),
      Float32List(0),
      Int32List(0),
      BlendMode.src,
      null,
      paint,
    );
    canvas.drawRawAtlas(image, Float32List(0), Float32List(0), null, null, rect, paint);

    expect(
      () => canvas.drawAtlas(
        image,
        <RSTransform>[transform],
        <Rect>[rect],
        <Color>[color],
        null,
        rect,
        paint,
      ),
      throwsA(isA<AssertionError>()),
    );
  });

  test('Data lengths must match for drawAtlas methods', () async {
    final Image image = await createImage(100, 100);
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect rect = Rect.fromLTWH(0, 0, 100, 100);
    final RSTransform transform = RSTransform(1, 0, 0, 0);
    const Color color = Color(0x00000000);
    final Paint paint = Paint();
    canvas.drawAtlas(
      image,
      <RSTransform>[transform],
      <Rect>[rect],
      <Color>[color],
      BlendMode.src,
      rect,
      paint,
    );
    canvas.drawAtlas(
      image,
      <RSTransform>[transform, transform],
      <Rect>[rect, rect],
      <Color>[color, color],
      BlendMode.src,
      rect,
      paint,
    );
    canvas.drawAtlas(image, <RSTransform>[transform], <Rect>[rect], <Color>[], null, rect, paint);
    canvas.drawAtlas(image, <RSTransform>[transform], <Rect>[rect], null, null, rect, paint);
    canvas.drawRawAtlas(
      image,
      Float32List(0),
      Float32List(0),
      Int32List(0),
      BlendMode.src,
      rect,
      paint,
    );
    canvas.drawRawAtlas(
      image,
      Float32List(4),
      Float32List(4),
      Int32List(1),
      BlendMode.src,
      rect,
      paint,
    );
    canvas.drawRawAtlas(image, Float32List(4), Float32List(4), null, null, rect, paint);

    expect(
      () => canvas.drawAtlas(
        image,
        <RSTransform>[transform],
        <Rect>[],
        <Color>[color],
        BlendMode.src,
        rect,
        paint,
      ),
      throwsArgumentError,
    );
    expect(
      () => canvas.drawAtlas(
        image,
        <RSTransform>[],
        <Rect>[rect],
        <Color>[color],
        BlendMode.src,
        rect,
        paint,
      ),
      throwsArgumentError,
    );
    expect(
      () => canvas.drawAtlas(
        image,
        <RSTransform>[transform],
        <Rect>[rect],
        <Color>[color, color],
        BlendMode.src,
        rect,
        paint,
      ),
      throwsArgumentError,
    );
    expect(
      () => canvas.drawAtlas(
        image,
        <RSTransform>[transform],
        <Rect>[rect, rect],
        <Color>[color],
        BlendMode.src,
        rect,
        paint,
      ),
      throwsArgumentError,
    );
    expect(
      () => canvas.drawAtlas(
        image,
        <RSTransform>[transform, transform],
        <Rect>[rect],
        <Color>[color],
        BlendMode.src,
        rect,
        paint,
      ),
      throwsArgumentError,
    );
    expect(
      () => canvas.drawRawAtlas(image, Float32List(3), Float32List(3), null, null, rect, paint),
      throwsArgumentError,
    );
    expect(
      () => canvas.drawRawAtlas(image, Float32List(4), Float32List(0), null, null, rect, paint),
      throwsArgumentError,
    );
    expect(
      () => canvas.drawRawAtlas(image, Float32List(0), Float32List(4), null, null, rect, paint),
      throwsArgumentError,
    );
    expect(
      () => canvas.drawRawAtlas(
        image,
        Float32List(4),
        Float32List(4),
        Int32List(2),
        BlendMode.src,
        rect,
        paint,
      ),
      throwsArgumentError,
    );
  });

  test('Canvas preserves perspective data in Matrix4', () async {
    const double rotateAroundX = pi / 6; // 30 degrees
    const double rotateAroundY = pi / 9; // 20 degrees
    const int width = 150;
    const int height = 150;
    const Color black = Color.fromARGB(255, 0, 0, 0);
    const Color green = Color.fromARGB(255, 0, 255, 0);
    void paint(Canvas canvas, CanvasCallback rotate) {
      canvas.translate(width * 0.5, height * 0.5);
      rotate(canvas);
      const double width3 = width / 3.0;
      const double width5 = width / 5.0;
      const double width10 = width / 10.0;
      canvas.drawRect(
        const Rect.fromLTRB(-width3, -width3, width3, width3),
        Paint()..color = green,
      );
      canvas.drawRect(
        const Rect.fromLTRB(-width5, -width5, -width10, width5),
        Paint()..color = black,
      );
      canvas.drawRect(
        const Rect.fromLTRB(-width5, -width5, width5, -width10),
        Paint()..color = black,
      );
    }

    final Image incrementalMatrixImage = await toImage(
      (Canvas canvas) {
        paint(canvas, (Canvas canvas) {
          final Matrix4 matrix = Matrix4.identity();
          matrix.setEntry(3, 2, 0.001);
          canvas.transform(matrix.storage);
          matrix.setRotationX(rotateAroundX);
          canvas.transform(matrix.storage);
          matrix.setRotationY(rotateAroundY);
          canvas.transform(matrix.storage);
        });
      },
      width,
      height,
    );
    final Image combinedMatrixImage = await toImage(
      (Canvas canvas) {
        paint(canvas, (Canvas canvas) {
          final Matrix4 matrix = Matrix4.identity();
          matrix.setEntry(3, 2, 0.001);
          matrix.rotateX(rotateAroundX);
          matrix.rotateY(rotateAroundY);
          canvas.transform(matrix.storage);
        });
      },
      width,
      height,
    );

    final bool areEqual = await comparer.fuzzyCompareImages(
      incrementalMatrixImage,
      combinedMatrixImage,
    );

    expect(areEqual, true);
  });

  test('Path effects from Paragraphs do not affect further rendering', () async {
    void drawText(
      Canvas canvas,
      String content,
      Offset offset, {
      TextDecorationStyle style = TextDecorationStyle.solid,
    }) {
      final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle());
      builder.pushStyle(
        TextStyle(
          decoration: TextDecoration.underline,
          decorationColor: const Color(0xFF0000FF),
          fontFamily: 'Ahem',
          fontSize: 10,
          color: const Color(0xFF000000),
          decorationStyle: style,
        ),
      );
      builder.addText(content);
      final Paragraph paragraph = builder.build();
      paragraph.layout(const ParagraphConstraints(width: 100));
      canvas.drawParagraph(paragraph, offset);
    }

    final Image image = await toImage(
      (Canvas canvas) {
        canvas.drawColor(const Color(0xFFFFFFFF), BlendMode.srcOver);
        final Paint paint =
            Paint()
              ..style = PaintingStyle.stroke
              ..strokeWidth = 5;
        drawText(canvas, 'Hello World', const Offset(20, 10));
        canvas.drawCircle(const Offset(150, 25), 15, paint..color = const Color(0xFF00FF00));
        drawText(canvas, 'Regular text', const Offset(20, 60));
        canvas.drawCircle(const Offset(150, 75), 15, paint..color = const Color(0xFFFFFF00));
        drawText(canvas, 'Dotted text', const Offset(20, 110), style: TextDecorationStyle.dotted);
        canvas.drawCircle(const Offset(150, 125), 15, paint..color = const Color(0xFFFF0000));
        drawText(canvas, 'Dashed text', const Offset(20, 160), style: TextDecorationStyle.dashed);
        canvas.drawCircle(const Offset(150, 175), 15, paint..color = const Color(0xFFFF0000));
        drawText(canvas, 'Wavy text', const Offset(20, 210), style: TextDecorationStyle.wavy);
        canvas.drawCircle(const Offset(150, 225), 15, paint..color = const Color(0xFFFF0000));
      },
      200,
      250,
    );
    expect(image.width, equals(200));
    expect(image.height, equals(250));

    await comparer.addGoldenImage(image, 'dotted_path_effect_mixed_with_stroked_geometry.png');
  });

  test('Gradients with matrices in Paragraphs render correctly', () async {
    final Image image = await toImage(
      (Canvas canvas) {
        final Paint p = Paint();
        final Float64List transform = Float64List.fromList(<double>[
          86.80000129342079,
          0.0,
          0.0,
          0.0,
          0.0,
          94.5,
          0.0,
          0.0,
          0.0,
          0.0,
          1.0,
          0.0,
          60.0,
          224.310302734375,
          0.0,
          1.0,
        ]);
        p.shader = Gradient.radial(
          const Offset(2.5, 0.33),
          0.8,
          <Color>[
            const Color(0xffff0000),
            const Color(0xff00ff00),
            const Color(0xff0000ff),
            const Color(0xffff00ff),
          ],
          <double>[0.0, 0.3, 0.7, 0.9],
          TileMode.mirror,
          transform,
          const Offset(2.55, 0.4),
        );
        final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle());
        builder.pushStyle(TextStyle(foreground: p, fontSize: 200));
        builder.addText('Woodstock!');
        final Paragraph paragraph = builder.build();
        paragraph.layout(const ParagraphConstraints(width: 1000));
        canvas.drawParagraph(paragraph, const Offset(10, 150));
      },
      600,
      400,
    );
    expect(image.width, equals(600));
    expect(image.height, equals(400));

    await comparer.addGoldenImage(image, 'text_with_gradient_with_matrix.png');
  });

  test('toImageSync - too big', () async {
    PictureRecorder recorder = PictureRecorder();
    Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint()..color = const Color(0xFF123456));
    final Picture picture = recorder.endRecording();
    final Image image = picture.toImageSync(300000, 4000000);
    picture.dispose();

    expect(image.width, 300000);
    expect(image.height, 4000000);

    recorder = PictureRecorder();
    canvas = Canvas(recorder);

    if (impellerEnabled) {
      // Impeller tries to automagically scale this. See
      // https://github.com/flutter/flutter/issues/128885
      canvas.drawImage(image, Offset.zero, Paint());
      return;
    }
    // On a slower CI machine, the raster thread may get behind the UI thread
    // here. However, once the image is in an error state it will immediately
    // throw on subsequent attempts.
    bool caughtException = false;
    for (int iterations = 0; iterations < 1000; iterations += 1) {
      try {
        canvas.drawImage(image, Offset.zero, Paint());
      } on PictureRasterizationException catch (e) {
        caughtException = true;
        expect(
          e.message,
          contains(
            'unable to create bitmap render target at specified size ${image.width}x${image.height}',
          ),
        );
        break;
      }
      // Let the event loop turn.
      await Future<void>.delayed(const Duration(milliseconds: 1));
    }
    expect(caughtException, true);
    expect(() => canvas.drawImageRect(image, Rect.zero, Rect.zero, Paint()), throwsException);
    expect(() => canvas.drawImageNine(image, Rect.zero, Rect.zero, Paint()), throwsException);
    expect(
      () => canvas.drawAtlas(image, <RSTransform>[], <Rect>[], null, null, null, Paint()),
      throwsException,
    );
  });

  test('toImageSync - succeeds', () async {
    PictureRecorder recorder = PictureRecorder();
    Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint()..color = const Color(0xFF123456));
    final Picture picture = recorder.endRecording();
    final Image image = picture.toImageSync(30, 40);
    picture.dispose();

    expect(image.width, 30);
    expect(image.height, 40);

    recorder = PictureRecorder();
    canvas = Canvas(recorder);
    expect(() => canvas.drawImage(image, Offset.zero, Paint()), returnsNormally);
    expect(() => canvas.drawImageRect(image, Rect.zero, Rect.zero, Paint()), returnsNormally);
    expect(() => canvas.drawImageNine(image, Rect.zero, Rect.zero, Paint()), returnsNormally);
    expect(
      () => canvas.drawAtlas(image, <RSTransform>[], <Rect>[], null, null, null, Paint()),
      returnsNormally,
    );
  });

  test('toImageSync - toByteData', () async {
    const Color color = Color(0xFF123456);
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint()..color = color);
    final Picture picture = recorder.endRecording();
    final Image image = picture.toImageSync(6, 8);
    picture.dispose();

    expect(image.width, 6);
    expect(image.height, 8);

    final ByteData? data = await image.toByteData();

    expect(data, isNotNull);
    expect(data!.lengthInBytes, 6 * 8 * 4);
    expect(data.buffer.asUint8List()[0], 0x12);
    expect(data.buffer.asUint8List()[1], 0x34);
    expect(data.buffer.asUint8List()[2], 0x56);
    expect(data.buffer.asUint8List()[3], 0xFF);
  });

  test('toImage and toImageSync have identical contents', () async {
    // Note: on linux this stil seems to be different.
    // TODO(jonahwilliams): https://github.com/flutter/flutter/issues/108835
    if (Platform.isLinux) {
      return;
    }

    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawRect(
      const Rect.fromLTWH(20, 20, 100, 100),
      Paint()..color = const Color(0xA0FF6D00),
    );
    final Picture picture = recorder.endRecording();
    final Image toImageImage = await picture.toImage(200, 200);
    final Image toImageSyncImage = picture.toImageSync(200, 200);

    // To trigger observable difference in alpha, draw image
    // on a second canvas.
    Future<ByteData> drawOnCanvas(Image image) async {
      final PictureRecorder recorder = PictureRecorder();
      final Canvas canvas = Canvas(recorder);
      canvas.drawPaint(Paint()..color = const Color(0x4FFFFFFF));
      canvas.drawImage(image, Offset.zero, Paint());
      final Image resultImage = await recorder.endRecording().toImage(200, 200);
      return (await resultImage.toByteData())!;
    }

    final ByteData dataSync = await drawOnCanvas(toImageImage);
    final ByteData data = await drawOnCanvas(toImageSyncImage);
    expect(data.buffer.asUint8List(), equals(dataSync.buffer.asUint8List()));
  });

  test('Canvas.drawParagraph throws when Paragraph.layout was not called', () async {
    // Regression test for https://github.com/flutter/flutter/issues/97172
    expect(() {
      toImage(
        (Canvas canvas) {
          final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle());
          builder.addText('Woodstock!');
          final Paragraph woodstock = builder.build();
          canvas.drawParagraph(woodstock, const Offset(0, 50));
        },
        100,
        100,
      );
    }, throwsA(isA<AssertionError>()));
  });

  Future<Image> drawText(String text) {
    return toImage(
      (Canvas canvas) {
        final ParagraphBuilder builder = ParagraphBuilder(
          ParagraphStyle(
            fontFamily: 'RobotoSerif',
            fontStyle: FontStyle.normal,
            fontWeight: FontWeight.normal,
            fontSize: 15.0,
          ),
        );
        builder.pushStyle(TextStyle(color: const Color(0xFF0000FF)));
        builder.addText(text);

        final Paragraph paragraph = builder.build();
        paragraph.layout(const ParagraphConstraints(width: 20 * 5.0));

        canvas.drawParagraph(paragraph, Offset.zero);
      },
      100,
      100,
    );
  }

  test('Canvas.drawParagraph renders tab as space instead of tofu', () async {
    // Skia renders a tofu if the font does not have a glyph for a character.
    // However, Flutter opts-in to a Skia feature to render tabs as a single space.
    // See: https://github.com/flutter/flutter/issues/79153
    final File file = File(
      path.join(_flutterBuildPath, 'flutter', 'third_party', 'txt', 'assets', 'Roboto-Regular.ttf'),
    );
    final Uint8List fontData = await file.readAsBytes();
    await loadFontFromList(fontData, fontFamily: 'RobotoSerif');

    // The backspace character, \b, does not have a corresponding glyph and is rendered as a tofu.
    final Image tabImage = await drawText('>\t<');
    final Image spaceImage = await drawText('> <');
    final Image tofuImage = await drawText('>\b<');

    // The tab's image should be identical to the space's image but not the tofu's image.
    final bool tabToSpaceComparison = await comparer.fuzzyCompareImages(tabImage, spaceImage);
    final bool tabToTofuComparison = await comparer.fuzzyCompareImages(tabImage, tofuImage);

    expect(tabToSpaceComparison, isTrue);
    expect(tabToTofuComparison, isFalse);
  });

  test('drawRect, drawOval, and clipRect render with unsorted rectangles', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    canvas.drawColor(const Color(0xFFE0E0E0), BlendMode.src);

    void draw(Rect rect, double x, double y, Color color) {
      final Paint paint =
          Paint()
            ..color = color
            ..strokeWidth = 5.0;

      final Rect tallThin = Rect.fromLTRB(
        min(rect.left, rect.right) - 10,
        rect.top,
        min(rect.left, rect.right) - 10,
        rect.bottom,
      );
      final Rect wideThin = Rect.fromLTRB(
        rect.left,
        min(rect.top, rect.bottom) - 10,
        rect.right,
        min(rect.top, rect.bottom) - 10,
      );

      canvas.save();
      canvas.translate(x, y);

      paint.style = PaintingStyle.fill;
      canvas.drawRect(rect, paint);
      canvas.drawRect(tallThin, paint);
      canvas.drawRect(wideThin, paint);

      canvas.save();
      canvas.translate(0, 100);
      paint.style = PaintingStyle.stroke;
      canvas.drawRect(rect, paint);
      canvas.drawRect(tallThin, paint);
      canvas.drawRect(wideThin, paint);
      canvas.restore();

      canvas.save();
      canvas.translate(100, 0);
      paint.style = PaintingStyle.fill;
      canvas.drawOval(rect, paint);
      canvas.drawOval(tallThin, paint);
      canvas.drawOval(wideThin, paint);
      canvas.restore();

      canvas.save();
      canvas.translate(100, 100);
      paint.style = PaintingStyle.stroke;
      canvas.drawOval(rect, paint);
      canvas.drawOval(tallThin, paint);
      canvas.drawOval(wideThin, paint);
      canvas.restore();

      canvas.save();
      canvas.translate(50, 50);

      canvas.save();
      canvas.clipRect(rect);
      canvas.drawPaint(paint);
      canvas.restore();

      canvas.save();
      canvas.clipRect(tallThin);
      canvas.drawPaint(paint);
      canvas.restore();

      canvas.save();
      canvas.clipRect(wideThin);
      canvas.drawPaint(paint);
      canvas.restore();

      canvas.restore();

      canvas.restore();
    }

    draw(const Rect.fromLTRB(10, 10, 40, 40), 50, 50, const Color(0xFF2196F3));
    draw(const Rect.fromLTRB(40, 10, 10, 40), 250, 50, const Color(0xFF4CAF50));
    draw(const Rect.fromLTRB(10, 40, 40, 10), 50, 250, const Color(0xFF9C27B0));
    draw(const Rect.fromLTRB(40, 40, 10, 10), 250, 250, const Color(0xFFFF9800));

    final Picture picture = recorder.endRecording();
    final Image image = await picture.toImage(450, 450);
    await comparer.addGoldenImage(image, 'render_unordered_rects.png');
  });

  test('Canvas.translate affects canvas.getTransform', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.translate(12, 14.5);
    final Float64List matrix = Matrix4.translationValues(12, 14.5, 0).storage;
    final Float64List curMatrix = canvas.getTransform();
    expect(curMatrix, closeToTransform(matrix));
    canvas.translate(10, 10);
    final Float64List newCurMatrix = canvas.getTransform();
    expect(newCurMatrix, isNot(closeToTransform(matrix)));
    expect(curMatrix, closeToTransform(matrix));
  });

  test('Canvas.scale affects canvas.getTransform', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.scale(12, 14.5);
    final Float64List matrix = Matrix4.diagonal3Values(12, 14.5, 1).storage;
    final Float64List curMatrix = canvas.getTransform();
    expect(curMatrix, closeToTransform(matrix));
    canvas.scale(10, 10);
    final Float64List newCurMatrix = canvas.getTransform();
    expect(newCurMatrix, isNot(closeToTransform(matrix)));
    expect(curMatrix, closeToTransform(matrix));
  });

  test('Canvas.rotate affects canvas.getTransform', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.rotate(pi);
    final Float64List matrix = Matrix4.rotationZ(pi).storage;
    final Float64List curMatrix = canvas.getTransform();
    expect(curMatrix, closeToTransform(matrix));
    canvas.rotate(pi / 2);
    final Float64List newCurMatrix = canvas.getTransform();
    expect(newCurMatrix, isNot(closeToTransform(matrix)));
    expect(curMatrix, closeToTransform(matrix));
  });

  test('Canvas.skew affects canvas.getTransform', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.skew(12, 14.5);
    final Float64List matrix =
        (Matrix4.identity()
              ..setEntry(0, 1, 12)
              ..setEntry(1, 0, 14.5))
            .storage;
    final Float64List curMatrix = canvas.getTransform();
    expect(curMatrix, closeToTransform(matrix));
    canvas.skew(10, 10);
    final Float64List newCurMatrix = canvas.getTransform();
    expect(newCurMatrix, isNot(closeToTransform(matrix)));
    expect(curMatrix, closeToTransform(matrix));
  });

  test('Canvas.transform affects canvas.getTransform', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    final Float64List matrix =
        (Matrix4.identity()
              ..translate(12.0, 14.5)
              ..scale(12.0, 14.5))
            .storage;
    canvas.transform(matrix);
    final Float64List curMatrix = canvas.getTransform();
    expect(curMatrix, closeToTransform(matrix));
    canvas.translate(10, 10);
    final Float64List newCurMatrix = canvas.getTransform();
    expect(newCurMatrix, isNot(closeToTransform(matrix)));
    expect(curMatrix, closeToTransform(matrix));
  });

  test('Canvas.clipRect affects canvas.getClipBounds', () async {
    void testRect(Rect clipRect, bool doAA) {
      final PictureRecorder recorder = PictureRecorder();
      final Canvas canvas = Canvas(recorder);
      canvas.clipRect(clipRect, doAntiAlias: doAA);

      final Rect clipSortedBounds = Rect.fromLTRB(
        min(clipRect.left, clipRect.right),
        min(clipRect.top, clipRect.bottom),
        max(clipRect.left, clipRect.right),
        max(clipRect.top, clipRect.bottom),
      );
      Rect clipExpandedBounds;
      if (doAA) {
        clipExpandedBounds = Rect.fromLTRB(
          clipSortedBounds.left.floorToDouble(),
          clipSortedBounds.top.floorToDouble(),
          clipSortedBounds.right.ceilToDouble(),
          clipSortedBounds.bottom.ceilToDouble(),
        );
      } else {
        clipExpandedBounds = clipSortedBounds;
      }

      // Save initial return values for testing restored values
      final Rect initialLocalBounds = canvas.getLocalClipBounds();
      final Rect initialDestinationBounds = canvas.getDestinationClipBounds();
      expect(initialLocalBounds, closeToRect(clipExpandedBounds));
      expect(initialDestinationBounds, closeToRect(clipExpandedBounds));

      canvas.save();
      canvas.clipRect(const Rect.fromLTRB(0, 0, 15, 15));
      // Both clip bounds have changed
      expect(canvas.getLocalClipBounds(), isNot(closeToRect(clipExpandedBounds)));
      expect(canvas.getDestinationClipBounds(), isNot(closeToRect(clipExpandedBounds)));
      // Previous return values have not changed
      expect(initialLocalBounds, closeToRect(clipExpandedBounds));
      expect(initialDestinationBounds, closeToRect(clipExpandedBounds));
      canvas.restore();

      // save/restore returned the values to their original values
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);

      canvas.save();
      canvas.scale(2, 2);
      final Rect scaledExpandedBounds = Rect.fromLTRB(
        clipExpandedBounds.left / 2.0,
        clipExpandedBounds.top / 2.0,
        clipExpandedBounds.right / 2.0,
        clipExpandedBounds.bottom / 2.0,
      );
      expect(canvas.getLocalClipBounds(), closeToRect(scaledExpandedBounds));
      // Destination bounds are unaffected by transform
      expect(canvas.getDestinationClipBounds(), closeToRect(clipExpandedBounds));
      canvas.restore();

      // save/restore returned the values to their original values
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
    }

    testRect(const Rect.fromLTRB(10.2, 11.3, 20.4, 25.7), false);
    testRect(const Rect.fromLTRB(10.2, 11.3, 20.4, 25.7), true);

    // LR swapped
    testRect(const Rect.fromLTRB(20.4, 11.3, 10.2, 25.7), false);
    testRect(const Rect.fromLTRB(20.4, 11.3, 10.2, 25.7), true);

    // TB swapped
    testRect(const Rect.fromLTRB(10.2, 25.7, 20.4, 11.3), false);
    testRect(const Rect.fromLTRB(10.2, 25.7, 20.4, 11.3), true);

    // LR and TB swapped
    testRect(const Rect.fromLTRB(20.4, 25.7, 10.2, 11.3), false);
    testRect(const Rect.fromLTRB(20.4, 25.7, 10.2, 11.3), true);
  });

  test('Canvas.clipRect with matrix affects canvas.getClipBounds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect clipBounds1 = Rect.fromLTRB(0.0, 0.0, 10.0, 10.0);
    const Rect clipBounds2 = Rect.fromLTRB(10.0, 10.0, 20.0, 20.0);

    canvas.save();
    canvas.clipRect(clipBounds1, doAntiAlias: false);
    canvas.translate(0, 10.0);
    canvas.clipRect(clipBounds1, doAntiAlias: false);
    expect(canvas.getDestinationClipBounds().isEmpty, isTrue);
    canvas.restore();

    canvas.save();
    canvas.clipRect(clipBounds1, doAntiAlias: false);
    canvas.translate(-10.0, -10.0);
    canvas.clipRect(clipBounds2, doAntiAlias: false);
    expect(canvas.getDestinationClipBounds(), clipBounds1);
    canvas.restore();
  });

  test('Canvas.clipRRect(doAA=true) affects canvas.getClipBounds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect clipBounds = Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
    const Rect clipExpandedBounds = Rect.fromLTRB(10, 11, 21, 26);
    final RRect clip = RRect.fromRectAndRadius(clipBounds, const Radius.circular(3));
    canvas.clipRRect(clip);

    // Save initial return values for testing restored values
    final Rect initialLocalBounds = canvas.getLocalClipBounds();
    final Rect initialDestinationBounds = canvas.getDestinationClipBounds();
    expect(initialLocalBounds, closeToRect(clipExpandedBounds));
    expect(initialDestinationBounds, closeToRect(clipExpandedBounds));

    canvas.save();
    canvas.clipRect(const Rect.fromLTRB(0, 0, 15, 15));
    // Both clip bounds have changed
    expect(canvas.getLocalClipBounds(), isNot(closeToRect(clipExpandedBounds)));
    expect(canvas.getDestinationClipBounds(), isNot(closeToRect(clipExpandedBounds)));
    // Previous return values have not changed
    expect(initialLocalBounds, closeToRect(clipExpandedBounds));
    expect(initialDestinationBounds, closeToRect(clipExpandedBounds));
    canvas.restore();

    // save/restore returned the values to their original values
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);

    canvas.save();
    canvas.scale(2, 2);
    const Rect scaledExpandedBounds = Rect.fromLTRB(5, 5.5, 10.5, 13);
    expect(canvas.getLocalClipBounds(), closeToRect(scaledExpandedBounds));
    // Destination bounds are unaffected by transform
    expect(canvas.getDestinationClipBounds(), closeToRect(clipExpandedBounds));
    canvas.restore();

    // save/restore returned the values to their original values
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
  });

  test('Canvas.clipRRect(doAA=false) affects canvas.getClipBounds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect clipBounds = Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
    final RRect clip = RRect.fromRectAndRadius(clipBounds, const Radius.circular(3));
    canvas.clipRRect(clip, doAntiAlias: false);

    // Save initial return values for testing restored values
    final Rect initialLocalBounds = canvas.getLocalClipBounds();
    final Rect initialDestinationBounds = canvas.getDestinationClipBounds();
    expect(initialLocalBounds, closeToRect(clipBounds));
    expect(initialDestinationBounds, closeToRect(clipBounds));

    canvas.save();
    canvas.clipRect(const Rect.fromLTRB(0, 0, 15, 15), doAntiAlias: false);
    // Both clip bounds have changed
    expect(canvas.getLocalClipBounds(), isNot(closeToRect(clipBounds)));
    expect(canvas.getDestinationClipBounds(), isNot(closeToRect(clipBounds)));
    // Previous return values have not changed
    expect(initialLocalBounds, closeToRect(clipBounds));
    expect(initialDestinationBounds, closeToRect(clipBounds));
    canvas.restore();

    // save/restore returned the values to their original values
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);

    canvas.save();
    canvas.scale(2, 2);
    const Rect scaledClipBounds = Rect.fromLTRB(5.1, 5.65, 10.2, 12.85);
    expect(canvas.getLocalClipBounds(), closeToRect(scaledClipBounds));
    // Destination bounds are unaffected by transform
    expect(canvas.getDestinationClipBounds(), closeToRect(clipBounds));
    canvas.restore();

    // save/restore returned the values to their original values
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
  });

  test('Canvas.clipRRect with matrix affects canvas.getClipBounds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect clipBounds1 = Rect.fromLTRB(0.0, 0.0, 10.0, 10.0);
    const Rect clipBounds2 = Rect.fromLTRB(10.0, 10.0, 20.0, 20.0);
    final RRect clip1 = RRect.fromRectAndRadius(clipBounds1, const Radius.circular(3));
    final RRect clip2 = RRect.fromRectAndRadius(clipBounds2, const Radius.circular(3));

    canvas.save();
    canvas.clipRRect(clip1, doAntiAlias: false);
    canvas.translate(0, 10.0);
    canvas.clipRRect(clip1, doAntiAlias: false);
    expect(canvas.getDestinationClipBounds().isEmpty, isTrue);
    canvas.restore();

    canvas.save();
    canvas.clipRRect(clip1, doAntiAlias: false);
    canvas.translate(-10.0, -10.0);
    canvas.clipRRect(clip2, doAntiAlias: false);
    expect(canvas.getDestinationClipBounds(), clipBounds1);
    canvas.restore();
  });

  test('Canvas.clipPath(doAA=true) affects canvas.getClipBounds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect clipBounds = Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
    const Rect clipExpandedBounds = Rect.fromLTRB(10, 11, 21, 26);
    final Path clip =
        Path()
          ..addRect(clipBounds)
          ..addOval(clipBounds);
    canvas.clipPath(clip);

    // Save initial return values for testing restored values
    final Rect initialLocalBounds = canvas.getLocalClipBounds();
    final Rect initialDestinationBounds = canvas.getDestinationClipBounds();
    expect(initialLocalBounds, closeToRect(clipExpandedBounds));
    expect(initialDestinationBounds, closeToRect(clipExpandedBounds));

    canvas.save();
    canvas.clipRect(const Rect.fromLTRB(0, 0, 15, 15));
    // Both clip bounds have changed
    expect(canvas.getLocalClipBounds(), isNot(closeToRect(clipExpandedBounds)));
    expect(canvas.getDestinationClipBounds(), isNot(closeToRect(clipExpandedBounds)));
    // Previous return values have not changed
    expect(initialLocalBounds, closeToRect(clipExpandedBounds));
    expect(initialDestinationBounds, closeToRect(clipExpandedBounds));
    canvas.restore();

    // save/restore returned the values to their original values
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);

    canvas.save();
    canvas.scale(2, 2);
    const Rect scaledExpandedBounds = Rect.fromLTRB(5, 5.5, 10.5, 13);
    expect(canvas.getLocalClipBounds(), closeToRect(scaledExpandedBounds));
    // Destination bounds are unaffected by transform
    expect(canvas.getDestinationClipBounds(), closeToRect(clipExpandedBounds));
    canvas.restore();

    // save/restore returned the values to their original values
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
  });

  test('Canvas.clipPath(doAA=false) affects canvas.getClipBounds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect clipBounds = Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
    final Path clip =
        Path()
          ..addRect(clipBounds)
          ..addOval(clipBounds);
    canvas.clipPath(clip, doAntiAlias: false);

    // Save initial return values for testing restored values
    final Rect initialLocalBounds = canvas.getLocalClipBounds();
    final Rect initialDestinationBounds = canvas.getDestinationClipBounds();
    expect(initialLocalBounds, closeToRect(clipBounds));
    expect(initialDestinationBounds, closeToRect(clipBounds));

    canvas.save();
    canvas.clipRect(const Rect.fromLTRB(0, 0, 15, 15), doAntiAlias: false);
    // Both clip bounds have changed
    expect(canvas.getLocalClipBounds(), isNot(closeToRect(clipBounds)));
    expect(canvas.getDestinationClipBounds(), isNot(closeToRect(clipBounds)));
    // Previous return values have not changed
    expect(initialLocalBounds, closeToRect(clipBounds));
    expect(initialDestinationBounds, closeToRect(clipBounds));
    canvas.restore();

    // save/restore returned the values to their original values
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);

    canvas.save();
    canvas.scale(2, 2);
    const Rect scaledClipBounds = Rect.fromLTRB(5.1, 5.65, 10.2, 12.85);
    expect(canvas.getLocalClipBounds(), closeToRect(scaledClipBounds));
    // Destination bounds are unaffected by transform
    expect(canvas.getDestinationClipBounds(), closeToRect(clipBounds));
    canvas.restore();

    // save/restore returned the values to their original values
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
  });

  test('Canvas.clipPath with matrix affects canvas.getClipBounds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect clipBounds1 = Rect.fromLTRB(0.0, 0.0, 10.0, 10.0);
    const Rect clipBounds2 = Rect.fromLTRB(10.0, 10.0, 20.0, 20.0);
    final Path clip1 =
        Path()
          ..addRect(clipBounds1)
          ..addOval(clipBounds1);
    final Path clip2 =
        Path()
          ..addRect(clipBounds2)
          ..addOval(clipBounds2);

    canvas.save();
    canvas.clipPath(clip1, doAntiAlias: false);
    canvas.translate(0, 10.0);
    canvas.clipPath(clip1, doAntiAlias: false);
    expect(canvas.getDestinationClipBounds().isEmpty, isTrue);
    canvas.restore();

    canvas.save();
    canvas.clipPath(clip1, doAntiAlias: false);
    canvas.translate(-10.0, -10.0);
    canvas.clipPath(clip2, doAntiAlias: false);
    expect(canvas.getDestinationClipBounds(), clipBounds1);
    canvas.restore();
  });

  test('Canvas.clipRect(diff) does not affect canvas.getClipBounds', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    const Rect clipBounds = Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
    canvas.clipRect(clipBounds, doAntiAlias: false);

    // Save initial return values for testing restored values
    final Rect initialLocalBounds = canvas.getLocalClipBounds();
    final Rect initialDestinationBounds = canvas.getDestinationClipBounds();
    expect(initialLocalBounds, closeToRect(clipBounds));
    expect(initialDestinationBounds, closeToRect(clipBounds));

    canvas.clipRect(
      const Rect.fromLTRB(0, 0, 15, 15),
      clipOp: ClipOp.difference,
      doAntiAlias: false,
    );
    expect(canvas.getLocalClipBounds(), initialLocalBounds);
    expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
  });

  test('RestoreToCount can work', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.save();
    canvas.save();
    canvas.save();
    canvas.save();
    canvas.save();
    expect(canvas.getSaveCount(), equals(6));
    canvas.restoreToCount(2);
    expect(canvas.getSaveCount(), equals(2));
    canvas.restore();
    expect(canvas.getSaveCount(), equals(1));
  });

  test('RestoreToCount count less than 1, the stack should be reset', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.save();
    canvas.save();
    canvas.save();
    canvas.save();
    canvas.save();
    expect(canvas.getSaveCount(), equals(6));
    canvas.restoreToCount(0);
    expect(canvas.getSaveCount(), equals(1));
  });

  test('RestoreToCount count greater than current [getSaveCount], nothing would happend', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.save();
    canvas.save();
    canvas.save();
    canvas.save();
    canvas.save();
    expect(canvas.getSaveCount(), equals(6));
    canvas.restoreToCount(canvas.getSaveCount() + 1);
    expect(canvas.getSaveCount(), equals(6));
  });

  test('TextDecoration renders non-solid lines', () async {
    final File file = File(
      path.join(_flutterBuildPath, 'flutter', 'third_party', 'txt', 'assets', 'Roboto-Regular.ttf'),
    );
    final Uint8List fontData = await file.readAsBytes();
    await loadFontFromList(fontData, fontFamily: 'RobotoSlab');

    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    for (final (int index, TextDecorationStyle style) in TextDecorationStyle.values.indexed) {
      final ParagraphBuilder builder = ParagraphBuilder(ParagraphStyle());
      builder.pushStyle(
        TextStyle(
          decoration: TextDecoration.underline,
          decorationStyle: style,
          decorationThickness: 1.0,
          decorationColor: const Color(0xFFFF0000),
          fontFamily: 'RobotoSlab',
          fontSize: 24.0,
          foreground: Paint()..color = const Color(0xFF0000FF),
        ),
      );

      builder.addText(style.name);
      final Paragraph paragraph = builder.build();
      paragraph.layout(const ParagraphConstraints(width: 1000));

      // Draw and layout based on the index vertically.
      canvas.drawParagraph(paragraph, Offset(0, index * 40.0));
    }

    final Picture picture = recorder.endRecording();
    final Image image = await picture.toImage(200, 200);
    await comparer.addGoldenImage(image, 'text_decoration.png');
  });

  test('Paint, when copied, has equivalent fields', () {
    final Paint paint =
        Paint()
          ..color = const Color(0xFF0000FF)
          ..strokeWidth = 10.0
          ..strokeCap = StrokeCap.round
          ..strokeJoin = StrokeJoin.round
          ..style = PaintingStyle.stroke
          ..blendMode = BlendMode.srcOver
          ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 10.0)
          ..filterQuality = FilterQuality.high
          ..colorFilter = const ColorFilter.mode(Color(0xFF00FF00), BlendMode.color)
          ..imageFilter = ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0);

    final Paint paintCopy = Paint.from(paint);
    expect(paintCopy.color, equals(const Color(0xFF0000FF)));
    expect(paintCopy.strokeWidth, equals(10.0));
    expect(paintCopy.strokeCap, equals(StrokeCap.round));
    expect(paintCopy.strokeJoin, equals(StrokeJoin.round));
    expect(paintCopy.style, equals(PaintingStyle.stroke));
    expect(paintCopy.blendMode, equals(BlendMode.srcOver));
    expect(paintCopy.maskFilter, equals(const MaskFilter.blur(BlurStyle.normal, 10.0)));
    expect(paintCopy.filterQuality, equals(FilterQuality.high));
    expect(
      paintCopy.colorFilter,
      equals(const ColorFilter.mode(Color(0xFF00FF00), BlendMode.color)),
    );
    expect(paintCopy.imageFilter, equals(ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0)));
  });

  test('Paint, when copied, does not mutate the original instance', () {
    final Paint paint =
        Paint()
          ..color = const Color(0xFF0000FF)
          ..strokeWidth = 10.0
          ..strokeCap = StrokeCap.round
          ..strokeJoin = StrokeJoin.round
          ..style = PaintingStyle.stroke
          ..blendMode = BlendMode.srcOver
          ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 10.0)
          ..filterQuality = FilterQuality.high
          ..colorFilter = const ColorFilter.mode(Color(0xFF00FF00), BlendMode.color)
          ..imageFilter = ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0);

    // Make a copy, and change every field of the copy.
    Paint.from(paint)
      ..color = const Color(0xFF00FF00)
      ..strokeWidth = 20.0
      ..strokeCap = StrokeCap.butt
      ..strokeJoin = StrokeJoin.bevel
      ..style = PaintingStyle.fill
      ..blendMode = BlendMode.srcIn
      ..maskFilter = const MaskFilter.blur(BlurStyle.solid, 20.0)
      ..filterQuality = FilterQuality.none
      ..colorFilter = const ColorFilter.mode(Color(0xFFFF0000), BlendMode.modulate)
      ..imageFilter = ImageFilter.blur(sigmaX: 20.0, sigmaY: 20.0);

    // The original paint should not have changed.
    expect(paint.color, equals(const Color(0xFF0000FF)));
    expect(paint.strokeWidth, equals(10.0));
    expect(paint.strokeCap, equals(StrokeCap.round));
    expect(paint.strokeJoin, equals(StrokeJoin.round));
    expect(paint.style, equals(PaintingStyle.stroke));
    expect(paint.blendMode, equals(BlendMode.srcOver));
    expect(paint.maskFilter, equals(const MaskFilter.blur(BlurStyle.normal, 10.0)));
    expect(paint.filterQuality, equals(FilterQuality.high));
    expect(paint.colorFilter, equals(const ColorFilter.mode(Color(0xFF00FF00), BlendMode.color)));
    expect(paint.imageFilter, equals(ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0)));
  });

  test('Paint, when copied, the original changing does not mutate the copy', () {
    final Paint paint =
        Paint()
          ..color = const Color(0xFF0000FF)
          ..strokeWidth = 10.0
          ..strokeCap = StrokeCap.round
          ..strokeJoin = StrokeJoin.round
          ..style = PaintingStyle.stroke
          ..blendMode = BlendMode.srcOver
          ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 10.0)
          ..filterQuality = FilterQuality.high
          ..colorFilter = const ColorFilter.mode(Color(0xFF00FF00), BlendMode.color)
          ..imageFilter = ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0);

    // Make a copy, and change every field of the original.
    final Paint paintCopy = Paint.from(paint);
    paint
      ..color = const Color(0xFF00FF00)
      ..strokeWidth = 20.0
      ..strokeCap = StrokeCap.butt
      ..strokeJoin = StrokeJoin.bevel
      ..style = PaintingStyle.fill
      ..blendMode = BlendMode.srcIn
      ..maskFilter = const MaskFilter.blur(BlurStyle.solid, 20.0)
      ..filterQuality = FilterQuality.none
      ..colorFilter = const ColorFilter.mode(Color(0xFFFF0000), BlendMode.modulate)
      ..imageFilter = ImageFilter.blur(sigmaX: 20.0, sigmaY: 20.0);

    // The copy should not have changed.
    expect(paintCopy.color, equals(const Color(0xFF0000FF)));
    expect(paintCopy.strokeWidth, equals(10.0));
    expect(paintCopy.strokeCap, equals(StrokeCap.round));
    expect(paintCopy.strokeJoin, equals(StrokeJoin.round));
    expect(paintCopy.style, equals(PaintingStyle.stroke));
    expect(paintCopy.blendMode, equals(BlendMode.srcOver));
    expect(paintCopy.maskFilter, equals(const MaskFilter.blur(BlurStyle.normal, 10.0)));
    expect(paintCopy.filterQuality, equals(FilterQuality.high));
    expect(
      paintCopy.colorFilter,
      equals(const ColorFilter.mode(Color(0xFF00FF00), BlendMode.color)),
    );
    expect(paintCopy.imageFilter, equals(ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0)));
  });

  test('DrawAtlas correctly copies color values into display list format', () async {
    final Image testImage = await createTestImage();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    // Make a drawAtlas call that should be solid red.
    canvas.drawAtlas(
      testImage,
      [
        RSTransform.fromComponents(
          rotation: 0,
          scale: 10,
          anchorX: 0,
          anchorY: 0,
          translateX: 0,
          translateY: 0,
        ),
      ],
      [const Rect.fromLTWH(0, 0, 1, 1)],
      [const Color.fromARGB(255, 255, 0, 0)],
      BlendMode.dst,
      null,
      Paint(),
    );

    final Image resultImage = await recorder.endRecording().toImage(1, 1);
    final ByteData? data = await resultImage.toByteData();
    if (data == null) {
      fail('Expected non-null byte data');
    }
    final int rgba = data.buffer.asUint32List()[0];
    expect(rgba, 0xFF0000FF);
  });

  test('DrawAtlas with no colors does not crash', () async {
    final Image testImage = await createTestImage();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    // Make a drawAtlas call that should be solid red.
    canvas.drawAtlas(
      testImage,
      [
        RSTransform.fromComponents(
          rotation: 0,
          scale: 10,
          anchorX: 0,
          anchorY: 0,
          translateX: 0,
          translateY: 0,
        ),
      ],
      [const Rect.fromLTWH(0, 0, 1, 1)],
      [],
      BlendMode.dst,
      null,
      Paint(),
    );

    final Image resultImage = await recorder.endRecording().toImage(1, 1);
    final ByteData? data = await resultImage.toByteData();
    expect(data, isNotNull);
  });

  Image makeCheckerBoard(int width, int height) {
    final recorder = PictureRecorder();
    final canvas = Canvas(recorder);

    const double left = 0;
    final double centerX = width * 0.5;
    final double right = width.toDouble();

    const double top = 0;
    final double centerY = height * 0.5;
    final double bottom = height.toDouble();

    canvas.drawRect(
      Rect.fromLTRB(left, top, centerX, centerY),
      Paint()..color = const Color.fromARGB(255, 0, 255, 0),
    );
    canvas.drawRect(
      Rect.fromLTRB(centerX, top, right, centerY),
      Paint()..color = const Color.fromARGB(255, 255, 255, 0),
    );
    canvas.drawRect(
      Rect.fromLTRB(left, centerY, centerX, bottom),
      Paint()..color = const Color.fromARGB(255, 0, 0, 255),
    );
    canvas.drawRect(
      Rect.fromLTRB(centerX, centerY, right, bottom),
      Paint()..color = const Color.fromARGB(255, 255, 0, 0),
    );

    final picture = recorder.endRecording();
    return picture.toImageSync(width, height);
  }

  Image renderingOpsWithTileMode(TileMode? tileMode) {
    final recorder = PictureRecorder();
    final canvas = Canvas(recorder);
    canvas.drawColor(const Color.fromARGB(255, 224, 224, 224), BlendMode.src);

    const Rect zone = Rect.fromLTWH(15, 15, 20, 20);
    final Rect arena = zone.inflate(15);
    const Rect ovalZone = Rect.fromLTWH(20, 15, 10, 20);

    final gradient = Gradient.linear(
      zone.topLeft,
      zone.bottomRight,
      <Color>[const Color.fromARGB(255, 0, 255, 0), const Color.fromARGB(255, 0, 0, 255)],
      <double>[0, 1],
    );
    final filter = ImageFilter.blur(sigmaX: 3.0, sigmaY: 3.0, tileMode: tileMode);
    final Paint white = Paint()..color = const Color.fromARGB(255, 255, 255, 255);
    final Paint grey = Paint()..color = const Color.fromARGB(255, 127, 127, 127);
    final Paint unblurredFill = Paint()..shader = gradient;
    final Paint blurredFill = Paint.from(unblurredFill)..imageFilter = filter;
    final Paint unblurredStroke =
        Paint.from(unblurredFill)
          ..style = PaintingStyle.stroke
          ..strokeCap = StrokeCap.round
          ..strokeJoin = StrokeJoin.round
          ..strokeWidth = 10;
    final Paint blurredStroke = Paint.from(unblurredStroke)..imageFilter = filter;
    final Image image = makeCheckerBoard(20, 20);
    const Rect imageBounds = Rect.fromLTRB(0, 0, 20, 20);
    const Rect imageCenter = Rect.fromLTRB(5, 5, 9, 9);
    final points = <Offset>[
      zone.topLeft,
      zone.topCenter,
      zone.topRight,
      zone.centerLeft,
      zone.center,
      zone.centerRight,
      zone.bottomLeft,
      zone.bottomCenter,
      zone.bottomRight,
    ];
    final vertices = Vertices(
      VertexMode.triangles,
      <Offset>[
        zone.topLeft,
        zone.bottomRight,
        zone.topRight,
        zone.topLeft,
        zone.bottomRight,
        zone.bottomLeft,
      ],
      colors: <Color>[
        const Color.fromARGB(255, 0, 255, 0),
        const Color.fromARGB(255, 255, 0, 0),
        const Color.fromARGB(255, 255, 255, 0),
        const Color.fromARGB(255, 0, 255, 0),
        const Color.fromARGB(255, 255, 0, 0),
        const Color.fromARGB(255, 0, 0, 255),
      ],
    );
    final atlasXforms = <RSTransform>[
      RSTransform.fromComponents(
        rotation: 0.0,
        scale: 1.0,
        anchorX: 0,
        anchorY: 0,
        translateX: zone.topLeft.dx,
        translateY: zone.topLeft.dy,
      ),
      RSTransform.fromComponents(
        rotation: pi / 2,
        scale: 1.0,
        anchorX: 0,
        anchorY: 0,
        translateX: zone.topRight.dx,
        translateY: zone.topRight.dy,
      ),
      RSTransform.fromComponents(
        rotation: pi,
        scale: 1.0,
        anchorX: 0,
        anchorY: 0,
        translateX: zone.bottomRight.dx,
        translateY: zone.bottomRight.dy,
      ),
      RSTransform.fromComponents(
        rotation: pi * 3 / 2,
        scale: 1.0,
        anchorX: 0,
        anchorY: 0,
        translateX: zone.bottomLeft.dx,
        translateY: zone.bottomLeft.dy,
      ),
      RSTransform.fromComponents(
        rotation: pi / 4,
        scale: 1.0,
        anchorX: 4,
        anchorY: 4,
        translateX: zone.center.dx,
        translateY: zone.center.dy,
      ),
    ];
    const atlasRects = <Rect>[
      Rect.fromLTRB(6, 6, 14, 14),
      Rect.fromLTRB(6, 6, 14, 14),
      Rect.fromLTRB(6, 6, 14, 14),
      Rect.fromLTRB(6, 6, 14, 14),
      Rect.fromLTRB(6, 6, 14, 14),
    ];

    const double pad = 10;
    final double offset = arena.width + pad;
    const int columns = 5;
    final Rect pairArena = Rect.fromLTRB(
      arena.left - 3,
      arena.top - 3,
      arena.right + 3,
      arena.bottom + offset + 3,
    );

    final List<void Function(Canvas canvas, Paint fill, Paint stroke)> renderers = [
      (canvas, fill, stroke) {
        canvas.saveLayer(zone.inflate(5), fill);
        canvas.drawLine(zone.topLeft, zone.bottomRight, unblurredStroke);
        canvas.drawLine(zone.topRight, zone.bottomLeft, unblurredStroke);
        canvas.restore();
      },
      (canvas, fill, stroke) => canvas.drawLine(zone.topLeft, zone.bottomRight, stroke),
      (canvas, fill, stroke) => canvas.drawRect(zone, fill),
      (canvas, fill, stroke) => canvas.drawOval(ovalZone, fill),
      (canvas, fill, stroke) => canvas.drawCircle(zone.center, zone.width * 0.5, fill),
      (canvas, fill, stroke) => canvas.drawRRect(RRect.fromRectXY(zone, 4.0, 4.0), fill),
      (canvas, fill, stroke) => canvas.drawDRRect(
        RRect.fromRectXY(zone, 4.0, 4.0),
        RRect.fromRectXY(zone.deflate(4), 4.0, 4.0),
        fill,
      ),
      (canvas, fill, stroke) => canvas.drawArc(zone, pi / 4, pi * 3 / 2, true, fill),
      (canvas, fill, stroke) => canvas.drawPath(
        Path()
          ..moveTo(zone.left, zone.top)
          ..lineTo(zone.right, zone.top)
          ..lineTo(zone.left, zone.bottom)
          ..lineTo(zone.right, zone.bottom),
        stroke,
      ),
      (canvas, fill, stroke) => canvas.drawImage(image, zone.topLeft, fill),
      (canvas, fill, stroke) => canvas.drawImageRect(image, imageBounds, zone.inflate(2), fill),
      (canvas, fill, stroke) => canvas.drawImageNine(image, imageCenter, zone.inflate(2), fill),
      (canvas, fill, stroke) => canvas.drawPoints(PointMode.points, points, stroke),
      (canvas, fill, stroke) => canvas.drawVertices(vertices, BlendMode.dstOver, fill),
      (canvas, fill, stroke) =>
          canvas.drawAtlas(image, atlasXforms, atlasRects, null, null, null, fill),
    ];

    canvas.save();
    canvas.translate(pad, pad);
    int renderIndex = 0;
    int rows = 0;
    while (renderIndex < renderers.length) {
      rows += 2;
      canvas.save();
      for (int col = 0; col < columns && renderIndex < renderers.length; col++) {
        final renderer = renderers[renderIndex++];
        canvas.drawRect(pairArena, grey);
        canvas.drawRect(arena, white);
        renderer(canvas, unblurredFill, unblurredStroke);
        canvas.save();
        canvas.translate(0, offset);
        canvas.drawRect(arena, white);
        renderer(canvas, blurredFill, blurredStroke);
        canvas.restore();
        canvas.translate(offset, 0);
      }
      canvas.restore();
      canvas.translate(0, offset * 2);
    }
    canvas.restore();

    final picture = recorder.endRecording();
    return picture.toImageSync((offset * columns + pad).round(), (offset * rows + pad).round());
  }

  test('Rendering ops with ImageFilter blur with default tile mode', () async {
    final image = renderingOpsWithTileMode(null);
    await comparer.addGoldenImage(
      image,
      'canvas_test_blurred_rendering_with_default_tile_mode.png',
    );
  });

  test('Rendering ops with ImageFilter blur with clamp tile mode', () async {
    final image = renderingOpsWithTileMode(TileMode.clamp);
    await comparer.addGoldenImage(image, 'canvas_test_blurred_rendering_with_clamp_tile_mode.png');
  });

  test('Rendering ops with ImageFilter blur with mirror tile mode', () async {
    final image = renderingOpsWithTileMode(TileMode.mirror);
    await comparer.addGoldenImage(image, 'canvas_test_blurred_rendering_with_mirror_tile_mode.png');
  });

  test('Rendering ops with ImageFilter blur with repeated tile mode', () async {
    final image = renderingOpsWithTileMode(TileMode.repeated);
    await comparer.addGoldenImage(
      image,
      'canvas_test_blurred_rendering_with_repeated_tile_mode.png',
    );
  });

  test('Rendering ops with ImageFilter blur with decal tile mode', () async {
    final image = renderingOpsWithTileMode(TileMode.decal);
    await comparer.addGoldenImage(image, 'canvas_test_blurred_rendering_with_decal_tile_mode.png');
  });
}

Future<Image> createTestImage() async {
  final PictureRecorder recorder = PictureRecorder();
  final Canvas recorderCanvas = Canvas(recorder);
  recorderCanvas.scale(1.0, 1.0);
  final Picture picture = recorder.endRecording();
  return picture.toImage(1, 1);
}

Matcher closeToRect(Rect rect) => _CloseToRectMatcher(rect);

final class _CloseToRectMatcher extends Matcher {
  const _CloseToRectMatcher(this._expectedRect);
  final Rect _expectedRect;

  @override
  bool matches(Object? item, Map<Object?, Object?> matchState) {
    if (item is! Rect) {
      return false;
    }
    return (item.left - _expectedRect.left).abs() < 1e-6 &&
        (item.top - _expectedRect.top).abs() < 1e-6 &&
        (item.right - _expectedRect.right).abs() < 1e-6 &&
        (item.bottom - _expectedRect.bottom).abs() < 1e-6;
  }

  @override
  Description describe(Description description) {
    return description.add('Rect is close (within 1e-6) to $_expectedRect');
  }
}

Matcher closeToTransform(Float64List expected) => _CloseToTransformMatcher(expected);

final class _CloseToTransformMatcher extends Matcher {
  _CloseToTransformMatcher(this._expected);
  final Float64List _expected;

  @override
  bool matches(Object? item, Map<Object?, Object?> matchState) {
    if (item is! Float64List) {
      return false;
    }
    if (item.length != 16 || _expected.length != 16) {
      return false;
    }
    for (int i = 0; i < 16; i++) {
      if ((item[i] - _expected[i]).abs() > 1e-10) {
        return false;
      }
    }
    return true;
  }

  @override
  Description describe(Description description) {
    return description.add('Transform is close (within 1e-10) to $_expected');
  }
}
