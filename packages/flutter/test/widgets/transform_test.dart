// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:math' as math;
import 'dart:ui' as ui;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  testWidgets('Transform origin', (WidgetTester tester) async {
    bool didReceiveTap = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              top: 100.0,
              left: 100.0,
              child: Container(width: 100.0, height: 100.0, color: const Color(0xFF0000FF)),
            ),
            Positioned(
              top: 100.0,
              left: 100.0,
              child: SizedBox(
                width: 100.0,
                height: 100.0,
                child: Transform(
                  transform: Matrix4.diagonal3Values(0.5, 0.5, 1.0),
                  origin: const Offset(100.0, 50.0),
                  child: GestureDetector(
                    onTap: () {
                      didReceiveTap = true;
                    },
                    child: Container(color: const Color(0xFF00FFFF)),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );

    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(110.0, 110.0));
    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(190.0, 150.0));
    expect(didReceiveTap, isTrue);
  });

  testWidgets('Transform alignment', (WidgetTester tester) async {
    bool didReceiveTap = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              top: 100.0,
              left: 100.0,
              child: Container(width: 100.0, height: 100.0, color: const Color(0xFF0000FF)),
            ),
            Positioned(
              top: 100.0,
              left: 100.0,
              child: SizedBox(
                width: 100.0,
                height: 100.0,
                child: Transform(
                  transform: Matrix4.diagonal3Values(0.5, 0.5, 1.0),
                  alignment: Alignment.centerRight,
                  child: GestureDetector(
                    onTap: () {
                      didReceiveTap = true;
                    },
                    child: Container(color: const Color(0xFF00FFFF)),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );

    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(110.0, 110.0));
    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(190.0, 150.0));
    expect(didReceiveTap, isTrue);
  });

  testWidgets('Transform AlignmentDirectional alignment', (WidgetTester tester) async {
    bool didReceiveTap = false;

    Widget buildFrame(TextDirection textDirection, AlignmentGeometry alignment) {
      return Directionality(
        textDirection: textDirection,
        child: Stack(
          children: <Widget>[
            Positioned(
              top: 100.0,
              left: 100.0,
              child: Container(width: 100.0, height: 100.0, color: const Color(0xFF0000FF)),
            ),
            Positioned(
              top: 100.0,
              left: 100.0,
              child: SizedBox(
                width: 100.0,
                height: 100.0,
                child: Transform(
                  transform: Matrix4.diagonal3Values(0.5, 0.5, 1.0),
                  alignment: alignment,
                  child: GestureDetector(
                    onTap: () {
                      didReceiveTap = true;
                    },
                    child: Container(color: const Color(0xFF00FFFF)),
                  ),
                ),
              ),
            ),
          ],
        ),
      );
    }

    await tester.pumpWidget(buildFrame(TextDirection.ltr, AlignmentDirectional.centerEnd));
    didReceiveTap = false;
    await tester.tapAt(const Offset(110.0, 110.0));
    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(190.0, 150.0));
    expect(didReceiveTap, isTrue);

    await tester.pumpWidget(buildFrame(TextDirection.rtl, AlignmentDirectional.centerStart));
    didReceiveTap = false;
    await tester.tapAt(const Offset(110.0, 110.0));
    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(190.0, 150.0));
    expect(didReceiveTap, isTrue);

    await tester.pumpWidget(buildFrame(TextDirection.ltr, AlignmentDirectional.centerStart));
    didReceiveTap = false;
    await tester.tapAt(const Offset(190.0, 150.0));
    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(110.0, 150.0));
    expect(didReceiveTap, isTrue);

    await tester.pumpWidget(buildFrame(TextDirection.rtl, AlignmentDirectional.centerEnd));
    didReceiveTap = false;
    await tester.tapAt(const Offset(190.0, 150.0));
    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(110.0, 150.0));
    expect(didReceiveTap, isTrue);
  });

  testWidgets('Transform offset + alignment', (WidgetTester tester) async {
    bool didReceiveTap = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              top: 100.0,
              left: 100.0,
              child: Container(width: 100.0, height: 100.0, color: const Color(0xFF0000FF)),
            ),
            Positioned(
              top: 100.0,
              left: 100.0,
              child: SizedBox(
                width: 100.0,
                height: 100.0,
                child: Transform(
                  transform: Matrix4.diagonal3Values(0.5, 0.5, 1.0),
                  origin: const Offset(100.0, 0.0),
                  alignment: Alignment.centerLeft,
                  child: GestureDetector(
                    onTap: () {
                      didReceiveTap = true;
                    },
                    child: Container(color: const Color(0xFF00FFFF)),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );

    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(110.0, 110.0));
    expect(didReceiveTap, isFalse);
    await tester.tapAt(const Offset(190.0, 150.0));
    expect(didReceiveTap, isTrue);
  });

  testWidgets('Composited transform offset', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 400.0,
          height: 300.0,
          child: ClipRect(
            child: Transform(
              transform: Matrix4.diagonal3Values(0.5, 0.5, 1.0),
              child: RepaintBoundary(child: Container(color: const Color(0xFF00FF00))),
            ),
          ),
        ),
      ),
    );

    final List<Layer> layers = tester.layers..retainWhere((Layer layer) => layer is TransformLayer);
    expect(layers.length, 2);
    // The first transform is from the render view.
    final TransformLayer layer = layers[1] as TransformLayer;
    final Matrix4 transform = layer.transform!;
    expect(transform.getTranslation(), equals(Vector3(100.0, 75.0, 0.0)));
  });

  testWidgets('Transform.rotate', (WidgetTester tester) async {
    await tester.pumpWidget(
      Transform.rotate(angle: math.pi / 2.0, child: RepaintBoundary(child: Container())),
    );

    final List<Layer> layers = tester.layers..retainWhere((Layer layer) => layer is TransformLayer);
    expect(layers.length, 2);
    // The first transform is from the render view.
    final TransformLayer layer = layers[1] as TransformLayer;
    final Matrix4 transform = layer.transform!;
    expect(transform.storage, <dynamic>[
      moreOrLessEquals(0.0),
      1.0,
      0.0,
      0.0,
      -1.0,
      moreOrLessEquals(0.0),
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      700.0,
      -100.0,
      0.0,
      1.0,
    ]);
  });

  testWidgets('applyPaintTransform of Transform in Padding', (WidgetTester tester) async {
    await tester.pumpWidget(
      Padding(
        padding: const EdgeInsets.only(left: 30.0, top: 20.0, right: 50.0, bottom: 70.0),
        child: Transform(
          transform: Matrix4.diagonal3Values(2.0, 2.0, 2.0),
          child: const Placeholder(),
        ),
      ),
    );
    expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(30.0, 20.0));
  });

  testWidgets('Transform.translate', (WidgetTester tester) async {
    await tester.pumpWidget(
      Transform.translate(
        offset: const Offset(100.0, 50.0),
        child: RepaintBoundary(child: Container()),
      ),
    );

    // This should not cause a transform layer to be inserted.
    final List<Layer> layers = tester.layers..retainWhere((Layer layer) => layer is TransformLayer);
    expect(layers.length, 1); // only the render view
    expect(tester.getTopLeft(find.byType(Container)), const Offset(100.0, 50.0));
  });

  testWidgets('Transform.scale', (WidgetTester tester) async {
    await tester.pumpWidget(
      Transform.scale(scale: 2.0, child: RepaintBoundary(child: Container())),
    );

    final List<Layer> layers = tester.layers..retainWhere((Layer layer) => layer is TransformLayer);
    expect(layers.length, 2);
    // The first transform is from the render view.
    final TransformLayer layer = layers[1] as TransformLayer;
    final Matrix4 transform = layer.transform!;
    expect(transform.storage, <dynamic>[
      // These are column-major, not row-major.
      2.0, 0.0, 0.0, 0.0,
      0.0, 2.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      -400.0, -300.0, 0.0, 1.0, // it's 1600x1200, centered in an 800x600 square
    ]);
  });

  testWidgets('Transform with nan value short-circuits rendering', (WidgetTester tester) async {
    await tester.pumpWidget(
      Transform(
        transform: Matrix4.identity()..storage[0] = double.nan,
        child: RepaintBoundary(child: Container()),
      ),
    );

    expect(tester.layers, hasLength(1));
  });

  testWidgets('Transform with inf value short-circuits rendering', (WidgetTester tester) async {
    await tester.pumpWidget(
      Transform(
        transform: Matrix4.identity()..storage[0] = double.infinity,
        child: RepaintBoundary(child: Container()),
      ),
    );

    expect(tester.layers, hasLength(1));
  });

  testWidgets('Transform with -inf value short-circuits rendering', (WidgetTester tester) async {
    await tester.pumpWidget(
      Transform(
        transform: Matrix4.identity()..storage[0] = double.negativeInfinity,
        child: RepaintBoundary(child: Container()),
      ),
    );

    expect(tester.layers, hasLength(1));
  });

  testWidgets('Transform.rotate does not remove layers due to singular short-circuit', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Transform.rotate(angle: math.pi / 2, child: RepaintBoundary(child: Container())),
    );

    expect(tester.layers, hasLength(3));
  });

  testWidgets('Transform.rotate creates nice rotation matrices for 0, 90, 180, 270 degrees', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Transform.rotate(angle: math.pi / 2, child: RepaintBoundary(child: Container())),
    );

    expect(
      tester.layers[1],
      isA<TransformLayer>().having(
        (TransformLayer layer) => layer.transform,
        'transform',
        equals(
          Matrix4.fromList(<double>[
            0.0,
            -1.0,
            0.0,
            700.0,
            1.0,
            0.0,
            0.0,
            -100.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
          ])..transpose(),
        ),
      ),
    );

    await tester.pumpWidget(
      Transform.rotate(angle: math.pi, child: RepaintBoundary(child: Container())),
    );

    expect(
      tester.layers[1],
      isA<TransformLayer>().having(
        (TransformLayer layer) => layer.transform,
        'transform',
        equals(
          Matrix4.fromList(<double>[
            -1.0,
            0.0,
            0.0,
            800.0,
            0.0,
            -1.0,
            0.0,
            600.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
          ])..transpose(),
        ),
      ),
    );

    await tester.pumpWidget(
      Transform.rotate(angle: 3 * math.pi / 2, child: RepaintBoundary(child: Container())),
    );

    expect(
      tester.layers[1],
      isA<TransformLayer>().having(
        (TransformLayer layer) => layer.transform,
        'transform',
        equals(
          Matrix4.fromList(<double>[
            0.0,
            1.0,
            0.0,
            100.0,
            -1.0,
            0.0,
            0.0,
            700.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
          ])..transpose(),
        ),
      ),
    );

    await tester.pumpWidget(Transform.rotate(angle: 0, child: RepaintBoundary(child: Container())));

    // No transform layer created
    expect(tester.layers[1], isA<OffsetLayer>());
    expect(tester.layers, hasLength(2));
  });

  testWidgets('Transform.scale with 0.0 does not paint child layers', (WidgetTester tester) async {
    await tester.pumpWidget(
      Transform.scale(scale: 0.0, child: RepaintBoundary(child: Container())),
    );

    expect(tester.layers, hasLength(1)); // root transform layer

    await tester.pumpWidget(
      Transform.scale(scaleX: 0.0, child: RepaintBoundary(child: Container())),
    );

    expect(tester.layers, hasLength(1));

    await tester.pumpWidget(
      Transform.scale(scaleY: 0.0, child: RepaintBoundary(child: Container())),
    );

    expect(tester.layers, hasLength(1));

    await tester.pumpWidget(
      Transform.scale(
        scale: 0.01, // small but non-zero
        child: RepaintBoundary(child: Container()),
      ),
    );

    expect(tester.layers, hasLength(3));
  });

  testWidgets('Translated child into translated box - hit test', (WidgetTester tester) async {
    final GlobalKey key1 = GlobalKey();
    bool pointerDown = false;
    await tester.pumpWidget(
      Transform.translate(
        offset: const Offset(100.0, 50.0),
        child: Transform.translate(
          offset: const Offset(1000.0, 1000.0),
          child: Listener(
            onPointerDown: (PointerDownEvent event) {
              pointerDown = true;
            },
            child: Container(key: key1, color: const Color(0xFF000000)),
          ),
        ),
      ),
    );
    expect(pointerDown, isFalse);
    await tester.tap(find.byKey(key1));
    expect(pointerDown, isTrue);
  });

  Widget generateTransform(bool needsCompositing, double angle) {
    final Widget customPaint = CustomPaint(painter: TestRectPainter());
    return Transform(
      transform: MatrixUtils.createCylindricalProjectionTransform(
        radius: 100,
        angle: angle,
        perspective: 0.003,
      ),
      // A RepaintBoundary child forces the Transform to needsCompositing
      child: needsCompositing ? RepaintBoundary(child: customPaint) : customPaint,
    );
  }

  testWidgets(
    '3D transform renders the same with or without needsCompositing',
    (WidgetTester tester) async {
      for (double angle = 0; angle <= math.pi / 4; angle += 0.01) {
        await tester.pumpWidget(RepaintBoundary(child: generateTransform(true, angle)));
        final RenderBox renderBox = tester.binding.renderView.child!;
        final OffsetLayer layer = renderBox.debugLayer! as OffsetLayer;
        final ui.Image imageWithCompositing = await layer.toImage(renderBox.paintBounds);
        addTearDown(imageWithCompositing.dispose);

        await tester.pumpWidget(RepaintBoundary(child: generateTransform(false, angle)));
        await expectLater(
          find.byType(RepaintBoundary).first,
          matchesReferenceImage(imageWithCompositing),
        );
      }
    },
    skip: isBrowser, // due to https://github.com/flutter/flutter/issues/49857
  );

  List<double> extractMatrix(ui.ImageFilter? filter) {
    final List<String> numbers = filter.toString().split('[').last.split(']').first.split(',');
    return numbers.map<double>((String str) => double.parse(str.trim())).toList();
  }

  testWidgets('Transform.translate with FilterQuality produces filter layer', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Transform.translate(
        offset: const Offset(25.0, 25.0),
        filterQuality: FilterQuality.low,
        child: const SizedBox(width: 100, height: 100),
      ),
    );
    expect(tester.layers.whereType<ImageFilterLayer>().length, 1);
    final ImageFilterLayer layer = tester.layers.whereType<ImageFilterLayer>().first;
    expect(extractMatrix(layer.imageFilter), <double>[
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
      25.0,
      25.0,
      0.0,
      1.0,
    ]);
  });

  testWidgets('Transform.scale with FilterQuality produces filter layer', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Transform.scale(
        scale: 3.14159,
        filterQuality: FilterQuality.low,
        child: const SizedBox(width: 100, height: 100),
      ),
    );
    expect(tester.layers.whereType<ImageFilterLayer>().length, 1);
    final ImageFilterLayer layer = tester.layers.whereType<ImageFilterLayer>().first;
    expect(extractMatrix(layer.imageFilter), <double>[
      3.14159,
      0.0,
      0.0,
      0.0,
      0.0,
      3.14159,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      -856.636,
      -642.477,
      0.0,
      1.0,
    ]);
  });

  testWidgets('Transform.rotate with FilterQuality produces filter layer', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Transform.rotate(
        angle: math.pi / 4,
        filterQuality: FilterQuality.low,
        child: const SizedBox(width: 100, height: 100),
      ),
    );
    expect(tester.layers.whereType<ImageFilterLayer>().length, 1);
    final ImageFilterLayer layer = tester.layers.whereType<ImageFilterLayer>().first;
    expect(extractMatrix(layer.imageFilter), <dynamic>[
      moreOrLessEquals(0.7071067811865476),
      moreOrLessEquals(0.7071067811865475),
      0.0,
      0.0,
      moreOrLessEquals(-0.7071067811865475),
      moreOrLessEquals(0.7071067811865476),
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      moreOrLessEquals(329.28932188134524),
      moreOrLessEquals(-194.97474683058329),
      0.0,
      1.0,
    ]);
  });

  testWidgets('Offset Transform.rotate with FilterQuality produces filter layer', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      SizedBox(
        width: 400,
        height: 400,
        child: Center(
          child: Transform.rotate(
            angle: math.pi / 4,
            filterQuality: FilterQuality.low,
            child: const SizedBox(width: 100, height: 100),
          ),
        ),
      ),
    );
    expect(tester.layers.whereType<ImageFilterLayer>().length, 1);
    final ImageFilterLayer layer = tester.layers.whereType<ImageFilterLayer>().first;
    expect(extractMatrix(layer.imageFilter), <dynamic>[
      moreOrLessEquals(0.7071067811865476),
      moreOrLessEquals(0.7071067811865475),
      0.0,
      0.0,
      moreOrLessEquals(-0.7071067811865475),
      moreOrLessEquals(0.7071067811865476),
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0,
      moreOrLessEquals(329.28932188134524),
      moreOrLessEquals(-194.97474683058329),
      0.0,
      1.0,
    ]);
  });

  testWidgets('Transform layers update to match child and filterQuality', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Transform.rotate(
        angle: math.pi / 4,
        filterQuality: FilterQuality.low,
        child: const SizedBox(width: 100, height: 100),
      ),
    );
    expect(tester.layers.whereType<ImageFilterLayer>(), hasLength(1));

    await tester.pumpWidget(
      Transform.rotate(angle: math.pi / 4, child: const SizedBox(width: 100, height: 100)),
    );
    expect(tester.layers.whereType<ImageFilterLayer>(), isEmpty);

    await tester.pumpWidget(Transform.rotate(angle: math.pi / 4, filterQuality: FilterQuality.low));
    expect(tester.layers.whereType<ImageFilterLayer>(), isEmpty);

    await tester.pumpWidget(
      Transform.rotate(
        angle: math.pi / 4,
        filterQuality: FilterQuality.low,
        child: const SizedBox(width: 100, height: 100),
      ),
    );
    expect(tester.layers.whereType<ImageFilterLayer>(), hasLength(1));
  });

  testWidgets('Transform layers with filterQuality golden', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(
          crossAxisCount: 3,
          children: <Widget>[
            Transform.rotate(
              angle: math.pi / 6,
              child: Center(
                child: Container(width: 100, height: 20, color: const Color(0xffffff00)),
              ),
            ),
            Transform.scale(
              scale: 1.5,
              child: Center(
                child: Container(width: 100, height: 20, color: const Color(0xffffff00)),
              ),
            ),
            Transform.translate(
              offset: const Offset(20.0, 60.0),
              child: Center(
                child: Container(width: 100, height: 20, color: const Color(0xffffff00)),
              ),
            ),
            Transform.rotate(
              angle: math.pi / 6,
              filterQuality: FilterQuality.low,
              child: Center(
                child: Container(width: 100, height: 20, color: const Color(0xff00ff00)),
              ),
            ),
            Transform.scale(
              scale: 1.5,
              filterQuality: FilterQuality.low,
              child: Center(
                child: Container(width: 100, height: 20, color: const Color(0xff00ff00)),
              ),
            ),
            Transform.translate(
              offset: const Offset(20.0, 60.0),
              filterQuality: FilterQuality.low,
              child: Center(
                child: Container(width: 100, height: 20, color: const Color(0xff00ff00)),
              ),
            ),
          ],
        ),
      ),
    );
    await expectLater(
      find.byType(GridView),
      matchesGoldenFile('transform_golden.BitmapRotate.png'),
    );
  });

  testWidgets(
    "Transform.scale() does not accept all three 'scale', 'scaleX' and 'scaleY' parameters to be non-null",
    (WidgetTester tester) async {
      await expectLater(() {
        tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: Transform.scale(
                scale: 1.0,
                scaleX: 1.0,
                scaleY: 1.0,
                child: const SizedBox(height: 100, width: 100),
              ),
            ),
          ),
        );
      }, throwsAssertionError);
    },
  );

  testWidgets(
    "Transform.scale() needs at least one of 'scale', 'scaleX' and 'scaleY' to be non-null, otherwise throws AssertionError",
    (WidgetTester tester) async {
      await expectLater(() {
        tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Center(child: Transform.scale(child: const SizedBox(height: 100, width: 100))),
          ),
        );
      }, throwsAssertionError);
    },
  );

  testWidgets("Transform.scale() scales widget uniformly with 'scale' parameter", (
    WidgetTester tester,
  ) async {
    const double scale = 1.5;
    const double height = 100;
    const double width = 150;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SizedBox(
          height: 400,
          width: 400,
          child: Center(
            child: Transform.scale(
              scale: scale,
              child: Container(height: height, width: width, decoration: const BoxDecoration()),
            ),
          ),
        ),
      ),
    );

    const Size target = Size(width * scale, height * scale);

    expect(
      tester.getBottomRight(find.byType(Container)),
      target.bottomRight(tester.getTopLeft(find.byType(Container))),
    );
  });

  testWidgets("Transform.scale() scales widget according to 'scaleX' and 'scaleY'", (
    WidgetTester tester,
  ) async {
    const double scaleX = 1.5;
    const double scaleY = 1.2;
    const double height = 100;
    const double width = 150;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SizedBox(
          height: 400,
          width: 400,
          child: Center(
            child: Transform.scale(
              scaleX: scaleX,
              scaleY: scaleY,
              child: Container(height: height, width: width, decoration: const BoxDecoration()),
            ),
          ),
        ),
      ),
    );

    const Size target = Size(width * scaleX, height * scaleY);

    expect(
      tester.getBottomRight(find.byType(Container)),
      target.bottomRight(tester.getTopLeft(find.byType(Container))),
    );
  });

  testWidgets('Transform.flip does flip child correctly', (WidgetTester tester) async {
    const Offset topRight = Offset(60, 20);
    const Offset bottomLeft = Offset(20, 60);
    const Offset bottomRight = Offset(60, 60);

    bool tappedRed = false;

    const Widget square = SizedBox.square(dimension: 40);
    final Widget child = Column(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        Row(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            GestureDetector(
              onTap: () => tappedRed = true,
              child: const ColoredBox(color: Color(0xffff0000), child: square),
            ),
            const ColoredBox(color: Color(0xff00ff00), child: square),
          ],
        ),
        const Row(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            ColoredBox(color: Color(0xff0000ff), child: square),
            ColoredBox(color: Color(0xffeeff00), child: square),
          ],
        ),
      ],
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: Transform.flip(flipX: true, child: child),
        ),
      ),
    );

    await tester.pumpAndSettle();

    await tester.tapAt(topRight);

    expect(tappedRed, isTrue, reason: 'Transform.flip cannot flipX');

    tappedRed = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: Transform.flip(flipY: true, child: child),
        ),
      ),
    );

    await tester.pumpAndSettle();

    await tester.tapAt(bottomLeft);

    expect(tappedRed, isTrue, reason: 'Transform.flip cannot flipY');

    tappedRed = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: Transform.flip(flipX: true, flipY: true, child: child),
        ),
      ),
    );

    await tester.pumpAndSettle();

    await tester.tapAt(bottomRight);

    expect(tappedRed, isTrue, reason: 'Transform.flip cannot flipX and flipY together');
  });
}

class TestRectPainter extends CustomPainter {
  @override
  void paint(ui.Canvas canvas, ui.Size size) {
    canvas.drawRect(
      const Offset(200, 200) & const Size(10, 10),
      Paint()..color = const Color(0xFFFF0000),
    );
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}
