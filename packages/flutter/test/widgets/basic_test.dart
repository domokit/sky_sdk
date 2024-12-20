// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:math' as math;
import 'dart:ui' as ui;
import 'dart:ui';

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  group('RawImage', () {
    testWidgets('properties', (WidgetTester tester) async {
      final ui.Image image1 = (await tester.runAsync<ui.Image>(() => createTestImage()))!;
      addTearDown(image1.dispose);

      await tester.pumpWidget(
        Directionality(textDirection: TextDirection.ltr, child: RawImage(image: image1)),
      );
      final RenderImage renderObject = tester.firstRenderObject<RenderImage>(find.byType(RawImage));

      // Expect default values
      expect(renderObject.image!.isCloneOf(image1), true);
      expect(renderObject.debugImageLabel, null);
      expect(renderObject.width, null);
      expect(renderObject.height, null);
      expect(renderObject.scale, 1.0);
      expect(renderObject.color, null);
      expect(renderObject.opacity, null);
      expect(renderObject.colorBlendMode, null);
      expect(renderObject.fit, null);
      expect(renderObject.alignment, Alignment.center);
      expect(renderObject.repeat, ImageRepeat.noRepeat);
      expect(renderObject.centerSlice, null);
      expect(renderObject.matchTextDirection, false);
      expect(renderObject.invertColors, false);
      expect(renderObject.filterQuality, FilterQuality.medium);
      expect(renderObject.isAntiAlias, false);

      final ui.Image image2 =
          (await tester.runAsync<ui.Image>(() => createTestImage(width: 2, height: 2)))!;
      addTearDown(image2.dispose);
      const String debugImageLabel = 'debugImageLabel';
      const double width = 1;
      const double height = 1;
      const double scale = 2.0;
      const Color color = Colors.black;
      const Animation<double> opacity = AlwaysStoppedAnimation<double>(0.0);
      const BlendMode colorBlendMode = BlendMode.difference;
      const BoxFit fit = BoxFit.contain;
      const AlignmentGeometry alignment = Alignment.topCenter;
      const ImageRepeat repeat = ImageRepeat.repeat;
      const Rect centerSlice = Rect.fromLTWH(0, 0, width, height);
      const bool matchTextDirection = true;
      const bool invertColors = true;
      const FilterQuality filterQuality = FilterQuality.high;
      const bool isAntiAlias = true;

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: RawImage(
            image: image2,
            debugImageLabel: debugImageLabel,
            width: width,
            height: height,
            scale: scale,
            color: color,
            opacity: opacity,
            colorBlendMode: colorBlendMode,
            fit: fit,
            alignment: alignment,
            repeat: repeat,
            centerSlice: centerSlice,
            matchTextDirection: matchTextDirection,
            invertColors: invertColors,
            filterQuality: filterQuality,
            isAntiAlias: isAntiAlias,
          ),
        ),
      );

      expect(renderObject.image!.isCloneOf(image2), true);
      expect(renderObject.debugImageLabel, debugImageLabel);
      expect(renderObject.width, width);
      expect(renderObject.height, height);
      expect(renderObject.scale, scale);
      expect(renderObject.color, color);
      expect(renderObject.opacity, opacity);
      expect(renderObject.colorBlendMode, colorBlendMode);
      expect(renderObject.fit, fit);
      expect(renderObject.alignment, alignment);
      expect(renderObject.repeat, repeat);
      expect(renderObject.centerSlice, centerSlice);
      expect(renderObject.matchTextDirection, matchTextDirection);
      expect(renderObject.invertColors, invertColors);
      expect(renderObject.filterQuality, filterQuality);
      expect(renderObject.isAntiAlias, isAntiAlias);
    });
  });

  group('PhysicalShape', () {
    testWidgets('properties', (WidgetTester tester) async {
      await tester.pumpWidget(
        const PhysicalShape(
          clipper: ShapeBorderClipper(shape: CircleBorder()),
          elevation: 2.0,
          color: Color(0xFF0000FF),
          shadowColor: Color(0xFF00FF00),
        ),
      );
      final RenderPhysicalShape renderObject = tester.renderObject(find.byType(PhysicalShape));
      expect(renderObject.clipper, const ShapeBorderClipper(shape: CircleBorder()));
      expect(renderObject.color, const Color(0xFF0000FF));
      expect(renderObject.shadowColor, const Color(0xFF00FF00));
      expect(renderObject.elevation, 2.0);
    });

    testWidgets('hit test', (WidgetTester tester) async {
      await tester.pumpWidget(
        PhysicalShape(
          clipper: const ShapeBorderClipper(shape: CircleBorder()),
          elevation: 2.0,
          color: const Color(0xFF0000FF),
          shadowColor: const Color(0xFF00FF00),
          child: Container(color: const Color(0xFF0000FF)),
        ),
      );

      final RenderPhysicalShape renderPhysicalShape = tester.renderObject(
        find.byType(PhysicalShape),
      );

      // The viewport is 800x600, the CircleBorder is centered and fits
      // the shortest edge, so we get a circle of radius 300, centered at
      // (400, 300).
      //
      // We test by sampling a few points around the left-most point of the
      // circle (100, 300).

      expect(tester.hitTestOnBinding(const Offset(99.0, 300.0)), doesNotHit(renderPhysicalShape));
      expect(tester.hitTestOnBinding(const Offset(100.0, 300.0)), hits(renderPhysicalShape));
      expect(tester.hitTestOnBinding(const Offset(100.0, 299.0)), doesNotHit(renderPhysicalShape));
      expect(tester.hitTestOnBinding(const Offset(100.0, 301.0)), doesNotHit(renderPhysicalShape));
    });
  });

  group('FractionalTranslation', () {
    testWidgets('hit test - entirely inside the bounding box', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey();
      bool pointerDown = false;

      await tester.pumpWidget(
        Center(
          child: FractionalTranslation(
            translation: Offset.zero,
            child: Listener(
              onPointerDown: (PointerDownEvent event) {
                pointerDown = true;
              },
              child: SizedBox(
                key: key1,
                width: 100.0,
                height: 100.0,
                child: Container(color: const Color(0xFF0000FF)),
              ),
            ),
          ),
        ),
      );
      expect(pointerDown, isFalse);
      await tester.tap(find.byKey(key1));
      expect(pointerDown, isTrue);
    });

    testWidgets('hit test - partially inside the bounding box', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey();
      bool pointerDown = false;

      await tester.pumpWidget(
        Center(
          child: FractionalTranslation(
            translation: const Offset(0.5, 0.5),
            child: Listener(
              onPointerDown: (PointerDownEvent event) {
                pointerDown = true;
              },
              child: SizedBox(
                key: key1,
                width: 100.0,
                height: 100.0,
                child: Container(color: const Color(0xFF0000FF)),
              ),
            ),
          ),
        ),
      );
      expect(pointerDown, isFalse);
      await tester.tap(find.byKey(key1));
      expect(pointerDown, isTrue);
    });

    testWidgets('hit test - completely outside the bounding box', (WidgetTester tester) async {
      final GlobalKey key1 = GlobalKey();
      bool pointerDown = false;

      await tester.pumpWidget(
        Center(
          child: FractionalTranslation(
            translation: const Offset(1.0, 1.0),
            child: Listener(
              onPointerDown: (PointerDownEvent event) {
                pointerDown = true;
              },
              child: SizedBox(
                key: key1,
                width: 100.0,
                height: 100.0,
                child: Container(color: const Color(0xFF0000FF)),
              ),
            ),
          ),
        ),
      );
      expect(pointerDown, isFalse);
      await tester.tap(find.byKey(key1));
      expect(pointerDown, isTrue);
    });

    testWidgets('semantics bounds are updated', (WidgetTester tester) async {
      final GlobalKey fractionalTranslationKey = GlobalKey();
      final GlobalKey textKey = GlobalKey();
      Offset offset = const Offset(0.4, 0.4);

      await tester.pumpWidget(
        StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Directionality(
              textDirection: TextDirection.ltr,
              child: Center(
                child: Semantics(
                  explicitChildNodes: true,
                  child: FractionalTranslation(
                    key: fractionalTranslationKey,
                    translation: offset,
                    child: GestureDetector(
                      onTap: () {
                        setState(() {
                          offset = const Offset(0.8, 0.8);
                        });
                      },
                      child: SizedBox(
                        width: 100.0,
                        height: 100.0,
                        child: Text('foo', key: textKey),
                      ),
                    ),
                  ),
                ),
              ),
            );
          },
        ),
      );

      expect(
        tester.getSemantics(find.byKey(textKey)).transform,
        Matrix4(
          3.0,
          0.0,
          0.0,
          0.0,
          0.0,
          3.0,
          0.0,
          0.0,
          0.0,
          0.0,
          1.0,
          0.0,
          1170.0,
          870.0,
          0.0,
          1.0,
        ),
      );

      await tester.tap(
        find.byKey(fractionalTranslationKey),
        warnIfMissed: false,
      ); // RenderFractionalTranslation can't be hit
      await tester.pump();
      expect(
        tester.getSemantics(find.byKey(textKey)).transform,
        Matrix4(
          3.0,
          0.0,
          0.0,
          0.0,
          0.0,
          3.0,
          0.0,
          0.0,
          0.0,
          0.0,
          1.0,
          0.0,
          1290.0,
          990.0,
          0.0,
          1.0,
        ),
      );
    });
  });

  group('Semantics', () {
    testWidgets('Semantics can set attributed Text', (WidgetTester tester) async {
      final UniqueKey key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Semantics(
              key: key,
              attributedLabel: AttributedString(
                'label',
                attributes: <StringAttribute>[
                  SpellOutStringAttribute(range: const TextRange(start: 0, end: 5)),
                ],
              ),
              attributedValue: AttributedString(
                'value',
                attributes: <StringAttribute>[
                  LocaleStringAttribute(
                    range: const TextRange(start: 0, end: 5),
                    locale: const Locale('en', 'MX'),
                  ),
                ],
              ),
              attributedHint: AttributedString(
                'hint',
                attributes: <StringAttribute>[
                  SpellOutStringAttribute(range: const TextRange(start: 1, end: 2)),
                ],
              ),
              child: const Placeholder(),
            ),
          ),
        ),
      );
      final AttributedString attributedLabel = tester.getSemantics(find.byKey(key)).attributedLabel;
      expect(attributedLabel.string, 'label');
      expect(attributedLabel.attributes.length, 1);
      expect(attributedLabel.attributes[0] is SpellOutStringAttribute, isTrue);
      expect(attributedLabel.attributes[0].range, const TextRange(start: 0, end: 5));

      final AttributedString attributedValue = tester.getSemantics(find.byKey(key)).attributedValue;
      expect(attributedValue.string, 'value');
      expect(attributedValue.attributes.length, 1);
      expect(attributedValue.attributes[0] is LocaleStringAttribute, isTrue);
      final LocaleStringAttribute valueLocale =
          attributedValue.attributes[0] as LocaleStringAttribute;
      expect(valueLocale.range, const TextRange(start: 0, end: 5));
      expect(valueLocale.locale, const Locale('en', 'MX'));

      final AttributedString attributedHint = tester.getSemantics(find.byKey(key)).attributedHint;
      expect(attributedHint.string, 'hint');
      expect(attributedHint.attributes.length, 1);
      expect(attributedHint.attributes[0] is SpellOutStringAttribute, isTrue);
      expect(attributedHint.attributes[0].range, const TextRange(start: 1, end: 2));
    });

    testWidgets('Semantics can merge attributed strings', (WidgetTester tester) async {
      final UniqueKey key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Semantics(
              key: key,
              attributedLabel: AttributedString(
                'label',
                attributes: <StringAttribute>[
                  SpellOutStringAttribute(range: const TextRange(start: 0, end: 5)),
                ],
              ),
              attributedHint: AttributedString(
                'hint',
                attributes: <StringAttribute>[
                  SpellOutStringAttribute(range: const TextRange(start: 1, end: 2)),
                ],
              ),
              child: Semantics(
                attributedLabel: AttributedString(
                  'label',
                  attributes: <StringAttribute>[
                    SpellOutStringAttribute(range: const TextRange(start: 0, end: 5)),
                  ],
                ),
                attributedHint: AttributedString(
                  'hint',
                  attributes: <StringAttribute>[
                    SpellOutStringAttribute(range: const TextRange(start: 1, end: 2)),
                  ],
                ),
                child: const Placeholder(),
              ),
            ),
          ),
        ),
      );
      final AttributedString attributedLabel = tester.getSemantics(find.byKey(key)).attributedLabel;
      expect(attributedLabel.string, 'label\nlabel');
      expect(attributedLabel.attributes.length, 2);
      expect(attributedLabel.attributes[0] is SpellOutStringAttribute, isTrue);
      expect(attributedLabel.attributes[0].range, const TextRange(start: 0, end: 5));
      expect(attributedLabel.attributes[1] is SpellOutStringAttribute, isTrue);
      expect(attributedLabel.attributes[1].range, const TextRange(start: 6, end: 11));

      final AttributedString attributedHint = tester.getSemantics(find.byKey(key)).attributedHint;
      expect(attributedHint.string, 'hint\nhint');
      expect(attributedHint.attributes.length, 2);
      expect(attributedHint.attributes[0] is SpellOutStringAttribute, isTrue);
      expect(attributedHint.attributes[0].range, const TextRange(start: 1, end: 2));
      expect(attributedHint.attributes[1] is SpellOutStringAttribute, isTrue);
      expect(attributedHint.attributes[1].range, const TextRange(start: 6, end: 7));
    });

    testWidgets('Semantics can merge attributed strings with non attributed string', (
      WidgetTester tester,
    ) async {
      final UniqueKey key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Semantics(
              key: key,
              attributedLabel: AttributedString(
                'label1',
                attributes: <StringAttribute>[
                  SpellOutStringAttribute(range: const TextRange(start: 0, end: 5)),
                ],
              ),
              child: Semantics(
                label: 'label2',
                child: Semantics(
                  attributedLabel: AttributedString(
                    'label3',
                    attributes: <StringAttribute>[
                      SpellOutStringAttribute(range: const TextRange(start: 1, end: 3)),
                    ],
                  ),
                  child: const Placeholder(),
                ),
              ),
            ),
          ),
        ),
      );
      final AttributedString attributedLabel = tester.getSemantics(find.byKey(key)).attributedLabel;
      expect(attributedLabel.string, 'label1\nlabel2\nlabel3');
      expect(attributedLabel.attributes.length, 2);
      expect(attributedLabel.attributes[0] is SpellOutStringAttribute, isTrue);
      expect(attributedLabel.attributes[0].range, const TextRange(start: 0, end: 5));
      expect(attributedLabel.attributes[1] is SpellOutStringAttribute, isTrue);
      expect(attributedLabel.attributes[1].range, const TextRange(start: 15, end: 17));
    });
  });

  group('Row', () {
    testWidgets('multiple baseline aligned children', (WidgetTester tester) async {
      final UniqueKey key1 = UniqueKey();
      final UniqueKey key2 = UniqueKey();
      // The point size of the font must be a multiple of 4 until
      // https://github.com/flutter/flutter/issues/122066 is resolved.
      const double fontSize1 = 52;
      const double fontSize2 = 12;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Row(
              crossAxisAlignment: CrossAxisAlignment.baseline,
              textBaseline: TextBaseline.alphabetic,
              children: <Widget>[
                Text(
                  'big text',
                  key: key1,
                  style: const TextStyle(
                    fontFamily: 'FlutterTest',
                    fontSize: fontSize1,
                    height: 1.0,
                  ),
                ),
                Text(
                  'one\ntwo\nthree\nfour\nfive\nsix\nseven',
                  key: key2,
                  style: const TextStyle(
                    fontFamily: 'FlutterTest',
                    fontSize: fontSize2,
                    height: 1.0,
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      final RenderBox textBox1 = tester.renderObject(find.byKey(key1));
      final RenderBox textBox2 = tester.renderObject(find.byKey(key2));
      final RenderBox rowBox = tester.renderObject(find.byType(Row));

      // The two Texts are baseline aligned, so some portion of them extends
      // both above and below the baseline. The first has a huge font size, so
      // it extends higher above the baseline than usual. The second has many
      // lines, but being aligned by the first line's baseline, they hang far
      // below the baseline. The size of the parent row is just enough to
      // contain both of them.
      const double ascentRatio = 0.75;
      const double aboveBaseline1 = fontSize1 * ascentRatio;
      const double belowBaseline1 = fontSize1 * (1 - ascentRatio);
      const double aboveBaseline2 = fontSize2 * ascentRatio;
      const double belowBaseline2 = fontSize2 * (1 - ascentRatio) + fontSize2 * 6;
      final double aboveBaseline = math.max(aboveBaseline1, aboveBaseline2);
      final double belowBaseline = math.max(belowBaseline1, belowBaseline2);
      expect(rowBox.size.height, greaterThan(textBox1.size.height));
      expect(rowBox.size.height, greaterThan(textBox2.size.height));
      expect(rowBox.size.height, aboveBaseline + belowBaseline);
      expect(tester.getTopLeft(find.byKey(key1)).dy, 0);
      expect(tester.getTopLeft(find.byKey(key2)).dy, aboveBaseline1 - aboveBaseline2);
    });

    testWidgets('baseline aligned children account for a larger, no-baseline child size', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/58898
      final UniqueKey key1 = UniqueKey();
      final UniqueKey key2 = UniqueKey();
      // The point size of the font must be a multiple of 4 until
      // https://github.com/flutter/flutter/issues/122066 is resolved.
      const double fontSize1 = 52;
      const double fontSize2 = 12;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Row(
              crossAxisAlignment: CrossAxisAlignment.baseline,
              textBaseline: TextBaseline.alphabetic,
              children: <Widget>[
                Text(
                  'big text',
                  key: key1,
                  style: const TextStyle(
                    fontFamily: 'FlutterTest',
                    fontSize: fontSize1,
                    height: 1.0,
                  ),
                ),
                Text(
                  'one\ntwo\nthree\nfour\nfive\nsix\nseven',
                  key: key2,
                  style: const TextStyle(
                    fontFamily: 'FlutterTest',
                    fontSize: fontSize2,
                    height: 1.0,
                  ),
                ),
                const FlutterLogo(size: 250),
              ],
            ),
          ),
        ),
      );

      final RenderBox textBox1 = tester.renderObject(find.byKey(key1));
      final RenderBox textBox2 = tester.renderObject(find.byKey(key2));
      final RenderBox rowBox = tester.renderObject(find.byType(Row));

      // The two Texts are baseline aligned, so some portion of them extends
      // both above and below the baseline. The first has a huge font size, so
      // it extends higher above the baseline than usual. The second has many
      // lines, but being aligned by the first line's baseline, they hang far
      // below the baseline. The FlutterLogo extends further than both Texts,
      // so the size of the parent row should contain the FlutterLogo as well.
      const double ascentRatio = 0.75;
      const double aboveBaseline1 = fontSize1 * ascentRatio;
      const double aboveBaseline2 = fontSize2 * ascentRatio;
      expect(rowBox.size.height, greaterThan(textBox1.size.height));
      expect(rowBox.size.height, greaterThan(textBox2.size.height));
      expect(rowBox.size.height, 250);
      expect(tester.getTopLeft(find.byKey(key1)).dy, 0);
      expect(tester.getTopLeft(find.byKey(key2)).dy, aboveBaseline1 - aboveBaseline2);
    });
  });

  test('UnconstrainedBox toString', () {
    expect(
      const UnconstrainedBox(constrainedAxis: Axis.vertical).toString(),
      equals('UnconstrainedBox(alignment: Alignment.center, constrainedAxis: vertical)'),
    );

    expect(
      const UnconstrainedBox(
        constrainedAxis: Axis.horizontal,
        textDirection: TextDirection.rtl,
        alignment: Alignment.topRight,
      ).toString(),
      equals(
        'UnconstrainedBox(alignment: Alignment.topRight, constrainedAxis: horizontal, textDirection: rtl)',
      ),
    );
  });

  testWidgets('UnconstrainedBox can set and update clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(const UnconstrainedBox());
    final RenderConstraintsTransformBox renderObject =
        tester.allRenderObjects.whereType<RenderConstraintsTransformBox>().first;
    expect(renderObject.clipBehavior, equals(Clip.none));

    await tester.pumpWidget(const UnconstrainedBox(clipBehavior: Clip.antiAlias));
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('UnconstrainedBox warns only when clipBehavior is Clip.none', (
    WidgetTester tester,
  ) async {
    for (final Clip? clip in <Clip?>[null, ...Clip.values]) {
      // Clear any render objects that were there before so that we can see more
      // than one error. Otherwise, it just throws the first one and skips the
      // rest, since the render objects haven't changed.
      await tester.pumpWidget(const SizedBox());
      await tester.pumpWidget(
        Center(
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxHeight: 200, maxWidth: 200),
            child:
                clip == null
                    ? const UnconstrainedBox(child: SizedBox(width: 400, height: 400))
                    : UnconstrainedBox(
                      clipBehavior: clip,
                      child: const SizedBox(width: 400, height: 400),
                    ),
          ),
        ),
      );

      final RenderConstraintsTransformBox renderObject =
          tester.allRenderObjects.whereType<RenderConstraintsTransformBox>().first;

      // Defaults to Clip.none
      expect(renderObject.clipBehavior, equals(clip ?? Clip.none), reason: 'for clip = $clip');

      switch (clip) {
        case null:
        case Clip.none:
          // the UnconstrainedBox overflows.
          final dynamic exception = tester.takeException();
          expect(exception, isFlutterError, reason: 'for clip = $clip');
          expect(
            exception.diagnostics.first.level, // ignore: avoid_dynamic_calls
            DiagnosticLevel.summary,
            reason: 'for clip = $clip',
          );
          expect(
            exception.diagnostics.first.toString(), // ignore: avoid_dynamic_calls
            startsWith('A RenderConstraintsTransformBox overflowed'),
            reason: 'for clip = $clip',
          );
        case Clip.hardEdge:
        case Clip.antiAlias:
        case Clip.antiAliasWithSaveLayer:
          expect(tester.takeException(), isNull, reason: 'for clip = $clip');
      }
    }
  });

  group('ConstraintsTransformBox', () {
    test('toString', () {
      expect(
        const ConstraintsTransformBox(
          constraintsTransform: ConstraintsTransformBox.unconstrained,
        ).toString(),
        equals(
          'ConstraintsTransformBox(alignment: Alignment.center, constraints transform: unconstrained)',
        ),
      );
      expect(
        const ConstraintsTransformBox(
          textDirection: TextDirection.rtl,
          alignment: Alignment.topRight,
          constraintsTransform: ConstraintsTransformBox.widthUnconstrained,
        ).toString(),
        equals(
          'ConstraintsTransformBox(alignment: Alignment.topRight, textDirection: rtl, constraints transform: width constraints removed)',
        ),
      );
    });
  });

  group('ColoredBox', () {
    late _MockCanvas mockCanvas;
    late _MockPaintingContext mockContext;
    const Color colorToPaint = Color(0xFFABCDEF);

    setUp(() {
      mockContext = _MockPaintingContext();
      mockCanvas = mockContext.canvas;
    });

    testWidgets('ColoredBox - no size, no child', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Flex(
          direction: Axis.horizontal,
          textDirection: TextDirection.ltr,
          children: <Widget>[SizedBox.shrink(child: ColoredBox(color: colorToPaint))],
        ),
      );
      expect(find.byType(ColoredBox), findsOneWidget);
      final RenderObject renderColoredBox = tester.renderObject(find.byType(ColoredBox));

      renderColoredBox.paint(mockContext, Offset.zero);

      expect(mockCanvas.rects, isEmpty);
      expect(mockCanvas.paints, isEmpty);
      expect(mockContext.children, isEmpty);
      expect(mockContext.offsets, isEmpty);
    });

    testWidgets('ColoredBox - no size, child', (WidgetTester tester) async {
      const ValueKey<int> key = ValueKey<int>(0);
      const Widget child = SizedBox.expand(key: key);
      await tester.pumpWidget(
        const Flex(
          direction: Axis.horizontal,
          textDirection: TextDirection.ltr,
          children: <Widget>[SizedBox.shrink(child: ColoredBox(color: colorToPaint, child: child))],
        ),
      );
      expect(find.byType(ColoredBox), findsOneWidget);
      final RenderObject renderColoredBox = tester.renderObject(find.byType(ColoredBox));
      final RenderObject renderSizedBox = tester.renderObject(find.byKey(key));

      renderColoredBox.paint(mockContext, Offset.zero);

      expect(mockCanvas.rects, isEmpty);
      expect(mockCanvas.paints, isEmpty);
      expect(mockContext.children.single, renderSizedBox);
      expect(mockContext.offsets.single, Offset.zero);
    });

    testWidgets('ColoredBox - size, no child', (WidgetTester tester) async {
      await tester.pumpWidget(const ColoredBox(color: colorToPaint));
      expect(find.byType(ColoredBox), findsOneWidget);
      final RenderObject renderColoredBox = tester.renderObject(find.byType(ColoredBox));

      renderColoredBox.paint(mockContext, Offset.zero);

      expect(mockCanvas.rects.single, const Rect.fromLTWH(0, 0, 800, 600));
      expect(mockCanvas.paints.single.color, isSameColorAs(colorToPaint));
      expect(mockContext.children, isEmpty);
      expect(mockContext.offsets, isEmpty);
    });

    testWidgets('ColoredBox - size, child', (WidgetTester tester) async {
      const ValueKey<int> key = ValueKey<int>(0);
      const Widget child = SizedBox.expand(key: key);
      await tester.pumpWidget(const ColoredBox(color: colorToPaint, child: child));
      expect(find.byType(ColoredBox), findsOneWidget);
      final RenderObject renderColoredBox = tester.renderObject(find.byType(ColoredBox));
      final RenderObject renderSizedBox = tester.renderObject(find.byKey(key));

      renderColoredBox.paint(mockContext, Offset.zero);

      expect(mockCanvas.rects.single, const Rect.fromLTWH(0, 0, 800, 600));
      expect(mockCanvas.paints.single.color, isSameColorAs(colorToPaint));
      expect(mockContext.children.single, renderSizedBox);
      expect(mockContext.offsets.single, Offset.zero);
    });

    testWidgets('ColoredBox - debugFillProperties', (WidgetTester tester) async {
      const ColoredBox box = ColoredBox(color: colorToPaint);
      final DiagnosticPropertiesBuilder properties = DiagnosticPropertiesBuilder();
      box.debugFillProperties(properties);

      expect(properties.properties.first.value, colorToPaint);
    });
  });

  testWidgets('Inconsequential golden test', (WidgetTester tester) async {
    // The test validates the Flutter Gold integration. Any changes to the
    // golden file can be approved at any time.
    await tester.pumpWidget(RepaintBoundary(child: Container(color: const Color(0xFF161145))));

    await tester.pumpAndSettle();
    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('inconsequential_golden_file.png'),
    );
  });

  testWidgets('IgnorePointer ignores pointers', (WidgetTester tester) async {
    final List<String> logs = <String>[];
    Widget target({required bool ignoring}) => Align(
      alignment: Alignment.topLeft,
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: SizedBox(
          width: 100,
          height: 100,
          child: Listener(
            onPointerDown: (_) {
              logs.add('down1');
            },
            child: MouseRegion(
              onEnter: (_) {
                logs.add('enter1');
              },
              onExit: (_) {
                logs.add('exit1');
              },
              cursor: SystemMouseCursors.forbidden,
              child: Stack(
                children: <Widget>[
                  Listener(
                    onPointerDown: (_) {
                      logs.add('down2');
                    },
                    child: MouseRegion(
                      cursor: SystemMouseCursors.click,
                      onEnter: (_) {
                        logs.add('enter2');
                      },
                      onExit: (_) {
                        logs.add('exit2');
                      },
                    ),
                  ),
                  IgnorePointer(
                    ignoring: ignoring,
                    child: Listener(
                      onPointerDown: (_) {
                        logs.add('down3');
                      },
                      child: MouseRegion(
                        cursor: SystemMouseCursors.text,
                        onEnter: (_) {
                          logs.add('enter3');
                        },
                        onExit: (_) {
                          logs.add('exit3');
                        },
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      pointer: 1,
      kind: PointerDeviceKind.mouse,
    );
    await gesture.addPointer(location: const Offset(200, 200));

    await tester.pumpWidget(target(ignoring: true));
    expect(logs, isEmpty);

    await gesture.moveTo(const Offset(50, 50));
    expect(logs, <String>['enter1', 'enter2']);
    logs.clear();

    await gesture.down(const Offset(50, 50));
    expect(logs, <String>['down2', 'down1']);
    logs.clear();

    await gesture.up();
    expect(logs, isEmpty);

    await tester.pumpWidget(target(ignoring: false));
    expect(logs, <String>['exit2', 'enter3']);
    logs.clear();

    await gesture.down(const Offset(50, 50));
    expect(logs, <String>['down3', 'down1']);
    logs.clear();

    await gesture.up();
    expect(logs, isEmpty);

    await tester.pumpWidget(target(ignoring: true));
    expect(logs, <String>['exit3', 'enter2']);
    logs.clear();
  });

  group('IgnorePointer semantics', () {
    testWidgets('does not change semantics when not ignoring', (WidgetTester tester) async {
      final UniqueKey key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: IgnorePointer(
            ignoring: false,
            child: ElevatedButton(key: key, onPressed: () {}, child: const Text('button')),
          ),
        ),
      );
      expect(
        tester.getSemantics(find.byKey(key)),
        matchesSemantics(
          label: 'button',
          hasTapAction: true,
          hasFocusAction: true,
          isButton: true,
          isFocusable: true,
          hasEnabledState: true,
          isEnabled: true,
        ),
      );
    });

    testWidgets('can toggle the ignoring.', (WidgetTester tester) async {
      final UniqueKey key1 = UniqueKey();
      final UniqueKey key2 = UniqueKey();
      final UniqueKey key3 = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: TestIgnorePointer(
            child: Semantics(
              key: key1,
              label: '1',
              onTap: () {},
              container: true,
              child: Semantics(
                key: key2,
                label: '2',
                onTap: () {},
                container: true,
                child: Semantics(
                  key: key3,
                  label: '3',
                  onTap: () {},
                  container: true,
                  child: const SizedBox(width: 10, height: 10),
                ),
              ),
            ),
          ),
        ),
      );
      expect(tester.getSemantics(find.byKey(key1)), matchesSemantics(label: '1'));
      expect(tester.getSemantics(find.byKey(key2)), matchesSemantics(label: '2'));
      expect(tester.getSemantics(find.byKey(key3)), matchesSemantics(label: '3'));

      final TestIgnorePointerState state = tester.state<TestIgnorePointerState>(
        find.byType(TestIgnorePointer),
      );
      state.setIgnore(false);
      await tester.pump();
      expect(
        tester.getSemantics(find.byKey(key1)),
        matchesSemantics(label: '1', hasTapAction: true),
      );
      expect(
        tester.getSemantics(find.byKey(key2)),
        matchesSemantics(label: '2', hasTapAction: true),
      );
      expect(
        tester.getSemantics(find.byKey(key3)),
        matchesSemantics(label: '3', hasTapAction: true),
      );

      state.setIgnore(true);
      await tester.pump();
      expect(tester.getSemantics(find.byKey(key1)), matchesSemantics(label: '1'));
      expect(tester.getSemantics(find.byKey(key2)), matchesSemantics(label: '2'));
      expect(tester.getSemantics(find.byKey(key3)), matchesSemantics(label: '3'));

      state.setIgnore(false);
      await tester.pump();
      expect(
        tester.getSemantics(find.byKey(key1)),
        matchesSemantics(label: '1', hasTapAction: true),
      );
      expect(
        tester.getSemantics(find.byKey(key2)),
        matchesSemantics(label: '2', hasTapAction: true),
      );
      expect(
        tester.getSemantics(find.byKey(key3)),
        matchesSemantics(label: '3', hasTapAction: true),
      );
    });

    testWidgets('drops semantics when its ignoringSemantics is true', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final UniqueKey key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: IgnorePointer(
            ignoringSemantics: true,
            child: ElevatedButton(key: key, onPressed: () {}, child: const Text('button')),
          ),
        ),
      );
      expect(semantics, isNot(includesNodeWith(label: 'button')));
      semantics.dispose();
    });

    testWidgets('ignores user interactions', (WidgetTester tester) async {
      final UniqueKey key = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: IgnorePointer(
            child: ElevatedButton(key: key, onPressed: () {}, child: const Text('button')),
          ),
        ),
      );
      expect(
        tester.getSemantics(find.byKey(key)),
        // Tap action is blocked.
        matchesSemantics(
          label: 'button',
          isButton: true,
          isFocusable: true,
          hasEnabledState: true,
          isEnabled: true,
        ),
      );
    });
  });

  testWidgets('AbsorbPointer absorbs pointers', (WidgetTester tester) async {
    final List<String> logs = <String>[];
    Widget target({required bool absorbing}) => Align(
      alignment: Alignment.topLeft,
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: SizedBox(
          width: 100,
          height: 100,
          child: Listener(
            onPointerDown: (_) {
              logs.add('down1');
            },
            child: MouseRegion(
              onEnter: (_) {
                logs.add('enter1');
              },
              onExit: (_) {
                logs.add('exit1');
              },
              cursor: SystemMouseCursors.forbidden,
              child: Stack(
                children: <Widget>[
                  Listener(
                    onPointerDown: (_) {
                      logs.add('down2');
                    },
                    child: MouseRegion(
                      cursor: SystemMouseCursors.click,
                      onEnter: (_) {
                        logs.add('enter2');
                      },
                      onExit: (_) {
                        logs.add('exit2');
                      },
                    ),
                  ),
                  AbsorbPointer(
                    absorbing: absorbing,
                    child: Listener(
                      onPointerDown: (_) {
                        logs.add('down3');
                      },
                      child: MouseRegion(
                        cursor: SystemMouseCursors.text,
                        onEnter: (_) {
                          logs.add('enter3');
                        },
                        onExit: (_) {
                          logs.add('exit3');
                        },
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      pointer: 1,
      kind: PointerDeviceKind.mouse,
    );
    await gesture.addPointer(location: const Offset(200, 200));

    await tester.pumpWidget(target(absorbing: true));
    expect(logs, isEmpty);

    await gesture.moveTo(const Offset(50, 50));
    expect(logs, <String>['enter1']);
    logs.clear();

    await gesture.down(const Offset(50, 50));
    expect(logs, <String>['down1']);
    logs.clear();

    await gesture.up();
    expect(logs, isEmpty);

    await tester.pumpWidget(target(absorbing: false));
    expect(logs, <String>['enter3']);
    logs.clear();

    await gesture.down(const Offset(50, 50));
    expect(logs, <String>['down3', 'down1']);
    logs.clear();

    await gesture.up();
    expect(logs, isEmpty);

    await tester.pumpWidget(target(absorbing: true));
    expect(logs, <String>['exit3']);
    logs.clear();
  });

  testWidgets('Wrap implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const Wrap(
      spacing: 8.0, // gap between adjacent Text widget
      runSpacing: 4.0, // gap between lines
      textDirection: TextDirection.ltr,
      verticalDirection: VerticalDirection.up,
      children: <Widget>[Text('Hamilton'), Text('Lafayette'), Text('Mulligan')],
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      unorderedMatches(<dynamic>[
        contains('direction: horizontal'),
        contains('alignment: start'),
        contains('spacing: 8.0'),
        contains('runAlignment: start'),
        contains('runSpacing: 4.0'),
        contains('crossAxisAlignment: start'),
        contains('textDirection: ltr'),
        contains('verticalDirection: up'),
      ]),
    );
  });

  testWidgets('Row and IgnoreBaseline (control -- with baseline)', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Row(
        crossAxisAlignment: CrossAxisAlignment.baseline,
        textBaseline: TextBaseline.alphabetic,
        textDirection: TextDirection.ltr,
        children: <Widget>[
          Text(
            'a',
            textDirection: TextDirection.ltr,
            style: TextStyle(fontSize: 128.0, fontFamily: 'FlutterTest'), // places baseline at y=96
          ),
          Text(
            'b',
            textDirection: TextDirection.ltr,
            style: TextStyle(
              fontSize: 32.0,
              fontFamily: 'FlutterTest',
            ), // 24 above baseline, 8 below baseline
          ),
        ],
      ),
    );

    final Offset aPos = tester.getTopLeft(find.text('a'));
    final Offset bPos = tester.getTopLeft(find.text('b'));
    expect(aPos.dy, 0.0);
    expect(bPos.dy, 96.0 - 24.0);
  });

  testWidgets('Row and IgnoreBaseline (with ignored baseline)', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Row(
        crossAxisAlignment: CrossAxisAlignment.baseline,
        textBaseline: TextBaseline.alphabetic,
        textDirection: TextDirection.ltr,
        children: <Widget>[
          IgnoreBaseline(
            child: Text(
              'a',
              textDirection: TextDirection.ltr,
              style: TextStyle(
                fontSize: 128.0,
                fontFamily: 'FlutterTest',
              ), // places baseline at y=96
            ),
          ),
          Text(
            'b',
            textDirection: TextDirection.ltr,
            style: TextStyle(
              fontSize: 32.0,
              fontFamily: 'FlutterTest',
            ), // 24 above baseline, 8 below baseline
          ),
        ],
      ),
    );

    final Offset aPos = tester.getTopLeft(find.text('a'));
    final Offset bPos = tester.getTopLeft(find.text('b'));
    expect(aPos.dy, 0.0);
    expect(bPos.dy, 0.0);
  });
}

HitsRenderBox hits(RenderBox renderBox) => HitsRenderBox(renderBox);

class HitsRenderBox extends Matcher {
  const HitsRenderBox(this.renderBox);

  final RenderBox renderBox;

  @override
  Description describe(Description description) =>
      description.add('hit test result contains ').addDescriptionOf(renderBox);

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    final HitTestResult hitTestResult = item as HitTestResult;
    return hitTestResult.path.where((HitTestEntry entry) => entry.target == renderBox).isNotEmpty;
  }
}

DoesNotHitRenderBox doesNotHit(RenderBox renderBox) => DoesNotHitRenderBox(renderBox);

class DoesNotHitRenderBox extends Matcher {
  const DoesNotHitRenderBox(this.renderBox);

  final RenderBox renderBox;

  @override
  Description describe(Description description) =>
      description.add("hit test result doesn't contain ").addDescriptionOf(renderBox);

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    final HitTestResult hitTestResult = item as HitTestResult;
    return hitTestResult.path.where((HitTestEntry entry) => entry.target == renderBox).isEmpty;
  }
}

class _MockPaintingContext extends Fake implements PaintingContext {
  final List<RenderObject> children = <RenderObject>[];
  final List<Offset> offsets = <Offset>[];

  @override
  final _MockCanvas canvas = _MockCanvas();

  @override
  void paintChild(RenderObject child, Offset offset) {
    children.add(child);
    offsets.add(offset);
  }
}

class _MockCanvas extends Fake implements Canvas {
  final List<Rect> rects = <Rect>[];
  final List<Paint> paints = <Paint>[];
  bool didPaint = false;

  @override
  void drawRect(Rect rect, Paint paint) {
    rects.add(rect);
    paints.add(paint);
  }
}

class TestIgnorePointer extends StatefulWidget {
  const TestIgnorePointer({super.key, required this.child});

  final Widget child;
  @override
  State<StatefulWidget> createState() => TestIgnorePointerState();
}

class TestIgnorePointerState extends State<TestIgnorePointer> {
  bool ignore = true;

  void setIgnore(bool newIgnore) {
    setState(() {
      ignore = newIgnore;
    });
  }

  @override
  Widget build(BuildContext context) {
    return IgnorePointer(ignoring: ignore, child: widget.child);
  }
}
