// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('toString control test', (WidgetTester tester) async {
    const Widget widget = FadeTransition(
      opacity: kAlwaysCompleteAnimation,
      child: Text('Ready', textDirection: TextDirection.ltr),
    );
    expect(widget.toString, isNot(throwsException));
  });

  group('DecoratedBoxTransition test', () {
    final DecorationTween decorationTween = DecorationTween(
      begin: BoxDecoration(
        color: const Color(0xFFFFFFFF),
        border: Border.all(width: 4.0),
        borderRadius: BorderRadius.zero,
        boxShadow: const <BoxShadow>[
          BoxShadow(color: Color(0x66000000), blurRadius: 10.0, spreadRadius: 4.0),
        ],
      ),
      end: BoxDecoration(
        color: const Color(0xFF000000),
        border: Border.all(color: const Color(0xFF202020)),
        borderRadius: const BorderRadius.all(Radius.circular(10.0)),
        // No shadow.
      ),
    );

    late AnimationController controller;

    setUp(() {
      controller = AnimationController(vsync: const TestVSync());
    });

    testWidgets('decoration test', (WidgetTester tester) async {
      final DecoratedBoxTransition transitionUnderTest = DecoratedBoxTransition(
        decoration: decorationTween.animate(controller),
        child: const Text("Doesn't matter", textDirection: TextDirection.ltr),
      );

      await tester.pumpWidget(transitionUnderTest);
      RenderDecoratedBox actualBox = tester.renderObject(find.byType(DecoratedBox));
      BoxDecoration actualDecoration = actualBox.decoration as BoxDecoration;

      expect(actualDecoration.color, isSameColorAs(const Color(0xFFFFFFFF)));
      expect(actualDecoration.boxShadow![0].blurRadius, 10.0);
      expect(actualDecoration.boxShadow![0].spreadRadius, 4.0);
      expect(actualDecoration.boxShadow![0].color, const Color(0x66000000));

      controller.value = 0.5;

      await tester.pump();
      actualBox = tester.renderObject(find.byType(DecoratedBox));
      actualDecoration = actualBox.decoration as BoxDecoration;

      expect(actualDecoration.color, isSameColorAs(const Color(0xFF7F7F7F)));
      expect(actualDecoration.border, isA<Border>());
      final Border border = actualDecoration.border! as Border;
      expect(border.left.width, 2.5);
      expect(border.left.style, BorderStyle.solid);
      expect(border.left.color, const Color(0xFF101010));
      expect(actualDecoration.borderRadius, const BorderRadius.all(Radius.circular(5.0)));
      expect(actualDecoration.shape, BoxShape.rectangle);
      expect(actualDecoration.boxShadow![0].blurRadius, 5.0);
      expect(actualDecoration.boxShadow![0].spreadRadius, 2.0);
      // Scaling a shadow doesn't change the color.
      expect(actualDecoration.boxShadow![0].color, const Color(0x66000000));

      controller.value = 1.0;

      await tester.pump();
      actualBox = tester.renderObject(find.byType(DecoratedBox));
      actualDecoration = actualBox.decoration as BoxDecoration;

      expect(actualDecoration.color, const Color(0xFF000000));
      expect(actualDecoration.boxShadow, null);
    });

    testWidgets('animations work with curves test', (WidgetTester tester) async {
      final CurvedAnimation curvedAnimation = CurvedAnimation(
        parent: controller,
        curve: Curves.easeOut,
      );
      addTearDown(curvedAnimation.dispose);
      final Animation<Decoration> curvedDecorationAnimation = decorationTween.animate(
        curvedAnimation,
      );

      final DecoratedBoxTransition transitionUnderTest = DecoratedBoxTransition(
        decoration: curvedDecorationAnimation,
        position: DecorationPosition.foreground,
        child: const Text("Doesn't matter", textDirection: TextDirection.ltr),
      );

      await tester.pumpWidget(transitionUnderTest);

      RenderDecoratedBox actualBox = tester.renderObject(find.byType(DecoratedBox));
      BoxDecoration actualDecoration = actualBox.decoration as BoxDecoration;

      expect(actualDecoration.color, isSameColorAs(const Color(0xFFFFFFFF)));
      expect(actualDecoration.boxShadow![0].blurRadius, 10.0);
      expect(actualDecoration.boxShadow![0].spreadRadius, 4.0);
      expect(actualDecoration.boxShadow![0].color, isSameColorAs(const Color(0x66000000)));

      controller.value = 0.5;

      await tester.pump();
      actualBox = tester.renderObject(find.byType(DecoratedBox));
      actualDecoration = actualBox.decoration as BoxDecoration;

      // Same as the test above but the values should be much closer to the
      // tween's end values given the easeOut curve.
      expect(actualDecoration.color, isSameColorAs(const Color(0xFF505050)));
      expect(actualDecoration.border, isA<Border>());
      final Border border = actualDecoration.border! as Border;
      expect(border.left.width, moreOrLessEquals(1.9, epsilon: 0.1));
      expect(border.left.style, BorderStyle.solid);
      expect(border.left.color, isSameColorAs(const Color(0xFF151515)));
      expect(
        actualDecoration.borderRadius!.resolve(TextDirection.ltr).topLeft.x,
        moreOrLessEquals(6.8, epsilon: 0.1),
      );
      expect(actualDecoration.shape, BoxShape.rectangle);
      expect(actualDecoration.boxShadow![0].blurRadius, moreOrLessEquals(3.1, epsilon: 0.1));
      expect(actualDecoration.boxShadow![0].spreadRadius, moreOrLessEquals(1.2, epsilon: 0.1));
      // Scaling a shadow doesn't change the color.
      expect(actualDecoration.boxShadow![0].color, const Color(0x66000000));
    });
  });

  testWidgets('AlignTransition animates', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Animation<Alignment> alignmentTween = AlignmentTween(
      begin: Alignment.centerLeft,
      end: Alignment.bottomRight,
    ).animate(controller);
    final Widget widget = AlignTransition(
      alignment: alignmentTween,
      child: const Text('Ready', textDirection: TextDirection.ltr),
    );

    await tester.pumpWidget(widget);

    final RenderPositionedBox actualPositionedBox = tester.renderObject(find.byType(Align));

    Alignment actualAlignment = actualPositionedBox.alignment as Alignment;
    expect(actualAlignment, Alignment.centerLeft);

    controller.value = 0.5;
    await tester.pump();
    actualAlignment = actualPositionedBox.alignment as Alignment;
    expect(actualAlignment, const Alignment(0.0, 0.5));
  });

  testWidgets('RelativePositionedTransition animates', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Animation<Rect?> rectTween = RectTween(
      begin: const Rect.fromLTWH(0, 0, 30, 40),
      end: const Rect.fromLTWH(100, 200, 100, 200),
    ).animate(controller);
    final Widget widget = Directionality(
      textDirection: TextDirection.rtl,
      child: Stack(
        alignment: Alignment.centerLeft,
        children: <Widget>[
          RelativePositionedTransition(
            size: const Size(200, 300),
            rect: rectTween,
            child: const Placeholder(),
          ),
        ],
      ),
    );

    await tester.pumpWidget(widget);

    final Positioned actualPositioned = tester.widget(find.byType(Positioned));
    final RenderBox renderBox = tester.renderObject(find.byType(Placeholder));

    Rect actualRect = Rect.fromLTRB(
      actualPositioned.left!,
      actualPositioned.top!,
      actualPositioned.right ?? 0.0,
      actualPositioned.bottom ?? 0.0,
    );
    expect(actualRect, equals(const Rect.fromLTRB(0, 0, 170, 260)));
    expect(renderBox.size, equals(const Size(630, 340)));

    controller.value = 0.5;
    await tester.pump();
    actualRect = Rect.fromLTRB(
      actualPositioned.left!,
      actualPositioned.top!,
      actualPositioned.right ?? 0.0,
      actualPositioned.bottom ?? 0.0,
    );
    expect(actualRect, equals(const Rect.fromLTWH(0, 0, 170, 260)));
    expect(renderBox.size, equals(const Size(665, 420)));
  });

  testWidgets('AlignTransition keeps width and height factors', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Animation<Alignment> alignmentTween = AlignmentTween(
      begin: Alignment.centerLeft,
      end: Alignment.bottomRight,
    ).animate(controller);
    final Widget widget = AlignTransition(
      alignment: alignmentTween,
      widthFactor: 0.3,
      heightFactor: 0.4,
      child: const Text('Ready', textDirection: TextDirection.ltr),
    );

    await tester.pumpWidget(widget);

    final Align actualAlign = tester.widget(find.byType(Align));

    expect(actualAlign.widthFactor, 0.3);
    expect(actualAlign.heightFactor, 0.4);
  });

  testWidgets('SizeTransition clamps negative size factors - vertical axis', (
    WidgetTester tester,
  ) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Animation<double> animation = Tween<double>(begin: -1.0, end: 1.0).animate(controller);

    final Widget widget = Directionality(
      textDirection: TextDirection.ltr,
      child: SizeTransition(
        sizeFactor: animation,
        fixedCrossAxisSizeFactor: 2.0,
        child: const Text('Ready'),
      ),
    );

    await tester.pumpWidget(widget);

    final RenderPositionedBox actualPositionedBox = tester.renderObject(find.byType(Align));
    expect(actualPositionedBox.heightFactor, 0.0);
    expect(actualPositionedBox.widthFactor, 2.0);

    controller.value = 0.0;
    await tester.pump();
    expect(actualPositionedBox.heightFactor, 0.0);
    expect(actualPositionedBox.widthFactor, 2.0);

    controller.value = 0.75;
    await tester.pump();
    expect(actualPositionedBox.heightFactor, 0.5);
    expect(actualPositionedBox.widthFactor, 2.0);

    controller.value = 1.0;
    await tester.pump();
    expect(actualPositionedBox.heightFactor, 1.0);
    expect(actualPositionedBox.widthFactor, 2.0);
  });

  testWidgets('SizeTransition clamps negative size factors - horizontal axis', (
    WidgetTester tester,
  ) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Animation<double> animation = Tween<double>(begin: -1.0, end: 1.0).animate(controller);

    final Widget widget = Directionality(
      textDirection: TextDirection.ltr,
      child: SizeTransition(
        axis: Axis.horizontal,
        sizeFactor: animation,
        fixedCrossAxisSizeFactor: 1.0,
        child: const Text('Ready'),
      ),
    );

    await tester.pumpWidget(widget);

    final RenderPositionedBox actualPositionedBox = tester.renderObject(find.byType(Align));
    expect(actualPositionedBox.widthFactor, 0.0);
    expect(actualPositionedBox.heightFactor, 1.0);

    controller.value = 0.0;
    await tester.pump();
    expect(actualPositionedBox.widthFactor, 0.0);
    expect(actualPositionedBox.heightFactor, 1.0);

    controller.value = 0.75;
    await tester.pump();
    expect(actualPositionedBox.widthFactor, 0.5);
    expect(actualPositionedBox.heightFactor, 1.0);

    controller.value = 1.0;
    await tester.pump();
    expect(actualPositionedBox.widthFactor, 1.0);
    expect(actualPositionedBox.heightFactor, 1.0);
  });

  testWidgets(
    'SizeTransition with fixedCrossAxisSizeFactor should size its cross axis from its children - vertical axis',
    (WidgetTester tester) async {
      final AnimationController controller = AnimationController(vsync: const TestVSync());
      addTearDown(controller.dispose);
      final Animation<double> animation = Tween<double>(begin: 0, end: 1.0).animate(controller);

      const Key key = Key('key');

      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            key: key,
            child: SizeTransition(
              sizeFactor: animation,
              fixedCrossAxisSizeFactor: 1.0,
              child: const SizedBox.square(dimension: 100),
            ),
          ),
        ),
      );

      await tester.pumpWidget(widget);

      final RenderPositionedBox actualPositionedBox = tester.renderObject(find.byType(Align));
      expect(actualPositionedBox.heightFactor, 0.0);
      expect(actualPositionedBox.widthFactor, 1.0);
      expect(tester.getSize(find.byKey(key)), const Size(100, 0));

      controller.value = 0.0;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 0.0);
      expect(actualPositionedBox.widthFactor, 1.0);
      expect(tester.getSize(find.byKey(key)), const Size(100, 0));

      controller.value = 0.5;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 0.5);
      expect(actualPositionedBox.widthFactor, 1.0);
      expect(tester.getSize(find.byKey(key)), const Size(100, 50));

      controller.value = 1.0;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 1.0);
      expect(actualPositionedBox.widthFactor, 1.0);
      expect(tester.getSize(find.byKey(key)), const Size.square(100));

      controller.value = 0.5;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 0.5);
      expect(actualPositionedBox.widthFactor, 1.0);
      expect(tester.getSize(find.byKey(key)), const Size(100, 50));

      controller.value = 0.0;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 0.0);
      expect(actualPositionedBox.widthFactor, 1.0);
      expect(tester.getSize(find.byKey(key)), const Size(100, 0));
    },
  );

  testWidgets(
    'SizeTransition with fixedCrossAxisSizeFactor should size its cross axis from its children - horizontal axis',
    (WidgetTester tester) async {
      final AnimationController controller = AnimationController(vsync: const TestVSync());
      addTearDown(controller.dispose);
      final Animation<double> animation = Tween<double>(begin: 0.0, end: 1.0).animate(controller);

      const Key key = Key('key');

      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            key: key,
            child: SizeTransition(
              axis: Axis.horizontal,
              sizeFactor: animation,
              fixedCrossAxisSizeFactor: 1.0,
              child: const SizedBox.square(dimension: 100),
            ),
          ),
        ),
      );

      await tester.pumpWidget(widget);

      final RenderPositionedBox actualPositionedBox = tester.renderObject(find.byType(Align));
      expect(actualPositionedBox.heightFactor, 1.0);
      expect(actualPositionedBox.widthFactor, 0.0);
      expect(tester.getSize(find.byKey(key)), const Size(0, 100));

      controller.value = 0.0;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 1.0);
      expect(actualPositionedBox.widthFactor, 0.0);
      expect(tester.getSize(find.byKey(key)), const Size(0, 100));

      controller.value = 0.5;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 1.0);
      expect(actualPositionedBox.widthFactor, 0.5);
      expect(tester.getSize(find.byKey(key)), const Size(50, 100));

      controller.value = 1.0;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 1.0);
      expect(actualPositionedBox.widthFactor, 1.0);
      expect(tester.getSize(find.byKey(key)), const Size.square(100));

      controller.value = 0.5;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 1.0);
      expect(actualPositionedBox.widthFactor, 0.5);
      expect(tester.getSize(find.byKey(key)), const Size(50, 100));

      controller.value = 0.0;
      await tester.pump();
      expect(actualPositionedBox.heightFactor, 1.0);
      expect(actualPositionedBox.widthFactor, 0.0);
      expect(tester.getSize(find.byKey(key)), const Size(0, 100));
    },
  );

  testWidgets('MatrixTransition animates', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Widget widget = MatrixTransition(
      alignment: Alignment.topRight,
      onTransform: (double value) => Matrix4.translationValues(value, value, value),
      animation: controller,
      child: const Text('Matrix', textDirection: TextDirection.ltr),
    );

    await tester.pumpWidget(widget);
    Transform actualTransformedBox = tester.widget(find.byType(Transform));
    Matrix4 actualTransform = actualTransformedBox.transform;
    expect(actualTransform, equals(Matrix4.rotationZ(0.0)));

    controller.value = 0.5;
    await tester.pump();
    actualTransformedBox = tester.widget(find.byType(Transform));
    actualTransform = actualTransformedBox.transform;
    expect(
      actualTransform,
      Matrix4.fromList(<double>[
        1.0,
        0.0,
        0.0,
        0.5,
        0.0,
        1.0,
        0.0,
        0.5,
        0.0,
        0.0,
        1.0,
        0.5,
        0.0,
        0.0,
        0.0,
        1.0,
      ])..transpose(),
    );

    controller.value = 0.75;
    await tester.pump();
    actualTransformedBox = tester.widget(find.byType(Transform));
    actualTransform = actualTransformedBox.transform;
    expect(
      actualTransform,
      Matrix4.fromList(<double>[
        1.0,
        0.0,
        0.0,
        0.75,
        0.0,
        1.0,
        0.0,
        0.75,
        0.0,
        0.0,
        1.0,
        0.75,
        0.0,
        0.0,
        0.0,
        1.0,
      ])..transpose(),
    );
  });

  testWidgets('MatrixTransition maintains chosen alignment during animation', (
    WidgetTester tester,
  ) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Widget widget = MatrixTransition(
      alignment: Alignment.topRight,
      onTransform: (double value) => Matrix4.identity(),
      animation: controller,
      child: const Text('Matrix', textDirection: TextDirection.ltr),
    );

    await tester.pumpWidget(widget);
    MatrixTransition actualTransformedBox = tester.widget(find.byType(MatrixTransition));
    Alignment actualAlignment = actualTransformedBox.alignment;
    expect(actualAlignment, Alignment.topRight);

    controller.value = 0.5;
    await tester.pump();
    actualTransformedBox = tester.widget(find.byType(MatrixTransition));
    actualAlignment = actualTransformedBox.alignment;
    expect(actualAlignment, Alignment.topRight);
  });

  testWidgets('RotationTransition animates', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Widget widget = RotationTransition(
      alignment: Alignment.topRight,
      turns: controller,
      child: const Text('Rotation', textDirection: TextDirection.ltr),
    );

    await tester.pumpWidget(widget);
    Transform actualRotatedBox = tester.widget(find.byType(Transform));
    Matrix4 actualTurns = actualRotatedBox.transform;
    expect(actualTurns, equals(Matrix4.rotationZ(0.0)));

    controller.value = 0.5;
    await tester.pump();
    actualRotatedBox = tester.widget(find.byType(Transform));
    actualTurns = actualRotatedBox.transform;
    expect(
      actualTurns,
      matrixMoreOrLessEquals(
        Matrix4.fromList(<double>[
          -1.0,
          0.0,
          0.0,
          0.0,
          0.0,
          -1.0,
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
        ])..transpose(),
      ),
    );

    controller.value = 0.75;
    await tester.pump();
    actualRotatedBox = tester.widget(find.byType(Transform));
    actualTurns = actualRotatedBox.transform;
    expect(
      actualTurns,
      matrixMoreOrLessEquals(
        Matrix4.fromList(<double>[
          0.0,
          1.0,
          0.0,
          0.0,
          -1.0,
          0.0,
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
        ])..transpose(),
      ),
    );
  });

  testWidgets('RotationTransition maintains chosen alignment during animation', (
    WidgetTester tester,
  ) async {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    addTearDown(controller.dispose);
    final Widget widget = RotationTransition(
      alignment: Alignment.topRight,
      turns: controller,
      child: const Text('Rotation', textDirection: TextDirection.ltr),
    );

    await tester.pumpWidget(widget);
    RotationTransition actualRotatedBox = tester.widget(find.byType(RotationTransition));
    Alignment actualAlignment = actualRotatedBox.alignment;
    expect(actualAlignment, Alignment.topRight);

    controller.value = 0.5;
    await tester.pump();
    actualRotatedBox = tester.widget(find.byType(RotationTransition));
    actualAlignment = actualRotatedBox.alignment;
    expect(actualAlignment, Alignment.topRight);
  });

  group('FadeTransition', () {
    double getOpacity(WidgetTester tester, String textValue) {
      final FadeTransition opacityWidget = tester.widget<FadeTransition>(
        find.ancestor(of: find.text(textValue), matching: find.byType(FadeTransition)).first,
      );
      return opacityWidget.opacity.value;
    }

    testWidgets('animates', (WidgetTester tester) async {
      final AnimationController controller = AnimationController(vsync: const TestVSync());
      addTearDown(controller.dispose);
      final Animation<double> animation = Tween<double>(begin: 0.0, end: 1.0).animate(controller);
      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: FadeTransition(opacity: animation, child: const Text('Fade In')),
      );

      await tester.pumpWidget(widget);

      expect(getOpacity(tester, 'Fade In'), 0.0);

      controller.value = 0.25;
      await tester.pump();
      expect(getOpacity(tester, 'Fade In'), 0.25);

      controller.value = 0.5;
      await tester.pump();
      expect(getOpacity(tester, 'Fade In'), 0.5);

      controller.value = 0.75;
      await tester.pump();
      expect(getOpacity(tester, 'Fade In'), 0.75);

      controller.value = 1.0;
      await tester.pump();
      expect(getOpacity(tester, 'Fade In'), 1.0);
    });
  });

  group('SliverFadeTransition', () {
    double getOpacity(WidgetTester tester, String textValue) {
      final SliverFadeTransition opacityWidget = tester.widget<SliverFadeTransition>(
        find.ancestor(of: find.text(textValue), matching: find.byType(SliverFadeTransition)).first,
      );
      return opacityWidget.opacity.value;
    }

    testWidgets('animates', (WidgetTester tester) async {
      final AnimationController controller = AnimationController(vsync: const TestVSync());
      addTearDown(controller.dispose);
      final Animation<double> animation = Tween<double>(begin: 0.0, end: 1.0).animate(controller);
      final Widget widget = Localizations(
        locale: const Locale('en', 'us'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultWidgetsLocalizations.delegate,
          DefaultMaterialLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: CustomScrollView(
              slivers: <Widget>[
                SliverFadeTransition(
                  opacity: animation,
                  sliver: const SliverToBoxAdapter(child: Text('Fade In')),
                ),
              ],
            ),
          ),
        ),
      );

      await tester.pumpWidget(widget);

      expect(getOpacity(tester, 'Fade In'), 0.0);

      controller.value = 0.25;
      await tester.pump();
      expect(getOpacity(tester, 'Fade In'), 0.25);

      controller.value = 0.5;
      await tester.pump();
      expect(getOpacity(tester, 'Fade In'), 0.5);

      controller.value = 0.75;
      await tester.pump();
      expect(getOpacity(tester, 'Fade In'), 0.75);

      controller.value = 1.0;
      await tester.pump();
      expect(getOpacity(tester, 'Fade In'), 1.0);
    });
  });

  group('MatrixTransition', () {
    testWidgets('uses ImageFilter when provided with FilterQuality argument', (
      WidgetTester tester,
    ) async {
      final AnimationController controller = AnimationController(vsync: const TestVSync());
      addTearDown(controller.dispose);
      final Animation<double> animation = Tween<double>(begin: 0.0, end: 1.0).animate(controller);
      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: MatrixTransition(
          animation: animation,
          onTransform: (double value) => Matrix4.identity(),
          filterQuality: FilterQuality.none,
          child: const Text('Matrix Transition'),
        ),
      );

      await tester.pumpWidget(widget);

      // Validate that expensive layer is not left in tree before animation has started.
      expect(tester.layers, isNot(contains(isA<ImageFilterLayer>())));

      controller.value = 0.25;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 0.5;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 0.75;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 1;
      await tester.pump();

      // Validate that expensive layer is not left in tree after animation has finished.
      expect(tester.layers, isNot(contains(isA<ImageFilterLayer>())));
    });
  });

  group('ScaleTransition', () {
    testWidgets('uses ImageFilter when provided with FilterQuality argument', (
      WidgetTester tester,
    ) async {
      final AnimationController controller = AnimationController(vsync: const TestVSync());
      addTearDown(controller.dispose);
      final Animation<double> animation = Tween<double>(begin: 0.0, end: 1.0).animate(controller);
      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: ScaleTransition(
          scale: animation,
          filterQuality: FilterQuality.none,
          child: const Text('Scale Transition'),
        ),
      );

      await tester.pumpWidget(widget);

      // Validate that expensive layer is not left in tree before animation has started.
      expect(tester.layers, isNot(contains(isA<ImageFilterLayer>())));

      controller.value = 0.25;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 0.5;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 0.75;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 1;
      await tester.pump();

      // Validate that expensive layer is not left in tree after animation has finished.
      expect(tester.layers, isNot(contains(isA<ImageFilterLayer>())));
    });
  });

  group('RotationTransition', () {
    testWidgets('uses ImageFilter when provided with FilterQuality argument', (
      WidgetTester tester,
    ) async {
      final AnimationController controller = AnimationController(vsync: const TestVSync());
      addTearDown(controller.dispose);
      final Animation<double> animation = Tween<double>(begin: 0.0, end: 1.0).animate(controller);
      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: RotationTransition(
          turns: animation,
          filterQuality: FilterQuality.none,
          child: const Text('Scale Transition'),
        ),
      );

      await tester.pumpWidget(widget);

      // Validate that expensive layer is not left in tree before animation has started.
      expect(tester.layers, isNot(contains(isA<ImageFilterLayer>())));

      controller.value = 0.25;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 0.5;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 0.75;
      await tester.pump();

      expect(
        tester.layers,
        contains(
          isA<ImageFilterLayer>().having(
            (ImageFilterLayer layer) => layer.imageFilter.toString(),
            'image filter',
            startsWith('ImageFilter.matrix('),
          ),
        ),
      );

      controller.value = 1;
      await tester.pump();

      // Validate that expensive layer is not left in tree after animation has finished.
      expect(tester.layers, isNot(contains(isA<ImageFilterLayer>())));
    });
  });

  group('Builders', () {
    testWidgets('AnimatedBuilder rebuilds when changed', (WidgetTester tester) async {
      final GlobalKey<RedrawCounterState> redrawKey = GlobalKey<RedrawCounterState>();
      final ChangeNotifier notifier = ChangeNotifier();
      addTearDown(notifier.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: AnimatedBuilder(
            animation: notifier,
            builder: (BuildContext context, Widget? child) {
              return RedrawCounter(key: redrawKey, child: child);
            },
          ),
        ),
      );

      expect(redrawKey.currentState!.redraws, equals(1));
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(1));
      notifier.notifyListeners();
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(2));

      // Pump a few more times to make sure that we don't rebuild unnecessarily.
      await tester.pump();
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(2));
    });

    testWidgets("AnimatedBuilder doesn't rebuild the child", (WidgetTester tester) async {
      final GlobalKey<RedrawCounterState> redrawKey = GlobalKey<RedrawCounterState>();
      final GlobalKey<RedrawCounterState> redrawKeyChild = GlobalKey<RedrawCounterState>();
      final ChangeNotifier notifier = ChangeNotifier();
      addTearDown(notifier.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: AnimatedBuilder(
            animation: notifier,
            builder: (BuildContext context, Widget? child) {
              return RedrawCounter(key: redrawKey, child: child);
            },
            child: RedrawCounter(key: redrawKeyChild),
          ),
        ),
      );

      expect(redrawKey.currentState!.redraws, equals(1));
      expect(redrawKeyChild.currentState!.redraws, equals(1));
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(1));
      expect(redrawKeyChild.currentState!.redraws, equals(1));
      notifier.notifyListeners();
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(2));
      expect(redrawKeyChild.currentState!.redraws, equals(1));

      // Pump a few more times to make sure that we don't rebuild unnecessarily.
      await tester.pump();
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(2));
      expect(redrawKeyChild.currentState!.redraws, equals(1));
    });

    testWidgets('ListenableBuilder rebuilds when changed', (WidgetTester tester) async {
      final GlobalKey<RedrawCounterState> redrawKey = GlobalKey<RedrawCounterState>();
      final ChangeNotifier notifier = ChangeNotifier();
      addTearDown(notifier.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListenableBuilder(
            listenable: notifier,
            builder: (BuildContext context, Widget? child) {
              return RedrawCounter(key: redrawKey, child: child);
            },
          ),
        ),
      );

      expect(redrawKey.currentState!.redraws, equals(1));
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(1));
      notifier.notifyListeners();
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(2));

      // Pump a few more times to make sure that we don't rebuild unnecessarily.
      await tester.pump();
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(2));
    });

    testWidgets("ListenableBuilder doesn't rebuild the child", (WidgetTester tester) async {
      final GlobalKey<RedrawCounterState> redrawKey = GlobalKey<RedrawCounterState>();
      final GlobalKey<RedrawCounterState> redrawKeyChild = GlobalKey<RedrawCounterState>();
      final ChangeNotifier notifier = ChangeNotifier();
      addTearDown(notifier.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListenableBuilder(
            listenable: notifier,
            builder: (BuildContext context, Widget? child) {
              return RedrawCounter(key: redrawKey, child: child);
            },
            child: RedrawCounter(key: redrawKeyChild),
          ),
        ),
      );

      expect(redrawKey.currentState!.redraws, equals(1));
      expect(redrawKeyChild.currentState!.redraws, equals(1));
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(1));
      expect(redrawKeyChild.currentState!.redraws, equals(1));
      notifier.notifyListeners();
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(2));
      expect(redrawKeyChild.currentState!.redraws, equals(1));

      // Pump a few more times to make sure that we don't rebuild unnecessarily.
      await tester.pump();
      await tester.pump();
      expect(redrawKey.currentState!.redraws, equals(2));
      expect(redrawKeyChild.currentState!.redraws, equals(1));
    });
  });
}

class RedrawCounter extends StatefulWidget {
  const RedrawCounter({super.key, this.child});

  final Widget? child;

  @override
  State<RedrawCounter> createState() => RedrawCounterState();
}

class RedrawCounterState extends State<RedrawCounter> {
  int redraws = 0;

  @override
  Widget build(BuildContext context) {
    redraws += 1;
    return SizedBox(child: widget.child);
  }
}
