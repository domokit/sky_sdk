// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('RenderAnimatedOpacityMixin does not drop layer when animating to 1', (
    WidgetTester tester,
  ) async {
    RenderTestObject.paintCount = 0;
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(seconds: 1),
    );
    addTearDown(controller.dispose);
    final Tween<double> opacityTween = Tween<double>(begin: 0, end: 1);
    await tester.pumpWidget(
      ColoredBox(
        color: Colors.red,
        child: FadeTransition(opacity: controller.drive(opacityTween), child: const TestWidget()),
      ),
    );

    expect(RenderTestObject.paintCount, 0);
    controller.forward();

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    expect(RenderTestObject.paintCount, 1);

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    expect(RenderTestObject.paintCount, 1);

    controller.stop();
    await tester.pump();

    expect(RenderTestObject.paintCount, 1);
  });

  testWidgets('RenderAnimatedOpacityMixin avoids repainting child as it animates', (
    WidgetTester tester,
  ) async {
    RenderTestObject.paintCount = 0;
    final AnimationController controller = AnimationController(
      vsync: const TestVSync(),
      duration: const Duration(seconds: 1),
    );
    addTearDown(controller.dispose);
    final Tween<double> opacityTween = Tween<double>(begin: 0, end: 0.99); // Layer is dropped at 1
    await tester.pumpWidget(
      ColoredBox(
        color: Colors.red,
        child: FadeTransition(opacity: controller.drive(opacityTween), child: const TestWidget()),
      ),
    );

    expect(RenderTestObject.paintCount, 0);
    controller.forward();

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    expect(RenderTestObject.paintCount, 1);

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    expect(RenderTestObject.paintCount, 1);

    controller.stop();
    await tester.pump();

    expect(RenderTestObject.paintCount, 1);
  });

  testWidgets(
    'RenderAnimatedOpacityMixin allows opacity layer to be disposed when animating to 0 opacity',
    (WidgetTester tester) async {
      RenderTestObject.paintCount = 0;
      final AnimationController controller = AnimationController(
        vsync: const TestVSync(),
        duration: const Duration(seconds: 1),
      );
      addTearDown(controller.dispose);
      final Tween<double> opacityTween = Tween<double>(begin: 0.99, end: 0);

      await tester.pumpWidget(
        ColoredBox(
          color: Colors.red,
          child: FadeTransition(opacity: controller.drive(opacityTween), child: const TestWidget()),
        ),
      );

      expect(RenderTestObject.paintCount, 1);
      expect(tester.layers, contains(isA<OpacityLayer>()));
      controller.forward();

      await tester.pump();
      await tester.pump(const Duration(seconds: 2));

      expect(RenderTestObject.paintCount, 1);

      controller.stop();
      await tester.pump();

      expect(tester.layers, isNot(contains(isA<OpacityLayer>())));
    },
  );
}

class TestWidget extends SingleChildRenderObjectWidget {
  const TestWidget({super.key, super.child});

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderTestObject();
  }
}

class RenderTestObject extends RenderProxyBox {
  static int paintCount = 0;

  @override
  void paint(PaintingContext context, Offset offset) {
    paintCount += 1;
    super.paint(context, offset);
  }
}
