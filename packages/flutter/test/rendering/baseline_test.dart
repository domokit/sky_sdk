// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('RenderBaseline', () {
    RenderBaseline parent;
    RenderSizedBox child;
    final RenderBox root = RenderPositionedBox(
      alignment: Alignment.topLeft,
      child:
          parent = RenderBaseline(
            baseline: 0.0,
            baselineType: TextBaseline.alphabetic,
            child: child = RenderSizedBox(const Size(100.0, 100.0)),
          ),
    );
    final BoxParentData childParentData = child.parentData! as BoxParentData;

    layout(root);
    expect(childParentData.offset.dx, equals(0.0));
    expect(childParentData.offset.dy, equals(-100.0));
    expect(parent.size, equals(const Size(100.0, 0.0)));

    parent.baseline = 25.0;
    pumpFrame();
    expect(childParentData.offset.dx, equals(0.0));
    expect(childParentData.offset.dy, equals(-75.0));
    expect(parent.size, equals(const Size(100.0, 25.0)));

    parent.baseline = 90.0;
    pumpFrame();
    expect(childParentData.offset.dx, equals(0.0));
    expect(childParentData.offset.dy, equals(-10.0));
    expect(parent.size, equals(const Size(100.0, 90.0)));

    parent.baseline = 100.0;
    pumpFrame();
    expect(childParentData.offset.dx, equals(0.0));
    expect(childParentData.offset.dy, equals(0.0));
    expect(parent.size, equals(const Size(100.0, 100.0)));

    parent.baseline = 110.0;
    pumpFrame();
    expect(childParentData.offset.dx, equals(0.0));
    expect(childParentData.offset.dy, equals(10.0));
    expect(parent.size, equals(const Size(100.0, 110.0)));
  });

  test('RenderBaseline different baseline types', () {
    final _RenderBaselineTester child =
        _RenderBaselineTester()
          ..boxSize = const Size.square(100)
          ..alphabeticBaselineOffset = 50
          ..ideographicBaselineOffset = 60;
    final RenderBaseline renderBaseline = RenderBaseline(
      baseline: 1.0,
      baselineType: TextBaseline.alphabetic,
      child: child,
    );

    layout(renderBaseline, phase: EnginePhase.paint);
    expect(renderBaseline.getDryBaseline(const BoxConstraints(), TextBaseline.alphabetic), 1.0);
    expect(renderBaseline.getDryBaseline(const BoxConstraints(), TextBaseline.ideographic), 11.0);

    child
      ..alphabeticBaselineOffset = null
      ..ideographicBaselineOffset = null
      ..markNeedsLayout(); // Clears baseline cache.

    renderBaseline.markNeedsLayout();

    pumpFrame(phase: EnginePhase.paint);
    expect(renderBaseline.getDryBaseline(const BoxConstraints(), TextBaseline.alphabetic), isNull);
    expect(renderBaseline.getDryBaseline(const BoxConstraints(), TextBaseline.ideographic), isNull);
  });

  test('RenderFlex and RenderIgnoreBaseline (control test -- with baseline)', () {
    final RenderBox a, b;
    final RenderBox root = RenderFlex(
      crossAxisAlignment: CrossAxisAlignment.baseline,
      textBaseline: TextBaseline.alphabetic,
      textDirection: TextDirection.ltr,
      children: <RenderBox>[
        a = RenderParagraph(
          const TextSpan(
            text: 'a',
            style: TextStyle(fontSize: 128.0, fontFamily: 'FlutterTest'),
          ), // places baseline at y=96
          textDirection: TextDirection.ltr,
        ),
        b = RenderParagraph(
          const TextSpan(
            text: 'b',
            style: TextStyle(fontSize: 32.0, fontFamily: 'FlutterTest'),
          ), // 24 above baseline, 8 below baseline
          textDirection: TextDirection.ltr,
        ),
      ],
    );
    layout(root);

    final Offset aPos = a.localToGlobal(Offset.zero);
    final Offset bPos = b.localToGlobal(Offset.zero);
    expect(aPos.dy, 0.0);
    expect(bPos.dy, 96.0 - 24.0);
  });

  test('RenderFlex and RenderIgnoreBaseline (with ignored baseline)', () {
    final RenderBox a, b;
    final RenderBox root = RenderFlex(
      crossAxisAlignment: CrossAxisAlignment.baseline,
      textBaseline: TextBaseline.alphabetic,
      textDirection: TextDirection.ltr,
      children: <RenderBox>[
        RenderIgnoreBaseline(
          child:
              a = RenderParagraph(
                const TextSpan(
                  text: 'a',
                  style: TextStyle(fontSize: 128.0, fontFamily: 'FlutterTest'),
                ),
                textDirection: TextDirection.ltr,
              ),
        ),
        b = RenderParagraph(
          const TextSpan(text: 'b', style: TextStyle(fontSize: 32.0, fontFamily: 'FlutterTest')),
          textDirection: TextDirection.ltr,
        ),
      ],
    );
    layout(root);

    final Offset aPos = a.localToGlobal(Offset.zero);
    final Offset bPos = b.localToGlobal(Offset.zero);
    expect(aPos.dy, 0.0);
    expect(bPos.dy, 0.0);
  });
}

class _RenderBaselineTester extends RenderBox {
  Size boxSize = Size.zero;
  double? alphabeticBaselineOffset;
  double? ideographicBaselineOffset;

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return switch (baseline) {
      TextBaseline.alphabetic => alphabeticBaselineOffset,
      TextBaseline.ideographic => ideographicBaselineOffset,
    };
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    return switch (baseline) {
      TextBaseline.alphabetic => alphabeticBaselineOffset,
      TextBaseline.ideographic => ideographicBaselineOffset,
    };
  }

  @override
  Size computeDryLayout(covariant BoxConstraints constraints) => constraints.constrain(boxSize);

  @override
  void performLayout() {
    size = computeDryLayout(constraints);
  }
}
