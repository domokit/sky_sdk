// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('overflow should not affect baseline', () {
    RenderBox root, child, text;
    late double baseline1, baseline2, height1, height2;

    root = RenderPositionedBox(
      child: RenderCustomPaint(
        child:
            child =
                text = RenderParagraph(
                  const TextSpan(text: 'Hello World'),
                  textDirection: TextDirection.ltr,
                ),
        painter: TestCallbackPainter(
          onPaint: () {
            baseline1 = child.getDistanceToBaseline(TextBaseline.alphabetic)!;
            height1 = text.size.height;
          },
        ),
      ),
    );
    layout(root, phase: EnginePhase.paint);

    root = RenderPositionedBox(
      child: RenderCustomPaint(
        child:
            child = RenderConstrainedOverflowBox(
              child:
                  text = RenderParagraph(
                    const TextSpan(text: 'Hello World'),
                    textDirection: TextDirection.ltr,
                  ),
              maxHeight: height1 / 2.0,
              alignment: Alignment.topLeft,
            ),
        painter: TestCallbackPainter(
          onPaint: () {
            baseline2 = child.getDistanceToBaseline(TextBaseline.alphabetic)!;
            height2 = text.size.height;
          },
        ),
      ),
    );
    layout(root, phase: EnginePhase.paint);

    expect(baseline1, lessThan(height1));
    expect(height2, equals(height1 / 2.0));
    expect(baseline2, equals(baseline1));
    expect(baseline2, greaterThan(height2));
  });
}
