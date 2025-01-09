// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['reduced-test-set'])
library;

import 'dart:math' as math show pi;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/semantics_tester.dart';

class MockCanvas extends Fake implements Canvas {
  late Path capturedPath;
  late Paint capturedPaint;

  @override
  void drawPath(Path path, Paint paint) {
    capturedPath = path;
    capturedPaint = paint;
  }

  late double capturedSx;
  late double capturedSy;

  @override
  void scale(double sx, [double? sy]) {
    capturedSx = sx;
    capturedSy = sy!;
    invocations.add(RecordedScale(sx, sy));
  }

  final List<RecordedCanvasCall> invocations = <RecordedCanvasCall>[];

  @override
  void rotate(double radians) {
    invocations.add(RecordedRotate(radians));
  }

  @override
  void translate(double dx, double dy) {
    invocations.add(RecordedTranslate(dx, dy));
  }
}

@immutable
abstract class RecordedCanvasCall {
  const RecordedCanvasCall();
}

class RecordedRotate extends RecordedCanvasCall {
  const RecordedRotate(this.radians);

  final double radians;

  @override
  bool operator ==(Object other) {
    return other is RecordedRotate && other.radians == radians;
  }

  @override
  int get hashCode => radians.hashCode;
}

class RecordedTranslate extends RecordedCanvasCall {
  const RecordedTranslate(this.dx, this.dy);

  final double dx;
  final double dy;

  @override
  bool operator ==(Object other) {
    return other is RecordedTranslate && other.dx == dx && other.dy == dy;
  }

  @override
  int get hashCode => Object.hash(dx, dy);
}

class RecordedScale extends RecordedCanvasCall {
  const RecordedScale(this.sx, this.sy);

  final double sx;
  final double sy;

  @override
  bool operator ==(Object other) {
    return other is RecordedScale && other.sx == sx && other.sy == sy;
  }

  @override
  int get hashCode => Object.hash(sx, sy);
}

void main() {
  testWidgets('IconTheme color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666)),
          child: AnimatedIcon(
            progress: AlwaysStoppedAnimation<double>(0.0),
            icon: AnimatedIcons.arrow_menu,
          ),
        ),
      ),
    );
    final CustomPaint customPaint = tester.widget(find.byType(CustomPaint));
    final MockCanvas canvas = MockCanvas();
    customPaint.painter!.paint(canvas, const Size(48.0, 48.0));
    expect(canvas.capturedPaint, hasColor(0xFF666666));
  });

  testWidgets('IconTheme opacity', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666), opacity: 0.5),
          child: AnimatedIcon(
            progress: AlwaysStoppedAnimation<double>(0.0),
            icon: AnimatedIcons.arrow_menu,
          ),
        ),
      ),
    );
    final CustomPaint customPaint = tester.widget(find.byType(CustomPaint));
    final MockCanvas canvas = MockCanvas();
    customPaint.painter!.paint(canvas, const Size(48.0, 48.0));
    expect(canvas.capturedPaint, hasColor(0x80666666));
  });

  testWidgets('color overrides IconTheme color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666)),
          child: AnimatedIcon(
            progress: AlwaysStoppedAnimation<double>(0.0),
            icon: AnimatedIcons.arrow_menu,
            color: Color(0xFF0000FF),
          ),
        ),
      ),
    );
    final CustomPaint customPaint = tester.widget(find.byType(CustomPaint));
    final MockCanvas canvas = MockCanvas();
    customPaint.painter!.paint(canvas, const Size(48.0, 48.0));
    expect(canvas.capturedPaint, hasColor(0xFF0000FF));
  });

  testWidgets('IconTheme size', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666), size: 12.0),
          child: AnimatedIcon(
            progress: AlwaysStoppedAnimation<double>(0.0),
            icon: AnimatedIcons.arrow_menu,
          ),
        ),
      ),
    );
    final CustomPaint customPaint = tester.widget(find.byType(CustomPaint));
    final MockCanvas canvas = MockCanvas();
    customPaint.painter!.paint(canvas, const Size(12.0, 12.0));
    // arrow_menu default size is 48x48 so we expect it to be scaled by 0.25.
    expect(canvas.capturedSx, 0.25);
    expect(canvas.capturedSy, 0.25);
  });

  testWidgets('size overridesIconTheme size', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666), size: 12.0),
          child: AnimatedIcon(
            progress: AlwaysStoppedAnimation<double>(0.0),
            icon: AnimatedIcons.arrow_menu,
            size: 96.0,
          ),
        ),
      ),
    );
    final CustomPaint customPaint = tester.widget(find.byType(CustomPaint));
    final MockCanvas canvas = MockCanvas();
    customPaint.painter!.paint(canvas, const Size(12.0, 12.0));
    // arrow_menu default size is 48x48 so we expect it to be scaled by 2.
    expect(canvas.capturedSx, 2);
    expect(canvas.capturedSy, 2);
  });

  testWidgets('Semantic label', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedIcon(
          progress: AlwaysStoppedAnimation<double>(0.0),
          icon: AnimatedIcons.arrow_menu,
          size: 96.0,
          semanticLabel: 'a label',
        ),
      ),
    );

    expect(semantics, includesNodeWith(label: 'a label'));

    semantics.dispose();
  });

  testWidgets('Inherited text direction rtl', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.rtl,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666)),
          child: RepaintBoundary(
            child: AnimatedIcon(
              progress: AlwaysStoppedAnimation<double>(0.0),
              icon: AnimatedIcons.arrow_menu,
            ),
          ),
        ),
      ),
    );
    final CustomPaint customPaint = tester.widget(find.byType(CustomPaint));
    final MockCanvas canvas = MockCanvas();
    customPaint.painter!.paint(canvas, const Size(48.0, 48.0));
    expect(canvas.invocations, const <RecordedCanvasCall>[
      RecordedRotate(math.pi),
      RecordedTranslate(-48, -48),
      RecordedScale(0.5, 0.5),
    ]);
    await expectLater(
      find.byType(AnimatedIcon),
      matchesGoldenFile('animated_icons_test.icon.rtl.png'),
    );
  });

  testWidgets('Inherited text direction ltr', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666)),
          child: RepaintBoundary(
            child: AnimatedIcon(
              progress: AlwaysStoppedAnimation<double>(0.0),
              icon: AnimatedIcons.arrow_menu,
            ),
          ),
        ),
      ),
    );
    final CustomPaint customPaint = tester.widget(find.byType(CustomPaint));
    final MockCanvas canvas = MockCanvas();
    customPaint.painter!.paint(canvas, const Size(48.0, 48.0));
    expect(canvas.invocations, const <RecordedCanvasCall>[RecordedScale(0.5, 0.5)]);
    await expectLater(
      find.byType(AnimatedIcon),
      matchesGoldenFile('animated_icons_test.icon.ltr.png'),
    );
  });

  testWidgets('Inherited text direction overridden', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: IconTheme(
          data: IconThemeData(color: Color(0xFF666666)),
          child: AnimatedIcon(
            progress: AlwaysStoppedAnimation<double>(0.0),
            icon: AnimatedIcons.arrow_menu,
            textDirection: TextDirection.rtl,
          ),
        ),
      ),
    );
    final CustomPaint customPaint = tester.widget(find.byType(CustomPaint));
    final MockCanvas canvas = MockCanvas();
    customPaint.painter!.paint(canvas, const Size(48.0, 48.0));
    expect(canvas.invocations, const <RecordedCanvasCall>[
      RecordedRotate(math.pi),
      RecordedTranslate(-48, -48),
      RecordedScale(0.5, 0.5),
    ]);
  });

  testWidgets('Direction has no effect on position of widget', (WidgetTester tester) async {
    const AnimatedIcon icon = AnimatedIcon(
      progress: AlwaysStoppedAnimation<double>(0.0),
      icon: AnimatedIcons.arrow_menu,
    );
    await tester.pumpWidget(const Directionality(textDirection: TextDirection.rtl, child: icon));
    final Rect rtlRect = tester.getRect(find.byType(AnimatedIcon));
    await tester.pumpWidget(const Directionality(textDirection: TextDirection.ltr, child: icon));
    final Rect ltrRect = tester.getRect(find.byType(AnimatedIcon));
    expect(rtlRect, ltrRect);
  });
}

PaintColorMatcher hasColor(int color, {double threshold = 1 / 255}) {
  return PaintColorMatcher(color, threshold);
}

class PaintColorMatcher extends Matcher {
  const PaintColorMatcher(this.expectedColor, this.threshold);

  final int expectedColor;
  final double threshold;

  @override
  Description describe(Description description) => description.add('color was not $expectedColor');

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    final Paint actualPaint = item as Paint;
    final Color expected = Color(expectedColor);
    return actualPaint.color.colorSpace == expected.colorSpace &&
        (actualPaint.color.a - expected.a).abs() < threshold &&
        (actualPaint.color.r - expected.r).abs() < threshold &&
        (actualPaint.color.g - expected.g).abs() < threshold &&
        (actualPaint.color.b - expected.b).abs() < threshold;
  }
}
