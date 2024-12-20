// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import '../impeller_test_helpers.dart';

Shader createShader(Rect bounds) {
  return const LinearGradient(
    begin: Alignment.topCenter,
    end: Alignment.bottomCenter,
    colors: <Color>[Color(0x00FFFFFF), Color(0xFFFFFFFF)],
    stops: <double>[0.1, 0.35],
  ).createShader(bounds);
}

void main() {
  testWidgets('Can be constructed', (WidgetTester tester) async {
    const Widget child = SizedBox(width: 100.0, height: 100.0);
    await tester.pumpWidget(const ShaderMask(shaderCallback: createShader, child: child));
  });

  testWidgets('Bounds rect includes offset', (WidgetTester tester) async {
    late Rect shaderBounds;
    Shader recordShaderBounds(Rect bounds) {
      shaderBounds = bounds;
      return createShader(bounds);
    }

    final Widget widget = Align(
      child: SizedBox(
        width: 400.0,
        height: 400.0,
        child: ShaderMask(
          shaderCallback: recordShaderBounds,
          child: const SizedBox(width: 100.0, height: 100.0),
        ),
      ),
    );
    await tester.pumpWidget(widget);

    // The shader bounds rectangle should reflect the position of the centered SizedBox.
    expect(shaderBounds, equals(const Rect.fromLTWH(0.0, 0.0, 400.0, 400.0)));
  });

  testWidgets('Bounds rect includes offset visual inspection', (WidgetTester tester) async {
    final Widget widgetBottomRight = Container(
      width: 400,
      height: 400,
      color: const Color(0xFFFFFFFF),
      child: RepaintBoundary(
        child: Align(
          alignment: Alignment.bottomRight,
          child: ShaderMask(
            shaderCallback:
                (Rect bounds) => const RadialGradient(
                  radius: 0.05,
                  colors: <Color>[Color(0xFFFF0000), Color(0xFF00FF00)],
                  tileMode: TileMode.mirror,
                ).createShader(bounds),
            child: Container(width: 100, height: 100, color: const Color(0xFFFFFFFF)),
          ),
        ),
      ),
    );
    await tester.pumpWidget(widgetBottomRight);

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('shader_mask.bounds.matches_bottom_right.png'),
    );

    final Widget widgetTopLeft = Container(
      width: 400,
      height: 400,
      color: const Color(0xFFFFFFFF),
      child: RepaintBoundary(
        child: Align(
          alignment: Alignment.topLeft,
          child: ShaderMask(
            shaderCallback:
                (Rect bounds) => const RadialGradient(
                  radius: 0.05,
                  colors: <Color>[Color(0xFFFF0000), Color(0xFF00FF00)],
                  tileMode: TileMode.mirror,
                ).createShader(bounds),
            child: Container(width: 100, height: 100, color: const Color(0xFFFFFFFF)),
          ),
        ),
      ),
    );
    await tester.pumpWidget(widgetTopLeft);

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('shader_mask.bounds.matches_top_left.png'),
    );
  }, skip: impellerEnabled); // https://github.com/flutter/flutter/issues/144555
}
