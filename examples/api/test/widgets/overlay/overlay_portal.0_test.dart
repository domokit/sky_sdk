// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/overlay/overlay_portal.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  const String tooltipText = 'tooltip';
  testWidgets('Tooltip is shown on press', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OverlayPortalExampleApp());
    expect(find.text(tooltipText), findsNothing);

    await tester.tap(find.byType(example.ClickableTooltipWidget));
    await tester.pump();
    expect(find.text(tooltipText), findsOneWidget);

    await tester.tap(find.byType(example.ClickableTooltipWidget));
    await tester.pump();
    expect(find.text(tooltipText), findsNothing);
  });

  testWidgets('Tooltip is shown at the right location', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OverlayPortalExampleApp());
    await tester.tap(find.byType(example.ClickableTooltipWidget));
    await tester.pump();

    final Size canvasSize = tester.getSize(find.byType(example.OverlayPortalExampleApp));
    expect(tester.getBottomRight(find.text(tooltipText)), canvasSize - const Size(50, 50));
  });

  testWidgets('Tooltip is shown with the right font size', (WidgetTester tester) async {
    await tester.pumpWidget(const example.OverlayPortalExampleApp());
    await tester.tap(find.byType(example.ClickableTooltipWidget));
    await tester.pump();

    final TextSpan textSpan =
        tester.renderObject<RenderParagraph>(find.text(tooltipText)).text as TextSpan;
    expect(textSpan.style?.fontSize, 50);
  });
}
