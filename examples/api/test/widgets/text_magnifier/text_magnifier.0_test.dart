// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/text_magnifier/text_magnifier.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

List<TextSelectionPoint> _globalize(Iterable<TextSelectionPoint> points, RenderBox box) {
  return points.map<TextSelectionPoint>((TextSelectionPoint point) {
    return TextSelectionPoint(box.localToGlobal(point.point), point.direction);
  }).toList();
}

RenderEditable _findRenderEditable<T extends State<StatefulWidget>>(WidgetTester tester) {
  return (tester.state(find.byType(TextField)) as TextSelectionGestureDetectorBuilderDelegate)
      .editableTextKey
      .currentState!
      .renderEditable;
}

Offset _textOffsetToPosition<T extends State<StatefulWidget>>(WidgetTester tester, int offset) {
  final RenderEditable renderEditable = _findRenderEditable(tester);

  final List<TextSelectionPoint> endpoints =
      renderEditable
          .getEndpointsForSelection(TextSelection.collapsed(offset: offset))
          .map<TextSelectionPoint>(
            (TextSelectionPoint point) =>
                TextSelectionPoint(renderEditable.localToGlobal(point.point), point.direction),
          )
          .toList();

  return endpoints[0].point + const Offset(0.0, -2.0);
}

void main() {
  const Duration durationBetweenActions = Duration(milliseconds: 20);
  const String defaultText = 'I am a magnifier, fear me!';

  Future<void> showMagnifier(WidgetTester tester, int textOffset) async {
    assert(textOffset >= 0);
    final Offset tapOffset = _textOffsetToPosition(tester, textOffset);

    // Double tap 'Magnifier' word to show the selection handles.
    final TestGesture testGesture = await tester.startGesture(tapOffset);
    await tester.pump(durationBetweenActions);
    await testGesture.up();
    await tester.pump(durationBetweenActions);
    await testGesture.down(tapOffset);
    await tester.pump(durationBetweenActions);
    await testGesture.up();
    await tester.pumpAndSettle();

    final TextEditingController controller =
        tester.firstWidget<TextField>(find.byType(TextField)).controller!;

    final TextSelection selection = controller.selection;
    final RenderEditable renderEditable = _findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = _globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );

    final Offset handlePos = endpoints.last.point + const Offset(10.0, 10.0);

    final TestGesture gesture = await tester.startGesture(handlePos);

    await gesture.moveTo(_textOffsetToPosition(tester, defaultText.length - 2));
    await tester.pump();
  }

  testWidgets(
    'should show custom magnifier on drag',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.TextMagnifierExampleApp(text: defaultText));

      await showMagnifier(tester, defaultText.indexOf('e'));
      expect(find.byType(example.CustomMagnifier), findsOneWidget);

      await expectLater(
        find.byType(example.TextMagnifierExampleApp),
        matchesGoldenFile('text_magnifier.0_test.png'),
      );
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
    skip: true, // This image is flaky. https://github.com/flutter/flutter/issues/144350
  );

  testWidgets('should show custom magnifier in RTL', (WidgetTester tester) async {
    const String text = 'أثارت زر';
    const String textToTapOn = 'ت';

    await tester.pumpWidget(
      const example.TextMagnifierExampleApp(textDirection: TextDirection.rtl, text: text),
    );

    await showMagnifier(tester, text.indexOf(textToTapOn));

    expect(find.byType(example.CustomMagnifier), findsOneWidget);
  });
}
