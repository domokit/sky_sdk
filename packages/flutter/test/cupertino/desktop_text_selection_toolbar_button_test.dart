// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('can press', (WidgetTester tester) async {
    bool pressed = false;
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDesktopTextSelectionToolbarButton(
            onPressed: () {
              pressed = true;
            },
            child: const Text('Tap me'),
          ),
        ),
      ),
    );

    expect(pressed, false);

    await tester.tap(find.byType(CupertinoDesktopTextSelectionToolbarButton));
    expect(pressed, true);
  });

  testWidgets('keeps contrast with background on hover', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDesktopTextSelectionToolbarButton.text(text: 'Tap me', onPressed: () {}),
        ),
      ),
    );

    final BuildContext context = tester.element(
      find.byType(CupertinoDesktopTextSelectionToolbarButton),
    );

    // The Text color is a CupertinoDynamicColor so we have to compare the color
    // values instead of just comparing the colors themselves.
    expect(
      (tester.firstWidget(find.text('Tap me')) as Text).style!.color!.value,
      CupertinoColors.black.value,
    );

    // Hover gesture
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    addTearDown(gesture.removePointer);
    await tester.pump();
    await gesture.moveTo(tester.getCenter(find.byType(CupertinoDesktopTextSelectionToolbarButton)));
    await tester.pumpAndSettle();

    // The color here should be a standard Color, there's no need to use value.
    expect(
      (tester.firstWidget(find.text('Tap me')) as Text).style!.color,
      CupertinoTheme.of(context).primaryContrastingColor,
    );
  });

  testWidgets('pressedOpacity defaults to 0.1', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoDesktopTextSelectionToolbarButton(
            onPressed: () {},
            child: const Text('Tap me'),
          ),
        ),
      ),
    );

    // Original at full opacity.
    FadeTransition opacity = tester.widget(
      find.descendant(
        of: find.byType(CupertinoDesktopTextSelectionToolbarButton),
        matching: find.byType(FadeTransition),
      ),
    );
    expect(opacity.opacity.value, 1.0);

    // Make a "down" gesture on the button.
    final Offset center = tester.getCenter(find.byType(CupertinoDesktopTextSelectionToolbarButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    // Opacity reduces during the down gesture.
    opacity = tester.widget(
      find.descendant(
        of: find.byType(CupertinoDesktopTextSelectionToolbarButton),
        matching: find.byType(FadeTransition),
      ),
    );
    expect(opacity.opacity.value, 0.7);

    // Release the down gesture.
    await gesture.up();
    await tester.pumpAndSettle();

    // Opacity is back to normal.
    opacity = tester.widget(
      find.descendant(
        of: find.byType(CupertinoDesktopTextSelectionToolbarButton),
        matching: find.byType(FadeTransition),
      ),
    );
    expect(opacity.opacity.value, 1.0);
  });

  testWidgets('passing null to onPressed disables the button', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoDesktopTextSelectionToolbarButton(onPressed: null, child: Text('Tap me')),
        ),
      ),
    );

    expect(find.byType(CupertinoButton), findsOneWidget);
    final CupertinoButton button = tester.widget(find.byType(CupertinoButton));
    expect(button.enabled, isFalse);
  });
}
