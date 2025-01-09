// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ButtonBar default control smoketest', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(textDirection: TextDirection.ltr, child: ButtonBar()),
    );
  });

  group('alignment', () {
    testWidgets('default alignment is MainAxisAlignment.end', (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(home: ButtonBar(children: <Widget>[SizedBox(width: 10.0, height: 10.0)])),
      );

      final Finder child = find.byType(SizedBox);
      // Should be positioned to the right of the bar,
      expect(tester.getRect(child).left, 782.0); // bar width - default padding - 10
      expect(tester.getRect(child).right, 792.0); // bar width - default padding
    });

    testWidgets('ButtonBarTheme.alignment overrides default', (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: ButtonBarTheme(
            data: ButtonBarThemeData(alignment: MainAxisAlignment.center),
            child: ButtonBar(children: <Widget>[SizedBox(width: 10.0, height: 10.0)]),
          ),
        ),
      );

      final Finder child = find.byType(SizedBox);
      // Should be positioned in the center
      expect(tester.getRect(child).left, 395.0); // (bar width - padding) / 2 - 10 / 2
      expect(tester.getRect(child).right, 405.0); // (bar width - padding) / 2 - 10 / 2 + 10
    });

    testWidgets('ButtonBar.alignment overrides ButtonBarTheme.alignment and default', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: ButtonBarTheme(
            data: ButtonBarThemeData(alignment: MainAxisAlignment.center),
            child: ButtonBar(
              alignment: MainAxisAlignment.start,
              children: <Widget>[SizedBox(width: 10.0, height: 10.0)],
            ),
          ),
        ),
      );

      final Finder child = find.byType(SizedBox);
      // Should be positioned on the left
      expect(tester.getRect(child).left, 8.0); // padding
      expect(tester.getRect(child).right, 18.0); // padding + 10
    });
  });

  group('mainAxisSize', () {
    testWidgets('Default mainAxisSize is MainAxisSize.max', (WidgetTester tester) async {
      const Key buttonBarKey = Key('row');
      const Key child0Key = Key('child0');
      const Key child1Key = Key('child1');
      const Key child2Key = Key('child2');

      await tester.pumpWidget(
        const MaterialApp(
          home: Center(
            child: ButtonBar(
              key: buttonBarKey,
              // buttonPadding set to zero to simplify test calculations.
              buttonPadding: EdgeInsets.zero,
              children: <Widget>[
                SizedBox(key: child0Key, width: 100.0, height: 100.0),
                SizedBox(key: child1Key, width: 100.0, height: 100.0),
                SizedBox(key: child2Key, width: 100.0, height: 100.0),
              ],
            ),
          ),
        ),
      );

      // ButtonBar should take up all the space it is provided by its parent.
      final Rect buttonBarRect = tester.getRect(find.byKey(buttonBarKey));
      expect(buttonBarRect.size.width, equals(800.0));
      expect(buttonBarRect.size.height, equals(100.0));

      // The children of [ButtonBar] are aligned by [MainAxisAlignment.end] by
      // default.
      Rect childRect;
      childRect = tester.getRect(find.byKey(child0Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      expect(childRect.right, 800.0 - 200.0);

      childRect = tester.getRect(find.byKey(child1Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      expect(childRect.right, 800.0 - 100.0);

      childRect = tester.getRect(find.byKey(child2Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      expect(childRect.right, 800.0);
    });

    testWidgets('ButtonBarTheme.mainAxisSize overrides default', (WidgetTester tester) async {
      const Key buttonBarKey = Key('row');
      const Key child0Key = Key('child0');
      const Key child1Key = Key('child1');
      const Key child2Key = Key('child2');
      await tester.pumpWidget(
        const MaterialApp(
          home: ButtonBarTheme(
            data: ButtonBarThemeData(mainAxisSize: MainAxisSize.min),
            child: Center(
              child: ButtonBar(
                key: buttonBarKey,
                // buttonPadding set to zero to simplify test calculations.
                buttonPadding: EdgeInsets.zero,
                children: <Widget>[
                  SizedBox(key: child0Key, width: 100.0, height: 100.0),
                  SizedBox(key: child1Key, width: 100.0, height: 100.0),
                  SizedBox(key: child2Key, width: 100.0, height: 100.0),
                ],
              ),
            ),
          ),
        ),
      );

      // ButtonBar should take up minimum space it requires.
      final Rect buttonBarRect = tester.getRect(find.byKey(buttonBarKey));
      expect(buttonBarRect.size.width, equals(300.0));
      expect(buttonBarRect.size.height, equals(100.0));

      Rect childRect;
      childRect = tester.getRect(find.byKey(child0Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      // Should be a center aligned because of [Center] widget.
      // First child is on the left side of the button bar.
      expect(childRect.left, (800.0 - buttonBarRect.width) / 2.0);

      childRect = tester.getRect(find.byKey(child1Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      // Should be a center aligned because of [Center] widget.
      // Second child is on the center the button bar.
      expect(childRect.left, ((800.0 - buttonBarRect.width) / 2.0) + 100.0);

      childRect = tester.getRect(find.byKey(child2Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      // Should be a center aligned because of [Center] widget.
      // Third child is on the right side of the button bar.
      expect(childRect.left, ((800.0 - buttonBarRect.width) / 2.0) + 200.0);
    });

    testWidgets('ButtonBar.mainAxisSize overrides ButtonBarTheme.mainAxisSize and default', (
      WidgetTester tester,
    ) async {
      const Key buttonBarKey = Key('row');
      const Key child0Key = Key('child0');
      const Key child1Key = Key('child1');
      const Key child2Key = Key('child2');
      await tester.pumpWidget(
        const MaterialApp(
          home: ButtonBarTheme(
            data: ButtonBarThemeData(mainAxisSize: MainAxisSize.min),
            child: Center(
              child: ButtonBar(
                key: buttonBarKey,
                // buttonPadding set to zero to simplify test calculations.
                buttonPadding: EdgeInsets.zero,
                mainAxisSize: MainAxisSize.max,
                children: <Widget>[
                  SizedBox(key: child0Key, width: 100.0, height: 100.0),
                  SizedBox(key: child1Key, width: 100.0, height: 100.0),
                  SizedBox(key: child2Key, width: 100.0, height: 100.0),
                ],
              ),
            ),
          ),
        ),
      );

      // ButtonBar should take up all the space it is provided by its parent.
      final Rect buttonBarRect = tester.getRect(find.byKey(buttonBarKey));
      expect(buttonBarRect.size.width, equals(800.0));
      expect(buttonBarRect.size.height, equals(100.0));

      // The children of [ButtonBar] are aligned by [MainAxisAlignment.end] by
      // default.
      Rect childRect;
      childRect = tester.getRect(find.byKey(child0Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      expect(childRect.right, 800.0 - 200.0);

      childRect = tester.getRect(find.byKey(child1Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      expect(childRect.right, 800.0 - 100.0);

      childRect = tester.getRect(find.byKey(child2Key));
      expect(childRect.size.width, equals(100.0));
      expect(childRect.size.height, equals(100.0));
      expect(childRect.right, 800.0);
    });
  });

  group('button properties override ButtonTheme', () {
    testWidgets('default button properties override ButtonTheme properties', (
      WidgetTester tester,
    ) async {
      late BuildContext capturedContext;
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            children: <Widget>[
              Builder(
                builder: (BuildContext context) {
                  capturedContext = context;
                  return Container();
                },
              ),
            ],
          ),
        ),
      );
      final ButtonThemeData buttonTheme = ButtonTheme.of(capturedContext);
      expect(buttonTheme.textTheme, equals(ButtonTextTheme.primary));
      expect(buttonTheme.minWidth, equals(64.0));
      expect(buttonTheme.height, equals(36.0));
      expect(buttonTheme.padding, equals(const EdgeInsets.symmetric(horizontal: 8.0)));
      expect(buttonTheme.alignedDropdown, equals(false));
      expect(buttonTheme.layoutBehavior, equals(ButtonBarLayoutBehavior.padded));
    });

    testWidgets('ButtonBarTheme button properties override defaults and ButtonTheme properties', (
      WidgetTester tester,
    ) async {
      late BuildContext capturedContext;
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBarTheme(
            data: const ButtonBarThemeData(
              buttonTextTheme: ButtonTextTheme.primary,
              buttonMinWidth: 42.0,
              buttonHeight: 84.0,
              buttonPadding: EdgeInsets.fromLTRB(10, 20, 30, 40),
              buttonAlignedDropdown: true,
              layoutBehavior: ButtonBarLayoutBehavior.constrained,
            ),
            child: ButtonBar(
              children: <Widget>[
                Builder(
                  builder: (BuildContext context) {
                    capturedContext = context;
                    return Container();
                  },
                ),
              ],
            ),
          ),
        ),
      );
      final ButtonThemeData buttonTheme = ButtonTheme.of(capturedContext);
      expect(buttonTheme.textTheme, equals(ButtonTextTheme.primary));
      expect(buttonTheme.minWidth, equals(42.0));
      expect(buttonTheme.height, equals(84.0));
      expect(buttonTheme.padding, equals(const EdgeInsets.fromLTRB(10, 20, 30, 40)));
      expect(buttonTheme.alignedDropdown, equals(true));
      expect(buttonTheme.layoutBehavior, equals(ButtonBarLayoutBehavior.constrained));
    });

    testWidgets(
      'ButtonBar button properties override ButtonBarTheme, defaults and ButtonTheme properties',
      (WidgetTester tester) async {
        late BuildContext capturedContext;
        await tester.pumpWidget(
          MaterialApp(
            home: ButtonBarTheme(
              data: const ButtonBarThemeData(
                buttonTextTheme: ButtonTextTheme.accent,
                buttonMinWidth: 4242.0,
                buttonHeight: 8484.0,
                buttonPadding: EdgeInsets.fromLTRB(50, 60, 70, 80),
                buttonAlignedDropdown: false,
                layoutBehavior: ButtonBarLayoutBehavior.padded,
              ),
              child: ButtonBar(
                buttonTextTheme: ButtonTextTheme.primary,
                buttonMinWidth: 42.0,
                buttonHeight: 84.0,
                buttonPadding: const EdgeInsets.fromLTRB(10, 20, 30, 40),
                buttonAlignedDropdown: true,
                layoutBehavior: ButtonBarLayoutBehavior.constrained,
                children: <Widget>[
                  Builder(
                    builder: (BuildContext context) {
                      capturedContext = context;
                      return Container();
                    },
                  ),
                ],
              ),
            ),
          ),
        );
        final ButtonThemeData buttonTheme = ButtonTheme.of(capturedContext);
        expect(buttonTheme.textTheme, equals(ButtonTextTheme.primary));
        expect(buttonTheme.minWidth, equals(42.0));
        expect(buttonTheme.height, equals(84.0));
        expect(buttonTheme.padding, equals(const EdgeInsets.fromLTRB(10, 20, 30, 40)));
        expect(buttonTheme.alignedDropdown, equals(true));
        expect(buttonTheme.layoutBehavior, equals(ButtonBarLayoutBehavior.constrained));
      },
    );
  });

  group('layoutBehavior', () {
    testWidgets('ButtonBar has a min height of 52 when using ButtonBarLayoutBehavior.constrained', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        const SingleChildScrollView(
          child: ListBody(
            children: <Widget>[
              Directionality(
                textDirection: TextDirection.ltr,
                child: ButtonBar(
                  layoutBehavior: ButtonBarLayoutBehavior.constrained,
                  children: <Widget>[SizedBox(width: 10.0, height: 10.0)],
                ),
              ),
            ],
          ),
        ),
      );

      final Finder buttonBar = find.byType(ButtonBar);
      expect(tester.getBottomRight(buttonBar).dy - tester.getTopRight(buttonBar).dy, 52.0);
    });

    testWidgets('ButtonBar has padding applied when using ButtonBarLayoutBehavior.padded', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        const SingleChildScrollView(
          child: ListBody(
            children: <Widget>[
              Directionality(
                textDirection: TextDirection.ltr,
                child: ButtonBar(
                  layoutBehavior: ButtonBarLayoutBehavior.padded,
                  children: <Widget>[SizedBox(width: 10.0, height: 10.0)],
                ),
              ),
            ],
          ),
        ),
      );

      final Finder buttonBar = find.byType(ButtonBar);
      expect(tester.getBottomRight(buttonBar).dy - tester.getTopRight(buttonBar).dy, 26.0);
    });
  });

  group("ButtonBar's children wrap when they overflow horizontally", () {
    testWidgets("ButtonBar's children wrap when buttons overflow", (WidgetTester tester) async {
      final Key keyOne = UniqueKey();
      final Key keyTwo = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 800.0),
              SizedBox(key: keyTwo, height: 50.0, width: 800.0),
            ],
          ),
        ),
      );

      // Second [Container] should wrap around to the next column since
      // they take up max width constraint.
      final Rect containerOneRect = tester.getRect(find.byKey(keyOne));
      final Rect containerTwoRect = tester.getRect(find.byKey(keyTwo));
      expect(containerOneRect.bottom, containerTwoRect.top);
      expect(containerOneRect.left, containerTwoRect.left);
    });

    testWidgets("ButtonBar's children overflow defaults - MainAxisAlignment.end", (
      WidgetTester tester,
    ) async {
      final Key keyOne = UniqueKey();
      final Key keyTwo = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            // Set padding to zero to align buttons with edge of button bar.
            buttonPadding: EdgeInsets.zero,
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 500.0),
              SizedBox(key: keyTwo, height: 50.0, width: 500.0),
            ],
          ),
        ),
      );

      final Rect buttonBarRect = tester.getRect(find.byType(ButtonBar));
      final Rect containerOneRect = tester.getRect(find.byKey(keyOne));
      final Rect containerTwoRect = tester.getRect(find.byKey(keyTwo));
      // Second [Container] should wrap around to the next row.
      expect(containerOneRect.bottom, containerTwoRect.top);
      // Second [Container] should align to the start of the ButtonBar.
      expect(containerOneRect.right, containerTwoRect.right);
      expect(containerOneRect.right, buttonBarRect.right);
    });

    testWidgets("ButtonBar's children overflow - MainAxisAlignment.start", (
      WidgetTester tester,
    ) async {
      final Key keyOne = UniqueKey();
      final Key keyTwo = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            alignment: MainAxisAlignment.start,
            // Set padding to zero to align buttons with edge of button bar.
            buttonPadding: EdgeInsets.zero,
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 500.0),
              SizedBox(key: keyTwo, height: 50.0, width: 500.0),
            ],
          ),
        ),
      );

      final Rect buttonBarRect = tester.getRect(find.byType(ButtonBar));
      final Rect containerOneRect = tester.getRect(find.byKey(keyOne));
      final Rect containerTwoRect = tester.getRect(find.byKey(keyTwo));
      // Second [Container] should wrap around to the next row.
      expect(containerOneRect.bottom, containerTwoRect.top);
      // [Container]s should align to the end of the ButtonBar.
      expect(containerOneRect.left, containerTwoRect.left);
      expect(containerOneRect.left, buttonBarRect.left);
    });

    testWidgets("ButtonBar's children overflow - MainAxisAlignment.center", (
      WidgetTester tester,
    ) async {
      final Key keyOne = UniqueKey();
      final Key keyTwo = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            alignment: MainAxisAlignment.center,
            // Set padding to zero to align buttons with edge of button bar.
            buttonPadding: EdgeInsets.zero,
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 500.0),
              SizedBox(key: keyTwo, height: 50.0, width: 500.0),
            ],
          ),
        ),
      );

      final Rect buttonBarRect = tester.getRect(find.byType(ButtonBar));
      final Rect containerOneRect = tester.getRect(find.byKey(keyOne));
      final Rect containerTwoRect = tester.getRect(find.byKey(keyTwo));
      // Second [Container] should wrap around to the next row.
      expect(containerOneRect.bottom, containerTwoRect.top);
      // [Container]s should center themselves in the ButtonBar.
      expect(containerOneRect.center.dx, containerTwoRect.center.dx);
      expect(containerOneRect.center.dx, buttonBarRect.center.dx);
    });

    testWidgets("ButtonBar's children default to MainAxisAlignment.start for horizontal "
        'alignment when overflowing in spaceBetween, spaceAround and spaceEvenly '
        'cases when overflowing.', (WidgetTester tester) async {
      final Key keyOne = UniqueKey();
      final Key keyTwo = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            alignment: MainAxisAlignment.spaceEvenly,
            // Set padding to zero to align buttons with edge of button bar.
            buttonPadding: EdgeInsets.zero,
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 500.0),
              SizedBox(key: keyTwo, height: 50.0, width: 500.0),
            ],
          ),
        ),
      );

      Rect buttonBarRect = tester.getRect(find.byType(ButtonBar));
      Rect containerOneRect = tester.getRect(find.byKey(keyOne));
      Rect containerTwoRect = tester.getRect(find.byKey(keyTwo));
      // Second [Container] should wrap around to the next row.
      expect(containerOneRect.bottom, containerTwoRect.top);
      // Should align horizontally to the start of the button bar.
      expect(containerOneRect.left, containerTwoRect.left);
      expect(containerOneRect.left, buttonBarRect.left);

      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            alignment: MainAxisAlignment.spaceAround,
            // Set padding to zero to align buttons with edge of button bar.
            buttonPadding: EdgeInsets.zero,
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 500.0),
              SizedBox(key: keyTwo, height: 50.0, width: 500.0),
            ],
          ),
        ),
      );

      buttonBarRect = tester.getRect(find.byType(ButtonBar));
      containerOneRect = tester.getRect(find.byKey(keyOne));
      containerTwoRect = tester.getRect(find.byKey(keyTwo));
      // Second [Container] should wrap around to the next row.
      expect(containerOneRect.bottom, containerTwoRect.top);
      // Should align horizontally to the start of the button bar.
      expect(containerOneRect.left, containerTwoRect.left);
      expect(containerOneRect.left, buttonBarRect.left);
    });

    testWidgets("ButtonBar's children respects verticalDirection when overflowing", (
      WidgetTester tester,
    ) async {
      final Key keyOne = UniqueKey();
      final Key keyTwo = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            alignment: MainAxisAlignment.center,
            // Set padding to zero to align buttons with edge of button bar.
            buttonPadding: EdgeInsets.zero,
            // Set the vertical direction to start from the bottom and lay
            // out upwards.
            overflowDirection: VerticalDirection.up,
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 500.0),
              SizedBox(key: keyTwo, height: 50.0, width: 500.0),
            ],
          ),
        ),
      );

      final Rect containerOneRect = tester.getRect(find.byKey(keyOne));
      final Rect containerTwoRect = tester.getRect(find.byKey(keyTwo));
      // Second [Container] should appear above first container.
      expect(containerTwoRect.bottom, lessThanOrEqualTo(containerOneRect.top));
    });

    testWidgets('ButtonBar has no spacing by default when overflowing', (
      WidgetTester tester,
    ) async {
      final Key keyOne = UniqueKey();
      final Key keyTwo = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            alignment: MainAxisAlignment.center,
            // Set padding to zero to align buttons with edge of button bar.
            buttonPadding: EdgeInsets.zero,
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 500.0),
              SizedBox(key: keyTwo, height: 50.0, width: 500.0),
            ],
          ),
        ),
      );

      final Rect containerOneRect = tester.getRect(find.byKey(keyOne));
      final Rect containerTwoRect = tester.getRect(find.byKey(keyTwo));
      expect(containerOneRect.bottom, containerTwoRect.top);
    });

    testWidgets("ButtonBar's children respects overflowButtonSpacing when overflowing", (
      WidgetTester tester,
    ) async {
      final Key keyOne = UniqueKey();
      final Key keyTwo = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: ButtonBar(
            alignment: MainAxisAlignment.center,
            // Set padding to zero to align buttons with edge of button bar.
            buttonPadding: EdgeInsets.zero,
            // Set the overflow button spacing to ensure add some space between
            // buttons in an overflow case.
            overflowButtonSpacing: 10.0,
            children: <Widget>[
              SizedBox(key: keyOne, height: 50.0, width: 500.0),
              SizedBox(key: keyTwo, height: 50.0, width: 500.0),
            ],
          ),
        ),
      );

      final Rect containerOneRect = tester.getRect(find.byKey(keyOne));
      final Rect containerTwoRect = tester.getRect(find.byKey(keyTwo));
      expect(containerOneRect.bottom, containerTwoRect.top - 10.0);
    });
  });

  testWidgets('_RenderButtonBarRow.constraints does not work before layout', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(home: ButtonBar()),
      duration: Duration.zero,
      phase: EnginePhase.build,
    );

    final Finder buttonBar = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_ButtonBarRow',
    );
    final RenderBox renderButtonBar = tester.renderObject(buttonBar) as RenderBox;

    expect(renderButtonBar.debugNeedsLayout, isTrue);
    expect(() => renderButtonBar.constraints, throwsStateError);
  });
}
