// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

const Color kSelectedColor = Color(0xFF00FF00);
const Color kUnselectedColor = Colors.transparent;

Widget buildFrame(
  TabController tabController, {
  Color? color,
  Color? selectedColor,
  double indicatorSize = 12.0,
  BorderStyle? borderStyle,
}) {
  return Localizations(
    locale: const Locale('en', 'US'),
    delegates: const <LocalizationsDelegate<dynamic>>[
      DefaultMaterialLocalizations.delegate,
      DefaultWidgetsLocalizations.delegate,
    ],
    child: Directionality(
      textDirection: TextDirection.ltr,
      child: Theme(
        data: ThemeData(colorScheme: const ColorScheme.light().copyWith(secondary: kSelectedColor)),
        child: SizedBox.expand(
          child: Center(
            child: SizedBox(
              width: 400.0,
              height: 400.0,
              child: Column(
                children: <Widget>[
                  TabPageSelector(
                    controller: tabController,
                    color: color,
                    selectedColor: selectedColor,
                    indicatorSize: indicatorSize,
                    borderStyle: borderStyle,
                  ),
                  Flexible(
                    child: TabBarView(
                      controller: tabController,
                      children: const <Widget>[
                        Center(child: Text('0')),
                        Center(child: Text('1')),
                        Center(child: Text('2')),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    ),
  );
}

List<Color> indicatorColors(WidgetTester tester) {
  final Iterable<TabPageSelectorIndicator> indicators = tester.widgetList(
    find.descendant(
      of: find.byType(TabPageSelector),
      matching: find.byType(TabPageSelectorIndicator),
    ),
  );
  return indicators
      .map<Color>((TabPageSelectorIndicator indicator) => indicator.backgroundColor)
      .toList();
}

void main() {
  testWidgets('PageSelector responds correctly to setting the TabController index', (
    WidgetTester tester,
  ) async {
    final TabController tabController = TabController(vsync: const TestVSync(), length: 3);
    addTearDown(tabController.dispose);
    await tester.pumpWidget(buildFrame(tabController));

    expect(tabController.index, 0);
    expect(indicatorColors(tester), const <Color>[
      kSelectedColor,
      kUnselectedColor,
      kUnselectedColor,
    ]);

    tabController.index = 1;
    await tester.pump();
    expect(tabController.index, 1);
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kSelectedColor,
      kUnselectedColor,
    ]);

    tabController.index = 2;
    await tester.pump();
    expect(tabController.index, 2);
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kUnselectedColor,
      kSelectedColor,
    ]);
  });

  testWidgets('PageSelector responds correctly to TabController.animateTo()', (
    WidgetTester tester,
  ) async {
    final TabController tabController = TabController(vsync: const TestVSync(), length: 3);
    addTearDown(tabController.dispose);
    await tester.pumpWidget(buildFrame(tabController));

    expect(tabController.index, 0);
    expect(indicatorColors(tester), const <Color>[
      kSelectedColor,
      kUnselectedColor,
      kUnselectedColor,
    ]);

    tabController.animateTo(1, duration: const Duration(milliseconds: 200));
    await tester.pump();
    // Verify that indicator 0's color is becoming increasingly transparent,
    // and indicator 1's color is becoming increasingly opaque during the
    // 200ms animation. Indicator 2 remains transparent throughout.
    await tester.pump(const Duration(milliseconds: 10));
    List<Color> colors = indicatorColors(tester);
    expect(colors[0].alpha, greaterThan(colors[1].alpha));
    expect(colors[2], kUnselectedColor);
    await tester.pump(const Duration(milliseconds: 175));
    colors = indicatorColors(tester);
    expect(colors[0].alpha, lessThan(colors[1].alpha));
    expect(colors[2], kUnselectedColor);
    await tester.pumpAndSettle();
    expect(tabController.index, 1);
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kSelectedColor,
      kUnselectedColor,
    ]);

    tabController.animateTo(2, duration: const Duration(milliseconds: 200));
    await tester.pump();
    // Same animation test as above for indicators 1 and 2.
    await tester.pump(const Duration(milliseconds: 10));
    colors = indicatorColors(tester);
    expect(colors[1].alpha, greaterThan(colors[2].alpha));
    expect(colors[0], kUnselectedColor);
    await tester.pump(const Duration(milliseconds: 175));
    colors = indicatorColors(tester);
    expect(colors[1].alpha, lessThan(colors[2].alpha));
    expect(colors[0], kUnselectedColor);
    await tester.pumpAndSettle();
    expect(tabController.index, 2);
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kUnselectedColor,
      kSelectedColor,
    ]);
  });

  testWidgets('PageSelector responds correctly to TabBarView drags', (WidgetTester tester) async {
    final TabController tabController = TabController(
      vsync: const TestVSync(),
      initialIndex: 1,
      length: 3,
    );
    addTearDown(tabController.dispose);
    await tester.pumpWidget(buildFrame(tabController));

    expect(tabController.index, 1);
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kSelectedColor,
      kUnselectedColor,
    ]);

    final TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));

    // Drag to the left moving the selection towards indicator 2. Indicator 2's
    // opacity should increase and Indicator 1's opacity should decrease.
    await gesture.moveBy(const Offset(-100.0, 0.0));
    await tester.pumpAndSettle();
    List<Color> colors = indicatorColors(tester);
    expect(colors[1].alpha, greaterThan(colors[2].alpha));
    expect(colors[0], kUnselectedColor);

    // Drag back to where we started.
    await gesture.moveBy(const Offset(100.0, 0.0));
    await tester.pumpAndSettle();
    colors = indicatorColors(tester);
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kSelectedColor,
      kUnselectedColor,
    ]);

    // Drag to the left moving the selection towards indicator 0. Indicator 0's
    // opacity should increase and Indicator 1's opacity should decrease.
    await gesture.moveBy(const Offset(100.0, 0.0));
    await tester.pumpAndSettle();
    colors = indicatorColors(tester);
    expect(colors[1].alpha, greaterThan(colors[0].alpha));
    expect(colors[2], kUnselectedColor);

    // Drag back to where we started.
    await gesture.moveBy(const Offset(-100.0, 0.0));
    await tester.pumpAndSettle();
    colors = indicatorColors(tester);
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kSelectedColor,
      kUnselectedColor,
    ]);

    // Completing the gesture doesn't change anything
    await gesture.up();
    await tester.pumpAndSettle();
    colors = indicatorColors(tester);
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kSelectedColor,
      kUnselectedColor,
    ]);

    // Fling to the left, selects indicator 2
    await tester.fling(find.byType(TabBarView), const Offset(-100.0, 0.0), 1000.0);
    await tester.pumpAndSettle();
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kUnselectedColor,
      kSelectedColor,
    ]);

    // Fling to the right, selects indicator 1
    await tester.fling(find.byType(TabBarView), const Offset(100.0, 0.0), 1000.0);
    await tester.pumpAndSettle();
    expect(indicatorColors(tester), const <Color>[
      kUnselectedColor,
      kSelectedColor,
      kUnselectedColor,
    ]);
  });

  testWidgets('PageSelector indicatorColors', (WidgetTester tester) async {
    const Color kRed = Color(0xFFFF0000);
    const Color kBlue = Color(0xFF0000FF);

    final TabController tabController = TabController(
      vsync: const TestVSync(),
      initialIndex: 1,
      length: 3,
    );
    addTearDown(tabController.dispose);
    await tester.pumpWidget(buildFrame(tabController, color: kRed, selectedColor: kBlue));

    expect(tabController.index, 1);
    expect(indicatorColors(tester), const <Color>[kRed, kBlue, kRed]);

    tabController.index = 0;
    await tester.pumpAndSettle();
    expect(indicatorColors(tester), const <Color>[kBlue, kRed, kRed]);
  });

  testWidgets('PageSelector indicatorSize', (WidgetTester tester) async {
    final TabController tabController = TabController(
      vsync: const TestVSync(),
      initialIndex: 1,
      length: 3,
    );
    addTearDown(tabController.dispose);
    await tester.pumpWidget(buildFrame(tabController, indicatorSize: 16.0));

    final Iterable<Element> indicatorElements =
        find
            .descendant(
              of: find.byType(TabPageSelector),
              matching: find.byType(TabPageSelectorIndicator),
            )
            .evaluate();

    // Indicators get an 8 pixel margin, 16 + 8 = 24.
    for (final Element indicatorElement in indicatorElements) {
      expect(indicatorElement.size, const Size(24.0, 24.0));
    }

    expect(tester.getSize(find.byType(TabPageSelector)).height, 24.0);
  });

  testWidgets('PageSelector circle border', (WidgetTester tester) async {
    final TabController tabController = TabController(
      vsync: const TestVSync(),
      initialIndex: 1,
      length: 3,
    );
    addTearDown(tabController.dispose);

    Iterable<TabPageSelectorIndicator> indicators;

    // Default border
    await tester.pumpWidget(buildFrame(tabController));
    indicators = tester.widgetList(
      find.descendant(
        of: find.byType(TabPageSelector),
        matching: find.byType(TabPageSelectorIndicator),
      ),
    );
    for (final TabPageSelectorIndicator indicator in indicators) {
      expect(indicator.borderStyle, BorderStyle.solid);
    }

    // No border
    await tester.pumpWidget(buildFrame(tabController, borderStyle: BorderStyle.none));
    indicators = tester.widgetList(
      find.descendant(
        of: find.byType(TabPageSelector),
        matching: find.byType(TabPageSelectorIndicator),
      ),
    );
    for (final TabPageSelectorIndicator indicator in indicators) {
      expect(indicator.borderStyle, BorderStyle.none);
    }

    // Solid border
    await tester.pumpWidget(buildFrame(tabController, borderStyle: BorderStyle.solid));
    indicators = tester.widgetList(
      find.descendant(
        of: find.byType(TabPageSelector),
        matching: find.byType(TabPageSelectorIndicator),
      ),
    );
    for (final TabPageSelectorIndicator indicator in indicators) {
      expect(indicator.borderStyle, BorderStyle.solid);
    }
  });

  testWidgets(
    'PageSelector responds correctly to TabController.animateTo() from the default tab controller',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        Localizations(
          locale: const Locale('en', 'US'),
          delegates: const <LocalizationsDelegate<dynamic>>[
            DefaultMaterialLocalizations.delegate,
            DefaultWidgetsLocalizations.delegate,
          ],
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Theme(
              data: ThemeData(
                colorScheme: const ColorScheme.light().copyWith(secondary: kSelectedColor),
              ),
              child: const SizedBox.expand(
                child: Center(
                  child: SizedBox(
                    width: 400.0,
                    height: 400.0,
                    child: DefaultTabController(
                      length: 3,
                      child: Column(
                        children: <Widget>[
                          TabPageSelector(),
                          Flexible(
                            child: TabBarView(
                              children: <Widget>[
                                Center(child: Text('0')),
                                Center(child: Text('1')),
                                Center(child: Text('2')),
                              ],
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      final TabController tabController = DefaultTabController.of(
        tester.element(find.byType(TabPageSelector)),
      );

      expect(tabController.index, 0);
      expect(indicatorColors(tester), const <Color>[
        kSelectedColor,
        kUnselectedColor,
        kUnselectedColor,
      ]);

      tabController.animateTo(1, duration: const Duration(milliseconds: 200));
      await tester.pump();
      // Verify that indicator 0's color is becoming increasingly transparent,
      // and indicator 1's color is becoming increasingly opaque during the
      // 200ms animation. Indicator 2 remains transparent throughout.
      await tester.pump(const Duration(milliseconds: 10));
      List<Color> colors = indicatorColors(tester);
      expect(colors[0].alpha, greaterThan(colors[1].alpha));
      expect(colors[2], kUnselectedColor);
      await tester.pump(const Duration(milliseconds: 175));
      colors = indicatorColors(tester);
      expect(colors[0].alpha, lessThan(colors[1].alpha));
      expect(colors[2], kUnselectedColor);
      await tester.pumpAndSettle();
      expect(tabController.index, 1);
      expect(indicatorColors(tester), const <Color>[
        kUnselectedColor,
        kSelectedColor,
        kUnselectedColor,
      ]);

      tabController.animateTo(2, duration: const Duration(milliseconds: 200));
      await tester.pump();
      // Same animation test as above for indicators 1 and 2.
      await tester.pump(const Duration(milliseconds: 10));
      colors = indicatorColors(tester);
      expect(colors[1].alpha, greaterThan(colors[2].alpha));
      expect(colors[0], kUnselectedColor);
      await tester.pump(const Duration(milliseconds: 175));
      colors = indicatorColors(tester);
      expect(colors[1].alpha, lessThan(colors[2].alpha));
      expect(colors[0], kUnselectedColor);
      await tester.pumpAndSettle();
      expect(tabController.index, 2);
      expect(indicatorColors(tester), const <Color>[
        kUnselectedColor,
        kUnselectedColor,
        kSelectedColor,
      ]);
    },
  );
}
