// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class TestRoute extends PageRouteBuilder<void> {
  TestRoute(Widget child)
    : super(pageBuilder: (BuildContext _, Animation<double> __, Animation<double> ___) => child);
}

class IconTextBox extends StatelessWidget {
  const IconTextBox(this.text, {super.key});
  final String text;
  @override
  Widget build(BuildContext context) {
    return Container(
      alignment: Alignment.center,
      child: Row(children: <Widget>[const Icon(IconData(0x41, fontFamily: 'Roboto')), Text(text)]),
    );
  }
}

void main() {
  testWidgets('InheritedTheme.captureAll()', (WidgetTester tester) async {
    const double fontSize = 32;
    const double iconSize = 48;
    const Color textColor = Color(0xFF00FF00);
    const Color iconColor = Color(0xFF0000FF);
    bool useCaptureAll = false;
    late BuildContext navigatorContext;

    Widget buildFrame() {
      return WidgetsApp(
        color: const Color(0xFFFFFFFF),
        onGenerateRoute: (RouteSettings settings) {
          return TestRoute(
            // The outer DefaultTextStyle and IconTheme widgets must have
            // no effect on the test because InheritedTheme.captureAll()
            // is required to only save the closest InheritedTheme ancestors.
            DefaultTextStyle(
              style: const TextStyle(fontSize: iconSize, color: iconColor),
              child: IconTheme(
                data: const IconThemeData(size: fontSize, color: textColor),
                // The inner DefaultTextStyle and IconTheme widgets define
                // InheritedThemes that captureAll() will wrap() around
                // TestRoute's IconTextBox child.
                child: DefaultTextStyle(
                  style: const TextStyle(fontSize: fontSize, color: textColor),
                  child: IconTheme(
                    data: const IconThemeData(size: iconSize, color: iconColor),
                    child: Builder(
                      builder: (BuildContext context) {
                        return GestureDetector(
                          behavior: HitTestBehavior.opaque,
                          onTap: () {
                            navigatorContext = context;
                            Navigator.of(context).push(
                              TestRoute(
                                useCaptureAll
                                    ? InheritedTheme.captureAll(context, const IconTextBox('Hello'))
                                    : const IconTextBox('Hello'),
                              ),
                            );
                          },
                          child: const IconTextBox('Tap'),
                        );
                      },
                    ),
                  ),
                ),
              ),
            ),
          );
        },
      );
    }

    TextStyle getIconStyle() {
      return tester
          .widget<RichText>(find.descendant(of: find.byType(Icon), matching: find.byType(RichText)))
          .text
          .style!;
    }

    TextStyle getTextStyle(String text) {
      return tester
          .widget<RichText>(find.descendant(of: find.text(text), matching: find.byType(RichText)))
          .text
          .style!;
    }

    useCaptureAll = false;
    await tester.pumpWidget(buildFrame());
    expect(find.text('Tap'), findsOneWidget);
    expect(find.text('Hello'), findsNothing);
    expect(getTextStyle('Tap').color, textColor);
    expect(getTextStyle('Tap').fontSize, fontSize);
    expect(getIconStyle().color, iconColor);
    expect(getIconStyle().fontSize, iconSize);

    // Tap to show the TestRoute
    await tester.tap(find.text('Tap'));
    await tester.pumpAndSettle(); // route transition
    expect(find.text('Tap'), findsNothing);
    expect(find.text('Hello'), findsOneWidget);
    // The new route's text and icons will NOT inherit the DefaultTextStyle or
    // IconTheme values.
    expect(getTextStyle('Hello').color, isNot(textColor));
    expect(getTextStyle('Hello').fontSize, isNot(fontSize));
    expect(getIconStyle().color, isNot(iconColor));
    expect(getIconStyle().fontSize, isNot(iconSize));

    // Return to the home route
    useCaptureAll = true;
    Navigator.of(navigatorContext).pop();
    await tester.pumpAndSettle(); // route transition

    // Verify that all is the same as it was when the test started
    expect(find.text('Tap'), findsOneWidget);
    expect(find.text('Hello'), findsNothing);
    expect(getTextStyle('Tap').color, textColor);
    expect(getTextStyle('Tap').fontSize, fontSize);
    expect(getIconStyle().color, iconColor);
    expect(getIconStyle().fontSize, iconSize);

    // Tap to show the TestRoute. The test route's IconTextBox will have been
    // wrapped with InheritedTheme.captureAll().
    await tester.tap(find.text('Tap'));
    await tester.pumpAndSettle(); // route transition
    expect(find.text('Tap'), findsNothing);
    expect(find.text('Hello'), findsOneWidget);
    // The new route's text and icons will inherit the DefaultTextStyle or
    // IconTheme values because captureAll.
    expect(getTextStyle('Hello').color, textColor);
    expect(getTextStyle('Hello').fontSize, fontSize);
    expect(getIconStyle().color, iconColor);
    expect(getIconStyle().fontSize, iconSize);
  });

  testWidgets('InheritedTheme.captureAll() multiple IconTheme ancestors', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/39087

    const Color outerColor = Color(0xFF0000FF);
    const Color innerColor = Color(0xFF00FF00);
    const double iconSize = 48;
    final Key icon1 = UniqueKey();
    final Key icon2 = UniqueKey();

    await tester.pumpWidget(
      WidgetsApp(
        color: const Color(0xFFFFFFFF),
        onGenerateRoute: (RouteSettings settings) {
          return TestRoute(
            IconTheme(
              data: const IconThemeData(color: outerColor),
              child: IconTheme(
                data: const IconThemeData(size: iconSize, color: innerColor),
                child: Center(
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: <Widget>[
                      Icon(const IconData(0x41, fontFamily: 'Roboto'), key: icon1),
                      Builder(
                        builder: (BuildContext context) {
                          // The same IconThemes are visible from this context
                          // and the context that the widget returned by captureAll()
                          // is built in. So only the inner green IconTheme should
                          // apply to the icon, i.e. both icons will be big and green.
                          return InheritedTheme.captureAll(
                            context,
                            Icon(const IconData(0x41, fontFamily: 'Roboto'), key: icon2),
                          );
                        },
                      ),
                    ],
                  ),
                ),
              ),
            ),
          );
        },
      ),
    );

    TextStyle getIconStyle(Key key) {
      return tester
          .widget<RichText>(find.descendant(of: find.byKey(key), matching: find.byType(RichText)))
          .text
          .style!;
    }

    expect(getIconStyle(icon1).color, innerColor);
    expect(getIconStyle(icon1).fontSize, iconSize);
    expect(getIconStyle(icon2).color, innerColor);
    expect(getIconStyle(icon2).fontSize, iconSize);
  });

  testWidgets('InheritedTheme.captureAll() multiple DefaultTextStyle ancestors', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/39087

    const Color textColor = Color(0xFF00FF00);

    await tester.pumpWidget(
      WidgetsApp(
        color: const Color(0xFFFFFFFF),
        onGenerateRoute: (RouteSettings settings) {
          return TestRoute(
            DefaultTextStyle(
              style: const TextStyle(fontSize: 48),
              child: DefaultTextStyle(
                style: const TextStyle(color: textColor),
                child: Row(
                  children: <Widget>[
                    const Text('Hello'),
                    Builder(
                      builder: (BuildContext context) {
                        return InheritedTheme.captureAll(context, const Text('World'));
                      },
                    ),
                  ],
                ),
              ),
            ),
          );
        },
      ),
    );

    TextStyle getTextStyle(String text) {
      return tester
          .widget<RichText>(find.descendant(of: find.text(text), matching: find.byType(RichText)))
          .text
          .style!;
    }

    expect(getTextStyle('Hello').fontSize, null);
    expect(getTextStyle('Hello').color, textColor);
    expect(getTextStyle('World').fontSize, null);
    expect(getTextStyle('World').color, textColor);
  });
}
