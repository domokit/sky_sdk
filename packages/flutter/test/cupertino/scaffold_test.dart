// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:flutter/cupertino.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';

/// Integration tests testing both [CupertinoPageScaffold] and [CupertinoTabScaffold].
void main() {
  testWidgets('Contents are behind translucent bar', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: CupertinoPageScaffold(
          // Default nav bar is translucent.
          navigationBar: CupertinoNavigationBar(middle: Text('Title')),
          child: Center(),
        ),
      ),
    );

    expect(tester.getTopLeft(find.byType(Center)), Offset.zero);
  });

  testWidgets('Opaque bar pushes contents down', (WidgetTester tester) async {
    late BuildContext childContext;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(top: 20)),
          child: CupertinoPageScaffold(
            navigationBar: const CupertinoNavigationBar(
              middle: Text('Opaque'),
              backgroundColor: Color(0xFFF8F8F8),
            ),
            child: Builder(
              builder: (BuildContext context) {
                childContext = context;
                return Container();
              },
            ),
          ),
        ),
      ),
    );

    expect(MediaQuery.of(childContext).padding.top, 0);
    // The top of the [Container] is 44 px from the top of the screen because
    // it's pushed down by the opaque navigation bar whose height is 44 px,
    // and the 20 px [MediaQuery] top padding is fully absorbed by the navigation bar.
    expect(tester.getRect(find.byType(Container)), const Rect.fromLTRB(0, 44, 800, 600));
  });

  testWidgets('dark mode and obstruction work', (WidgetTester tester) async {
    const Color dynamicColor = CupertinoDynamicColor.withBrightness(
      color: Color(0xFFF8F8F8),
      darkColor: Color(0xEE333333),
    );

    const CupertinoDynamicColor backgroundColor = CupertinoDynamicColor.withBrightness(
      color: Color(0xFFFFFFFF),
      darkColor: Color(0xFF000000),
    );

    late BuildContext childContext;
    Widget scaffoldWithBrightness(Brightness brightness) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: MediaQueryData(
            platformBrightness: brightness,
            viewInsets: const EdgeInsets.only(top: 20),
          ),
          child: CupertinoPageScaffold(
            backgroundColor: backgroundColor,
            navigationBar: const CupertinoNavigationBar(
              middle: Text('Title'),
              backgroundColor: dynamicColor,
            ),
            child: Builder(
              builder: (BuildContext context) {
                childContext = context;
                return Container();
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(scaffoldWithBrightness(Brightness.light));

    expect(MediaQuery.of(childContext).padding.top, 0);
    expect(find.byType(CupertinoPageScaffold), paints..rect(color: backgroundColor.color));

    await tester.pumpWidget(scaffoldWithBrightness(Brightness.dark));

    expect(MediaQuery.of(childContext).padding.top, greaterThan(0));
    expect(find.byType(CupertinoPageScaffold), paints..rect(color: backgroundColor.darkColor));
  });

  testWidgets('Contents padding from viewInsets', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 100.0)),
          child: CupertinoPageScaffold(
            navigationBar: const CupertinoNavigationBar(
              middle: Text('Opaque'),
              backgroundColor: Color(0xFFF8F8F8),
            ),
            child: Container(),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(Container)).height, 600.0 - 44.0 - 100.0);

    late BuildContext childContext;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 100.0)),
          child: CupertinoPageScaffold(
            navigationBar: const CupertinoNavigationBar(middle: Text('Transparent')),
            child: Builder(
              builder: (BuildContext context) {
                childContext = context;
                return Container();
              },
            ),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(Container)).height, 600.0 - 100.0);
    // The shouldn't see a media query view inset because it was consumed by
    // the scaffold.
    expect(MediaQuery.of(childContext).viewInsets.bottom, 0);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 100.0)),
          child: CupertinoPageScaffold(
            navigationBar: const CupertinoNavigationBar(middle: Text('Title')),
            resizeToAvoidBottomInset: false,
            child: Container(),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(Container)).height, 600.0);
  });

  testWidgets(
    'Contents bottom padding are not consumed by viewInsets when resizeToAvoidBottomInset overridden',
    (WidgetTester tester) async {
      const Widget child = CupertinoPageScaffold(
        resizeToAvoidBottomInset: false,
        navigationBar: CupertinoNavigationBar(
          middle: Text('Opaque'),
          backgroundColor: Color(0xFFF8F8F8),
        ),
        child: Placeholder(),
      );

      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: MediaQueryData(viewInsets: EdgeInsets.only(bottom: 20.0)),
            child: child,
          ),
        ),
      );

      final Offset initialPoint = tester.getCenter(find.byType(Placeholder));
      // Consume bottom padding - as if by the keyboard opening
      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: MediaQueryData(
              viewPadding: EdgeInsets.only(bottom: 20),
              viewInsets: EdgeInsets.only(bottom: 300),
            ),
            child: child,
          ),
        ),
      );
      final Offset finalPoint = tester.getCenter(find.byType(Placeholder));
      expect(initialPoint, finalPoint);
    },
  );

  testWidgets('Contents are between opaque bars', (WidgetTester tester) async {
    const Center page1Center = Center();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(
            backgroundColor: CupertinoColors.white,
            items: <BottomNavigationBarItem>[
              BottomNavigationBarItem(
                icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
                label: 'Tab 1',
              ),
              BottomNavigationBarItem(
                icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
                label: 'Tab 2',
              ),
            ],
          ),
          tabBuilder: (BuildContext context, int index) {
            return index == 0
                ? const CupertinoPageScaffold(
                  navigationBar: CupertinoNavigationBar(
                    backgroundColor: CupertinoColors.white,
                    middle: Text('Title'),
                  ),
                  child: page1Center,
                )
                : const Stack();
          },
        ),
      ),
    );

    expect(tester.getSize(find.byWidget(page1Center)).height, 600.0 - 44.0 - 50.0);
  });

  testWidgets('Contents have automatic sliver padding between translucent bars', (
    WidgetTester tester,
  ) async {
    const SizedBox content = SizedBox(height: 600.0, width: 600.0);

    await tester.pumpWidget(
      CupertinoApp(
        home: MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.symmetric(vertical: 20.0)),
          child: CupertinoTabScaffold(
            tabBar: CupertinoTabBar(
              items: <BottomNavigationBarItem>[
                BottomNavigationBarItem(
                  icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
                  label: 'Tab 1',
                ),
                BottomNavigationBarItem(
                  icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
                  label: 'Tab 2',
                ),
              ],
            ),
            tabBuilder: (BuildContext context, int index) {
              return index == 0
                  ? CupertinoPageScaffold(
                    navigationBar: const CupertinoNavigationBar(middle: Text('Title')),
                    child: ListView(children: const <Widget>[content]),
                  )
                  : const Stack();
            },
          ),
        ),
      ),
    );

    // List content automatically padded by nav bar and top media query padding.
    expect(tester.getTopLeft(find.byWidget(content)).dy, 20.0 + 44.0);

    // Overscroll to the bottom.
    await tester.drag(
      find.byWidget(content),
      const Offset(0.0, -400.0),
      warnIfMissed: false,
    ); // can't be hit (it's empty) but we're aiming for the list really so it doesn't matter
    // Let it bounce back.
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    // List content automatically padded by tab bar and bottom media query padding.
    expect(tester.getBottomLeft(find.byWidget(content)).dy, 600 - 20.0 - 50.0);
  });

  testWidgets('iOS independent tab navigation', (WidgetTester tester) async {
    // A full on iOS information architecture app with 2 tabs, and 2 pages
    // in each with independent navigation states.
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(
            items: <BottomNavigationBarItem>[
              BottomNavigationBarItem(
                icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
                label: 'Tab 1',
              ),
              BottomNavigationBarItem(
                icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
                label: 'Tab 2',
              ),
            ],
          ),
          tabBuilder: (BuildContext context, int index) {
            // For 1-indexed readability.
            ++index;
            return CupertinoTabView(
              builder: (BuildContext context) {
                return CupertinoPageScaffold(
                  navigationBar: CupertinoNavigationBar(middle: Text('Page 1 of tab $index')),
                  child: Center(
                    child: CupertinoButton(
                      child: const Text('Next'),
                      onPressed: () {
                        Navigator.of(context).push(
                          CupertinoPageRoute<void>(
                            builder: (BuildContext context) {
                              return CupertinoPageScaffold(
                                navigationBar: CupertinoNavigationBar(
                                  middle: Text('Page 2 of tab $index'),
                                ),
                                child: Center(
                                  child: CupertinoButton(
                                    child: const Text('Back'),
                                    onPressed: () {
                                      Navigator.of(context).pop();
                                    },
                                  ),
                                ),
                              );
                            },
                          ),
                        );
                      },
                    ),
                  ),
                );
              },
            );
          },
        ),
      ),
    );

    expect(find.text('Page 1 of tab 1'), findsOneWidget);
    expect(find.text('Page 1 of tab 2'), findsNothing); // Lazy building so not built yet.

    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    expect(find.text('Page 1 of tab 1'), findsNothing); // It's offstage now.
    expect(find.text('Page 1 of tab 1', skipOffstage: false), findsOneWidget);
    expect(find.text('Page 1 of tab 2'), findsOneWidget);

    // Navigate in tab 2.
    await tester.tap(find.text('Next'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 600));

    expect(find.text('Page 2 of tab 2'), isOnstage);
    expect(find.text('Page 1 of tab 1', skipOffstage: false), isOffstage);

    await tester.tap(find.text('Tab 1'));
    await tester.pump();

    // Independent navigation stacks.
    expect(find.text('Page 1 of tab 1'), isOnstage);
    expect(find.text('Page 2 of tab 2', skipOffstage: false), isOffstage);

    // Navigate in tab 1.
    await tester.tap(find.text('Next'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 600));

    expect(find.text('Page 2 of tab 1'), isOnstage);
    expect(find.text('Page 2 of tab 2', skipOffstage: false), isOffstage);

    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    expect(find.text('Page 2 of tab 2'), isOnstage);
    expect(find.text('Page 2 of tab 1', skipOffstage: false), isOffstage);

    // Pop in tab 2
    await tester.tap(find.text('Back'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 600));

    expect(find.text('Page 1 of tab 2'), isOnstage);
    expect(find.text('Page 2 of tab 1', skipOffstage: false), isOffstage);
  });

  testWidgets('Decorated with white background by default', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: CupertinoPageScaffold(child: Center())));

    final DecoratedBox decoratedBox =
        tester.widgetList(find.byType(DecoratedBox)).elementAt(1) as DecoratedBox;
    expect(decoratedBox.decoration.runtimeType, BoxDecoration);

    final BoxDecoration decoration = decoratedBox.decoration as BoxDecoration;
    expect(decoration.color, isSameColorAs(CupertinoColors.white));
  });

  testWidgets('Overrides background color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: CupertinoPageScaffold(backgroundColor: Color(0xFF010203), child: Center()),
      ),
    );

    final DecoratedBox decoratedBox =
        tester.widgetList(find.byType(DecoratedBox)).elementAt(1) as DecoratedBox;
    expect(decoratedBox.decoration.runtimeType, BoxDecoration);

    final BoxDecoration decoration = decoratedBox.decoration as BoxDecoration;
    expect(decoration.color, const Color(0xFF010203));
  });

  testWidgets('Lists in CupertinoPageScaffold scroll to the top when status bar tapped', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        builder: (BuildContext context, Widget? child) {
          // Acts as a 20px status bar at the root of the app.
          return MediaQuery(
            data: MediaQuery.of(context).copyWith(padding: const EdgeInsets.only(top: 20)),
            child: child!,
          );
        },
        home: CupertinoPageScaffold(
          // Default nav bar is translucent.
          navigationBar: const CupertinoNavigationBar(middle: Text('Title')),
          child: ListView.builder(
            itemExtent: 50,
            itemBuilder: (BuildContext context, int index) => Text(index.toString()),
          ),
        ),
      ),
    );
    // Top media query padding 20 + translucent nav bar 44.
    expect(tester.getTopLeft(find.text('0')).dy, 64);
    expect(tester.getTopLeft(find.text('6')).dy, 364);

    await tester.fling(
      find.text('5'), // Find some random text on the screen.
      const Offset(0, -200),
      20,
    );

    await tester.pumpAndSettle();

    expect(tester.getTopLeft(find.text('6')).dy, moreOrLessEquals(166.833, epsilon: 0.1));
    expect(
      tester.getTopLeft(find.text('12')).dy,
      moreOrLessEquals(466.8333333333334, epsilon: 0.1),
    );

    // The media query top padding is 20. Tapping at 20 should do nothing.
    await tester.tapAt(const Offset(400, 20));
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(find.text('6')).dy, moreOrLessEquals(166.833, epsilon: 0.1));
    expect(
      tester.getTopLeft(find.text('12')).dy,
      moreOrLessEquals(466.8333333333334, epsilon: 0.1),
    );

    // Tap 1 pixel higher.
    await tester.tapAt(const Offset(400, 19));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));
    expect(tester.getTopLeft(find.text('0')).dy, 64);
    expect(tester.getTopLeft(find.text('6')).dy, 364);
    expect(find.text('12'), findsNothing);
  });

  testWidgets('resizeToAvoidBottomInset is supported even when no navigationBar', (
    WidgetTester tester,
  ) async {
    Widget buildFrame(bool showNavigationBar, bool showKeyboard) {
      return CupertinoApp(
        home: MediaQuery(
          data: MediaQueryData(
            viewPadding: const EdgeInsets.only(bottom: 20),
            viewInsets: EdgeInsets.only(bottom: showKeyboard ? 300 : 20),
          ),
          child: CupertinoPageScaffold(
            navigationBar:
                showNavigationBar ? const CupertinoNavigationBar(middle: Text('Title')) : null,
            child: Builder(
              builder:
                  (BuildContext context) => Center(
                    child: CupertinoTextField(
                      placeholder: MediaQuery.viewInsetsOf(context).toString(),
                    ),
                  ),
            ),
          ),
        ),
      );
    }

    // CupertinoPageScaffold should consume the viewInsets in all cases
    final String expectedViewInsets = EdgeInsets.zero.toString();

    // When there is a nav bar and no keyboard.
    await tester.pumpWidget(buildFrame(true, false));
    final Offset positionNoInsetWithNavBar = tester.getTopLeft(find.byType(CupertinoTextField));
    expect(
      (find.byType(CupertinoTextField).evaluate().first.widget as CupertinoTextField).placeholder,
      expectedViewInsets,
    );

    // When there is a nav bar and keyboard, the CupertinoTextField moves up.
    await tester.pumpWidget(buildFrame(true, true));
    await tester.pumpAndSettle();
    final Offset positionWithInsetWithNavBar = tester.getTopLeft(find.byType(CupertinoTextField));
    expect(positionWithInsetWithNavBar.dy, lessThan(positionNoInsetWithNavBar.dy));
    expect(
      (find.byType(CupertinoTextField).evaluate().first.widget as CupertinoTextField).placeholder,
      expectedViewInsets,
    );

    // When there is no nav bar and no keyboard, the CupertinoTextField is still
    // centered.
    await tester.pumpWidget(buildFrame(false, false));
    final Offset positionNoInsetNoNavBar = tester.getTopLeft(find.byType(CupertinoTextField));
    expect(positionNoInsetNoNavBar, equals(positionNoInsetWithNavBar));
    expect(
      (find.byType(CupertinoTextField).evaluate().first.widget as CupertinoTextField).placeholder,
      expectedViewInsets,
    );

    // When there is a keyboard but no nav bar, the CupertinoTextField also
    // moves up to the same position as when there is a keyboard and nav bar.
    await tester.pumpWidget(buildFrame(false, true));
    await tester.pumpAndSettle();
    final Offset positionWithInsetNoNavBar = tester.getTopLeft(find.byType(CupertinoTextField));
    expect(positionWithInsetNoNavBar.dy, lessThan(positionNoInsetNoNavBar.dy));
    expect(positionWithInsetNoNavBar, equals(positionWithInsetWithNavBar));
    expect(
      (find.byType(CupertinoTextField).evaluate().first.widget as CupertinoTextField).placeholder,
      expectedViewInsets,
    );
  });

  testWidgets('textScaleFactor is set to 1.0', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Builder(
          builder: (BuildContext context) {
            return MediaQuery.withClampedTextScaling(
              minScaleFactor: 99,
              maxScaleFactor: 99,
              child: const CupertinoPageScaffold(
                navigationBar: CupertinoNavigationBar(
                  middle: Text('middle'),
                  leading: Text('leading'),
                  trailing: Text('trailing'),
                ),
                child: Text('content'),
              ),
            );
          },
        ),
      ),
    );
    final Iterable<RichText> richTextList = tester.widgetList<RichText>(
      find.descendant(of: find.byType(CupertinoNavigationBar), matching: find.byType(RichText)),
    );

    expect(richTextList.length, greaterThan(0));
    expect(richTextList.any((RichText text) => text.textScaleFactor != 1), isFalse);

    expect(
      tester
          .widget<RichText>(
            find.descendant(of: find.text('content'), matching: find.byType(RichText)),
          )
          .textScaler,
      const TextScaler.linear(99.0),
    );
  });
}
