// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';
import '../widgets/semantics_tester.dart';

Future<void> pumpWidgetWithBoilerplate(WidgetTester tester, Widget widget) async {
  await tester.pumpWidget(
    Localizations(
      locale: const Locale('en', 'US'),
      delegates: const <LocalizationsDelegate<dynamic>>[
        DefaultWidgetsLocalizations.delegate,
        DefaultCupertinoLocalizations.delegate,
      ],
      child: Directionality(textDirection: TextDirection.ltr, child: widget),
    ),
  );
}

Future<void> main() async {
  testWidgets('Need at least 2 tabs', (WidgetTester tester) async {
    await expectLater(
      () => pumpWidgetWithBoilerplate(
        tester,
        CupertinoTabBar(
          items: <BottomNavigationBarItem>[
            BottomNavigationBarItem(
              icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
              label: 'Tab 1',
            ),
          ],
        ),
      ),
      throwsA(
        isAssertionError.having(
          (AssertionError error) => error.toString(),
          '.toString()',
          contains('items.length'),
        ),
      ),
    );
  });

  testWidgets('Active and inactive colors', (WidgetTester tester) async {
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
          currentIndex: 1,
          activeColor: const Color(0xFF123456),
          inactiveColor: const Color(0xFF654321),
        ),
      ),
    );

    final RichText actualInactive = tester.widget(
      find.descendant(of: find.text('Tab 1'), matching: find.byType(RichText)),
    );
    expect(actualInactive.text.style!.color, const Color(0xFF654321));

    final RichText actualActive = tester.widget(
      find.descendant(of: find.text('Tab 2'), matching: find.byType(RichText)),
    );
    expect(actualActive.text.style!.color, const Color(0xFF123456));
  });

  testWidgets('BottomNavigationBar.label will create a text widget', (WidgetTester tester) async {
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
          currentIndex: 1,
        ),
      ),
    );

    expect(find.text('Tab 1'), findsOneWidget);
    expect(find.text('Tab 2'), findsOneWidget);
  });

  testWidgets('Active and inactive colors dark mode', (WidgetTester tester) async {
    const CupertinoDynamicColor dynamicActiveColor = CupertinoDynamicColor.withBrightness(
      color: Color(0xFF000000),
      darkColor: Color(0xFF000001),
    );

    const CupertinoDynamicColor dynamicInactiveColor = CupertinoDynamicColor.withBrightness(
      color: Color(0xFF000002),
      darkColor: Color(0xFF000003),
    );

    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
          currentIndex: 1,
          activeColor: dynamicActiveColor,
          inactiveColor: dynamicInactiveColor,
        ),
      ),
    );

    RichText actualInactive = tester.widget(
      find.descendant(of: find.text('Tab 1'), matching: find.byType(RichText)),
    );
    expect(actualInactive.text.style!.color!.value, 0xFF000002);

    RichText actualActive = tester.widget(
      find.descendant(of: find.text('Tab 2'), matching: find.byType(RichText)),
    );
    expect(actualActive.text.style!.color!.value, 0xFF000000);

    final RenderDecoratedBox renderDecoratedBox = tester.renderObject(
      find.descendant(of: find.byType(BackdropFilter), matching: find.byType(DecoratedBox)),
    );

    // Border color is resolved correctly.
    final BoxDecoration decoration1 = renderDecoratedBox.decoration as BoxDecoration;
    expect(decoration1.border!.top.color.value, 0x4D000000);

    // Switch to dark mode.
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(platformBrightness: Brightness.dark),
        child: CupertinoTabBar(
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
          currentIndex: 1,
          activeColor: dynamicActiveColor,
          inactiveColor: dynamicInactiveColor,
        ),
      ),
    );

    actualInactive = tester.widget(
      find.descendant(of: find.text('Tab 1'), matching: find.byType(RichText)),
    );
    expect(actualInactive.text.style!.color!.value, 0xFF000003);

    actualActive = tester.widget(
      find.descendant(of: find.text('Tab 2'), matching: find.byType(RichText)),
    );
    expect(actualActive.text.style!.color!.value, 0xFF000001);

    // Border color is resolved correctly.
    final BoxDecoration decoration2 = renderDecoratedBox.decoration as BoxDecoration;
    expect(decoration2.border!.top.color.value, 0x29000000);
  });

  testWidgets('Tabs respects themes', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabBar(
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
          currentIndex: 1,
        ),
      ),
    );

    RichText actualInactive = tester.widget(
      find.descendant(of: find.text('Tab 1'), matching: find.byType(RichText)),
    );
    expect(actualInactive.text.style!.color!.value, 0xFF999999);

    RichText actualActive = tester.widget(
      find.descendant(of: find.text('Tab 2'), matching: find.byType(RichText)),
    );
    expect(actualActive.text.style!.color, CupertinoColors.activeBlue);

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: CupertinoTabBar(
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
          currentIndex: 1,
        ),
      ),
    );

    actualInactive = tester.widget(
      find.descendant(of: find.text('Tab 1'), matching: find.byType(RichText)),
    );
    expect(actualInactive.text.style!.color!.value, 0xFF757575);

    actualActive = tester.widget(
      find.descendant(of: find.text('Tab 2'), matching: find.byType(RichText)),
    );

    expect(actualActive.text.style!.color, isSameColorAs(CupertinoColors.activeBlue.darkColor));
  });

  testWidgets('Use active icon', (WidgetTester tester) async {
    final MemoryImage activeIcon = MemoryImage(Uint8List.fromList(kBlueSquarePng));
    final MemoryImage inactiveIcon = MemoryImage(Uint8List.fromList(kTransparentImage));

    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
          items: <BottomNavigationBarItem>[
            BottomNavigationBarItem(
              icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
              label: 'Tab 1',
            ),
            BottomNavigationBarItem(
              icon: ImageIcon(inactiveIcon),
              activeIcon: ImageIcon(activeIcon),
              label: 'Tab 2',
            ),
          ],
          currentIndex: 1,
          activeColor: const Color(0xFF123456),
          inactiveColor: const Color(0xFF654321),
        ),
      ),
    );

    final Image image = tester.widget(
      find.descendant(
        of: find.widgetWithText(GestureDetector, 'Tab 2'),
        matching: find.byType(Image),
      ),
    );

    expect(image.color, const Color(0xFF123456));
    expect(image.image, activeIcon);
  });

  testWidgets('Adjusts height to account for bottom padding', (WidgetTester tester) async {
    final CupertinoTabBar tabBar = CupertinoTabBar(
      items: <BottomNavigationBarItem>[
        BottomNavigationBarItem(
          icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
          label: 'Aka',
        ),
        BottomNavigationBarItem(
          icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
          label: 'Shiro',
        ),
      ],
    );

    // Verify height with no bottom padding.
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabScaffold(
          tabBar: tabBar,
          tabBuilder: (BuildContext context, int index) {
            return const Placeholder();
          },
        ),
      ),
    );
    expect(tester.getSize(find.byType(CupertinoTabBar)).height, 50.0);

    // Verify height with bottom padding.
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(viewPadding: EdgeInsets.only(bottom: 40.0)),
        child: CupertinoTabScaffold(
          tabBar: tabBar,
          tabBuilder: (BuildContext context, int index) {
            return const Placeholder();
          },
        ),
      ),
    );
    expect(tester.getSize(find.byType(CupertinoTabBar)).height, 90.0);
  });

  testWidgets('Set custom height', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/51704
    const double tabBarHeight = 56.0;
    final CupertinoTabBar tabBar = CupertinoTabBar(
      height: tabBarHeight,
      items: <BottomNavigationBarItem>[
        BottomNavigationBarItem(
          icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
          label: 'Aka',
        ),
        BottomNavigationBarItem(
          icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
          label: 'Shiro',
        ),
      ],
    );

    // Verify height with no bottom padding.
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabScaffold(
          tabBar: tabBar,
          tabBuilder: (BuildContext context, int index) {
            return const Placeholder();
          },
        ),
      ),
    );
    expect(tester.getSize(find.byType(CupertinoTabBar)).height, tabBarHeight);

    // Verify height with bottom padding.
    const double bottomPadding = 40.0;
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(viewPadding: EdgeInsets.only(bottom: bottomPadding)),
        child: CupertinoTabScaffold(
          tabBar: tabBar,
          tabBuilder: (BuildContext context, int index) {
            return const Placeholder();
          },
        ),
      ),
    );
    expect(tester.getSize(find.byType(CupertinoTabBar)).height, tabBarHeight + bottomPadding);
  });

  testWidgets('Ensure bar height will not change when toggle keyboard', (
    WidgetTester tester,
  ) async {
    const double tabBarHeight = 56.0;
    final CupertinoTabBar tabBar = CupertinoTabBar(
      height: tabBarHeight,
      items: <BottomNavigationBarItem>[
        BottomNavigationBarItem(
          icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
          label: 'Aka',
        ),
        BottomNavigationBarItem(
          icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
          label: 'Shiro',
        ),
      ],
    );

    const double bottomPadding = 34.0;

    // Test the height is correct when keyboard not showing.
    // So viewInset should be 0.0.
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(
          padding: EdgeInsets.only(bottom: bottomPadding),
          viewPadding: EdgeInsets.only(bottom: bottomPadding),
        ),
        child: CupertinoTabScaffold(
          tabBar: tabBar,
          tabBuilder: (BuildContext context, int index) {
            return const Placeholder();
          },
        ),
      ),
    );
    expect(tester.getSize(find.byType(CupertinoTabBar)).height, tabBarHeight + bottomPadding);

    // Now show keyboard, and test the bar height will not change.
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(
          viewPadding: EdgeInsets.only(bottom: bottomPadding),
          viewInsets: EdgeInsets.only(bottom: 336.0),
        ),
        child: CupertinoTabScaffold(
          tabBar: tabBar,
          tabBuilder: (BuildContext context, int index) {
            return const Placeholder();
          },
        ),
      ),
    );

    // Expect the bar height should not change.
    expect(tester.getSize(find.byType(CupertinoTabBar)).height, tabBarHeight + bottomPadding);
  });

  testWidgets('Opaque background does not add blur effects', (WidgetTester tester) async {
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
      ),
    );

    expect(find.byType(BackdropFilter), findsOneWidget);

    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
          backgroundColor: const Color(0xFFFFFFFF), // Opaque white.
        ),
      ),
    );

    expect(find.byType(BackdropFilter), findsNothing);
  });

  testWidgets('Tap callback', (WidgetTester tester) async {
    late int callbackTab;

    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
          currentIndex: 1,
          onTap: (int tab) {
            callbackTab = tab;
          },
        ),
      ),
    );

    await tester.tap(find.text('Tab 1'));
    expect(callbackTab, 0);

    await tester.tap(find.text('Tab 2'));
    expect(callbackTab, 1);
  });

  testWidgets('tabs announce semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        label: 'Tab 1',
        hint: 'Tab 1 of 2',
        flags: <SemanticsFlag>[SemanticsFlag.hasSelectedState, SemanticsFlag.isSelected],
        textDirection: TextDirection.ltr,
      ),
    );

    expect(
      semantics,
      includesNodeWith(label: 'Tab 2', hint: 'Tab 2 of 2', textDirection: TextDirection.ltr),
    );

    semantics.dispose();
  });

  testWidgets('Label of items should be nullable', (WidgetTester tester) async {
    final MemoryImage iconProvider = MemoryImage(Uint8List.fromList(kTransparentImage));
    final List<int> itemsTapped = <int>[];

    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
          items: <BottomNavigationBarItem>[
            BottomNavigationBarItem(
              icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
              label: 'Tab 1',
            ),
            BottomNavigationBarItem(icon: ImageIcon(iconProvider)),
          ],
          onTap: (int index) => itemsTapped.add(index),
        ),
      ),
    );

    expect(find.text('Tab 1'), findsOneWidget);

    final Finder finder = find.byWidgetPredicate(
      (Widget widget) => widget is Image && widget.image == iconProvider,
    );

    await tester.tap(finder);
    expect(itemsTapped, <int>[1]);
  });

  testWidgets('Hide border hides the top border of the tabBar', (WidgetTester tester) async {
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
      ),
    );

    final DecoratedBox decoratedBox = tester.widget(find.byType(DecoratedBox));
    final BoxDecoration boxDecoration = decoratedBox.decoration as BoxDecoration;
    expect(boxDecoration.border, isNotNull);

    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoTabBar(
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
          backgroundColor: const Color(0xFFFFFFFF), // Opaque white.
          border: null,
        ),
      ),
    );

    final DecoratedBox decoratedBoxHiddenBorder = tester.widget(find.byType(DecoratedBox));
    final BoxDecoration boxDecorationHiddenBorder =
        decoratedBoxHiddenBorder.decoration as BoxDecoration;
    expect(boxDecorationHiddenBorder.border, isNull);
  });

  testWidgets('Hovering over tab bar item updates cursor to clickable on Web', (
    WidgetTester tester,
  ) async {
    await pumpWidgetWithBoilerplate(
      tester,
      MediaQuery(
        data: const MediaQueryData(),
        child: Center(
          child: CupertinoTabBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(CupertinoIcons.alarm), label: 'Tab 1'),
              BottomNavigationBarItem(icon: Icon(CupertinoIcons.app_badge), label: 'Tab 2'),
            ],
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: const Offset(10, 10));
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    final Offset tabItem = tester.getCenter(find.text('Tab 1'));
    await gesture.moveTo(tabItem);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
    );
  });
}
