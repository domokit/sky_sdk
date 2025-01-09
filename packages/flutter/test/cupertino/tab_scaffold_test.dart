// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';
import '../rendering/rendering_tester.dart' show TestCallbackPainter;
import '../widgets/navigator_utils.dart';

late List<int> selectedTabs;

class MockCupertinoTabController extends CupertinoTabController {
  MockCupertinoTabController({required super.initialIndex});

  bool isDisposed = false;
  int numOfListeners = 0;

  @override
  void addListener(VoidCallback listener) {
    numOfListeners++;
    super.addListener(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    numOfListeners--;
    super.removeListener(listener);
  }

  @override
  void dispose() {
    isDisposed = true;
    super.dispose();
  }
}

void main() {
  setUp(() {
    selectedTabs = <int>[];
  });

  tearDown(() {
    imageCache.clear();
  });

  BottomNavigationBarItem tabGenerator(int index) {
    return BottomNavigationBarItem(
      icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
      label: 'Tab ${index + 1}',
    );
  }

  testWidgets('Tab switching', (WidgetTester tester) async {
    final List<int> tabsPainted = <int>[];

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          tabBuilder: (BuildContext context, int index) {
            return CustomPaint(
              painter: TestCallbackPainter(
                onPaint: () {
                  tabsPainted.add(index);
                },
              ),
              child: Text('Page ${index + 1}'),
            );
          },
        ),
      ),
    );

    expect(tabsPainted, const <int>[0]);
    RichText tab1 = tester.widget(
      find.descendant(of: find.text('Tab 1'), matching: find.byType(RichText)),
    );
    expect(tab1.text.style!.color, CupertinoColors.activeBlue);
    RichText tab2 = tester.widget(
      find.descendant(of: find.text('Tab 2'), matching: find.byType(RichText)),
    );
    expect(tab2.text.style!.color!.value, 0xFF999999);

    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    expect(tabsPainted, const <int>[0, 1]);
    tab1 = tester.widget(find.descendant(of: find.text('Tab 1'), matching: find.byType(RichText)));
    expect(tab1.text.style!.color!.value, 0xFF999999);
    tab2 = tester.widget(find.descendant(of: find.text('Tab 2'), matching: find.byType(RichText)));
    expect(tab2.text.style!.color, CupertinoColors.activeBlue);

    await tester.tap(find.text('Tab 1'));
    await tester.pump();

    expect(tabsPainted, const <int>[0, 1, 0]);
    // CupertinoTabBar's onTap callbacks are passed on.
    expect(selectedTabs, const <int>[1, 0]);
  });

  testWidgets('Tabs are lazy built and moved offstage when inactive', (WidgetTester tester) async {
    final List<int> tabsBuilt = <int>[];

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          tabBuilder: (BuildContext context, int index) {
            tabsBuilt.add(index);
            return Text('Page ${index + 1}');
          },
        ),
      ),
    );

    expect(tabsBuilt, const <int>[0]);
    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2'), findsNothing);

    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    // Both tabs are built but only one is onstage.
    expect(tabsBuilt, const <int>[0, 0, 1]);
    expect(find.text('Page 1', skipOffstage: false), isOffstage);
    expect(find.text('Page 2'), findsOneWidget);

    await tester.tap(find.text('Tab 1'));
    await tester.pump();

    expect(tabsBuilt, const <int>[0, 0, 1, 0, 1]);
    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2', skipOffstage: false), isOffstage);
  });

  testWidgets('Last tab gets focus', (WidgetTester tester) async {
    // 2 nodes for 2 tabs
    final List<FocusNode> focusNodes = <FocusNode>[
      FocusNode(debugLabel: 'Node 1'),
      FocusNode(debugLabel: 'Node 2'),
    ];
    for (final FocusNode focusNode in focusNodes) {
      addTearDown(focusNode.dispose);
    }

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          tabBuilder: (BuildContext context, int index) {
            return CupertinoTextField(focusNode: focusNodes[index], autofocus: true);
          },
        ),
      ),
    );

    expect(focusNodes[0].hasFocus, isTrue);

    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    expect(focusNodes[0].hasFocus, isFalse);
    expect(focusNodes[1].hasFocus, isTrue);

    await tester.tap(find.text('Tab 1'));
    await tester.pump();

    expect(focusNodes[0].hasFocus, isTrue);
    expect(focusNodes[1].hasFocus, isFalse);
  });

  testWidgets('Do not affect focus order in the route', (WidgetTester tester) async {
    final List<FocusNode> focusNodes = <FocusNode>[
      FocusNode(debugLabel: 'Node 1'),
      FocusNode(debugLabel: 'Node 2'),
      FocusNode(debugLabel: 'Node 3'),
      FocusNode(debugLabel: 'Node 4'),
    ];
    for (final FocusNode focusNode in focusNodes) {
      addTearDown(focusNode.dispose);
    }

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          tabBuilder: (BuildContext context, int index) {
            return Column(
              children: <Widget>[
                CupertinoTextField(focusNode: focusNodes[index * 2], placeholder: 'TextField 1'),
                CupertinoTextField(
                  focusNode: focusNodes[index * 2 + 1],
                  placeholder: 'TextField 2',
                ),
              ],
            );
          },
        ),
      ),
    );

    expect(focusNodes.any((FocusNode node) => node.hasFocus), isFalse);

    await tester.tap(find.widgetWithText(CupertinoTextField, 'TextField 2'));

    expect(focusNodes.indexOf(focusNodes.singleWhere((FocusNode node) => node.hasFocus)), 1);

    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    await tester.tap(find.widgetWithText(CupertinoTextField, 'TextField 1'));

    expect(focusNodes.indexOf(focusNodes.singleWhere((FocusNode node) => node.hasFocus)), 2);

    await tester.tap(find.text('Tab 1'));
    await tester.pump();

    // Upon going back to tab 1, the item it tab 1 that previously had the focus
    // (TextField 2) gets it back.
    expect(focusNodes.indexOf(focusNodes.singleWhere((FocusNode node) => node.hasFocus)), 1);
  });

  testWidgets('Programmatic tab switching by changing the index of an existing controller', (
    WidgetTester tester,
  ) async {
    final CupertinoTabController controller = CupertinoTabController(initialIndex: 1);
    addTearDown(controller.dispose);
    final List<int> tabsPainted = <int>[];

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          controller: controller,
          tabBuilder: (BuildContext context, int index) {
            return CustomPaint(
              painter: TestCallbackPainter(
                onPaint: () {
                  tabsPainted.add(index);
                },
              ),
              child: Text('Page ${index + 1}'),
            );
          },
        ),
      ),
    );

    expect(tabsPainted, const <int>[1]);

    controller.index = 0;
    await tester.pump();

    expect(tabsPainted, const <int>[1, 0]);
    // onTap is not called when changing tabs programmatically.
    expect(selectedTabs, isEmpty);

    // Can still tap out of the programmatically selected tab.
    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    expect(tabsPainted, const <int>[1, 0, 1]);
    expect(selectedTabs, const <int>[1]);
  });

  testWidgets('Programmatic tab switching by passing in a new controller', (
    WidgetTester tester,
  ) async {
    final List<int> tabsPainted = <int>[];

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          tabBuilder: (BuildContext context, int index) {
            return CustomPaint(
              painter: TestCallbackPainter(
                onPaint: () {
                  tabsPainted.add(index);
                },
              ),
              child: Text('Page ${index + 1}'),
            );
          },
        ),
      ),
    );

    expect(tabsPainted, const <int>[0]);

    final CupertinoTabController controller = CupertinoTabController(initialIndex: 1);
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          controller: controller, // Programmatically change the tab now.
          tabBuilder: (BuildContext context, int index) {
            return CustomPaint(
              painter: TestCallbackPainter(
                onPaint: () {
                  tabsPainted.add(index);
                },
              ),
              child: Text('Page ${index + 1}'),
            );
          },
        ),
      ),
    );

    expect(tabsPainted, const <int>[0, 1]);
    // onTap is not called when changing tabs programmatically.
    expect(selectedTabs, isEmpty);

    // Can still tap out of the programmatically selected tab.
    await tester.tap(find.text('Tab 1'));
    await tester.pump();

    expect(tabsPainted, const <int>[0, 1, 0]);
    expect(selectedTabs, const <int>[0]);
  });

  testWidgets('Tab bar respects themes', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          tabBuilder: (BuildContext context, int index) {
            return const Placeholder();
          },
        ),
      ),
    );

    BoxDecoration tabDecoration =
        tester
                .widget<DecoratedBox>(
                  find.descendant(
                    of: find.byType(CupertinoTabBar),
                    matching: find.byType(DecoratedBox),
                  ),
                )
                .decoration
            as BoxDecoration;

    expect(tabDecoration.color, isSameColorAs(const Color(0xF0F9F9F9))); // Inherited from theme.

    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    // Pump again but with dark theme.
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(
          brightness: Brightness.dark,
          primaryColor: CupertinoColors.destructiveRed,
        ),
        home: CupertinoTabScaffold(
          tabBar: _buildTabBar(),
          tabBuilder: (BuildContext context, int index) {
            return const Placeholder();
          },
        ),
      ),
    );

    tabDecoration =
        tester
                .widget<DecoratedBox>(
                  find.descendant(
                    of: find.byType(CupertinoTabBar),
                    matching: find.byType(DecoratedBox),
                  ),
                )
                .decoration
            as BoxDecoration;

    expect(tabDecoration.color, isSameColorAs(const Color(0xF01D1D1D)));

    final RichText tab1 = tester.widget(
      find.descendant(of: find.text('Tab 1'), matching: find.byType(RichText)),
    );
    // Tab 2 should still be selected after changing theme.
    expect(tab1.text.style!.color!.value, 0xFF757575);
    final RichText tab2 = tester.widget(
      find.descendant(of: find.text('Tab 2'), matching: find.byType(RichText)),
    );
    expect(tab2.text.style!.color, isSameColorAs(CupertinoColors.systemRed.darkColor));
  });

  testWidgets('Tab contents are padded when there are view insets', (WidgetTester tester) async {
    late BuildContext innerContext;

    await tester.pumpWidget(
      CupertinoApp(
        home: MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 200)),
          child: CupertinoTabScaffold(
            tabBar: _buildTabBar(),
            tabBuilder: (BuildContext context, int index) {
              innerContext = context;
              return const Placeholder();
            },
          ),
        ),
      ),
    );

    expect(tester.getRect(find.byType(Placeholder)), const Rect.fromLTWH(0, 0, 800, 400));
    // Don't generate more media query padding from the translucent bottom
    // tab since the tab is behind the keyboard now.
    expect(MediaQuery.of(innerContext).padding.bottom, 0);
  });

  testWidgets('Tab contents are not inset when resizeToAvoidBottomInset overridden', (
    WidgetTester tester,
  ) async {
    late BuildContext innerContext;

    await tester.pumpWidget(
      CupertinoApp(
        home: MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 200)),
          child: CupertinoTabScaffold(
            resizeToAvoidBottomInset: false,
            tabBar: _buildTabBar(),
            tabBuilder: (BuildContext context, int index) {
              innerContext = context;
              return const Placeholder();
            },
          ),
        ),
      ),
    );

    expect(tester.getRect(find.byType(Placeholder)), const Rect.fromLTWH(0, 0, 800, 600));
    // Media query padding shows up in the inner content because it wasn't masked
    // by the view inset.
    expect(MediaQuery.of(innerContext).padding.bottom, 50);
  });

  testWidgets(
    'Tab contents bottom padding are not consumed by viewInsets when resizeToAvoidBottomInset overridden',
    (WidgetTester tester) async {
      final Widget child = Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultWidgetsLocalizations.delegate,
          DefaultCupertinoLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: CupertinoTabScaffold(
            resizeToAvoidBottomInset: false,
            tabBar: _buildTabBar(),
            tabBuilder: (BuildContext context, int index) {
              return const Placeholder();
            },
          ),
        ),
      );

      await tester.pumpWidget(
        CupertinoApp(
          home: MediaQuery(
            data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 20.0)),
            child: child,
          ),
        ),
      );

      final Offset initialPoint = tester.getCenter(find.byType(Placeholder));

      // Consume bottom padding - as if by the keyboard opening
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: 20),
            viewInsets: EdgeInsets.only(bottom: 300),
          ),
          child: child,
        ),
      );

      final Offset finalPoint = tester.getCenter(find.byType(Placeholder));

      expect(initialPoint, finalPoint);
    },
  );

  testWidgets('Opaque tab bar consumes bottom padding while non opaque tab bar does not', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/43581.
    Future<EdgeInsets> getContentPaddingWithTabBarColor(Color color) async {
      late EdgeInsets contentPadding;

      await tester.pumpWidget(
        CupertinoApp(
          home: MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.only(bottom: 50)),
            child: CupertinoTabScaffold(
              tabBar: CupertinoTabBar(
                backgroundColor: color,
                items: List<BottomNavigationBarItem>.generate(2, tabGenerator),
              ),
              tabBuilder: (BuildContext context, int index) {
                contentPadding = MediaQuery.paddingOf(context);
                return const Placeholder();
              },
            ),
          ),
        ),
      );
      return contentPadding;
    }

    expect(await getContentPaddingWithTabBarColor(const Color(0xAAFFFFFF)), isNot(EdgeInsets.zero));
    expect(await getContentPaddingWithTabBarColor(const Color(0xFFFFFFFF)), EdgeInsets.zero);
  });

  testWidgets('Tab and page scaffolds do not double stack view insets', (
    WidgetTester tester,
  ) async {
    late BuildContext innerContext;

    await tester.pumpWidget(
      CupertinoApp(
        home: MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 200)),
          child: CupertinoTabScaffold(
            tabBar: _buildTabBar(),
            tabBuilder: (BuildContext context, int index) {
              return CupertinoPageScaffold(
                child: Builder(
                  builder: (BuildContext context) {
                    innerContext = context;
                    return const Placeholder();
                  },
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(tester.getRect(find.byType(Placeholder)), const Rect.fromLTWH(0, 0, 800, 400));
    expect(MediaQuery.of(innerContext).padding.bottom, 0);
  });

  testWidgets('Deleting tabs after selecting them should switch to the last available tab', (
    WidgetTester tester,
  ) async {
    final List<int> tabsBuilt = <int>[];

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(
            items: List<BottomNavigationBarItem>.generate(4, tabGenerator),
            onTap: (int newTab) => selectedTabs.add(newTab),
          ),
          tabBuilder: (BuildContext context, int index) {
            tabsBuilt.add(index);
            return Text('Page ${index + 1}');
          },
        ),
      ),
    );

    expect(tabsBuilt, const <int>[0]);
    // selectedTabs list is appended to on onTap callbacks. We didn't tap
    // any tabs yet.
    expect(selectedTabs, const <int>[]);
    tabsBuilt.clear();

    await tester.tap(find.text('Tab 4'));
    await tester.pump();

    // Tabs 1 and 4 are built but only one is onstage.
    expect(tabsBuilt, const <int>[0, 3]);
    expect(selectedTabs, const <int>[3]);
    expect(find.text('Page 1', skipOffstage: false), isOffstage);
    expect(find.text('Page 4'), findsOneWidget);
    tabsBuilt.clear();

    // Delete 2 tabs while Page 4 is still selected.
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(
            items: List<BottomNavigationBarItem>.generate(2, tabGenerator),
            onTap: (int newTab) => selectedTabs.add(newTab),
          ),
          tabBuilder: (BuildContext context, int index) {
            tabsBuilt.add(index);
            // Change the builder too.
            return Text('Different page ${index + 1}');
          },
        ),
      ),
    );

    expect(tabsBuilt, const <int>[0, 1]);
    // We didn't tap on any additional tabs to invoke the onTap callback. We
    // just deleted a tab.
    expect(selectedTabs, const <int>[3]);
    // Tab 1 was previously built so it's rebuilt again, albeit offstage.
    expect(find.text('Different page 1', skipOffstage: false), isOffstage);
    // Since all the tabs after tab 2 are deleted, tab 2 is now the last tab and
    // the actively shown tab.
    expect(find.text('Different page 2'), findsOneWidget);
    // No more tab 4 since it's deleted.
    expect(find.text('Different page 4', skipOffstage: false), findsNothing);
    // We also changed the builder so no tabs should be built with the old
    // builder.
    expect(find.text('Page 1', skipOffstage: false), findsNothing);
    expect(find.text('Page 2', skipOffstage: false), findsNothing);
    expect(find.text('Page 4', skipOffstage: false), findsNothing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/33455
  testWidgets('Adding new tabs does not crash the app', (WidgetTester tester) async {
    final List<int> tabsPainted = <int>[];
    final CupertinoTabController controller = CupertinoTabController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(10, tabGenerator)),
          controller: controller,
          tabBuilder: (BuildContext context, int index) {
            return CustomPaint(
              painter: TestCallbackPainter(
                onPaint: () {
                  tabsPainted.add(index);
                },
              ),
              child: Text('Page ${index + 1}'),
            );
          },
        ),
      ),
    );

    expect(tabsPainted, const <int>[0]);

    // Increase the num of tabs to 20.
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(20, tabGenerator)),
          controller: controller,
          tabBuilder: (BuildContext context, int index) {
            return CustomPaint(
              painter: TestCallbackPainter(
                onPaint: () {
                  tabsPainted.add(index);
                },
              ),
              child: Text('Page ${index + 1}'),
            );
          },
        ),
      ),
    );

    expect(tabsPainted, const <int>[0, 0]);

    await tester.tap(find.text('Tab 19'));
    await tester.pump();

    // Tapping the tabs should still work.
    expect(tabsPainted, const <int>[0, 0, 18]);
  });

  testWidgets('If a controller is initially provided then the parent stops doing so for rebuilds, '
      'a new instance of CupertinoTabController should be created and used by the widget, '
      "while preserving the previous controller's tab index", (WidgetTester tester) async {
    final List<int> tabsPainted = <int>[];
    final CupertinoTabController oldController = CupertinoTabController();
    addTearDown(oldController.dispose);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(10, tabGenerator)),
          controller: oldController,
          tabBuilder: (BuildContext context, int index) {
            return CustomPaint(
              painter: TestCallbackPainter(
                onPaint: () {
                  tabsPainted.add(index);
                },
              ),
              child: Text('Page ${index + 1}'),
            );
          },
        ),
      ),
    );

    expect(tabsPainted, const <int>[0]);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(10, tabGenerator)),
          tabBuilder: (BuildContext context, int index) {
            return CustomPaint(
              painter: TestCallbackPainter(
                onPaint: () {
                  tabsPainted.add(index);
                },
              ),
              child: Text('Page ${index + 1}'),
            );
          },
        ),
      ),
    );

    expect(tabsPainted, const <int>[0, 0]);

    await tester.tap(find.text('Tab 2'));
    await tester.pump();

    // Tapping the tabs should still work.
    expect(tabsPainted, const <int>[0, 0, 1]);

    oldController.index = 10;
    await tester.pump();

    // Changing [index] of the oldController should not work.
    expect(tabsPainted, const <int>[0, 0, 1]);
  });

  testWidgets('Do not call dispose on a controller that we do not own '
      'but do remove from its listeners when done listening to it', (WidgetTester tester) async {
    final MockCupertinoTabController mockController = MockCupertinoTabController(initialIndex: 0);
    addTearDown(mockController.dispose);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(2, tabGenerator)),
          controller: mockController,
          tabBuilder: (BuildContext context, int index) => const Placeholder(),
        ),
      ),
    );

    expect(mockController.numOfListeners, 1);
    expect(mockController.isDisposed, isFalse);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(2, tabGenerator)),
          tabBuilder: (BuildContext context, int index) => const Placeholder(),
        ),
      ),
    );

    expect(mockController.numOfListeners, 0);
    expect(mockController.isDisposed, isFalse);
  });

  testWidgets('The owner can dispose the old controller', (WidgetTester tester) async {
    CupertinoTabController controller = CupertinoTabController(initialIndex: 2);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(3, tabGenerator)),
          controller: controller,
          tabBuilder: (BuildContext context, int index) => const Placeholder(),
        ),
      ),
    );
    expect(find.text('Tab 1'), findsOneWidget);
    expect(find.text('Tab 2'), findsOneWidget);
    expect(find.text('Tab 3'), findsOneWidget);

    controller.dispose();
    controller = CupertinoTabController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(2, tabGenerator)),
          controller: controller,
          tabBuilder: (BuildContext context, int index) => const Placeholder(),
        ),
      ),
    );

    // Should not crash here.
    expect(find.text('Tab 1'), findsOneWidget);
    expect(find.text('Tab 2'), findsOneWidget);
    expect(find.text('Tab 3'), findsNothing);
  });

  testWidgets('A controller can control more than one CupertinoTabScaffold, '
      'removal of listeners does not break the controller', (WidgetTester tester) async {
    final List<int> tabsPainted0 = <int>[];
    final List<int> tabsPainted1 = <int>[];
    MockCupertinoTabController controller = MockCupertinoTabController(initialIndex: 2);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Stack(
            children: <Widget>[
              CupertinoTabScaffold(
                tabBar: CupertinoTabBar(
                  items: List<BottomNavigationBarItem>.generate(3, tabGenerator),
                ),
                controller: controller,
                tabBuilder: (BuildContext context, int index) {
                  return CustomPaint(
                    painter: TestCallbackPainter(onPaint: () => tabsPainted0.add(index)),
                  );
                },
              ),
              CupertinoTabScaffold(
                tabBar: CupertinoTabBar(
                  items: List<BottomNavigationBarItem>.generate(3, tabGenerator),
                ),
                controller: controller,
                tabBuilder: (BuildContext context, int index) {
                  return CustomPaint(
                    painter: TestCallbackPainter(onPaint: () => tabsPainted1.add(index)),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );
    expect(tabsPainted0, const <int>[2]);
    expect(tabsPainted1, const <int>[2]);
    expect(controller.numOfListeners, 2);

    controller.index = 0;
    await tester.pump();
    expect(tabsPainted0, const <int>[2, 0]);
    expect(tabsPainted1, const <int>[2, 0]);

    controller.index = 1;
    // Removing one of the tabs works.
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Stack(
            children: <Widget>[
              CupertinoTabScaffold(
                tabBar: CupertinoTabBar(
                  items: List<BottomNavigationBarItem>.generate(3, tabGenerator),
                ),
                controller: controller,
                tabBuilder: (BuildContext context, int index) {
                  return CustomPaint(
                    painter: TestCallbackPainter(onPaint: () => tabsPainted0.add(index)),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );

    expect(tabsPainted0, const <int>[2, 0, 1]);
    expect(tabsPainted1, const <int>[2, 0]);
    expect(controller.numOfListeners, 1);

    // Replacing controller works.
    controller.dispose();
    controller = MockCupertinoTabController(initialIndex: 2);
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Stack(
            children: <Widget>[
              CupertinoTabScaffold(
                tabBar: CupertinoTabBar(
                  items: List<BottomNavigationBarItem>.generate(3, tabGenerator),
                ),
                controller: controller,
                tabBuilder: (BuildContext context, int index) {
                  return CustomPaint(
                    painter: TestCallbackPainter(onPaint: () => tabsPainted0.add(index)),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );
    expect(tabsPainted0, const <int>[2, 0, 1, 2]);
    expect(tabsPainted1, const <int>[2, 0]);
    expect(controller.numOfListeners, 1);
  });

  testWidgets('Assert when current tab index >= number of tabs', (WidgetTester tester) async {
    final CupertinoTabController controller = CupertinoTabController(initialIndex: 2);
    addTearDown(controller.dispose);

    try {
      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoTabScaffold(
            tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(2, tabGenerator)),
            controller: controller,
            tabBuilder: (BuildContext context, int index) => Text('Different page ${index + 1}'),
          ),
        ),
      );
    } on AssertionError catch (e) {
      expect(e.toString(), contains('controller.index < tabBar.items.length'));
    }

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(3, tabGenerator)),
          controller: controller,
          tabBuilder: (BuildContext context, int index) => Text('Different page ${index + 1}'),
        ),
      ),
    );

    expect(tester.takeException(), null);

    controller.index = 10;
    await tester.pump();

    final String message = tester.takeException().toString();
    expect(message, contains('current index ${controller.index}'));
    expect(message, contains('with 3 tabs'));
  });

  testWidgets("Don't replace focus nodes for existing tabs when changing tab count", (
    WidgetTester tester,
  ) async {
    final CupertinoTabController controller = CupertinoTabController(initialIndex: 2);
    addTearDown(controller.dispose);

    final List<FocusScopeNode> scopes = <FocusScopeNode>[];
    for (int i = 0; i < 5; i++) {
      final FocusScopeNode scope = FocusScopeNode();
      addTearDown(scope.dispose);
      scopes.add(scope);
    }
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(3, tabGenerator)),
          controller: controller,
          tabBuilder: (BuildContext context, int index) {
            scopes[index] = FocusScope.of(context);
            return Container();
          },
        ),
      ),
    );

    for (int i = 0; i < 3; i++) {
      controller.index = i;
      await tester.pump();
    }
    await tester.pump();

    final List<FocusScopeNode> newScopes = <FocusScopeNode>[];
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoTabScaffold(
          tabBar: CupertinoTabBar(items: List<BottomNavigationBarItem>.generate(5, tabGenerator)),
          controller: controller,
          tabBuilder: (BuildContext context, int index) {
            newScopes.add(FocusScope.of(context));
            return Container();
          },
        ),
      ),
    );
    for (int i = 0; i < 5; i++) {
      controller.index = i;
      await tester.pump();
    }
    await tester.pump();

    expect(scopes.sublist(0, 3), equals(newScopes.sublist(0, 3)));
  });

  testWidgets('Current tab index cannot go below zero or be null', (WidgetTester tester) async {
    void expectAssertionError(VoidCallback callback, String errorMessage) {
      try {
        callback();
      } on AssertionError catch (e) {
        expect(e.toString(), contains(errorMessage));
      }
    }

    expectAssertionError(() => CupertinoTabController(initialIndex: -1), '>= 0');

    final CupertinoTabController controller = CupertinoTabController();
    addTearDown(controller.dispose);

    expectAssertionError(() => controller.index = -1, '>= 0');
  });

  testWidgets('Does not lose state when focusing on text input', (WidgetTester tester) async {
    // Regression testing for https://github.com/flutter/flutter/issues/28457.

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: CupertinoApp(
          home: CupertinoTabScaffold(
            tabBar: _buildTabBar(),
            tabBuilder: (BuildContext context, int index) {
              return const CupertinoTextField();
            },
          ),
        ),
      ),
    );

    final EditableTextState editableState = tester.state<EditableTextState>(
      find.byType(EditableText),
    );

    await tester.enterText(find.byType(CupertinoTextField), "don't lose me");

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 100)),
        child: CupertinoApp(
          home: CupertinoTabScaffold(
            tabBar: _buildTabBar(),
            tabBuilder: (BuildContext context, int index) {
              return const CupertinoTextField();
            },
          ),
        ),
      ),
    );

    // The exact same state instance is still there.
    expect(tester.state<EditableTextState>(find.byType(EditableText)), editableState);
    expect(find.text("don't lose me"), findsOneWidget);
  });

  testWidgets('textScaleFactor is set to 1.0', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Builder(
          builder: (BuildContext context) {
            return MediaQuery.withClampedTextScaling(
              minScaleFactor: 99,
              maxScaleFactor: 99,
              child: CupertinoTabScaffold(
                tabBar: CupertinoTabBar(
                  items: List<BottomNavigationBarItem>.generate(
                    10,
                    (int i) => BottomNavigationBarItem(
                      icon: ImageIcon(MemoryImage(Uint8List.fromList(kTransparentImage))),
                      label: '$i',
                    ),
                  ),
                ),
                tabBuilder: (BuildContext context, int index) => const Text('content'),
              ),
            );
          },
        ),
      ),
    );

    final Iterable<RichText> barItems = tester.widgetList<RichText>(
      find.descendant(of: find.byType(CupertinoTabBar), matching: find.byType(RichText)),
    );

    final Iterable<RichText> contents = tester.widgetList<RichText>(
      find.descendant(
        of: find.text('content'),
        matching: find.byType(RichText),
        skipOffstage: false,
      ),
    );

    expect(barItems.length, greaterThan(0));
    expect(
      barItems,
      isNot(contains(predicate((RichText t) => t.textScaler != TextScaler.noScaling))),
    );

    expect(contents.length, greaterThan(0));
    expect(
      contents,
      isNot(contains(predicate((RichText t) => t.textScaler != const TextScaler.linear(99.0)))),
    );
  });

  testWidgets('state restoration', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        restorationScopeId: 'app',
        home: CupertinoTabScaffold(
          restorationId: 'scaffold',
          tabBar: CupertinoTabBar(
            items: List<BottomNavigationBarItem>.generate(
              4,
              (int i) =>
                  BottomNavigationBarItem(icon: const Icon(CupertinoIcons.map), label: 'Tab $i'),
            ),
          ),
          tabBuilder: (BuildContext context, int i) => Text('Content $i'),
        ),
      ),
    );

    expect(find.text('Content 0'), findsOneWidget);
    expect(find.text('Content 1'), findsNothing);
    expect(find.text('Content 2'), findsNothing);
    expect(find.text('Content 3'), findsNothing);

    await tester.tap(find.text('Tab 2'));
    await tester.pumpAndSettle();

    expect(find.text('Content 0'), findsNothing);
    expect(find.text('Content 1'), findsNothing);
    expect(find.text('Content 2'), findsOneWidget);
    expect(find.text('Content 3'), findsNothing);

    await tester.restartAndRestore();

    expect(find.text('Content 0'), findsNothing);
    expect(find.text('Content 1'), findsNothing);
    expect(find.text('Content 2'), findsOneWidget);
    expect(find.text('Content 3'), findsNothing);

    final TestRestorationData data = await tester.getRestorationData();

    await tester.tap(find.text('Tab 1'));
    await tester.pumpAndSettle();

    expect(find.text('Content 0'), findsNothing);
    expect(find.text('Content 1'), findsOneWidget);
    expect(find.text('Content 2'), findsNothing);
    expect(find.text('Content 3'), findsNothing);

    await tester.restoreFrom(data);

    expect(find.text('Content 0'), findsNothing);
    expect(find.text('Content 1'), findsNothing);
    expect(find.text('Content 2'), findsOneWidget);
    expect(find.text('Content 3'), findsNothing);
  });

  testWidgets('switch from internal to external controller with state restoration', (
    WidgetTester tester,
  ) async {
    Widget buildWidget({CupertinoTabController? controller}) {
      return CupertinoApp(
        restorationScopeId: 'app',
        home: CupertinoTabScaffold(
          controller: controller,
          restorationId: 'scaffold',
          tabBar: CupertinoTabBar(
            items: List<BottomNavigationBarItem>.generate(
              4,
              (int i) =>
                  BottomNavigationBarItem(icon: const Icon(CupertinoIcons.map), label: 'Tab $i'),
            ),
          ),
          tabBuilder: (BuildContext context, int i) => Text('Content $i'),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    expect(find.text('Content 0'), findsOneWidget);
    expect(find.text('Content 1'), findsNothing);
    expect(find.text('Content 2'), findsNothing);
    expect(find.text('Content 3'), findsNothing);

    await tester.tap(find.text('Tab 2'));
    await tester.pumpAndSettle();

    expect(find.text('Content 0'), findsNothing);
    expect(find.text('Content 1'), findsNothing);
    expect(find.text('Content 2'), findsOneWidget);
    expect(find.text('Content 3'), findsNothing);

    final CupertinoTabController controller = CupertinoTabController(initialIndex: 3);
    addTearDown(controller.dispose);
    await tester.pumpWidget(buildWidget(controller: controller));

    expect(find.text('Content 0'), findsNothing);
    expect(find.text('Content 1'), findsNothing);
    expect(find.text('Content 2'), findsNothing);
    expect(find.text('Content 3'), findsOneWidget);

    await tester.pumpWidget(buildWidget());

    expect(find.text('Content 0'), findsOneWidget);
    expect(find.text('Content 1'), findsNothing);
    expect(find.text('Content 2'), findsNothing);
    expect(find.text('Content 3'), findsNothing);
  });

  group('Android Predictive Back', () {
    bool? lastFrameworkHandlesBack;
    setUp(() async {
      lastFrameworkHandlesBack = null;
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          if (methodCall.method == 'SystemNavigator.setFrameworkHandlesBack') {
            expect(methodCall.arguments, isA<bool>());
            lastFrameworkHandlesBack = methodCall.arguments as bool;
          }
          return;
        },
      );
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/lifecycle',
        const StringCodec().encodeMessage(AppLifecycleState.resumed.toString()),
        (ByteData? data) {},
      );
    });

    tearDown(() {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        null,
      );
    });

    testWidgets(
      'System back navigation inside of tabs',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: MediaQuery(
              data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 200)),
              child: CupertinoTabScaffold(
                tabBar: _buildTabBar(),
                tabBuilder: (BuildContext context, int index) {
                  return CupertinoTabView(
                    builder: (BuildContext context) {
                      return CupertinoPageScaffold(
                        navigationBar: CupertinoNavigationBar(
                          middle: Text('Page 1 of tab ${index + 1}'),
                        ),
                        child: Center(
                          child: CupertinoButton(
                            child: const Text('Next page'),
                            onPressed: () {
                              Navigator.of(context).push(
                                CupertinoPageRoute<void>(
                                  builder: (BuildContext context) {
                                    return CupertinoPageScaffold(
                                      navigationBar: CupertinoNavigationBar(
                                        middle: Text('Page 2 of tab ${index + 1}'),
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
          ),
        );

        expect(find.text('Page 1 of tab 1'), findsOneWidget);
        expect(find.text('Page 2 of tab 1'), findsNothing);
        expect(lastFrameworkHandlesBack, isFalse);

        await tester.tap(find.text('Next page'));
        await tester.pumpAndSettle();
        expect(find.text('Page 1 of tab 1'), findsNothing);
        expect(find.text('Page 2 of tab 1'), findsOneWidget);
        expect(lastFrameworkHandlesBack, isTrue);

        await simulateSystemBack();
        await tester.pumpAndSettle();
        expect(find.text('Page 1 of tab 1'), findsOneWidget);
        expect(find.text('Page 2 of tab 1'), findsNothing);
        expect(lastFrameworkHandlesBack, isFalse);

        await tester.tap(find.text('Next page'));
        await tester.pumpAndSettle();
        expect(find.text('Page 1 of tab 1'), findsNothing);
        expect(find.text('Page 2 of tab 1'), findsOneWidget);
        expect(lastFrameworkHandlesBack, isTrue);

        await tester.tap(find.text('Tab 2'));
        await tester.pumpAndSettle();
        expect(find.text('Page 1 of tab 2'), findsOneWidget);
        expect(find.text('Page 2 of tab 2'), findsNothing);
        expect(lastFrameworkHandlesBack, isFalse);

        await tester.tap(find.text('Tab 1'));
        await tester.pumpAndSettle();
        expect(find.text('Page 1 of tab 1'), findsNothing);
        expect(find.text('Page 2 of tab 1'), findsOneWidget);
        expect(lastFrameworkHandlesBack, isTrue);

        await simulateSystemBack();
        await tester.pumpAndSettle();
        expect(find.text('Page 1 of tab 1'), findsOneWidget);
        expect(find.text('Page 2 of tab 1'), findsNothing);
        expect(lastFrameworkHandlesBack, isFalse);

        await tester.tap(find.text('Tab 2'));
        await tester.pumpAndSettle();
        expect(find.text('Page 1 of tab 2'), findsOneWidget);
        expect(find.text('Page 2 of tab 2'), findsNothing);
        expect(lastFrameworkHandlesBack, isFalse);

        imageCache.clear();
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
      skip: kIsWeb, // [intended] frameworkHandlesBack not used on web.
    );
  });
}

CupertinoTabBar _buildTabBar({int selectedTab = 0}) {
  return CupertinoTabBar(
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
    currentIndex: selectedTab,
    onTap: (int newTab) => selectedTabs.add(newTab),
  );
}
