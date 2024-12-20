// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';
import 'app_bar_utils.dart';

Widget buildSliverAppBarApp({
  bool floating = false,
  bool pinned = false,
  double? collapsedHeight,
  double? expandedHeight,
  bool snap = false,
  double toolbarHeight = kToolbarHeight,
}) {
  return MaterialApp(
    home: Scaffold(
      body: DefaultTabController(
        length: 3,
        child: CustomScrollView(
          primary: true,
          slivers: <Widget>[
            SliverAppBar(
              title: const Text('AppBar Title'),
              floating: floating,
              pinned: pinned,
              collapsedHeight: collapsedHeight,
              expandedHeight: expandedHeight,
              toolbarHeight: toolbarHeight,
              snap: snap,
              bottom: TabBar(
                tabs:
                    <String>['A', 'B', 'C'].map<Widget>((String t) => Tab(text: 'TAB $t')).toList(),
              ),
            ),
            SliverToBoxAdapter(child: Container(height: 1200.0, color: Colors.orange[400])),
          ],
        ),
      ),
    ),
  );
}

void main() {
  setUp(() {
    debugResetSemanticsIdCounter();
  });

  testWidgets('SliverAppBar large & medium title respects automaticallyImplyLeading', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/121511
    const String title = 'AppBar Title';
    const double titleSpacing = 16.0;

    Widget buildWidget() {
      return MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return Center(
                child: FilledButton(
                  onPressed: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute<void>(
                        builder: (BuildContext context) {
                          return Scaffold(
                            body: CustomScrollView(
                              primary: true,
                              slivers: <Widget>[
                                const SliverAppBar.large(title: Text(title)),
                                SliverToBoxAdapter(
                                  child: Container(height: 1200, color: Colors.orange[400]),
                                ),
                              ],
                            ),
                          );
                        },
                      ),
                    );
                  },
                  child: const Text('Go to page'),
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    expect(find.byType(BackButton), findsNothing);

    await tester.tap(find.byType(FilledButton));
    await tester.pumpAndSettle();

    final Finder collapsedTitle = find.text(title).last;
    // Get the offset of the Center widget that wraps the IconButton.
    final Offset backButtonOffset = tester.getTopRight(
      find.ancestor(of: find.byType(IconButton), matching: find.byType(Center)),
    );
    final Offset titleOffset = tester.getTopLeft(collapsedTitle);
    expect(titleOffset.dx, backButtonOffset.dx + titleSpacing);
  });

  testWidgets('SliverAppBar.medium with bottom widget', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/115091
    const double collapsedAppBarHeight = 64;
    const double expandedAppBarHeight = 112;
    const double bottomHeight = 48;
    const String title = 'Medium App Bar';

    Widget buildWidget() {
      return MaterialApp(
        home: DefaultTabController(
          length: 3,
          child: Scaffold(
            body: CustomScrollView(
              primary: true,
              slivers: <Widget>[
                SliverAppBar.medium(
                  leading: IconButton(onPressed: () {}, icon: const Icon(Icons.menu)),
                  title: const Text(title),
                  bottom: const TabBar(
                    tabs: <Widget>[Tab(text: 'Tab 1'), Tab(text: 'Tab 2'), Tab(text: 'Tab 3')],
                  ),
                ),
                SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
              ],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight + bottomHeight);

    final Finder expandedTitle = find.text(title).first;
    final Offset expandedTitleOffset = tester.getBottomLeft(expandedTitle);
    final Offset tabOffset = tester.getTopLeft(find.byType(TabBar));
    expect(expandedTitleOffset.dy, tabOffset.dy);

    // Scroll CustomScrollView to collapse SliverAppBar.
    final ScrollController controller = primaryScrollController(tester);
    controller.jumpTo(160);
    await tester.pumpAndSettle();

    expect(appBarHeight(tester), collapsedAppBarHeight + bottomHeight);
  });

  testWidgets('SliverAppBar.large with bottom widget', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/115091
    const double collapsedAppBarHeight = 64;
    const double expandedAppBarHeight = 152;
    const double bottomHeight = 48;
    const String title = 'Large App Bar';

    Widget buildWidget() {
      return MaterialApp(
        home: DefaultTabController(
          length: 3,
          child: Scaffold(
            body: CustomScrollView(
              primary: true,
              slivers: <Widget>[
                SliverAppBar.large(
                  leading: IconButton(onPressed: () {}, icon: const Icon(Icons.menu)),
                  title: const Text(title),
                  bottom: const TabBar(
                    tabs: <Widget>[Tab(text: 'Tab 1'), Tab(text: 'Tab 2'), Tab(text: 'Tab 3')],
                  ),
                ),
                SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
              ],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight + bottomHeight);

    final Finder expandedTitle = find.text(title).first;
    final Offset expandedTitleOffset = tester.getBottomLeft(expandedTitle);
    final Offset tabOffset = tester.getTopLeft(find.byType(TabBar));
    expect(expandedTitleOffset.dy, tabOffset.dy);

    // Scroll CustomScrollView to collapse SliverAppBar.
    final ScrollController controller = primaryScrollController(tester);
    controller.jumpTo(200);
    await tester.pumpAndSettle();

    expect(appBarHeight(tester), collapsedAppBarHeight + bottomHeight);
  });

  testWidgets('SliverAppBar.medium expanded title has upper limit on text scaling', (
    WidgetTester tester,
  ) async {
    const String title = 'Medium AppBar';
    Widget buildAppBar({double textScaleFactor = 1.0}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: MediaQuery.withClampedTextScaling(
          minScaleFactor: textScaleFactor,
          maxScaleFactor: textScaleFactor,
          child: Material(
            child: CustomScrollView(
              slivers: <Widget>[
                const SliverAppBar.medium(title: Text(title)),
                SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
              ],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAppBar());

    final Finder expandedTitle = find.text(title).first;
    expect(tester.getRect(expandedTitle).height, 32.0);
    verifyTextNotClipped(expandedTitle, tester);

    await tester.pumpWidget(buildAppBar(textScaleFactor: 2.0));
    expect(tester.getRect(expandedTitle).height, 43.0);
    verifyTextNotClipped(expandedTitle, tester);

    await tester.pumpWidget(buildAppBar(textScaleFactor: 3.0));
    expect(tester.getRect(expandedTitle).height, 43.0);
    verifyTextNotClipped(expandedTitle, tester);
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('SliverAppBar.large expanded title has upper limit on text scaling', (
    WidgetTester tester,
  ) async {
    const String title = 'Large AppBar';
    Widget buildAppBar({double textScaleFactor = 1.0}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: MediaQuery.withClampedTextScaling(
          minScaleFactor: textScaleFactor,
          maxScaleFactor: textScaleFactor,
          child: Material(
            child: CustomScrollView(
              slivers: <Widget>[
                const SliverAppBar.large(title: Text(title, maxLines: 1)),
                SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
              ],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAppBar());

    final Finder expandedTitle = find.text(title).first;
    expect(tester.getRect(expandedTitle).height, 36.0);

    await tester.pumpWidget(buildAppBar(textScaleFactor: 2.0));
    expect(tester.getRect(expandedTitle).height, closeTo(48.0, 0.1));

    await tester.pumpWidget(buildAppBar(textScaleFactor: 3.0));
    expect(tester.getRect(expandedTitle).height, closeTo(48.0, 0.1));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets(
    'SliverAppBar.medium expanded title position is adjusted with textScaleFactor',
    (WidgetTester tester) async {
      const String title = 'Medium AppBar';
      Widget buildAppBar({double textScaleFactor = 1.0}) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: true),
          home: MediaQuery.withClampedTextScaling(
            minScaleFactor: textScaleFactor,
            maxScaleFactor: textScaleFactor,
            child: Material(
              child: CustomScrollView(
                slivers: <Widget>[
                  const SliverAppBar.medium(title: Text(title, maxLines: 1)),
                  SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
                ],
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildAppBar());

      final Finder expandedTitle = find.text(title).first;
      expect(tester.getBottomLeft(expandedTitle).dy, 96.0);
      verifyTextNotClipped(expandedTitle, tester);

      await tester.pumpWidget(buildAppBar(textScaleFactor: 2.0));
      expect(tester.getBottomLeft(expandedTitle).dy, 107.0);
      verifyTextNotClipped(expandedTitle, tester);

      await tester.pumpWidget(buildAppBar(textScaleFactor: 3.0));
      expect(tester.getBottomLeft(expandedTitle).dy, 107.0);
      verifyTextNotClipped(expandedTitle, tester);
    },
    skip: kIsWeb && !isSkiaWeb, // https://github.com/flutter/flutter/issues/99933
  );

  testWidgets('SliverAppBar.large expanded title position is adjusted with textScaleFactor', (
    WidgetTester tester,
  ) async {
    const String title = 'Large AppBar';
    Widget buildAppBar({double textScaleFactor = 1.0}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: MediaQuery.withClampedTextScaling(
          minScaleFactor: textScaleFactor,
          maxScaleFactor: textScaleFactor,
          child: Material(
            child: CustomScrollView(
              slivers: <Widget>[
                const SliverAppBar.large(title: Text(title, maxLines: 1)),
                SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
              ],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAppBar());
    final Finder expandedTitle = find.text(title).first;
    final RenderSliver renderSliverAppBar = tester.renderObject(find.byType(SliverAppBar));
    expect(
      tester.getBottomLeft(expandedTitle).dy,
      renderSliverAppBar.geometry!.scrollExtent - 28.0,
      reason: 'bottom padding of a large expanded title should be 28.',
    );
    verifyTextNotClipped(expandedTitle, tester);

    await tester.pumpWidget(buildAppBar(textScaleFactor: 2.0));
    expect(
      tester.getBottomLeft(expandedTitle).dy,
      renderSliverAppBar.geometry!.scrollExtent - 28.0,
      reason: 'bottom padding of a large expanded title should be 28.',
    );
    verifyTextNotClipped(expandedTitle, tester);

    // The bottom padding of the expanded title needs to be reduced for it to be
    // fully visible.
    await tester.pumpWidget(buildAppBar(textScaleFactor: 3.0));
    expect(tester.getBottomLeft(expandedTitle).dy, 124.0);
    verifyTextNotClipped(expandedTitle, tester);
  });

  testWidgets('SliverAppBar.medium collapsed title does not overlap with leading/actions widgets', (
    WidgetTester tester,
  ) async {
    const String title = 'Medium SliverAppBar Very Long Title';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              SliverPadding(
                padding: const EdgeInsets.symmetric(horizontal: 200),
                sliver: SliverAppBar.medium(
                  leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
                  title: const Text(title, maxLines: 1),
                  centerTitle: true,
                  actions: const <Widget>[
                    Icon(Icons.search),
                    Icon(Icons.sort),
                    Icon(Icons.more_vert),
                  ],
                ),
              ),
              SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
            ],
          ),
        ),
      ),
    );

    // Scroll to collapse the SliverAppBar.
    final ScrollController controller = primaryScrollController(tester);
    controller.jumpTo(45);
    await tester.pumpAndSettle();

    final Offset leadingOffset = tester.getTopRight(find.byIcon(Icons.menu));
    Offset titleOffset = tester.getTopLeft(find.text(title).last);
    // The title widget should be to the right of the leading widget.
    expect(titleOffset.dx, greaterThan(leadingOffset.dx));

    titleOffset = tester.getTopRight(find.text(title).last);
    final Offset searchOffset = tester.getTopLeft(find.byIcon(Icons.search));
    // The title widget should be to the left of the search icon.
    expect(titleOffset.dx, lessThan(searchOffset.dx));
  });

  testWidgets('SliverAppBar.large collapsed title does not overlap with leading/actions widgets', (
    WidgetTester tester,
  ) async {
    const String title = 'Large SliverAppBar Very Long Title';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              SliverPadding(
                padding: const EdgeInsets.symmetric(horizontal: 200),
                sliver: SliverAppBar.large(
                  leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
                  title: const Text(title, maxLines: 1),
                  centerTitle: true,
                  actions: const <Widget>[
                    Icon(Icons.search),
                    Icon(Icons.sort),
                    Icon(Icons.more_vert),
                  ],
                ),
              ),
              SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
            ],
          ),
        ),
      ),
    );

    // Scroll to collapse the SliverAppBar.
    final ScrollController controller = primaryScrollController(tester);
    controller.jumpTo(45);
    await tester.pumpAndSettle();

    final Offset leadingOffset = tester.getTopRight(find.byIcon(Icons.menu));
    Offset titleOffset = tester.getTopLeft(find.text(title).last);
    // The title widget should be to the right of the leading widget.
    expect(titleOffset.dx, greaterThan(leadingOffset.dx));

    titleOffset = tester.getTopRight(find.text(title).last);
    final Offset searchOffset = tester.getTopLeft(find.byIcon(Icons.search));
    // The title widget should be to the left of the search icon.
    expect(titleOffset.dx, lessThan(searchOffset.dx));
  });

  testWidgets('SliverAppBar.medium respects title spacing', (WidgetTester tester) async {
    const String title = 'Medium SliverAppBar Very Long Title';
    const double titleSpacing = 16.0;

    Widget buildWidget({double? titleSpacing, bool? centerTitle}) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              SliverPadding(
                padding: const EdgeInsets.symmetric(horizontal: 200),
                sliver: SliverAppBar.medium(
                  leading: IconButton(onPressed: () {}, icon: const Icon(Icons.menu)),
                  title: const Text(title, maxLines: 1),
                  centerTitle: centerTitle,
                  titleSpacing: titleSpacing,
                  actions: <Widget>[
                    IconButton(onPressed: () {}, icon: const Icon(Icons.sort)),
                    IconButton(onPressed: () {}, icon: const Icon(Icons.more_vert)),
                  ],
                ),
              ),
              SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    final Finder collapsedTitle = find.text(title).last;

    // Scroll to collapse the SliverAppBar.
    ScrollController controller = primaryScrollController(tester);
    controller.jumpTo(120);
    await tester.pumpAndSettle();

    // By default, title widget should be to the right of the
    // leading widget and title spacing should be respected.
    Offset titleOffset = tester.getTopLeft(collapsedTitle);
    Offset iconButtonOffset = tester.getTopRight(
      find.ancestor(
        of: find.widgetWithIcon(IconButton, Icons.menu),
        matching: find.byType(ConstrainedBox),
      ),
    );
    expect(titleOffset.dx, iconButtonOffset.dx + titleSpacing);

    await tester.pumpWidget(buildWidget(centerTitle: true));
    // Scroll to collapse the SliverAppBar.
    controller = primaryScrollController(tester);
    controller.jumpTo(120);
    await tester.pumpAndSettle();

    // By default, title widget should be to the left of the first
    // trailing widget and title spacing should be respected.
    titleOffset = tester.getTopRight(collapsedTitle);
    iconButtonOffset = tester.getTopLeft(find.widgetWithIcon(IconButton, Icons.sort));
    expect(titleOffset.dx, iconButtonOffset.dx - titleSpacing);

    // Test custom title spacing, set to 0.0.
    await tester.pumpWidget(buildWidget(titleSpacing: 0.0));
    // Scroll to collapse the SliverAppBar.
    controller = primaryScrollController(tester);
    controller.jumpTo(120);
    await tester.pumpAndSettle();

    // The title widget should be to the right of the leading
    // widget with no spacing.
    titleOffset = tester.getTopLeft(collapsedTitle);
    iconButtonOffset = tester.getTopRight(
      find.ancestor(
        of: find.widgetWithIcon(IconButton, Icons.menu),
        matching: find.byType(ConstrainedBox),
      ),
    );
    expect(titleOffset.dx, iconButtonOffset.dx);

    // Set centerTitle to true so the end of the title can reach
    // the action widgets.
    await tester.pumpWidget(buildWidget(titleSpacing: 0.0, centerTitle: true));
    // Scroll to collapse the SliverAppBar.
    controller = primaryScrollController(tester);
    controller.jumpTo(120);
    await tester.pumpAndSettle();

    // The title widget should be to the left of the first
    // leading widget with no spacing.
    titleOffset = tester.getTopRight(collapsedTitle);
    iconButtonOffset = tester.getTopLeft(find.widgetWithIcon(IconButton, Icons.sort));
    expect(titleOffset.dx, iconButtonOffset.dx);
  });

  testWidgets('SliverAppBar.large respects title spacing', (WidgetTester tester) async {
    const String title = 'Large SliverAppBar Very Long Title';
    const double titleSpacing = 16.0;

    Widget buildWidget({double? titleSpacing, bool? centerTitle}) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              SliverPadding(
                padding: const EdgeInsets.symmetric(horizontal: 200),
                sliver: SliverAppBar.large(
                  leading: IconButton(onPressed: () {}, icon: const Icon(Icons.menu)),
                  title: const Text(title, maxLines: 1),
                  centerTitle: centerTitle,
                  titleSpacing: titleSpacing,
                  actions: <Widget>[
                    IconButton(onPressed: () {}, icon: const Icon(Icons.sort)),
                    IconButton(onPressed: () {}, icon: const Icon(Icons.more_vert)),
                  ],
                ),
              ),
              SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    final Finder collapsedTitle = find.text(title).last;

    // Scroll to collapse the SliverAppBar.
    ScrollController controller = primaryScrollController(tester);
    controller.jumpTo(160);
    await tester.pumpAndSettle();

    // By default, title widget should be to the right of the leading
    // widget and title spacing should be respected.
    Offset titleOffset = tester.getTopLeft(collapsedTitle);
    Offset iconButtonOffset = tester.getTopRight(
      find.ancestor(
        of: find.widgetWithIcon(IconButton, Icons.menu),
        matching: find.byType(ConstrainedBox),
      ),
    );
    expect(titleOffset.dx, iconButtonOffset.dx + titleSpacing);

    await tester.pumpWidget(buildWidget(centerTitle: true));
    // Scroll to collapse the SliverAppBar.
    controller = primaryScrollController(tester);
    controller.jumpTo(160);
    await tester.pumpAndSettle();

    // By default, title widget should be to the left of the
    // leading widget and title spacing should be respected.
    titleOffset = tester.getTopRight(collapsedTitle);
    iconButtonOffset = tester.getTopLeft(find.widgetWithIcon(IconButton, Icons.sort));
    expect(titleOffset.dx, iconButtonOffset.dx - titleSpacing);

    // Test custom title spacing, set to 0.0.
    await tester.pumpWidget(buildWidget(titleSpacing: 0.0));
    controller = primaryScrollController(tester);
    controller.jumpTo(160);
    await tester.pumpAndSettle();

    // The title widget should be to the right of the leading
    // widget with no spacing.
    titleOffset = tester.getTopLeft(collapsedTitle);
    iconButtonOffset = tester.getTopRight(
      find.ancestor(
        of: find.widgetWithIcon(IconButton, Icons.menu),
        matching: find.byType(ConstrainedBox),
      ),
    );
    expect(titleOffset.dx, iconButtonOffset.dx);

    // Set centerTitle to true so the end of the title can reach
    // the action widgets.
    await tester.pumpWidget(buildWidget(titleSpacing: 0.0, centerTitle: true));
    // Scroll to collapse the SliverAppBar.
    controller = primaryScrollController(tester);
    controller.jumpTo(160);
    await tester.pumpAndSettle();

    // The title widget should be to the left of the first
    // leading widget with no spacing.
    titleOffset = tester.getTopRight(collapsedTitle);
    iconButtonOffset = tester.getTopLeft(find.widgetWithIcon(IconButton, Icons.sort));
    expect(titleOffset.dx, iconButtonOffset.dx);
  });

  testWidgets('SliverAppBar.medium without the leading widget updates collapsed title padding', (
    WidgetTester tester,
  ) async {
    const String title = 'Medium SliverAppBar Title';
    const double leadingPadding = 56.0;
    const double titleSpacing = 16.0;

    Widget buildWidget({bool showLeading = true}) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              SliverAppBar.medium(
                automaticallyImplyLeading: false,
                leading:
                    showLeading ? IconButton(icon: const Icon(Icons.menu), onPressed: () {}) : null,
                title: const Text(title),
              ),
              SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    final Finder collapsedTitle = find.text(title).last;

    // Scroll to collapse the SliverAppBar.
    ScrollController controller = primaryScrollController(tester);
    controller.jumpTo(45);
    await tester.pumpAndSettle();

    // If the leading widget is present, the title widget should be to the
    // right of the leading widget and title spacing should be respected.
    Offset titleOffset = tester.getTopLeft(collapsedTitle);
    expect(titleOffset.dx, leadingPadding + titleSpacing);

    // Hide the leading widget.
    await tester.pumpWidget(buildWidget(showLeading: false));
    // Scroll to collapse the SliverAppBar.
    controller = primaryScrollController(tester);
    controller.jumpTo(45);
    await tester.pumpAndSettle();

    // If the leading widget is not present, the title widget will
    // only have the default title spacing.
    titleOffset = tester.getTopLeft(collapsedTitle);
    expect(titleOffset.dx, titleSpacing);
  });

  testWidgets('SliverAppBar.large without the leading widget updates collapsed title padding', (
    WidgetTester tester,
  ) async {
    const String title = 'Large SliverAppBar Title';
    const double leadingPadding = 56.0;
    const double titleSpacing = 16.0;

    Widget buildWidget({bool showLeading = true}) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              SliverAppBar.large(
                automaticallyImplyLeading: false,
                leading:
                    showLeading ? IconButton(icon: const Icon(Icons.menu), onPressed: () {}) : null,
                title: const Text(title),
              ),
              SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    final Finder collapsedTitle = find.text(title).last;

    // Scroll CustomScrollView to collapse SliverAppBar.
    ScrollController controller = primaryScrollController(tester);
    controller.jumpTo(45);
    await tester.pumpAndSettle();

    // If the leading widget is present, the title widget should be to the
    // right of the leading widget and title spacing should be respected.
    Offset titleOffset = tester.getTopLeft(collapsedTitle);
    expect(titleOffset.dx, leadingPadding + titleSpacing);

    // Hide the leading widget.
    await tester.pumpWidget(buildWidget(showLeading: false));
    // Scroll to collapse the SliverAppBar.
    controller = primaryScrollController(tester);
    controller.jumpTo(45);
    await tester.pumpAndSettle();

    // If the leading widget is not present, the title widget will
    // only have the default title spacing.
    titleOffset = tester.getTopLeft(collapsedTitle);
    expect(titleOffset.dx, titleSpacing);
  });

  group('MaterialStateColor scrolledUnder', () {
    const double collapsedHeight = kToolbarHeight;
    const double expandedHeight = 200.0;
    const Color scrolledColor = Color(0xff00ff00);
    const Color defaultColor = Color(0xff0000ff);

    Widget buildSliverApp({
      required double contentHeight,
      bool reverse = false,
      bool includeFlexibleSpace = false,
    }) {
      return MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            reverse: reverse,
            slivers: <Widget>[
              SliverAppBar(
                elevation: 0,
                backgroundColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
                  return states.contains(MaterialState.scrolledUnder)
                      ? scrolledColor
                      : defaultColor;
                }),
                expandedHeight: expandedHeight,
                pinned: true,
                flexibleSpace:
                    includeFlexibleSpace
                        ? const FlexibleSpaceBar(title: Text('SliverAppBar'))
                        : null,
              ),
              SliverList(
                delegate: SliverChildListDelegate(<Widget>[
                  Container(height: contentHeight, color: Colors.teal),
                ]),
              ),
            ],
          ),
        ),
      );
    }

    testWidgets('backgroundColor', (WidgetTester tester) async {
      await tester.pumpWidget(buildSliverApp(contentHeight: 1200.0));

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, -expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, collapsedHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      await gesture.moveBy(const Offset(0.0, expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);
    });

    testWidgets('backgroundColor with FlexibleSpace', (WidgetTester tester) async {
      await tester.pumpWidget(buildSliverApp(contentHeight: 1200.0, includeFlexibleSpace: true));

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, -expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, collapsedHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      await gesture.moveBy(const Offset(0.0, expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);
    });

    testWidgets('backgroundColor - reverse', (WidgetTester tester) async {
      await tester.pumpWidget(buildSliverApp(contentHeight: 1200.0, reverse: true));

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, collapsedHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      await gesture.moveBy(const Offset(0.0, -expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);
    });

    testWidgets('backgroundColor with FlexibleSpace - reverse', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildSliverApp(contentHeight: 1200.0, reverse: true, includeFlexibleSpace: true),
      );

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);

      TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), scrolledColor);
      expect(tester.getSize(findAppBarMaterial()).height, collapsedHeight);

      gesture = await tester.startGesture(const Offset(50.0, 300.0));
      await gesture.moveBy(const Offset(0.0, -expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);
    });

    testWidgets('backgroundColor - not triggered in reverse for short content', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(buildSliverApp(contentHeight: 200, reverse: true));

      // In reverse, the content here is not long enough to scroll under the app
      // bar.
      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);
    });

    testWidgets('backgroundColor with FlexibleSpace - not triggered in reverse for short content', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildSliverApp(contentHeight: 200, reverse: true, includeFlexibleSpace: true),
      );

      // In reverse, the content here is not long enough to scroll under the app
      // bar.
      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);

      final TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
      await gesture.moveBy(const Offset(0.0, expandedHeight));
      await gesture.up();
      await tester.pumpAndSettle();

      expect(getAppBarBackgroundColor(tester), defaultColor);
      expect(tester.getSize(findAppBarMaterial()).height, expandedHeight);
    });
  });

  testWidgets('SliverAppBar default configuration', (WidgetTester tester) async {
    await tester.pumpWidget(buildSliverAppBarApp());

    final ScrollController controller = primaryScrollController(tester);
    expect(controller.offset, 0.0);
    expect(find.byType(SliverAppBar), findsOneWidget);

    final double initialAppBarHeight = appBarHeight(tester);
    final double initialTabBarHeight = tabBarHeight(tester);

    // Scroll the not-pinned appbar partially out of view
    controller.jumpTo(50.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), initialAppBarHeight);
    expect(tabBarHeight(tester), initialTabBarHeight);

    // Scroll the not-pinned appbar out of view
    controller.jumpTo(600.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsNothing);
    expect(appBarHeight(tester), initialAppBarHeight);
    expect(tabBarHeight(tester), initialTabBarHeight);

    // Scroll the not-pinned appbar back into view
    controller.jumpTo(0.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), initialAppBarHeight);
    expect(tabBarHeight(tester), initialTabBarHeight);
  });

  testWidgets('SliverAppBar expandedHeight, pinned', (WidgetTester tester) async {
    await tester.pumpWidget(buildSliverAppBarApp(pinned: true, expandedHeight: 128.0));

    final ScrollController controller = primaryScrollController(tester);
    expect(controller.offset, 0.0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), 128.0);

    const double initialAppBarHeight = 128.0;
    final double initialTabBarHeight = tabBarHeight(tester);

    // Scroll the not-pinned appbar, collapsing the expanded height. At this
    // point both the toolbar and the tabbar are visible.
    controller.jumpTo(600.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(tabBarHeight(tester), initialTabBarHeight);
    expect(appBarHeight(tester), lessThan(initialAppBarHeight));
    expect(appBarHeight(tester), greaterThan(initialTabBarHeight));

    // Scroll the not-pinned appbar back into view
    controller.jumpTo(0.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), initialAppBarHeight);
    expect(tabBarHeight(tester), initialTabBarHeight);
  });

  testWidgets('SliverAppBar expandedHeight, pinned and floating', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildSliverAppBarApp(floating: true, pinned: true, expandedHeight: 128.0),
    );

    final ScrollController controller = primaryScrollController(tester);
    expect(controller.offset, 0.0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), 128.0);

    const double initialAppBarHeight = 128.0;
    final double initialTabBarHeight = tabBarHeight(tester);

    // Scroll the floating-pinned appbar, collapsing the expanded height. At this
    // point only the tabBar is visible.
    controller.jumpTo(600.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(tabBarHeight(tester), initialTabBarHeight);
    expect(appBarHeight(tester), lessThan(initialAppBarHeight));
    expect(appBarHeight(tester), initialTabBarHeight);

    // Scroll the floating-pinned appbar back into view
    controller.jumpTo(0.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), initialAppBarHeight);
    expect(tabBarHeight(tester), initialTabBarHeight);
  });

  testWidgets('SliverAppBar expandedHeight, floating with snap:true', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildSliverAppBarApp(floating: true, snap: true, expandedHeight: 128.0),
    );
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), 128.0);
    expect(appBarBottom(tester), 128.0);

    // Scroll to the middle of the list. The (floating) appbar is no longer visible.
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    position.jumpTo(256.00);
    await tester.pumpAndSettle();
    expect(find.byType(SliverAppBar), findsNothing);
    expect(appBarTop(tester), lessThanOrEqualTo(-128.0));

    // Drag the scrollable up and down. The app bar should not snap open, its
    // height should just track the drag offset.
    TestGesture gesture = await tester.startGesture(const Offset(50.0, 256.0));
    await gesture.moveBy(const Offset(0.0, 128.0)); // drag the appbar all the way open
    await tester.pump();
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), 128.0);

    await gesture.moveBy(const Offset(0.0, -50.0));
    await tester.pump();
    expect(appBarBottom(tester), 78.0); // 78 == 128 - 50

    // Trigger the snap open animation: drag down and release
    await gesture.moveBy(const Offset(0.0, 10.0));
    await gesture.up();

    // Now verify that the appbar is animating open
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    double bottom = appBarBottom(tester);
    expect(bottom, greaterThan(88.0)); // 88 = 78 + 10

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(appBarBottom(tester), greaterThan(bottom));

    // The animation finishes when the appbar is full height.
    await tester.pumpAndSettle();
    expect(appBarHeight(tester), 128.0);

    // Now that the app bar is open, perform the same drag scenario
    // in reverse: drag the appbar up and down and then trigger the
    // snap closed animation.
    gesture = await tester.startGesture(const Offset(50.0, 256.0));
    await gesture.moveBy(const Offset(0.0, -128.0)); // drag the appbar closed
    await tester.pump();
    expect(appBarBottom(tester), 0.0);

    await gesture.moveBy(const Offset(0.0, 100.0));
    await tester.pump();
    expect(appBarBottom(tester), 100.0);

    // Trigger the snap close animation: drag upwards and release
    await gesture.moveBy(const Offset(0.0, -10.0));
    await gesture.up();

    // Now verify that the appbar is animating closed
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    bottom = appBarBottom(tester);
    expect(bottom, lessThan(90.0));

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(appBarBottom(tester), lessThan(bottom));

    // The animation finishes when the appbar is off screen.
    await tester.pumpAndSettle();
    expect(appBarTop(tester), lessThanOrEqualTo(0.0));
    expect(appBarBottom(tester), lessThanOrEqualTo(0.0));
  });

  testWidgets('SliverAppBar expandedHeight, floating and pinned with snap:true', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      buildSliverAppBarApp(floating: true, pinned: true, snap: true, expandedHeight: 128.0),
    );
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), 128.0);
    expect(appBarBottom(tester), 128.0);

    // Scroll to the middle of the list. The only the tab bar is visible
    // because this is a pinned appbar.
    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    position.jumpTo(256.0);
    await tester.pumpAndSettle();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), kTextTabBarHeight);

    // Drag the scrollable up and down. The app bar should not snap open, the
    // bottom of the appbar should just track the drag offset.
    TestGesture gesture = await tester.startGesture(const Offset(50.0, 200.0));
    await gesture.moveBy(const Offset(0.0, 100.0));
    await tester.pump();
    expect(appBarHeight(tester), 100.0);

    await gesture.moveBy(const Offset(0.0, -25.0));
    await tester.pump();
    expect(appBarHeight(tester), 75.0);

    // Trigger the snap animation: drag down and release
    await gesture.moveBy(const Offset(0.0, 10.0));
    await gesture.up();

    // Now verify that the appbar is animating open
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    final double height = appBarHeight(tester);
    expect(height, greaterThan(85.0));
    expect(height, lessThan(128.0));

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(appBarHeight(tester), greaterThan(height));
    expect(appBarHeight(tester), lessThan(128.0));

    // The animation finishes when the appbar is fully expanded
    await tester.pumpAndSettle();
    expect(appBarTop(tester), 0.0);
    expect(appBarHeight(tester), 128.0);
    expect(appBarBottom(tester), 128.0);

    // Now that the appbar is fully expanded, Perform the same drag
    // scenario in reverse: drag the appbar up and down and then trigger
    // the snap closed animation.
    gesture = await tester.startGesture(const Offset(50.0, 256.0));
    await gesture.moveBy(const Offset(0.0, -128.0));
    await tester.pump();
    expect(appBarBottom(tester), kTextTabBarHeight);

    await gesture.moveBy(const Offset(0.0, 100.0));
    await tester.pump();
    expect(appBarBottom(tester), 100.0);

    // Trigger the snap close animation: drag upwards and release
    await gesture.moveBy(const Offset(0.0, -10.0));
    await gesture.up();

    // Now verify that the appbar is animating closed
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    final double bottom = appBarBottom(tester);
    expect(bottom, lessThan(90.0));

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(appBarBottom(tester), lessThan(bottom));

    // The animation finishes when the appbar shrinks back to its pinned height
    await tester.pumpAndSettle();
    expect(appBarTop(tester), lessThanOrEqualTo(0.0));
    expect(appBarBottom(tester), kTextTabBarHeight);
  });

  testWidgets('SliverAppBar expandedHeight, collapsedHeight', (WidgetTester tester) async {
    const double expandedAppBarHeight = 400.0;
    const double collapsedAppBarHeight = 200.0;

    await tester.pumpWidget(
      buildSliverAppBarApp(
        collapsedHeight: collapsedAppBarHeight,
        expandedHeight: expandedAppBarHeight,
      ),
    );

    final ScrollController controller = primaryScrollController(tester);
    expect(controller.offset, 0.0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);

    final double initialTabBarHeight = tabBarHeight(tester);

    // Scroll the not-pinned appbar partially out of view.
    controller.jumpTo(50.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight - 50.0);
    expect(tabBarHeight(tester), initialTabBarHeight);

    // Scroll the not-pinned appbar out of view, to its collapsed height.
    controller.jumpTo(600.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsNothing);
    expect(appBarHeight(tester), collapsedAppBarHeight + initialTabBarHeight);
    expect(tabBarHeight(tester), initialTabBarHeight);

    // Scroll the not-pinned appbar back into view.
    controller.jumpTo(0.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tabBarHeight(tester), initialTabBarHeight);
  });

  testWidgets('Material3 - SliverAppBar.medium defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: true);
    const double collapsedAppBarHeight = 64;
    const double expandedAppBarHeight = 112;

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              const SliverAppBar.medium(title: Text('AppBar Title')),
              SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
            ],
          ),
        ),
      ),
    );

    final ScrollController controller = primaryScrollController(tester);
    // There are two widgets for the title. The first title is a larger version
    // that is shown at the bottom when the app bar is expanded. It scrolls under
    // the main row until it is completely hidden and then the first title is
    // faded in. The last is the title on the mainrow with the icons. It is
    // transparent when the app bar is expanded, and opaque when it is collapsed.
    final Finder expandedTitle = find.text('AppBar Title').first;
    final Finder expandedTitleClip =
        find.ancestor(of: expandedTitle, matching: find.byType(ClipRect)).first;
    final Finder collapsedTitle = find.text('AppBar Title').last;
    final Finder collapsedTitleOpacity = find.ancestor(
      of: collapsedTitle,
      matching: find.byType(AnimatedOpacity),
    );

    // Default, fully expanded app bar.
    expect(controller.offset, 0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
    expect(tester.getSize(expandedTitleClip).height, expandedAppBarHeight - collapsedAppBarHeight);

    // Test the expanded title is positioned correctly.
    final Offset titleOffset = tester.getBottomLeft(expandedTitle);
    expect(titleOffset.dx, 16.0);
    if (!kIsWeb || isSkiaWeb) {
      expect(titleOffset.dy, 96.0);
    }

    verifyTextNotClipped(expandedTitle, tester);

    // Test the expanded title default color.
    expect(
      tester.renderObject<RenderParagraph>(expandedTitle).text.style!.color,
      theme.colorScheme.onSurface,
    );

    // Scroll the expanded app bar partially out of view.
    controller.jumpTo(45);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight - 45);
    expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
    expect(
      tester.getSize(expandedTitleClip).height,
      expandedAppBarHeight - collapsedAppBarHeight - 45,
    );

    // Scroll so that it is completely collapsed.
    controller.jumpTo(600);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), collapsedAppBarHeight);
    expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 1);
    expect(tester.getSize(expandedTitleClip).height, 0);

    // Scroll back to fully expanded.
    controller.jumpTo(0);
    await tester.pumpAndSettle();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
    expect(tester.getSize(expandedTitleClip).height, expandedAppBarHeight - collapsedAppBarHeight);
  });

  testWidgets('Material3 - SliverAppBar.large defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: true);
    const double collapsedAppBarHeight = 64;
    const double expandedAppBarHeight = 152;

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              const SliverAppBar.large(title: Text('AppBar Title')),
              SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
            ],
          ),
        ),
      ),
    );

    final ScrollController controller = primaryScrollController(tester);
    // There are two widgets for the title. The first title is a larger version
    // that is shown at the bottom when the app bar is expanded. It scrolls under
    // the main row until it is completely hidden and then the first title is
    // faded in. The last is the title on the mainrow with the icons. It is
    // transparent when the app bar is expanded, and opaque when it is collapsed.
    final Finder expandedTitle = find.text('AppBar Title').first;
    final Finder expandedTitleClip =
        find.ancestor(of: expandedTitle, matching: find.byType(ClipRect)).first;
    final Finder collapsedTitle = find.text('AppBar Title').last;
    final Finder collapsedTitleOpacity = find.ancestor(
      of: collapsedTitle,
      matching: find.byType(AnimatedOpacity),
    );

    // Default, fully expanded app bar.
    expect(controller.offset, 0);
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
    expect(tester.getSize(expandedTitleClip).height, expandedAppBarHeight - collapsedAppBarHeight);

    // Test the expanded title is positioned correctly.
    final Offset titleOffset = tester.getBottomLeft(expandedTitle);
    expect(titleOffset.dx, 16.0);
    final RenderSliver renderSliverAppBar = tester.renderObject(find.byType(SliverAppBar));
    // The expanded title and the bottom padding fits in the flexible space.
    expect(
      titleOffset.dy,
      renderSliverAppBar.geometry!.scrollExtent - 28.0,
      reason: 'bottom padding of a large expanded title should be 28.',
    );
    verifyTextNotClipped(expandedTitle, tester);

    // Test the expanded title default color.
    expect(
      tester.renderObject<RenderParagraph>(expandedTitle).text.style!.color,
      theme.colorScheme.onSurface,
    );

    // Scroll the expanded app bar partially out of view.
    controller.jumpTo(45);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight - 45);
    expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
    expect(
      tester.getSize(expandedTitleClip).height,
      expandedAppBarHeight - collapsedAppBarHeight - 45,
    );

    // Scroll so that it is completely collapsed.
    controller.jumpTo(600);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), collapsedAppBarHeight);
    expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 1);
    expect(tester.getSize(expandedTitleClip).height, 0);

    // Scroll back to fully expanded.
    controller.jumpTo(0);
    await tester.pumpAndSettle();
    expect(find.byType(SliverAppBar), findsOneWidget);
    expect(appBarHeight(tester), expandedAppBarHeight);
    expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
    expect(tester.getSize(expandedTitleClip).height, expandedAppBarHeight - collapsedAppBarHeight);
  });

  group('SliverAppBar elevation', () {
    Widget buildSliverAppBar(bool forceElevated, {double? elevation, double? themeElevation}) {
      return MaterialApp(
        theme: ThemeData(
          appBarTheme: AppBarTheme(
            elevation: themeElevation,
            scrolledUnderElevation: themeElevation,
          ),
        ),
        home: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar(
              title: const Text('Title'),
              forceElevated: forceElevated,
              elevation: elevation,
              scrolledUnderElevation: elevation,
            ),
          ],
        ),
      );
    }

    testWidgets('Respects forceElevated parameter', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/59158.
      AppBar getAppBar() => tester.widget<AppBar>(find.byType(AppBar));
      Material getMaterial() => tester.widget<Material>(find.byType(Material));
      final bool useMaterial3 = ThemeData().useMaterial3;

      // When forceElevated is off, SliverAppBar should not be elevated.
      await tester.pumpWidget(buildSliverAppBar(false));
      expect(getMaterial().elevation, 0.0);

      // Default elevation should be used by the material, but
      // the AppBar's elevation should not be specified by SliverAppBar.
      // When useMaterial3 is true, and forceElevated is true, the default elevation
      // should be the value of `scrolledUnderElevation` which is 3.0
      await tester.pumpWidget(buildSliverAppBar(true));
      expect(getMaterial().elevation, useMaterial3 ? 3.0 : 4.0);
      expect(getAppBar().elevation, null);

      // SliverAppBar should use the specified elevation.
      await tester.pumpWidget(buildSliverAppBar(true, elevation: 8.0));
      expect(getMaterial().elevation, 8.0);
    });

    testWidgets('Uses elevation of AppBarTheme by default', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/73525.
      Material getMaterial() => tester.widget<Material>(find.byType(Material));

      await tester.pumpWidget(buildSliverAppBar(false, themeElevation: 12.0));
      expect(getMaterial().elevation, 0.0);

      await tester.pumpWidget(buildSliverAppBar(true, themeElevation: 12.0));
      expect(getMaterial().elevation, 12.0);

      await tester.pumpWidget(buildSliverAppBar(true, elevation: 8.0, themeElevation: 12.0));
      expect(getMaterial().elevation, 8.0);
    });
  });

  group('SliverAppBar.forceMaterialTransparency', () {
    Material getSliverAppBarMaterial(WidgetTester tester) {
      return tester.widget<Material>(
        find.descendant(of: find.byType(SliverAppBar), matching: find.byType(Material)).first,
      );
    }

    // Generates a MaterialApp with a SliverAppBar in a CustomScrollView.
    // The first cell of the scroll view contains a button at its top, and is
    // initially scrolled so that it is beneath the SliverAppBar.
    (ScrollController, Widget) buildWidget({
      required bool forceMaterialTransparency,
      required VoidCallback onPressed,
    }) {
      const double appBarHeight = 120;
      final ScrollController controller = ScrollController(initialScrollOffset: appBarHeight);

      return (
        controller,
        MaterialApp(
          home: Scaffold(
            body: CustomScrollView(
              controller: controller,
              slivers: <Widget>[
                SliverAppBar(
                  collapsedHeight: appBarHeight,
                  expandedHeight: appBarHeight,
                  pinned: true,
                  elevation: 0,
                  backgroundColor: Colors.transparent,
                  forceMaterialTransparency: forceMaterialTransparency,
                  title: const Text('AppBar'),
                ),
                SliverList(
                  delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
                    return SizedBox(
                      height: appBarHeight,
                      child:
                          index == 0
                              ? Align(
                                alignment: Alignment.topCenter,
                                child: TextButton(onPressed: onPressed, child: const Text('press')),
                              )
                              : const SizedBox(),
                    );
                  }, childCount: 20),
                ),
              ],
            ),
          ),
        ),
      );
    }

    testWidgets('forceMaterialTransparency == true allows gestures beneath the app bar', (
      WidgetTester tester,
    ) async {
      bool buttonWasPressed = false;
      final (ScrollController controller, Widget widget) = buildWidget(
        forceMaterialTransparency: true,
        onPressed: () {
          buttonWasPressed = true;
        },
      );
      await tester.pumpWidget(widget);

      final Material material = getSliverAppBarMaterial(tester);
      expect(material.type, MaterialType.transparency);

      final Finder buttonFinder = find.byType(TextButton);
      await tester.tap(buttonFinder);
      await tester.pump();
      expect(buttonWasPressed, isTrue);

      controller.dispose();
    });

    testWidgets('forceMaterialTransparency == false does not allow gestures beneath the app bar', (
      WidgetTester tester,
    ) async {
      // Set this, and tester.tap(warnIfMissed:false), to suppress
      // errors/warning that the button is not hittable (which is expected).
      WidgetController.hitTestWarningShouldBeFatal = false;

      bool buttonWasPressed = false;
      final (ScrollController controller, Widget widget) = buildWidget(
        forceMaterialTransparency: false,
        onPressed: () {
          buttonWasPressed = true;
        },
      );
      await tester.pumpWidget(widget);

      final Material material = getSliverAppBarMaterial(tester);
      expect(material.type, MaterialType.canvas);

      final Finder buttonFinder = find.byType(TextButton);
      await tester.tap(buttonFinder, warnIfMissed: false);
      await tester.pump();
      expect(buttonWasPressed, isFalse);

      controller.dispose();
    });
  });

  testWidgets('SliverAppBar positioning of leading and trailing widgets with top padding', (
    WidgetTester tester,
  ) async {
    const MediaQueryData topPadding100 = MediaQueryData(padding: EdgeInsets.only(top: 100.0));
    final Key leadingKey = UniqueKey();
    final Key titleKey = UniqueKey();
    final Key trailingKey = UniqueKey();

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: MediaQuery(
            data: topPadding100,
            child: CustomScrollView(
              primary: true,
              slivers: <Widget>[
                SliverAppBar(
                  leading: Placeholder(key: leadingKey),
                  title: Placeholder(key: titleKey, fallbackHeight: kToolbarHeight),
                  actions: <Widget>[Placeholder(key: trailingKey)],
                ),
              ],
            ),
          ),
        ),
      ),
    );
    expect(tester.getTopLeft(find.byType(AppBar)), Offset.zero);
    expect(tester.getTopLeft(find.byKey(leadingKey)), const Offset(800.0 - 56.0, 100.0));
    expect(tester.getTopLeft(find.byKey(titleKey)), const Offset(416.0, 100.0));
    expect(tester.getTopLeft(find.byKey(trailingKey)), const Offset(0.0, 100.0));
  });

  testWidgets('SliverAppBar positioning of leading and trailing widgets with bottom padding', (
    WidgetTester tester,
  ) async {
    const MediaQueryData topPadding100 = MediaQueryData(
      padding: EdgeInsets.only(top: 100.0, bottom: 50.0),
    );
    final Key leadingKey = UniqueKey();
    final Key titleKey = UniqueKey();
    final Key trailingKey = UniqueKey();

    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'US'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultMaterialLocalizations.delegate,
          DefaultWidgetsLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: MediaQuery(
            data: topPadding100,
            child: CustomScrollView(
              primary: true,
              slivers: <Widget>[
                SliverAppBar(
                  leading: Placeholder(key: leadingKey),
                  title: Placeholder(key: titleKey),
                  actions: <Widget>[Placeholder(key: trailingKey)],
                ),
              ],
            ),
          ),
        ),
      ),
    );
    expect(
      tester.getRect(find.byType(AppBar)),
      const Rect.fromLTRB(0.0, 0.0, 800.00, 100.0 + 56.0),
    );
    expect(
      tester.getRect(find.byKey(leadingKey)),
      const Rect.fromLTRB(800.0 - 56.0, 100.0, 800.0, 100.0 + 56.0),
    );
    expect(
      tester.getRect(find.byKey(trailingKey)),
      const Rect.fromLTRB(0.0, 100.0, 400.0, 100.0 + 56.0),
    );
  });

  testWidgets('SliverAppBar provides correct semantics in LTR', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: AppBar(
            leading: const Text('Leading'),
            title: const Text('Title'),
            actions: const <Widget>[Text('Action 1'), Text('Action 2'), Text('Action 3')],
            bottom: const PreferredSize(
              preferredSize: Size(0.0, kToolbarHeight),
              child: Text('Bottom'),
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(label: 'Leading', textDirection: TextDirection.ltr),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.namesRoute,
                                SemanticsFlag.isHeader,
                              ],
                              label: 'Title',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(label: 'Action 1', textDirection: TextDirection.ltr),
                            TestSemantics(label: 'Action 2', textDirection: TextDirection.ltr),
                            TestSemantics(label: 'Action 3', textDirection: TextDirection.ltr),
                            TestSemantics(label: 'Bottom', textDirection: TextDirection.ltr),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('SliverAppBar provides correct semantics in RTL', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Semantics(
          textDirection: TextDirection.rtl,
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: Center(
              child: AppBar(
                leading: const Text('Leading'),
                title: const Text('Title'),
                actions: const <Widget>[Text('Action 1'), Text('Action 2'), Text('Action 3')],
                bottom: const PreferredSize(
                  preferredSize: Size(0.0, kToolbarHeight),
                  child: Text('Bottom'),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          textDirection: TextDirection.rtl,
                          children: <TestSemantics>[
                            TestSemantics(
                              children: <TestSemantics>[
                                TestSemantics(label: 'Leading', textDirection: TextDirection.rtl),
                                TestSemantics(
                                  flags: <SemanticsFlag>[
                                    SemanticsFlag.namesRoute,
                                    SemanticsFlag.isHeader,
                                  ],
                                  label: 'Title',
                                  textDirection: TextDirection.rtl,
                                ),
                                TestSemantics(label: 'Action 1', textDirection: TextDirection.rtl),
                                TestSemantics(label: 'Action 2', textDirection: TextDirection.rtl),
                                TestSemantics(label: 'Action 3', textDirection: TextDirection.rtl),
                                TestSemantics(label: 'Bottom', textDirection: TextDirection.rtl),
                              ],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('SliverAppBar excludes header semantics correctly', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar(
              leading: Text('Leading'),
              flexibleSpace: ExcludeSemantics(child: Text('Title')),
              actions: <Widget>[Text('Action 1')],
              excludeHeaderSemantics: true,
            ),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(
                              children: <TestSemantics>[
                                TestSemantics(
                                  children: <TestSemantics>[
                                    TestSemantics(
                                      label: 'Leading',
                                      textDirection: TextDirection.ltr,
                                    ),
                                    TestSemantics(
                                      label: 'Action 1',
                                      textDirection: TextDirection.ltr,
                                    ),
                                  ],
                                ),
                                TestSemantics(),
                              ],
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('SliverAppBar with flexible space has correct semantics order', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/64922.
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar(
              leading: Text('Leading'),
              flexibleSpace: Text('Flexible space'),
              actions: <Widget>[Text('Action 1')],
            ),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(
                              children: <TestSemantics>[
                                TestSemantics(
                                  children: <TestSemantics>[
                                    TestSemantics(
                                      label: 'Leading',
                                      textDirection: TextDirection.ltr,
                                    ),
                                    TestSemantics(
                                      label: 'Action 1',
                                      textDirection: TextDirection.ltr,
                                    ),
                                  ],
                                ),
                                TestSemantics(
                                  children: <TestSemantics>[
                                    TestSemantics(
                                      flags: <SemanticsFlag>[SemanticsFlag.isHeader],
                                      label: 'Flexible space',
                                      textDirection: TextDirection.ltr,
                                    ),
                                  ],
                                ),
                              ],
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Changing SliverAppBar snap from true to false', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/17598
    const double appBarHeight = 256.0;
    bool snap = true;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: CustomScrollView(
                slivers: <Widget>[
                  SliverAppBar(
                    expandedHeight: appBarHeight,
                    floating: true,
                    snap: snap,
                    actions: <Widget>[
                      TextButton(
                        child: const Text('snap=false'),
                        onPressed: () {
                          setState(() {
                            snap = false;
                          });
                        },
                      ),
                    ],
                    flexibleSpace: FlexibleSpaceBar(
                      background: Container(height: appBarHeight, color: Colors.orange),
                    ),
                  ),
                  SliverList(
                    delegate: SliverChildListDelegate(<Widget>[
                      Container(height: 1200.0, color: Colors.teal),
                    ]),
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );

    TestGesture gesture = await tester.startGesture(const Offset(50.0, 400.0));
    await gesture.moveBy(const Offset(0.0, -100.0));
    await gesture.up();

    await tester.tap(find.text('snap=false'));
    await tester.pumpAndSettle();

    gesture = await tester.startGesture(const Offset(50.0, 400.0));
    await gesture.moveBy(const Offset(0.0, -100.0));
    await gesture.up();
    await tester.pump();
  });

  testWidgets('SliverAppBar shape default', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar(
              leading: Text('L'),
              title: Text('No Scaffold'),
              actions: <Widget>[Text('A1'), Text('A2')],
            ),
          ],
        ),
      ),
    );

    final Finder sliverAppBarFinder = find.byType(SliverAppBar);
    SliverAppBar getSliverAppBarWidget(Finder finder) => tester.widget<SliverAppBar>(finder);
    expect(getSliverAppBarWidget(sliverAppBarFinder).shape, null);

    final Finder materialFinder = find.byType(Material);
    Material getMaterialWidget(Finder finder) => tester.widget<Material>(finder);
    expect(getMaterialWidget(materialFinder).shape, null);
  });

  testWidgets('SliverAppBar with shape', (WidgetTester tester) async {
    const RoundedRectangleBorder roundedRectangleBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(15.0)),
    );
    await tester.pumpWidget(
      const MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar(
              leading: Text('L'),
              title: Text('No Scaffold'),
              actions: <Widget>[Text('A1'), Text('A2')],
              shape: roundedRectangleBorder,
            ),
          ],
        ),
      ),
    );

    final Finder sliverAppBarFinder = find.byType(SliverAppBar);
    SliverAppBar getSliverAppBarWidget(Finder finder) => tester.widget<SliverAppBar>(finder);
    expect(getSliverAppBarWidget(sliverAppBarFinder).shape, roundedRectangleBorder);

    final Finder materialFinder = find.byType(Material);
    Material getMaterialWidget(Finder finder) => tester.widget<Material>(finder);
    expect(getMaterialWidget(materialFinder).shape, roundedRectangleBorder);
  });

  testWidgets('SliverAppBar configures the delegate properly', (WidgetTester tester) async {
    Future<void> buildAndVerifyDelegate({
      required bool pinned,
      required bool floating,
      required bool snap,
    }) async {
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[
              SliverAppBar(
                title: const Text('Jumbo'),
                pinned: pinned,
                floating: floating,
                snap: snap,
              ),
            ],
          ),
        ),
      );

      final SliverPersistentHeaderDelegate delegate =
          tester.widget<SliverPersistentHeader>(find.byType(SliverPersistentHeader)).delegate;

      // Ensure we have a non-null vsync when it's needed.
      if (!floating ||
          (delegate.snapConfiguration == null && delegate.showOnScreenConfiguration == null)) {
        expect(delegate.vsync, isNotNull);
      }

      expect(delegate.showOnScreenConfiguration != null, snap && floating);
    }

    await buildAndVerifyDelegate(pinned: false, floating: true, snap: false);
    await buildAndVerifyDelegate(pinned: false, floating: true, snap: true);

    await buildAndVerifyDelegate(pinned: true, floating: true, snap: false);
    await buildAndVerifyDelegate(pinned: true, floating: true, snap: true);
  });

  testWidgets('SliverAppBar default collapsedHeight with respect to toolbarHeight', (
    WidgetTester tester,
  ) async {
    const double toolbarHeight = 100.0;

    await tester.pumpWidget(buildSliverAppBarApp(toolbarHeight: toolbarHeight));

    final ScrollController controller = primaryScrollController(tester);
    final double initialTabBarHeight = tabBarHeight(tester);

    // Scroll the not-pinned appbar out of view, to its collapsed height.
    controller.jumpTo(300.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsNothing);
    // By default, the collapsedHeight is toolbarHeight + bottom.preferredSize.height,
    // in this case initialTabBarHeight.
    expect(appBarHeight(tester), toolbarHeight + initialTabBarHeight);
  });

  testWidgets('SliverAppBar collapsedHeight with toolbarHeight', (WidgetTester tester) async {
    const double toolbarHeight = 100.0;
    const double collapsedHeight = 150.0;

    await tester.pumpWidget(
      buildSliverAppBarApp(toolbarHeight: toolbarHeight, collapsedHeight: collapsedHeight),
    );

    final ScrollController controller = primaryScrollController(tester);
    final double initialTabBarHeight = tabBarHeight(tester);

    // Scroll the not-pinned appbar out of view, to its collapsed height.
    controller.jumpTo(300.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsNothing);
    expect(appBarHeight(tester), collapsedHeight + initialTabBarHeight);
  });

  testWidgets('SliverAppBar collapsedHeight', (WidgetTester tester) async {
    const double collapsedHeight = 56.0;

    await tester.pumpWidget(buildSliverAppBarApp(collapsedHeight: collapsedHeight));

    final ScrollController controller = primaryScrollController(tester);
    final double initialTabBarHeight = tabBarHeight(tester);

    // Scroll the not-pinned appbar out of view, to its collapsed height.
    controller.jumpTo(300.0);
    await tester.pump();
    expect(find.byType(SliverAppBar), findsNothing);
    expect(appBarHeight(tester), collapsedHeight + initialTabBarHeight);
  });

  testWidgets('SliverAppBar respects leadingWidth', (WidgetTester tester) async {
    const Key key = Key('leading');
    await tester.pumpWidget(
      const MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar(leading: Placeholder(key: key), leadingWidth: 100, title: Text('Title')),
          ],
        ),
      ),
    );

    // By default toolbarHeight is 56.0.
    expect(tester.getRect(find.byKey(key)), const Rect.fromLTRB(0, 0, 100, 56));
  });

  testWidgets('SliverAppBar.titleSpacing defaults to NavigationToolbar.kMiddleSpacing', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(buildSliverAppBarApp());

    final NavigationToolbar navToolBar = tester.widget(find.byType(NavigationToolbar));
    expect(navToolBar.middleSpacing, NavigationToolbar.kMiddleSpacing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/158158.
  testWidgets('SliverAppBar should update TabBar before TabBar build', (WidgetTester tester) async {
    final List<Tab> tabs = <Tab>[];

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return DefaultTabController(
              length: tabs.length,
              child: Scaffold(
                body: CustomScrollView(
                  slivers: <Widget>[
                    SliverAppBar(
                      actions: <Widget>[
                        TextButton(
                          child: const Text('Add Tab'),
                          onPressed: () {
                            setState(() {
                              tabs.add(Tab(text: 'Tab ${tabs.length + 1}'));
                            });
                          },
                        ),
                      ],
                      bottom: TabBar(tabs: tabs),
                    ),
                  ],
                ),
              ),
            );
          },
        ),
      ),
    );

    // Initializes with zero tabs.
    expect(find.text('Tab 1'), findsNothing);
    expect(find.text('Tab 2'), findsNothing);

    // No crash after tabs added.
    await tester.tap(find.text('Add Tab'));
    await tester.pumpAndSettle();
    expect(find.text('Tab 1'), findsOneWidget);
    expect(find.text('Tab 2'), findsNothing);
    expect(tester.takeException(), isNull);
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('Material2 - SliverAppBar.medium defaults', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: false);
      const double collapsedAppBarHeight = 64;
      const double expandedAppBarHeight = 112;

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Scaffold(
            body: CustomScrollView(
              primary: true,
              slivers: <Widget>[
                const SliverAppBar.medium(title: Text('AppBar Title')),
                SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
              ],
            ),
          ),
        ),
      );

      final ScrollController controller = primaryScrollController(tester);
      // There are two widgets for the title. The first title is a larger version
      // that is shown at the bottom when the app bar is expanded. It scrolls under
      // the main row until it is completely hidden and then the first title is
      // faded in. The last is the title on the mainrow with the icons. It is
      // transparent when the app bar is expanded, and opaque when it is collapsed.
      final Finder expandedTitle = find.text('AppBar Title').first;
      final Finder expandedTitleClip = find.ancestor(
        of: expandedTitle,
        matching: find.byType(ClipRect),
      );
      final Finder collapsedTitle = find.text('AppBar Title').last;
      final Finder collapsedTitleOpacity = find.ancestor(
        of: collapsedTitle,
        matching: find.byType(AnimatedOpacity),
      );

      // Default, fully expanded app bar.
      expect(controller.offset, 0);
      expect(find.byType(SliverAppBar), findsOneWidget);
      expect(appBarHeight(tester), expandedAppBarHeight);
      expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
      expect(
        tester.getSize(expandedTitleClip).height,
        expandedAppBarHeight - collapsedAppBarHeight,
      );

      // Test the expanded title is positioned correctly.
      final Offset titleOffset = tester.getBottomLeft(expandedTitle);
      expect(titleOffset, const Offset(16.0, 92.0));

      // Test the expanded title default color.
      expect(
        tester.renderObject<RenderParagraph>(expandedTitle).text.style!.color,
        theme.colorScheme.onPrimary,
      );

      // Scroll the expanded app bar partially out of view.
      controller.jumpTo(45);
      await tester.pump();
      expect(find.byType(SliverAppBar), findsOneWidget);
      expect(appBarHeight(tester), expandedAppBarHeight - 45);
      expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
      expect(
        tester.getSize(expandedTitleClip).height,
        expandedAppBarHeight - collapsedAppBarHeight - 45,
      );

      // Scroll so that it is completely collapsed.
      controller.jumpTo(600);
      await tester.pump();
      expect(find.byType(SliverAppBar), findsOneWidget);
      expect(appBarHeight(tester), collapsedAppBarHeight);
      expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 1);
      expect(tester.getSize(expandedTitleClip).height, 0);

      // Scroll back to fully expanded.
      controller.jumpTo(0);
      await tester.pumpAndSettle();
      expect(find.byType(SliverAppBar), findsOneWidget);
      expect(appBarHeight(tester), expandedAppBarHeight);
      expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
      expect(
        tester.getSize(expandedTitleClip).height,
        expandedAppBarHeight - collapsedAppBarHeight,
      );
    });

    testWidgets('Material2 - SliverAppBar.large defaults', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: false);
      const double collapsedAppBarHeight = 64;
      const double expandedAppBarHeight = 152;

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Scaffold(
            body: CustomScrollView(
              primary: true,
              slivers: <Widget>[
                const SliverAppBar.large(title: Text('AppBar Title')),
                SliverToBoxAdapter(child: Container(height: 1200, color: Colors.orange[400])),
              ],
            ),
          ),
        ),
      );

      final ScrollController controller = primaryScrollController(tester);
      // There are two widgets for the title. The first title is a larger version
      // that is shown at the bottom when the app bar is expanded. It scrolls under
      // the main row until it is completely hidden and then the first title is
      // faded in. The last is the title on the mainrow with the icons. It is
      // transparent when the app bar is expanded, and opaque when it is collapsed.
      final Finder expandedTitle = find.text('AppBar Title').first;
      final Finder expandedTitleClip = find.ancestor(
        of: expandedTitle,
        matching: find.byType(ClipRect),
      );
      final Finder collapsedTitle = find.text('AppBar Title').last;
      final Finder collapsedTitleOpacity = find.ancestor(
        of: collapsedTitle,
        matching: find.byType(AnimatedOpacity),
      );

      // Default, fully expanded app bar.
      expect(controller.offset, 0);
      expect(find.byType(SliverAppBar), findsOneWidget);
      expect(appBarHeight(tester), expandedAppBarHeight);
      expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
      expect(
        tester.getSize(expandedTitleClip).height,
        expandedAppBarHeight - collapsedAppBarHeight,
      );

      // Test the expanded title is positioned correctly.
      final Offset titleOffset = tester.getBottomLeft(expandedTitle);
      expect(titleOffset, const Offset(16.0, 124.0));

      // Test the expanded title default color.
      expect(
        tester.renderObject<RenderParagraph>(expandedTitle).text.style!.color,
        theme.colorScheme.onPrimary,
      );

      // Scroll the expanded app bar partially out of view.
      controller.jumpTo(45);
      await tester.pump();
      expect(find.byType(SliverAppBar), findsOneWidget);
      expect(appBarHeight(tester), expandedAppBarHeight - 45);
      expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
      expect(
        tester.getSize(expandedTitleClip).height,
        expandedAppBarHeight - collapsedAppBarHeight - 45,
      );

      // Scroll so that it is completely collapsed.
      controller.jumpTo(600);
      await tester.pump();
      expect(find.byType(SliverAppBar), findsOneWidget);
      expect(appBarHeight(tester), collapsedAppBarHeight);
      expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 1);
      expect(tester.getSize(expandedTitleClip).height, 0);

      // Scroll back to fully expanded.
      controller.jumpTo(0);
      await tester.pumpAndSettle();
      expect(find.byType(SliverAppBar), findsOneWidget);
      expect(appBarHeight(tester), expandedAppBarHeight);
      expect(tester.widget<AnimatedOpacity>(collapsedTitleOpacity).opacity, 0);
      expect(
        tester.getSize(expandedTitleClip).height,
        expandedAppBarHeight - collapsedAppBarHeight,
      );
    });
  });
}
