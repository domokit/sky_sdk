// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../widgets/semantics_tester.dart';

// From bottom_sheet.dart.
const Duration _bottomSheetExitDuration = Duration(milliseconds: 200);

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/103741
  testWidgets('extendBodyBehindAppBar change should not cause the body widget lose state', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    Widget buildFrame({required bool extendBodyBehindAppBar}) {
      return MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Scaffold(
            extendBodyBehindAppBar: extendBodyBehindAppBar,
            resizeToAvoidBottomInset: false,
            body: SingleChildScrollView(
              controller: controller,
              child: const FlutterLogo(size: 1107),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: true));
    expect(controller.position.pixels, 0.0);

    controller.jumpTo(100.0);
    await tester.pump();
    expect(controller.position.pixels, 100.0);

    await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: false));
    expect(controller.position.pixels, 100.0);
  });

  testWidgets('Scaffold drawer callback test', (WidgetTester tester) async {
    bool isDrawerOpen = false;
    bool isEndDrawerOpen = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          drawer: Container(color: Colors.blue),
          onDrawerChanged: (bool isOpen) {
            isDrawerOpen = isOpen;
          },
          endDrawer: Container(color: Colors.green),
          onEndDrawerChanged: (bool isOpen) {
            isEndDrawerOpen = isOpen;
          },
          body: Container(),
        ),
      ),
    );

    final ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));

    scaffoldState.openDrawer();
    await tester.pumpAndSettle();
    expect(isDrawerOpen, true);
    scaffoldState.openEndDrawer();
    await tester.pumpAndSettle();
    expect(isDrawerOpen, false);

    scaffoldState.openEndDrawer();
    await tester.pumpAndSettle();
    expect(isEndDrawerOpen, true);
    scaffoldState.openDrawer();
    await tester.pumpAndSettle();
    expect(isEndDrawerOpen, false);
  });

  testWidgets('Scaffold drawer callback test - only call when changed', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/87914
    bool onDrawerChangedCalled = false;
    bool onEndDrawerChangedCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          drawer: Container(color: Colors.blue),
          onDrawerChanged: (bool isOpen) {
            onDrawerChangedCalled = true;
          },
          endDrawer: Container(color: Colors.green),
          onEndDrawerChanged: (bool isOpen) {
            onEndDrawerChangedCalled = true;
          },
          body: Container(),
        ),
      ),
    );

    await tester.flingFrom(Offset.zero, const Offset(10.0, 0.0), 10.0);
    expect(onDrawerChangedCalled, false);

    await tester.pumpAndSettle();

    final double width = tester.getSize(find.byType(MaterialApp)).width;
    await tester.flingFrom(Offset(width - 1, 0.0), const Offset(-10.0, 0.0), 10.0);
    await tester.pumpAndSettle();
    expect(onEndDrawerChangedCalled, false);
  });

  testWidgets('Scaffold control test', (WidgetTester tester) async {
    final Key bodyKey = UniqueKey();
    Widget boilerplate(Widget child) {
      return Localizations(
        locale: const Locale('en', 'us'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultWidgetsLocalizations.delegate,
          DefaultMaterialLocalizations.delegate,
        ],
        child: Directionality(textDirection: TextDirection.ltr, child: child),
      );
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(appBar: AppBar(title: const Text('Title')), body: Container(key: bodyKey)),
      ),
    );
    RenderBox bodyBox = tester.renderObject(find.byKey(bodyKey));
    expect(bodyBox.size, equals(const Size(800.0, 544.0)));

    await tester.pumpWidget(
      boilerplate(
        MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 100.0)),
          child: Scaffold(
            appBar: AppBar(title: const Text('Title')),
            body: Container(key: bodyKey),
          ),
        ),
      ),
    );

    bodyBox = tester.renderObject(find.byKey(bodyKey));
    expect(bodyBox.size, equals(const Size(800.0, 444.0)));

    await tester.pumpWidget(
      boilerplate(
        MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 100.0)),
          child: Scaffold(
            appBar: AppBar(title: const Text('Title')),
            body: Container(key: bodyKey),
            resizeToAvoidBottomInset: false,
          ),
        ),
      ),
    );

    bodyBox = tester.renderObject(find.byKey(bodyKey));
    expect(bodyBox.size, equals(const Size(800.0, 544.0)));
  });

  testWidgets('Scaffold large bottom padding test', (WidgetTester tester) async {
    final Key bodyKey = UniqueKey();

    Widget boilerplate(Widget child) {
      return Localizations(
        locale: const Locale('en', 'us'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultWidgetsLocalizations.delegate,
          DefaultMaterialLocalizations.delegate,
        ],
        child: Directionality(textDirection: TextDirection.ltr, child: child),
      );
    }

    await tester.pumpWidget(
      boilerplate(
        MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 700.0)),
          child: Scaffold(body: Container(key: bodyKey)),
        ),
      ),
    );

    final RenderBox bodyBox = tester.renderObject(find.byKey(bodyKey));
    expect(bodyBox.size, equals(const Size(800.0, 0.0)));

    await tester.pumpWidget(
      boilerplate(
        MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 500.0)),
          child: Scaffold(body: Container(key: bodyKey)),
        ),
      ),
    );

    expect(bodyBox.size, equals(const Size(800.0, 100.0)));

    await tester.pumpWidget(
      boilerplate(
        MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 580.0)),
          child: Scaffold(
            appBar: AppBar(title: const Text('Title')),
            body: Container(key: bodyKey),
          ),
        ),
      ),
    );

    expect(bodyBox.size, equals(const Size(800.0, 0.0)));
  });

  testWidgets('Floating action entrance/exit animation', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          floatingActionButton: FloatingActionButton(
            key: Key('one'),
            onPressed: null,
            child: Text('1'),
          ),
        ),
      ),
    );

    expect(tester.binding.transientCallbackCount, 0);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          floatingActionButton: FloatingActionButton(
            key: Key('two'),
            onPressed: null,
            child: Text('2'),
          ),
        ),
      ),
    );

    expect(tester.binding.transientCallbackCount, greaterThan(0));
    await tester.pumpWidget(Container());
    expect(tester.binding.transientCallbackCount, 0);

    await tester.pumpWidget(const MaterialApp(home: Scaffold()));

    expect(tester.binding.transientCallbackCount, 0);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          floatingActionButton: FloatingActionButton(
            key: Key('one'),
            onPressed: null,
            child: Text('1'),
          ),
        ),
      ),
    );

    expect(tester.binding.transientCallbackCount, greaterThan(0));
  });

  testWidgets('Floating action button shrinks when bottom sheet becomes dominant', (
    WidgetTester tester,
  ) async {
    final DraggableScrollableController draggableController = DraggableScrollableController();
    addTearDown(draggableController.dispose);
    const double kBottomSheetDominatesPercentage = 0.3;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          floatingActionButton: const FloatingActionButton(
            key: Key('one'),
            onPressed: null,
            child: Text('1'),
          ),
          bottomSheet: DraggableScrollableSheet(
            expand: false,
            controller: draggableController,
            builder: (BuildContext context, ScrollController scrollController) {
              return SingleChildScrollView(controller: scrollController, child: const SizedBox());
            },
          ),
        ),
      ),
    );

    double getScale() =>
        tester.firstWidget<ScaleTransition>(find.byType(ScaleTransition)).scale.value;

    for (double i = 0, extent = i / 10; i <= 10; i++, extent = i / 10) {
      draggableController.jumpTo(extent);

      final double extentRemaining = 1.0 - extent;
      if (extentRemaining < kBottomSheetDominatesPercentage) {
        final double visValue = extentRemaining * kBottomSheetDominatesPercentage * 10;
        // since FAB uses easeIn curve, we're testing this by using the fact that
        // easeIn curve is always less than or equal to x=y curve.
        expect(getScale(), lessThanOrEqualTo(visValue));
      } else {
        expect(getScale(), equals(1.0));
      }
    }
  });

  testWidgets('Scaffold shows scrim when bottom sheet becomes dominant', (
    WidgetTester tester,
  ) async {
    final DraggableScrollableController draggableController = DraggableScrollableController();
    addTearDown(draggableController.dispose);
    const double kBottomSheetDominatesPercentage = 0.3;
    const double kMinBottomSheetScrimOpacity = 0.1;
    const double kMaxBottomSheetScrimOpacity = 0.6;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomSheet: DraggableScrollableSheet(
            expand: false,
            controller: draggableController,
            builder: (BuildContext context, ScrollController scrollController) {
              return SingleChildScrollView(controller: scrollController, child: const SizedBox());
            },
          ),
        ),
      ),
    );

    Finder findModalBarrier() =>
        find.descendant(of: find.byType(Scaffold), matching: find.byType(ModalBarrier));
    double getOpacity() => tester.firstWidget<ModalBarrier>(findModalBarrier()).color!.opacity;
    double getExpectedOpacity(double visValue) =>
        math.max(kMinBottomSheetScrimOpacity, kMaxBottomSheetScrimOpacity - visValue);

    for (double i = 0, extent = i / 10; i <= 10; i++, extent = i / 10) {
      draggableController.jumpTo(extent);
      await tester.pump();

      final double extentRemaining = 1.0 - extent;
      if (extentRemaining < kBottomSheetDominatesPercentage) {
        final double visValue = extentRemaining * kBottomSheetDominatesPercentage * 10;

        expect(findModalBarrier(), findsOneWidget);
        expect(getOpacity(), moreOrLessEquals(getExpectedOpacity(visValue), epsilon: 0.02));
      } else {
        expect(findModalBarrier(), findsNothing);
      }
    }
  });

  testWidgets('Floating action button directionality', (WidgetTester tester) async {
    Widget build(TextDirection textDirection) {
      return Directionality(
        textDirection: textDirection,
        child: const MediaQuery(
          data: MediaQueryData(viewInsets: EdgeInsets.only(bottom: 200.0)),
          child: Scaffold(
            floatingActionButton: FloatingActionButton(onPressed: null, child: Text('1')),
          ),
        ),
      );
    }

    await tester.pumpWidget(build(TextDirection.ltr));

    expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(756.0, 356.0));

    await tester.pumpWidget(build(TextDirection.rtl));
    expect(tester.binding.transientCallbackCount, 0);

    expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(44.0, 356.0));
  });

  testWidgets('Floating Action Button bottom padding not consumed by viewInsets', (
    WidgetTester tester,
  ) async {
    final Widget child = Directionality(
      textDirection: TextDirection.ltr,
      child: Scaffold(
        resizeToAvoidBottomInset: false,
        body: Container(),
        floatingActionButton: const Placeholder(),
      ),
    );

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(viewPadding: EdgeInsets.only(bottom: 20.0)),
        child: child,
      ),
    );
    final Offset initialPoint = tester.getCenter(find.byType(Placeholder));
    expect(
      tester.getBottomLeft(find.byType(Placeholder)).dy,
      moreOrLessEquals(600.0 - 20.0 - kFloatingActionButtonMargin),
    );

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
  });

  testWidgets('viewPadding change should trigger _ScaffoldLayout re-layout', (
    WidgetTester tester,
  ) async {
    Widget buildFrame(EdgeInsets viewPadding) {
      return MediaQuery(
        data: MediaQueryData(viewPadding: viewPadding),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Scaffold(
            resizeToAvoidBottomInset: false,
            body: Container(),
            floatingActionButton: const Placeholder(),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(const EdgeInsets.only(bottom: 300)));

    final RenderBox renderBox = tester.renderObject<RenderBox>(find.byType(CustomMultiChildLayout));
    expect(renderBox.debugNeedsLayout, false);

    await tester.pumpWidget(
      buildFrame(const EdgeInsets.only(bottom: 400)),
      phase: EnginePhase.build,
    );

    expect(renderBox.debugNeedsLayout, true);
  });

  testWidgets('Drawer scrolling', (WidgetTester tester) async {
    final Key drawerKey = UniqueKey();
    const double appBarHeight = 256.0;

    final ScrollController scrollOffset = ScrollController();
    addTearDown(scrollOffset.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          drawer: Drawer(
            key: drawerKey,
            child: ListView(
              dragStartBehavior: DragStartBehavior.down,
              controller: scrollOffset,
              children: List<Widget>.generate(
                10,
                (int index) => SizedBox(height: 100.0, child: Text('D$index')),
              ),
            ),
          ),
          body: CustomScrollView(
            slivers: <Widget>[
              const SliverAppBar(
                pinned: true,
                expandedHeight: appBarHeight,
                title: Text('Title'),
                flexibleSpace: FlexibleSpaceBar(title: Text('Title')),
              ),
              SliverPadding(
                padding: const EdgeInsets.only(top: appBarHeight),
                sliver: SliverList(
                  delegate: SliverChildListDelegate(
                    List<Widget>.generate(
                      10,
                      (int index) => SizedBox(height: 100.0, child: Text('B$index')),
                    ),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );

    final ScaffoldState state = tester.firstState(find.byType(Scaffold));
    state.openDrawer();

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(scrollOffset.offset, 0.0);

    const double scrollDelta = 80.0;
    await tester.drag(find.byKey(drawerKey), const Offset(0.0, -scrollDelta));
    await tester.pump();

    expect(scrollOffset.offset, scrollDelta);

    final RenderBox renderBox = tester.renderObject(find.byType(AppBar));
    expect(renderBox.size.height, equals(appBarHeight));
  });

  Widget buildStatusBarTestApp(TargetPlatform? platform) {
    return MaterialApp(
      theme: ThemeData(platform: platform),
      home: MediaQuery(
        data: const MediaQueryData(padding: EdgeInsets.only(top: 25.0)), // status bar
        child: Scaffold(
          body: CustomScrollView(
            primary: true,
            slivers: <Widget>[
              const SliverAppBar(title: Text('Title')),
              SliverList(
                delegate: SliverChildListDelegate(
                  List<Widget>.generate(
                    20,
                    (int index) => SizedBox(height: 100.0, child: Text('$index')),
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  testWidgets(
    'Tapping the status bar scrolls to top',
    (WidgetTester tester) async {
      await tester.pumpWidget(buildStatusBarTestApp(debugDefaultTargetPlatformOverride));
      final ScrollableState scrollable = tester.state(find.byType(Scrollable));
      scrollable.position.jumpTo(500.0);
      expect(scrollable.position.pixels, equals(500.0));
      await tester.tapAt(const Offset(100.0, 10.0));
      await tester.pumpAndSettle();
      expect(scrollable.position.pixels, equals(0.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Tapping the status bar scrolls to top with ease out curve animation',
    (WidgetTester tester) async {
      const int duration = 1000;
      final List<double> stops = <double>[0.842, 0.959, 0.993, 1.0];
      const double scrollOffset = 1000;

      await tester.pumpWidget(buildStatusBarTestApp(debugDefaultTargetPlatformOverride));
      final ScrollableState scrollable = tester.state(find.byType(Scrollable));
      scrollable.position.jumpTo(scrollOffset);
      await tester.tapAt(const Offset(100.0, 10.0));

      await tester.pump(Duration.zero);
      expect(scrollable.position.pixels, equals(scrollOffset));

      for (int i = 0; i < stops.length; i++) {
        await tester.pump(Duration(milliseconds: duration ~/ stops.length));
        // Scroll pixel position is very long double, compare with floored int
        // pixel position
        expect(scrollable.position.pixels.toInt(), equals((scrollOffset * (1 - stops[i])).toInt()));
      }

      // Finally stops at the top.
      expect(scrollable.position.pixels, equals(0.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Tapping the status bar does not scroll to top',
    (WidgetTester tester) async {
      await tester.pumpWidget(buildStatusBarTestApp(TargetPlatform.android));
      final ScrollableState scrollable = tester.state(find.byType(Scrollable));
      scrollable.position.jumpTo(500.0);
      expect(scrollable.position.pixels, equals(500.0));
      await tester.tapAt(const Offset(100.0, 10.0));
      await tester.pump();
      await tester.pump(const Duration(seconds: 1));
      expect(scrollable.position.pixels, equals(500.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );

  testWidgets('Bottom sheet cannot overlap app bar', (WidgetTester tester) async {
    final Key sheetKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: Scaffold(
          appBar: AppBar(title: const Text('Title')),
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  Scaffold.of(context).showBottomSheet((BuildContext context) {
                    return Container(key: sheetKey, color: Colors.blue[500]);
                  });
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(seconds: 1));

    final RenderBox appBarBox = tester.renderObject(find.byType(AppBar));
    final RenderBox sheetBox = tester.renderObject(find.byKey(sheetKey));

    final Offset appBarBottomRight = appBarBox.localToGlobal(
      appBarBox.size.bottomRight(Offset.zero),
    );
    final Offset sheetTopRight = sheetBox.localToGlobal(sheetBox.size.topRight(Offset.zero));

    expect(appBarBottomRight, equals(sheetTopRight));
  });

  testWidgets('BottomSheet bottom padding is not consumed by viewInsets', (
    WidgetTester tester,
  ) async {
    final Widget child = Directionality(
      textDirection: TextDirection.ltr,
      child: Scaffold(
        resizeToAvoidBottomInset: false,
        body: Container(),
        bottomSheet: const Placeholder(),
      ),
    );

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(padding: EdgeInsets.only(bottom: 20.0)), child: child),
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
  });

  testWidgets('Persistent bottom buttons are persistent', (WidgetTester tester) async {
    bool didPressButton = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: Container(color: Colors.amber[500], height: 5000.0, child: const Text('body')),
          ),
          persistentFooterButtons: <Widget>[
            TextButton(
              onPressed: () {
                didPressButton = true;
              },
              child: const Text('X'),
            ),
          ],
        ),
      ),
    );

    await tester.drag(find.byType(SingleChildScrollView), const Offset(0.0, -1000.0));
    expect(didPressButton, isFalse);
    await tester.tap(find.text('X'));
    expect(didPressButton, isTrue);
  });

  testWidgets('Persistent bottom buttons alignment', (WidgetTester tester) async {
    Widget buildApp(AlignmentDirectional persistentAlignment) {
      return MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: Container(color: Colors.amber[500], height: 5000.0, child: const Text('body')),
          ),
          persistentFooterAlignment: persistentAlignment,
          persistentFooterButtons: <Widget>[TextButton(onPressed: () {}, child: const Text('X'))],
        ),
      );
    }

    await tester.pumpWidget(buildApp(AlignmentDirectional.centerEnd));
    Finder footerButton = find.byType(TextButton);
    expect(tester.getTopRight(footerButton).dx, 800.0 - 8.0);

    await tester.pumpWidget(buildApp(AlignmentDirectional.center));
    footerButton = find.byType(TextButton);
    expect(tester.getCenter(footerButton).dx, 800.0 / 2);

    await tester.pumpWidget(buildApp(AlignmentDirectional.centerStart));
    footerButton = find.byType(TextButton);
    expect(tester.getTopLeft(footerButton).dx, 8.0);
  });

  testWidgets('Persistent bottom buttons apply media padding', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.fromLTRB(10.0, 20.0, 30.0, 40.0)),
          child: Scaffold(
            body: SingleChildScrollView(
              child: Container(color: Colors.amber[500], height: 5000.0, child: const Text('body')),
            ),
            persistentFooterButtons: const <Widget>[Placeholder()],
          ),
        ),
      ),
    );

    final Finder buttonsBar =
        find.ancestor(of: find.byType(OverflowBar), matching: find.byType(Padding)).first;
    expect(tester.getBottomLeft(buttonsBar), const Offset(10.0, 560.0));
    expect(tester.getBottomRight(buttonsBar), const Offset(770.0, 560.0));
  });

  testWidgets('persistentFooterButtons with bottomNavigationBar apply SafeArea properly', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/pull/92039
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: MediaQuery(
          data: const MediaQueryData(
            // Representing a navigational notch at the bottom of the screen
            viewPadding: EdgeInsets.fromLTRB(0.0, 0.0, 0.0, 40.0),
          ),
          child: Scaffold(
            body: SingleChildScrollView(
              child: Container(color: Colors.amber[500], height: 5000.0, child: const Text('body')),
            ),
            bottomNavigationBar: BottomNavigationBar(
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.home), label: 'Home'),
                BottomNavigationBarItem(icon: Icon(Icons.business), label: 'Business'),
                BottomNavigationBarItem(icon: Icon(Icons.school), label: 'School'),
              ],
            ),
            persistentFooterButtons: const <Widget>[Placeholder()],
          ),
        ),
      ),
    );

    final Finder buttonsBar =
        find.ancestor(of: find.byType(OverflowBar), matching: find.byType(Padding)).first;
    // The SafeArea of the persistentFooterButtons should not pad below them
    // since they are stacked on top of the bottomNavigationBar. The
    // bottomNavigationBar will handle the padding instead.
    // 488 represents the height of the persistentFooterButtons, with the bottom
    // of the screen being 600. If the 40 pixels of bottom padding were being
    // errantly applied, the buttons would be higher (448).
    expect(tester.getTopLeft(buttonsBar), const Offset(0.0, 488.0));
  });

  testWidgets('Persistent bottom buttons bottom padding is not consumed by viewInsets', (
    WidgetTester tester,
  ) async {
    final Widget child = Directionality(
      textDirection: TextDirection.ltr,
      child: Scaffold(
        resizeToAvoidBottomInset: false,
        body: Container(),
        persistentFooterButtons: const <Widget>[Placeholder()],
      ),
    );

    await tester.pumpWidget(
      MediaQuery(data: const MediaQueryData(padding: EdgeInsets.only(bottom: 20.0)), child: child),
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
  });

  group('back arrow', () {
    Future<void> expectBackIcon(WidgetTester tester, IconData expectedIcon) async {
      final GlobalKey rootKey = GlobalKey();
      final Map<String, WidgetBuilder> routes = <String, WidgetBuilder>{
        '/': (_) => Container(key: rootKey, child: const Text('Home')),
        '/scaffold': (_) => Scaffold(appBar: AppBar(), body: const Text('Scaffold')),
      };
      await tester.pumpWidget(MaterialApp(routes: routes));

      Navigator.pushNamed(rootKey.currentContext!, '/scaffold');
      await tester.pump();
      await tester.pump(const Duration(seconds: 1));

      final Icon icon = tester.widget(find.byType(Icon));
      expect(icon.icon, expectedIcon);
    }

    testWidgets(
      'Back arrow uses correct default',
      (WidgetTester tester) async {
        await expectBackIcon(tester, Icons.arrow_back);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.fuchsia,
      }),
    );

    testWidgets(
      'Back arrow uses correct default',
      (WidgetTester tester) async {
        await expectBackIcon(tester, kIsWeb ? Icons.arrow_back : Icons.arrow_back_ios_new_rounded);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );
  });

  group('close button', () {
    Future<void> expectCloseIcon(
      WidgetTester tester,
      PageRoute<void> Function() routeBuilder,
      String type,
    ) async {
      const IconData expectedIcon = Icons.close;
      await tester.pumpWidget(
        MaterialApp(home: Scaffold(appBar: AppBar(), body: const Text('Page 1'))),
      );

      tester.state<NavigatorState>(find.byType(Navigator)).push(routeBuilder());

      await tester.pump();
      await tester.pump(const Duration(seconds: 1));

      final Icon icon = tester.widget(find.byType(Icon));
      expect(icon.icon, expectedIcon, reason: "didn't find close icon for $type");
      expect(
        find.byKey(StandardComponentType.closeButton.key),
        findsOneWidget,
        reason: "didn't find close button for $type",
      );
    }

    PageRoute<void> materialRouteBuilder() {
      return MaterialPageRoute<void>(
        builder: (BuildContext context) {
          return Scaffold(appBar: AppBar(), body: const Text('Page 2'));
        },
        fullscreenDialog: true,
      );
    }

    PageRoute<void> pageRouteBuilder() {
      return PageRouteBuilder<void>(
        pageBuilder: (
          BuildContext context,
          Animation<double> animation,
          Animation<double> secondaryAnimation,
        ) {
          return Scaffold(appBar: AppBar(), body: const Text('Page 2'));
        },
        fullscreenDialog: true,
      );
    }

    PageRoute<void> customPageRouteBuilder() {
      return _CustomPageRoute<void>(
        builder: (BuildContext context) {
          return Scaffold(appBar: AppBar(), body: const Text('Page 2'));
        },
        fullscreenDialog: true,
      );
    }

    testWidgets('Close button shows correctly', (WidgetTester tester) async {
      await expectCloseIcon(tester, materialRouteBuilder, 'materialRouteBuilder');
    }, variant: TargetPlatformVariant.all());

    testWidgets('Close button shows correctly with PageRouteBuilder', (WidgetTester tester) async {
      await expectCloseIcon(tester, pageRouteBuilder, 'pageRouteBuilder');
    }, variant: TargetPlatformVariant.all());

    testWidgets('Close button shows correctly with custom page route', (WidgetTester tester) async {
      await expectCloseIcon(tester, customPageRouteBuilder, 'customPageRouteBuilder');
    }, variant: TargetPlatformVariant.all());
  });

  group('body size', () {
    testWidgets('body size with container', (WidgetTester tester) async {
      final Key testKey = UniqueKey();
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: Scaffold(body: Container(key: testKey)),
          ),
        ),
      );
      expect(tester.element(find.byKey(testKey)).size, const Size(800.0, 600.0));
      expect(
        tester.renderObject<RenderBox>(find.byKey(testKey)).localToGlobal(Offset.zero),
        Offset.zero,
      );
    });

    testWidgets('body size with sized container', (WidgetTester tester) async {
      final Key testKey = UniqueKey();
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: Scaffold(body: Container(key: testKey, height: 100.0)),
          ),
        ),
      );
      expect(tester.element(find.byKey(testKey)).size, const Size(800.0, 100.0));
      expect(
        tester.renderObject<RenderBox>(find.byKey(testKey)).localToGlobal(Offset.zero),
        Offset.zero,
      );
    });

    testWidgets('body size with centered container', (WidgetTester tester) async {
      final Key testKey = UniqueKey();
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: Scaffold(body: Center(child: Container(key: testKey))),
          ),
        ),
      );
      expect(tester.element(find.byKey(testKey)).size, const Size(800.0, 600.0));
      expect(
        tester.renderObject<RenderBox>(find.byKey(testKey)).localToGlobal(Offset.zero),
        Offset.zero,
      );
    });

    testWidgets('body size with button', (WidgetTester tester) async {
      final Key testKey = UniqueKey();
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: Scaffold(
              body: TextButton(key: testKey, onPressed: () {}, child: const Text('')),
            ),
          ),
        ),
      );
      expect(tester.element(find.byKey(testKey)).size, const Size(64.0, 48.0));
      expect(
        tester.renderObject<RenderBox>(find.byKey(testKey)).localToGlobal(Offset.zero),
        Offset.zero,
      );
    });

    testWidgets('body size with extendBody', (WidgetTester tester) async {
      final Key bodyKey = UniqueKey();
      late double mediaQueryBottom;

      Widget buildFrame({
        required bool extendBody,
        bool? resizeToAvoidBottomInset,
        double viewInsetBottom = 0.0,
      }) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: MediaQuery(
            data: MediaQueryData(viewInsets: EdgeInsets.only(bottom: viewInsetBottom)),
            child: Scaffold(
              resizeToAvoidBottomInset: resizeToAvoidBottomInset,
              extendBody: extendBody,
              body: Builder(
                builder: (BuildContext context) {
                  mediaQueryBottom = MediaQuery.paddingOf(context).bottom;
                  return Container(key: bodyKey);
                },
              ),
              bottomNavigationBar: const BottomAppBar(child: SizedBox(height: 48.0)),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(extendBody: true));
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));
      expect(mediaQueryBottom, 48.0);

      await tester.pumpWidget(buildFrame(extendBody: false));
      expect(
        tester.getSize(find.byKey(bodyKey)),
        const Size(800.0, 552.0),
      ); // 552 = 600 - 48 (BAB height)
      expect(mediaQueryBottom, 0.0);

      // If resizeToAvoidBottomInsets is false, same results as if it was unspecified (null).
      await tester.pumpWidget(
        buildFrame(extendBody: true, resizeToAvoidBottomInset: false, viewInsetBottom: 100.0),
      );
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));
      expect(mediaQueryBottom, 48.0);

      await tester.pumpWidget(
        buildFrame(extendBody: false, resizeToAvoidBottomInset: false, viewInsetBottom: 100.0),
      );
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 552.0));
      expect(mediaQueryBottom, 0.0);

      // If resizeToAvoidBottomInsets is true and viewInsets.bottom is > the bottom
      // navigation bar's height then the body always resizes and the MediaQuery
      // isn't adjusted. This case corresponds to the keyboard appearing.
      await tester.pumpWidget(
        buildFrame(extendBody: true, resizeToAvoidBottomInset: true, viewInsetBottom: 100.0),
      );
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 500.0));
      expect(mediaQueryBottom, 0.0);

      await tester.pumpWidget(
        buildFrame(extendBody: false, resizeToAvoidBottomInset: true, viewInsetBottom: 100.0),
      );
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 500.0));
      expect(mediaQueryBottom, 0.0);
    });

    testWidgets('body size with extendBodyBehindAppBar', (WidgetTester tester) async {
      final Key appBarKey = UniqueKey();
      final Key bodyKey = UniqueKey();

      const double appBarHeight = 100;
      const double windowPaddingTop = 24;
      late bool fixedHeightAppBar;
      late double mediaQueryTop;

      Widget buildFrame({required bool extendBodyBehindAppBar, required bool hasAppBar}) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.only(top: windowPaddingTop)),
            child: Builder(
              builder: (BuildContext context) {
                return Scaffold(
                  extendBodyBehindAppBar: extendBodyBehindAppBar,
                  appBar:
                      !hasAppBar
                          ? null
                          : PreferredSize(
                            key: appBarKey,
                            preferredSize: const Size.fromHeight(appBarHeight),
                            child: Container(
                              constraints: BoxConstraints(
                                minHeight: appBarHeight,
                                maxHeight: fixedHeightAppBar ? appBarHeight : double.infinity,
                              ),
                            ),
                          ),
                  body: Builder(
                    builder: (BuildContext context) {
                      mediaQueryTop = MediaQuery.paddingOf(context).top;
                      return Container(key: bodyKey);
                    },
                  ),
                );
              },
            ),
          ),
        );
      }

      fixedHeightAppBar = false;

      // When an appbar is provided, the Scaffold's body is built within a
      // MediaQuery with padding.top = 0, and the appBar's maxHeight is
      // constrained to its preferredSize.height + the original MediaQuery
      // padding.top. When extendBodyBehindAppBar is true, an additional
      // inner MediaQuery is added around the Scaffold's body with padding.top
      // equal to the overall height of the appBar. See _BodyBuilder in
      // material/scaffold.dart.

      await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: true, hasAppBar: true));
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));
      expect(
        tester.getSize(find.byKey(appBarKey)),
        const Size(800.0, appBarHeight + windowPaddingTop),
      );
      expect(mediaQueryTop, appBarHeight + windowPaddingTop);

      await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: true, hasAppBar: false));
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));
      expect(find.byKey(appBarKey), findsNothing);
      expect(mediaQueryTop, windowPaddingTop);

      await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: false, hasAppBar: true));
      expect(
        tester.getSize(find.byKey(bodyKey)),
        const Size(800.0, 600.0 - appBarHeight - windowPaddingTop),
      );
      expect(
        tester.getSize(find.byKey(appBarKey)),
        const Size(800.0, appBarHeight + windowPaddingTop),
      );
      expect(mediaQueryTop, 0.0);

      await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: false, hasAppBar: false));
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));
      expect(find.byKey(appBarKey), findsNothing);
      expect(mediaQueryTop, windowPaddingTop);

      fixedHeightAppBar = true;

      await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: true, hasAppBar: true));
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));
      expect(tester.getSize(find.byKey(appBarKey)), const Size(800.0, appBarHeight));
      expect(mediaQueryTop, appBarHeight);

      await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: true, hasAppBar: false));
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));
      expect(find.byKey(appBarKey), findsNothing);
      expect(mediaQueryTop, windowPaddingTop);

      await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: false, hasAppBar: true));
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0 - appBarHeight));
      expect(tester.getSize(find.byKey(appBarKey)), const Size(800.0, appBarHeight));
      expect(mediaQueryTop, 0.0);

      await tester.pumpWidget(buildFrame(extendBodyBehindAppBar: false, hasAppBar: false));
      expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));
      expect(find.byKey(appBarKey), findsNothing);
      expect(mediaQueryTop, windowPaddingTop);
    });
  });

  testWidgets('Open drawer hides underlying semantics tree', (WidgetTester tester) async {
    const String bodyLabel = 'I am the body';
    const String persistentFooterButtonLabel = 'a button on the bottom';
    const String bottomNavigationBarLabel = 'a bar in an app';
    const String floatingActionButtonLabel = 'I float in space';
    const String drawerLabel = 'I am the reason for this test';

    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: Text(bodyLabel),
          persistentFooterButtons: <Widget>[Text(persistentFooterButtonLabel)],
          bottomNavigationBar: Text(bottomNavigationBarLabel),
          floatingActionButton: Text(floatingActionButtonLabel),
          drawer: Drawer(child: Text(drawerLabel)),
        ),
      ),
    );

    expect(semantics, includesNodeWith(label: bodyLabel));
    expect(semantics, includesNodeWith(label: persistentFooterButtonLabel));
    expect(semantics, includesNodeWith(label: bottomNavigationBarLabel));
    expect(semantics, includesNodeWith(label: floatingActionButtonLabel));
    expect(semantics, isNot(includesNodeWith(label: drawerLabel)));

    final ScaffoldState state = tester.firstState(find.byType(Scaffold));
    state.openDrawer();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(semantics, isNot(includesNodeWith(label: bodyLabel)));
    expect(semantics, isNot(includesNodeWith(label: persistentFooterButtonLabel)));
    expect(semantics, isNot(includesNodeWith(label: bottomNavigationBarLabel)));
    expect(semantics, isNot(includesNodeWith(label: floatingActionButtonLabel)));
    expect(semantics, includesNodeWith(label: drawerLabel));

    semantics.dispose();
  });

  testWidgets('Scaffold and extreme window padding', (WidgetTester tester) async {
    final Key appBar = UniqueKey();
    final Key body = UniqueKey();
    final Key floatingActionButton = UniqueKey();
    final Key persistentFooterButton = UniqueKey();
    final Key drawer = UniqueKey();
    final Key bottomNavigationBar = UniqueKey();
    final Key insideAppBar = UniqueKey();
    final Key insideBody = UniqueKey();
    final Key insideFloatingActionButton = UniqueKey();
    final Key insidePersistentFooterButton = UniqueKey();
    final Key insideDrawer = UniqueKey();
    final Key insideBottomNavigationBar = UniqueKey();
    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'us'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultWidgetsLocalizations.delegate,
          DefaultMaterialLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(left: 20.0, top: 30.0, right: 50.0, bottom: 60.0),
              viewInsets: EdgeInsets.only(bottom: 200.0),
            ),
            child: Scaffold(
              drawerDragStartBehavior: DragStartBehavior.down,
              appBar: PreferredSize(
                preferredSize: const Size(11.0, 13.0),
                child: Container(
                  key: appBar,
                  child: SafeArea(child: Placeholder(key: insideAppBar)),
                ),
              ),
              body: Container(key: body, child: SafeArea(child: Placeholder(key: insideBody))),
              floatingActionButton: SizedBox(
                key: floatingActionButton,
                width: 77.0,
                height: 77.0,
                child: SafeArea(child: Placeholder(key: insideFloatingActionButton)),
              ),
              persistentFooterButtons: <Widget>[
                SizedBox(
                  key: persistentFooterButton,
                  width: 100.0,
                  height: 90.0,
                  child: SafeArea(child: Placeholder(key: insidePersistentFooterButton)),
                ),
              ],
              drawer: SizedBox(
                key: drawer,
                width: 204.0,
                child: SafeArea(child: Placeholder(key: insideDrawer)),
              ),
              bottomNavigationBar: SizedBox(
                key: bottomNavigationBar,
                height: 85.0,
                child: SafeArea(child: Placeholder(key: insideBottomNavigationBar)),
              ),
            ),
          ),
        ),
      ),
    );
    // open drawer
    await tester.flingFrom(const Offset(795.0, 5.0), const Offset(-200.0, 0.0), 10.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(tester.getRect(find.byKey(appBar)), const Rect.fromLTRB(0.0, 0.0, 800.0, 43.0));
    expect(tester.getRect(find.byKey(body)), const Rect.fromLTRB(0.0, 43.0, 800.0, 400.0));
    expect(
      tester.getRect(find.byKey(floatingActionButton)),
      rectMoreOrLessEquals(const Rect.fromLTRB(36.0, 307.0, 113.0, 384.0)),
    );
    expect(
      tester.getRect(find.byKey(persistentFooterButton)),
      const Rect.fromLTRB(28.0, 417.0, 128.0, 507.0),
    ); // Includes 8px each top/bottom padding.
    expect(tester.getRect(find.byKey(drawer)), const Rect.fromLTRB(596.0, 0.0, 800.0, 600.0));
    expect(
      tester.getRect(find.byKey(bottomNavigationBar)),
      const Rect.fromLTRB(0.0, 515.0, 800.0, 600.0),
    );
    expect(tester.getRect(find.byKey(insideAppBar)), const Rect.fromLTRB(20.0, 30.0, 750.0, 43.0));
    expect(tester.getRect(find.byKey(insideBody)), const Rect.fromLTRB(20.0, 43.0, 750.0, 400.0));
    expect(
      tester.getRect(find.byKey(insideFloatingActionButton)),
      rectMoreOrLessEquals(const Rect.fromLTRB(36.0, 307.0, 113.0, 384.0)),
    );
    expect(
      tester.getRect(find.byKey(insidePersistentFooterButton)),
      const Rect.fromLTRB(28.0, 417.0, 128.0, 507.0),
    );
    expect(
      tester.getRect(find.byKey(insideDrawer)),
      const Rect.fromLTRB(596.0, 30.0, 750.0, 540.0),
    );
    expect(
      tester.getRect(find.byKey(insideBottomNavigationBar)),
      const Rect.fromLTRB(20.0, 515.0, 750.0, 540.0),
    );
  });

  testWidgets('Scaffold and extreme window padding - persistent footer buttons only', (
    WidgetTester tester,
  ) async {
    final Key appBar = UniqueKey();
    final Key body = UniqueKey();
    final Key floatingActionButton = UniqueKey();
    final Key persistentFooterButton = UniqueKey();
    final Key drawer = UniqueKey();
    final Key insideAppBar = UniqueKey();
    final Key insideBody = UniqueKey();
    final Key insideFloatingActionButton = UniqueKey();
    final Key insidePersistentFooterButton = UniqueKey();
    final Key insideDrawer = UniqueKey();
    await tester.pumpWidget(
      Localizations(
        locale: const Locale('en', 'us'),
        delegates: const <LocalizationsDelegate<dynamic>>[
          DefaultWidgetsLocalizations.delegate,
          DefaultMaterialLocalizations.delegate,
        ],
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(left: 20.0, top: 30.0, right: 50.0, bottom: 60.0),
              viewInsets: EdgeInsets.only(bottom: 200.0),
            ),
            child: Scaffold(
              appBar: PreferredSize(
                preferredSize: const Size(11.0, 13.0),
                child: Container(
                  key: appBar,
                  child: SafeArea(child: Placeholder(key: insideAppBar)),
                ),
              ),
              body: Container(key: body, child: SafeArea(child: Placeholder(key: insideBody))),
              floatingActionButton: SizedBox(
                key: floatingActionButton,
                width: 77.0,
                height: 77.0,
                child: SafeArea(child: Placeholder(key: insideFloatingActionButton)),
              ),
              persistentFooterButtons: <Widget>[
                SizedBox(
                  key: persistentFooterButton,
                  width: 100.0,
                  height: 90.0,
                  child: SafeArea(child: Placeholder(key: insidePersistentFooterButton)),
                ),
              ],
              drawer: SizedBox(
                key: drawer,
                width: 204.0,
                child: SafeArea(child: Placeholder(key: insideDrawer)),
              ),
            ),
          ),
        ),
      ),
    );
    // open drawer
    await tester.flingFrom(const Offset(795.0, 5.0), const Offset(-200.0, 0.0), 10.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(tester.getRect(find.byKey(appBar)), const Rect.fromLTRB(0.0, 0.0, 800.0, 43.0));
    expect(tester.getRect(find.byKey(body)), const Rect.fromLTRB(0.0, 43.0, 800.0, 400.0));
    expect(
      tester.getRect(find.byKey(floatingActionButton)),
      rectMoreOrLessEquals(const Rect.fromLTRB(36.0, 307.0, 113.0, 384.0)),
    );
    expect(
      tester.getRect(find.byKey(persistentFooterButton)),
      const Rect.fromLTRB(28.0, 442.0, 128.0, 532.0),
    ); // Includes 8px each top/bottom padding.
    expect(tester.getRect(find.byKey(drawer)), const Rect.fromLTRB(596.0, 0.0, 800.0, 600.0));
    expect(tester.getRect(find.byKey(insideAppBar)), const Rect.fromLTRB(20.0, 30.0, 750.0, 43.0));
    expect(tester.getRect(find.byKey(insideBody)), const Rect.fromLTRB(20.0, 43.0, 750.0, 400.0));
    expect(
      tester.getRect(find.byKey(insideFloatingActionButton)),
      rectMoreOrLessEquals(const Rect.fromLTRB(36.0, 307.0, 113.0, 384.0)),
    );
    expect(
      tester.getRect(find.byKey(insidePersistentFooterButton)),
      const Rect.fromLTRB(28.0, 442.0, 128.0, 532.0),
    );
    expect(
      tester.getRect(find.byKey(insideDrawer)),
      const Rect.fromLTRB(596.0, 30.0, 750.0, 540.0),
    );
  });

  group('ScaffoldGeometry', () {
    testWidgets('bottomNavigationBar', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Container(),
            bottomNavigationBar: ConstrainedBox(
              key: key,
              constraints: const BoxConstraints.expand(height: 80.0),
              child: const _GeometryListener(),
            ),
          ),
        ),
      );

      final RenderBox navigationBox = tester.renderObject(find.byKey(key));
      final RenderBox appBox = tester.renderObject(find.byType(MaterialApp));
      final _GeometryListenerState listenerState = tester.state(find.byType(_GeometryListener));
      final ScaffoldGeometry geometry = listenerState.cache.value;

      expect(geometry.bottomNavigationBarTop, appBox.size.height - navigationBox.size.height);
    });

    testWidgets('no bottomNavigationBar', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ConstrainedBox(
              constraints: const BoxConstraints.expand(height: 80.0),
              child: const _GeometryListener(),
            ),
          ),
        ),
      );

      final _GeometryListenerState listenerState = tester.state(find.byType(_GeometryListener));
      final ScaffoldGeometry geometry = listenerState.cache.value;

      expect(geometry.bottomNavigationBarTop, null);
    });

    testWidgets('Scaffold BottomNavigationBar bottom padding is not consumed by viewInsets.', (
      WidgetTester tester,
    ) async {
      Widget boilerplate(Widget child) {
        return Localizations(
          locale: const Locale('en', 'us'),
          delegates: const <LocalizationsDelegate<dynamic>>[
            DefaultWidgetsLocalizations.delegate,
            DefaultMaterialLocalizations.delegate,
          ],
          child: Directionality(textDirection: TextDirection.ltr, child: child),
        );
      }

      final Widget child = boilerplate(
        Scaffold(
          resizeToAvoidBottomInset: false,
          body: const Placeholder(),
          bottomNavigationBar: Navigator(
            onGenerateRoute: (RouteSettings settings) {
              return MaterialPageRoute<void>(
                builder: (BuildContext context) {
                  return BottomNavigationBar(
                    items: const <BottomNavigationBarItem>[
                      BottomNavigationBarItem(icon: Icon(Icons.add), label: 'test'),
                      BottomNavigationBarItem(icon: Icon(Icons.add), label: 'test'),
                    ],
                  );
                },
              );
            },
          ),
        ),
      );

      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.only(bottom: 20.0)),
          child: child,
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
    });

    testWidgets('floatingActionButton', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Container(),
            floatingActionButton: FloatingActionButton(
              key: key,
              child: const _GeometryListener(),
              onPressed: () {},
            ),
          ),
        ),
      );

      final RenderBox floatingActionButtonBox = tester.renderObject(find.byKey(key));
      final _GeometryListenerState listenerState = tester.state(find.byType(_GeometryListener));
      final ScaffoldGeometry geometry = listenerState.cache.value;

      final Rect fabRect =
          floatingActionButtonBox.localToGlobal(Offset.zero) & floatingActionButtonBox.size;

      expect(geometry.floatingActionButtonArea, fabRect);
    });

    testWidgets('no floatingActionButton', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ConstrainedBox(
              constraints: const BoxConstraints.expand(height: 80.0),
              child: const _GeometryListener(),
            ),
          ),
        ),
      );

      final _GeometryListenerState listenerState = tester.state(find.byType(_GeometryListener));
      final ScaffoldGeometry geometry = listenerState.cache.value;

      expect(geometry.floatingActionButtonArea, null);
    });

    testWidgets('floatingActionButton entrance/exit animation', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ConstrainedBox(
              constraints: const BoxConstraints.expand(height: 80.0),
              child: const _GeometryListener(),
            ),
          ),
        ),
      );

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Container(),
            floatingActionButton: FloatingActionButton(
              key: key,
              child: const _GeometryListener(),
              onPressed: () {},
            ),
          ),
        ),
      );

      final _GeometryListenerState listenerState = tester.state(find.byType(_GeometryListener));
      await tester.pump(const Duration(milliseconds: 50));

      ScaffoldGeometry geometry = listenerState.cache.value;
      final Rect transitioningFabRect = geometry.floatingActionButtonArea!;

      final double transitioningRotation =
          tester.widget<RotationTransition>(find.byType(RotationTransition)).turns.value;

      await tester.pump(const Duration(seconds: 3));
      geometry = listenerState.cache.value;
      final RenderBox floatingActionButtonBox = tester.renderObject(find.byKey(key));
      final Rect fabRect =
          floatingActionButtonBox.localToGlobal(Offset.zero) & floatingActionButtonBox.size;

      final double completedRotation =
          tester.widget<RotationTransition>(find.byType(RotationTransition)).turns.value;

      expect(transitioningRotation, lessThan(1.0));

      expect(completedRotation, equals(1.0));

      expect(geometry.floatingActionButtonArea, fabRect);

      expect(geometry.floatingActionButtonArea!.center, transitioningFabRect.center);

      expect(geometry.floatingActionButtonArea!.width, greaterThan(transitioningFabRect.width));

      expect(geometry.floatingActionButtonArea!.height, greaterThan(transitioningFabRect.height));
    });

    testWidgets('change notifications', (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      int numNotificationsAtLastFrame = 0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ConstrainedBox(
              constraints: const BoxConstraints.expand(height: 80.0),
              child: const _GeometryListener(),
            ),
          ),
        ),
      );

      final _GeometryListenerState listenerState = tester.state(find.byType(_GeometryListener));

      expect(listenerState.numNotifications, greaterThan(numNotificationsAtLastFrame));
      numNotificationsAtLastFrame = listenerState.numNotifications;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Container(),
            floatingActionButton: FloatingActionButton(
              key: key,
              child: const _GeometryListener(),
              onPressed: () {},
            ),
          ),
        ),
      );

      expect(listenerState.numNotifications, greaterThan(numNotificationsAtLastFrame));
      numNotificationsAtLastFrame = listenerState.numNotifications;

      await tester.pump(const Duration(milliseconds: 50));

      expect(listenerState.numNotifications, greaterThan(numNotificationsAtLastFrame));
      numNotificationsAtLastFrame = listenerState.numNotifications;

      await tester.pump(const Duration(seconds: 3));

      expect(listenerState.numNotifications, greaterThan(numNotificationsAtLastFrame));
      numNotificationsAtLastFrame = listenerState.numNotifications;
    });

    testWidgets('Simultaneous drawers on either side', (WidgetTester tester) async {
      const String bodyLabel = 'I am the body';
      const String drawerLabel = 'I am the label on start side';
      const String endDrawerLabel = 'I am the label on end side';

      final SemanticsTester semantics = SemanticsTester(tester);
      await tester.pumpWidget(
        const MaterialApp(
          home: Scaffold(
            body: Text(bodyLabel),
            drawer: Drawer(child: Text(drawerLabel)),
            endDrawer: Drawer(child: Text(endDrawerLabel)),
          ),
        ),
      );

      expect(semantics, includesNodeWith(label: bodyLabel));
      expect(semantics, isNot(includesNodeWith(label: drawerLabel)));
      expect(semantics, isNot(includesNodeWith(label: endDrawerLabel)));

      final ScaffoldState state = tester.firstState(find.byType(Scaffold));
      state.openDrawer();
      await tester.pump();
      await tester.pump(const Duration(seconds: 1));

      expect(semantics, isNot(includesNodeWith(label: bodyLabel)));
      expect(semantics, includesNodeWith(label: drawerLabel));

      state.openEndDrawer();
      await tester.pump();
      await tester.pump(const Duration(seconds: 1));

      expect(semantics, isNot(includesNodeWith(label: bodyLabel)));
      expect(semantics, includesNodeWith(label: endDrawerLabel));

      semantics.dispose();
    });

    testWidgets('Drawer state query correctly', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SafeArea(
            left: false,
            right: false,
            bottom: false,
            child: Scaffold(
              endDrawer: const Drawer(child: Text('endDrawer')),
              drawer: const Drawer(child: Text('drawer')),
              body: const Text('scaffold body'),
              appBar: AppBar(centerTitle: true, title: const Text('Title')),
            ),
          ),
        ),
      );

      final ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));

      final Finder drawerOpenButton = find.byType(IconButton).first;
      final Finder endDrawerOpenButton = find.byType(IconButton).last;

      await tester.tap(drawerOpenButton);
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, true);
      await tester.tap(endDrawerOpenButton, warnIfMissed: false); // hits the modal barrier
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, false);

      await tester.tap(endDrawerOpenButton);
      await tester.pumpAndSettle();
      expect(scaffoldState.isEndDrawerOpen, true);
      await tester.tap(drawerOpenButton, warnIfMissed: false); // hits the modal barrier
      await tester.pumpAndSettle();
      expect(scaffoldState.isEndDrawerOpen, false);

      scaffoldState.openDrawer();
      expect(scaffoldState.isDrawerOpen, true);
      await tester.tap(endDrawerOpenButton, warnIfMissed: false); // hits the modal barrier
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, false);

      scaffoldState.openEndDrawer();
      expect(scaffoldState.isEndDrawerOpen, true);

      scaffoldState.openDrawer();
      expect(scaffoldState.isDrawerOpen, true);
    });

    testWidgets('Dual Drawer Opening', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SafeArea(
            left: false,
            right: false,
            bottom: false,
            child: Scaffold(
              endDrawer: const Drawer(child: Text('endDrawer')),
              drawer: const Drawer(child: Text('drawer')),
              body: const Text('scaffold body'),
              appBar: AppBar(centerTitle: true, title: const Text('Title')),
            ),
          ),
        ),
      );

      // Open Drawer, tap on end drawer, which closes the drawer, but does
      // not open the drawer.
      await tester.tap(find.byType(IconButton).first);
      await tester.pumpAndSettle();
      await tester.tap(find.byType(IconButton).last, warnIfMissed: false); // hits the modal barrier
      await tester.pumpAndSettle();

      expect(find.text('endDrawer'), findsNothing);
      expect(find.text('drawer'), findsNothing);

      // Tapping the first opens the first drawer
      await tester.tap(find.byType(IconButton).first);
      await tester.pumpAndSettle();

      expect(find.text('endDrawer'), findsNothing);
      expect(find.text('drawer'), findsOneWidget);

      // Tapping on the end drawer and then on the drawer should close the
      // drawer and then reopen it.
      await tester.tap(find.byType(IconButton).last, warnIfMissed: false); // hits the modal barrier
      await tester.pumpAndSettle();
      await tester.tap(find.byType(IconButton).first);
      await tester.pumpAndSettle();

      expect(find.text('endDrawer'), findsNothing);
      expect(find.text('drawer'), findsOneWidget);
    });

    testWidgets('Drawer opens correctly with padding from MediaQuery (LTR)', (
      WidgetTester tester,
    ) async {
      const double simulatedNotchSize = 40.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            drawer: const Drawer(child: Text('Drawer')),
            body: const Text('Scaffold Body'),
            appBar: AppBar(centerTitle: true, title: const Text('Title')),
          ),
        ),
      );

      ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));
      expect(scaffoldState.isDrawerOpen, false);

      await tester.dragFrom(const Offset(simulatedNotchSize + 15.0, 100), const Offset(300, 0));
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, false);

      await tester.pumpWidget(
        MaterialApp(
          home: MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.fromLTRB(simulatedNotchSize, 0, 0, 0)),
            child: Scaffold(
              drawer: const Drawer(child: Text('Drawer')),
              body: const Text('Scaffold Body'),
              appBar: AppBar(centerTitle: true, title: const Text('Title')),
            ),
          ),
        ),
      );
      scaffoldState = tester.state(find.byType(Scaffold));
      expect(scaffoldState.isDrawerOpen, false);

      await tester.dragFrom(const Offset(simulatedNotchSize + 15.0, 100), const Offset(300, 0));
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, true);
    });

    testWidgets('Drawer opens correctly with padding from MediaQuery (RTL)', (
      WidgetTester tester,
    ) async {
      const double simulatedNotchSize = 40.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            drawer: const Drawer(child: Text('Drawer')),
            body: const Text('Scaffold Body'),
            appBar: AppBar(centerTitle: true, title: const Text('Title')),
          ),
        ),
      );

      final double scaffoldWidth = tester.renderObject<RenderBox>(find.byType(Scaffold)).size.width;
      ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));
      expect(scaffoldState.isDrawerOpen, false);

      await tester.dragFrom(
        Offset(scaffoldWidth - simulatedNotchSize - 15.0, 100),
        const Offset(-300, 0),
      );
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, false);

      await tester.pumpWidget(
        MaterialApp(
          home: MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.fromLTRB(0, 0, simulatedNotchSize, 0)),
            child: Directionality(
              textDirection: TextDirection.rtl,
              child: Scaffold(
                drawer: const Drawer(child: Text('Drawer')),
                body: const Text('Scaffold body'),
                appBar: AppBar(centerTitle: true, title: const Text('Title')),
              ),
            ),
          ),
        ),
      );
      scaffoldState = tester.state(find.byType(Scaffold));
      expect(scaffoldState.isDrawerOpen, false);

      await tester.dragFrom(
        Offset(scaffoldWidth - simulatedNotchSize - 15.0, 100),
        const Offset(-300, 0),
      );
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, true);
    });
  });

  testWidgets('Drawer opens correctly with custom edgeDragWidth', (WidgetTester tester) async {
    // The default edge drag width is 20.0.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          drawer: const Drawer(child: Text('Drawer')),
          body: const Text('Scaffold body'),
          appBar: AppBar(centerTitle: true, title: const Text('Title')),
        ),
      ),
    );
    ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));
    expect(scaffoldState.isDrawerOpen, false);

    await tester.dragFrom(const Offset(35, 100), const Offset(300, 0));
    await tester.pumpAndSettle();
    expect(scaffoldState.isDrawerOpen, false);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          drawer: const Drawer(child: Text('Drawer')),
          drawerEdgeDragWidth: 40.0,
          body: const Text('Scaffold Body'),
          appBar: AppBar(centerTitle: true, title: const Text('Title')),
        ),
      ),
    );
    scaffoldState = tester.state(find.byType(Scaffold));
    expect(scaffoldState.isDrawerOpen, false);

    await tester.dragFrom(const Offset(35, 100), const Offset(300, 0));
    await tester.pumpAndSettle();
    expect(scaffoldState.isDrawerOpen, true);
  });

  testWidgets(
    'Drawer does not open with a drag gesture when it is disabled on mobile',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            drawer: const Drawer(child: Text('Drawer')),
            body: const Text('Scaffold Body'),
            appBar: AppBar(centerTitle: true, title: const Text('Title')),
          ),
        ),
      );
      ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));
      expect(scaffoldState.isDrawerOpen, false);

      // Test that we can open the drawer with a drag gesture when
      // `Scaffold.drawerEnableDragGesture` is true.
      await tester.dragFrom(const Offset(0, 100), const Offset(300, 0));
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, true);

      await tester.dragFrom(const Offset(300, 100), const Offset(-300, 0));
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, false);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            drawer: const Drawer(child: Text('Drawer')),
            drawerEnableOpenDragGesture: false,
            body: const Text('Scaffold body'),
            appBar: AppBar(centerTitle: true, title: const Text('Title')),
          ),
        ),
      );
      scaffoldState = tester.state(find.byType(Scaffold));
      expect(scaffoldState.isDrawerOpen, false);

      // Test that we cannot open the drawer with a drag gesture when
      // `Scaffold.drawerEnableDragGesture` is false.
      await tester.dragFrom(const Offset(0, 100), const Offset(300, 0));
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, false);

      // Test that we can close drawer with a drag gesture when
      // `Scaffold.drawerEnableDragGesture` is false.
      final Finder drawerOpenButton = find.byType(IconButton).first;
      await tester.tap(drawerOpenButton);
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, true);

      await tester.dragFrom(const Offset(300, 100), const Offset(-300, 0));
      await tester.pumpAndSettle();
      expect(scaffoldState.isDrawerOpen, false);
    },
    variant: TargetPlatformVariant.mobile(),
  );

  testWidgets('Drawer does not open with a drag gesture on desktop', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          drawer: const Drawer(child: Text('Drawer')),
          body: const Text('Scaffold Body'),
          appBar: AppBar(centerTitle: true, title: const Text('Title')),
        ),
      ),
    );
    final ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));
    expect(scaffoldState.isDrawerOpen, false);

    // Test that we cannot open the drawer with a drag gesture.
    await tester.dragFrom(const Offset(0, 100), const Offset(300, 0));
    await tester.pumpAndSettle();
    expect(scaffoldState.isDrawerOpen, false);

    // Test that we can open the drawer with a tap gesture on drawer icon button.
    final Finder drawerOpenButton = find.byType(IconButton).first;
    await tester.tap(drawerOpenButton);
    await tester.pumpAndSettle();
    expect(scaffoldState.isDrawerOpen, true);

    // Test that we cannot close the drawer with a drag gesture.
    await tester.dragFrom(const Offset(300, 100), const Offset(-300, 0));
    await tester.pumpAndSettle();
    expect(scaffoldState.isDrawerOpen, true);

    // Test that we can close the drawer with a tap gesture in the body.
    await tester.tapAt(const Offset(500, 300));
    await tester.pumpAndSettle();
    expect(scaffoldState.isDrawerOpen, false);
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('End drawer does not open with a drag gesture when it is disabled', (
    WidgetTester tester,
  ) async {
    late double screenWidth;
    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            screenWidth = MediaQuery.sizeOf(context).width;
            return Scaffold(
              endDrawer: const Drawer(child: Text('Drawer')),
              body: const Text('Scaffold Body'),
              appBar: AppBar(centerTitle: true, title: const Text('Title')),
            );
          },
        ),
      ),
    );
    ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));
    expect(scaffoldState.isEndDrawerOpen, false);

    // Test that we can open the end drawer with a drag gesture when
    // `Scaffold.endDrawerEnableDragGesture` is true.
    await tester.dragFrom(Offset(screenWidth - 1, 100), const Offset(-300, 0));
    await tester.pumpAndSettle();
    expect(scaffoldState.isEndDrawerOpen, true);

    await tester.dragFrom(Offset(screenWidth - 300, 100), const Offset(300, 0));
    await tester.pumpAndSettle();
    expect(scaffoldState.isEndDrawerOpen, false);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          endDrawer: const Drawer(child: Text('Drawer')),
          endDrawerEnableOpenDragGesture: false,
          body: const Text('Scaffold body'),
          appBar: AppBar(centerTitle: true, title: const Text('Title')),
        ),
      ),
    );
    scaffoldState = tester.state(find.byType(Scaffold));
    expect(scaffoldState.isEndDrawerOpen, false);

    // Test that we cannot open the end drawer with a drag gesture when
    // `Scaffold.endDrawerEnableDragGesture` is false.
    await tester.dragFrom(Offset(screenWidth - 1, 100), const Offset(-300, 0));
    await tester.pumpAndSettle();
    expect(scaffoldState.isEndDrawerOpen, false);

    // Test that we can close the end drawer a with drag gesture when
    // `Scaffold.endDrawerEnableDragGesture` is false.
    final Finder endDrawerOpenButton = find.byType(IconButton).first;
    await tester.tap(endDrawerOpenButton);
    await tester.pumpAndSettle();
    expect(scaffoldState.isEndDrawerOpen, true);

    await tester.dragFrom(Offset(screenWidth - 300, 100), const Offset(300, 0));
    await tester.pumpAndSettle();
    expect(scaffoldState.isEndDrawerOpen, false);
  });

  testWidgets('Nested scaffold body insets', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/20295
    final Key bodyKey = UniqueKey();

    Widget buildFrame(bool? innerResizeToAvoidBottomInset, bool? outerResizeToAvoidBottomInset) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(viewInsets: EdgeInsets.only(bottom: 100.0)),
          child: Builder(
            builder: (BuildContext context) {
              return Scaffold(
                resizeToAvoidBottomInset: outerResizeToAvoidBottomInset,
                body: Builder(
                  builder: (BuildContext context) {
                    return Scaffold(
                      resizeToAvoidBottomInset: innerResizeToAvoidBottomInset,
                      body: Container(key: bodyKey),
                    );
                  },
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(true, true));
    expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 500.0));

    await tester.pumpWidget(buildFrame(false, true));
    expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 500.0));

    await tester.pumpWidget(buildFrame(true, false));
    expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 500.0));

    // This is the only case where the body is not bottom inset.
    await tester.pumpWidget(buildFrame(false, false));
    expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 600.0));

    await tester.pumpWidget(buildFrame(null, null)); // resizeToAvoidBottomInset default is true
    expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 500.0));

    await tester.pumpWidget(buildFrame(null, false));
    expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 500.0));

    await tester.pumpWidget(buildFrame(false, null));
    expect(tester.getSize(find.byKey(bodyKey)), const Size(800.0, 500.0));
  });

  group('FlutterError control test', () {
    testWidgets('showBottomSheet() while Scaffold has bottom sheet', (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> key = GlobalKey<ScaffoldState>();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            key: key,
            body: Center(child: Container()),
            bottomSheet: const Text('Bottom sheet'),
          ),
        ),
      );
      late FlutterError error;
      try {
        key.currentState!.showBottomSheet((BuildContext context) {
          final ThemeData themeData = Theme.of(context);
          return Container(
            decoration: BoxDecoration(
              border: Border(top: BorderSide(color: themeData.disabledColor)),
            ),
            child: Padding(
              padding: const EdgeInsets.all(32.0),
              child: Text(
                'This is a Material persistent bottom sheet. Drag downwards to dismiss it.',
                textAlign: TextAlign.center,
                style: TextStyle(color: themeData.colorScheme.secondary, fontSize: 24.0),
              ),
            ),
          );
        });
      } on FlutterError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(
          error.toStringDeep(),
          equalsIgnoringHashCodes(
            'FlutterError\n'
            '   Scaffold.bottomSheet cannot be specified while a bottom sheet\n'
            '   displayed with showBottomSheet() is still visible.\n'
            '   Rebuild the Scaffold with a null bottomSheet before calling\n'
            '   showBottomSheet().\n',
          ),
        );
      }
    });

    testWidgets(
      'didUpdate bottomSheet while a previous bottom sheet is still displayed',
      experimentalLeakTesting:
          LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
      (WidgetTester tester) async {
        final GlobalKey<ScaffoldState> key = GlobalKey<ScaffoldState>();
        const Key buttonKey = Key('button');
        final List<FlutterErrorDetails> errors = <FlutterErrorDetails>[];
        FlutterError.onError = (FlutterErrorDetails error) => errors.add(error);
        int state = 0;
        await tester.pumpWidget(
          MaterialApp(
            home: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Scaffold(
                  key: key,
                  body: Container(),
                  floatingActionButton: FloatingActionButton(
                    key: buttonKey,
                    onPressed: () {
                      state += 1;
                      setState(() {});
                    },
                  ),
                  bottomSheet: state == 0 ? null : const SizedBox(),
                );
              },
            ),
          ),
        );
        key.currentState!.showBottomSheet((_) => Container());
        await tester.tap(find.byKey(buttonKey));
        await tester.pump();
        expect(errors, isNotEmpty);
        expect(errors.first.exception, isFlutterError);
        final FlutterError error = errors.first.exception as FlutterError;
        expect(error.diagnostics.length, 2);
        expect(error.diagnostics.last.level, DiagnosticLevel.hint);
        expect(
          error.diagnostics.last.toStringDeep(),
          'Use the PersistentBottomSheetController returned by\n'
          'showBottomSheet() to close the old bottom sheet before creating a\n'
          'Scaffold with a (non null) bottomSheet.\n',
        );
        expect(
          error.toStringDeep(),
          'FlutterError\n'
          '   Scaffold.bottomSheet cannot be specified while a bottom sheet\n'
          '   displayed with showBottomSheet() is still visible.\n'
          '   Use the PersistentBottomSheetController returned by\n'
          '   showBottomSheet() to close the old bottom sheet before creating a\n'
          '   Scaffold with a (non null) bottomSheet.\n',
        );
        await tester.pumpAndSettle();
      },
    );

    testWidgets('Call to Scaffold.of() without context', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Builder(
            builder: (BuildContext context) {
              Scaffold.of(context).showBottomSheet((BuildContext context) {
                return Container();
              });
              return Container();
            },
          ),
        ),
      );
      final dynamic exception = tester.takeException();
      expect(exception, isFlutterError);
      final FlutterError error = exception as FlutterError;
      expect(error.diagnostics.length, 5);
      expect(error.diagnostics[2].level, DiagnosticLevel.hint);
      expect(
        error.diagnostics[2].toStringDeep(),
        equalsIgnoringHashCodes(
          'There are several ways to avoid this problem. The simplest is to\n'
          'use a Builder to get a context that is "under" the Scaffold. For\n'
          'an example of this, please see the documentation for\n'
          'Scaffold.of():\n'
          '  https://api.flutter.dev/flutter/material/Scaffold/of.html\n',
        ),
      );
      expect(error.diagnostics[3].level, DiagnosticLevel.hint);
      expect(
        error.diagnostics[3].toStringDeep(),
        equalsIgnoringHashCodes(
          'A more efficient solution is to split your build function into\n'
          'several widgets. This introduces a new context from which you can\n'
          'obtain the Scaffold. In this solution, you would have an outer\n'
          'widget that creates the Scaffold populated by instances of your\n'
          'new inner widgets, and then in these inner widgets you would use\n'
          'Scaffold.of().\n'
          'A less elegant but more expedient solution is assign a GlobalKey\n'
          'to the Scaffold, then use the key.currentState property to obtain\n'
          'the ScaffoldState rather than using the Scaffold.of() function.\n',
        ),
      );
      expect(error.diagnostics[4], isA<DiagnosticsProperty<Element>>());
      expect(
        error.toStringDeep(),
        'FlutterError\n'
        '   Scaffold.of() called with a context that does not contain a\n'
        '   Scaffold.\n'
        '   No Scaffold ancestor could be found starting from the context\n'
        '   that was passed to Scaffold.of(). This usually happens when the\n'
        '   context provided is from the same StatefulWidget as that whose\n'
        '   build function actually creates the Scaffold widget being sought.\n'
        '   There are several ways to avoid this problem. The simplest is to\n'
        '   use a Builder to get a context that is "under" the Scaffold. For\n'
        '   an example of this, please see the documentation for\n'
        '   Scaffold.of():\n'
        '     https://api.flutter.dev/flutter/material/Scaffold/of.html\n'
        '   A more efficient solution is to split your build function into\n'
        '   several widgets. This introduces a new context from which you can\n'
        '   obtain the Scaffold. In this solution, you would have an outer\n'
        '   widget that creates the Scaffold populated by instances of your\n'
        '   new inner widgets, and then in these inner widgets you would use\n'
        '   Scaffold.of().\n'
        '   A less elegant but more expedient solution is assign a GlobalKey\n'
        '   to the Scaffold, then use the key.currentState property to obtain\n'
        '   the ScaffoldState rather than using the Scaffold.of() function.\n'
        '   The context used was:\n'
        '     Builder\n',
      );
      await tester.pumpAndSettle();
    });

    testWidgets('Call to Scaffold.geometryOf() without context', (WidgetTester tester) async {
      ValueListenable<ScaffoldGeometry>? geometry;
      await tester.pumpWidget(
        MaterialApp(
          home: Builder(
            builder: (BuildContext context) {
              geometry = Scaffold.geometryOf(context);
              return Container();
            },
          ),
        ),
      );
      final dynamic exception = tester.takeException();
      expect(exception, isFlutterError);
      expect(geometry, isNull);
      final FlutterError error = exception as FlutterError;
      expect(error.diagnostics.length, 5);
      expect(error.diagnostics[2].level, DiagnosticLevel.hint);
      expect(
        error.diagnostics[2].toStringDeep(),
        equalsIgnoringHashCodes(
          'There are several ways to avoid this problem. The simplest is to\n'
          'use a Builder to get a context that is "under" the Scaffold. For\n'
          'an example of this, please see the documentation for\n'
          'Scaffold.of():\n'
          '  https://api.flutter.dev/flutter/material/Scaffold/of.html\n',
        ),
      );
      expect(error.diagnostics[3].level, DiagnosticLevel.hint);
      expect(
        error.diagnostics[3].toStringDeep(),
        equalsIgnoringHashCodes(
          'A more efficient solution is to split your build function into\n'
          'several widgets. This introduces a new context from which you can\n'
          'obtain the Scaffold. In this solution, you would have an outer\n'
          'widget that creates the Scaffold populated by instances of your\n'
          'new inner widgets, and then in these inner widgets you would use\n'
          'Scaffold.geometryOf().\n',
        ),
      );
      expect(error.diagnostics[4], isA<DiagnosticsProperty<Element>>());
      expect(
        error.toStringDeep(),
        'FlutterError\n'
        '   Scaffold.geometryOf() called with a context that does not contain\n'
        '   a Scaffold.\n'
        '   This usually happens when the context provided is from the same\n'
        '   StatefulWidget as that whose build function actually creates the\n'
        '   Scaffold widget being sought.\n'
        '   There are several ways to avoid this problem. The simplest is to\n'
        '   use a Builder to get a context that is "under" the Scaffold. For\n'
        '   an example of this, please see the documentation for\n'
        '   Scaffold.of():\n'
        '     https://api.flutter.dev/flutter/material/Scaffold/of.html\n'
        '   A more efficient solution is to split your build function into\n'
        '   several widgets. This introduces a new context from which you can\n'
        '   obtain the Scaffold. In this solution, you would have an outer\n'
        '   widget that creates the Scaffold populated by instances of your\n'
        '   new inner widgets, and then in these inner widgets you would use\n'
        '   Scaffold.geometryOf().\n'
        '   The context used was:\n'
        '     Builder\n',
      );
      await tester.pumpAndSettle();
    });

    testWidgets(
      'FloatingActionButton always keeps the same position regardless of extendBodyBehindAppBar',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              appBar: AppBar(),
              floatingActionButton: FloatingActionButton(
                onPressed: () {},
                child: const Icon(Icons.add),
              ),
              floatingActionButtonLocation: FloatingActionButtonLocation.endTop,
            ),
          ),
        );
        final Offset defaultOffset = tester.getCenter(find.byType(FloatingActionButton));

        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              appBar: AppBar(),
              floatingActionButton: FloatingActionButton(
                onPressed: () {},
                child: const Icon(Icons.add),
              ),
              floatingActionButtonLocation: FloatingActionButtonLocation.endTop,
              extendBodyBehindAppBar: true,
            ),
          ),
        );
        final Offset extendedBodyOffset = tester.getCenter(find.byType(FloatingActionButton));

        expect(defaultOffset.dy, extendedBodyOffset.dy);
      },
    );
  });

  testWidgets('ScaffoldMessenger.maybeOf can return null if not found', (
    WidgetTester tester,
  ) async {
    ScaffoldMessengerState? scaffoldMessenger;
    const Key tapTarget = Key('tap-target');
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(),
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    scaffoldMessenger = ScaffoldMessenger.maybeOf(context);
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.byKey(tapTarget));
    await tester.pump();
    expect(scaffoldMessenger, isNull);
  });

  testWidgets('ScaffoldMessenger.of will assert if not found', (WidgetTester tester) async {
    const Key tapTarget = Key('tap-target');

    final List<dynamic> exceptions = <dynamic>[];
    final FlutterExceptionHandler? oldHandler = FlutterError.onError;
    FlutterError.onError = (FlutterErrorDetails details) {
      exceptions.add(details.exception);
    };

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context);
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.byKey(tapTarget));
    FlutterError.onError = oldHandler;

    expect(exceptions.length, 1);
    // ignore: avoid_dynamic_calls
    expect(exceptions.single.runtimeType, FlutterError);
    final FlutterError error = exceptions.first as FlutterError;
    expect(error.diagnostics.length, 5);
    expect(error.diagnostics[2], isA<DiagnosticsProperty<Element>>());
    expect(error.diagnostics[3], isA<DiagnosticsBlock>());
    expect(error.diagnostics[4].level, DiagnosticLevel.hint);
    expect(
      error.diagnostics[4].toStringDeep(),
      equalsIgnoringHashCodes(
        'Typically, the ScaffoldMessenger widget is introduced by the\n'
        'MaterialApp at the top of your application widget tree.\n',
      ),
    );
    expect(
      error.toStringDeep(),
      startsWith(
        'FlutterError\n'
        '   No ScaffoldMessenger widget found.\n'
        '   Builder widgets require a ScaffoldMessenger widget ancestor.\n'
        '   The specific widget that could not find a ScaffoldMessenger\n'
        '   ancestor was:\n'
        '     Builder\n'
        '   The ancestors of this widget were:\n',
      ),
    );
    expect(
      error.toStringDeep(),
      endsWith(
        '     [root]\n'
        '   Typically, the ScaffoldMessenger widget is introduced by the\n'
        '   MaterialApp at the top of your application widget tree.\n',
      ),
    );
  });

  testWidgets('ScaffoldMessenger checks for nesting when a new Scaffold is registered', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/77251
    const String snackBarContent = 'SnackBar Content';
    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder:
              (BuildContext context) => Scaffold(
                body: Scaffold(
                  body: TextButton(
                    onPressed: () {
                      Navigator.push(
                        context,
                        MaterialPageRoute<void>(
                          builder: (BuildContext context) {
                            return Scaffold(
                              body: Column(
                                children: <Widget>[
                                  TextButton(
                                    onPressed: () {
                                      const SnackBar snackBar = SnackBar(
                                        content: Text(snackBarContent),
                                        behavior: SnackBarBehavior.floating,
                                      );
                                      ScaffoldMessenger.of(context).showSnackBar(snackBar);
                                    },
                                    child: const Text('Show SnackBar'),
                                  ),
                                  TextButton(
                                    onPressed: () {
                                      Navigator.pop(context);
                                    },
                                    child: const Text('Pop route'),
                                  ),
                                ],
                              ),
                            );
                          },
                        ),
                      );
                    },
                    child: const Text('Push route'),
                  ),
                ),
              ),
        ),
      ),
    );

    expect(find.text(snackBarContent), findsNothing);
    await tester.tap(find.text('Push route'));
    await tester.pumpAndSettle();
    expect(find.text(snackBarContent), findsNothing);
    expect(find.text('Pop route'), findsOneWidget);

    // Show SnackBar on second page
    await tester.tap(find.text('Show SnackBar'));
    await tester.pump();
    expect(find.text(snackBarContent), findsOneWidget);
    // Pop the second page, the SnackBar completes a hero animation to the next route.
    // If we have not handled the nested Scaffolds properly, this will throw an
    // exception as duplicate SnackBars on the first route would have a common hero tag.
    await tester.tap(find.text('Pop route'));
    await tester.pump();
    // There are SnackBars two during the execution of the hero animation.
    expect(find.text(snackBarContent), findsNWidgets(2));
    await tester.pumpAndSettle();
    expect(find.text(snackBarContent), findsOneWidget);
    // Allow the SnackBar to animate out
    await tester.pump(const Duration(seconds: 4));
    await tester.pumpAndSettle();
    expect(find.text(snackBarContent), findsNothing);
  });

  testWidgets('Drawer can be dismissed with escape keyboard shortcut', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/106131
    bool isDrawerOpen = false;
    bool isEndDrawerOpen = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          drawer: Container(color: Colors.blue),
          onDrawerChanged: (bool isOpen) {
            isDrawerOpen = isOpen;
          },
          endDrawer: Container(color: Colors.green),
          onEndDrawerChanged: (bool isOpen) {
            isEndDrawerOpen = isOpen;
          },
          body: Container(),
        ),
      ),
    );

    final ScaffoldState scaffoldState = tester.state(find.byType(Scaffold));

    scaffoldState.openDrawer();
    await tester.pumpAndSettle();
    expect(isDrawerOpen, true);
    expect(isEndDrawerOpen, false);

    // Try to dismiss the drawer with the shortcut key
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pumpAndSettle();
    expect(isDrawerOpen, false);
    expect(isEndDrawerOpen, false);

    scaffoldState.openEndDrawer();
    await tester.pumpAndSettle();
    expect(isDrawerOpen, false);
    expect(isEndDrawerOpen, true);

    // Try to dismiss the drawer with the shortcut key
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pumpAndSettle();
    expect(isDrawerOpen, false);
    expect(isEndDrawerOpen, false);
  });

  testWidgets(
    'ScaffoldMessenger showSnackBar throws an intuitive error message if called during build',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                ScaffoldMessenger.of(
                  context,
                ).showSnackBar(const SnackBar(content: Text('SnackBar')));
                return const SizedBox.shrink();
              },
            ),
          ),
        ),
      );

      final FlutterError error = tester.takeException() as FlutterError;
      final ErrorSummary summary = error.diagnostics.first as ErrorSummary;
      expect(summary.toString(), 'The showSnackBar() method cannot be called during build.');
    },
  );

  testWidgets('Persistent BottomSheet is not dismissible via a11y means', (
    WidgetTester tester,
  ) async {
    final Key bottomSheetKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomSheet: Container(
            key: bottomSheetKey,
            height: 44,
            color: Colors.blue,
            child: const Text('BottomSheet'),
          ),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byKey(bottomSheetKey)),
      // Having the redundant argument value makes the intent of the test clear.
      // ignore: avoid_redundant_argument_values
      matchesSemantics(label: 'BottomSheet', hasDismissAction: false),
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/117004
  testWidgets('can rebuild and remove bottomSheet at the same time', (WidgetTester tester) async {
    bool themeIsLight = true;
    bool? defaultBottomSheet = true;
    final GlobalKey bottomSheetKey1 = GlobalKey();
    final GlobalKey bottomSheetKey2 = GlobalKey();
    late StateSetter setState;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter stateSetter) {
          setState = stateSetter;
          return MaterialApp(
            theme: themeIsLight ? ThemeData.light() : ThemeData.dark(),
            home: Scaffold(
              bottomSheet:
                  defaultBottomSheet == null
                      ? null
                      : defaultBottomSheet!
                      ? Container(
                        key: bottomSheetKey1,
                        width: double.infinity,
                        height: 100,
                        color: Colors.blue,
                        child: const Text('BottomSheet'),
                      )
                      : Container(
                        key: bottomSheetKey2,
                        width: double.infinity,
                        height: 100,
                        color: Colors.red,
                        child: const Text('BottomSheet'),
                      ),
              body: const Placeholder(),
            ),
          );
        },
      ),
    );

    expect(find.byKey(bottomSheetKey1), findsOneWidget);
    expect(find.byKey(bottomSheetKey2), findsNothing);

    // Change to the other bottomSheet.
    setState(() {
      defaultBottomSheet = false;
    });
    expect(find.byKey(bottomSheetKey1), findsOneWidget);
    expect(find.byKey(bottomSheetKey2), findsNothing);
    await tester.pumpAndSettle();
    expect(find.byKey(bottomSheetKey1), findsNothing);
    expect(find.byKey(bottomSheetKey2), findsOneWidget);

    // Set bottomSheet to null, which starts its exit animation.
    setState(() {
      defaultBottomSheet = null;
    });
    expect(find.byKey(bottomSheetKey1), findsNothing);
    expect(find.byKey(bottomSheetKey2), findsOneWidget);

    // While the bottomSheet is on the way out, change the theme to cause it to
    // rebuild.
    setState(() {
      themeIsLight = false;
    });
    expect(find.byKey(bottomSheetKey1), findsNothing);
    expect(find.byKey(bottomSheetKey2), findsOneWidget);

    // The most recent bottomSheet remains on screen during the exit animation.
    await tester.pump(_bottomSheetExitDuration);
    expect(find.byKey(bottomSheetKey1), findsNothing);
    expect(find.byKey(bottomSheetKey2), findsOneWidget);

    // After animating out, the bottomSheet is gone.
    await tester.pumpAndSettle();
    expect(find.byKey(bottomSheetKey1), findsNothing);
    expect(find.byKey(bottomSheetKey2), findsNothing);

    expect(tester.takeException(), isNull);
  });

  testWidgets('showBottomSheet removes scrim when draggable sheet is dismissed', (
    WidgetTester tester,
  ) async {
    final DraggableScrollableController draggableController = DraggableScrollableController();
    addTearDown(draggableController.dispose);
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey();
    PersistentBottomSheetController? sheetController;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    sheetController = scaffoldKey.currentState!.showBottomSheet((_) {
      return DraggableScrollableSheet(
        expand: false,
        controller: draggableController,
        builder: (BuildContext context, ScrollController scrollController) {
          return SingleChildScrollView(controller: scrollController, child: const Placeholder());
        },
      );
    });

    Finder findModalBarrier() =>
        find.descendant(of: find.byType(Scaffold), matching: find.byType(ModalBarrier));

    await tester.pump();
    expect(find.byType(BottomSheet), findsOneWidget);

    // The scrim is not present yet.
    expect(findModalBarrier(), findsNothing);

    // Expand the sheet to 80% of parent height to show the scrim.
    draggableController.jumpTo(0.8);
    await tester.pump();
    expect(findModalBarrier(), findsOneWidget);

    // Dismiss the sheet.
    sheetController.close();
    await tester.pumpAndSettle();

    // The scrim should be gone.
    expect(findModalBarrier(), findsNothing);
  });

  testWidgets("Closing bottom sheet & removing FAB at the same time doesn't throw assertion", (
    WidgetTester tester,
  ) async {
    final Key bottomSheetKey = UniqueKey();
    PersistentBottomSheetController? controller;
    bool show = true;

    await tester.pumpWidget(
      StatefulBuilder(
        builder:
            (_, StateSetter setState) => MaterialApp(
              home: Scaffold(
                body: Center(
                  child: Builder(
                    builder:
                        (BuildContext context) => ElevatedButton(
                          onPressed: () {
                            if (controller == null) {
                              controller = showBottomSheet(
                                context: context,
                                builder: (_) => Container(key: bottomSheetKey, height: 200),
                              );
                            } else {
                              controller!.close();
                              controller = null;
                            }
                          },
                          child: const Text('BottomSheet'),
                        ),
                  ),
                ),
                floatingActionButton:
                    show
                        ? FloatingActionButton(onPressed: () => setState(() => show = false))
                        : null,
              ),
            ),
      ),
    );

    // Show bottom sheet.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    // Bottom sheet and FAB are visible.
    expect(find.byType(FloatingActionButton), findsOneWidget);
    expect(find.byKey(bottomSheetKey), findsOneWidget);

    // Close bottom sheet while removing FAB.
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump(); // start animation
    await tester.tap(find.byType(ElevatedButton));
    // Let the animation finish.
    await tester.pumpAndSettle(const Duration(seconds: 1));

    // Bottom sheet and FAB are gone.
    expect(find.byType(FloatingActionButton), findsNothing);
    expect(find.byKey(bottomSheetKey), findsNothing);

    // No exception is thrown.
    expect(tester.takeException(), isNull);
  });

  // Regression test for https://github.com/flutter/flutter/issues/115924.
  testWidgets('Default ScaffoldMessenger can access ambient theme', (WidgetTester tester) async {
    final GlobalKey<ScaffoldMessengerState> scaffoldMessengerKey =
        GlobalKey<ScaffoldMessengerState>();

    final ColorScheme colorScheme = ColorScheme.fromSeed(seedColor: Colors.deepPurple);
    final ThemeData customTheme = ThemeData(
      colorScheme: colorScheme,
      visualDensity: VisualDensity.comfortable,
    );

    await tester.pumpWidget(
      MaterialApp(
        scaffoldMessengerKey: scaffoldMessengerKey,
        theme: customTheme,
        home: const SizedBox.shrink(),
      ),
    );

    final ThemeData messengerTheme = Theme.of(scaffoldMessengerKey.currentContext!);
    expect(messengerTheme.colorScheme, colorScheme);
    expect(messengerTheme.visualDensity, VisualDensity.comfortable);
  });

  testWidgets('ScaffoldMessenger showSnackBar default animation', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return ElevatedButton(
                onPressed: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(content: Text('I am a snack bar.'), showCloseIcon: true),
                  );
                },
                child: const Text('Show SnackBar'),
              );
            },
          ),
        ),
      ),
    );

    // Tap the button to show the SnackBar.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 125)); // Advance the animation by 125ms.

    // The SnackBar is partially visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    await tester.pump(const Duration(milliseconds: 125)); // Advance the animation by 125ms.

    // The SnackBar is fully visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(566, 0.1));

    // Tap the close button to dismiss the SnackBar.
    await tester.tap(find.byType(IconButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 125)); // Advance the animation by 125ms.

    // The SnackBar is partially visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    await tester.pump(const Duration(milliseconds: 125)); // Advance the animation by 125ms.

    // The SnackBar is dismissed.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(614, 0.1));
  });

  testWidgets('ScaffoldMessenger showSnackBar animation can be customized', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return ElevatedButton(
                onPressed: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(content: Text('I am a snack bar.'), showCloseIcon: true),
                    snackBarAnimationStyle: AnimationStyle(
                      duration: const Duration(milliseconds: 1200),
                      reverseDuration: const Duration(milliseconds: 600),
                    ),
                  );
                },
                child: const Text('Show SnackBar'),
              );
            },
          ),
        ),
      ),
    );

    // Tap the button to show the SnackBar.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 300)); // Advance the animation by 300ms.

    // The SnackBar is partially visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(602.6, 0.1));

    await tester.pump(const Duration(milliseconds: 300)); // Advance the animation by 300ms.

    // The SnackBar is partially visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    await tester.pump(const Duration(milliseconds: 600)); // Advance the animation by 600ms.

    // The SnackBar is fully visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(566, 0.1));

    // Tap the close button to dismiss the SnackBar.
    await tester.tap(find.byType(IconButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 300)); // Advance the animation by 300ns.

    // The SnackBar is partially visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    await tester.pump(const Duration(milliseconds: 300)); // Advance the animation by 300ms.

    // The SnackBar is dismissed.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(614, 0.1));
  });

  testWidgets('Updated snackBarAnimationStyle updates snack bar animation', (
    WidgetTester tester,
  ) async {
    Widget buildSnackBar(AnimationStyle snackBarAnimationStyle) {
      return MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return ElevatedButton(
                onPressed: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(content: Text('I am a snack bar.'), showCloseIcon: true),
                    snackBarAnimationStyle: snackBarAnimationStyle,
                  );
                },
                child: const Text('Show SnackBar'),
              );
            },
          ),
        ),
      );
    }

    // Test custom animation style.
    await tester.pumpWidget(
      buildSnackBar(
        AnimationStyle(
          duration: const Duration(milliseconds: 800),
          reverseDuration: const Duration(milliseconds: 400),
        ),
      ),
    );

    // Tap the button to show the SnackBar.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 400)); // Advance the animation by 400ms.

    // The SnackBar is partially visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    await tester.pump(const Duration(milliseconds: 400)); // Advance the animation by 400ms.

    // The SnackBar is fully visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(566, 0.1));

    // Tap the close button to dismiss the SnackBar.
    await tester.tap(find.byType(IconButton));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 400)); // Advance the animation by 400ms.

    // The SnackBar is dismissed.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(614, 0.1));

    // Test no animation style.
    await tester.pumpWidget(buildSnackBar(AnimationStyle.noAnimation));
    await tester.pumpAndSettle();

    // Tap the button to show the SnackBar.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();

    // The SnackBar is fully visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(566, 0.1));

    // Tap the close button to dismiss the SnackBar.
    await tester.tap(find.byType(IconButton));
    await tester.pump();

    // The SnackBar is dismissed.
    expect(find.text('I am a snack bar.'), findsNothing);
  });

  testWidgets('snackBarAnimationStyle with only reverseDuration uses default forward duration', (
    WidgetTester tester,
  ) async {
    Widget buildSnackBar(AnimationStyle snackBarAnimationStyle) {
      return MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return ElevatedButton(
                onPressed: () {
                  ScaffoldMessenger.of(context).showSnackBar(
                    const SnackBar(content: Text('I am a snack bar.'), showCloseIcon: true),
                    snackBarAnimationStyle: snackBarAnimationStyle,
                  );
                },
                child: const Text('Show SnackBar'),
              );
            },
          ),
        ),
      );
    }

    // Test custom animation style with only reverseDuration.
    await tester.pumpWidget(
      buildSnackBar(AnimationStyle(reverseDuration: const Duration(milliseconds: 400))),
    );

    // Tap the button to show the SnackBar.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();
    // Advance the animation by 1/2 of the default forward duration.
    await tester.pump(const Duration(milliseconds: 125));

    // The SnackBar is partially visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    // Advance the animation by 1/2 of the default forward duration.
    await tester.pump(const Duration(milliseconds: 125)); // Advance the animation by 125ms.

    // The SnackBar is fully visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(566, 0.1));

    // Tap the close button to dismiss the SnackBar.
    await tester.tap(find.byType(IconButton));
    await tester.pump();
    // Advance the animation by 1/2 of the reverse duration.
    await tester.pump(const Duration(milliseconds: 200));

    // The SnackBar is partially visible.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(576.7, 0.1));

    // Advance the animation by 1/2 of the reverse duration.
    await tester.pump(const Duration(milliseconds: 200)); // Advance the animation by 200ms.

    // The SnackBar is dismissed.
    expect(tester.getTopLeft(find.text('I am a snack bar.')).dy, closeTo(614, 0.1));
  });

  testWidgets('Scaffold showBottomSheet default animation', (WidgetTester tester) async {
    final Key sheetKey = UniqueKey();

    // Test default bottom sheet animation.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  Scaffold.of(context).showBottomSheet((BuildContext context) {
                    return SizedBox.expand(
                      child: ColoredBox(
                        key: sheetKey,
                        color: Theme.of(context).colorScheme.primary,
                        child: FilledButton(
                          onPressed: () {
                            Navigator.pop(context);
                          },
                          child: const Text('Close'),
                        ),
                      ),
                    );
                  });
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      ),
    );

    // Tap the 'X' to show the bottom sheet.
    await tester.tap(find.text('X'));
    await tester.pump();
    // Advance the animation by 1/2 of the default forward duration.
    await tester.pump(const Duration(milliseconds: 125));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(134.6, 0.1));

    // Advance the animation by 1/2 of the default forward duration.
    await tester.pump(const Duration(milliseconds: 125));

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(0.0));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(FilledButton, 'Close'));
    await tester.pump();
    // Advance the animation by 1/2 of the default reverse duration.
    await tester.pump(const Duration(milliseconds: 100));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(134.6, 0.1));

    // Advance the animation by 1/2 of the default reverse duration.
    await tester.pump(const Duration(milliseconds: 100));

    // The bottom sheet is dismissed.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(600.0));
  });

  testWidgets('Scaffold showBottomSheet animation can be customized', (WidgetTester tester) async {
    final Key sheetKey = UniqueKey();

    Widget buildWidget({AnimationStyle? sheetAnimationStyle}) {
      return MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () {
                  Scaffold.of(context).showBottomSheet(sheetAnimationStyle: sheetAnimationStyle, (
                    BuildContext context,
                  ) {
                    return SizedBox.expand(
                      child: ColoredBox(
                        key: sheetKey,
                        color: Theme.of(context).colorScheme.primary,
                        child: FilledButton(
                          onPressed: () {
                            Navigator.pop(context);
                          },
                          child: const Text('Close'),
                        ),
                      ),
                    );
                  });
                },
                child: const Text('X'),
              );
            },
          ),
        ),
      );
    }

    // Test custom animation style.
    await tester.pumpWidget(
      buildWidget(
        sheetAnimationStyle: AnimationStyle(
          duration: const Duration(milliseconds: 800),
          reverseDuration: const Duration(milliseconds: 400),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pump();
    // Advance the animation by 1/2 of the custom forward duration.
    await tester.pump(const Duration(milliseconds: 400));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(134.6, 0.1));

    // Advance the animation by 1/2 of the custom forward duration.
    await tester.pump(const Duration(milliseconds: 400));

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(0.0));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(FilledButton, 'Close'));
    await tester.pump();
    // Advance the animation by 1/2 of the custom reverse duration.
    await tester.pump(const Duration(milliseconds: 200));

    // The bottom sheet is partially visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, closeTo(134.6, 0.1));

    // Advance the animation by 1/2 of the custom reverse duration.
    await tester.pump(const Duration(milliseconds: 200));

    // The bottom sheet is dismissed.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(600.0));

    // Test no animation style.
    await tester.pumpWidget(buildWidget(sheetAnimationStyle: AnimationStyle.noAnimation));
    await tester.pumpAndSettle();
    await tester.tap(find.text('X'));
    await tester.pump();

    // The bottom sheet is fully visible.
    expect(tester.getTopLeft(find.byKey(sheetKey)).dy, equals(0.0));

    // Dismiss the bottom sheet.
    await tester.tap(find.widgetWithText(FilledButton, 'Close'));
    await tester.pump();

    // The bottom sheet is dismissed.
    expect(find.byKey(sheetKey), findsNothing);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/145585.
  testWidgets('FAB default entrance and exit animations', (WidgetTester tester) async {
    bool showFab = false;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: ElevatedButton(
                onPressed: () {
                  setState(() {
                    showFab = !showFab;
                  });
                },
                child: const Text('Toggle FAB'),
              ),
              floatingActionButton:
                  !showFab
                      ? null
                      : FloatingActionButton(onPressed: () {}, child: const Icon(Icons.add)),
            );
          },
        ),
      ),
    );

    // FAB is not visible.
    expect(find.byType(FloatingActionButton), findsNothing);

    // Tap the button to show the FAB.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Toggle FAB'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100)); // Advance the animation by 100ms.
    // FAB is partially animated in.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, closeTo(743.8, 0.1));

    await tester.pump(const Duration(milliseconds: 100)); // Advance the animation by 100ms.
    // FAB is fully animated in.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, equals(728.0));

    // Tap the button to hide the FAB.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Toggle FAB'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100)); // Advance the animation by 100ms.
    // FAB is partially animated out.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, closeTo(747.1, 0.1));

    await tester.pump(const Duration(milliseconds: 100)); // Advance the animation by 100ms.
    // FAB is fully animated out.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, equals(756.0));

    await tester.pump(const Duration(milliseconds: 50)); // Advance the animation by 50ms.
    // FAB is not visible.
    expect(find.byType(FloatingActionButton), findsNothing);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/145585.
  testWidgets('FAB default entrance and exit animations can be disabled', (
    WidgetTester tester,
  ) async {
    bool showFab = false;
    FloatingActionButtonLocation fabLocation = FloatingActionButtonLocation.endFloat;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              // Disable FAB animations.
              floatingActionButtonAnimator: FloatingActionButtonAnimator.noAnimation,
              floatingActionButtonLocation: fabLocation,
              body: Column(
                children: <Widget>[
                  ElevatedButton(
                    onPressed: () {
                      setState(() {
                        showFab = !showFab;
                      });
                    },
                    child: const Text('Toggle FAB'),
                  ),
                  ElevatedButton(
                    onPressed: () {
                      setState(() {
                        fabLocation = FloatingActionButtonLocation.centerFloat;
                      });
                    },
                    child: const Text('Update FAB Location'),
                  ),
                ],
              ),
              floatingActionButton:
                  !showFab
                      ? null
                      : FloatingActionButton(onPressed: () {}, child: const Icon(Icons.add)),
            );
          },
        ),
      ),
    );

    // FAB is not visible.
    expect(find.byType(FloatingActionButton), findsNothing);

    // Tap the button to show the FAB.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Toggle FAB'));
    await tester.pump();
    // FAB is visible.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, equals(728.0));

    // Tap the button to hide the FAB.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Toggle FAB'));
    await tester.pump();
    // FAB is not visible.
    expect(find.byType(FloatingActionButton), findsNothing);

    // Tap the button to show the FAB.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Toggle FAB'));
    await tester.pump();
    // FAB is visible.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, equals(728.0));

    // Tap the update location button.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Update FAB Location'));
    await tester.pump();

    // FAB is visible at the new location.
    expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, equals(372.0));

    // Tap the button to hide the FAB.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Toggle FAB'));
    await tester.pump();
    // FAB is not visible.
    expect(find.byType(FloatingActionButton), findsNothing);
  });

  testWidgets('Scaffold background color defaults to ColorScheme.surface', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(
      colorScheme: ThemeData().colorScheme.copyWith(
        surface: Colors.orange,
        background: Colors.green,
      ),
    );
    await tester.pumpWidget(
      MaterialApp(theme: theme, home: const Scaffold(body: SizedBox.expand())),
    );

    final Material scaffoldMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(Scaffold), matching: find.byType(Material).first),
    );
    expect(scaffoldMaterial.color, theme.colorScheme.surface);
  });

  testWidgets(
    'Body height remains Scaffold height when keyboard is smaller than bottomNavigationBar and extendBody is true',
    (WidgetTester tester) async {
      final Key bodyKey = UniqueKey();
      Widget buildFrame({double keyboardHeight = 0}) {
        return MaterialApp(
          home: Builder(
            builder: (BuildContext context) {
              return MediaQuery(
                data: MediaQuery.of(
                  context,
                ).copyWith(viewInsets: EdgeInsets.only(bottom: keyboardHeight)),
                child: Scaffold(
                  extendBody: true,
                  body: SizedBox.expand(key: bodyKey),
                  bottomNavigationBar: const SizedBox(height: 100),
                ),
              );
            },
          ),
        );
      }

      await tester.pumpWidget(buildFrame());
      expect(tester.getSize(find.byKey(bodyKey)).height, 600);

      await tester.pumpWidget(buildFrame(keyboardHeight: 100));
      expect(tester.getSize(find.byKey(bodyKey)).height, 600);

      await tester.pumpWidget(buildFrame(keyboardHeight: 200));
      expect(tester.getSize(find.byKey(bodyKey)).height, 400);
    },
  );
}

class _GeometryListener extends StatefulWidget {
  const _GeometryListener();

  @override
  _GeometryListenerState createState() => _GeometryListenerState();
}

class _GeometryListenerState extends State<_GeometryListener> {
  @override
  Widget build(BuildContext context) {
    return CustomPaint(painter: cache);
  }

  int numNotifications = 0;
  ValueListenable<ScaffoldGeometry>? geometryListenable;
  late _GeometryCachePainter cache;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final ValueListenable<ScaffoldGeometry> newListenable = Scaffold.geometryOf(context);
    if (geometryListenable == newListenable) {
      return;
    }

    geometryListenable?.removeListener(onGeometryChanged);
    geometryListenable = newListenable..addListener(onGeometryChanged);
    cache = _GeometryCachePainter(geometryListenable!);
  }

  void onGeometryChanged() {
    numNotifications += 1;
  }
}

// The Scaffold.geometryOf() value is only available at paint time.
// To fetch it for the tests we implement this CustomPainter that just
// caches the ScaffoldGeometry value in its paint method.
class _GeometryCachePainter extends CustomPainter {
  _GeometryCachePainter(this.geometryListenable) : super(repaint: geometryListenable);

  final ValueListenable<ScaffoldGeometry> geometryListenable;

  late ScaffoldGeometry value;
  @override
  void paint(Canvas canvas, Size size) {
    value = geometryListenable.value;
  }

  @override
  bool shouldRepaint(_GeometryCachePainter oldDelegate) {
    return true;
  }
}

class _CustomPageRoute<T> extends PageRoute<T> {
  _CustomPageRoute({
    required this.builder,
    RouteSettings super.settings = const RouteSettings(),
    this.maintainState = true,
    super.fullscreenDialog,
  });

  final WidgetBuilder builder;

  @override
  Duration get transitionDuration => const Duration(milliseconds: 300);

  @override
  Color? get barrierColor => null;

  @override
  String? get barrierLabel => null;

  @override
  final bool maintainState;

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return builder(context);
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return child;
  }
}
