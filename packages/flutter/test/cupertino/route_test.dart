// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('!chrome')
library;

import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  late MockNavigatorObserver navigatorObserver;

  setUp(() {
    navigatorObserver = MockNavigatorObserver();
  });

  testWidgets('Middle auto-populates with title', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Placeholder()));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            title: 'An iPod',
            builder: (BuildContext context) {
              return const CupertinoPageScaffold(
                navigationBar: CupertinoNavigationBar(),
                child: Placeholder(),
              );
            },
          ),
        );

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    // There should be a Text widget with the title in the nav bar even though
    // we didn't specify anything in the nav bar constructor.
    expect(find.widgetWithText(CupertinoNavigationBar, 'An iPod'), findsOneWidget);

    // As a title, it should also be centered.
    expect(tester.getCenter(find.text('An iPod')).dx, 400.0);
  });

  testWidgets('Large title auto-populates with title', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Placeholder()));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            title: 'An iPod',
            builder: (BuildContext context) {
              return const CupertinoPageScaffold(
                child: CustomScrollView(slivers: <Widget>[CupertinoSliverNavigationBar()]),
              );
            },
          ),
        );

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    // There should be 2 Text widget with the title in the nav bar. One in the
    // large title position and one in the middle position (though the middle
    // position Text is initially invisible while the sliver is expanded).
    expect(find.widgetWithText(CupertinoSliverNavigationBar, 'An iPod'), findsNWidgets(2));

    final List<Element> titles =
        tester.elementList(find.text('An iPod')).toList()..sort((Element a, Element b) {
          final RenderParagraph aParagraph = a.renderObject! as RenderParagraph;
          final RenderParagraph bParagraph = b.renderObject! as RenderParagraph;
          return aParagraph.text.style!.fontSize!.compareTo(bParagraph.text.style!.fontSize!);
        });

    final Iterable<double> opacities = titles.map<double>((Element element) {
      final RenderAnimatedOpacity renderOpacity =
          element.findAncestorRenderObjectOfType<RenderAnimatedOpacity>()!;
      return renderOpacity.opacity.value;
    });

    expect(opacities, <double>[
      0.0, // Initially the smaller font title is invisible.
      1.0, // The larger font title is visible.
    ]);

    // Check that the large font title is at the right spot.
    expect(tester.getTopLeft(find.byWidget(titles[1].widget)), const Offset(16.0, 54.0));

    // The smaller, initially invisible title, should still be positioned in the
    // center.
    expect(tester.getCenter(find.byWidget(titles[0].widget)).dx, 400.0);
  });

  testWidgets('Leading auto-populates with back button with previous title', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const CupertinoApp(home: Placeholder()));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            title: 'An iPod',
            builder: (BuildContext context) {
              return const CupertinoPageScaffold(
                navigationBar: CupertinoNavigationBar(),
                child: Placeholder(),
              );
            },
          ),
        );

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 600));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            title: 'A Phone',
            builder: (BuildContext context) {
              return const CupertinoPageScaffold(
                navigationBar: CupertinoNavigationBar(),
                child: Placeholder(),
              );
            },
          ),
        );

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 600));

    expect(find.widgetWithText(CupertinoNavigationBar, 'A Phone'), findsOneWidget);
    expect(tester.getCenter(find.text('A Phone')).dx, 400.0);

    // Also shows the previous page's title next to the back button.
    expect(find.widgetWithText(CupertinoButton, 'An iPod'), findsOneWidget);
    // 3 paddings + 1 test font character at font size 34.0.
    // The epsilon is needed since the text theme has a negative letter spacing thus.
    expect(
      tester.getTopLeft(find.text('An iPod')).dx,
      moreOrLessEquals(8.0 + 4.0 + 34.0 + 6.0, epsilon: 0.5),
    );
  });

  testWidgets('Previous title is correct on first transition frame', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Placeholder()));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            title: 'An iPod',
            builder: (BuildContext context) {
              return const CupertinoPageScaffold(
                navigationBar: CupertinoNavigationBar(),
                child: Placeholder(),
              );
            },
          ),
        );

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .push(
          CupertinoPageRoute<void>(
            title: 'A Phone',
            builder: (BuildContext context) {
              return const CupertinoPageScaffold(
                navigationBar: CupertinoNavigationBar(),
                child: Placeholder(),
              );
            },
          ),
        );

    // Trigger the route push
    await tester.pump();
    // Draw the first frame.
    await tester.pump();

    // Also shows the previous page's title next to the back button.
    expect(find.widgetWithText(CupertinoButton, 'An iPod'), findsOneWidget);
  });

  testWidgets('Previous title stays up to date with changing routes', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Placeholder()));

    final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
      title: 'An iPod',
      builder: (BuildContext context) {
        return const CupertinoPageScaffold(
          navigationBar: CupertinoNavigationBar(),
          child: Placeholder(),
        );
      },
    );

    final CupertinoPageRoute<void> route3 = CupertinoPageRoute<void>(
      title: 'A Phone',
      builder: (BuildContext context) {
        return const CupertinoPageScaffold(
          navigationBar: CupertinoNavigationBar(),
          child: Placeholder(),
        );
      },
    );

    tester.state<NavigatorState>(find.byType(Navigator)).push(route2);

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    tester.state<NavigatorState>(find.byType(Navigator)).push(route3);

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .replace(
          oldRoute: route2,
          newRoute: CupertinoPageRoute<void>(
            title: 'An Internet communicator',
            builder: (BuildContext context) {
              return const CupertinoPageScaffold(
                navigationBar: CupertinoNavigationBar(),
                child: Placeholder(),
              );
            },
          ),
        );

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    expect(find.widgetWithText(CupertinoNavigationBar, 'A Phone'), findsOneWidget);
    expect(tester.getCenter(find.text('A Phone')).dx, 400.0);

    // After swapping the route behind the top one, the previous label changes
    // from An iPod to Back (since An Internet communicator is too long to
    // fit in the back button).
    expect(find.widgetWithText(CupertinoButton, 'Back'), findsOneWidget);
    // The epsilon is needed since the text theme has a negative letter spacing thus.
    expect(
      tester.getTopLeft(find.text('Back')).dx,
      moreOrLessEquals(8.0 + 4.0 + 34.0 + 6.0, epsilon: 0.5),
    );
  });

  testWidgets('Back swipe dismiss interrupted by route push', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/28728
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: CupertinoButton(
              onPressed: () {
                Navigator.push<void>(
                  scaffoldKey.currentContext!,
                  CupertinoPageRoute<void>(
                    builder: (BuildContext context) {
                      return const CupertinoPageScaffold(child: Center(child: Text('route')));
                    },
                  ),
                );
              },
              child: const Text('push'),
            ),
          ),
        ),
      ),
    );

    // Check the basic iOS back-swipe dismiss transition. Dragging the pushed
    // route halfway across the screen will trigger the iOS dismiss animation

    await tester.tap(find.text('push'));
    await tester.pumpAndSettle();
    expect(find.text('route'), findsOneWidget);
    expect(find.text('push'), findsNothing);

    TestGesture gesture = await tester.startGesture(const Offset(5, 300));
    await gesture.moveBy(const Offset(400, 0));
    await gesture.up();
    await tester.pump();
    expect(
      // The 'route' route has been dragged to the right, halfway across the screen
      tester.getTopLeft(
        find.ancestor(of: find.text('route'), matching: find.byType(CupertinoPageScaffold)),
      ),
      const Offset(400, 0),
    );
    expect(
      // The 'push' route is sliding in from the left.
      tester
          .getTopLeft(
            find.ancestor(of: find.text('push'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dx,
      lessThan(0),
    );
    await tester.pumpAndSettle();
    expect(find.text('push'), findsOneWidget);
    expect(
      tester.getTopLeft(
        find.ancestor(of: find.text('push'), matching: find.byType(CupertinoPageScaffold)),
      ),
      Offset.zero,
    );
    expect(find.text('route'), findsNothing);

    // Run the dismiss animation 60%, which exposes the route "push" button,
    // and then press the button.

    await tester.tap(find.text('push'));
    await tester.pumpAndSettle();
    expect(find.text('route'), findsOneWidget);
    expect(find.text('push'), findsNothing);

    gesture = await tester.startGesture(const Offset(5, 300));
    await gesture.moveBy(const Offset(400, 0)); // Drag halfway.
    await gesture.up();
    // Trigger the snapping animation.
    // Since the back swipe drag was brought to >=50% of the screen, it will
    // self snap to finish the pop transition as the gesture is lifted.
    //
    // This drag drop animation is 400ms when dropped exactly halfway
    // (800 / [pixel distance remaining], see
    // _CupertinoBackGestureController.dragEnd). It follows a curve that is very
    // steep initially.
    await tester.pump();
    expect(
      tester.getTopLeft(
        find.ancestor(of: find.text('route'), matching: find.byType(CupertinoPageScaffold)),
      ),
      const Offset(400, 0),
    );
    // Let the dismissing snapping animation go 60%.
    await tester.pump(const Duration(milliseconds: 210));
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('route'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dx,
      moreOrLessEquals(789, epsilon: 1),
    );

    // Use the navigator to push a route instead of tapping the 'push' button.
    // The topmost route (the one that's animating away), ignores input while
    // the pop is underway because route.navigator.userGestureInProgress.
    Navigator.push<void>(
      scaffoldKey.currentContext!,
      CupertinoPageRoute<void>(
        builder: (BuildContext context) {
          return const CupertinoPageScaffold(child: Center(child: Text('route')));
        },
      ),
    );

    await tester.pumpAndSettle();
    expect(find.text('route'), findsOneWidget);
    expect(find.text('push'), findsNothing);
    expect(tester.state<NavigatorState>(find.byType(Navigator)).userGestureInProgress, false);
  });

  testWidgets('Back swipe less than halfway is interrupted by route pop', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/141268
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    Navigator.push<void>(
                      scaffoldKey.currentContext!,
                      CupertinoPageRoute<void>(
                        builder: (BuildContext context) {
                          return const CupertinoPageScaffold(child: Center(child: Text('Page 2')));
                        },
                      ),
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), findsOneWidget);

    // Start a back gesture and move it less than 50% across the screen.
    final TestGesture gesture = await tester.startGesture(const Offset(5.0, 300.0));
    await gesture.moveBy(const Offset(100.0, 0.0));
    await tester.pump();
    expect(
      // The second route has been dragged to the right.
      tester.getTopLeft(
        find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
      ),
      const Offset(100.0, 0.0),
    );
    expect(
      // The first route is sliding in from the left.
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dx,
      lessThan(0),
    );

    // Programmatically pop and observe that Page 2 was popped as if there were
    // no back gesture.
    Navigator.pop<void>(scaffoldKey.currentContext!);
    await tester.pumpAndSettle();
    expect(
      tester.getTopLeft(
        find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
      ),
      Offset.zero,
    );
    expect(find.text('Page 2'), findsNothing);
  });

  testWidgets('Back swipe more than halfway is interrupted by route pop', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/141268
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    Navigator.push<void>(
                      scaffoldKey.currentContext!,
                      CupertinoPageRoute<void>(
                        builder: (BuildContext context) {
                          return const CupertinoPageScaffold(child: Center(child: Text('Page 2')));
                        },
                      ),
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), findsOneWidget);

    // Start a back gesture and move it more than 50% across the screen.
    final TestGesture gesture = await tester.startGesture(const Offset(5.0, 300.0));
    await gesture.moveBy(const Offset(500.0, 0.0));
    await tester.pump();
    expect(
      // The second route has been dragged to the right.
      tester.getTopLeft(
        find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
      ),
      const Offset(500.0, 0.0),
    );
    expect(
      // The first route is sliding in from the left.
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dx,
      lessThan(0),
    );

    // Programmatically pop and observe that Page 2 was popped as if there were
    // no back gesture.
    Navigator.pop<void>(scaffoldKey.currentContext!);
    await tester.pumpAndSettle();
    expect(
      tester.getTopLeft(
        find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
      ),
      Offset.zero,
    );
    expect(find.text('Page 2'), findsNothing);
  });

  testWidgets('Back swipe less than halfway is interrupted by route push', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/141268
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    Navigator.push<void>(
                      scaffoldKey.currentContext!,
                      CupertinoPageRoute<void>(
                        builder: (BuildContext context) {
                          return const CupertinoPageScaffold(child: Center(child: Text('Page 2')));
                        },
                      ),
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), findsOneWidget);

    // Start a back gesture and move it less than 50% across the screen.
    final TestGesture gesture = await tester.startGesture(const Offset(5.0, 300.0));
    await gesture.moveBy(const Offset(100.0, 0.0));
    await tester.pump();
    expect(
      // The second route has been dragged to the right.
      tester.getTopLeft(
        find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
      ),
      const Offset(100.0, 0.0),
    );
    expect(
      // The first route is sliding in from the left.
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dx,
      lessThan(0),
    );

    // Programmatically push and observe that Page 3 was pushed as if there were
    // no back gesture.
    Navigator.push<void>(
      scaffoldKey.currentContext!,
      CupertinoPageRoute<void>(
        builder: (BuildContext context) {
          return const CupertinoPageScaffold(child: Center(child: Text('Page 3')));
        },
      ),
    );
    await tester.pumpAndSettle();
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), findsNothing);
    expect(
      tester.getTopLeft(
        find.ancestor(of: find.text('Page 3'), matching: find.byType(CupertinoPageScaffold)),
      ),
      Offset.zero,
    );
  });

  testWidgets('Back swipe more than halfway is interrupted by route push', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/141268
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    Navigator.push<void>(
                      scaffoldKey.currentContext!,
                      CupertinoPageRoute<void>(
                        builder: (BuildContext context) {
                          return const CupertinoPageScaffold(child: Center(child: Text('Page 2')));
                        },
                      ),
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), findsOneWidget);

    // Start a back gesture and move it more than 50% across the screen.
    final TestGesture gesture = await tester.startGesture(const Offset(5.0, 300.0));
    await gesture.moveBy(const Offset(500.0, 0.0));
    await tester.pump();
    expect(
      // The second route has been dragged to the right.
      tester.getTopLeft(
        find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
      ),
      const Offset(500.0, 0.0),
    );
    expect(
      // The first route is sliding in from the left.
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dx,
      lessThan(0),
    );

    // Programmatically push and observe that Page 3 was pushed as if there were
    // no back gesture.
    Navigator.push<void>(
      scaffoldKey.currentContext!,
      CupertinoPageRoute<void>(
        builder: (BuildContext context) {
          return const CupertinoPageScaffold(child: Center(child: Text('Page 3')));
        },
      ),
    );
    await tester.pumpAndSettle();
    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), findsNothing);
    expect(
      tester.getTopLeft(
        find.ancestor(of: find.text('Page 3'), matching: find.byType(CupertinoPageScaffold)),
      ),
      Offset.zero,
    );
  });

  testWidgets('Fullscreen route animates correct transform values over time', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Builder(
          builder: (BuildContext context) {
            return CupertinoButton(
              child: const Text('Button'),
              onPressed: () {
                Navigator.push<void>(
                  context,
                  CupertinoPageRoute<void>(
                    fullscreenDialog: true,
                    builder: (BuildContext context) {
                      return Column(
                        children: <Widget>[
                          const Placeholder(),
                          CupertinoButton(
                            child: const Text('Close'),
                            onPressed: () {
                              Navigator.pop<void>(context);
                            },
                          ),
                        ],
                      );
                    },
                  ),
                );
              },
            );
          },
        ),
      ),
    );

    // Enter animation.
    await tester.tap(find.text('Button'));
    await tester.pump();

    // We use a higher number of intervals since the animation has to scale the
    // entire screen.

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(475.6, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(350.0, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(237.4, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(149.2, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(89.5, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(54.4, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(33.2, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(20.4, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(12.6, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(7.4, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(3, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(0, epsilon: 0.1));

    // Give time to the animation to finish and update its status to
    // AnimationState.completed, so the reverse curved can be used in the next
    // step.
    await tester.pumpAndSettle(const Duration(milliseconds: 1));

    // Exit animation
    await tester.tap(find.text('Close'));
    await tester.pump();

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(156.3, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(308.1, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(411.03, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(484.35, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(530.67, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(557.61, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(573.88, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(583.86, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(590.26, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(594.58, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(597.66, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dy, moreOrLessEquals(600.0, epsilon: 0.1));
  });

  Future<void> testParallax(WidgetTester tester, {required bool fromFullscreenDialog}) async {
    await tester.pumpWidget(
      CupertinoApp(
        onGenerateRoute:
            (RouteSettings settings) => CupertinoPageRoute<void>(
              fullscreenDialog: fromFullscreenDialog,
              settings: settings,
              builder: (BuildContext context) {
                return Column(
                  children: <Widget>[
                    const Placeholder(),
                    CupertinoButton(
                      child: const Text('Button'),
                      onPressed: () {
                        Navigator.push<void>(
                          context,
                          CupertinoPageRoute<void>(
                            builder: (BuildContext context) {
                              return CupertinoButton(
                                child: const Text('Close'),
                                onPressed: () {
                                  Navigator.pop<void>(context);
                                },
                              );
                            },
                          ),
                        );
                      },
                    ),
                  ],
                );
              },
            ),
      ),
    );

    // Enter animation.
    await tester.tap(find.text('Button'));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(0.0, epsilon: 0.1));
    await tester.pump();

    // We use a higher number of intervals since the animation has to scale the
    // entire screen.

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-55.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-111.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-161.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-200.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-226.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-242.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-251.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-257.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-261.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-263.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-265.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-266.0, epsilon: 1.0));

    // Give time to the animation to finish and update its status to
    // AnimationState.completed, so the reverse curved can be used in the next
    // step.
    await tester.pumpAndSettle(const Duration(milliseconds: 1));

    // Exit animation
    await tester.tap(find.text('Close'));
    await tester.pump();

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-197.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-129.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-83.0, epsilon: 1.0));

    await tester.pump(const Duration(milliseconds: 360));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-0.0, epsilon: 1.0));
  }

  testWidgets('CupertinoPageRoute has parallax when non fullscreenDialog route is pushed on top', (
    WidgetTester tester,
  ) async {
    await testParallax(tester, fromFullscreenDialog: false);
  });

  testWidgets(
    'FullscreenDialog CupertinoPageRoute has parallax when non fullscreenDialog route is pushed on top',
    (WidgetTester tester) async {
      await testParallax(tester, fromFullscreenDialog: true);
    },
  );

  group('Interrupted push', () {
    Future<void> testParallax(WidgetTester tester, {required bool fromFullscreenDialog}) async {
      await tester.pumpWidget(
        CupertinoApp(
          onGenerateRoute:
              (RouteSettings settings) => CupertinoPageRoute<void>(
                fullscreenDialog: fromFullscreenDialog,
                settings: settings,
                builder: (BuildContext context) {
                  return Column(
                    children: <Widget>[
                      const Placeholder(),
                      CupertinoButton(
                        child: const Text('Button'),
                        onPressed: () {
                          Navigator.push<void>(
                            context,
                            CupertinoPageRoute<void>(
                              builder: (BuildContext context) {
                                return CupertinoButton(
                                  child: const Text('Close'),
                                  onPressed: () {
                                    Navigator.pop<void>(context);
                                  },
                                );
                              },
                            ),
                          );
                        },
                      ),
                    ],
                  );
                },
              ),
        ),
      );

      // Enter animation.
      await tester.tap(find.text('Button'));
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(0.0, epsilon: 0.1));
      await tester.pump();

      // The push animation duration is 500ms. We let it run for 400ms before
      // interrupting and popping it.

      await tester.pump(const Duration(milliseconds: 40));
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-55.0, epsilon: 1.0));

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-111.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-161.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-200.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-226.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-242.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-251.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-257.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-261.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-263.0, epsilon: 1.0),
      );

      // Exit animation
      await tester.tap(find.text('Close'));
      await tester.pump();

      // When the push animation is interrupted, the forward curved is used for
      // the reversed animation to avoid discontinuities.

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-261.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-257.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-251.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-242.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-226.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-200.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-161.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(
        tester.getTopLeft(find.byType(Placeholder)).dx,
        moreOrLessEquals(-111.0, epsilon: 1.0),
      );

      await tester.pump(const Duration(milliseconds: 40));
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(-55.0, epsilon: 1.0));

      await tester.pump(const Duration(milliseconds: 40));
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(0.0, epsilon: 1.0));
    }

    testWidgets(
      'CupertinoPageRoute has parallax when non fullscreenDialog route is pushed on top and gets popped before the end of the animation',
      (WidgetTester tester) async {
        await testParallax(tester, fromFullscreenDialog: false);
      },
    );

    testWidgets(
      'FullscreenDialog CupertinoPageRoute has parallax when non fullscreenDialog route is pushed on top and gets popped before the end of the animation',
      (WidgetTester tester) async {
        await testParallax(tester, fromFullscreenDialog: true);
      },
    );
  });

  Future<void> testNoParallax(WidgetTester tester, {required bool fromFullscreenDialog}) async {
    await tester.pumpWidget(
      CupertinoApp(
        onGenerateRoute:
            (RouteSettings settings) => CupertinoPageRoute<void>(
              fullscreenDialog: fromFullscreenDialog,
              builder: (BuildContext context) {
                return Column(
                  children: <Widget>[
                    const Placeholder(),
                    CupertinoButton(
                      child: const Text('Button'),
                      onPressed: () {
                        Navigator.push<void>(
                          context,
                          CupertinoPageRoute<void>(
                            fullscreenDialog: true,
                            builder: (BuildContext context) {
                              return CupertinoButton(
                                child: const Text('Close'),
                                onPressed: () {
                                  Navigator.pop<void>(context);
                                },
                              );
                            },
                          ),
                        );
                      },
                    ),
                  ],
                );
              },
            ),
      ),
    );

    // Enter animation.
    await tester.tap(find.text('Button'));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, moreOrLessEquals(0.0, epsilon: 0.1));
    await tester.pump();

    // We use a higher number of intervals since the animation has to scale the
    // entire screen.

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    // Exit animation
    await tester.tap(find.text('Close'));
    await tester.pump();

    await tester.pump(const Duration(milliseconds: 40));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);

    await tester.pump(const Duration(milliseconds: 360));
    expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);
  }

  testWidgets('CupertinoPageRoute has no parallax when fullscreenDialog route is pushed on top', (
    WidgetTester tester,
  ) async {
    await testNoParallax(tester, fromFullscreenDialog: false);
  });

  testWidgets(
    'FullscreenDialog CupertinoPageRoute has no parallax when fullscreenDialog route is pushed on top',
    (WidgetTester tester) async {
      await testNoParallax(tester, fromFullscreenDialog: true);
    },
  );

  testWidgets('Animated push/pop is not linear', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Text('1')));

    final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
      builder: (BuildContext context) {
        return const CupertinoPageScaffold(child: Text('2'));
      },
    );

    tester.state<NavigatorState>(find.byType(Navigator)).push(route2);
    // The whole transition is 500ms based on CupertinoPageRoute.transitionDuration.
    // Break it up into small chunks.
    //
    // The screen width is 800.
    // The top left corner of the text 1 will go from 0 to -800 / 3 = - 266.67.
    // The top left corner of the text 2 will go from 800 to 0.

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-69, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(609, epsilon: 1));

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-136, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(362, epsilon: 1));

    await tester.pump(const Duration(milliseconds: 50));
    // Translation slows down as time goes on.
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-191, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(192, epsilon: 1));

    // Finish the rest of the animation
    await tester.pump(const Duration(milliseconds: 350));
    // Give time to the animation to finish and update its status to
    // AnimationState.completed, so the reverse curved can be used in the next
    // step.
    await tester.pumpAndSettle(const Duration(milliseconds: 1));

    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    // The top left corner of the text 1 will go from -800 / 3 = - 266.67 to 0.
    // The top left corner of the text 2 will go from 0 to 800.
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-197, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(190, epsilon: 1));

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-129, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(437, epsilon: 1));

    await tester.pump(const Duration(milliseconds: 50));
    // Translation slows down as time goes on.
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-74, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(607, epsilon: 1));
  });

  testWidgets('Dragged pop gesture is linear', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Text('1')));

    final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
      builder: (BuildContext context) {
        return const CupertinoPageScaffold(child: Text('2'));
      },
    );

    tester.state<NavigatorState>(find.byType(Navigator)).push(route2);

    await tester.pumpAndSettle();

    expect(find.text('1'), findsNothing);
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(0));

    final TestGesture swipeGesture = await tester.startGesture(const Offset(5, 100));

    await swipeGesture.moveBy(const Offset(100, 0));
    await tester.pump();
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-233, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(100));
    expect(tester.state<NavigatorState>(find.byType(Navigator)).userGestureInProgress, true);

    await swipeGesture.moveBy(const Offset(100, 0));
    await tester.pump();
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-200));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(200));

    // Moving by the same distance each time produces linear movements on both
    // routes.
    await swipeGesture.moveBy(const Offset(100, 0));
    await tester.pump();
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-166, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(300));
  });

  // Regression test for https://github.com/flutter/flutter/issues/137033.
  testWidgets('Update pages during a drag gesture will not stuck', (WidgetTester tester) async {
    await tester.pumpWidget(const _TestPageUpdate());

    // Tap this button will update the pages in two seconds.
    await tester.tap(find.text('Update Pages'));
    await tester.pump();

    // Start swiping.
    final TestGesture swipeGesture = await tester.startGesture(const Offset(5, 100));
    await swipeGesture.moveBy(const Offset(100, 0));
    await tester.pump();

    expect(
      tester.stateList<NavigatorState>(find.byType(Navigator)).last.userGestureInProgress,
      true,
    );

    // Wait for pages to update.
    await tester.pump(const Duration(seconds: 3));

    // Verify pages are updated.
    expect(find.text('New page'), findsOneWidget);
    // Verify `userGestureInProgress` is set to false.
    expect(
      tester.stateList<NavigatorState>(find.byType(Navigator)).last.userGestureInProgress,
      false,
    );
  });

  testWidgets('Pop gesture snapping is not linear', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Text('1')));

    final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
      builder: (BuildContext context) {
        return const CupertinoPageScaffold(child: Text('2'));
      },
    );

    tester.state<NavigatorState>(find.byType(Navigator)).push(route2);

    await tester.pumpAndSettle();

    final TestGesture swipeGesture = await tester.startGesture(const Offset(5, 100));

    await swipeGesture.moveBy(const Offset(500, 0));
    await swipeGesture.up();
    await tester.pump();
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-100));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(500));
    expect(tester.state<NavigatorState>(find.byType(Navigator)).userGestureInProgress, true);

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-61, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(614, epsilon: 1));

    await tester.pump(const Duration(milliseconds: 50));
    // Rate of change is slowing down.
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(-26, epsilon: 1));
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(721, epsilon: 1));

    await tester.pumpAndSettle();
    expect(tester.state<NavigatorState>(find.byType(Navigator)).userGestureInProgress, false);
  });

  testWidgets('Snapped drags forwards and backwards should signal didStart/StopUserGesture', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> navigatorKey = GlobalKey();
    await tester.pumpWidget(
      CupertinoApp(
        navigatorObservers: <NavigatorObserver>[navigatorObserver],
        navigatorKey: navigatorKey,
        home: const Text('1'),
      ),
    );

    final CupertinoPageRoute<void> route2 = CupertinoPageRoute<void>(
      builder: (BuildContext context) {
        return const CupertinoPageScaffold(child: Text('2'));
      },
    );

    navigatorKey.currentState!.push(route2);
    await tester.pumpAndSettle();
    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didPush);

    await tester.dragFrom(const Offset(5, 100), const Offset(100, 0));
    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didStartUserGesture);
    await tester.pump();
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(100));
    expect(navigatorKey.currentState!.userGestureInProgress, true);

    // Didn't drag far enough to snap into dismissing this route.
    // Each 100px distance takes 100ms to snap back.
    await tester.pump(const Duration(milliseconds: 351));
    // Back to the page covering the whole screen.
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(0));
    expect(navigatorKey.currentState!.userGestureInProgress, false);

    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didStopUserGesture);
    expect(navigatorObserver.invocations.removeLast(), isNot(NavigatorInvocation.didPop));

    await tester.dragFrom(const Offset(5, 100), const Offset(500, 0));
    await tester.pump();
    expect(tester.getTopLeft(find.text('2')).dx, moreOrLessEquals(500));
    expect(navigatorKey.currentState!.userGestureInProgress, true);
    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didPop);

    // Did go far enough to snap out of this route.
    await tester.pump(const Duration(milliseconds: 351));
    // Back to the page covering the whole screen.
    expect(find.text('2'), findsNothing);
    // First route covers the whole screen.
    expect(tester.getTopLeft(find.text('1')).dx, moreOrLessEquals(0));
    expect(navigatorKey.currentState!.userGestureInProgress, false);
  });

  /// Regression test for https://github.com/flutter/flutter/issues/29596.
  testWidgets('test edge swipe then drop back at ending point works', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        navigatorObservers: <NavigatorObserver>[navigatorObserver],
        onGenerateRoute: (RouteSettings settings) {
          return CupertinoPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) {
              final String pageNumber = settings.name == '/' ? '1' : '2';
              return Center(child: Text('Page $pageNumber'));
            },
          );
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    final TestGesture gesture = await tester.startGesture(const Offset(5, 200));
    // The width of the page.
    await gesture.moveBy(const Offset(800, 0));
    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didStartUserGesture);
    await gesture.up();
    await tester.pump();

    expect(find.text('Page 1'), isOnstage);
    expect(find.text('Page 2'), findsNothing);
    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didStopUserGesture);
    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didPop);
  });

  testWidgets('test edge swipe then drop back at starting point works', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        navigatorObservers: <NavigatorObserver>[navigatorObserver],
        onGenerateRoute: (RouteSettings settings) {
          return CupertinoPageRoute<void>(
            settings: settings,
            builder: (BuildContext context) {
              final String pageNumber = settings.name == '/' ? '1' : '2';
              return Center(child: Text('Page $pageNumber'));
            },
          );
        },
      ),
    );

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);

    final TestGesture gesture = await tester.startGesture(const Offset(5, 200));
    // Move right a bit
    await gesture.moveBy(const Offset(300, 0));
    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didStartUserGesture);
    expect(tester.state<NavigatorState>(find.byType(Navigator)).userGestureInProgress, true);
    await tester.pump();

    // Move back to where we started.
    await gesture.moveBy(const Offset(-300, 0));
    await gesture.up();
    await tester.pump();

    expect(find.text('Page 1'), findsNothing);
    expect(find.text('Page 2'), isOnstage);
    expect(navigatorObserver.invocations.removeLast(), NavigatorInvocation.didStopUserGesture);
    expect(navigatorObserver.invocations.removeLast(), isNot(NavigatorInvocation.didPop));
    expect(tester.state<NavigatorState>(find.byType(Navigator)).userGestureInProgress, false);
  });

  group('Cupertino page transitions', () {
    CupertinoPageRoute<void> buildRoute({required bool fullscreenDialog}) {
      return CupertinoPageRoute<void>(
        fullscreenDialog: fullscreenDialog,
        builder: (_) => const SizedBox(),
      );
    }

    testWidgets('when route is not fullscreenDialog, it has a barrierColor', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(const MaterialApp(home: SizedBox.expand()));

      tester
          .state<NavigatorState>(find.byType(Navigator))
          .push(buildRoute(fullscreenDialog: false));
      await tester.pumpAndSettle();

      expect(
        tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color,
        const Color(0x18000000),
      );
    });

    testWidgets('when route is a fullscreenDialog, it has no barrierColor', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(const MaterialApp(home: SizedBox.expand()));

      tester.state<NavigatorState>(find.byType(Navigator)).push(buildRoute(fullscreenDialog: true));
      await tester.pumpAndSettle();

      expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, isNull);
    });

    testWidgets('when route is not fullscreenDialog, it has a _CupertinoEdgeShadowDecoration', (
      WidgetTester tester,
    ) async {
      PaintPattern paintsShadowRect({required double dx, required Color color}) {
        return paints..everything((Symbol methodName, List<dynamic> arguments) {
          if (methodName != #drawRect) {
            return true;
          }
          final Rect rect = arguments[0] as Rect;
          final Color paintColor = (arguments[1] as Paint).color;
          // _CupertinoEdgeShadowDecoration draws the shadows with a series of
          // differently colored 1px-wide rects. Skip rects that aren't being
          // drawn by the _CupertinoEdgeShadowDecoration.
          if (rect.top != 0 || rect.width != 1.0 || rect.height != 600) {
            return true;
          }
          // Skip calls for rects until the one with the given position offset
          if ((rect.left - dx).abs() >= 1) {
            return true;
          }
          if (paintColor.value == color.value) {
            return true;
          }
          throw '''
  For a rect with an expected left-side position: $dx (drawn at ${rect.left}):
              Expected a rect with color: $color,
              And drew a rect with color: $paintColor.
          ''';
        });
      }

      await tester.pumpWidget(const MaterialApp(home: SizedBox.expand()));

      tester
          .state<NavigatorState>(find.byType(Navigator))
          .push(buildRoute(fullscreenDialog: false));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1));

      final RenderBox box = tester.firstRenderObject<RenderBox>(find.byType(CustomPaint));

      // Animation starts with effectively no shadow
      expect(box, paintsShadowRect(dx: 795, color: CupertinoColors.transparent));
      expect(box, paintsShadowRect(dx: 785, color: CupertinoColors.transparent));
      expect(box, paintsShadowRect(dx: 775, color: CupertinoColors.transparent));
      expect(box, paintsShadowRect(dx: 765, color: CupertinoColors.transparent));
      expect(box, paintsShadowRect(dx: 755, color: CupertinoColors.transparent));

      await tester.pump(const Duration(milliseconds: 100));

      // Part-way through the transition, the shadow is approaching the full gradient
      expect(box, paintsShadowRect(dx: 296, color: const Color(0x03000000)));
      expect(box, paintsShadowRect(dx: 286, color: const Color(0x02000000)));
      expect(box, paintsShadowRect(dx: 276, color: const Color(0x01000000)));
      expect(box, paintsShadowRect(dx: 266, color: CupertinoColors.transparent));
      expect(box, paintsShadowRect(dx: 266, color: CupertinoColors.transparent));

      await tester.pumpAndSettle();

      // At the end of the transition, the shadow is a gradient between
      // 0x04000000 and 0x00000000 and is now offscreen
      expect(box, paintsShadowRect(dx: -1, color: const Color(0x04000000)));
      expect(box, paintsShadowRect(dx: -10, color: const Color(0x03000000)));
      expect(box, paintsShadowRect(dx: -20, color: const Color(0x02000000)));
      expect(box, paintsShadowRect(dx: -30, color: const Color(0x01000000)));
      expect(box, paintsShadowRect(dx: -40, color: CupertinoColors.transparent));

      // Start animation in reverse
      tester.state<NavigatorState>(find.byType(Navigator)).pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      expect(box, paintsShadowRect(dx: 498, color: const Color(0x04000000)));
      expect(box, paintsShadowRect(dx: 488, color: const Color(0x03000000)));
      expect(box, paintsShadowRect(dx: 478, color: const Color(0x02000000)));
      expect(box, paintsShadowRect(dx: 468, color: const Color(0x01000000)));
      expect(box, paintsShadowRect(dx: 458, color: CupertinoColors.transparent));

      await tester.pump(const Duration(milliseconds: 150));

      // At the end of the animation, the shadow approaches full transparency
      expect(box, paintsShadowRect(dx: 794, color: const Color(0x01000000)));
      expect(box, paintsShadowRect(dx: 784, color: CupertinoColors.transparent));
      expect(box, paintsShadowRect(dx: 774, color: CupertinoColors.transparent));
      expect(box, paintsShadowRect(dx: 764, color: CupertinoColors.transparent));
      expect(box, paintsShadowRect(dx: 754, color: CupertinoColors.transparent));
    });

    testWidgets(
      'when route is fullscreenDialog, it has no visible _CupertinoEdgeShadowDecoration',
      (WidgetTester tester) async {
        PaintPattern paintsNoShadows() {
          return paints..everything((Symbol methodName, List<dynamic> arguments) {
            if (methodName != #drawRect) {
              return true;
            }
            final Rect rect = arguments[0] as Rect;
            // _CupertinoEdgeShadowDecoration draws the shadows with a series of
            // differently colored 1px rects. Skip all rects not drawn by a
            // _CupertinoEdgeShadowDecoration.
            if (rect.width != 1.0) {
              return true;
            }
            throw '''
    Expected: no rects with a width of 1px.
          Found: $rect.
          ''';
          });
        }

        await tester.pumpWidget(const MaterialApp(home: SizedBox.expand()));

        final RenderBox box = tester.firstRenderObject<RenderBox>(find.byType(CustomPaint));

        tester
            .state<NavigatorState>(find.byType(Navigator))
            .push(buildRoute(fullscreenDialog: true));

        await tester.pumpAndSettle();
        expect(box, paintsNoShadows());

        tester.state<NavigatorState>(find.byType(Navigator)).pop();

        await tester.pumpAndSettle();
        expect(box, paintsNoShadows());
      },
    );
  });

  testWidgets('ModalPopup overlay dark mode', (WidgetTester tester) async {
    late StateSetter stateSetter;
    Brightness brightness = Brightness.light;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setter) {
          stateSetter = setter;
          return CupertinoApp(
            theme: CupertinoThemeData(brightness: brightness),
            home: CupertinoPageScaffold(
              child: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    onTap: () async {
                      await showCupertinoModalPopup<void>(
                        context: context,
                        builder: (BuildContext context) => const SizedBox(),
                      );
                    },
                    child: const Text('tap'),
                  );
                },
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();

    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color!.value, 0x33000000);

    stateSetter(() {
      brightness = Brightness.dark;
    });
    await tester.pump();

    // TODO(LongCatIsLooong): The background overlay SHOULD switch to dark color.
    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color!.value, 0x33000000);

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: CupertinoPageScaffold(
          child: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () async {
                  await showCupertinoModalPopup<void>(
                    context: context,
                    builder: (BuildContext context) => const SizedBox(),
                  );
                },
                child: const Text('tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();

    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color!.value, 0x7A000000);
  });

  testWidgets('During back swipe the route ignores input', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/39989

    final GlobalKey homeScaffoldKey = GlobalKey();
    final GlobalKey pageScaffoldKey = GlobalKey();
    int homeTapCount = 0;
    int pageTapCount = 0;

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: homeScaffoldKey,
          child: GestureDetector(
            onTap: () {
              homeTapCount += 1;
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(homeScaffoldKey));
    expect(homeTapCount, 1);
    expect(pageTapCount, 0);

    Navigator.push<void>(
      homeScaffoldKey.currentContext!,
      CupertinoPageRoute<void>(
        builder: (BuildContext context) {
          return CupertinoPageScaffold(
            key: pageScaffoldKey,
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: GestureDetector(
                onTap: () {
                  pageTapCount += 1;
                },
              ),
            ),
          );
        },
      ),
    );

    await tester.pumpAndSettle();
    await tester.tap(find.byKey(pageScaffoldKey));
    expect(homeTapCount, 1);
    expect(pageTapCount, 1);

    // Start the basic iOS back-swipe dismiss transition. Drag the pushed
    // "page" route halfway across the screen. The underlying "home" will
    // start sliding in from the left.

    final TestGesture gesture = await tester.startGesture(const Offset(5, 300));
    await gesture.moveBy(const Offset(400, 0));
    await tester.pump();
    expect(tester.getTopLeft(find.byKey(pageScaffoldKey)), const Offset(400, 0));
    expect(tester.getTopLeft(find.byKey(homeScaffoldKey)).dx, lessThan(0));

    // Tapping on the "page" route doesn't trigger the GestureDetector because
    // it's being dragged.
    await tester.tap(find.byKey(pageScaffoldKey), warnIfMissed: false);
    expect(homeTapCount, 1);
    expect(pageTapCount, 1);
  });

  testWidgets('showCupertinoModalPopup uses root navigator by default', (
    WidgetTester tester,
  ) async {
    final PopupObserver rootObserver = PopupObserver();
    final PopupObserver nestedObserver = PopupObserver();

    await tester.pumpWidget(
      CupertinoApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoModalPopup<void>(
                      context: context,
                      builder: (BuildContext context) => const SizedBox(),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.text('tap'));

    expect(rootObserver.popupCount, 1);
    expect(nestedObserver.popupCount, 0);
  });

  testWidgets('back swipe to screen edges does not dismiss the hero animation', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> navigator = GlobalKey<NavigatorState>();
    final UniqueKey container = UniqueKey();
    await tester.pumpWidget(
      CupertinoApp(
        navigatorKey: navigator,
        routes: <String, WidgetBuilder>{
          '/': (BuildContext context) {
            return CupertinoPageScaffold(
              child: Center(
                child: Hero(
                  tag: 'tag',
                  transitionOnUserGestures: true,
                  child: SizedBox(key: container, height: 150.0, width: 150.0),
                ),
              ),
            );
          },
          '/page2': (BuildContext context) {
            return CupertinoPageScaffold(
              child: Center(
                child: Padding(
                  padding: const EdgeInsets.fromLTRB(100.0, 0.0, 0.0, 0.0),
                  child: Hero(
                    tag: 'tag',
                    transitionOnUserGestures: true,
                    child: SizedBox(key: container, height: 150.0, width: 150.0),
                  ),
                ),
              ),
            );
          },
        },
      ),
    );

    RenderBox box = tester.renderObject(find.byKey(container)) as RenderBox;
    final double initialPosition = box.localToGlobal(Offset.zero).dx;

    navigator.currentState!.pushNamed('/page2');
    await tester.pumpAndSettle();
    box = tester.renderObject(find.byKey(container)) as RenderBox;
    final double finalPosition = box.localToGlobal(Offset.zero).dx;

    final TestGesture gesture = await tester.startGesture(const Offset(5, 300));
    await gesture.moveBy(const Offset(200, 0));
    await tester.pump();
    box = tester.renderObject(find.byKey(container)) as RenderBox;
    final double firstPosition = box.localToGlobal(Offset.zero).dx;
    // Checks the hero is in-transit.
    expect(finalPosition, greaterThan(firstPosition));
    expect(firstPosition, greaterThan(initialPosition));

    // Goes back to final position.
    await gesture.moveBy(const Offset(-200, 0));
    await tester.pump();
    box = tester.renderObject(find.byKey(container)) as RenderBox;
    final double secondPosition = box.localToGlobal(Offset.zero).dx;
    // There will be a small difference.
    expect(finalPosition - secondPosition, lessThan(0.001));

    await gesture.moveBy(const Offset(400, 0));
    await tester.pump();
    box = tester.renderObject(find.byKey(container)) as RenderBox;
    final double thirdPosition = box.localToGlobal(Offset.zero).dx;
    // Checks the hero is still in-transit and moves further away from the first
    // position.
    expect(finalPosition, greaterThan(thirdPosition));
    expect(thirdPosition, greaterThan(initialPosition));
    expect(firstPosition, greaterThan(thirdPosition));
  });

  testWidgets('showCupertinoModalPopup uses nested navigator if useRootNavigator is false', (
    WidgetTester tester,
  ) async {
    final PopupObserver rootObserver = PopupObserver();
    final PopupObserver nestedObserver = PopupObserver();

    await tester.pumpWidget(
      CupertinoApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoModalPopup<void>(
                      context: context,
                      useRootNavigator: false,
                      builder: (BuildContext context) => const SizedBox(),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.text('tap'));

    expect(rootObserver.popupCount, 0);
    expect(nestedObserver.popupCount, 1);
  });

  testWidgets('showCupertinoDialog uses root navigator by default', (WidgetTester tester) async {
    final DialogObserver rootObserver = DialogObserver();
    final DialogObserver nestedObserver = DialogObserver();

    await tester.pumpWidget(
      CupertinoApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoDialog<void>(
                      context: context,
                      builder: (BuildContext context) => const SizedBox(),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.text('tap'));

    expect(rootObserver.dialogCount, 1);
    expect(nestedObserver.dialogCount, 0);
  });

  testWidgets('showCupertinoDialog uses nested navigator if useRootNavigator is false', (
    WidgetTester tester,
  ) async {
    final DialogObserver rootObserver = DialogObserver();
    final DialogObserver nestedObserver = DialogObserver();

    await tester.pumpWidget(
      CupertinoApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoDialog<void>(
                      context: context,
                      useRootNavigator: false,
                      builder: (BuildContext context) => const SizedBox(),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.text('tap'));

    expect(rootObserver.dialogCount, 0);
    expect(nestedObserver.dialogCount, 1);
  });

  testWidgets('showCupertinoModalPopup does not allow for semantics dismiss by default', (
    WidgetTester tester,
  ) async {
    debugDefaultTargetPlatformOverride = TargetPlatform.iOS;
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      CupertinoApp(
        home: Navigator(
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoModalPopup<void>(
                      context: context,
                      builder: (BuildContext context) => const SizedBox(),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Push the route.
    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();

    expect(
      semantics,
      isNot(
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
          label: 'Dismiss',
        ),
      ),
    );
    debugDefaultTargetPlatformOverride = null;
    semantics.dispose();
  });

  testWidgets('showCupertinoModalPopup allows for semantics dismiss when set', (
    WidgetTester tester,
  ) async {
    debugDefaultTargetPlatformOverride = TargetPlatform.iOS;
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      CupertinoApp(
        home: Navigator(
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoModalPopup<void>(
                      context: context,
                      semanticsDismissible: true,
                      builder: (BuildContext context) => const SizedBox(),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Push the route.
    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();

    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
        label: 'Dismiss',
      ),
    );
    debugDefaultTargetPlatformOverride = null;
    semantics.dispose();
  });

  testWidgets('showCupertinoModalPopup passes RouteSettings to PopupRoute', (
    WidgetTester tester,
  ) async {
    final RouteSettingsObserver routeSettingsObserver = RouteSettingsObserver();

    await tester.pumpWidget(
      CupertinoApp(
        navigatorObservers: <NavigatorObserver>[routeSettingsObserver],
        home: Navigator(
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoModalPopup<void>(
                      context: context,
                      builder: (BuildContext context) => const SizedBox(),
                      routeSettings: const RouteSettings(name: '/modal'),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.text('tap'));

    expect(routeSettingsObserver.routeName, '/modal');
  });

  testWidgets('showCupertinoModalPopup transparent barrier color is transparent', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () async {
                  await showCupertinoModalPopup<void>(
                    context: context,
                    builder: (BuildContext context) => const SizedBox(),
                    barrierColor: CupertinoColors.transparent,
                  );
                },
                child: const Text('tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();

    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, null);
  });

  testWidgets('showCupertinoModalPopup null barrier color must be default gray barrier color', (
    WidgetTester tester,
  ) async {
    // Barrier color for a Cupertino modal barrier.
    // Extracted from https://developer.apple.com/design/resources/.
    const Color kModalBarrierColor = CupertinoDynamicColor.withBrightness(
      color: Color(0x33000000),
      darkColor: Color(0x7A000000),
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () async {
                  await showCupertinoModalPopup<void>(
                    context: context,
                    builder: (BuildContext context) => const SizedBox(),
                  );
                },
                child: const Text('tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();

    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, kModalBarrierColor);
  });

  testWidgets('showCupertinoModalPopup custom barrier color', (WidgetTester tester) async {
    const Color customColor = Color(0x11223344);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () async {
                  await showCupertinoModalPopup<void>(
                    context: context,
                    builder: (BuildContext context) => const SizedBox(),
                    barrierColor: customColor,
                  );
                },
                child: const Text('tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();

    expect(tester.widget<ModalBarrier>(find.byType(ModalBarrier).last).color, customColor);
  });

  testWidgets('showCupertinoModalPopup barrier dismissible', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () async {
                  await showCupertinoModalPopup<void>(
                    context: context,
                    builder: (BuildContext context) => const Text('Visible'),
                  );
                },
                child: const Text('tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();
    await tester.tapAt(
      tester.getTopLeft(
        find.ancestor(of: find.text('tap'), matching: find.byType(CupertinoPageScaffold)),
      ),
    );
    await tester.pumpAndSettle();

    expect(find.text('Visible'), findsNothing);
  });

  testWidgets('showCupertinoModalPopup barrier not dismissible', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          child: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                onTap: () async {
                  await showCupertinoModalPopup<void>(
                    context: context,
                    builder: (BuildContext context) => const Text('Visible'),
                    barrierDismissible: false,
                  );
                },
                child: const Text('tap'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();
    await tester.tapAt(
      tester.getTopLeft(
        find.ancestor(of: find.text('tap'), matching: find.byType(CupertinoPageScaffold)),
      ),
    );
    await tester.pumpAndSettle();

    expect(find.text('Visible'), findsOneWidget);
  });

  testWidgets('CupertinoPage works', (WidgetTester tester) async {
    final LocalKey pageKey = UniqueKey();
    final TransitionDetector detector = TransitionDetector();
    List<Page<void>> myPages = <Page<void>>[
      CupertinoPage<void>(
        key: pageKey,
        title: 'title one',
        child: CupertinoPageScaffold(
          navigationBar: CupertinoNavigationBar(key: UniqueKey()),
          child: const Text('first'),
        ),
      ),
    ];
    await tester.pumpWidget(
      buildNavigator(
        view: tester.view,
        pages: myPages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          assert(false); // The test shouldn't call this.
          return true;
        },
        transitionDelegate: detector,
      ),
    );

    expect(detector.hasTransition, isFalse);
    expect(find.widgetWithText(CupertinoNavigationBar, 'title one'), findsOneWidget);
    expect(find.text('first'), findsOneWidget);

    myPages = <Page<void>>[
      CupertinoPage<void>(
        key: pageKey,
        title: 'title two',
        child: CupertinoPageScaffold(
          navigationBar: CupertinoNavigationBar(key: UniqueKey()),
          child: const Text('second'),
        ),
      ),
    ];

    await tester.pumpWidget(
      buildNavigator(
        view: tester.view,
        pages: myPages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          assert(false); // The test shouldn't call this.
          return true;
        },
        transitionDelegate: detector,
      ),
    );

    // There should be no transition because the page has the same key.
    expect(detector.hasTransition, isFalse);
    // The content does update.
    expect(find.text('first'), findsNothing);
    expect(find.widgetWithText(CupertinoNavigationBar, 'title one'), findsNothing);
    expect(find.text('second'), findsOneWidget);
    expect(find.widgetWithText(CupertinoNavigationBar, 'title two'), findsOneWidget);
  });

  testWidgets('CupertinoPage can toggle MaintainState', (WidgetTester tester) async {
    final LocalKey pageKeyOne = UniqueKey();
    final LocalKey pageKeyTwo = UniqueKey();
    final TransitionDetector detector = TransitionDetector();
    List<Page<void>> myPages = <Page<void>>[
      CupertinoPage<void>(key: pageKeyOne, maintainState: false, child: const Text('first')),
      CupertinoPage<void>(key: pageKeyTwo, child: const Text('second')),
    ];
    await tester.pumpWidget(
      buildNavigator(
        view: tester.view,
        pages: myPages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          assert(false); // The test shouldn't call this.
          return true;
        },
        transitionDelegate: detector,
      ),
    );

    expect(detector.hasTransition, isFalse);
    // Page one does not maintain state.
    expect(find.text('first', skipOffstage: false), findsNothing);
    expect(find.text('second'), findsOneWidget);

    myPages = <Page<void>>[
      CupertinoPage<void>(key: pageKeyOne, child: const Text('first')),
      CupertinoPage<void>(key: pageKeyTwo, child: const Text('second')),
    ];

    await tester.pumpWidget(
      buildNavigator(
        view: tester.view,
        pages: myPages,
        onPopPage: (Route<dynamic> route, dynamic result) {
          assert(false); // The test shouldn't call this.
          return true;
        },
        transitionDelegate: detector,
      ),
    );
    // There should be no transition because the page has the same key.
    expect(detector.hasTransition, isFalse);
    // Page one sets the maintain state to be true, its widget tree should be
    // built.
    expect(find.text('first', skipOffstage: false), findsOneWidget);
    expect(find.text('second'), findsOneWidget);
  });

  testWidgets('Popping routes should cancel down events', (WidgetTester tester) async {
    await tester.pumpWidget(const _TestPostRouteCancel());

    final TestGesture gesture = await tester.createGesture();
    await gesture.down(tester.getCenter(find.text('PointerCancelEvents: 0')));
    await gesture.up();

    await tester.pumpAndSettle();
    expect(find.byType(CupertinoButton), findsNothing);
    expect(find.text('Hold'), findsOneWidget);

    await gesture.down(tester.getCenter(find.text('Hold')));
    await tester.pump(const Duration(seconds: 2));
    await tester.pumpAndSettle();
    expect(find.text('Hold'), findsNothing);
    expect(find.byType(CupertinoButton), findsOneWidget);
    expect(find.text('PointerCancelEvents: 1'), findsOneWidget);
  });

  testWidgets('Popping routes during back swipe should not crash', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/63984#issuecomment-675679939

    final CupertinoPageRoute<void> r = CupertinoPageRoute<void>(
      builder: (BuildContext context) {
        return const Scaffold(body: Center(child: Text('child')));
      },
    );

    late NavigatorState navigator;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: Builder(
            builder: (BuildContext context) {
              return ElevatedButton(
                child: const Text('Home'),
                onPressed: () {
                  navigator = Navigator.of(context);
                  navigator.push<void>(r);
                },
              );
            },
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture();
    await gesture.down(tester.getCenter(find.byType(ElevatedButton)));
    await gesture.up();

    await tester.pumpAndSettle();

    await gesture.down(const Offset(3, 300));

    // Need 2 events to form a valid drag
    await tester.pump(const Duration(milliseconds: 100));
    await gesture.moveTo(const Offset(30, 300), timeStamp: const Duration(milliseconds: 100));
    await tester.pump(const Duration(milliseconds: 200));
    await gesture.moveTo(const Offset(50, 300), timeStamp: const Duration(milliseconds: 200));

    // Pause a while so that the route is popped when the drag is canceled
    await tester.pump(const Duration(milliseconds: 1000));
    await gesture.moveTo(const Offset(51, 300), timeStamp: const Duration(milliseconds: 1200));

    // Remove the drag
    navigator.removeRoute(r);
    await tester.pump();
  });

  testWidgets('CupertinoModalPopupRoute is state restorable', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(restorationScopeId: 'app', home: _RestorableModalTestWidget()),
    );

    expect(find.byType(CupertinoActionSheet), findsNothing);

    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    expect(find.byType(CupertinoActionSheet), findsOneWidget);
    final TestRestorationData restorationData = await tester.getRestorationData();

    await tester.restartAndRestore();

    expect(find.byType(CupertinoActionSheet), findsOneWidget);

    // Tap on the barrier.
    await tester.tapAt(const Offset(10.0, 10.0));
    await tester.pumpAndSettle();

    expect(find.byType(CupertinoActionSheet), findsNothing);

    await tester.restoreFrom(restorationData);
    expect(find.byType(CupertinoActionSheet), findsOneWidget);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  group('showCupertinoDialog avoids overlapping display features', () {
    testWidgets('positioning with anchorPoint', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );
      final BuildContext context = tester.element(find.text('Test'));

      showCupertinoDialog<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
        anchorPoint: const Offset(1000, 0),
      );
      await tester.pumpAndSettle();

      // Should take the right side of the screen
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(410.0, 0.0));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(800.0, 600.0));
    });

    testWidgets('positioning with Directionality', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: Directionality(textDirection: TextDirection.rtl, child: child!),
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );
      final BuildContext context = tester.element(find.text('Test'));

      showCupertinoDialog<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
      );
      await tester.pumpAndSettle();

      // Since this is RTL, it should place the dialog on the right screen
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(410.0, 0.0));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(800.0, 600.0));
    });

    testWidgets('positioning by default', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );
      final BuildContext context = tester.element(find.text('Test'));

      showCupertinoDialog<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
      );
      await tester.pumpAndSettle();

      // By default it should place the dialog on the left screen
      expect(tester.getTopLeft(find.byType(Placeholder)), Offset.zero);
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(390.0, 600.0));
    });
  });

  group('showCupertinoModalPopup avoids overlapping display features', () {
    testWidgets('positioning using anchorPoint', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showCupertinoModalPopup<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
        anchorPoint: const Offset(1000, 0),
      );
      await tester.pumpAndSettle();

      // Should take the right side of the screen
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, 410);
      expect(tester.getBottomRight(find.byType(Placeholder)).dx, 800);
    });

    testWidgets('positioning using Directionality', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: Directionality(textDirection: TextDirection.rtl, child: child!),
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showCupertinoModalPopup<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
      );
      await tester.pumpAndSettle();

      // This is RTL, so it should place the dialog on the right screen
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, 410);
      expect(tester.getBottomRight(find.byType(Placeholder)).dx, 800);
    });

    testWidgets('default positioning', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showCupertinoModalPopup<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
      );
      await tester.pumpAndSettle();

      // By default it should place the dialog on the left screen
      expect(tester.getTopLeft(find.byType(Placeholder)).dx, 0.0);
      expect(tester.getBottomRight(find.byType(Placeholder)).dx, 390.0);
    });
  });

  testWidgets('Fullscreen route does not leak CurveAnimation', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            return CupertinoButton(
              child: const Text('Button'),
              onPressed: () {
                Navigator.push<void>(
                  context,
                  CupertinoPageRoute<void>(
                    fullscreenDialog: true,
                    builder: (BuildContext context) {
                      return Column(
                        children: <Widget>[
                          const Placeholder(),
                          CupertinoButton(
                            child: const Text('Close'),
                            onPressed: () {
                              Navigator.pop<void>(context);
                            },
                          ),
                        ],
                      );
                    },
                  ),
                );
              },
            );
          },
        ),
      ),
    );

    // Enter animation.
    await tester.tap(find.text('Button'));
    await tester.pump();

    await tester.pump(const Duration(milliseconds: 400));

    // Exit animation
    await tester.tap(find.text('Close'));
    await tester.pump();

    await tester.pump(const Duration(milliseconds: 400));
  });

  testWidgets('CupertinoModalPopupRoute does not leak CurveAnimation', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Navigator(
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoModalPopup<void>(
                      context: context,
                      semanticsDismissible: true,
                      builder: (BuildContext context) => const SizedBox(),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Push the route.
    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();
  });

  testWidgets('CupertinoDialogRoute does not leak CurveAnimation', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Navigator(
          onGenerateRoute: (RouteSettings settings) {
            return PageRouteBuilder<dynamic>(
              pageBuilder: (BuildContext context, Animation<double> _, Animation<double> __) {
                return GestureDetector(
                  onTap: () async {
                    await showCupertinoDialog<void>(
                      context: context,
                      useRootNavigator: false,
                      builder: (BuildContext context) => const SizedBox(),
                    );
                  },
                  child: const Text('tap'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.text('tap'));
    await tester.pumpAndSettle();
  });

  testWidgets('fullscreen routes do not transition previous route', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        initialRoute: '/',
        onGenerateRoute: (RouteSettings settings) {
          if (settings.name == '/') {
            return PageRouteBuilder<void>(
              pageBuilder: (
                BuildContext context,
                Animation<double> animation,
                Animation<double> secondaryAnimation,
              ) {
                return CupertinoPageScaffold(
                  navigationBar: const CupertinoNavigationBar(middle: Text('Page 1')),
                  child: Container(),
                );
              },
            );
          }
          return CupertinoPageRoute<void>(
            builder: (BuildContext context) {
              return CupertinoPageScaffold(
                navigationBar: const CupertinoNavigationBar(middle: Text('Page 2')),
                child: Container(),
              );
            },
            fullscreenDialog: true,
          );
        },
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2'), findsNothing);

    final double pageTitleDX = tester.getTopLeft(find.text('Page 1')).dx;

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    // Second page transition has started.
    expect(find.text('Page 2'), findsOneWidget);

    // First page has not moved.
    expect(tester.getTopLeft(find.text('Page 1')).dx, equals(pageTitleDX));
  });

  testWidgets(
    'Setting CupertinoDialogRoute.requestFocus to false does not request focus on the dialog',
    (WidgetTester tester) async {
      late BuildContext savedContext;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      const String dialogText = 'Dialog Text';
      await tester.pumpWidget(
        CupertinoApp(
          home: Builder(
            builder: (BuildContext context) {
              savedContext = context;
              return CupertinoTextField(focusNode: focusNode);
            },
          ),
        ),
      );
      await tester.pump();

      FocusNode? getCupertinoTextFieldFocusNode() {
        return tester
            .widget<Focus>(
              find.descendant(of: find.byType(CupertinoTextField), matching: find.byType(Focus)),
            )
            .focusNode;
      }

      // Initially, there is no dialog and the text field has no focus.
      expect(find.text(dialogText), findsNothing);
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, false);

      // Request focus on the text field.
      focusNode.requestFocus();
      await tester.pump();
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, true);

      // Bring up dialog.
      final NavigatorState navigator = Navigator.of(savedContext);
      navigator.push(
        CupertinoDialogRoute<void>(
          context: savedContext,
          builder: (BuildContext context) => const Text(dialogText),
        ),
      );
      await tester.pump();

      // The dialog is showing and the text field has lost focus.
      expect(find.text(dialogText), findsOneWidget);
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, false);

      // Dismiss the dialog.
      navigator.pop();
      await tester.pump();

      // The dialog is dismissed and the focus is shifted back to the text field.
      expect(find.text(dialogText), findsNothing);
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, true);

      // Bring up dialog again with requestFocus to false.
      navigator.push(
        CupertinoDialogRoute<void>(
          context: savedContext,
          requestFocus: false,
          builder: (BuildContext context) => const Text(dialogText),
        ),
      );
      await tester.pump();

      // The dialog is showing and the text field still has focus.
      expect(find.text(dialogText), findsOneWidget);
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, true);
    },
  );

  testWidgets(
    'Setting CupertinoModalPopupRoute.requestFocus to false does not request focus on the popup',
    (WidgetTester tester) async {
      late BuildContext savedContext;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      const String dialogText = 'Popup Text';
      await tester.pumpWidget(
        CupertinoApp(
          home: Builder(
            builder: (BuildContext context) {
              savedContext = context;
              return CupertinoTextField(focusNode: focusNode);
            },
          ),
        ),
      );
      await tester.pump();

      FocusNode? getCupertinoTextFieldFocusNode() {
        return tester
            .widget<Focus>(
              find.descendant(of: find.byType(CupertinoTextField), matching: find.byType(Focus)),
            )
            .focusNode;
      }

      // Initially, there is no popup and the text field has no focus.
      expect(find.text(dialogText), findsNothing);
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, false);

      // Request focus on the text field.
      focusNode.requestFocus();
      await tester.pump();
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, true);

      // Bring up popup.
      final NavigatorState navigator = Navigator.of(savedContext);
      navigator.push(
        CupertinoModalPopupRoute<void>(builder: (BuildContext context) => const Text(dialogText)),
      );
      await tester.pump();

      // The popup is showing and the text field has lost focus.
      expect(find.text(dialogText), findsOneWidget);
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, false);

      // Dismiss the popup.
      navigator.pop();
      await tester.pump();

      // The popup is dismissed and the focus is shifted back to the text field.
      expect(find.text(dialogText), findsNothing);
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, true);

      // Bring up popup again with requestFocus to false.
      navigator.push(
        CupertinoModalPopupRoute<void>(
          requestFocus: false,
          builder: (BuildContext context) => const Text(dialogText),
        ),
      );
      await tester.pump();

      // The popup is showing and the text field still has focus.
      expect(find.text(dialogText), findsOneWidget);
      expect(getCupertinoTextFieldFocusNode()?.hasFocus, true);
    },
  );

  testWidgets(
    'Setting CupertinoPageRoute.requestFocus to false does not request focus on the page',
    (WidgetTester tester) async {
      late BuildContext savedContext;
      const String pageTwoText = 'Page Two';
      await tester.pumpWidget(
        CupertinoApp(
          home: Builder(
            builder: (BuildContext context) {
              savedContext = context;
              return Container();
            },
          ),
        ),
      );
      await tester.pump();

      // Check page two is not on the screen.
      expect(find.text(pageTwoText), findsNothing);

      // Navigate to page two with text.
      final NavigatorState navigator = Navigator.of(savedContext);
      navigator.push(
        CupertinoPageRoute<void>(
          builder: (BuildContext context) {
            return const Text(pageTwoText);
          },
        ),
      );
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100)); // Advance route transition animation.

      // The page two is showing and the text widget has focus.
      Element textOnPageTwo = tester.element(find.text(pageTwoText));
      FocusScopeNode focusScopeNode = FocusScope.of(textOnPageTwo);
      expect(focusScopeNode.hasFocus, isTrue);

      // Navigate back to page one.
      navigator.pop();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100)); // Advance route transition animation.

      // Navigate to page two again with requestFocus set to false.
      navigator.push(
        CupertinoPageRoute<void>(
          requestFocus: false,
          builder: (BuildContext context) {
            return const Text(pageTwoText);
          },
        ),
      );
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100)); // Advance route transition animation.

      // The page two is showing and the text widget is not focused.
      textOnPageTwo = tester.element(find.text(pageTwoText));
      focusScopeNode = FocusScope.of(textOnPageTwo);
      expect(focusScopeNode.hasFocus, isFalse);
    },
  );
}

class MockNavigatorObserver extends NavigatorObserver {
  final List<NavigatorInvocation> invocations = <NavigatorInvocation>[];

  @override
  void didStartUserGesture(Route<dynamic> route, Route<dynamic>? previousRoute) {
    invocations.add(NavigatorInvocation.didStartUserGesture);
  }

  @override
  void didPop(Route<dynamic> route, Route<dynamic>? previousRoute) {
    invocations.add(NavigatorInvocation.didPop);
  }

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    invocations.add(NavigatorInvocation.didPush);
  }

  @override
  void didStopUserGesture() {
    invocations.add(NavigatorInvocation.didStopUserGesture);
  }
}

enum NavigatorInvocation { didStartUserGesture, didPop, didPush, didStopUserGesture }

class PopupObserver extends NavigatorObserver {
  int popupCount = 0;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is CupertinoModalPopupRoute) {
      popupCount++;
    }
    super.didPush(route, previousRoute);
  }
}

class DialogObserver extends NavigatorObserver {
  int dialogCount = 0;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is CupertinoDialogRoute) {
      dialogCount++;
    }
    super.didPush(route, previousRoute);
  }
}

class RouteSettingsObserver extends NavigatorObserver {
  String? routeName;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is CupertinoModalPopupRoute) {
      routeName = route.settings.name;
    }
    super.didPush(route, previousRoute);
  }
}

class TransitionDetector extends DefaultTransitionDelegate<void> {
  bool hasTransition = false;
  @override
  Iterable<RouteTransitionRecord> resolve({
    required List<RouteTransitionRecord> newPageRouteHistory,
    required Map<RouteTransitionRecord?, RouteTransitionRecord> locationToExitingPageRoute,
    required Map<RouteTransitionRecord?, List<RouteTransitionRecord>> pageRouteToPagelessRoutes,
  }) {
    hasTransition = true;
    return super.resolve(
      newPageRouteHistory: newPageRouteHistory,
      locationToExitingPageRoute: locationToExitingPageRoute,
      pageRouteToPagelessRoutes: pageRouteToPagelessRoutes,
    );
  }
}

Widget buildNavigator({
  required List<Page<dynamic>> pages,
  required FlutterView view,
  PopPageCallback? onPopPage,
  GlobalKey<NavigatorState>? key,
  TransitionDelegate<dynamic>? transitionDelegate,
}) {
  return MediaQuery(
    data: MediaQueryData.fromView(view),
    child: Localizations(
      locale: const Locale('en', 'US'),
      delegates: const <LocalizationsDelegate<dynamic>>[
        DefaultCupertinoLocalizations.delegate,
        DefaultWidgetsLocalizations.delegate,
      ],
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: Navigator(
          key: key,
          pages: pages,
          onPopPage: onPopPage,
          transitionDelegate: transitionDelegate ?? const DefaultTransitionDelegate<dynamic>(),
        ),
      ),
    ),
  );
}

// A test target to updating pages in navigator.
//
// It contains 3 routes:
//
//  * The initial route, 'home'.
//  * The 'old' route, displays a button showing 'Update pages'. Tap the button
//    will update pages.
//  * The 'new' route, displays the new page.
class _TestPageUpdate extends StatefulWidget {
  const _TestPageUpdate();

  @override
  State<StatefulWidget> createState() => _TestPageUpdateState();
}

class _TestPageUpdateState extends State<_TestPageUpdate> {
  bool updatePages = false;

  @override
  Widget build(BuildContext context) {
    final GlobalKey<State<StatefulWidget>> navKey = GlobalKey();
    return MaterialApp(
      home: Navigator(
        key: navKey,
        pages:
            updatePages
                ? <Page<dynamic>>[
                  const CupertinoPage<dynamic>(name: '/home', child: Text('home')),
                  const CupertinoPage<dynamic>(name: '/home/new', child: Text('New page')),
                ]
                : <Page<dynamic>>[
                  const CupertinoPage<dynamic>(name: '/home', child: Text('home')),
                  CupertinoPage<dynamic>(name: '/home/old', child: buildMainPage()),
                ],
        onPopPage: (_, __) {
          return false;
        },
      ),
    );
  }

  Widget buildMainPage() {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Main'),
            ElevatedButton(
              onPressed: () {
                Future<void>.delayed(const Duration(seconds: 2), () {
                  setState(() {
                    updatePages = true;
                  });
                });
              },
              child: const Text('Update Pages'),
            ),
          ],
        ),
      ),
    );
  }
}

// A test target for post-route cancel events.
//
// It contains 2 routes:
//
//  * The initial route, 'home', displays a button showing 'PointerCancelEvents: #',
//    where # is the number of cancel events received. Tapping the button pushes
//    route 'sub'.
//  * The 'sub' route, displays a text showing 'Hold'. Holding the button (a down
//    event) will pop this route after 1 second.
//
// Holding the 'Hold' button at the moment of popping will force the navigator to
// cancel the down event, increasing the Home counter by 1.
class _TestPostRouteCancel extends StatefulWidget {
  const _TestPostRouteCancel();

  @override
  State<StatefulWidget> createState() => _TestPostRouteCancelState();
}

class _TestPostRouteCancelState extends State<_TestPostRouteCancel> {
  int counter = 0;

  Widget _buildHome(BuildContext context) {
    return Center(
      child: CupertinoButton(
        child: Text('PointerCancelEvents: $counter'),
        onPressed: () => Navigator.pushNamed<void>(context, 'sub'),
      ),
    );
  }

  Widget _buildSub(BuildContext context) {
    return Listener(
      onPointerDown: (_) {
        Future<void>.delayed(const Duration(seconds: 1)).then((_) {
          Navigator.pop(context);
        });
      },
      onPointerCancel: (_) {
        setState(() {
          counter += 1;
        });
      },
      child: const Center(child: Text('Hold', style: TextStyle(color: Colors.blue))),
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoApp(
      initialRoute: 'home',
      onGenerateRoute:
          (RouteSettings settings) => CupertinoPageRoute<void>(
            settings: settings,
            builder:
                (BuildContext context) => switch (settings.name) {
                  'home' => _buildHome(context),
                  'sub' => _buildSub(context),
                  _ => throw UnimplementedError(),
                },
          ),
    );
  }
}

@pragma('vm:entry-point')
class _RestorableModalTestWidget extends StatelessWidget {
  const _RestorableModalTestWidget();

  @pragma('vm:entry-point')
  static Route<void> _modalBuilder(BuildContext context, Object? arguments) {
    return CupertinoModalPopupRoute<void>(
      builder: (BuildContext context) {
        return CupertinoActionSheet(
          title: const Text('Title'),
          message: const Text('Message'),
          actions: <Widget>[
            CupertinoActionSheetAction(
              child: const Text('Action One'),
              onPressed: () {
                Navigator.pop(context);
              },
            ),
            CupertinoActionSheetAction(
              child: const Text('Action Two'),
              onPressed: () {
                Navigator.pop(context);
              },
            ),
          ],
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('Home')),
      child: Center(
        child: CupertinoButton(
          onPressed: () {
            Navigator.of(context).restorablePush(_modalBuilder);
          },
          child: const Text('X'),
        ),
      ),
    );
  }
}
