// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../rendering/rendering_tester.dart' show TestClipPaintingContext;
import 'semantics_tester.dart';
import 'states.dart';

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/100451
  testWidgets('PageView.builder respects findChildIndexCallback', (WidgetTester tester) async {
    bool finderCalled = false;
    int itemCount = 7;
    late StateSetter stateSetter;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            stateSetter = setState;
            return PageView.builder(
              itemCount: itemCount,
              itemBuilder:
                  (BuildContext _, int index) => Container(key: Key('$index'), height: 2000.0),
              findChildIndexCallback: (Key key) {
                finderCalled = true;
                return null;
              },
            );
          },
        ),
      ),
    );
    expect(finderCalled, false);

    // Trigger update.
    stateSetter(() => itemCount = 77);
    await tester.pump();

    expect(finderCalled, true);
  });

  testWidgets('PageView resize from zero-size viewport should not lose state', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/88956
    final PageController controller = PageController(initialPage: 1);
    addTearDown(controller.dispose);

    Widget build(Size size) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox.fromSize(
            size: size,
            child: PageView(
              controller: controller,
              onPageChanged: (int page) {},
              children: kStates.map<Widget>((String state) => Text(state)).toList(),
            ),
          ),
        ),
      );
    }

    // The pageView have a zero viewport, so nothing display.
    await tester.pumpWidget(build(Size.zero));
    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Alabama', skipOffstage: false), findsOneWidget);

    // Resize from zero viewport to non-zero, the controller's initialPage 1 will display.
    await tester.pumpWidget(build(const Size(200.0, 200.0)));
    expect(find.text('Alaska'), findsOneWidget);

    // Jump to page 'Iowa'.
    controller.jumpToPage(kStates.indexOf('Iowa'));
    await tester.pump();
    expect(find.text('Iowa'), findsOneWidget);

    // Resize to zero viewport again, nothing display.
    await tester.pumpWidget(build(Size.zero));
    expect(find.text('Iowa'), findsNothing);

    // Resize from zero to non-zero, the pageView should not lose state, so the page 'Iowa' show again.
    await tester.pumpWidget(build(const Size(200.0, 200.0)));
    expect(find.text('Iowa'), findsOneWidget);
  });

  testWidgets('Change the page through the controller when zero-size viewport', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/88956
    final PageController controller = PageController(initialPage: 1);
    addTearDown(controller.dispose);

    Widget build(Size size) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox.fromSize(
            size: size,
            child: PageView(
              controller: controller,
              onPageChanged: (int page) {},
              children: kStates.map<Widget>((String state) => Text(state)).toList(),
            ),
          ),
        ),
      );
    }

    // The pageView have a zero viewport, so nothing display.
    await tester.pumpWidget(build(Size.zero));
    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Alabama', skipOffstage: false), findsOneWidget);

    // Change the page through the page controller when zero viewport
    controller.animateToPage(
      kStates.indexOf('Iowa'),
      duration: kTabScrollDuration,
      curve: Curves.ease,
    );
    expect(controller.page, kStates.indexOf('Iowa'));

    controller.jumpToPage(kStates.indexOf('Illinois'));
    expect(controller.page, kStates.indexOf('Illinois'));

    // Resize from zero viewport to non-zero, the latest state should not lost.
    await tester.pumpWidget(build(const Size(200.0, 200.0)));
    expect(controller.page, kStates.indexOf('Illinois'));
    expect(find.text('Illinois'), findsOneWidget);
  });

  testWidgets('_PagePosition.applyViewportDimension should not throw', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/101007
    final PageController controller = PageController(initialPage: 1);
    addTearDown(controller.dispose);

    // Set the starting viewportDimension to 0.0
    await tester.binding.setSurfaceSize(Size.zero);
    final MediaQueryData mediaQueryData = MediaQueryData.fromView(tester.view);

    Widget build(Size size) {
      return MediaQuery(
        data: mediaQueryData.copyWith(size: size),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox.expand(
              child: PageView(
                controller: controller,
                onPageChanged: (int page) {},
                children: kStates.map<Widget>((String state) => Text(state)).toList(),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build(Size.zero));
    const Size surfaceSize = Size(500, 400);
    await tester.binding.setSurfaceSize(surfaceSize);
    await tester.pumpWidget(build(surfaceSize));

    expect(tester.takeException(), isNull);

    // Reset TestWidgetsFlutterBinding surfaceSize
    await tester.binding.setSurfaceSize(null);
  });

  testWidgets('PageController cannot return page while unattached', (WidgetTester tester) async {
    final PageController controller = PageController();
    addTearDown(controller.dispose);
    expect(() => controller.page, throwsAssertionError);
  });

  testWidgets('PageView control test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          dragStartBehavior: DragStartBehavior.down,
          children:
              kStates.map<Widget>((String state) {
                return GestureDetector(
                  dragStartBehavior: DragStartBehavior.down,
                  onTap: () {
                    log.add(state);
                  },
                  child: Container(
                    height: 200.0,
                    color: const Color(0xFF0000FF),
                    child: Text(state),
                  ),
                );
              }).toList(),
        ),
      ),
    );

    await tester.tap(find.text('Alabama'));
    expect(log, equals(<String>['Alabama']));
    log.clear();

    expect(find.text('Alaska'), findsNothing);

    await tester.drag(find.byType(PageView), const Offset(-20.0, 0.0));
    await tester.pump();

    expect(find.text('Alabama'), findsOneWidget);
    expect(find.text('Alaska'), findsOneWidget);
    expect(find.text('Arizona'), findsNothing);

    await tester.pumpAndSettle();

    expect(find.text('Alabama'), findsOneWidget);
    expect(find.text('Alaska'), findsNothing);

    await tester.drag(find.byType(PageView), const Offset(-401.0, 0.0));
    await tester.pumpAndSettle();

    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Alaska'), findsOneWidget);
    expect(find.text('Arizona'), findsNothing);

    await tester.tap(find.text('Alaska'));
    expect(log, equals(<String>['Alaska']));
    log.clear();

    await tester.fling(find.byType(PageView), const Offset(-200.0, 0.0), 1000.0);
    await tester.pumpAndSettle();

    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Alaska'), findsNothing);
    expect(find.text('Arizona'), findsOneWidget);

    await tester.fling(find.byType(PageView), const Offset(200.0, 0.0), 1000.0);
    await tester.pumpAndSettle();

    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Alaska'), findsOneWidget);
    expect(find.text('Arizona'), findsNothing);
  });

  testWidgets(
    'PageView does not squish when overscrolled',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: PageView(
            children: List<Widget>.generate(10, (int i) {
              return Container(key: ValueKey<int>(i), color: const Color(0xFF0000FF));
            }),
          ),
        ),
      );

      Size sizeOf(int i) => tester.getSize(find.byKey(ValueKey<int>(i)));
      double leftOf(int i) => tester.getTopLeft(find.byKey(ValueKey<int>(i))).dx;

      expect(leftOf(0), equals(0.0));
      expect(sizeOf(0), equals(const Size(800.0, 600.0)));

      // Going into overscroll.
      await tester.drag(find.byType(PageView), const Offset(100.0, 0.0));
      await tester.pump();

      expect(leftOf(0), greaterThan(0.0));
      expect(sizeOf(0), equals(const Size(800.0, 600.0)));

      // Easing overscroll past overscroll limit.
      if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
        await tester.drag(find.byType(PageView), const Offset(-500.0, 0.0));
      } else {
        await tester.drag(find.byType(PageView), const Offset(-200.0, 0.0));
      }
      await tester.pump();

      expect(leftOf(0), lessThan(0.0));
      expect(sizeOf(0), equals(const Size(800.0, 600.0)));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('PageController control test', (WidgetTester tester) async {
    final PageController controller = PageController(initialPage: 4);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 600.0,
            height: 400.0,
            child: PageView(
              controller: controller,
              children: kStates.map<Widget>((String state) => Text(state)).toList(),
            ),
          ),
        ),
      ),
    );

    expect(find.text('California'), findsOneWidget);

    controller.nextPage(duration: const Duration(milliseconds: 150), curve: Curves.ease);
    await tester.pumpAndSettle();

    expect(find.text('Colorado'), findsOneWidget);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 300.0,
            height: 400.0,
            child: PageView(
              controller: controller,
              children: kStates.map<Widget>((String state) => Text(state)).toList(),
            ),
          ),
        ),
      ),
    );

    expect(find.text('Colorado'), findsOneWidget);

    controller.previousPage(duration: const Duration(milliseconds: 150), curve: Curves.ease);
    await tester.pumpAndSettle();

    expect(find.text('California'), findsOneWidget);
  });

  testWidgets('PageController page stability', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 600.0,
            height: 400.0,
            child: PageView(children: kStates.map<Widget>((String state) => Text(state)).toList()),
          ),
        ),
      ),
    );

    expect(find.text('Alabama'), findsOneWidget);

    await tester.drag(find.byType(PageView), const Offset(-1250.0, 0.0));
    await tester.pumpAndSettle();

    expect(find.text('Arizona'), findsOneWidget);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 250.0,
            height: 100.0,
            child: PageView(children: kStates.map<Widget>((String state) => Text(state)).toList()),
          ),
        ),
      ),
    );

    expect(find.text('Arizona'), findsOneWidget);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 450.0,
            height: 400.0,
            child: PageView(children: kStates.map<Widget>((String state) => Text(state)).toList()),
          ),
        ),
      ),
    );

    expect(find.text('Arizona'), findsOneWidget);
  });

  testWidgets('PageController nextPage and previousPage return Futures that resolve', (
    WidgetTester tester,
  ) async {
    final PageController controller = PageController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          controller: controller,
          children: kStates.map<Widget>((String state) => Text(state)).toList(),
        ),
      ),
    );

    bool nextPageCompleted = false;
    controller
        .nextPage(duration: const Duration(milliseconds: 150), curve: Curves.ease)
        .then((_) => nextPageCompleted = true);

    expect(nextPageCompleted, false);
    await tester.pump(const Duration(milliseconds: 200));
    expect(nextPageCompleted, false);
    await tester.pump(const Duration(milliseconds: 200));
    expect(nextPageCompleted, true);

    bool previousPageCompleted = false;
    controller
        .previousPage(duration: const Duration(milliseconds: 150), curve: Curves.ease)
        .then((_) => previousPageCompleted = true);

    expect(previousPageCompleted, false);
    await tester.pump(const Duration(milliseconds: 200));
    expect(previousPageCompleted, false);
    await tester.pump(const Duration(milliseconds: 200));
    expect(previousPageCompleted, true);
  });

  testWidgets('PageView in zero-size container', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox.shrink(
            child: PageView(children: kStates.map<Widget>((String state) => Text(state)).toList()),
          ),
        ),
      ),
    );

    expect(find.text('Alabama', skipOffstage: false), findsOneWidget);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 200.0,
            height: 200.0,
            child: PageView(children: kStates.map<Widget>((String state) => Text(state)).toList()),
          ),
        ),
      ),
    );

    expect(find.text('Alabama'), findsOneWidget);
  });

  testWidgets('Page changes at halfway point', (WidgetTester tester) async {
    final List<int> log = <int>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          onPageChanged: log.add,
          children: kStates.map<Widget>((String state) => Text(state)).toList(),
        ),
      ),
    );

    expect(log, isEmpty);

    final TestGesture gesture = await tester.startGesture(const Offset(100.0, 100.0));
    // The page view is 800.0 wide, so this move is just short of halfway.
    await gesture.moveBy(const Offset(-380.0, 0.0));

    expect(log, isEmpty);

    // We've crossed the halfway mark.
    await gesture.moveBy(const Offset(-40.0, 0.0));

    expect(log, equals(const <int>[1]));
    log.clear();

    // Moving a bit more should not generate redundant notifications.
    await gesture.moveBy(const Offset(-40.0, 0.0));

    expect(log, isEmpty);

    await gesture.moveBy(const Offset(-40.0, 0.0));
    await tester.pump();

    await gesture.moveBy(const Offset(-40.0, 0.0));
    await tester.pump();

    await gesture.moveBy(const Offset(-40.0, 0.0));
    await tester.pump();

    expect(log, isEmpty);

    await gesture.up();
    await tester.pumpAndSettle();

    expect(log, isEmpty);

    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Alaska'), findsOneWidget);
  });

  testWidgets('Bouncing scroll physics ballistics does not overshoot', (WidgetTester tester) async {
    final List<int> log = <int>[];
    final PageController controller = PageController(viewportFraction: 0.9);
    addTearDown(controller.dispose);

    Widget build(PageController controller, {Size? size}) {
      final Widget pageView = Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          controller: controller,
          onPageChanged: log.add,
          physics: const BouncingScrollPhysics(),
          children: kStates.map<Widget>((String state) => Text(state)).toList(),
        ),
      );

      if (size != null) {
        return OverflowBox(
          minWidth: size.width,
          minHeight: size.height,
          maxWidth: size.width,
          maxHeight: size.height,
          child: pageView,
        );
      } else {
        return pageView;
      }
    }

    await tester.pumpWidget(build(controller));
    expect(log, isEmpty);

    // Fling right to move to a non-existent page at the beginning of the
    // PageView, and confirm that the PageView settles back on the first page.
    await tester.fling(find.byType(PageView), const Offset(100.0, 0.0), 800.0);
    await tester.pumpAndSettle();
    expect(log, isEmpty);

    expect(find.text('Alabama'), findsOneWidget);
    expect(find.text('Alaska'), findsOneWidget);
    expect(find.text('Arizona'), findsNothing);

    // Try again with a Cupertino "Plus" device size.
    await tester.pumpWidget(build(controller, size: const Size(414.0, 736.0)));
    expect(log, isEmpty);

    await tester.fling(find.byType(PageView), const Offset(100.0, 0.0), 800.0);
    await tester.pumpAndSettle();
    expect(log, isEmpty);

    expect(find.text('Alabama'), findsOneWidget);
    expect(find.text('Alaska'), findsOneWidget);
    expect(find.text('Arizona'), findsNothing);
  });

  testWidgets('PageView viewportFraction', (WidgetTester tester) async {
    PageController controller = PageController(viewportFraction: 7 / 8);
    addTearDown(controller.dispose);

    Widget build(PageController controller) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageView.builder(
          controller: controller,
          itemCount: kStates.length,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 200.0,
              color: index.isEven ? const Color(0xFF0000FF) : const Color(0xFF00FF00),
              child: Text(kStates[index]),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(build(controller));

    expect(tester.getTopLeft(find.text('Alabama')), const Offset(50.0, 0.0));
    expect(tester.getTopLeft(find.text('Alaska')), const Offset(750.0, 0.0));

    controller.jumpToPage(10);
    await tester.pump();

    expect(tester.getTopLeft(find.text('Georgia')), const Offset(-650.0, 0.0));
    expect(tester.getTopLeft(find.text('Hawaii')), const Offset(50.0, 0.0));
    expect(tester.getTopLeft(find.text('Idaho')), const Offset(750.0, 0.0));

    controller = PageController(viewportFraction: 39 / 40);
    addTearDown(controller.dispose);

    await tester.pumpWidget(build(controller));

    expect(tester.getTopLeft(find.text('Georgia')), const Offset(-770.0, 0.0));
    expect(tester.getTopLeft(find.text('Hawaii')), const Offset(10.0, 0.0));
    expect(tester.getTopLeft(find.text('Idaho')), const Offset(790.0, 0.0));
  });

  testWidgets('Page snapping disable and reenable', (WidgetTester tester) async {
    final List<int> log = <int>[];

    Widget build({required bool pageSnapping}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          pageSnapping: pageSnapping,
          onPageChanged: log.add,
          children: kStates.map<Widget>((String state) => Text(state)).toList(),
        ),
      );
    }

    await tester.pumpWidget(build(pageSnapping: true));
    expect(log, isEmpty);

    // Drag more than halfway to the next page, to confirm the default behavior.
    TestGesture gesture = await tester.startGesture(const Offset(100.0, 100.0));
    // The page view is 800.0 wide, so this move is just beyond halfway.
    await gesture.moveBy(const Offset(-420.0, 0.0));

    expect(log, equals(const <int>[1]));
    log.clear();

    // Release the gesture, confirm that the page settles on the next.
    await gesture.up();
    await tester.pumpAndSettle();

    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Alaska'), findsOneWidget);

    // Disable page snapping, and try moving halfway. Confirm it doesn't snap.
    await tester.pumpWidget(build(pageSnapping: false));
    gesture = await tester.startGesture(const Offset(100.0, 100.0));
    // Move just beyond halfway, again.
    await gesture.moveBy(const Offset(-420.0, 0.0));

    // Page notifications still get sent.
    expect(log, equals(const <int>[2]));
    log.clear();

    // Release the gesture, confirm that both pages are visible.
    await gesture.up();
    await tester.pumpAndSettle();

    expect(find.text('Alabama'), findsNothing);
    expect(find.text('Alaska'), findsOneWidget);
    expect(find.text('Arizona'), findsOneWidget);
    expect(find.text('Arkansas'), findsNothing);

    // Now re-enable snapping, confirm that we've settled on a page.
    await tester.pumpWidget(build(pageSnapping: true));
    await tester.pumpAndSettle();

    expect(log, isEmpty);

    expect(find.text('Alaska'), findsNothing);
    expect(find.text('Arizona'), findsOneWidget);
    expect(find.text('Arkansas'), findsNothing);
  });

  testWidgets('PageView small viewportFraction', (WidgetTester tester) async {
    final PageController controller = PageController(viewportFraction: 1 / 8);
    addTearDown(controller.dispose);

    Widget build(PageController controller) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageView.builder(
          controller: controller,
          itemCount: kStates.length,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 200.0,
              color: index.isEven ? const Color(0xFF0000FF) : const Color(0xFF00FF00),
              child: Text(kStates[index]),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(build(controller));

    expect(tester.getTopLeft(find.text('Alabama')), const Offset(350.0, 0.0));
    expect(tester.getTopLeft(find.text('Alaska')), const Offset(450.0, 0.0));
    expect(tester.getTopLeft(find.text('Arizona')), const Offset(550.0, 0.0));
    expect(tester.getTopLeft(find.text('Arkansas')), const Offset(650.0, 0.0));
    expect(tester.getTopLeft(find.text('California')), const Offset(750.0, 0.0));

    controller.jumpToPage(10);
    await tester.pump();

    expect(tester.getTopLeft(find.text('Connecticut')), const Offset(-50.0, 0.0));
    expect(tester.getTopLeft(find.text('Delaware')), const Offset(50.0, 0.0));
    expect(tester.getTopLeft(find.text('Florida')), const Offset(150.0, 0.0));
    expect(tester.getTopLeft(find.text('Georgia')), const Offset(250.0, 0.0));
    expect(tester.getTopLeft(find.text('Hawaii')), const Offset(350.0, 0.0));
    expect(tester.getTopLeft(find.text('Idaho')), const Offset(450.0, 0.0));
    expect(tester.getTopLeft(find.text('Illinois')), const Offset(550.0, 0.0));
    expect(tester.getTopLeft(find.text('Indiana')), const Offset(650.0, 0.0));
    expect(tester.getTopLeft(find.text('Iowa')), const Offset(750.0, 0.0));
  });

  testWidgets('PageView large viewportFraction', (WidgetTester tester) async {
    final PageController controller = PageController(viewportFraction: 5 / 4);
    addTearDown(controller.dispose);

    Widget build(PageController controller) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageView.builder(
          controller: controller,
          itemCount: kStates.length,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 200.0,
              color: index.isEven ? const Color(0xFF0000FF) : const Color(0xFF00FF00),
              child: Text(kStates[index]),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(build(controller));

    expect(tester.getTopLeft(find.text('Alabama')), const Offset(-100.0, 0.0));
    expect(tester.getBottomRight(find.text('Alabama')), const Offset(900.0, 600.0));

    controller.jumpToPage(10);
    await tester.pump();

    expect(tester.getTopLeft(find.text('Hawaii')), const Offset(-100.0, 0.0));
  });

  testWidgets('Updating PageView large viewportFraction', (WidgetTester tester) async {
    Widget build(PageController controller) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageView.builder(
          controller: controller,
          itemCount: kStates.length,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 200.0,
              color: index.isEven ? const Color(0xFF0000FF) : const Color(0xFF00FF00),
              child: Text(kStates[index]),
            );
          },
        ),
      );
    }

    final PageController oldController = PageController(viewportFraction: 5 / 4);
    addTearDown(oldController.dispose);
    await tester.pumpWidget(build(oldController));

    expect(tester.getTopLeft(find.text('Alabama')), const Offset(-100, 0));
    expect(tester.getBottomRight(find.text('Alabama')), const Offset(900.0, 600.0));

    final PageController newController = PageController(viewportFraction: 4);
    addTearDown(newController.dispose);
    await tester.pumpWidget(build(newController));
    newController.jumpToPage(10);
    await tester.pump();

    expect(tester.getTopLeft(find.text('Hawaii')), const Offset(-(4 - 1) * 800 / 2, 0));
  });

  testWidgets('PageView large viewportFraction can scroll to the last page and snap', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/45096.
    final PageController controller = PageController(viewportFraction: 5 / 4);
    addTearDown(controller.dispose);

    Widget build(PageController controller) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageView.builder(
          controller: controller,
          itemCount: 3,
          itemBuilder: (BuildContext context, int index) {
            return Container(
              height: 200.0,
              color: index.isEven ? const Color(0xFF0000FF) : const Color(0xFF00FF00),
              child: Text(index.toString()),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(build(controller));

    expect(tester.getCenter(find.text('0')), const Offset(400, 300));

    controller.jumpToPage(2);
    await tester.pump();
    await tester.pumpAndSettle();

    expect(tester.getCenter(find.text('2')), const Offset(400, 300));
  });

  testWidgets('All visible pages are able to receive touch events', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/23873.
    final PageController controller = PageController(viewportFraction: 1 / 4);
    addTearDown(controller.dispose);
    late int tappedIndex;

    Widget build() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageView.builder(
          controller: controller,
          itemCount: 20,
          itemBuilder: (BuildContext context, int index) {
            return GestureDetector(
              onTap: () => tappedIndex = index,
              child: SizedBox.expand(child: Text('$index')),
            );
          },
        ),
      );
    }

    Iterable<int> visiblePages = const <int>[0, 1, 2];
    await tester.pumpWidget(build());

    // The first 3 items should be visible and tappable.
    for (final int index in visiblePages) {
      expect(find.text(index.toString()), findsOneWidget);
      // The center of page 2's x-coordinate is 800, so we have to manually
      // offset it a bit to make sure the tap lands within the screen.
      final Offset center = tester.getCenter(find.text('$index')) - const Offset(3, 0);
      await tester.tapAt(center);
      expect(tappedIndex, index);
    }

    controller.jumpToPage(19);
    await tester.pump();
    // The last 3 items should be visible and tappable.
    visiblePages = const <int>[17, 18, 19];
    for (final int index in visiblePages) {
      expect(find.text('$index'), findsOneWidget);
      await tester.tap(find.text('$index'));
      expect(tappedIndex, index);
    }
  });

  testWidgets('the current item remains centered on constraint change', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/50505.
    final PageController controller = PageController(
      initialPage: kStates.length - 1,
      viewportFraction: 0.5,
    );
    addTearDown(controller.dispose);

    Widget build(Size size) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox.fromSize(
            size: size,
            child: PageView(
              controller: controller,
              children: kStates.map<Widget>((String state) => Text(state)).toList(),
              onPageChanged: (int page) {},
            ),
          ),
        ),
      );
    }

    // Verifies that the last item is centered on screen.
    void verifyCentered() {
      expect(
        tester.getCenter(find.text(kStates.last)),
        offsetMoreOrLessEquals(const Offset(400, 300)),
      );
    }

    await tester.pumpWidget(build(const Size(300, 300)));
    await tester.pumpAndSettle();

    verifyCentered();

    await tester.pumpWidget(build(const Size(200, 300)));
    await tester.pumpAndSettle();

    verifyCentered();
  });

  testWidgets('PageView does not report page changed on overscroll', (WidgetTester tester) async {
    final PageController controller = PageController(initialPage: kStates.length - 1);
    addTearDown(controller.dispose);
    int changeIndex = 0;
    Widget build() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          controller: controller,
          children: kStates.map<Widget>((String state) => Text(state)).toList(),
          onPageChanged: (int page) {
            changeIndex = page;
          },
        ),
      );
    }

    await tester.pumpWidget(build());
    controller.jumpToPage(kStates.length * 2); // try to move beyond max range
    // change index should be zero, shouldn't fire onPageChanged
    expect(changeIndex, 0);
    await tester.pump();
    expect(changeIndex, 0);
  });

  testWidgets('PageView can restore page', (WidgetTester tester) async {
    final PageController controller = PageController();
    addTearDown(controller.dispose);
    expect(
      () => controller.page,
      throwsA(
        isAssertionError.having(
          (AssertionError error) => error.message,
          'message',
          equals('PageController.page cannot be accessed before a PageView is built with it.'),
        ),
      ),
    );
    final PageStorageBucket bucket = PageStorageBucket();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageStorage(
          bucket: bucket,
          child: PageView(
            key: const PageStorageKey<String>('PageView'),
            controller: controller,
            children: const <Widget>[Placeholder(), Placeholder(), Placeholder()],
          ),
        ),
      ),
    );
    expect(controller.page, 0);
    controller.jumpToPage(2);
    expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 2);
    expect(controller.page, 2);
    await tester.pumpWidget(PageStorage(bucket: bucket, child: Container()));
    expect(
      () => controller.page,
      throwsA(
        isAssertionError.having(
          (AssertionError error) => error.message,
          'message',
          equals('PageController.page cannot be accessed before a PageView is built with it.'),
        ),
      ),
    );
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageStorage(
          bucket: bucket,
          child: PageView(
            key: const PageStorageKey<String>('PageView'),
            controller: controller,
            children: const <Widget>[Placeholder(), Placeholder(), Placeholder()],
          ),
        ),
      ),
    );
    expect(controller.page, 2);

    final PageController controller2 = PageController(keepPage: false);
    addTearDown(controller2.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageStorage(
          bucket: bucket,
          child: PageView(
            key: const PageStorageKey<String>(
              'Check it again against your list and see consistency!',
            ),
            controller: controller2,
            children: const <Widget>[Placeholder(), Placeholder(), Placeholder()],
          ),
        ),
      ),
    );
    expect(controller2.page, 0);
  });

  testWidgets('PageView exposes semantics of children', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final PageController controller = PageController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          controller: controller,
          children: List<Widget>.generate(3, (int i) {
            return Semantics(container: true, child: Text('Page #$i'));
          }),
        ),
      ),
    );
    expect(controller.page, 0);

    expect(semantics, includesNodeWith(label: 'Page #0'));
    expect(semantics, isNot(includesNodeWith(label: 'Page #1')));
    expect(semantics, isNot(includesNodeWith(label: 'Page #2')));

    controller.jumpToPage(1);
    await tester.pumpAndSettle();

    expect(semantics, isNot(includesNodeWith(label: 'Page #0')));
    expect(semantics, includesNodeWith(label: 'Page #1'));
    expect(semantics, isNot(includesNodeWith(label: 'Page #2')));

    controller.jumpToPage(2);
    await tester.pumpAndSettle();

    expect(semantics, isNot(includesNodeWith(label: 'Page #0')));
    expect(semantics, isNot(includesNodeWith(label: 'Page #1')));
    expect(semantics, includesNodeWith(label: 'Page #2'));

    semantics.dispose();
  });

  testWidgets('PageMetrics', (WidgetTester tester) async {
    final PageMetrics page = PageMetrics(
      minScrollExtent: 100.0,
      maxScrollExtent: 200.0,
      pixels: 150.0,
      viewportDimension: 25.0,
      axisDirection: AxisDirection.right,
      viewportFraction: 1.0,
      devicePixelRatio: tester.view.devicePixelRatio,
    );
    expect(page.page, 6);
    final PageMetrics page2 = page.copyWith(pixels: page.pixels - 100.0);
    expect(page2.page, 4.0);
  });

  testWidgets('Page controller can handle rounding issue', (WidgetTester tester) async {
    final PageController pageController = PageController();
    addTearDown(pageController.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          controller: pageController,
          children: List<Widget>.generate(3, (int i) {
            return Semantics(container: true, child: Text('Page #$i'));
          }),
        ),
      ),
    );
    // Simulate precision error.
    pageController.position.jumpTo(799.99999999999);
    expect(pageController.page, 1);
  });

  testWidgets('PageView can participate in a11y scrolling', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final PageController controller = PageController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          controller: controller,
          allowImplicitScrolling: true,
          children: List<Widget>.generate(4, (int i) {
            return Semantics(container: true, child: Text('Page #$i'));
          }),
        ),
      ),
    );
    expect(controller.page, 0);

    expect(semantics, includesNodeWith(flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling]));
    expect(semantics, includesNodeWith(label: 'Page #0'));
    expect(
      semantics,
      includesNodeWith(label: 'Page #1', flags: <SemanticsFlag>[SemanticsFlag.isHidden]),
    );
    expect(
      semantics,
      isNot(includesNodeWith(label: 'Page #2', flags: <SemanticsFlag>[SemanticsFlag.isHidden])),
    );
    expect(
      semantics,
      isNot(includesNodeWith(label: 'Page #3', flags: <SemanticsFlag>[SemanticsFlag.isHidden])),
    );

    controller.nextPage(duration: const Duration(milliseconds: 150), curve: Curves.ease);
    await tester.pumpAndSettle();
    expect(
      semantics,
      includesNodeWith(label: 'Page #0', flags: <SemanticsFlag>[SemanticsFlag.isHidden]),
    );
    expect(semantics, includesNodeWith(label: 'Page #1'));
    expect(
      semantics,
      includesNodeWith(label: 'Page #2', flags: <SemanticsFlag>[SemanticsFlag.isHidden]),
    );
    expect(
      semantics,
      isNot(includesNodeWith(label: 'Page #3', flags: <SemanticsFlag>[SemanticsFlag.isHidden])),
    );

    controller.nextPage(duration: const Duration(milliseconds: 150), curve: Curves.ease);
    await tester.pumpAndSettle();
    expect(
      semantics,
      isNot(includesNodeWith(label: 'Page #0', flags: <SemanticsFlag>[SemanticsFlag.isHidden])),
    );
    expect(
      semantics,
      includesNodeWith(label: 'Page #1', flags: <SemanticsFlag>[SemanticsFlag.isHidden]),
    );
    expect(semantics, includesNodeWith(label: 'Page #2'));
    expect(
      semantics,
      includesNodeWith(label: 'Page #3', flags: <SemanticsFlag>[SemanticsFlag.isHidden]),
    );

    semantics.dispose();
  });

  testWidgets('PageView respects clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(children: <Widget>[Container(height: 2000.0)]),
      ),
    );

    // 1st, check that the render object has received the default clip behavior.
    final RenderViewport renderObject = tester.allRenderObjects.whereType<RenderViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.hardEdge));

    // 2nd, check that the painting context has received the default clip behavior.
    final TestClipPaintingContext context = TestClipPaintingContext();
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.hardEdge));

    // 3rd, pump a new widget to check that the render object can update its clip behavior.
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: PageView(
          clipBehavior: Clip.antiAlias,
          children: <Widget>[Container(height: 2000.0)],
        ),
      ),
    );
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));

    // 4th, check that a non-default clip behavior can be sent to the painting context.
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.antiAlias));
  });

  testWidgets('PageView.padEnds tests', (WidgetTester tester) async {
    Finder viewportFinder() => find.byType(SliverFillViewport, skipOffstage: false);

    // PageView() defaults to true.
    await tester.pumpWidget(Directionality(textDirection: TextDirection.ltr, child: PageView()));

    expect(tester.widget<SliverFillViewport>(viewportFinder()).padEnds, true);

    // PageView(padEnds: false) is propagated properly.
    await tester.pumpWidget(
      Directionality(textDirection: TextDirection.ltr, child: PageView(padEnds: false)),
    );

    expect(tester.widget<SliverFillViewport>(viewportFinder()).padEnds, false);
  });

  testWidgets('PageView - precision error inside RenderSliverFixedExtentBoxAdaptor', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/95101
    final PageController controller = PageController(initialPage: 152);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Center(
        child: SizedBox(
          width: 392.72727272727275,
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: PageView.builder(
              controller: controller,
              itemCount: 366,
              itemBuilder: (BuildContext context, int index) {
                return const SizedBox();
              },
            ),
          ),
        ),
      ),
    );

    controller.jumpToPage(365);
    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  testWidgets('PageView content should not be stretched on precision error', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/126561.
    final PageController controller = PageController();
    addTearDown(controller.dispose);

    const double pixel6EmulatorWidth = 411.42857142857144;

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SizedBox(
            width: pixel6EmulatorWidth,
            child: PageView(
              controller: controller,
              physics: const PageScrollPhysics().applyTo(const ClampingScrollPhysics()),
              children: const <Widget>[
                Center(child: Text('First Page')),
                Center(child: Text('Second Page')),
                Center(child: Text('Third Page')),
              ],
            ),
          ),
        ),
      ),
    );

    controller.animateToPage(2, duration: const Duration(milliseconds: 300), curve: Curves.ease);
    await tester.pumpAndSettle();

    final Finder transformFinder = find.descendant(
      of: find.byType(PageView),
      matching: find.byType(Transform),
    );
    expect(transformFinder, findsOneWidget);

    // Get the Transform widget that stretches the PageView.
    final Transform transform = tester.firstWidget<Transform>(
      find.descendant(of: find.byType(PageView), matching: find.byType(Transform)),
    );

    // Check the stretch factor in the first element of the transform matrix.
    expect(transform.transform.storage.first, 1.0);
  });

  testWidgets('PageController onAttach, onDetach', (WidgetTester tester) async {
    int attach = 0;
    int detach = 0;
    final PageController controller = PageController(
      onAttach: (_) {
        attach++;
      },
      onDetach: (_) {
        detach++;
      },
    );
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: PageView(
            controller: controller,
            physics: const PageScrollPhysics().applyTo(const ClampingScrollPhysics()),
            children: const <Widget>[
              Center(child: Text('First Page')),
              Center(child: Text('Second Page')),
              Center(child: Text('Third Page')),
            ],
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(attach, 1);
    expect(detach, 0);

    await tester.pumpWidget(Container());
    await tester.pumpAndSettle();

    expect(attach, 1);
    expect(detach, 1);
  });

  group('$PageView handles change of controller', () {
    final GlobalKey key = GlobalKey();

    Widget createPageView(PageController? controller) {
      return MaterialApp(
        home: Scaffold(
          body: PageView(
            key: key,
            controller: controller,
            children: const <Widget>[
              Center(child: Text('0')),
              Center(child: Text('1')),
              Center(child: Text('2')),
            ],
          ),
        ),
      );
    }

    Future<void> testPageViewWithController(
      PageController controller,
      WidgetTester tester,
      bool controls,
    ) async {
      int currentVisiblePage() {
        return int.parse(tester.widgetList(find.byType(Text)).whereType<Text>().first.data!);
      }

      final int initialPageInView = currentVisiblePage();

      for (int i = 0; i < 3; i++) {
        if (controls) {
          controller.jumpToPage(i);
          await tester.pumpAndSettle();
          expect(currentVisiblePage(), i);
        } else {
          expect(() => controller.jumpToPage(i), throwsAssertionError);
          expect(currentVisiblePage(), initialPageInView);
        }
      }
    }

    testWidgets('null to value', (WidgetTester tester) async {
      final PageController controller = PageController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(createPageView(null));
      await tester.pumpWidget(createPageView(controller));
      await testPageViewWithController(controller, tester, true);
    });

    testWidgets('value to value', (WidgetTester tester) async {
      final PageController controller1 = PageController();
      addTearDown(controller1.dispose);
      final PageController controller2 = PageController();
      addTearDown(controller2.dispose);
      await tester.pumpWidget(createPageView(controller1));
      await testPageViewWithController(controller1, tester, true);
      await tester.pumpWidget(createPageView(controller2));
      await testPageViewWithController(controller1, tester, false);
      await testPageViewWithController(controller2, tester, true);
    });

    testWidgets('value to null', (WidgetTester tester) async {
      final PageController controller = PageController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(createPageView(controller));
      await testPageViewWithController(controller, tester, true);
      await tester.pumpWidget(createPageView(null));
      await testPageViewWithController(controller, tester, false);
    });

    testWidgets('null to null', (WidgetTester tester) async {
      await tester.pumpWidget(createPageView(null));
      await tester.pumpWidget(createPageView(null));
    });
  });

  group('Asserts in jumpToPage and animateToPage methods works properly', () {
    Widget createPageView([PageController? controller]) {
      return MaterialApp(
        home: Scaffold(
          body: PageView(
            controller: controller,
            children: <Widget>[
              Container(color: Colors.red),
              Container(color: Colors.green),
              Container(color: Colors.blue),
            ],
          ),
        ),
      );
    }

    group('One pageController is attached to multiple PageViews', () {
      Widget createMultiplePageViews(PageController controller) {
        return MaterialApp(
          home: Scaffold(
            body: Column(
              children: <Widget>[
                Expanded(
                  child: PageView(
                    controller: controller,
                    children: <Widget>[
                      Container(color: Colors.red),
                      Container(color: Colors.green),
                      Container(color: Colors.blue),
                    ],
                  ),
                ),
                Expanded(
                  child: PageView(
                    controller: controller,
                    children: <Widget>[
                      Container(color: Colors.orange),
                      Container(color: Colors.purple),
                      Container(color: Colors.yellow),
                    ],
                  ),
                ),
              ],
            ),
          ),
        );
      }

      testWidgets(
        'animateToPage assertion is working properly when pageController is attached to multiple PageViews',
        (WidgetTester tester) async {
          final PageController controller = PageController();
          addTearDown(controller.dispose);
          await tester.pumpWidget(createMultiplePageViews(controller));

          expect(
            () => controller.animateToPage(
              2,
              duration: const Duration(milliseconds: 300),
              curve: Curves.ease,
            ),
            throwsA(
              isAssertionError.having(
                (AssertionError error) => error.message,
                'message',
                equals(
                  'Multiple PageViews are attached to '
                  'the same PageController.',
                ),
              ),
            ),
          );
        },
      );

      testWidgets(
        'jumpToPage assertion is working properly when pageController is attached to multiple PageViews',
        (WidgetTester tester) async {
          final PageController controller = PageController();
          addTearDown(controller.dispose);
          await tester.pumpWidget(createMultiplePageViews(controller));

          expect(
            () => controller.jumpToPage(2),
            throwsA(
              isAssertionError.having(
                (AssertionError error) => error.message,
                'message',
                equals(
                  'Multiple PageViews are attached to '
                  'the same PageController.',
                ),
              ),
            ),
          );
        },
      );
    });

    group('PageController is attached or is not attached to PageView', () {
      testWidgets('Assert behavior of animateToPage works properly', (WidgetTester tester) async {
        final PageController controller = PageController();
        addTearDown(controller.dispose);

        // pageController is not attached to PageView
        await tester.pumpWidget(createPageView());
        expect(
          () => controller.animateToPage(
            2,
            duration: const Duration(milliseconds: 300),
            curve: Curves.ease,
          ),
          throwsA(
            isAssertionError.having(
              (AssertionError error) => error.message,
              'message',
              equals('PageController is not attached to a PageView.'),
            ),
          ),
        );

        // pageController is attached to PageView
        await tester.pumpWidget(createPageView(controller));
        expect(
          () => controller.animateToPage(
            2,
            duration: const Duration(milliseconds: 300),
            curve: Curves.ease,
          ),
          returnsNormally,
        );
      });

      testWidgets('Assert behavior of jumpToPage works properly', (WidgetTester tester) async {
        final PageController controller = PageController();
        addTearDown(controller.dispose);

        // pageController is not attached to PageView
        await tester.pumpWidget(createPageView());
        expect(
          () => controller.jumpToPage(2),
          throwsA(
            isAssertionError.having(
              (AssertionError error) => error.message,
              'message',
              equals('PageController is not attached to a PageView.'),
            ),
          ),
        );

        // pageController is attached to PageView
        await tester.pumpWidget(createPageView(controller));
        expect(() => controller.jumpToPage(2), returnsNormally);
      });
    });
  });

  testWidgets(
    'Get the page value before the content dimension is determined,do not throw an assertion and return null',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/146986.
      final PageController controller = PageController();
      late String currentPage;
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Scaffold(
                  body: PageView(
                    controller: controller,
                    children: <Widget>[
                      Builder(
                        builder: (BuildContext context) {
                          currentPage = controller.page == null ? 'null' : 'not empty';
                          return Center(child: Text(currentPage));
                        },
                      ),
                    ],
                  ),
                  floatingActionButton: FloatingActionButton(
                    onPressed: () {
                      setState(() {});
                    },
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(find.text('null'), findsOneWidget);
      expect(find.text('not empty'), findsNothing);
      expect(currentPage, 'null');

      await tester.tap(find.byType(FloatingActionButton));
      await tester.pump();
      currentPage = controller.page == null ? 'null' : 'not empty';
      expect(find.text('not empty'), findsOneWidget);
      expect(find.text('null'), findsNothing);
      expect(currentPage, 'not empty');
    },
  );

  testWidgets('Does not crash when calling jumpToPage before layout', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/86222.
    final PageController controller = PageController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Navigator(
            onDidRemovePage: (Page<Object?> page) {},
            pages: <Page<void>>[
              MaterialPage<void>(
                child: Scaffold(
                  body: PageView(
                    controller: controller,
                    children: const <Widget>[
                      Scaffold(body: Text('One')),
                      Scaffold(body: Text('Two')),
                    ],
                  ),
                ),
              ),
              const MaterialPage<void>(child: Scaffold()),
            ],
          ),
        ),
      ),
    );

    controller.jumpToPage(1);
    expect(tester.takeException(), null);
  });

  testWidgets('Does not crash when calling animateToPage before layout', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/86222.
    final PageController controller = PageController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Navigator(
            onDidRemovePage: (Page<Object?> page) {},
            pages: <Page<void>>[
              MaterialPage<void>(
                child: Scaffold(
                  body: PageView(
                    controller: controller,
                    children: const <Widget>[
                      Scaffold(body: Text('One')),
                      Scaffold(body: Text('Two')),
                    ],
                  ),
                ),
              ),
              const MaterialPage<void>(child: Scaffold()),
            ],
          ),
        ),
      ),
    );

    controller.animateToPage(1, duration: const Duration(milliseconds: 50), curve: Curves.bounceIn);
    expect(tester.takeException(), null);
  });
}
