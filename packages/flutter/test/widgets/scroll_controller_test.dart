// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'states.dart';

void main() {
  testWidgets('ScrollController control test', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children:
              kStates.map<Widget>((String state) {
                return SizedBox(height: 200.0, child: Text(state));
              }).toList(),
        ),
      ),
    );

    double realOffset() {
      return tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels;
    }

    expect(controller.offset, equals(0.0));
    expect(realOffset(), equals(controller.offset));

    controller.jumpTo(653.0);

    expect(controller.offset, equals(653.0));
    expect(realOffset(), equals(controller.offset));

    await tester.pump();

    expect(controller.offset, equals(653.0));
    expect(realOffset(), equals(controller.offset));

    controller.animateTo(326.0, duration: const Duration(milliseconds: 300), curve: Curves.ease);
    await tester.pumpAndSettle();

    expect(controller.offset, equals(326.0));
    expect(realOffset(), equals(controller.offset));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          key: const Key('second'),
          controller: controller,
          children:
              kStates.map<Widget>((String state) {
                return SizedBox(height: 200.0, child: Text(state));
              }).toList(),
        ),
      ),
    );

    expect(controller.offset, equals(0.0));
    expect(realOffset(), equals(controller.offset));

    controller.jumpTo(653.0);

    expect(controller.offset, equals(653.0));
    expect(realOffset(), equals(controller.offset));

    final ScrollController controller2 = ScrollController();
    addTearDown(controller2.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          key: const Key('second'),
          controller: controller2,
          children:
              kStates.map<Widget>((String state) {
                return SizedBox(height: 200.0, child: Text(state));
              }).toList(),
        ),
      ),
    );

    expect(() => controller.offset, throwsAssertionError);
    expect(controller2.offset, equals(653.0));
    expect(realOffset(), equals(controller2.offset));

    expect(() => controller.jumpTo(120.0), throwsAssertionError);
    expect(
      () => controller.animateTo(
        132.0,
        duration: const Duration(milliseconds: 300),
        curve: Curves.ease,
      ),
      throwsAssertionError,
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          key: const Key('second'),
          controller: controller2,
          physics: const BouncingScrollPhysics(),
          children:
              kStates.map<Widget>((String state) {
                return SizedBox(height: 200.0, child: Text(state));
              }).toList(),
        ),
      ),
    );

    expect(controller2.offset, equals(653.0));
    expect(realOffset(), equals(controller2.offset));

    controller2.jumpTo(432.0);

    expect(controller2.offset, equals(432.0));
    expect(realOffset(), equals(controller2.offset));

    await tester.pump();

    expect(controller2.offset, equals(432.0));
    expect(realOffset(), equals(controller2.offset));
  });

  testWidgets('ScrollController control test', (WidgetTester tester) async {
    final ScrollController controller = ScrollController(initialScrollOffset: 209.0);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(
          crossAxisCount: 4,
          controller: controller,
          children: kStates.map<Widget>((String state) => Text(state)).toList(),
        ),
      ),
    );

    double realOffset() {
      return tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels;
    }

    expect(controller.offset, equals(209.0));
    expect(realOffset(), equals(controller.offset));

    controller.jumpTo(105.0);

    await tester.pump();

    expect(controller.offset, equals(105.0));
    expect(realOffset(), equals(controller.offset));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: GridView.count(
          crossAxisCount: 2,
          controller: controller,
          children: kStates.map<Widget>((String state) => Text(state)).toList(),
        ),
      ),
    );

    expect(controller.offset, equals(105.0));
    expect(realOffset(), equals(controller.offset));
  });

  testWidgets('DrivenScrollActivity ending after dispose', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(controller: controller, children: <Widget>[Container(height: 200000.0)]),
      ),
    );

    controller.animateTo(1000.0, duration: const Duration(seconds: 1), curve: Curves.linear);

    await tester.pump(); // Start the animation.

    // We will now change the tree on the same frame as the animation ends.
    await tester.pumpWidget(Container(), duration: const Duration(seconds: 2));
  });

  testWidgets('Read operations on ScrollControllers with no positions fail', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    expect(() => controller.offset, throwsAssertionError);
    expect(() => controller.position, throwsAssertionError);
  });

  testWidgets('Read operations on ScrollControllers with more than one position fail', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          children: <Widget>[
            Container(
              constraints: const BoxConstraints(maxHeight: 500.0),
              child: ListView(
                controller: controller,
                children:
                    kStates.map<Widget>((String state) {
                      return SizedBox(height: 200.0, child: Text(state));
                    }).toList(),
              ),
            ),
            Container(
              constraints: const BoxConstraints(maxHeight: 500.0),
              child: ListView(
                controller: controller,
                children:
                    kStates.map<Widget>((String state) {
                      return SizedBox(height: 200.0, child: Text(state));
                    }).toList(),
              ),
            ),
          ],
        ),
      ),
    );

    expect(() => controller.offset, throwsAssertionError);
    expect(() => controller.position, throwsAssertionError);
  });

  testWidgets('Write operations on ScrollControllers with no positions fail', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    expect(
      () => controller.animateTo(1.0, duration: const Duration(seconds: 1), curve: Curves.linear),
      throwsAssertionError,
    );
    expect(() => controller.jumpTo(1.0), throwsAssertionError);
  });

  testWidgets('Write operations on ScrollControllers with more than one position do not throw', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          children: <Widget>[
            Container(
              constraints: const BoxConstraints(maxHeight: 500.0),
              child: ListView(
                controller: controller,
                children:
                    kStates.map<Widget>((String state) {
                      return SizedBox(height: 200.0, child: Text(state));
                    }).toList(),
              ),
            ),
            Container(
              constraints: const BoxConstraints(maxHeight: 500.0),
              child: ListView(
                controller: controller,
                children:
                    kStates.map<Widget>((String state) {
                      return SizedBox(height: 200.0, child: Text(state));
                    }).toList(),
              ),
            ),
          ],
        ),
      ),
    );

    controller.jumpTo(1.0);
    controller.animateTo(1.0, duration: const Duration(seconds: 1), curve: Curves.linear);
    await tester.pumpAndSettle();
  });

  testWidgets('Scroll controllers notify when the position changes', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();

    final List<double> log = <double>[];

    controller.addListener(() {
      log.add(controller.offset);
    });

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children:
              kStates.map<Widget>((String state) {
                return SizedBox(height: 200.0, child: Text(state));
              }).toList(),
        ),
      ),
    );

    expect(log, isEmpty);

    await tester.drag(find.byType(ListView), const Offset(0.0, -250.0));

    expect(log, equals(<double>[20.0, 250.0]));
    log.clear();

    controller.dispose();

    await tester.drag(find.byType(ListView), const Offset(0.0, -130.0));
    expect(log, isEmpty);
  });

  testWidgets('keepScrollOffset', (WidgetTester tester) async {
    final PageStorageBucket bucket = PageStorageBucket();

    Widget buildFrame(ScrollController controller) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: PageStorage(
          bucket: bucket,
          child: KeyedSubtree(
            key: const PageStorageKey<String>('ListView'),
            child: ListView(
              key: UniqueKey(), // it's a different ListView every time
              controller: controller,
              children:
                  List<Widget>.generate(50, (int index) {
                    return SizedBox(height: 100.0, child: Text('Item $index'));
                  }).toList(),
            ),
          ),
        ),
      );
    }

    // keepScrollOffset: true (the default). The scroll offset is restored
    // when the ListView is recreated with a new ScrollController.

    // The initialScrollOffset is used in this case, because there's no saved
    // scroll offset.
    ScrollController controller = ScrollController(initialScrollOffset: 200.0);
    addTearDown(controller.dispose);
    await tester.pumpWidget(buildFrame(controller));
    expect(tester.getTopLeft(find.widgetWithText(SizedBox, 'Item 2')), Offset.zero);

    controller.jumpTo(2000.0);
    await tester.pump();
    expect(tester.getTopLeft(find.widgetWithText(SizedBox, 'Item 20')), Offset.zero);

    // The initialScrollOffset isn't used in this case, because the scrolloffset
    // can be restored.
    controller = ScrollController(initialScrollOffset: 25.0);
    addTearDown(controller.dispose);
    await tester.pumpWidget(buildFrame(controller));
    expect(controller.offset, 2000.0);
    expect(tester.getTopLeft(find.widgetWithText(SizedBox, 'Item 20')), Offset.zero);

    // keepScrollOffset: false. The scroll offset is -not- restored
    // when the ListView is recreated with a new ScrollController and
    // the initialScrollOffset is used.

    controller = ScrollController(keepScrollOffset: false, initialScrollOffset: 100.0);
    addTearDown(controller.dispose);
    await tester.pumpWidget(buildFrame(controller));
    expect(controller.offset, 100.0);
    expect(tester.getTopLeft(find.widgetWithText(SizedBox, 'Item 1')), Offset.zero);
  });

  testWidgets('isScrollingNotifier works with pointer scroll', (WidgetTester tester) async {
    Widget buildFrame(ScrollController controller) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children:
              List<Widget>.generate(50, (int index) {
                return SizedBox(height: 100.0, child: Text('Item $index'));
              }).toList(),
        ),
      );
    }

    bool isScrolling = false;
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    controller.addListener(() {
      isScrolling = controller.position.isScrollingNotifier.value;
    });
    await tester.pumpWidget(buildFrame(controller));
    final Offset scrollEventLocation = tester.getCenter(find.byType(ListView));
    final TestPointer testPointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    // Create a hover event so that |testPointer| has a location when generating the scroll.
    testPointer.hover(scrollEventLocation);
    await tester.sendEventToBinding(testPointer.scroll(const Offset(0.0, 300.0)));
    // When the listener was notified, the value of the isScrollingNotifier
    // should have been true
    expect(isScrolling, isTrue);
  });

  test('$ScrollController dispatches object creation in constructor', () async {
    await expectLater(
      await memoryEvents(() => ScrollController().dispose(), ScrollController),
      areCreateAndDispose,
    );
  });
}
