// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

ScrollController _controller = ScrollController(initialScrollOffset: 110.0);

class ThePositiveNumbers extends StatelessWidget {
  const ThePositiveNumbers({super.key, required this.from});
  final int from;
  @override
  Widget build(BuildContext context) {
    return ListView.builder(
      key: const PageStorageKey<String>('ThePositiveNumbers'),
      itemExtent: 100.0,
      controller: _controller,
      itemBuilder: (BuildContext context, int index) {
        return Text('${index + from}', key: ValueKey<int>(index));
      },
    );
  }
}

Future<void> performTest(WidgetTester tester, bool maintainState) async {
  final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();
  await tester.pumpWidget(
    Directionality(
      textDirection: TextDirection.ltr,
      child: MediaQuery(
        data: MediaQueryData.fromView(tester.view),
        child: Navigator(
          key: navigatorKey,
          onGenerateRoute: (RouteSettings settings) {
            if (settings.name == '/') {
              return MaterialPageRoute<void>(
                settings: settings,
                builder: (_) => const ThePositiveNumbers(from: 0),
                maintainState: maintainState,
              );
            } else if (settings.name == '/second') {
              return MaterialPageRoute<void>(
                settings: settings,
                builder: (_) => const ThePositiveNumbers(from: 10000),
                maintainState: maintainState,
              );
            }
            return null;
          },
        ),
      ),
    ),
  );

  // we're 600 pixels high, each item is 100 pixels high, scroll position is
  // 110.0, so we should have 7 items, 1..7.
  expect(find.text('0'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('1'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('2'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('3'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('4'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('5'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('6'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('7'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('8'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('10'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('100'), findsNothing, reason: 'with maintainState: $maintainState');

  tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(1000.0);
  await tester.pump(const Duration(seconds: 1));

  // we're 600 pixels high, each item is 100 pixels high, scroll position is
  // 1000, so we should have exactly 6 items, 10..15.

  expect(find.text('0'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('1'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('8'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('9'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('10'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('11'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('12'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('13'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('14'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('15'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('16'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('100'), findsNothing, reason: 'with maintainState: $maintainState');

  navigatorKey.currentState!.pushNamed('/second');
  await tester.pump(); // navigating always takes two frames, one to start...
  await tester.pump(const Duration(seconds: 1)); // ...and one to end the transition

  // the second list is now visible, starting at 10001
  expect(find.text('0'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('1'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('10'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('11'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('10000'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('10001'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('10002'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('10003'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('10004'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('10005'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('10006'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('10007'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('10008'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('10010'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('10100'), findsNothing, reason: 'with maintainState: $maintainState');

  navigatorKey.currentState!.pop();
  await tester.pump(); // again, navigating always takes two frames

  // Ensure we don't clamp the scroll offset even during the navigation.
  // https://github.com/flutter/flutter/issues/4883
  final ScrollableState state = tester.state(find.byType(Scrollable).first);
  expect(state.position.pixels, equals(1000.0), reason: 'with maintainState: $maintainState');

  await tester.pump(const Duration(seconds: 1));

  // we're 600 pixels high, each item is 100 pixels high, scroll position is
  // 1000, so we should have exactly 6 items, 10..15.

  expect(find.text('0'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('1'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('8'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('9'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('10'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('11'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('12'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('13'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('14'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('15'), findsOneWidget, reason: 'with maintainState: $maintainState');
  expect(find.text('16'), findsNothing, reason: 'with maintainState: $maintainState');
  expect(find.text('100'), findsNothing, reason: 'with maintainState: $maintainState');
}

void main() {
  testWidgets("ScrollPosition jumpTo() doesn't call notifyListeners twice", (
    WidgetTester tester,
  ) async {
    int count = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: ListView.builder(
          itemBuilder: (BuildContext context, int index) {
            return Text('$index', textDirection: TextDirection.ltr);
          },
        ),
      ),
    );

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    position.addListener(() {
      count++;
    });
    position.jumpTo(100);

    expect(count, 1);
  });

  testWidgets('whether we remember our scroll position', (WidgetTester tester) async {
    await performTest(tester, true);
    await performTest(tester, false);
  });

  testWidgets('scroll alignment is honored by ensureVisible', (WidgetTester tester) async {
    final List<int> items = List<int>.generate(11, (int index) => index).toList();
    final List<FocusNode> nodes =
        List<FocusNode>.generate(
          11,
          (int index) => FocusNode(debugLabel: 'Item ${index + 1}'),
        ).toList();
    addTearDown(() {
      for (final FocusNode node in nodes) {
        node.dispose();
      }
    });
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: ListView(
          controller: controller,
          children:
              items.map<Widget>((int item) {
                return Focus(
                  key: ValueKey<int>(item),
                  focusNode: nodes[item],
                  child: Container(height: 110),
                );
              }).toList(),
        ),
      ),
    );

    controller.position.ensureVisible(
      tester.renderObject(find.byKey(const ValueKey<int>(0))),
      alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
    );
    expect(controller.position.pixels, equals(0.0));

    controller.position.ensureVisible(
      tester.renderObject(find.byKey(const ValueKey<int>(1))),
      alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
    );
    expect(controller.position.pixels, equals(0.0));

    controller.position.ensureVisible(
      tester.renderObject(find.byKey(const ValueKey<int>(1))),
      alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
    );
    expect(controller.position.pixels, equals(0.0));

    controller.position.ensureVisible(
      tester.renderObject(find.byKey(const ValueKey<int>(4))),
      alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
    );
    expect(controller.position.pixels, equals(0.0));

    controller.position.ensureVisible(
      tester.renderObject(find.byKey(const ValueKey<int>(5))),
      alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
    );
    expect(controller.position.pixels, equals(0.0));

    controller.position.ensureVisible(
      tester.renderObject(find.byKey(const ValueKey<int>(5))),
      alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
    );
    expect(controller.position.pixels, equals(60.0));

    controller.position.ensureVisible(
      tester.renderObject(find.byKey(const ValueKey<int>(0))),
      alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
    );
    expect(controller.position.pixels, equals(0.0));
  });

  testWidgets('jumpTo recommends deferred loading', (WidgetTester tester) async {
    int loadedWithDeferral = 0;
    int buildCount = 0;
    const double height = 500;
    await tester.pumpWidget(
      MaterialApp(
        home: ListView.builder(
          itemBuilder: (BuildContext context, int index) {
            buildCount += 1;
            if (Scrollable.recommendDeferredLoadingForContext(context)) {
              loadedWithDeferral += 1;
            }
            return const SizedBox(height: height);
          },
        ),
      ),
    );

    // The two visible on screen should have loaded without deferral.
    expect(buildCount, 2);
    expect(loadedWithDeferral, 0);

    final ScrollPosition position = tester.state<ScrollableState>(find.byType(Scrollable)).position;
    position.jumpTo(height * 100);
    await tester.pump();

    // All but the first two that were loaded normally should have gotten a
    // recommendation to defer.
    expect(buildCount, 102);
    expect(loadedWithDeferral, 100);

    position.jumpTo(height * 102);
    await tester.pump();

    // The smaller jump should not have recommended deferral.
    expect(buildCount, 104);
    expect(loadedWithDeferral, 100);
  });
}
