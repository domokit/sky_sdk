// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

bool refreshCalled = false;

Future<void> refresh() {
  refreshCalled = true;
  return Future<void>.value();
}

Future<void> holdRefresh() {
  refreshCalled = true;
  return Completer<void>().future;
}

void main() {
  testWidgets('RefreshIndicator', (WidgetTester tester) async {
    refreshCalled = false;
    final SemanticsHandle handle = tester.ensureSemantics();
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children:
                <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                  return SizedBox(height: 200.0, child: Text(item));
                }).toList(),
          ),
        ),
      ),
    );

    await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0);
    await tester.pump();

    expect(
      tester.getSemantics(find.byType(RefreshProgressIndicator)),
      matchesSemantics(label: 'Refresh'),
    );

    await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation
    expect(refreshCalled, true);
    handle.dispose();
  });

  testWidgets('Refresh Indicator - nested', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          notificationPredicate: (ScrollNotification notification) => notification.depth == 1,
          onRefresh: refresh,
          child: SingleChildScrollView(
            scrollDirection: Axis.horizontal,
            child: SizedBox(
              width: 600.0,
              child: ListView(
                physics: const AlwaysScrollableScrollPhysics(),
                children:
                    <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                      return SizedBox(height: 200.0, child: Text(item));
                    }).toList(),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.fling(find.text('A'), const Offset(300.0, 0.0), 1000.0); // horizontal fling
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation
    expect(refreshCalled, false);

    await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0); // vertical fling
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation
    expect(refreshCalled, true);
  });

  testWidgets('RefreshIndicator - reverse', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            reverse: true,
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    await tester.fling(find.text('X'), const Offset(0.0, 600.0), 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation
    expect(refreshCalled, true);
  });

  testWidgets('RefreshIndicator - top - position', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: holdRefresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    await tester.fling(find.text('X'), const Offset(0.0, 300.0), 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(tester.getCenter(find.byType(RefreshProgressIndicator)).dy, lessThan(300.0));
  });

  testWidgets('RefreshIndicator - reverse - position', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: holdRefresh,
          child: ListView(
            reverse: true,
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    await tester.fling(find.text('X'), const Offset(0.0, 600.0), 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(tester.getCenter(find.byType(RefreshProgressIndicator)).dy, lessThan(300.0));
  });

  testWidgets('RefreshIndicator - no movement', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    // this fling is horizontal, not up or down
    await tester.fling(find.text('X'), const Offset(1.0, 0.0), 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, false);
  });

  testWidgets('RefreshIndicator - not enough', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    await tester.fling(find.text('X'), const Offset(0.0, 50.0), 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, false);
  });

  testWidgets('RefreshIndicator - just enough', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    await tester.fling(find.text('X'), const Offset(0.0, 200.0), 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, true);
  });

  testWidgets('RefreshIndicator - drag back not far enough to cancel', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[
              SizedBox(height: 200.0, child: Text('X')),
              SizedBox(height: 1000),
            ],
          ),
        ),
      ),
    );

    final Offset startLocation = tester.getCenter(
      find.text('X'),
      warnIfMissed: true,
      callee: 'drag',
    );
    final TestPointer testPointer = TestPointer();
    await tester.sendEventToBinding(testPointer.down(startLocation));
    await tester.sendEventToBinding(testPointer.move(startLocation + const Offset(0.0, 175)));
    await tester.pump();
    await tester.sendEventToBinding(testPointer.move(startLocation + const Offset(0.0, 150)));
    await tester.pump();
    await tester.sendEventToBinding(testPointer.up());
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, true);
  });

  testWidgets('RefreshIndicator - drag back far enough to cancel', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[
              SizedBox(height: 200.0, child: Text('X')),
              SizedBox(height: 1000),
            ],
          ),
        ),
      ),
    );

    final Offset startLocation = tester.getCenter(
      find.text('X'),
      warnIfMissed: true,
      callee: 'drag',
    );
    final TestPointer testPointer = TestPointer();
    await tester.sendEventToBinding(testPointer.down(startLocation));
    await tester.sendEventToBinding(testPointer.move(startLocation + const Offset(0.0, 175)));
    await tester.pump();
    await tester.sendEventToBinding(testPointer.move(startLocation + const Offset(0.0, 149)));
    await tester.pump();
    await tester.sendEventToBinding(testPointer.up());
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, false);
  });

  testWidgets('RefreshIndicator - show - slow', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: holdRefresh, // this one never returns
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    bool completed = false;
    tester.state<RefreshIndicatorState>(find.byType(RefreshIndicator)).show().then<void>((
      void value,
    ) {
      completed = true;
    });
    await tester.pump();
    expect(completed, false);
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, true);
    expect(completed, false);
    completed = false;
    refreshCalled = false;
    tester.state<RefreshIndicatorState>(find.byType(RefreshIndicator)).show().then<void>((
      void value,
    ) {
      completed = true;
    });
    await tester.pump();
    expect(completed, false);
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, false);
  });

  testWidgets('RefreshIndicator - show - fast', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    bool completed = false;
    tester.state<RefreshIndicatorState>(find.byType(RefreshIndicator)).show().then<void>((
      void value,
    ) {
      completed = true;
    });
    await tester.pump();
    expect(completed, false);
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, true);
    expect(completed, true);
    completed = false;
    refreshCalled = false;
    tester.state<RefreshIndicatorState>(find.byType(RefreshIndicator)).show().then<void>((
      void value,
    ) {
      completed = true;
    });
    await tester.pump();
    expect(completed, false);
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, true);
    expect(completed, true);
  });

  testWidgets('RefreshIndicator - show - fast - twice', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[SizedBox(height: 200.0, child: Text('X'))],
          ),
        ),
      ),
    );

    bool completed1 = false;
    tester.state<RefreshIndicatorState>(find.byType(RefreshIndicator)).show().then<void>((
      void value,
    ) {
      completed1 = true;
    });
    bool completed2 = false;
    tester.state<RefreshIndicatorState>(find.byType(RefreshIndicator)).show().then<void>((
      void value,
    ) {
      completed2 = true;
    });
    await tester.pump();
    expect(completed1, false);
    expect(completed2, false);
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(refreshCalled, true);
    expect(completed1, true);
    expect(completed2, true);
  });

  testWidgets(
    'Refresh starts while scroll view moves back to 0.0 after overscroll',
    (WidgetTester tester) async {
      refreshCalled = false;
      double lastScrollOffset;
      final ScrollController controller = ScrollController();

      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator(
            onRefresh: refresh,
            child: ListView(
              controller: controller,
              physics: const AlwaysScrollableScrollPhysics(),
              children:
                  <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                    return SizedBox(height: 200.0, child: Text(item));
                  }).toList(),
            ),
          ),
        ),
      );

      if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
        await tester.fling(find.text('A'), const Offset(0.0, 1500.0), 10000.0);
      } else {
        await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0);
      }
      await tester.pump(const Duration(milliseconds: 100));
      expect(lastScrollOffset = controller.offset, lessThan(0.0));
      expect(refreshCalled, isFalse);

      await tester.pump(const Duration(milliseconds: 400));
      expect(controller.offset, greaterThan(lastScrollOffset));
      expect(controller.offset, lessThan(0.0));
      expect(refreshCalled, isTrue);

      controller.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('RefreshIndicator does not force child to relayout', (WidgetTester tester) async {
    int layoutCount = 0;

    Widget layoutCallback(BuildContext context, BoxConstraints constraints) {
      layoutCount++;
      return ListView(
        physics: const AlwaysScrollableScrollPhysics(),
        children:
            <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
              return SizedBox(height: 200.0, child: Text(item));
            }).toList(),
      );
    }

    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(onRefresh: refresh, child: LayoutBuilder(builder: layoutCallback)),
      ),
    );

    await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0); // trigger refresh
    await tester.pump();

    await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation

    expect(layoutCount, 1);
  });

  testWidgets('RefreshIndicator responds to strokeWidth', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: () async {},
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children:
                <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                  return SizedBox(height: 200.0, child: Text(item));
                }).toList(),
          ),
        ),
      ),
    );

    // Check for the default value
    expect(
      tester.widget<RefreshIndicator>(find.byType(RefreshIndicator)).strokeWidth,
      RefreshProgressIndicator.defaultStrokeWidth,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: () async {},
          strokeWidth: 4.0,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children:
                <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                  return SizedBox(height: 200.0, child: Text(item));
                }).toList(),
          ),
        ),
      ),
    );

    expect(tester.widget<RefreshIndicator>(find.byType(RefreshIndicator)).strokeWidth, 4.0);
  });

  testWidgets('RefreshIndicator responds to edgeOffset', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: () async {},
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children:
                <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                  return SizedBox(height: 200.0, child: Text(item));
                }).toList(),
          ),
        ),
      ),
    );

    //By default the value of edgeOffset is 0.0
    expect(tester.widget<RefreshIndicator>(find.byType(RefreshIndicator)).edgeOffset, 0.0);

    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: () async {},
          edgeOffset: kToolbarHeight,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children:
                <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                  return SizedBox(height: 200.0, child: Text(item));
                }).toList(),
          ),
        ),
      ),
    );

    expect(
      tester.widget<RefreshIndicator>(find.byType(RefreshIndicator)).edgeOffset,
      kToolbarHeight,
    );
  });

  testWidgets('RefreshIndicator appears at edgeOffset', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          edgeOffset: kToolbarHeight,
          displacement: kToolbarHeight,
          onRefresh: () async {
            await Future<void>.delayed(const Duration(seconds: 1), () {});
          },
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children:
                <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                  return SizedBox(height: 200.0, child: Text(item));
                }).toList(),
          ),
        ),
      ),
    );

    await tester.fling(find.byType(ListView), const Offset(0.0, 2.0 * kToolbarHeight), 1000.0);
    await tester.pump(const Duration(seconds: 2));

    expect(
      tester.getTopLeft(find.byType(RefreshProgressIndicator)).dy,
      greaterThanOrEqualTo(2.0 * kToolbarHeight),
    );
  });

  testWidgets(
    'Top RefreshIndicator(anywhere mode) should be shown when dragging from non-zero scroll position',
    (WidgetTester tester) async {
      refreshCalled = false;
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator(
            triggerMode: RefreshIndicatorTriggerMode.anywhere,
            onRefresh: holdRefresh,
            child: ListView(
              controller: scrollController,
              physics: const AlwaysScrollableScrollPhysics(),
              children: const <Widget>[
                SizedBox(height: 200.0, child: Text('X')),
                SizedBox(height: 800.0, child: Text('Y')),
              ],
            ),
          ),
        ),
      );

      scrollController.jumpTo(50.0);

      await tester.fling(find.text('X'), const Offset(0.0, 300.0), 1000.0);
      await tester.pump();
      await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
      await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
      expect(tester.getCenter(find.byType(RefreshProgressIndicator)).dy, lessThan(300.0));

      scrollController.dispose();
    },
  );

  testWidgets(
    'Reverse RefreshIndicator(anywhere mode) should be shown when dragging from non-zero scroll position',
    (WidgetTester tester) async {
      refreshCalled = false;
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator(
            triggerMode: RefreshIndicatorTriggerMode.anywhere,
            onRefresh: holdRefresh,
            child: ListView(
              reverse: true,
              controller: scrollController,
              physics: const AlwaysScrollableScrollPhysics(),
              children: const <Widget>[
                SizedBox(height: 200.0, child: Text('X')),
                SizedBox(height: 800.0, child: Text('Y')),
              ],
            ),
          ),
        ),
      );

      scrollController.jumpTo(50.0);

      await tester.fling(find.text('X'), const Offset(0.0, 600.0), 1000.0);
      await tester.pump();
      await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
      await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
      expect(tester.getCenter(find.byType(RefreshProgressIndicator)).dy, lessThan(300.0));

      scrollController.dispose();
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/71936
  testWidgets(
    'RefreshIndicator(anywhere mode) should not be shown when overscroll occurs due to inertia',
    (WidgetTester tester) async {
      refreshCalled = false;
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator(
            triggerMode: RefreshIndicatorTriggerMode.anywhere,
            onRefresh: holdRefresh,
            child: ListView(
              controller: scrollController,
              physics: const AlwaysScrollableScrollPhysics(),
              children: const <Widget>[
                SizedBox(height: 200.0, child: Text('X')),
                SizedBox(height: 2000.0, child: Text('Y')),
              ],
            ),
          ),
        ),
      );

      scrollController.jumpTo(100.0);

      // Release finger before reach the edge.
      await tester.fling(find.text('X'), const Offset(0.0, 99.0), 1000.0);
      await tester.pump();
      await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
      await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
      expect(find.byType(RefreshProgressIndicator), findsNothing);

      scrollController.dispose();
    },
  );

  testWidgets(
    'Top RefreshIndicator(onEdge mode) should not be shown when dragging from non-zero scroll position',
    (WidgetTester tester) async {
      refreshCalled = false;
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator(
            onRefresh: holdRefresh,
            child: ListView(
              controller: scrollController,
              physics: const AlwaysScrollableScrollPhysics(),
              children: const <Widget>[
                SizedBox(height: 200.0, child: Text('X')),
                SizedBox(height: 800.0, child: Text('Y')),
              ],
            ),
          ),
        ),
      );

      scrollController.jumpTo(50.0);

      await tester.fling(find.text('X'), const Offset(0.0, 300.0), 1000.0);
      await tester.pump();
      await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
      await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
      expect(find.byType(RefreshProgressIndicator), findsNothing);

      scrollController.dispose();
    },
  );

  testWidgets(
    'Reverse RefreshIndicator(onEdge mode) should be shown when dragging from non-zero scroll position',
    (WidgetTester tester) async {
      refreshCalled = false;
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator(
            onRefresh: holdRefresh,
            child: ListView(
              reverse: true,
              controller: scrollController,
              physics: const AlwaysScrollableScrollPhysics(),
              children: const <Widget>[
                SizedBox(height: 200.0, child: Text('X')),
                SizedBox(height: 800.0, child: Text('Y')),
              ],
            ),
          ),
        ),
      );

      scrollController.jumpTo(50.0);

      await tester.fling(find.text('X'), const Offset(0.0, -300.0), 1000.0);
      await tester.pump();
      await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
      await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
      expect(find.byType(RefreshProgressIndicator), findsNothing);

      scrollController.dispose();
    },
  );

  testWidgets('ScrollController.jumpTo should not trigger the refresh indicator', (
    WidgetTester tester,
  ) async {
    refreshCalled = false;
    final ScrollController scrollController = ScrollController(initialScrollOffset: 500.0);
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            controller: scrollController,
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[
              SizedBox(height: 800.0, child: Text('X')),
              SizedBox(height: 800.0, child: Text('Y')),
            ],
          ),
        ),
      ),
    );

    scrollController.jumpTo(0.0);
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation

    expect(refreshCalled, false);

    scrollController.dispose();
  });

  testWidgets('RefreshIndicator.adaptive', (WidgetTester tester) async {
    Widget buildFrame(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: RefreshIndicator.adaptive(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children:
                <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                  return SizedBox(height: 200.0, child: Text(item));
                }).toList(),
          ),
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(buildFrame(platform));
      await tester.pumpAndSettle(); // Finish the theme change animation.
      await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0);
      await tester.pump();

      expect(find.byType(CupertinoActivityIndicator), findsOneWidget);
      expect(find.byType(RefreshProgressIndicator), findsNothing);
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(buildFrame(platform));
      await tester.pumpAndSettle(); // Finish the theme change animation.
      await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0);
      await tester.pump();

      expect(
        tester.getSemantics(find.byType(RefreshProgressIndicator)),
        matchesSemantics(label: 'Refresh'),
      );
      expect(find.byType(CupertinoActivityIndicator), findsNothing);
    }
  });

  testWidgets('RefreshIndicator color defaults to ColorScheme.primary', (
    WidgetTester tester,
  ) async {
    const Color primaryColor = Color(0xff4caf50);
    final ThemeData theme = ThemeData.from(
      colorScheme: const ColorScheme.light().copyWith(primary: primaryColor),
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter stateSetter) {
            return RefreshIndicator(
              triggerMode: RefreshIndicatorTriggerMode.anywhere,
              onRefresh: holdRefresh,
              child: ListView(
                reverse: true,
                physics: const AlwaysScrollableScrollPhysics(),
                children: const <Widget>[
                  SizedBox(height: 200.0, child: Text('X')),
                  SizedBox(height: 800.0, child: Text('Y')),
                ],
              ),
            );
          },
        ),
      ),
    );

    await tester.fling(find.text('X'), const Offset(0.0, 600.0), 1000.0);
    await tester.pump();
    expect(
      tester
          .widget<RefreshProgressIndicator>(find.byType(RefreshProgressIndicator))
          .valueColor!
          .value,
      primaryColor,
    );
  });

  testWidgets('RefreshIndicator.color can be updated at runtime', (WidgetTester tester) async {
    refreshCalled = false;
    Color refreshIndicatorColor = Colors.green;
    const Color red = Colors.red;
    late StateSetter setState;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter stateSetter) {
            setState = stateSetter;
            return RefreshIndicator(
              triggerMode: RefreshIndicatorTriggerMode.anywhere,
              onRefresh: holdRefresh,
              color: refreshIndicatorColor,
              child: ListView(
                reverse: true,
                physics: const AlwaysScrollableScrollPhysics(),
                children: const <Widget>[
                  SizedBox(height: 200.0, child: Text('X')),
                  SizedBox(height: 800.0, child: Text('Y')),
                ],
              ),
            );
          },
        ),
      ),
    );

    await tester.fling(find.text('X'), const Offset(0.0, 600.0), 1000.0);
    await tester.pump();
    expect(
      tester
          .widget<RefreshProgressIndicator>(find.byType(RefreshProgressIndicator))
          .valueColor!
          .value,
      refreshIndicatorColor.withOpacity(1.0),
    );

    setState(() {
      refreshIndicatorColor = red;
    });

    await tester.pump();
    expect(
      tester
          .widget<RefreshProgressIndicator>(find.byType(RefreshProgressIndicator))
          .valueColor!
          .value,
      red.withOpacity(1.0),
    );
  });

  testWidgets('RefreshIndicator - reverse - BouncingScrollPhysics', (WidgetTester tester) async {
    refreshCalled = false;
    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            reverse: true,
            physics: const BouncingScrollPhysics(),
            children: <Widget>[
              for (int i = 0; i < 4; i++) SizedBox(height: 200.0, child: Text('X - $i')),
            ],
          ),
        ),
      ),
    );

    // Scroll to top
    await tester.fling(find.text('X - 0'), const Offset(0.0, 800.0), 1000.0);
    await tester.pumpAndSettle();

    // Fling down to show refresh indicator
    await tester.fling(find.text('X - 3'), const Offset(0.0, 250.0), 1000.0);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation
    expect(refreshCalled, true);
  });

  testWidgets('RefreshIndicator disallows indicator - glow', (WidgetTester tester) async {
    refreshCalled = false;
    bool glowAccepted = true;
    ScrollNotification? lastNotification;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: RefreshIndicator(
          onRefresh: refresh,
          child: Builder(
            builder: (BuildContext context) {
              return NotificationListener<ScrollNotification>(
                onNotification: (ScrollNotification notification) {
                  if (notification is OverscrollNotification &&
                      lastNotification is! OverscrollNotification) {
                    final OverscrollIndicatorNotification confirmationNotification =
                        OverscrollIndicatorNotification(leading: true);
                    confirmationNotification.dispatch(context);
                    glowAccepted = confirmationNotification.accepted;
                  }
                  lastNotification = notification;
                  return false;
                },
                child: ListView(
                  physics: const AlwaysScrollableScrollPhysics(),
                  children:
                      <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                        return SizedBox(height: 200.0, child: Text(item));
                      }).toList(),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(find.byType(StretchingOverscrollIndicator), findsNothing);
    expect(find.byType(GlowingOverscrollIndicator), findsOneWidget);

    await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0);
    await tester.pump();

    await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation
    expect(refreshCalled, true);
    expect(glowAccepted, false);
  });

  testWidgets('RefreshIndicator disallows indicator - stretch', (WidgetTester tester) async {
    refreshCalled = false;
    bool stretchAccepted = true;
    ScrollNotification? lastNotification;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light().copyWith(useMaterial3: true),
        home: RefreshIndicator(
          onRefresh: refresh,
          child: Builder(
            builder: (BuildContext context) {
              return NotificationListener<ScrollNotification>(
                onNotification: (ScrollNotification notification) {
                  if (notification is OverscrollNotification &&
                      lastNotification is! OverscrollNotification) {
                    final OverscrollIndicatorNotification confirmationNotification =
                        OverscrollIndicatorNotification(leading: true);
                    confirmationNotification.dispatch(context);
                    stretchAccepted = confirmationNotification.accepted;
                  }
                  lastNotification = notification;
                  return false;
                },
                child: ListView(
                  physics: const AlwaysScrollableScrollPhysics(),
                  children:
                      <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                        return SizedBox(height: 200.0, child: Text(item));
                      }).toList(),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(find.byType(StretchingOverscrollIndicator), findsOneWidget);
    expect(find.byType(GlowingOverscrollIndicator), findsNothing);

    await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0);
    await tester.pump();

    await tester.pump(const Duration(seconds: 1)); // finish the scroll animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator settle animation
    await tester.pump(const Duration(seconds: 1)); // finish the indicator hide animation
    expect(refreshCalled, true);
    expect(stretchAccepted, false);
  });

  group('RefreshIndicator.noSpinner', () {
    testWidgets('onStatusChange and onRefresh Trigger', (WidgetTester tester) async {
      refreshCalled = false;
      bool modeSnap = false;
      bool modeDrag = false;
      bool modeArmed = false;
      bool modeDone = false;

      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator.noSpinner(
            onStatusChange: (RefreshIndicatorStatus? mode) {
              if (mode == RefreshIndicatorStatus.armed) {
                modeArmed = true;
              }
              if (mode == RefreshIndicatorStatus.drag) {
                modeDrag = true;
              }
              if (mode == RefreshIndicatorStatus.snap) {
                modeSnap = true;
              }
              if (mode == RefreshIndicatorStatus.done) {
                modeDone = true;
              }
            },
            onRefresh: refresh,
            child: ListView(
              physics: const AlwaysScrollableScrollPhysics(),
              children:
                  <String>['A', 'B', 'C', 'D', 'E', 'F'].map<Widget>((String item) {
                    return SizedBox(height: 200.0, child: Text(item));
                  }).toList(),
            ),
          ),
        ),
      );

      await tester.fling(find.text('A'), const Offset(0.0, 300.0), 1000.0);
      await tester.pump();

      // Finish the scroll animation.
      await tester.pump(const Duration(seconds: 1));

      // Finish the indicator settle animation.
      await tester.pump(const Duration(seconds: 1));

      // Finish the indicator hide animation.
      await tester.pump(const Duration(seconds: 1));

      expect(refreshCalled, true);
      expect(modeSnap, true);
      expect(modeDrag, true);
      expect(modeArmed, true);
      expect(modeDone, true);
    });
  });

  testWidgets('RefreshIndicator manipulates value color opacity correctly', (
    WidgetTester tester,
  ) async {
    final List<Color> colors = <Color>[
      Colors.black,
      Colors.black54,
      Colors.white,
      Colors.white54,
      Colors.transparent,
    ];
    const List<double> positions = <double>[50.0, 100.0, 150.0];

    Future<void> testColor(Color color) async {
      final AnimationController positionController = AnimationController(vsync: const TestVSync());
      addTearDown(positionController.dispose);
      // Correspond to [_setupColorTween].
      final Animation<Color?> valueColorAnimation = positionController.drive(
        ColorTween(begin: color.withAlpha(0), end: color.withAlpha(color.alpha)).chain(
          CurveTween(
            // Correspond to [_kDragSizeFactorLimit].
            curve: const Interval(0.0, 1.0 / 1.5),
          ),
        ),
      );
      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator(
            onRefresh: refresh,
            color: color,
            child: ListView(
              physics: const AlwaysScrollableScrollPhysics(),
              children: const <Widget>[Text('X')],
            ),
          ),
        ),
      );

      RefreshProgressIndicator getIndicator() {
        return tester.widget<RefreshProgressIndicator>(find.byType(RefreshProgressIndicator));
      }

      // Correspond to [_kDragContainerExtentPercentage].
      final double maxPosition =
          tester.view.physicalSize.height / tester.view.devicePixelRatio * 0.25;
      for (final double position in positions) {
        await tester.fling(find.text('X'), Offset(0.0, position), 1.0);
        await tester.pump();
        positionController.value = position / maxPosition;
        expect(getIndicator().valueColor!.value!.alpha, valueColorAnimation.value!.alpha);
        // Wait until the fling finishes before starting the next fling.
        await tester.pumpAndSettle();
      }
    }

    for (final Color color in colors) {
      await testColor(color);
    }
  });

  testWidgets('RefreshIndicator passes the default elevation through correctly', (
    WidgetTester tester,
  ) async {
    final AnimationController positionController = AnimationController(vsync: const TestVSync());
    addTearDown(positionController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: RefreshIndicator(
          onRefresh: refresh,
          child: ListView(
            physics: const AlwaysScrollableScrollPhysics(),
            children: const <Widget>[Text('X')],
          ),
        ),
      ),
    );

    final double maxPosition =
        tester.view.physicalSize.height / tester.view.devicePixelRatio * 0.25;
    const double position = 50.0;
    await tester.fling(find.text('X'), const Offset(0.0, position), 1.0);
    await tester.pump();
    positionController.value = position / maxPosition;
    expect(
      tester.widget<RefreshProgressIndicator>(find.byType(RefreshProgressIndicator)).elevation,
      2.0,
    );
  });

  testWidgets('RefreshIndicator passes custom elevation values through correctly', (
    WidgetTester tester,
  ) async {
    for (final double elevation in <double>[0.0, 2.0]) {
      final AnimationController positionController = AnimationController(vsync: const TestVSync());
      addTearDown(positionController.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: RefreshIndicator(
            elevation: elevation,
            onRefresh: refresh,
            child: ListView(
              physics: const AlwaysScrollableScrollPhysics(),
              children: const <Widget>[Text('X')],
            ),
          ),
        ),
      );

      final double maxPosition =
          tester.view.physicalSize.height / tester.view.devicePixelRatio * 0.25;
      const double position = 50.0;
      await tester.fling(find.text('X'), const Offset(0.0, position), 1.0);
      await tester.pump();
      positionController.value = position / maxPosition;
      expect(
        tester.widget<RefreshProgressIndicator>(find.byType(RefreshProgressIndicator)).elevation,
        elevation,
      );
      await tester.pumpAndSettle();
    }
  });
}
