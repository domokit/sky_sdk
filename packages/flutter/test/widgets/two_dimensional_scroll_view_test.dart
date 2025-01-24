// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/src/gestures/monodrag.dart';
import 'package:flutter_test/flutter_test.dart';

import 'two_dimensional_utils.dart';

Widget? _testChildBuilder(BuildContext context, ChildVicinity vicinity) {
  return SizedBox(
    height: 200,
    width: 200,
    child: Center(child: Text('C${vicinity.xIndex}:R${vicinity.yIndex}')),
  );
}

void main() {
  group('TwoDimensionalScrollView', () {
    testWidgets(
      'asserts the axis directions do not conflict with one another',
      (WidgetTester tester) async {
        final List<Object> exceptions = <Object>[];
        final FlutterExceptionHandler? oldHandler = FlutterError.onError;
        FlutterError.onError = (FlutterErrorDetails details) {
          exceptions.add(details.exception);
        };
        // Horizontal wrong
        late final TwoDimensionalChildBuilderDelegate delegate1;
        addTearDown(() => delegate1.dispose());
        await tester.pumpWidget(
          MaterialApp(
            home: SimpleBuilderTableView(
              delegate: delegate1 = TwoDimensionalChildBuilderDelegate(builder: (_, _) => null),
              horizontalDetails: const ScrollableDetails.vertical(),
              // Horizontal has default const ScrollableDetails.horizontal()
            ),
          ),
        );

        // Vertical wrong
        late final TwoDimensionalChildBuilderDelegate delegate2;
        addTearDown(() => delegate2.dispose());
        await tester.pumpWidget(
          MaterialApp(
            home: SimpleBuilderTableView(
              delegate: delegate2 = TwoDimensionalChildBuilderDelegate(builder: (_, _) => null),
              verticalDetails: const ScrollableDetails.horizontal(),
              // Horizontal has default const ScrollableDetails.horizontal()
            ),
          ),
        );

        // Both wrong
        late final TwoDimensionalChildBuilderDelegate delegate3;
        addTearDown(() => delegate3.dispose());
        await tester.pumpWidget(
          MaterialApp(
            home: SimpleBuilderTableView(
              delegate: delegate3 = TwoDimensionalChildBuilderDelegate(builder: (_, _) => null),
              verticalDetails: const ScrollableDetails.horizontal(),
              horizontalDetails: const ScrollableDetails.vertical(),
            ),
          ),
        );

        FlutterError.onError = oldHandler;
        expect(exceptions.length, 3);
        for (final Object exception in exceptions) {
          expect(exception, isAssertionError);
          expect((exception as AssertionError).message, contains('are not Axis'));
        }
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets(
      'ScrollableDetails.controller can set initial scroll positions, modify within bounds',
      (WidgetTester tester) async {
        final ScrollController verticalController = ScrollController(initialScrollOffset: 100);
        addTearDown(verticalController.dispose);
        final ScrollController horizontalController = ScrollController(initialScrollOffset: 50);
        addTearDown(horizontalController.dispose);
        late final TwoDimensionalChildBuilderDelegate delegate;
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(
          MaterialApp(
            home: SimpleBuilderTableView(
              verticalDetails: ScrollableDetails.vertical(controller: verticalController),
              horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
              delegate:
                  delegate = TwoDimensionalChildBuilderDelegate(
                    builder: _testChildBuilder,
                    maxXIndex: 99,
                    maxYIndex: 99,
                  ),
            ),
          ),
        );
        await tester.pumpAndSettle();

        expect(verticalController.position.pixels, 100);
        expect(verticalController.position.maxScrollExtent, 19400);
        expect(horizontalController.position.pixels, 50);
        expect(horizontalController.position.maxScrollExtent, 19200);

        verticalController.jumpTo(verticalController.position.maxScrollExtent);
        horizontalController.jumpTo(horizontalController.position.maxScrollExtent);
        await tester.pump();

        expect(verticalController.position.pixels, 19400);
        expect(horizontalController.position.pixels, 19200);

        // Out of bounds
        verticalController.jumpTo(verticalController.position.maxScrollExtent + 100);
        horizontalController.jumpTo(horizontalController.position.maxScrollExtent + 100);
        // Account for varying scroll physics for different platforms (overscroll)
        await tester.pumpAndSettle();

        expect(verticalController.position.pixels, 19400);
        expect(horizontalController.position.pixels, 19200);
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets(
      'Properly assigns the PrimaryScrollController to the main axis on the correct platform',
      (WidgetTester tester) async {
        late ScrollController controller;
        Widget buildForPrimaryScrollController({
          bool? explicitPrimary,
          Axis mainAxis = Axis.vertical,
          bool addControllerConflict = false,
        }) {
          final ScrollController verticalController = ScrollController();
          addTearDown(verticalController.dispose);
          final ScrollController horizontalController = ScrollController();
          addTearDown(horizontalController.dispose);
          late final TwoDimensionalChildBuilderDelegate delegate;
          addTearDown(() => delegate.dispose());

          return MaterialApp(
            home: PrimaryScrollController(
              controller: controller,
              child: SimpleBuilderTableView(
                mainAxis: mainAxis,
                primary: explicitPrimary,
                verticalDetails: ScrollableDetails.vertical(
                  controller:
                      addControllerConflict && mainAxis == Axis.vertical
                          ? verticalController
                          : null,
                ),
                horizontalDetails: ScrollableDetails.horizontal(
                  controller:
                      addControllerConflict && mainAxis == Axis.horizontal
                          ? horizontalController
                          : null,
                ),
                delegate:
                    delegate = TwoDimensionalChildBuilderDelegate(
                      builder: _testChildBuilder,
                      maxXIndex: 99,
                      maxYIndex: 99,
                    ),
              ),
            ),
          );
        }

        // Horizontal default - horizontal never automatically adopts PSC
        controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(buildForPrimaryScrollController(mainAxis: Axis.horizontal));
        await tester.pumpAndSettle();

        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            expect(controller.hasClients, isFalse);
        }

        // Horizontal explicitly true
        controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          buildForPrimaryScrollController(mainAxis: Axis.horizontal, explicitPrimary: true),
        );
        await tester.pumpAndSettle();

        switch (defaultTargetPlatform) {
          // Primary explicitly true is always adopted.
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            expect(controller.hasClients, isTrue);
            expect(controller.position.axis, Axis.horizontal);
        }

        // Horizontal explicitly false
        controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          buildForPrimaryScrollController(mainAxis: Axis.horizontal, explicitPrimary: false),
        );
        await tester.pumpAndSettle();

        switch (defaultTargetPlatform) {
          // Primary explicitly false is never adopted.
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            expect(controller.hasClients, isFalse);
        }

        // Vertical default
        controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(buildForPrimaryScrollController());
        await tester.pumpAndSettle();

        switch (defaultTargetPlatform) {
          // Mobile platforms inherit the PSC without explicitly setting
          // primary
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
            expect(controller.hasClients, isTrue);
            expect(controller.position.axis, Axis.vertical);
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            expect(controller.hasClients, isFalse);
        }

        // Vertical explicitly true
        controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(buildForPrimaryScrollController(explicitPrimary: true));
        await tester.pumpAndSettle();

        switch (defaultTargetPlatform) {
          // Primary explicitly true is always adopted.
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            expect(controller.hasClients, isTrue);
            expect(controller.position.axis, Axis.vertical);
        }

        // Vertical explicitly false
        controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(buildForPrimaryScrollController(explicitPrimary: false));
        await tester.pumpAndSettle();

        switch (defaultTargetPlatform) {
          // Primary explicitly false is never adopted.
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            expect(controller.hasClients, isFalse);
        }

        // Assertions
        final List<Object> exceptions = <Object>[];
        final FlutterExceptionHandler? oldHandler = FlutterError.onError;
        FlutterError.onError = (FlutterErrorDetails details) {
          exceptions.add(details.exception);
        };
        // Vertical asserts ScrollableDetails.controller has not been provided if
        // primary is explicitly set
        controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          buildForPrimaryScrollController(explicitPrimary: true, addControllerConflict: true),
        );
        expect(exceptions.length, 1);
        expect(exceptions[0], isAssertionError);
        expect(
          (exceptions[0] as AssertionError).message,
          contains('TwoDimensionalScrollView.primary was explicitly set to true'),
        );
        exceptions.clear();

        // Horizontal asserts ScrollableDetails.controller has not been provided
        // if primary is explicitly set true
        controller = ScrollController();
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          buildForPrimaryScrollController(
            mainAxis: Axis.horizontal,
            explicitPrimary: true,
            addControllerConflict: true,
          ),
        );
        expect(exceptions.length, 1);
        expect(exceptions[0], isAssertionError);
        expect(
          (exceptions[0] as AssertionError).message,
          contains('TwoDimensionalScrollView.primary was explicitly set to true'),
        );
        FlutterError.onError = oldHandler;
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets(
      'TwoDimensionalScrollable receives the correct details from TwoDimensionalScrollView',
      (WidgetTester tester) async {
        late BuildContext capturedContext;
        // Default
        late final TwoDimensionalChildBuilderDelegate delegate1;
        addTearDown(() => delegate1.dispose());
        await tester.pumpWidget(
          MaterialApp(
            home: SimpleBuilderTableView(
              delegate:
                  delegate1 = TwoDimensionalChildBuilderDelegate(
                    builder: (BuildContext context, ChildVicinity vicinity) {
                      capturedContext = context;
                      return Text(vicinity.toString());
                    },
                  ),
            ),
          ),
        );
        await tester.pumpAndSettle();
        TwoDimensionalScrollableState scrollable = TwoDimensionalScrollable.of(capturedContext);
        expect(scrollable.widget.verticalDetails.direction, AxisDirection.down);
        expect(scrollable.widget.horizontalDetails.direction, AxisDirection.right);
        expect(scrollable.widget.diagonalDragBehavior, DiagonalDragBehavior.none);
        expect(scrollable.widget.dragStartBehavior, DragStartBehavior.start);

        // Customized
        late final TwoDimensionalChildBuilderDelegate delegate2;
        addTearDown(() => delegate2.dispose());
        await tester.pumpWidget(
          MaterialApp(
            home: SimpleBuilderTableView(
              verticalDetails: const ScrollableDetails.vertical(reverse: true),
              horizontalDetails: const ScrollableDetails.horizontal(reverse: true),
              diagonalDragBehavior: DiagonalDragBehavior.weightedContinuous,
              dragStartBehavior: DragStartBehavior.down,
              delegate: delegate2 = TwoDimensionalChildBuilderDelegate(builder: _testChildBuilder),
            ),
          ),
        );
        await tester.pumpAndSettle();
        scrollable = TwoDimensionalScrollable.of(capturedContext);
        expect(scrollable.widget.verticalDetails.direction, AxisDirection.up);
        expect(scrollable.widget.horizontalDetails.direction, AxisDirection.left);
        expect(scrollable.widget.diagonalDragBehavior, DiagonalDragBehavior.weightedContinuous);
        expect(scrollable.widget.dragStartBehavior, DragStartBehavior.down);
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets(
      'TwoDimensionalScrollable with hitTestBehavior.translucent lets widgets underneath catch the hit',
      (WidgetTester tester) async {
        bool tapped = false;
        final Key key = UniqueKey();
        late final TwoDimensionalChildBuilderDelegate delegate;
        addTearDown(() => delegate.dispose());
        await tester.pumpWidget(
          MaterialApp(
            home: Stack(
              children: <Widget>[
                Positioned.fill(
                  child: GestureDetector(
                    behavior: HitTestBehavior.opaque,
                    onTap: () => tapped = true,
                    child: SizedBox(key: key, height: 300),
                  ),
                ),
                SimpleBuilderTableView(
                  hitTestBehavior: HitTestBehavior.translucent,
                  delegate:
                      delegate = TwoDimensionalChildBuilderDelegate(
                        builder: (BuildContext context, ChildVicinity vicinity) {
                          return const SizedBox(width: 50, height: 50);
                        },
                      ),
                ),
              ],
            ),
          ),
        );
        await tester.pumpAndSettle();
        await tester.tapAt(tester.getCenter(find.byKey(key)));
        expect(tapped, isTrue);
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets('Interrupt fling with tap stops scrolling', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/133529
      final List<String> log = <String>[];
      final ScrollController verticalController = ScrollController();
      addTearDown(verticalController.dispose);
      final ScrollController horizontalController = ScrollController();
      addTearDown(horizontalController.dispose);
      late final TwoDimensionalChildBuilderDelegate delegate;
      addTearDown(() => delegate.dispose());

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: SimpleBuilderTableView(
            verticalDetails: ScrollableDetails.vertical(controller: verticalController),
            horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
            diagonalDragBehavior: DiagonalDragBehavior.free,
            delegate:
                delegate = TwoDimensionalChildBuilderDelegate(
                  maxXIndex: 100,
                  maxYIndex: 100,
                  builder: (BuildContext context, ChildVicinity vicinity) {
                    return GestureDetector(
                      onTapUp: (TapUpDetails details) {
                        log.add('Tapped: $vicinity');
                      },
                      child: Text('$vicinity'),
                    );
                  },
                ),
          ),
        ),
      );

      await tester.pumpAndSettle();
      expect(log, equals(<String>[]));
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Tap once
      await tester.tap(find.byType(TwoDimensionalScrollable));
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Fling the scrollview to get it scrolling, verify that no tap occurs.
      await tester.fling(find.byType(TwoDimensionalScrollable), const Offset(0.0, -200.0), 2000.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(verticalController.position.pixels, greaterThan(170.0));
      double unchangedOffset = verticalController.position.pixels;
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity!.isScrolling, isTrue);
      expect(horizontalController.position.activity!.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, greaterThan(1500));
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Tap to stop the scroll movement, this should stop the fling but not tap anything
      await tester.tap(find.byType(TwoDimensionalScrollable));
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(verticalController.position.pixels, unchangedOffset);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Another tap.
      await tester.tap(find.byType(TwoDimensionalScrollable));
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, <String>['Tapped: (xIndex: 0, yIndex: 0)', 'Tapped: (xIndex: 0, yIndex: 0)']);
      expect(verticalController.position.pixels, unchangedOffset);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      log.clear();
      verticalController.jumpTo(0.0);
      await tester.pump();
      // Fling off in the other direction now ----------------------------------
      expect(log, equals(<String>[]));
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Tap once
      await tester.tap(find.byType(TwoDimensionalScrollable));
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Fling the scrollview to get it scrolling, verify that no tap occurs.
      await tester.fling(find.byType(TwoDimensionalScrollable), const Offset(-200.0, 0.0), 2000.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(horizontalController.position.pixels, greaterThan(170.0));
      unchangedOffset = horizontalController.position.pixels;
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.activity!.isScrolling, isTrue);
      expect(verticalController.position.activity!.isScrolling, isFalse);
      expect(horizontalController.position.activity!.velocity, greaterThan(1500));
      expect(verticalController.position.activity!.velocity, 0.0);

      // Tap to stop the scroll movement, this should stop the fling but not tap anything
      await tester.tap(find.byType(TwoDimensionalScrollable));
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(horizontalController.position.pixels, unchangedOffset);
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity!.velocity, 0.0);
      expect(verticalController.position.activity!.velocity, 0.0);

      // Another tap.
      await tester.tap(find.byType(TwoDimensionalScrollable));
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, <String>['Tapped: (xIndex: 0, yIndex: 0)', 'Tapped: (xIndex: 0, yIndex: 0)']);
      expect(horizontalController.position.pixels, unchangedOffset);
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity!.velocity, 0.0);
      expect(verticalController.position.activity!.velocity, 0.0);
    }, variant: TargetPlatformVariant.all());

    testWidgets('Fling, wait to stop and tap', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/133529
      final List<String> log = <String>[];
      final ScrollController verticalController = ScrollController();
      addTearDown(verticalController.dispose);
      final ScrollController horizontalController = ScrollController();
      addTearDown(horizontalController.dispose);
      late final TwoDimensionalChildBuilderDelegate delegate;
      addTearDown(() => delegate.dispose());

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: SimpleBuilderTableView(
            verticalDetails: ScrollableDetails.vertical(controller: verticalController),
            horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
            diagonalDragBehavior: DiagonalDragBehavior.free,
            delegate:
                delegate = TwoDimensionalChildBuilderDelegate(
                  maxXIndex: 100,
                  maxYIndex: 100,
                  builder: (BuildContext context, ChildVicinity vicinity) {
                    return GestureDetector(
                      onTapUp: (TapUpDetails details) {
                        log.add('Tapped: $vicinity');
                      },
                      child: Text('$vicinity'),
                    );
                  },
                ),
          ),
        ),
      );

      await tester.pumpAndSettle();
      expect(log, equals(<String>[]));
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Tap once
      await tester.tap(find.byType(TwoDimensionalScrollable));
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(verticalController.position.pixels, 0.0);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Fling the scrollview to get it scrolling, verify that no tap occurs.
      await tester.fling(find.byType(TwoDimensionalScrollable), const Offset(0.0, -200.0), 2000.0);
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(verticalController.position.pixels, greaterThan(170.0));
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity!.isScrolling, isTrue);
      expect(horizontalController.position.activity!.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, greaterThan(1500));
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Wait for the fling to finish.
      await tester.pumpAndSettle();
      expect(log, equals(<String>['Tapped: (xIndex: 0, yIndex: 0)']));
      expect(verticalController.position.pixels, greaterThan(800.0));
      final double unchangedOffset = verticalController.position.pixels;
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity!.velocity, 0.0);
      expect(horizontalController.position.activity!.velocity, 0.0);

      // Another tap.
      await tester.tap(find.byType(TwoDimensionalScrollable));
      await tester.pump(const Duration(milliseconds: 50));
      expect(log, <String>['Tapped: (xIndex: 0, yIndex: 0)', 'Tapped: (xIndex: 0, yIndex: 4)']);
      expect(horizontalController.position.pixels, 0.0);
      expect(verticalController.position.pixels, unchangedOffset);
      expect(horizontalController.position.activity?.isScrolling, isFalse);
      expect(verticalController.position.activity?.isScrolling, isFalse);
      expect(horizontalController.position.activity!.velocity, 0.0);
      expect(verticalController.position.activity!.velocity, 0.0);
    });

    group('Can drag horizontally when there is not enough vertical content', () {
      testWidgets('DiagonalDragBehavior.free', (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/144982
        final ScrollController verticalController = ScrollController();
        addTearDown(verticalController.dispose);
        final ScrollController horizontalController = ScrollController();
        addTearDown(horizontalController.dispose);
        late final TwoDimensionalChildBuilderDelegate delegate;
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: SimpleBuilderTableView(
              verticalDetails: ScrollableDetails.vertical(controller: verticalController),
              horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
              diagonalDragBehavior: DiagonalDragBehavior.free,
              delegate:
                  delegate = TwoDimensionalChildBuilderDelegate(
                    maxXIndex: 20,
                    maxYIndex: 1,
                    builder: _testChildBuilder,
                  ),
            ),
          ),
        );

        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        expect(verticalController.position.maxScrollExtent, 0.0);
        expect(horizontalController.position.maxScrollExtent, 3400.0);
        // Fling vertically, nothing should happen.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(0.0, -200.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        // Fling horizontally, the horizontal position should change.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(-200.0, 0.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, greaterThan(840.0));
      });

      testWidgets('DiagonalDragBehavior.weightedEvent', (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/144982
        final ScrollController verticalController = ScrollController();
        addTearDown(verticalController.dispose);
        final ScrollController horizontalController = ScrollController();
        addTearDown(horizontalController.dispose);
        late final TwoDimensionalChildBuilderDelegate delegate;
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: SimpleBuilderTableView(
              verticalDetails: ScrollableDetails.vertical(controller: verticalController),
              horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
              diagonalDragBehavior: DiagonalDragBehavior.weightedEvent,
              delegate:
                  delegate = TwoDimensionalChildBuilderDelegate(
                    maxXIndex: 20,
                    maxYIndex: 1,
                    builder: _testChildBuilder,
                  ),
            ),
          ),
        );

        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        expect(verticalController.position.maxScrollExtent, 0.0);
        expect(horizontalController.position.maxScrollExtent, 3400.0);
        // Fling vertically, nothing should happen.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(0.0, -200.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        // Fling horizontally, the horizontal position should change.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(-200.0, 0.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, greaterThan(840.0));
      });

      testWidgets('DiagonalDragBehavior.weightedContinuous', (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/144982
        final ScrollController verticalController = ScrollController();
        addTearDown(verticalController.dispose);
        final ScrollController horizontalController = ScrollController();
        addTearDown(horizontalController.dispose);
        late final TwoDimensionalChildBuilderDelegate delegate;
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: SimpleBuilderTableView(
              verticalDetails: ScrollableDetails.vertical(controller: verticalController),
              horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
              diagonalDragBehavior: DiagonalDragBehavior.weightedContinuous,
              delegate:
                  delegate = TwoDimensionalChildBuilderDelegate(
                    maxXIndex: 20,
                    maxYIndex: 1,
                    builder: _testChildBuilder,
                  ),
            ),
          ),
        );

        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        expect(verticalController.position.maxScrollExtent, 0.0);
        expect(horizontalController.position.maxScrollExtent, 3400.0);
        // Fling vertically, nothing should happen.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(0.0, -200.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        // Fling horizontally, the horizontal position should change.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(-200.0, 0.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, greaterThan(840.0));
      });
    });

    group('Can drag vertically when there is not enough horizontal content', () {
      testWidgets('DiagonalDragBehavior.free', (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/144982
        final ScrollController verticalController = ScrollController();
        addTearDown(verticalController.dispose);
        final ScrollController horizontalController = ScrollController();
        addTearDown(horizontalController.dispose);
        late final TwoDimensionalChildBuilderDelegate delegate;
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: SimpleBuilderTableView(
              verticalDetails: ScrollableDetails.vertical(controller: verticalController),
              horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
              diagonalDragBehavior: DiagonalDragBehavior.free,
              delegate:
                  delegate = TwoDimensionalChildBuilderDelegate(
                    maxXIndex: 1,
                    maxYIndex: 20,
                    builder: _testChildBuilder,
                  ),
            ),
          ),
        );

        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        expect(verticalController.position.maxScrollExtent, 3600.0);
        expect(horizontalController.position.maxScrollExtent, 0.0);
        // Fling horizontally, nothing should happen.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(-200.0, 0.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        // Fling vertically, the vertical position should change.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(0.0, -200.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, greaterThan(840.0));
        expect(horizontalController.position.pixels, 0.0);
      });

      testWidgets('DiagonalDragBehavior.weightedEvent', (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/144982
        final ScrollController verticalController = ScrollController();
        addTearDown(verticalController.dispose);
        final ScrollController horizontalController = ScrollController();
        addTearDown(horizontalController.dispose);
        late final TwoDimensionalChildBuilderDelegate delegate;
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: SimpleBuilderTableView(
              verticalDetails: ScrollableDetails.vertical(controller: verticalController),
              horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
              diagonalDragBehavior: DiagonalDragBehavior.weightedEvent,
              delegate:
                  delegate = TwoDimensionalChildBuilderDelegate(
                    maxXIndex: 1,
                    maxYIndex: 20,
                    builder: _testChildBuilder,
                  ),
            ),
          ),
        );

        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        expect(verticalController.position.maxScrollExtent, 3600.0);
        expect(horizontalController.position.maxScrollExtent, 0.0);
        // Fling horizontally, nothing should happen.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(-200.0, 0.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        // Fling vertically, the vertical position should change.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(0.0, -200.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, greaterThan(840.0));
        expect(horizontalController.position.pixels, 0.0);
      });

      testWidgets('DiagonalDragBehavior.weightedContinuous', (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/144982
        final ScrollController verticalController = ScrollController();
        addTearDown(verticalController.dispose);
        final ScrollController horizontalController = ScrollController();
        addTearDown(horizontalController.dispose);
        late final TwoDimensionalChildBuilderDelegate delegate;
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: SimpleBuilderTableView(
              verticalDetails: ScrollableDetails.vertical(controller: verticalController),
              horizontalDetails: ScrollableDetails.horizontal(controller: horizontalController),
              diagonalDragBehavior: DiagonalDragBehavior.weightedContinuous,
              delegate:
                  delegate = TwoDimensionalChildBuilderDelegate(
                    maxXIndex: 1,
                    maxYIndex: 20,
                    builder: _testChildBuilder,
                  ),
            ),
          ),
        );

        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        expect(verticalController.position.maxScrollExtent, 3600.0);
        expect(horizontalController.position.maxScrollExtent, 0.0);
        // Fling horizontally, nothing should happen.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(-200.0, 0.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, 0.0);
        expect(horizontalController.position.pixels, 0.0);
        // Fling vertically, the vertical position should change.
        await tester.fling(
          find.byType(TwoDimensionalScrollable),
          const Offset(0.0, -200.0),
          2000.0,
        );
        await tester.pumpAndSettle();
        expect(verticalController.position.pixels, greaterThan(840.0));
        expect(horizontalController.position.pixels, 0.0);
      });
    });

    testWidgets('Dismiss keyboard onDrag and keep dismissed on drawer opened', (
      WidgetTester tester,
    ) async {
      late final TwoDimensionalChildBuilderDelegate delegate;
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      addTearDown(() => delegate.dispose());

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            key: scaffoldKey,
            drawer: Container(),
            body: Column(
              children: <Widget>[
                const TextField(),
                Expanded(
                  child: SimpleBuilderTableView(
                    keyboardDismissBehavior: ScrollViewKeyboardDismissBehavior.onDrag,
                    delegate:
                        delegate = TwoDimensionalChildBuilderDelegate(
                          builder: _testChildBuilder,
                          maxXIndex: 99,
                          maxYIndex: 99,
                        ),
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      await tester.pumpAndSettle();

      expect(tester.testTextInput.isVisible, isFalse);
      final Finder finder = find.byType(TextField).first;
      await tester.tap(finder);
      expect(tester.testTextInput.isVisible, isTrue);

      await tester.drag(find.byType(SimpleBuilderTableView).first, const Offset(-40.0, -40.0));
      await tester.pumpAndSettle();

      expect(tester.testTextInput.isVisible, isFalse);
      scaffoldKey.currentState!.openDrawer();
      await tester.pumpAndSettle();

      expect(tester.testTextInput.isVisible, isFalse);
    });
  });
}
