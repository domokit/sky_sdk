// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['reduced-test-set'])
library;

import 'package:fake_async/fake_async.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class _MockAnimationController extends AnimationController {
  _MockAnimationController()
    : super(duration: const Duration(minutes: 1), vsync: const TestVSync());
  int forwardCalls = 0;
  int reverseCalls = 0;

  @override
  TickerFuture forward({double? from}) {
    forwardCalls++;
    return super.forward(from: from);
  }

  @override
  TickerFuture reverse({double? from}) {
    reverseCalls++;
    return super.reverse(from: from);
  }
}

void main() {
  Future<T> runFakeAsync<T>(Future<T> Function(FakeAsync time) f) async {
    return FakeAsync().run((FakeAsync time) async {
      bool pump = true;
      final Future<T> future = f(time).whenComplete(() => pump = false);
      while (pump) {
        time.flushMicrotasks();
      }
      return future;
    });
  }

  group('Raw Magnifier', () {
    testWidgets('should render with correct focal point and decoration', (
      WidgetTester tester,
    ) async {
      final Key appKey = UniqueKey();
      const Size magnifierSize = Size(100, 100);
      const Offset magnifierFocalPoint = Offset(50, 50);
      const Offset magnifierPosition = Offset(200, 200);
      const double magnificationScale = 2;

      await tester.pumpWidget(
        MaterialApp(
          key: appKey,
          home: Container(
            color: Colors.blue,
            width: double.infinity,
            height: double.infinity,
            child: Stack(
              children: <Widget>[
                Positioned(
                  // Positioned so that it is right in the center of the magnifier
                  // focal point.
                  left: magnifierPosition.dx + magnifierFocalPoint.dx,
                  top: magnifierPosition.dy + magnifierFocalPoint.dy,
                  child: Container(
                    color: Colors.black,
                    // Since it is the size of the magnifier but over its
                    // magnificationScale, it should take up the whole magnifier.
                    width: (magnifierSize.width * 1.5) / magnificationScale,
                    height: (magnifierSize.height * 1.5) / magnificationScale,
                  ),
                ),
                Positioned(
                  left: magnifierPosition.dx,
                  top: magnifierPosition.dy,
                  child: const RawMagnifier(
                    size: magnifierSize,
                    focalPointOffset: magnifierFocalPoint,
                    magnificationScale: magnificationScale,
                    clipBehavior: Clip.hardEdge,
                    decoration: MagnifierDecoration(
                      shadows: <BoxShadow>[
                        BoxShadow(
                          spreadRadius: 10.0,
                          blurRadius: 10.0,
                          color: Colors.yellow,
                          offset: Offset(5.0, 5.0),
                        ),
                      ],
                      opacity: 0.5,
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      // Should look like a blue screen, with two black boxes. The larger black
      // box is in the magnifier, is outlined in yellow, and is doubled in size
      // (from magnification). The magnifier should be slightly transparent.
      await expectLater(find.byKey(appKey), matchesGoldenFile('widgets.magnifier.styled.png'));
    }, skip: kIsWeb); // [intended] Bdf does not display on web.

    group('transition states', () {
      final AnimationController animationController = AnimationController(
        vsync: const TestVSync(),
        duration: const Duration(minutes: 2),
      );
      final MagnifierController magnifierController = MagnifierController();

      tearDown(() {
        animationController.value = 0;
        magnifierController.hide();

        magnifierController.removeFromOverlay();
      });

      testWidgets('should immediately remove from overlay on no animation controller', (
        WidgetTester tester,
      ) async {
        await runFakeAsync((FakeAsync async) async {
          const RawMagnifier testMagnifier = RawMagnifier(size: Size(100, 100));

          await tester.pumpWidget(const MaterialApp(home: Placeholder()));

          final BuildContext context = tester.firstElement(find.byType(Placeholder));

          magnifierController.show(
            context: context,
            builder: (BuildContext context) => testMagnifier,
          );

          WidgetsBinding.instance.scheduleFrame();
          await tester.pump();

          expect(magnifierController.overlayEntry, isNot(isNull));

          magnifierController.hide();
          WidgetsBinding.instance.scheduleFrame();
          await tester.pump();

          expect(magnifierController.overlayEntry, isNull);
        });
      });

      testWidgets('should update shown based on animation status', (WidgetTester tester) async {
        await runFakeAsync((FakeAsync async) async {
          final MagnifierController magnifierController = MagnifierController(
            animationController: animationController,
          );

          const RawMagnifier testMagnifier = RawMagnifier(size: Size(100, 100));

          await tester.pumpWidget(const MaterialApp(home: Placeholder()));

          final BuildContext context = tester.firstElement(find.byType(Placeholder));

          magnifierController.show(
            context: context,
            builder: (BuildContext context) => testMagnifier,
          );

          WidgetsBinding.instance.scheduleFrame();
          await tester.pump();

          // No time has passed, so the animation controller has not completed.
          expect(magnifierController.animationController?.status, AnimationStatus.forward);
          expect(magnifierController.shown, true);

          async.elapse(animationController.duration!);
          await tester.pumpAndSettle();

          expect(magnifierController.animationController?.status, AnimationStatus.completed);
          expect(magnifierController.shown, true);

          magnifierController.hide();

          WidgetsBinding.instance.scheduleFrame();
          await tester.pump();

          expect(magnifierController.animationController?.status, AnimationStatus.reverse);
          expect(magnifierController.shown, false);

          async.elapse(animationController.duration!);
          await tester.pumpAndSettle();

          expect(magnifierController.animationController?.status, AnimationStatus.dismissed);
          expect(magnifierController.shown, false);
        });
      });
    });
  });

  group('magnifier controller', () {
    final MagnifierController magnifierController = MagnifierController();

    tearDown(() {
      magnifierController.removeFromOverlay();
    });

    group('show', () {
      testWidgets('should insert below below widget', (WidgetTester tester) async {
        await tester.pumpWidget(const MaterialApp(home: Text('text')));

        final BuildContext context = tester.firstElement(find.byType(Text));

        final Widget fakeMagnifier = Placeholder(key: UniqueKey());
        final Widget fakeBefore = Placeholder(key: UniqueKey());

        final OverlayEntry fakeBeforeOverlayEntry = OverlayEntry(builder: (_) => fakeBefore);
        addTearDown(
          () =>
              fakeBeforeOverlayEntry
                ..remove()
                ..dispose(),
        );

        Overlay.of(context).insert(fakeBeforeOverlayEntry);
        magnifierController.show(
          context: context,
          builder: (_) => fakeMagnifier,
          below: fakeBeforeOverlayEntry,
        );

        WidgetsBinding.instance.scheduleFrame();
        await tester.pumpAndSettle();

        final Iterable<Element> allOverlayChildren =
            find
                .descendant(of: find.byType(Overlay), matching: find.byType(Placeholder))
                .evaluate();

        // Expect the magnifier to be the first child, even though it was inserted
        // after the fakeBefore.
        expect(allOverlayChildren.last.widget.key, fakeBefore.key);
        expect(allOverlayChildren.first.widget.key, fakeMagnifier.key);
      });

      testWidgets('should insert newly built widget without animating out if overlay != null', (
        WidgetTester tester,
      ) async {
        await runFakeAsync((FakeAsync async) async {
          final _MockAnimationController animationController = _MockAnimationController();
          addTearDown(animationController.dispose);

          const RawMagnifier testMagnifier = RawMagnifier(size: Size(100, 100));
          const RawMagnifier testMagnifier2 = RawMagnifier(size: Size(100, 100));

          await tester.pumpWidget(const MaterialApp(home: Placeholder()));

          final BuildContext context = tester.firstElement(find.byType(Placeholder));

          magnifierController.show(
            context: context,
            builder: (BuildContext context) => testMagnifier,
          );

          WidgetsBinding.instance.scheduleFrame();
          await tester.pump();

          async.elapse(animationController.duration!);
          await tester.pumpAndSettle();

          magnifierController.show(context: context, builder: (_) => testMagnifier2);

          WidgetsBinding.instance.scheduleFrame();
          await tester.pump();

          expect(
            animationController.reverseCalls,
            0,
            reason: 'should not have called reverse on animation controller due to force remove',
          );

          expect(find.byWidget(testMagnifier2), findsOneWidget);
        });
      });
    });

    group('shift within bounds', () {
      final List<Rect> boundsRects = <Rect>[
        const Rect.fromLTRB(0, 0, 100, 100),
        const Rect.fromLTRB(0, 0, 100, 100),
        const Rect.fromLTRB(0, 0, 100, 100),
        const Rect.fromLTRB(0, 0, 100, 100),
      ];
      final List<Rect> inputRects = <Rect>[
        const Rect.fromLTRB(-100, -100, -80, -80),
        const Rect.fromLTRB(0, 0, 20, 20),
        const Rect.fromLTRB(110, 0, 120, 10),
        const Rect.fromLTRB(110, 110, 120, 120),
      ];
      final List<Rect> outputRects = <Rect>[
        const Rect.fromLTRB(0, 0, 20, 20),
        const Rect.fromLTRB(0, 0, 20, 20),
        const Rect.fromLTRB(90, 0, 100, 10),
        const Rect.fromLTRB(90, 90, 100, 100),
      ];

      for (int i = 0; i < boundsRects.length; i++) {
        test('should shift ${inputRects[i]} to ${outputRects[i]} for bounds ${boundsRects[i]}', () {
          final Rect outputRect = MagnifierController.shiftWithinBounds(
            bounds: boundsRects[i],
            rect: inputRects[i],
          );
          expect(outputRect, outputRects[i]);
        });
      }
    });
  });

  testWidgets('MagnifierInfo.toString', (WidgetTester tester) async {
    expect(
      MagnifierInfo.empty.toString(),
      'MagnifierInfo(position: Offset(0.0, 0.0), line: Rect.fromLTRB(0.0, 0.0, 0.0, 0.0), '
      'caret: Rect.fromLTRB(0.0, 0.0, 0.0, 0.0), field: Rect.fromLTRB(0.0, 0.0, 0.0, 0.0))',
    );
  });
}
