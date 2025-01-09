// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('Basic floating action button locations', () {
    testWidgets('still animates motion when the floating action button is null', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(_buildFrame(fab: null));

      expect(find.byType(FloatingActionButton), findsNothing);
      expect(tester.binding.transientCallbackCount, 0);

      await tester.pumpWidget(
        _buildFrame(fab: null, location: FloatingActionButtonLocation.endFloat),
      );

      expect(find.byType(FloatingActionButton), findsNothing);
      expect(tester.binding.transientCallbackCount, greaterThan(0));

      await tester.pumpWidget(
        _buildFrame(fab: null, location: FloatingActionButtonLocation.centerFloat),
      );

      expect(find.byType(FloatingActionButton), findsNothing);
      expect(tester.binding.transientCallbackCount, greaterThan(0));
    });

    testWidgets('moves fab from center to end and back', (WidgetTester tester) async {
      await tester.pumpWidget(_buildFrame(location: FloatingActionButtonLocation.endFloat));

      expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(756.0, 356.0));
      expect(tester.binding.transientCallbackCount, 0);

      await tester.pumpWidget(_buildFrame(location: FloatingActionButtonLocation.centerFloat));

      expect(tester.binding.transientCallbackCount, greaterThan(0));

      await tester.pumpAndSettle();

      expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(400.0, 356.0));
      expect(tester.binding.transientCallbackCount, 0);

      await tester.pumpWidget(_buildFrame(location: FloatingActionButtonLocation.endFloat));

      expect(tester.binding.transientCallbackCount, greaterThan(0));

      await tester.pumpAndSettle();

      expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(756.0, 356.0));
      expect(tester.binding.transientCallbackCount, 0);
    });

    testWidgets('moves to and from custom-defined positions', (WidgetTester tester) async {
      await tester.pumpWidget(_buildFrame(location: const _StartTopFloatingActionButtonLocation()));

      expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(44.0, 56.0));

      await tester.pumpWidget(_buildFrame(location: FloatingActionButtonLocation.centerFloat));
      expect(tester.binding.transientCallbackCount, greaterThan(0));

      await tester.pumpAndSettle();

      expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(400.0, 356.0));
      expect(tester.binding.transientCallbackCount, 0);

      await tester.pumpWidget(_buildFrame(location: const _StartTopFloatingActionButtonLocation()));

      expect(tester.binding.transientCallbackCount, greaterThan(0));

      await tester.pumpAndSettle();

      expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(44.0, 56.0));
      expect(tester.binding.transientCallbackCount, 0);
    });

    group('interrupts in-progress animations without jumps', () {
      _GeometryListener? geometryListener;
      ScaffoldGeometry? geometry;
      _GeometryListenerState? listenerState;
      Size? previousRect;
      Iterable<double>? previousRotations;

      // The maximum amounts we expect the fab width and height to change
      // during one step of a transition.
      const double maxDeltaWidth = 12.5;
      const double maxDeltaHeight = 12.5;

      // The maximum amounts we expect the fab icon to rotate during one step
      // of a transition.
      const double maxDeltaRotation = 0.09;

      // We'll listen to the Scaffold's geometry for any 'jumps' to detect
      // changes in the size and rotation of the fab.
      void setupListener(WidgetTester tester) {
        // Measure the delta in width and height of the fab, and check that it never grows
        // by more than the expected maximum deltas.
        void check() {
          geometry = listenerState?.cache.value;
          final Size? currentRect = geometry?.floatingActionButtonArea?.size;
          // Measure the delta in width and height of the rect, and check that
          // it never grows by more than a safe amount.
          if (previousRect != null && currentRect != null) {
            final double deltaWidth = currentRect.width - previousRect!.width;
            final double deltaHeight = currentRect.height - previousRect!.height;
            expect(
              deltaWidth.abs(),
              lessThanOrEqualTo(maxDeltaWidth),
              reason:
                  "The Floating Action Button's width should not change "
                  'faster than $maxDeltaWidth per animation step.\n'
                  'Previous rect: $previousRect, current rect: $currentRect',
            );
            expect(
              deltaHeight.abs(),
              lessThanOrEqualTo(maxDeltaHeight),
              reason:
                  "The Floating Action Button's width should not change "
                  'faster than $maxDeltaHeight per animation step.\n'
                  'Previous rect: $previousRect, current rect: $currentRect',
            );
          }
          previousRect = currentRect;

          // Measure the delta in rotation.
          // Check that it never grows by more than a safe amount.
          //
          // There may be multiple transitions all active at
          // the same time. We are concerned only with the closest one.
          final Iterable<RotationTransition> rotationTransitions = tester.widgetList(
            find.byType(RotationTransition),
          );
          final Iterable<double> currentRotations = rotationTransitions.map(
            (RotationTransition t) => t.turns.value,
          );

          if ((previousRotations?.isNotEmpty ?? false) &&
              currentRotations.isNotEmpty &&
              previousRect != null &&
              currentRect != null) {
            final List<double> deltas = <double>[];
            for (final double currentRotation in currentRotations) {
              late double minDelta;
              for (final double previousRotation in previousRotations!) {
                final double delta = (previousRotation - currentRotation).abs();
                minDelta = delta;
                minDelta = min(delta, minDelta);
              }
              deltas.add(minDelta);
            }

            if (deltas.where((double delta) => delta < maxDeltaRotation).isEmpty) {
              fail(
                "The Floating Action Button's rotation should not change "
                'faster than $maxDeltaRotation per animation step.\n'
                'Detected deltas were: $deltas\n'
                'Previous values: $previousRotations, current values: $currentRotations\n'
                'Previous rect: $previousRect, current rect: $currentRect',
              );
            }
          }
          previousRotations = currentRotations;
        }

        listenerState = tester.state(find.byType(_GeometryListener));
        listenerState!.geometryListenable!.addListener(check);
      }

      setUp(() {
        // We create the geometry listener here, but it can only be set up
        // after it is pumped into the widget tree and a tester is
        // available.
        geometryListener = const _GeometryListener();
        geometry = null;
        listenerState = null;
        previousRect = null;
        previousRotations = null;
      });

      testWidgets('moving the fab to centerFloat', (WidgetTester tester) async {
        // Create a scaffold with the fab at endFloat
        await tester.pumpWidget(
          _buildFrame(location: FloatingActionButtonLocation.endFloat, listener: geometryListener),
        );
        setupListener(tester);

        // Move the fab to centerFloat'
        await tester.pumpWidget(
          _buildFrame(
            location: FloatingActionButtonLocation.centerFloat,
            listener: geometryListener,
          ),
        );
        await tester.pumpAndSettle();
      });

      testWidgets('interrupting motion towards the StartTop location.', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(
          _buildFrame(
            location: FloatingActionButtonLocation.centerFloat,
            listener: geometryListener,
          ),
        );
        setupListener(tester);

        // Move the fab to the top start after creating the fab.
        await tester.pumpWidget(
          _buildFrame(
            location: const _StartTopFloatingActionButtonLocation(),
            listener: geometryListener,
          ),
        );
        await tester.pump(kFloatingActionButtonSegue ~/ 2);

        // Interrupt motion to move to the end float
        await tester.pumpWidget(
          _buildFrame(location: FloatingActionButtonLocation.endFloat, listener: geometryListener),
        );
        await tester.pumpAndSettle();
      });

      testWidgets('interrupting entrance to remove the fab.', (WidgetTester tester) async {
        await tester.pumpWidget(
          _buildFrame(
            fab: null,
            location: FloatingActionButtonLocation.centerFloat,
            listener: geometryListener,
          ),
        );
        setupListener(tester);

        // Animate the fab in.
        await tester.pumpWidget(
          _buildFrame(location: FloatingActionButtonLocation.endFloat, listener: geometryListener),
        );
        await tester.pump(kFloatingActionButtonSegue ~/ 2);

        // Remove the fab.
        await tester.pumpWidget(
          _buildFrame(
            fab: null,
            location: FloatingActionButtonLocation.endFloat,
            listener: geometryListener,
          ),
        );
        await tester.pumpAndSettle();
      });

      testWidgets('interrupting entrance of a new fab.', (WidgetTester tester) async {
        await tester.pumpWidget(
          _buildFrame(
            fab: null,
            location: FloatingActionButtonLocation.endFloat,
            listener: geometryListener,
          ),
        );
        setupListener(tester);

        // Bring in a new fab.
        await tester.pumpWidget(
          _buildFrame(
            location: FloatingActionButtonLocation.centerFloat,
            listener: geometryListener,
          ),
        );
        await tester.pump(kFloatingActionButtonSegue ~/ 2);

        // Interrupt motion to move the fab.
        await tester.pumpWidget(
          _buildFrame(location: FloatingActionButtonLocation.endFloat, listener: geometryListener),
        );
        await tester.pumpAndSettle();
      });
    });
  });

  testWidgets('Docked floating action button locations', (WidgetTester tester) async {
    await tester.pumpWidget(
      _buildFrame(
        location: FloatingActionButtonLocation.endDocked,
        bab: const SizedBox(height: 100.0),
        viewInsets: EdgeInsets.zero,
      ),
    );

    // Scaffold 800x600, FAB is 56x56, BAB is 800x100, FAB's center is
    // at the top of the BAB.
    expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(756.0, 500.0));

    await tester.pumpWidget(
      _buildFrame(
        location: FloatingActionButtonLocation.centerDocked,
        bab: const SizedBox(height: 100.0),
        viewInsets: EdgeInsets.zero,
      ),
    );
    await tester.pumpAndSettle();
    expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(400.0, 500.0));

    await tester.pumpWidget(
      _buildFrame(
        location: FloatingActionButtonLocation.endDocked,
        bab: const SizedBox(height: 100.0),
        viewInsets: EdgeInsets.zero,
      ),
    );
    await tester.pumpAndSettle();
    expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(756.0, 500.0));
  });

  testWidgets('Docked floating action button locations: no BAB, small BAB', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      _buildFrame(location: FloatingActionButtonLocation.endDocked, viewInsets: EdgeInsets.zero),
    );
    expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(756.0, 572.0));

    await tester.pumpWidget(
      _buildFrame(
        location: FloatingActionButtonLocation.endDocked,
        bab: const SizedBox(height: 16.0),
        viewInsets: EdgeInsets.zero,
      ),
    );
    expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(756.0, 572.0));
  });

  testWidgets('Contained floating action button locations', (WidgetTester tester) async {
    await tester.pumpWidget(
      _buildFrame(
        location: FloatingActionButtonLocation.endContained,
        bab: const SizedBox(height: 100.0),
        viewInsets: EdgeInsets.zero,
      ),
    );

    // Scaffold 800x600, FAB is 56x56, BAB is 800x100, FAB's center is
    // at the top of the BAB.
    // Formula: scaffold height - BAB height + FAB height / 2 + BAB top & bottom margins.
    expect(tester.getCenter(find.byType(FloatingActionButton)), const Offset(756.0, 550.0));
  });

  testWidgets('Mini-start-top floating action button location', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(),
          floatingActionButton: FloatingActionButton(onPressed: () {}, mini: true),
          floatingActionButtonLocation: FloatingActionButtonLocation.miniStartTop,
          body: const Column(children: <Widget>[ListTile(leading: CircleAvatar())]),
        ),
      ),
    );
    expect(
      tester.getCenter(find.byType(FloatingActionButton)).dx,
      tester.getCenter(find.byType(CircleAvatar)).dx,
    );
    expect(tester.getCenter(find.byType(FloatingActionButton)).dy, kToolbarHeight);
  });

  testWidgets('Start-top floating action button location LTR', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(),
          floatingActionButton: const FloatingActionButton(onPressed: null),
          floatingActionButtonLocation: FloatingActionButtonLocation.startTop,
        ),
      ),
    );
    expect(
      tester.getRect(find.byType(FloatingActionButton)),
      rectMoreOrLessEquals(const Rect.fromLTWH(16.0, 28.0, 56.0, 56.0)),
    );
  });

  testWidgets('End-top floating action button location RTL', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Scaffold(
            appBar: AppBar(),
            floatingActionButton: const FloatingActionButton(onPressed: null),
            floatingActionButtonLocation: FloatingActionButtonLocation.endTop,
          ),
        ),
      ),
    );
    expect(
      tester.getRect(find.byType(FloatingActionButton)),
      rectMoreOrLessEquals(const Rect.fromLTWH(16.0, 28.0, 56.0, 56.0)),
    );
  });

  testWidgets('Start-top floating action button location RTL', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Scaffold(
            appBar: AppBar(),
            floatingActionButton: const FloatingActionButton(onPressed: null),
            floatingActionButtonLocation: FloatingActionButtonLocation.startTop,
          ),
        ),
      ),
    );
    expect(
      tester.getRect(find.byType(FloatingActionButton)),
      rectMoreOrLessEquals(const Rect.fromLTWH(800.0 - 56.0 - 16.0, 28.0, 56.0, 56.0)),
    );
  });

  testWidgets('End-top floating action button location LTR', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(),
          floatingActionButton: const FloatingActionButton(onPressed: null),
          floatingActionButtonLocation: FloatingActionButtonLocation.endTop,
        ),
      ),
    );
    expect(
      tester.getRect(find.byType(FloatingActionButton)),
      rectMoreOrLessEquals(const Rect.fromLTWH(800.0 - 56.0 - 16.0, 28.0, 56.0, 56.0)),
    );
  });

  group('New Floating Action Button Locations', () {
    testWidgets('startTop', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.startTop));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX, _topOffsetY),
      );
    });

    testWidgets('centerTop', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.centerTop));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_centerOffsetX, _topOffsetY),
      );
    });

    testWidgets('endTop', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.endTop));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_rightOffsetX, _topOffsetY),
      );
    });

    testWidgets('startFloat', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.startFloat));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX, _floatOffsetY),
      );
    });

    testWidgets('centerFloat', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.centerFloat));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_centerOffsetX, _floatOffsetY),
      );
    });

    testWidgets('endFloat', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.endFloat));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_rightOffsetX, _floatOffsetY),
      );
    });

    testWidgets('startDocked', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.startDocked));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX, _dockedOffsetY),
      );
    });

    testWidgets('centerDocked', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.centerDocked));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_centerOffsetX, _dockedOffsetY),
      );
    });

    testWidgets('endDocked', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.endDocked));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_rightOffsetX, _dockedOffsetY),
      );
    });

    testWidgets('endContained', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.endContained));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_rightOffsetX, _containedOffsetY),
      );
    });

    testWidgets('miniStartTop', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.miniStartTop));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_miniLeftOffsetX, _topOffsetY),
      );
    });

    testWidgets('miniEndTop', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.miniEndTop));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_miniRightOffsetX, _topOffsetY),
      );
    });

    testWidgets('miniStartFloat', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.miniStartFloat));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_miniLeftOffsetX, _miniFloatOffsetY),
      );
    });

    testWidgets('miniCenterFloat', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.miniCenterFloat));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_centerOffsetX, _miniFloatOffsetY),
      );
    });

    testWidgets('miniEndFloat', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.miniEndFloat));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_miniRightOffsetX, _miniFloatOffsetY),
      );
    });

    testWidgets('miniStartDocked', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.miniStartDocked));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_miniLeftOffsetX, _dockedOffsetY),
      );
    });

    testWidgets('miniEndDocked', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.miniEndDocked));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_miniRightOffsetX, _dockedOffsetY),
      );
    });

    // Test a few RTL cases.

    testWidgets('endTop, RTL', (WidgetTester tester) async {
      await tester.pumpWidget(
        _singleFabScaffold(FloatingActionButtonLocation.endTop, textDirection: TextDirection.rtl),
      );

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX, _topOffsetY),
      );
    });

    testWidgets('miniStartFloat, RTL', (WidgetTester tester) async {
      await tester.pumpWidget(
        _singleFabScaffold(
          FloatingActionButtonLocation.miniStartFloat,
          textDirection: TextDirection.rtl,
        ),
      );

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_miniRightOffsetX, _miniFloatOffsetY),
      );
    });
  });

  group('Custom Floating Action Button Locations', () {
    testWidgets('Almost end float', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(_AlmostEndFloatFabLocation()));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_rightOffsetX - 50, _floatOffsetY),
      );
    });

    testWidgets('Almost end float, RTL', (WidgetTester tester) async {
      await tester.pumpWidget(
        _singleFabScaffold(_AlmostEndFloatFabLocation(), textDirection: TextDirection.rtl),
      );

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX + 50, _floatOffsetY),
      );
    });

    testWidgets('Quarter end top', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(_QuarterEndTopFabLocation()));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_rightOffsetX * 0.75 + _leftOffsetX * 0.25, _topOffsetY),
      );
    });

    testWidgets('Quarter end top, RTL', (WidgetTester tester) async {
      await tester.pumpWidget(
        _singleFabScaffold(_QuarterEndTopFabLocation(), textDirection: TextDirection.rtl),
      );

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX * 0.75 + _rightOffsetX * 0.25, _topOffsetY),
      );
    });
  });

  group('Moves involving new locations', () {
    testWidgets('Moves between new locations and new locations', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.centerTop));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_centerOffsetX, _topOffsetY),
      );

      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.startFloat));

      expect(tester.binding.transientCallbackCount, greaterThan(0));
      await tester.pumpAndSettle();
      expect(tester.binding.transientCallbackCount, 0);

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX, _floatOffsetY),
      );

      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.startDocked));

      expect(tester.binding.transientCallbackCount, greaterThan(0));
      await tester.pumpAndSettle();
      expect(tester.binding.transientCallbackCount, 0);

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX, _dockedOffsetY),
      );
    });

    testWidgets('Moves between new locations and old locations', (WidgetTester tester) async {
      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.endDocked));

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_rightOffsetX, _dockedOffsetY),
      );

      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.startDocked));

      expect(tester.binding.transientCallbackCount, greaterThan(0));
      await tester.pumpAndSettle();
      expect(tester.binding.transientCallbackCount, 0);

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_leftOffsetX, _dockedOffsetY),
      );

      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.centerFloat));

      expect(tester.binding.transientCallbackCount, greaterThan(0));
      await tester.pumpAndSettle();
      expect(tester.binding.transientCallbackCount, 0);

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_centerOffsetX, _floatOffsetY),
      );

      await tester.pumpWidget(_singleFabScaffold(FloatingActionButtonLocation.centerTop));

      expect(tester.binding.transientCallbackCount, greaterThan(0));
      await tester.pumpAndSettle();
      expect(tester.binding.transientCallbackCount, 0);

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        const Offset(_centerOffsetX, _topOffsetY),
      );
    });

    testWidgets('Moves between new locations and old locations with custom animator', (
      WidgetTester tester,
    ) async {
      final FloatingActionButtonAnimator animator = _LinearMovementFabAnimator();
      const Offset begin = Offset(_centerOffsetX, _topOffsetY);
      const Offset end = Offset(_rightOffsetX - 50, _floatOffsetY);

      final Duration animationDuration = kFloatingActionButtonSegue * 2;

      await tester.pumpWidget(
        _singleFabScaffold(FloatingActionButtonLocation.centerTop, animator: animator),
      );

      expect(find.byType(FloatingActionButton), findsOneWidget);

      expect(tester.binding.transientCallbackCount, 0);

      await tester.pumpWidget(_singleFabScaffold(_AlmostEndFloatFabLocation(), animator: animator));

      expect(tester.binding.transientCallbackCount, greaterThan(0));

      await tester.pump(animationDuration * 0.25);

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        offsetMoreOrLessEquals(begin * 0.75 + end * 0.25),
      );

      await tester.pump(animationDuration * 0.25);

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        offsetMoreOrLessEquals(begin * 0.5 + end * 0.5),
      );

      await tester.pump(animationDuration * 0.25);

      expect(
        tester.getCenter(find.byType(FloatingActionButton)),
        offsetMoreOrLessEquals(begin * 0.25 + end * 0.75),
      );

      await tester.pumpAndSettle();

      expect(tester.getCenter(find.byType(FloatingActionButton)), end);

      expect(tester.binding.transientCallbackCount, 0);
    });

    testWidgets('Animator can be updated', (WidgetTester tester) async {
      FloatingActionButtonAnimator fabAnimator = FloatingActionButtonAnimator.scaling;
      FloatingActionButtonLocation fabLocation = FloatingActionButtonLocation.startFloat;

      final Duration animationDuration = kFloatingActionButtonSegue * 2;

      await tester.pumpWidget(_singleFabScaffold(fabLocation, animator: fabAnimator));

      expect(find.byType(FloatingActionButton), findsOneWidget);
      expect(tester.binding.transientCallbackCount, 0);
      expect(tester.getCenter(find.byType(FloatingActionButton)).dx, 44.0);

      fabLocation = FloatingActionButtonLocation.endFloat;
      await tester.pumpWidget(_singleFabScaffold(fabLocation, animator: fabAnimator));

      expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, lessThan(16.0));

      await tester.pump(animationDuration * 0.25);
      expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, greaterThan(16));

      await tester.pump(animationDuration * 0.25);
      expect(tester.getCenter(find.byType(FloatingActionButton)).dx, 756.0);
      expect(tester.getTopRight(find.byType(FloatingActionButton)).dx, lessThan(800 - 16));

      await tester.pump(animationDuration * 0.25);
      expect(tester.getTopRight(find.byType(FloatingActionButton)).dx, lessThan(800 - 16));

      await tester.pump(animationDuration * 0.25);
      expect(tester.getTopRight(find.byType(FloatingActionButton)).dx, equals(800 - 16));

      fabLocation = FloatingActionButtonLocation.startFloat;
      fabAnimator = _NoScalingFabAnimator();
      await tester.pumpWidget(_singleFabScaffold(fabLocation, animator: fabAnimator));

      await tester.pump(animationDuration * 0.25);
      expect(tester.getCenter(find.byType(FloatingActionButton)).dx, 756.0);
      expect(tester.getTopRight(find.byType(FloatingActionButton)).dx, equals(800 - 16));

      await tester.pump(animationDuration * 0.25);
      expect(tester.getCenter(find.byType(FloatingActionButton)).dx, 44.0);
      expect(tester.getTopLeft(find.byType(FloatingActionButton)).dx, lessThan(16.0));
    });
  });

  group('Locations account for safe interactive areas', () {
    Widget buildTest(
      FloatingActionButtonLocation location,
      MediaQueryData data,
      Key key, {
      bool mini = false,
      bool appBar = false,
      bool bottomNavigationBar = false,
      bool bottomSheet = false,
      bool resizeToAvoidBottomInset = true,
    }) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: MediaQuery(
          data: data,
          child: Scaffold(
            resizeToAvoidBottomInset: resizeToAvoidBottomInset,
            bottomSheet:
                bottomSheet
                    ? const SizedBox(height: 100, child: Center(child: Text('BottomSheet')))
                    : null,
            appBar: appBar ? AppBar(title: const Text('Demo')) : null,
            bottomNavigationBar:
                bottomNavigationBar
                    ? BottomNavigationBar(
                      items: const <BottomNavigationBarItem>[
                        BottomNavigationBarItem(icon: Icon(Icons.star), label: '0'),
                        BottomNavigationBarItem(icon: Icon(Icons.star_border), label: '1'),
                      ],
                    )
                    : null,
            floatingActionButtonLocation: location,
            floatingActionButton: Builder(
              builder: (BuildContext context) {
                return FloatingActionButton(
                  onPressed: () {
                    ScaffoldMessenger.of(
                      context,
                    ).showSnackBar(const SnackBar(content: Text('Snacky!')));
                  },
                  mini: mini,
                  key: key,
                  child: const Text('FAB'),
                );
              },
            ),
          ),
        ),
      );
    }

    // Test float locations, for each (6), keyboard presented or not:
    //  - Default
    //  - with resizeToAvoidBottomInset: false
    //  - with BottomNavigationBar
    //  - with BottomNavigationBar and resizeToAvoidBottomInset: false
    //  - with BottomNavigationBar & BottomSheet
    //  - with BottomNavigationBar & BottomSheet, resizeToAvoidBottomInset: false
    //  - with BottomSheet
    //  - with BottomSheet and resizeToAvoidBottomInset: false
    //  - with SnackBar
    Future<void> runFloatTests(
      WidgetTester tester,
      FloatingActionButtonLocation location, {
      required Rect defaultRect,
      required Rect bottomNavigationBarRect,
      required Rect bottomSheetRect,
      required Rect snackBarRect,
      bool mini = false,
    }) async {
      const double keyboardHeight = 200.0;
      const double viewPadding = 50.0;
      final Key floatingActionButton = UniqueKey();
      const double bottomNavHeight = 106.0;
      // Default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(viewPadding: EdgeInsets.only(bottom: viewPadding)),
          floatingActionButton,
          mini: mini,
        ),
      );
      expect(tester.getRect(find.byKey(floatingActionButton)), rectMoreOrLessEquals(defaultRect));
      // Present keyboard and check position, should change
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(defaultRect.translate(0.0, viewPadding - keyboardHeight)),
      );

      // With resizeToAvoidBottomInset: false
      // With keyboard presented, should maintain default position
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          resizeToAvoidBottomInset: false,
          mini: mini,
        ),
      );
      expect(tester.getRect(find.byKey(floatingActionButton)), rectMoreOrLessEquals(defaultRect));

      // BottomNavigationBar default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomNavigationBarRect),
      );
      // Present keyboard and check position, FAB position changes
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(
          bottomNavigationBarRect.translate(0.0, -keyboardHeight + bottomNavHeight),
        ),
      );

      // BottomNavigationBar with resizeToAvoidBottomInset: false
      // With keyboard presented, should maintain default position
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          resizeToAvoidBottomInset: false,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomNavigationBarRect),
      );

      // BottomNavigationBar + BottomSheet default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          bottomSheet: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect.translate(0.0, -bottomNavHeight)),
      );
      // Present keyboard and check position, FAB position changes
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          bottomSheet: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect.translate(0.0, -keyboardHeight)),
      );

      // BottomNavigationBar + BottomSheet with resizeToAvoidBottomInset: false
      // With keyboard presented, should maintain default position
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          bottomSheet: true,
          resizeToAvoidBottomInset: false,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect.translate(0.0, -bottomNavHeight)),
      );

      // BottomSheet default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(viewPadding: EdgeInsets.only(bottom: viewPadding)),
          floatingActionButton,
          bottomSheet: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect),
      );
      // Present keyboard and check position, bottomSheet and FAB both resize
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomSheet: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect.translate(0.0, -keyboardHeight)),
      );

      // bottomSheet with resizeToAvoidBottomInset: false
      // With keyboard presented, should maintain default bottomSheet position
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomSheet: true,
          resizeToAvoidBottomInset: false,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect),
      );

      // SnackBar default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(viewPadding: EdgeInsets.only(bottom: viewPadding)),
          floatingActionButton,
          mini: mini,
        ),
      );
      await tester.tap(find.byKey(floatingActionButton));
      await tester.pumpAndSettle(); // Show SnackBar
      expect(tester.getRect(find.byKey(floatingActionButton)), rectMoreOrLessEquals(snackBarRect));

      // SnackBar when resized for presented keyboard
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          mini: mini,
        ),
      );
      await tester.tap(find.byKey(floatingActionButton));
      await tester.pumpAndSettle(); // Show SnackBar
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(
          snackBarRect.translate(0.0, -keyboardHeight + kFloatingActionButtonMargin / 2),
        ),
      );
    }

    testWidgets('startFloat', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(16.0, 478.0, 72.0, 534.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(16.0, 422.0, 72.0, 478.0);
      // Position relative to BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(16.0, 472.0, 72.0, 528.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(16.0, 478.0, 72.0, 534.0);
      await runFloatTests(
        tester,
        FloatingActionButtonLocation.startFloat,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
      );
    });

    testWidgets('miniStartFloat', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(12.0, 490.0, 60.0, 538.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(12.0, 434.0, 60.0, 482.0);
      // Positioned relative to BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(12.0, 480.0, 60.0, 528.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(12.0, 490.0, 60.0, 538.0);
      await runFloatTests(
        tester,
        FloatingActionButtonLocation.miniStartFloat,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
        mini: true,
      );
    });

    testWidgets('centerFloat', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(372.0, 478.0, 428.0, 534.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(372.0, 422.0, 428.0, 478.0);
      // Positioned relative to BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(372.0, 472.0, 428.0, 528.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(372.0, 478.0, 428.0, 534.0);
      await runFloatTests(
        tester,
        FloatingActionButtonLocation.centerFloat,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
      );
    });

    testWidgets('miniCenterFloat', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(376.0, 490.0, 424.0, 538.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(376.0, 434.0, 424.0, 482.0);
      // Positioned relative to BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(376.0, 480.0, 424.0, 528.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(376.0, 490.0, 424.0, 538.0);
      await runFloatTests(
        tester,
        FloatingActionButtonLocation.miniCenterFloat,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
        mini: true,
      );
    });

    testWidgets('endFloat', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(728.0, 478.0, 784.0, 534.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(728.0, 422.0, 784.0, 478.0);
      // Positioned relative to BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(728.0, 472.0, 784.0, 528.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(728.0, 478.0, 784.0, 534.0);
      await runFloatTests(
        tester,
        FloatingActionButtonLocation.endFloat,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
      );
    });

    testWidgets('miniEndFloat', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(740.0, 490.0, 788.0, 538.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(740.0, 434.0, 788.0, 482.0);
      // Positioned relative to BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(740.0, 480.0, 788.0, 528.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(740.0, 490.0, 788.0, 538.0);
      await runFloatTests(
        tester,
        FloatingActionButtonLocation.miniEndFloat,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
        mini: true,
      );
    });

    // Test docked locations, for each (6), keyboard presented or not.
    // If keyboard is presented and resizeToAvoidBottomInset: true, test whether
    // the FAB is away from the keyboard(and thus not clipped):
    //  - Default
    //  - Default with resizeToAvoidBottomInset: false
    //  - docked with BottomNavigationBar
    //  - docked with BottomNavigationBar and resizeToAvoidBottomInset: false
    //  - docked with BottomNavigationBar & BottomSheet
    //  - docked with BottomNavigationBar & BottomSheet, resizeToAvoidBottomInset: false
    //  - with SnackBar
    Future<void> runDockedTests(
      WidgetTester tester,
      FloatingActionButtonLocation location, {
      required Rect defaultRect,
      required Rect bottomNavigationBarRect,
      required Rect bottomSheetRect,
      required Rect snackBarRect,
      bool mini = false,
    }) async {
      const double keyboardHeight = 200.0;
      const double viewPadding = 50.0;
      const double bottomNavHeight = 106.0;
      const double scaffoldHeight = 600.0;
      final Key floatingActionButton = UniqueKey();
      final double fabHeight = mini ? 48.0 : 56.0;
      // Default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(viewPadding: EdgeInsets.only(bottom: viewPadding)),
          floatingActionButton,
          mini: mini,
        ),
      );
      expect(tester.getRect(find.byKey(floatingActionButton)), rectMoreOrLessEquals(defaultRect));
      // Present keyboard and check position, should change
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(
          defaultRect.translate(0.0, viewPadding - keyboardHeight - kFloatingActionButtonMargin),
        ),
      );
      // The FAB should be away from the keyboard
      expect(
        tester.getRect(find.byKey(floatingActionButton)).bottom,
        lessThan(scaffoldHeight - keyboardHeight),
      );

      // With resizeToAvoidBottomInset: false
      // With keyboard presented, should maintain default position
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          resizeToAvoidBottomInset: false,
          mini: mini,
        ),
      );
      expect(tester.getRect(find.byKey(floatingActionButton)), rectMoreOrLessEquals(defaultRect));

      // BottomNavigationBar default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomNavigationBarRect),
      );
      // Present keyboard and check position, FAB position changes
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(
          bottomNavigationBarRect.translate(
            0.0,
            bottomNavHeight +
                fabHeight / 2.0 -
                keyboardHeight -
                kFloatingActionButtonMargin -
                fabHeight,
          ),
        ),
      );
      // The FAB should be away from the keyboard
      expect(
        tester.getRect(find.byKey(floatingActionButton)).bottom,
        lessThan(scaffoldHeight - keyboardHeight),
      );

      // BottomNavigationBar with resizeToAvoidBottomInset: false
      // With keyboard presented, should maintain default position
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          resizeToAvoidBottomInset: false,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomNavigationBarRect),
      );

      // BottomNavigationBar + BottomSheet default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          bottomSheet: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect),
      );
      // Present keyboard and check position, FAB position changes
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          bottomSheet: true,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect.translate(0.0, -keyboardHeight + bottomNavHeight)),
      );
      // The FAB should be away from the keyboard
      expect(
        tester.getRect(find.byKey(floatingActionButton)).bottom,
        lessThan(scaffoldHeight - keyboardHeight),
      );

      // BottomNavigationBar + BottomSheet with resizeToAvoidBottomInset: false
      // With keyboard presented, should maintain default position
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          bottomSheet: true,
          resizeToAvoidBottomInset: false,
          mini: mini,
        ),
      );
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(bottomSheetRect),
      );

      // SnackBar default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(viewPadding: EdgeInsets.only(bottom: viewPadding)),
          floatingActionButton,
          mini: mini,
        ),
      );
      await tester.tap(find.byKey(floatingActionButton));
      await tester.pumpAndSettle(); // Show SnackBar
      expect(tester.getRect(find.byKey(floatingActionButton)), rectMoreOrLessEquals(snackBarRect));

      // SnackBar with BottomNavigationBar
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            padding: EdgeInsets.only(bottom: viewPadding),
            viewPadding: EdgeInsets.only(bottom: viewPadding),
          ),
          floatingActionButton,
          bottomNavigationBar: true,
          mini: mini,
        ),
      );
      await tester.tap(find.byKey(floatingActionButton));
      await tester.pumpAndSettle(); // Show SnackBar
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(snackBarRect.translate(0.0, -bottomNavHeight)),
      );

      // SnackBar when resized for presented keyboard
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: viewPadding),
            viewInsets: EdgeInsets.only(bottom: keyboardHeight),
          ),
          floatingActionButton,
          mini: mini,
        ),
      );
      await tester.tap(find.byKey(floatingActionButton));
      await tester.pumpAndSettle(); // Show SnackBar
      expect(
        tester.getRect(find.byKey(floatingActionButton)),
        rectMoreOrLessEquals(snackBarRect.translate(0.0, -keyboardHeight)),
      );
      // The FAB should be away from the keyboard
      expect(
        tester.getRect(find.byKey(floatingActionButton)).bottom,
        lessThan(scaffoldHeight - keyboardHeight),
      );
    }

    testWidgets('startDocked', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(16.0, 494.0, 72.0, 550.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(16.0, 466.0, 72.0, 522.0);
      // Positioned relative to BottomNavigationBar & BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(16.0, 366.0, 72.0, 422.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(16.0, 486.0, 72.0, 542.0);
      await runDockedTests(
        tester,
        FloatingActionButtonLocation.startDocked,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
      );
    });

    testWidgets('miniStartDocked', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(12.0, 502.0, 60.0, 550.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(12.0, 470.0, 60.0, 518.0);
      // Positioned relative to BottomNavigationBar & BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(12.0, 370.0, 60.0, 418.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(12.0, 494.0, 60.0, 542.0);
      await runDockedTests(
        tester,
        FloatingActionButtonLocation.miniStartDocked,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
        mini: true,
      );
    });

    testWidgets('centerDocked', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(372.0, 494.0, 428.0, 550.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(372.0, 466.0, 428.0, 522.0);
      // Positioned relative to BottomNavigationBar & BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(372.0, 366.0, 428.0, 422.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(372.0, 486.0, 428.0, 542.0);
      await runDockedTests(
        tester,
        FloatingActionButtonLocation.centerDocked,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
      );
    });

    testWidgets('miniCenterDocked', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(376.0, 502.0, 424.0, 550.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(376.0, 470.0, 424.0, 518.0);
      // Positioned relative to BottomNavigationBar & BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(376.0, 370.0, 424.0, 418.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(376.0, 494.0, 424.0, 542.0);
      await runDockedTests(
        tester,
        FloatingActionButtonLocation.miniCenterDocked,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
        mini: true,
      );
    });

    testWidgets('endDocked', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(728.0, 494.0, 784.0, 550.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(728.0, 466.0, 784.0, 522.0);
      // Positioned relative to BottomNavigationBar & BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(728.0, 366.0, 784.0, 422.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(728.0, 486.0, 784.0, 542.0);
      await runDockedTests(
        tester,
        FloatingActionButtonLocation.endDocked,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
      );
    });

    testWidgets('miniEndDocked', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(740.0, 502.0, 788.0, 550.0);
      // Positioned relative to BottomNavigationBar
      const Rect bottomNavigationBarRect = Rect.fromLTRB(740.0, 470.0, 788.0, 518.0);
      // Positioned relative to BottomNavigationBar & BottomSheet
      const Rect bottomSheetRect = Rect.fromLTRB(740.0, 370.0, 788.0, 418.0);
      // Positioned relative to SnackBar
      const Rect snackBarRect = Rect.fromLTRB(740.0, 494.0, 788.0, 542.0);
      await runDockedTests(
        tester,
        FloatingActionButtonLocation.miniEndDocked,
        defaultRect: defaultRect,
        bottomNavigationBarRect: bottomNavigationBarRect,
        bottomSheetRect: bottomSheetRect,
        snackBarRect: snackBarRect,
        mini: true,
      );
    });

    // Test top locations, for each (6):
    //  - Default
    //  - with an AppBar
    Future<void> runTopTests(
      WidgetTester tester,
      FloatingActionButtonLocation location, {
      required Rect defaultRect,
      required Rect appBarRect,
      bool mini = false,
    }) async {
      const double viewPadding = 50.0;
      final Key floatingActionButton = UniqueKey();
      // Default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(viewPadding: EdgeInsets.only(top: viewPadding)),
          floatingActionButton,
          mini: mini,
        ),
      );
      expect(tester.getRect(find.byKey(floatingActionButton)), rectMoreOrLessEquals(defaultRect));

      // AppBar default
      await tester.pumpWidget(
        buildTest(
          location,
          const MediaQueryData(viewPadding: EdgeInsets.only(top: viewPadding)),
          floatingActionButton,
          appBar: true,
          mini: mini,
        ),
      );
      expect(tester.getRect(find.byKey(floatingActionButton)), rectMoreOrLessEquals(appBarRect));
    }

    testWidgets('startTop', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(16.0, 50.0, 72.0, 106.0);
      // Positioned relative to AppBar
      const Rect appBarRect = Rect.fromLTRB(16.0, 28.0, 72.0, 84.0);
      await runTopTests(
        tester,
        FloatingActionButtonLocation.startTop,
        defaultRect: defaultRect,
        appBarRect: appBarRect,
      );
    });

    testWidgets('miniStartTop', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(12.0, 50.0, 60.0, 98.0);
      // Positioned relative to AppBar
      const Rect appBarRect = Rect.fromLTRB(12.0, 32.0, 60.0, 80.0);
      await runTopTests(
        tester,
        FloatingActionButtonLocation.miniStartTop,
        defaultRect: defaultRect,
        appBarRect: appBarRect,
        mini: true,
      );
    });

    testWidgets('centerTop', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(372.0, 50.0, 428.0, 106.0);
      // Positioned relative to AppBar
      const Rect appBarRect = Rect.fromLTRB(372.0, 28.0, 428.0, 84.0);
      await runTopTests(
        tester,
        FloatingActionButtonLocation.centerTop,
        defaultRect: defaultRect,
        appBarRect: appBarRect,
      );
    });

    testWidgets('miniCenterTop', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(376.0, 50.0, 424.0, 98.0);
      // Positioned relative to AppBar
      const Rect appBarRect = Rect.fromLTRB(376.0, 32.0, 424.0, 80.0);
      await runTopTests(
        tester,
        FloatingActionButtonLocation.miniCenterTop,
        defaultRect: defaultRect,
        appBarRect: appBarRect,
        mini: true,
      );
    });

    testWidgets('endTop', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(728.0, 50.0, 784.0, 106.0);
      // Positioned relative to AppBar
      const Rect appBarRect = Rect.fromLTRB(728.0, 28.0, 784.0, 84.0);
      await runTopTests(
        tester,
        FloatingActionButtonLocation.endTop,
        defaultRect: defaultRect,
        appBarRect: appBarRect,
      );
    });

    testWidgets('miniEndTop', (WidgetTester tester) async {
      const Rect defaultRect = Rect.fromLTRB(740.0, 50.0, 788.0, 98.0);
      // Positioned relative to AppBar
      const Rect appBarRect = Rect.fromLTRB(740.0, 32.0, 788.0, 80.0);
      await runTopTests(
        tester,
        FloatingActionButtonLocation.miniEndTop,
        defaultRect: defaultRect,
        appBarRect: appBarRect,
        mini: true,
      );
    });
  });
}

class _GeometryListener extends StatefulWidget {
  const _GeometryListener();

  @override
  State createState() => _GeometryListenerState();
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

const double _leftOffsetX = 44.0;
const double _centerOffsetX = 400.0;
const double _rightOffsetX = 756.0;
const double _miniLeftOffsetX = _leftOffsetX - kMiniButtonOffsetAdjustment;
const double _miniRightOffsetX = _rightOffsetX + kMiniButtonOffsetAdjustment;

const double _topOffsetY = 56.0;
const double _floatOffsetY = 500.0;
const double _dockedOffsetY = 544.0;
const double _containedOffsetY = 544.0 + 56.0 / 2;
const double _miniFloatOffsetY = _floatOffsetY + kMiniButtonOffsetAdjustment;

Widget _singleFabScaffold(
  FloatingActionButtonLocation location, {
  bool useMaterial3 = false,
  FloatingActionButtonAnimator? animator,
  bool mini = false,
  TextDirection textDirection = TextDirection.ltr,
}) {
  return MaterialApp(
    theme: ThemeData(useMaterial3: useMaterial3),
    home: Directionality(
      textDirection: textDirection,
      child: Scaffold(
        appBar: AppBar(title: const Text('FloatingActionButtonLocation Test.')),
        bottomNavigationBar: BottomNavigationBar(
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(icon: Icon(Icons.home), label: 'Home'),
            BottomNavigationBarItem(icon: Icon(Icons.school), label: 'School'),
          ],
        ),
        floatingActionButton: FloatingActionButton(
          onPressed: () {},
          mini: mini,
          child: const Icon(Icons.beach_access),
        ),
        floatingActionButtonLocation: location,
        floatingActionButtonAnimator: animator,
      ),
    ),
  );
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

Widget _buildFrame({
  FloatingActionButton? fab = const FloatingActionButton(onPressed: null, child: Text('1')),
  FloatingActionButtonLocation? location,
  _GeometryListener? listener,
  TextDirection textDirection = TextDirection.ltr,
  EdgeInsets viewInsets = const EdgeInsets.only(bottom: 200.0),
  Widget? bab,
}) {
  return Localizations(
    locale: const Locale('en', 'us'),
    delegates: const <LocalizationsDelegate<dynamic>>[
      DefaultWidgetsLocalizations.delegate,
      DefaultMaterialLocalizations.delegate,
    ],
    child: Directionality(
      textDirection: textDirection,
      child: MediaQuery(
        data: MediaQueryData(viewInsets: viewInsets),
        child: Scaffold(
          appBar: AppBar(title: const Text('FabLocation Test')),
          floatingActionButtonLocation: location,
          floatingActionButton: fab,
          bottomNavigationBar: bab,
          body: listener,
        ),
      ),
    ),
  );
}

class _StartTopFloatingActionButtonLocation extends FloatingActionButtonLocation {
  const _StartTopFloatingActionButtonLocation();

  @override
  Offset getOffset(ScaffoldPrelayoutGeometry scaffoldGeometry) {
    double fabX;
    switch (scaffoldGeometry.textDirection) {
      case TextDirection.rtl:
        final double startPadding = kFloatingActionButtonMargin + scaffoldGeometry.minInsets.right;
        fabX =
            scaffoldGeometry.scaffoldSize.width -
            scaffoldGeometry.floatingActionButtonSize.width -
            startPadding;
      case TextDirection.ltr:
        final double startPadding = kFloatingActionButtonMargin + scaffoldGeometry.minInsets.left;
        fabX = startPadding;
    }
    final double fabY =
        scaffoldGeometry.contentTop - (scaffoldGeometry.floatingActionButtonSize.height / 2.0);
    return Offset(fabX, fabY);
  }
}

class _AlmostEndFloatFabLocation extends StandardFabLocation with FabEndOffsetX, FabFloatOffsetY {
  @override
  double getOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    final double directionalAdjustment =
        scaffoldGeometry.textDirection == TextDirection.ltr ? -50.0 : 50.0;
    return super.getOffsetX(scaffoldGeometry, adjustment) + directionalAdjustment;
  }
}

class _QuarterEndTopFabLocation extends StandardFabLocation with FabEndOffsetX, FabTopOffsetY {
  @override
  double getOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    return super.getOffsetX(scaffoldGeometry, adjustment) * 0.75 +
        (FloatingActionButtonLocation.startFloat as StandardFabLocation).getOffsetX(
              scaffoldGeometry,
              adjustment,
            ) *
            0.25;
  }
}

class _LinearMovementFabAnimator extends FloatingActionButtonAnimator {
  @override
  Offset getOffset({required Offset begin, required Offset end, required double progress}) {
    return Offset.lerp(begin, end, progress)!;
  }

  @override
  Animation<double> getScaleAnimation({required Animation<double> parent}) {
    return const AlwaysStoppedAnimation<double>(1.0);
  }

  @override
  Animation<double> getRotationAnimation({required Animation<double> parent}) {
    return const AlwaysStoppedAnimation<double>(1.0);
  }
}

class _NoScalingFabAnimator extends FloatingActionButtonAnimator {
  @override
  Offset getOffset({required Offset begin, required Offset end, required double progress}) {
    return progress < 0.5 ? begin : end;
  }

  @override
  Animation<double> getScaleAnimation({required Animation<double> parent}) {
    return const AlwaysStoppedAnimation<double>(1.0);
  }

  @override
  Animation<double> getRotationAnimation({required Animation<double> parent}) {
    return const AlwaysStoppedAnimation<double>(1.0);
  }
}
