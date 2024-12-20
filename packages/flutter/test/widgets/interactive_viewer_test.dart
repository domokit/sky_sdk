// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart' show Matrix4, Quad, Vector3;

import 'gesture_utils.dart';

void main() {
  group('InteractiveViewer', () {
    late TransformationController transformationController;

    setUp(() {
      transformationController = TransformationController();
    });

    tearDown(() {
      transformationController.dispose();
    });

    testWidgets('child fits in viewport', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Attempting to drag to pan doesn't work because the child fits inside
      // the viewport and has a tight boundary.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      TestGesture gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));

      // Pinch to zoom works.
      final Offset scaleStart1 = childInterior;
      final Offset scaleStart2 = Offset(childInterior.dx + 10.0, childInterior.dy);
      final Offset scaleEnd1 = Offset(childInterior.dx - 10.0, childInterior.dy);
      final Offset scaleEnd2 = Offset(childInterior.dx + 20.0, childInterior.dy);
      gesture = await tester.createGesture();
      final TestGesture gesture2 = await tester.createGesture();
      await gesture.down(scaleStart1);
      await gesture2.down(scaleStart2);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, isNot(equals(Matrix4.identity())));
    });

    testWidgets('boundary slightly bigger than child', (WidgetTester tester) async {
      const double boundaryMargin = 10.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Dragging to pan works only until it hits the boundary.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      TestGesture gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      final Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, -boundaryMargin);
      expect(translation.y, -boundaryMargin);

      // Pinch to zoom also only works until expanding to the boundary.
      final Offset scaleStart1 = childInterior;
      final Offset scaleStart2 = Offset(childInterior.dx + 20.0, childInterior.dy);
      final Offset scaleEnd1 = Offset(scaleStart1.dx + 5.0, scaleStart1.dy);
      final Offset scaleEnd2 = Offset(scaleStart2.dx - 5.0, scaleStart2.dy);
      gesture = await tester.createGesture();
      final TestGesture gesture2 = await tester.createGesture();
      await gesture.down(scaleStart1);
      await gesture2.down(scaleStart2);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      // The new scale is the scale that makes the original size (200.0) as big
      // as the boundary (220.0).
      expect(transformationController.value.getMaxScaleOnAxis(), 200.0 / 220.0);
    });

    testWidgets('child bigger than viewport', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                constrained: false,
                scaleEnabled: false,
                transformationController: transformationController,
                child: const SizedBox(width: 2000.0, height: 2000.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Attempting to move against the boundary doesn't work.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      TestGesture gesture = await tester.startGesture(childOffset);
      await tester.pump();
      await gesture.moveTo(childInterior);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));

      // Attempting to pinch to zoom doesn't work because it's disabled.
      final Offset scaleStart1 = childInterior;
      final Offset scaleStart2 = Offset(childInterior.dx + 10.0, childInterior.dy);
      final Offset scaleEnd1 = Offset(childInterior.dx - 10.0, childInterior.dy);
      final Offset scaleEnd2 = Offset(childInterior.dx + 20.0, childInterior.dy);
      gesture = await tester.startGesture(scaleStart1);
      TestGesture gesture2 = await tester.startGesture(scaleStart2);
      addTearDown(gesture2.removePointer);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));

      // Attempting to pinch to rotate doesn't work because it's disabled.
      final Offset rotateStart1 = childInterior;
      final Offset rotateStart2 = Offset(childInterior.dx + 10.0, childInterior.dy);
      final Offset rotateEnd1 = Offset(childInterior.dx + 5.0, childInterior.dy + 5.0);
      final Offset rotateEnd2 = Offset(childInterior.dx - 5.0, childInterior.dy - 5.0);
      gesture = await tester.startGesture(rotateStart1);
      gesture2 = await tester.startGesture(rotateStart2);
      await tester.pump();
      await gesture.moveTo(rotateEnd1);
      await gesture2.moveTo(rotateEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));

      // Drag to pan away from the boundary.
      gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, isNot(equals(Matrix4.identity())));
    });

    testWidgets('child has no dimensions', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                constrained: false,
                scaleEnabled: false,
                transformationController: transformationController,
                child: const SizedBox.shrink(),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Interacting throws an error because the child has no size.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      final TestGesture gesture = await tester.startGesture(childOffset);
      await tester.pump();
      await gesture.moveTo(childInterior);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));
      expect(tester.takeException(), isAssertionError);
    });

    testWidgets('no boundary', (WidgetTester tester) async {
      const double minScale = 0.8;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(double.infinity),
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Drag to pan works because even though the viewport fits perfectly
      // around the child, there is no boundary.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      TestGesture gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      final Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, childOffset.dx - childInterior.dx);
      expect(translation.y, childOffset.dy - childInterior.dy);

      // It's also possible to zoom out and view beyond the child because there
      // is no boundary.
      final Offset scaleStart1 = childInterior;
      final Offset scaleStart2 = Offset(childInterior.dx + 20.0, childInterior.dy);
      final Offset scaleEnd1 = Offset(childInterior.dx + 5.0, childInterior.dy);
      final Offset scaleEnd2 = Offset(childInterior.dx - 5.0, childInterior.dy);
      gesture = await tester.createGesture();
      final TestGesture gesture2 = await tester.createGesture();
      await gesture.down(scaleStart1);
      await gesture2.down(scaleStart2);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(transformationController.value.getMaxScaleOnAxis(), minScale);
    });

    testWidgets('PanAxis.free allows panning in all directions for diagonal gesture', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(double.infinity),
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Perform a diagonal drag gesture.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      final TestGesture gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      // Translation has only happened along the y axis (the default axis when
      // a gesture is perfectly at 45 degrees to the axes).
      final Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, childOffset.dx - childInterior.dx);
      expect(translation.y, childOffset.dy - childInterior.dy);
    });

    testWidgets('PanAxis.aligned allows panning in one direction only for diagonal gesture', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                panAxis: PanAxis.aligned,
                boundaryMargin: const EdgeInsets.all(double.infinity),
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Perform a diagonal drag gesture.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      final TestGesture gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      // Translation has only happened along the y axis (the default axis when
      // a gesture is perfectly at 45 degrees to the axes).
      final Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, 0.0);
      expect(translation.y, childOffset.dy - childInterior.dy);
    });

    testWidgets(
      'PanAxis.aligned allows panning in one direction only for horizontal leaning gesture',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  panAxis: PanAxis.aligned,
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        expect(transformationController.value, equals(Matrix4.identity()));

        // Perform a horizontally leaning diagonal drag gesture.
        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 10.0);
        final TestGesture gesture = await tester.startGesture(childInterior);
        await tester.pump();
        await gesture.moveTo(childOffset);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        // Translation happened only along the x axis because that's the axis that
        // had the greatest movement.
        final Vector3 translation = transformationController.value.getTranslation();
        expect(translation.x, childOffset.dx - childInterior.dx);
        expect(translation.y, 0.0);
      },
    );

    testWidgets(
      'PanAxis.horizontal allows panning in the horizontal direction only for diagonal gesture',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  panAxis: PanAxis.horizontal,
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        expect(transformationController.value, equals(Matrix4.identity()));

        // Perform a diagonal drag gesture.
        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
        final TestGesture gesture = await tester.startGesture(childInterior);
        await tester.pump();
        await gesture.moveTo(childOffset);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        // Translation has only happened along the x axis (the default axis when
        // a gesture is perfectly at 45 degrees to the axes).
        final Vector3 translation = transformationController.value.getTranslation();
        expect(translation.x, childOffset.dx - childInterior.dx);
        expect(translation.y, 0.0);
      },
    );

    testWidgets(
      'PanAxis.horizontal allows panning in the horizontal direction only for horizontal leaning gesture',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  panAxis: PanAxis.horizontal,
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        expect(transformationController.value, equals(Matrix4.identity()));

        // Perform a horizontally leaning diagonal drag gesture.
        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 10.0);
        final TestGesture gesture = await tester.startGesture(childInterior);
        await tester.pump();
        await gesture.moveTo(childOffset);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        // Translation happened only along the x axis because that's the axis that
        // had been set to the panningDirection parameter.
        final Vector3 translation = transformationController.value.getTranslation();
        expect(translation.x, childOffset.dx - childInterior.dx);
        expect(translation.y, 0.0);
      },
    );

    testWidgets(
      'PanAxis.horizontal does not allow panning in vertical direction on vertical gesture',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  panAxis: PanAxis.horizontal,
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        expect(transformationController.value, equals(Matrix4.identity()));

        // Perform a horizontally leaning diagonal drag gesture.
        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 0.0, childOffset.dy + 10.0);
        final TestGesture gesture = await tester.startGesture(childInterior);
        await tester.pump();
        await gesture.moveTo(childOffset);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        // Translation didn't happen because the only axis allowed to do panning
        // is the horizontal.
        final Vector3 translation = transformationController.value.getTranslation();
        expect(translation.x, 0.0);
        expect(translation.y, 0.0);
      },
    );

    testWidgets(
      'PanAxis.vertical allows panning in the vertical direction only for diagonal gesture',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  panAxis: PanAxis.vertical,
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        expect(transformationController.value, equals(Matrix4.identity()));

        // Perform a diagonal drag gesture.
        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
        final TestGesture gesture = await tester.startGesture(childInterior);
        await tester.pump();
        await gesture.moveTo(childOffset);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        // Translation has only happened along the x axis (the default axis when
        // a gesture is perfectly at 45 degrees to the axes).
        final Vector3 translation = transformationController.value.getTranslation();
        expect(translation.y, childOffset.dy - childInterior.dy);
        expect(translation.x, 0.0);
      },
    );

    testWidgets(
      'PanAxis.vertical allows panning in the vertical direction only for vertical leaning gesture',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  panAxis: PanAxis.vertical,
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        expect(transformationController.value, equals(Matrix4.identity()));

        // Perform a horizontally leaning diagonal drag gesture.
        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 10.0);
        final TestGesture gesture = await tester.startGesture(childInterior);
        await tester.pump();
        await gesture.moveTo(childOffset);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        // Translation happened only along the x axis because that's the axis that
        // had been set to the panningDirection parameter.
        final Vector3 translation = transformationController.value.getTranslation();
        expect(translation.y, childOffset.dy - childInterior.dy);
        expect(translation.x, 0.0);
      },
    );

    testWidgets(
      'PanAxis.vertical does not allow panning in horizontal direction on vertical gesture',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  panAxis: PanAxis.vertical,
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        expect(transformationController.value, equals(Matrix4.identity()));

        // Perform a horizontally leaning diagonal drag gesture.
        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 10.0, childOffset.dy + 0.0);
        final TestGesture gesture = await tester.startGesture(childInterior);
        await tester.pump();
        await gesture.moveTo(childOffset);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();

        // Translation didn't happen because the only axis allowed to do panning
        // is the horizontal.
        final Vector3 translation = transformationController.value.getTranslation();
        expect(translation.x, 0.0);
        expect(translation.y, 0.0);
      },
    );

    testWidgets('inertia fling and boundary sliding', (WidgetTester tester) async {
      const double boundaryMargin = 50.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      // Fling the child.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      const Offset flingEnd = Offset(20.0, 15.0);
      await tester.flingFrom(childOffset, flingEnd, 1000.0);
      await tester.pump();

      // Immediately after the gesture, the child has moved to exactly follow
      // the gesture.
      Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, flingEnd.dx);
      expect(translation.y, flingEnd.dy);

      // A short time after the gesture was released, it continues to move with
      // inertia.
      await tester.pump(const Duration(milliseconds: 10));
      translation = transformationController.value.getTranslation();
      expect(translation.x, greaterThan(20.0));
      expect(translation.y, greaterThan(10.0));
      expect(translation.x, lessThan(boundaryMargin));
      expect(translation.y, lessThan(boundaryMargin));

      // It hits the boundary in the x direction first.
      await tester.pump(const Duration(milliseconds: 60));
      translation = transformationController.value.getTranslation();
      expect(translation.x, moreOrLessEquals(boundaryMargin, epsilon: 1e-9));
      expect(translation.y, lessThan(boundaryMargin));
      final double yWhenXHits = translation.y;

      // x is held to the boundary while y slides along.
      await tester.pump(const Duration(milliseconds: 50));
      translation = transformationController.value.getTranslation();
      expect(translation.x, moreOrLessEquals(boundaryMargin, epsilon: 1e-9));
      expect(translation.y, greaterThan(yWhenXHits));
      expect(translation.y, lessThan(boundaryMargin));

      // Eventually it ends up in the corner.
      await tester.pumpAndSettle();
      translation = transformationController.value.getTranslation();
      expect(translation.x, moreOrLessEquals(boundaryMargin, epsilon: 1e-9));
      expect(translation.y, moreOrLessEquals(boundaryMargin, epsilon: 1e-9));
    });

    testWidgets('Scaling automatically causes a centering translation', (
      WidgetTester tester,
    ) async {
      const double boundaryMargin = 50.0;
      const double minScale = 0.1;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                minScale: minScale,
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, 0.0);
      expect(translation.y, 0.0);

      // Pan into the corner of the boundaries.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      const Offset flingEnd = Offset(20.0, 15.0);
      await tester.flingFrom(childOffset, flingEnd, 1000.0);
      await tester.pumpAndSettle();
      translation = transformationController.value.getTranslation();
      expect(translation.x, moreOrLessEquals(boundaryMargin, epsilon: 1e-9));
      expect(translation.y, moreOrLessEquals(boundaryMargin, epsilon: 1e-9));

      // Zoom out so the entire child is visible. The child will also be
      // translated in order to keep it inside the boundaries.
      final Offset childCenter = tester.getCenter(find.byType(SizedBox));
      Offset scaleStart1 = Offset(childCenter.dx - 40.0, childCenter.dy);
      Offset scaleStart2 = Offset(childCenter.dx + 40.0, childCenter.dy);
      Offset scaleEnd1 = Offset(childCenter.dx - 10.0, childCenter.dy);
      Offset scaleEnd2 = Offset(childCenter.dx + 10.0, childCenter.dy);
      TestGesture gesture = await tester.createGesture();
      TestGesture gesture2 = await tester.createGesture();
      await gesture.down(scaleStart1);
      await gesture2.down(scaleStart2);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(transformationController.value.getMaxScaleOnAxis(), lessThan(1.0));
      translation = transformationController.value.getTranslation();
      expect(translation.x, lessThan(boundaryMargin));
      expect(translation.y, lessThan(boundaryMargin));
      expect(translation.x, greaterThan(0.0));
      expect(translation.y, greaterThan(0.0));
      expect(translation.x, moreOrLessEquals(translation.y, epsilon: 1e-9));

      // Zoom in on a point that's not the center, and see that it remains at
      // roughly the same location in the viewport after the zoom.
      scaleStart1 = Offset(childCenter.dx - 50.0, childCenter.dy);
      scaleStart2 = Offset(childCenter.dx - 30.0, childCenter.dy);
      scaleEnd1 = Offset(childCenter.dx - 51.0, childCenter.dy);
      scaleEnd2 = Offset(childCenter.dx - 29.0, childCenter.dy);
      final Offset viewportFocalPoint = Offset(
        childCenter.dx - 40.0 - childOffset.dx,
        childCenter.dy - childOffset.dy,
      );
      final Offset sceneFocalPoint = transformationController.toScene(viewportFocalPoint);
      gesture = await tester.createGesture();
      gesture2 = await tester.createGesture();
      await gesture.down(scaleStart1);
      await gesture2.down(scaleStart2);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      final Offset newSceneFocalPoint = transformationController.toScene(viewportFocalPoint);
      expect(newSceneFocalPoint.dx, moreOrLessEquals(sceneFocalPoint.dx, epsilon: 1.0));
      expect(newSceneFocalPoint.dy, moreOrLessEquals(sceneFocalPoint.dy, epsilon: 1.0));
    });

    testWidgets(
      'Scaling automatically causes a centering translation even when alignPanAxis is set',
      (WidgetTester tester) async {
        const double boundaryMargin = 50.0;
        const double minScale = 0.1;
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  panAxis: PanAxis.aligned,
                  boundaryMargin: const EdgeInsets.all(boundaryMargin),
                  minScale: minScale,
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        Vector3 translation = transformationController.value.getTranslation();
        expect(translation.x, 0.0);
        expect(translation.y, 0.0);

        // Pan into the corner of the boundaries in two gestures, since
        // alignPanAxis prevents diagonal panning.
        final Offset childOffset1 = tester.getTopLeft(find.byType(SizedBox));
        const Offset flingEnd1 = Offset(20.0, 0.0);
        await tester.flingFrom(childOffset1, flingEnd1, 1000.0);
        await tester.pumpAndSettle();
        await tester.pump(const Duration(seconds: 5));
        final Offset childOffset2 = tester.getTopLeft(find.byType(SizedBox));
        const Offset flingEnd2 = Offset(0.0, 15.0);
        await tester.flingFrom(childOffset2, flingEnd2, 1000.0);
        await tester.pumpAndSettle();
        translation = transformationController.value.getTranslation();
        expect(translation.x, moreOrLessEquals(boundaryMargin, epsilon: 1e-9));
        expect(translation.y, moreOrLessEquals(boundaryMargin, epsilon: 1e-9));

        // Zoom out so the entire child is visible. The child will also be
        // translated in order to keep it inside the boundaries.
        final Offset childCenter = tester.getCenter(find.byType(SizedBox));
        Offset scaleStart1 = Offset(childCenter.dx - 40.0, childCenter.dy);
        Offset scaleStart2 = Offset(childCenter.dx + 40.0, childCenter.dy);
        Offset scaleEnd1 = Offset(childCenter.dx - 10.0, childCenter.dy);
        Offset scaleEnd2 = Offset(childCenter.dx + 10.0, childCenter.dy);
        TestGesture gesture = await tester.createGesture();
        TestGesture gesture2 = await tester.createGesture();
        await gesture.down(scaleStart1);
        await gesture2.down(scaleStart2);
        await tester.pump();
        await gesture.moveTo(scaleEnd1);
        await gesture2.moveTo(scaleEnd2);
        await tester.pump();
        await gesture.up();
        await gesture2.up();
        await tester.pumpAndSettle();
        expect(transformationController.value.getMaxScaleOnAxis(), lessThan(1.0));
        translation = transformationController.value.getTranslation();
        expect(translation.x, lessThan(boundaryMargin));
        expect(translation.y, lessThan(boundaryMargin));
        expect(translation.x, greaterThan(0.0));
        expect(translation.y, greaterThan(0.0));
        expect(translation.x, moreOrLessEquals(translation.y, epsilon: 1e-9));

        // Zoom in on a point that's not the center, and see that it remains at
        // roughly the same location in the viewport after the zoom.
        scaleStart1 = Offset(childCenter.dx - 50.0, childCenter.dy);
        scaleStart2 = Offset(childCenter.dx - 30.0, childCenter.dy);
        scaleEnd1 = Offset(childCenter.dx - 51.0, childCenter.dy);
        scaleEnd2 = Offset(childCenter.dx - 29.0, childCenter.dy);
        final Offset viewportFocalPoint = Offset(
          childCenter.dx - 40.0 - childOffset1.dx,
          childCenter.dy - childOffset1.dy,
        );
        final Offset sceneFocalPoint = transformationController.toScene(viewportFocalPoint);
        gesture = await tester.createGesture();
        gesture2 = await tester.createGesture();
        await gesture.down(scaleStart1);
        await gesture2.down(scaleStart2);
        await tester.pump();
        await gesture.moveTo(scaleEnd1);
        await gesture2.moveTo(scaleEnd2);
        await tester.pump();
        await gesture.up();
        await gesture2.up();
        await tester.pumpAndSettle();
        final Offset newSceneFocalPoint = transformationController.toScene(viewportFocalPoint);
        expect(newSceneFocalPoint.dx, moreOrLessEquals(sceneFocalPoint.dx, epsilon: 1.0));
        expect(newSceneFocalPoint.dy, moreOrLessEquals(sceneFocalPoint.dy, epsilon: 1.0));
      },
    );

    testWidgets('Can scale with mouse', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      final Offset center = tester.getCenter(find.byType(InteractiveViewer));
      await scrollAt(center, tester, const Offset(0.0, -20.0));
      await tester.pumpAndSettle();

      expect(transformationController.value.getMaxScaleOnAxis(), greaterThan(1.0));
    });

    testWidgets('Cannot scale with mouse when scale is disabled', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                transformationController: transformationController,
                scaleEnabled: false,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      final Offset center = tester.getCenter(find.byType(InteractiveViewer));
      await scrollAt(center, tester, const Offset(0.0, -20.0));
      await tester.pumpAndSettle();

      expect(transformationController.value.getMaxScaleOnAxis(), equals(1.0));
    });

    testWidgets('Scale with mouse returns onInteraction properties', (WidgetTester tester) async {
      late Offset focalPoint;
      late Offset localFocalPoint;
      late double scaleChange;
      late Velocity currentVelocity;
      late bool calledStart;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                transformationController: transformationController,
                onInteractionStart: (ScaleStartDetails details) {
                  calledStart = true;
                },
                onInteractionUpdate: (ScaleUpdateDetails details) {
                  scaleChange = details.scale;
                  focalPoint = details.focalPoint;
                  localFocalPoint = details.localFocalPoint;
                },
                onInteractionEnd: (ScaleEndDetails details) {
                  currentVelocity = details.velocity;
                },
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      final Offset center = tester.getCenter(find.byType(InteractiveViewer));
      await scrollAt(center, tester, const Offset(0.0, -20.0));
      await tester.pumpAndSettle();
      const Velocity noMovement = Velocity.zero;
      final double afterScaling = transformationController.value.getMaxScaleOnAxis();

      expect(scaleChange, greaterThan(1.0));
      expect(afterScaling, scaleChange);
      expect(currentVelocity, equals(noMovement));
      expect(calledStart, equals(true));
      // Focal points are given in coordinates outside of InteractiveViewer,
      // with local being in relation to the viewport.
      expect(focalPoint, center);
      expect(localFocalPoint, const Offset(100, 100));

      // The scene point is the same as localFocalPoint because the center of
      // the scene is at the center of the viewport.
      final Offset scenePoint = transformationController.toScene(localFocalPoint);
      expect(scenePoint, const Offset(100, 100));
    });

    testWidgets('Scaling amount is equal forth and back with a mouse scroll', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                constrained: false,
                maxScale: 100000,
                minScale: 0.01,
                transformationController: transformationController,
                child: const SizedBox(width: 1000.0, height: 1000.0),
              ),
            ),
          ),
        ),
      );

      final Offset center = tester.getCenter(find.byType(InteractiveViewer));
      await scrollAt(center, tester, const Offset(0.0, -200.0));
      await tester.pumpAndSettle();
      expect(transformationController.value.getMaxScaleOnAxis(), math.exp(200 / 200));
      await scrollAt(center, tester, const Offset(0.0, -200.0));
      await tester.pumpAndSettle();
      // math.exp round the number too short compared to the one in transformationController.
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        closeTo(math.exp(400 / 200), 0.000000000000001),
      );
      await scrollAt(center, tester, const Offset(0.0, 200.0));
      await scrollAt(center, tester, const Offset(0.0, 200.0));
      await tester.pumpAndSettle();
      expect(transformationController.value.getMaxScaleOnAxis(), 1.0);
    });

    testWidgets('onInteraction can be used to get scene point', (WidgetTester tester) async {
      late Offset focalPoint;
      late Offset localFocalPoint;
      late double scaleChange;
      late Velocity currentVelocity;
      late bool calledStart;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                transformationController: transformationController,
                onInteractionStart: (ScaleStartDetails details) {
                  calledStart = true;
                },
                onInteractionUpdate: (ScaleUpdateDetails details) {
                  scaleChange = details.scale;
                  focalPoint = details.focalPoint;
                  localFocalPoint = details.localFocalPoint;
                },
                onInteractionEnd: (ScaleEndDetails details) {
                  currentVelocity = details.velocity;
                },
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      final Offset center = tester.getCenter(find.byType(InteractiveViewer));
      final Offset offCenter = Offset(center.dx - 20.0, center.dy - 20.0);
      await scrollAt(offCenter, tester, const Offset(0.0, -20.0));
      await tester.pumpAndSettle();
      const Velocity noMovement = Velocity.zero;
      final double afterScaling = transformationController.value.getMaxScaleOnAxis();

      expect(scaleChange, greaterThan(1.0));
      expect(afterScaling, scaleChange);
      expect(currentVelocity, equals(noMovement));
      expect(calledStart, equals(true));
      // Focal points are given in coordinates outside of InteractiveViewer,
      // with local being in relation to the viewport.
      expect(focalPoint, offCenter);
      expect(localFocalPoint, const Offset(80, 80));

      // The top left corner of the viewport is not at the top left corner of
      // the scene.
      final Offset scenePoint = transformationController.toScene(Offset.zero);
      expect(scenePoint.dx, greaterThan(0.0));
      expect(scenePoint.dy, greaterThan(0.0));
    });

    testWidgets(
      'onInteraction is called even when disabled (touch)',
      (WidgetTester tester) async {
        bool calledStart = false;
        bool calledUpdate = false;
        bool calledEnd = false;
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  transformationController: transformationController,
                  scaleEnabled: false,
                  onInteractionStart: (ScaleStartDetails details) {
                    calledStart = true;
                  },
                  onInteractionUpdate: (ScaleUpdateDetails details) {
                    calledUpdate = true;
                  },
                  onInteractionEnd: (ScaleEndDetails details) {
                    calledEnd = true;
                  },
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
        TestGesture gesture = await tester.startGesture(childOffset);

        // Attempting to pan doesn't work because it's disabled, but the
        // interaction methods are still called.
        await tester.pump();
        await gesture.moveTo(childInterior);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(transformationController.value, equals(Matrix4.identity()));
        expect(calledStart, isTrue);
        expect(calledUpdate, isTrue);
        expect(calledEnd, isTrue);

        // Attempting to pinch to zoom doesn't work because it's disabled, but the
        // interaction methods are still called.
        calledStart = false;
        calledUpdate = false;
        calledEnd = false;
        final Offset scaleStart1 = childInterior;
        final Offset scaleStart2 = Offset(childInterior.dx + 10.0, childInterior.dy);
        final Offset scaleEnd1 = Offset(childInterior.dx - 10.0, childInterior.dy);
        final Offset scaleEnd2 = Offset(childInterior.dx + 20.0, childInterior.dy);
        gesture = await tester.startGesture(scaleStart1);
        final TestGesture gesture2 = await tester.startGesture(scaleStart2);
        addTearDown(gesture2.removePointer);
        await tester.pump();
        await gesture.moveTo(scaleEnd1);
        await gesture2.moveTo(scaleEnd2);
        await tester.pump();
        await gesture.up();
        await gesture2.up();
        await tester.pumpAndSettle();
        expect(transformationController.value, equals(Matrix4.identity()));
        expect(calledStart, isTrue);
        expect(calledUpdate, isTrue);
        expect(calledEnd, isTrue);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.iOS,
      }),
    );

    testWidgets(
      'onInteraction is called even when disabled (mouse)',
      (WidgetTester tester) async {
        bool calledStart = false;
        bool calledUpdate = false;
        bool calledEnd = false;
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  transformationController: transformationController,
                  scaleEnabled: false,
                  onInteractionStart: (ScaleStartDetails details) {
                    calledStart = true;
                  },
                  onInteractionUpdate: (ScaleUpdateDetails details) {
                    calledUpdate = true;
                  },
                  onInteractionEnd: (ScaleEndDetails details) {
                    calledEnd = true;
                  },
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );

        final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
        final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
        final TestGesture gesture = await tester.startGesture(
          childOffset,
          kind: PointerDeviceKind.mouse,
        );

        // Attempting to pan doesn't work because it's disabled, but the
        // interaction methods are still called.
        await tester.pump();
        await gesture.moveTo(childInterior);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(transformationController.value, equals(Matrix4.identity()));
        expect(calledStart, isTrue);
        expect(calledUpdate, isTrue);
        expect(calledEnd, isTrue);

        // Attempting to scroll with a mouse to zoom doesn't work because it's
        // disabled, but the interaction methods are still called.
        calledStart = false;
        calledUpdate = false;
        calledEnd = false;
        await scrollAt(childInterior, tester, const Offset(0.0, -20.0));
        await tester.pumpAndSettle();
        expect(transformationController.value, equals(Matrix4.identity()));
        expect(calledStart, isTrue);
        expect(calledUpdate, isTrue);
        expect(calledEnd, isTrue);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.macOS,
        TargetPlatform.linux,
        TargetPlatform.windows,
      }),
    );

    testWidgets('viewport changes size', (WidgetTester tester) async {
      addTearDown(tester.view.reset);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                transformationController: transformationController,
                child: Container(),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Attempting to drag to pan doesn't work because the child fits inside
      // the viewport and has a tight boundary.
      final Offset childOffset = tester.getTopLeft(find.byType(Container));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      TestGesture gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));

      // Shrink the size of the screen.
      tester.view.physicalSize = const Size(100.0, 100.0);
      await tester.pump();

      // Attempting to drag to pan still doesn't work, because the image has
      // resized itself to fit the new screen size, and InteractiveViewer has
      // updated its measurements to take that into consideration.
      gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));
    });

    testWidgets('gesture can start as pan and become scale', (WidgetTester tester) async {
      const double boundaryMargin = 50.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, 0.0);
      expect(translation.y, 0.0);

      // Start a pan gesture.
      final Offset childCenter = tester.getCenter(find.byType(SizedBox));
      final TestGesture gesture = await tester.createGesture();
      await gesture.down(childCenter);
      await tester.pump();
      await gesture.moveTo(Offset(childCenter.dx + 5.0, childCenter.dy + 5.0));
      await tester.pump();
      translation = transformationController.value.getTranslation();
      expect(translation.x, greaterThan(0.0));
      expect(translation.y, greaterThan(0.0));

      // Put another finger down and turn it into a scale gesture.
      final TestGesture gesture2 = await tester.createGesture();
      await gesture2.down(Offset(childCenter.dx - 5.0, childCenter.dy - 5.0));
      await tester.pump();
      await gesture.moveTo(Offset(childCenter.dx + 25.0, childCenter.dy + 25.0));
      await gesture2.moveTo(Offset(childCenter.dx - 25.0, childCenter.dy - 25.0));
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(transformationController.value.getMaxScaleOnAxis(), greaterThan(1.0));
    });

    // Regression test for https://github.com/flutter/flutter/issues/65304
    testWidgets('can view beyond boundary when necessary for a small child', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                constrained: false,
                minScale: 1.0,
                maxScale: 1.0,
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // Pinch to zoom does nothing because minScale and maxScale are 1.0.
      final Offset center = tester.getCenter(find.byType(SizedBox));
      final Offset scaleStart1 = Offset(center.dx - 10.0, center.dy - 10.0);
      final Offset scaleStart2 = Offset(center.dx + 10.0, center.dy + 10.0);
      final Offset scaleEnd1 = Offset(center.dx - 20.0, center.dy - 20.0);
      final Offset scaleEnd2 = Offset(center.dx + 20.0, center.dy + 20.0);
      final TestGesture gesture = await tester.createGesture();
      final TestGesture gesture2 = await tester.createGesture();
      await gesture.down(scaleStart1);
      await gesture2.down(scaleStart2);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));
    });

    testWidgets('scale does not jump when wrapped in GestureDetector', (WidgetTester tester) async {
      double? initialScale;
      double? scale;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: GestureDetector(
                onTapUp: (TapUpDetails details) {},
                child: InteractiveViewer(
                  onInteractionUpdate: (ScaleUpdateDetails details) {
                    initialScale ??= details.scale;
                    scale = details.scale;
                  },
                  transformationController: transformationController,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));
      expect(initialScale, null);
      expect(scale, null);

      // Pinch to zoom isn't immediately detected for a small amount of
      // movement due to the GestureDetector.
      final Offset childOffset = tester.getTopLeft(find.byType(SizedBox));
      final Offset childInterior = Offset(childOffset.dx + 20.0, childOffset.dy + 20.0);
      final Offset scaleStart1 = childInterior;
      final Offset scaleStart2 = Offset(childInterior.dx + 10.0, childInterior.dy);
      Offset scaleEnd1 = Offset(childInterior.dx - 10.0, childInterior.dy);
      Offset scaleEnd2 = Offset(childInterior.dx + 20.0, childInterior.dy);
      TestGesture gesture = await tester.createGesture();
      TestGesture gesture2 = await tester.createGesture();
      addTearDown(gesture2.removePointer);
      await gesture.down(scaleStart1);
      await gesture2.down(scaleStart2);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, equals(Matrix4.identity()));
      expect(initialScale, null);
      expect(scale, null);

      // Pinch to zoom for a larger amount is detected. It starts smoothly at
      // 1.0 despite the fact that the gesture has already moved a bit.
      scaleEnd1 = Offset(childInterior.dx - 38.0, childInterior.dy);
      scaleEnd2 = Offset(childInterior.dx + 48.0, childInterior.dy);
      gesture = await tester.createGesture();
      gesture2 = await tester.createGesture();
      addTearDown(gesture2.removePointer);
      await gesture.down(scaleStart1);
      await gesture2.down(scaleStart2);
      await tester.pump();
      await gesture.moveTo(scaleEnd1);
      await gesture2.moveTo(scaleEnd2);
      await tester.pump();
      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();
      expect(initialScale, 1.0);
      expect(scale, greaterThan(1.0));
      expect(transformationController.value.getMaxScaleOnAxis(), greaterThan(1.0));
    });

    testWidgets('Check if ClipRect is present in the tree', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                constrained: false,
                clipBehavior: Clip.none,
                minScale: 1.0,
                maxScale: 1.0,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      final RenderClipRect renderClip = tester.allRenderObjects.whereType<RenderClipRect>().first;
      expect(renderClip.clipBehavior, equals(Clip.none));

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                constrained: false,
                minScale: 1.0,
                maxScale: 1.0,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(find.byType(ClipRect), findsOneWidget);
    });

    testWidgets('builder can change widgets that are off-screen', (WidgetTester tester) async {
      const double childHeight = 10.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: SizedBox(
                key: const Key('outer box'),
                height: 50.0,
                child: InteractiveViewer.builder(
                  transformationController: transformationController,
                  scaleEnabled: false,
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  // Build visible children green, off-screen children red.
                  builder: (BuildContext context, Quad viewportQuad) {
                    final Rect viewport = _axisAlignedBoundingBox(viewportQuad);
                    final List<Container> children = <Container>[];
                    for (int i = 0; i < 10; i++) {
                      final double childTop = i * childHeight;
                      final double childBottom = childTop + childHeight;
                      final bool visible =
                          (childBottom >= viewport.top && childBottom <= viewport.bottom) ||
                          (childTop >= viewport.top && childTop <= viewport.bottom);
                      children.add(
                        Container(height: childHeight, color: visible ? Colors.green : Colors.red),
                      );
                    }
                    return Column(children: children);
                  },
                ),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value, equals(Matrix4.identity()));

      // The first six are partially visible and therefore green.
      int i = 0;
      for (final Element element in find.byType(Container, skipOffstage: false).evaluate()) {
        final Container container = element.widget as Container;
        if (i < 6) {
          expect(container.color, Colors.green);
        } else {
          expect(container.color, Colors.red);
        }
        i++;
      }

      // Drag to pan down past the first child.
      final Offset childOffset = tester.getTopLeft(find.byKey(const Key('outer box')));
      const double translationY = 15.0;
      final Offset childInterior = Offset(childOffset.dx, childOffset.dy + translationY);
      final TestGesture gesture = await tester.startGesture(childInterior);
      await tester.pump();
      await gesture.moveTo(childOffset);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(transformationController.value, isNot(Matrix4.identity()));
      expect(transformationController.value.getTranslation().y, -translationY);

      // After scrolling down a bit, the first child is not visible, the next
      // six are, and the final three are not.
      i = 0;
      for (final Element element in find.byType(Container, skipOffstage: false).evaluate()) {
        final Container container = element.widget as Container;
        if (i > 0 && i < 7) {
          expect(container.color, Colors.green);
        } else {
          expect(container.color, Colors.red);
        }
        i++;
      }
    });

    // Accessing the intrinsic size of a LayoutBuilder throws an error, so
    // InteractiveViewer only uses a LayoutBuilder when it's needed by
    // InteractiveViewer.builder.
    testWidgets('LayoutBuilder is only used for InteractiveViewer.builder', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(child: const SizedBox(width: 200.0, height: 200.0)),
            ),
          ),
        ),
      );

      expect(find.byType(LayoutBuilder), findsNothing);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer.builder(
                builder: (BuildContext context, Quad viewport) {
                  return const SizedBox(width: 200.0, height: 200.0);
                },
              ),
            ),
          ),
        ),
      );

      expect(find.byType(LayoutBuilder), findsOneWidget);
    });

    testWidgets('scaleFactor', (WidgetTester tester) async {
      const double scrollAmount = 30.0;
      Future<void> pumpScaleFactor(double scaleFactor) {
        return tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: InteractiveViewer(
                  boundaryMargin: const EdgeInsets.all(double.infinity),
                  transformationController: transformationController,
                  scaleFactor: scaleFactor,
                  child: const SizedBox(width: 200.0, height: 200.0),
                ),
              ),
            ),
          ),
        );
      }

      // Start with the default scaleFactor.
      await pumpScaleFactor(200.0);

      expect(transformationController.value, equals(Matrix4.identity()));

      // Zoom out. The scale decreases.
      final Offset center = tester.getCenter(find.byType(InteractiveViewer));
      await scrollAt(center, tester, const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      final double scaleZoomedOut = transformationController.value.getMaxScaleOnAxis();
      expect(scaleZoomedOut, lessThan(1.0));

      // Zoom in. The scale increases.
      await scrollAt(center, tester, const Offset(0.0, -scrollAmount));
      await tester.pumpAndSettle();
      final double scaleZoomedIn = transformationController.value.getMaxScaleOnAxis();
      expect(scaleZoomedIn, greaterThan(scaleZoomedOut));

      // Reset and decrease the scaleFactor below the default, so that scaling
      // will happen more quickly.
      transformationController.value = Matrix4.identity();
      await pumpScaleFactor(100.0);

      // Zoom out. The scale decreases more quickly than with the default
      // (higher) scaleFactor.
      await scrollAt(center, tester, const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      final double scaleLowZoomedOut = transformationController.value.getMaxScaleOnAxis();
      expect(scaleLowZoomedOut, lessThan(1.0));
      expect(scaleLowZoomedOut, lessThan(scaleZoomedOut));

      // Zoom in. The scale increases more quickly than with the default
      // (higher) scaleFactor.
      await scrollAt(center, tester, const Offset(0.0, -scrollAmount));
      await tester.pumpAndSettle();
      final double scaleLowZoomedIn = transformationController.value.getMaxScaleOnAxis();
      expect(scaleLowZoomedIn, greaterThan(scaleLowZoomedOut));
      expect(scaleLowZoomedIn - scaleLowZoomedOut, greaterThan(scaleZoomedIn - scaleZoomedOut));

      // Reset and increase the scaleFactor above the default.
      transformationController.value = Matrix4.identity();
      await pumpScaleFactor(400.0);

      // Zoom out. The scale decreases, but not by as much as with the default
      // (higher) scaleFactor.
      await scrollAt(center, tester, const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      final double scaleHighZoomedOut = transformationController.value.getMaxScaleOnAxis();
      expect(scaleHighZoomedOut, lessThan(1.0));
      expect(scaleHighZoomedOut, greaterThan(scaleZoomedOut));

      // Zoom in. The scale increases, but not by as much as with the default
      // (higher) scaleFactor.
      await scrollAt(center, tester, const Offset(0.0, -scrollAmount));
      await tester.pumpAndSettle();
      final double scaleHighZoomedIn = transformationController.value.getMaxScaleOnAxis();
      expect(scaleHighZoomedIn, greaterThan(scaleHighZoomedOut));
      expect(scaleHighZoomedIn - scaleHighZoomedOut, lessThan(scaleZoomedIn - scaleZoomedOut));
    });

    testWidgets('alignment argument is used properly', (WidgetTester tester) async {
      const Alignment alignment = Alignment.center;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(body: InteractiveViewer(alignment: alignment, child: Container())),
        ),
      );

      final Transform transform = tester.firstWidget(find.byType(Transform));
      expect(transform.alignment, alignment);
    });

    testWidgets('interactionEndFrictionCoefficient', (WidgetTester tester) async {
      // Use the default interactionEndFrictionCoefficient.
      final TransformationController transformationController1 = TransformationController();
      addTearDown(transformationController1.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: SizedBox(
              width: 200,
              height: 200,
              child: InteractiveViewer(
                constrained: false,
                transformationController: transformationController1,
                child: const SizedBox(width: 2000.0, height: 2000.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController1.value, equals(Matrix4.identity()));

      await tester.flingFrom(const Offset(100, 100), const Offset(0, -50), 100.0);
      await tester.pumpAndSettle();
      final Vector3 translation1 = transformationController1.value.getTranslation();
      expect(translation1.y, lessThan(-58.0));

      // Next try a custom interactionEndFrictionCoefficient.
      final TransformationController transformationController2 = TransformationController();
      addTearDown(transformationController2.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: SizedBox(
              width: 200,
              height: 200,
              child: InteractiveViewer(
                constrained: false,
                interactionEndFrictionCoefficient: 0.01,
                transformationController: transformationController2,
                child: const SizedBox(width: 2000.0, height: 2000.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController2.value, equals(Matrix4.identity()));

      await tester.flingFrom(const Offset(100, 100), const Offset(0, -50), 100.0);
      await tester.pumpAndSettle();
      final Vector3 translation2 = transformationController2.value.getTranslation();

      // The coefficient 0.01 is greater than the default of 0.0000135,
      // so the translation comes to a stop more quickly.
      expect(translation2.y, lessThan(translation1.y));
    });

    testWidgets('discrete scroll pointer events', (WidgetTester tester) async {
      const double boundaryMargin = 50.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value.getMaxScaleOnAxis(), 1.0);
      Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, 0);
      expect(translation.y, 0);

      // Send a mouse scroll event, it should cause a scale.
      final TestPointer mouse = TestPointer(1, PointerDeviceKind.mouse);
      await tester.sendEventToBinding(mouse.hover(tester.getCenter(find.byType(SizedBox))));
      await tester.sendEventToBinding(mouse.scroll(const Offset(300, -200)));
      await tester.pump();
      expect(transformationController.value.getMaxScaleOnAxis(), 2.5);
      translation = transformationController.value.getTranslation();
      // Will be translated to maintain centering.
      expect(translation.x, -150);
      expect(translation.y, -150);

      // Send a trackpad scroll event, it should cause a pan and no scale.
      final TestPointer trackpad = TestPointer(1, PointerDeviceKind.trackpad);
      await tester.sendEventToBinding(trackpad.hover(tester.getCenter(find.byType(SizedBox))));
      await tester.sendEventToBinding(trackpad.scroll(const Offset(100, -25)));
      await tester.pump();
      expect(transformationController.value.getMaxScaleOnAxis(), 2.5);
      translation = transformationController.value.getTranslation();
      expect(translation.x, -250);
      expect(translation.y, -125);
    });

    testWidgets('discrete scale pointer event', (WidgetTester tester) async {
      const double boundaryMargin = 50.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                transformationController: transformationController,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value.getMaxScaleOnAxis(), 1.0);

      // Send a scale event.
      final TestPointer pointer = TestPointer(1, PointerDeviceKind.trackpad);
      await tester.sendEventToBinding(pointer.hover(tester.getCenter(find.byType(SizedBox))));
      await tester.sendEventToBinding(pointer.scale(1.5));
      await tester.pump();
      expect(transformationController.value.getMaxScaleOnAxis(), 1.5);

      // Send another scale event.
      await tester.sendEventToBinding(pointer.scale(1.5));
      await tester.pump();
      expect(transformationController.value.getMaxScaleOnAxis(), 2.25);

      // Send another scale event.
      await tester.sendEventToBinding(pointer.scale(1.5));
      await tester.pump();
      expect(transformationController.value.getMaxScaleOnAxis(), 2.5); // capped at maxScale (2.5)
    });

    testWidgets('trackpadScrollCausesScale', (WidgetTester tester) async {
      const double boundaryMargin = 50.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                transformationController: transformationController,
                trackpadScrollCausesScale: true,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value.getMaxScaleOnAxis(), 1.0);

      // Send a vertical scroll.
      final TestPointer pointer = TestPointer(1, PointerDeviceKind.trackpad);
      final Offset center = tester.getCenter(find.byType(SizedBox));
      await tester.sendEventToBinding(pointer.panZoomStart(center));
      await tester.pump();
      expect(transformationController.value.getMaxScaleOnAxis(), 1.0);
      await tester.sendEventToBinding(pointer.panZoomUpdate(center, pan: const Offset(0, -81)));
      await tester.pump();
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        moreOrLessEquals(1.499302500056767),
      );

      // Send a horizontal scroll (should have no effect).
      await tester.sendEventToBinding(pointer.panZoomUpdate(center, pan: const Offset(81, -81)));
      await tester.pump();
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        moreOrLessEquals(1.499302500056767),
      );
    });

    testWidgets('trackpad pointer scroll events cause scale', (WidgetTester tester) async {
      const double boundaryMargin = 50.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                transformationController: transformationController,
                trackpadScrollCausesScale: true,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value.getMaxScaleOnAxis(), 1.0);

      // Send a vertical scroll.
      final TestPointer pointer = TestPointer(1, PointerDeviceKind.trackpad);
      final Offset center = tester.getCenter(find.byType(SizedBox));
      Offset scrollAmnt = const Offset(0, -138.0);
      await tester.sendEventToBinding(pointer.hover(center));
      await tester.pump();
      expect(transformationController.value.getMaxScaleOnAxis(), 1.0);
      await tester.sendEventToBinding(pointer.scroll(scrollAmnt));
      await tester.pump();
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        moreOrLessEquals(1.9937155332430823),
      );

      // Scroll should not have translated the box, so the box should still be at the
      // center of the InteractiveViewer.
      Vector3 translation = transformationController.value.getTranslation();
      expect(translation.x, moreOrLessEquals(-99.37155332430822));
      expect(translation.y, moreOrLessEquals(-99.37155332430822));

      // Send a horizontal scroll.
      scrollAmnt = const Offset(-138, 0);
      await tester.sendEventToBinding(pointer.scroll(scrollAmnt));
      await tester.pump();

      // Horizontal scroll should not cause a scale change.
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        moreOrLessEquals(1.9937155332430823),
      );

      // Horizontal scroll should not have changed the translation of the box.
      translation = transformationController.value.getTranslation();
      expect(translation.x, moreOrLessEquals(-99.37155332430822));
      expect(translation.y, moreOrLessEquals(-99.37155332430822));
    });

    testWidgets('Scaling inertia', (WidgetTester tester) async {
      const double boundaryMargin = 50.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: InteractiveViewer(
                boundaryMargin: const EdgeInsets.all(boundaryMargin),
                transformationController: transformationController,
                trackpadScrollCausesScale: true,
                child: const SizedBox(width: 200.0, height: 200.0),
              ),
            ),
          ),
        ),
      );

      expect(transformationController.value.getMaxScaleOnAxis(), 1.0);

      // Send a vertical scroll fling, which will cause inertia.
      await tester.trackpadFling(find.byType(InteractiveViewer), const Offset(0, -100), 3000);
      await tester.pump();
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        moreOrLessEquals(1.6487212707001282),
      );
      await tester.pump(const Duration(milliseconds: 80));
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        moreOrLessEquals(1.7966838346780103),
      );
      await tester.pumpAndSettle();
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        moreOrLessEquals(1.9984509673751225),
      );
      await tester.pump(const Duration(seconds: 10));
      expect(
        transformationController.value.getMaxScaleOnAxis(),
        moreOrLessEquals(1.9984509673751225),
      );
    });
  });

  group('getNearestPointOnLine', () {
    test('does not modify parameters', () {
      final Vector3 point = Vector3(5.0, 5.0, 0.0);
      final Vector3 a = Vector3(0.0, 0.0, 0.0);
      final Vector3 b = Vector3(10.0, 0.0, 0.0);

      final Vector3 closestPoint = InteractiveViewer.getNearestPointOnLine(point, a, b);

      expect(closestPoint, Vector3(5.0, 0.0, 0.0));
      expect(point, Vector3(5.0, 5.0, 0.0));
      expect(a, Vector3(0.0, 0.0, 0.0));
      expect(b, Vector3(10.0, 0.0, 0.0));
    });

    test('simple example', () {
      final Vector3 point = Vector3(0.0, 5.0, 0.0);
      final Vector3 a = Vector3(0.0, 0.0, 0.0);
      final Vector3 b = Vector3(5.0, 5.0, 0.0);

      expect(InteractiveViewer.getNearestPointOnLine(point, a, b), Vector3(2.5, 2.5, 0.0));
    });

    test('closest to a', () {
      final Vector3 point = Vector3(-1.0, -1.0, 0.0);
      final Vector3 a = Vector3(0.0, 0.0, 0.0);
      final Vector3 b = Vector3(5.0, 5.0, 0.0);

      expect(InteractiveViewer.getNearestPointOnLine(point, a, b), a);
    });

    test('closest to b', () {
      final Vector3 point = Vector3(6.0, 6.0, 0.0);
      final Vector3 a = Vector3(0.0, 0.0, 0.0);
      final Vector3 b = Vector3(5.0, 5.0, 0.0);

      expect(InteractiveViewer.getNearestPointOnLine(point, a, b), b);
    });

    test('point already on the line returns the point', () {
      final Vector3 point = Vector3(2.0, 2.0, 0.0);
      final Vector3 a = Vector3(0.0, 0.0, 0.0);
      final Vector3 b = Vector3(5.0, 5.0, 0.0);

      expect(InteractiveViewer.getNearestPointOnLine(point, a, b), point);
    });

    test('real example', () {
      final Vector3 point = Vector3(-436.9, 433.6, 0.0);
      final Vector3 a = Vector3(-1114.0, -60.3, 0.0);
      final Vector3 b = Vector3(288.8, 432.7, 0.0);

      final Vector3 closestPoint = InteractiveViewer.getNearestPointOnLine(point, a, b);

      expect(closestPoint.x, moreOrLessEquals(-356.8, epsilon: 0.1));
      expect(closestPoint.y, moreOrLessEquals(205.8, epsilon: 0.1));
    });
  });

  group('getAxisAlignedBoundingBox', () {
    test('rectangle already axis aligned returns the rectangle', () {
      final Quad quad = Quad.points(
        Vector3(0.0, 0.0, 0.0),
        Vector3(10.0, 0.0, 0.0),
        Vector3(10.0, 10.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
      );

      final Quad aabb = InteractiveViewer.getAxisAlignedBoundingBox(quad);

      expect(aabb.point0, quad.point0);
      expect(aabb.point1, quad.point1);
      expect(aabb.point2, quad.point2);
      expect(aabb.point3, quad.point3);
    });

    test('rectangle rotated by 45 degrees', () {
      final Quad quad = Quad.points(
        Vector3(0.0, 5.0, 0.0),
        Vector3(5.0, 10.0, 0.0),
        Vector3(10.0, 5.0, 0.0),
        Vector3(5.0, 0.0, 0.0),
      );

      final Quad aabb = InteractiveViewer.getAxisAlignedBoundingBox(quad);

      expect(aabb.point0, Vector3(0.0, 0.0, 0.0));
      expect(aabb.point1, Vector3(10.0, 0.0, 0.0));
      expect(aabb.point2, Vector3(10.0, 10.0, 0.0));
      expect(aabb.point3, Vector3(0.0, 10.0, 0.0));
    });

    test('rectangle rotated very slightly', () {
      final Quad quad = Quad.points(
        Vector3(0.0, 1.0, 0.0),
        Vector3(1.0, 11.0, 0.0),
        Vector3(11.0, 9.0, 0.0),
        Vector3(9.0, -1.0, 0.0),
      );

      final Quad aabb = InteractiveViewer.getAxisAlignedBoundingBox(quad);

      expect(aabb.point0, Vector3(0.0, -1.0, 0.0));
      expect(aabb.point1, Vector3(11.0, -1.0, 0.0));
      expect(aabb.point2, Vector3(11.0, 11.0, 0.0));
      expect(aabb.point3, Vector3(0.0, 11.0, 0.0));
    });

    test('example from hexagon board', () {
      final Quad quad = Quad.points(
        Vector3(-462.7, 165.9, 0.0),
        Vector3(690.6, -576.7, 0.0),
        Vector3(1188.1, 196.0, 0.0),
        Vector3(34.9, 938.6, 0.0),
      );

      final Quad aabb = InteractiveViewer.getAxisAlignedBoundingBox(quad);

      expect(aabb.point0, Vector3(-462.7, -576.7, 0.0));
      expect(aabb.point1, Vector3(1188.1, -576.7, 0.0));
      expect(aabb.point2, Vector3(1188.1, 938.6, 0.0));
      expect(aabb.point3, Vector3(-462.7, 938.6, 0.0));
    });
  });

  group('pointIsInside', () {
    test('inside', () {
      final Quad quad = Quad.points(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(10.0, 10.0, 0.0),
        Vector3(10.0, 0.0, 0.0),
      );
      final Vector3 point = Vector3(5.0, 5.0, 0.0);

      expect(InteractiveViewer.pointIsInside(point, quad), true);
    });

    test('outside', () {
      final Quad quad = Quad.points(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(10.0, 10.0, 0.0),
        Vector3(10.0, 0.0, 0.0),
      );
      final Vector3 point = Vector3(12.0, 0.0, 0.0);

      expect(InteractiveViewer.pointIsInside(point, quad), false);
    });

    test('on the edge', () {
      final Quad quad = Quad.points(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(10.0, 10.0, 0.0),
        Vector3(10.0, 0.0, 0.0),
      );
      final Vector3 point = Vector3(0.0, 0.0, 0.0);

      expect(InteractiveViewer.pointIsInside(point, quad), true);
    });
  });

  group('getNearestPointInside', () {
    test('point already inside quad', () {
      final Vector3 point = Vector3(5.0, 5.0, 0.0);
      final Quad quad = Quad.points(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(10.0, 10.0, 0.0),
        Vector3(10.0, 0.0, 0.0),
      );

      final Vector3 nearestPoint = InteractiveViewer.getNearestPointInside(point, quad);

      expect(nearestPoint, point);
    });

    test('axis aligned quad', () {
      final Vector3 point = Vector3(5.0, 15.0, 0.0);
      final Quad quad = Quad.points(
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.0, 10.0, 0.0),
        Vector3(10.0, 10.0, 0.0),
        Vector3(10.0, 0.0, 0.0),
      );

      final Vector3 nearestPoint = InteractiveViewer.getNearestPointInside(point, quad);

      expect(nearestPoint, Vector3(5.0, 10.0, 0.0));
    });

    test('not axis aligned quad', () {
      final Vector3 point = Vector3(5.0, 15.0, 0.0);
      final Quad quad = Quad.points(
        Vector3(0.0, 0.0, 0.0),
        Vector3(2.0, 10.0, 0.0),
        Vector3(12.0, 12.0, 0.0),
        Vector3(10.0, 2.0, 0.0),
      );

      final Vector3 nearestPoint = InteractiveViewer.getNearestPointInside(point, quad);

      expect(nearestPoint.x, moreOrLessEquals(5.8, epsilon: 0.1));
      expect(nearestPoint.y, moreOrLessEquals(10.8, epsilon: 0.1));
    });
  });
}

Rect _axisAlignedBoundingBox(Quad quad) {
  double? xMin;
  double? xMax;
  double? yMin;
  double? yMax;
  for (final Vector3 point in <Vector3>[quad.point0, quad.point1, quad.point2, quad.point3]) {
    if (xMin == null || point.x < xMin) {
      xMin = point.x;
    }
    if (xMax == null || point.x > xMax) {
      xMax = point.x;
    }
    if (yMin == null || point.y < yMin) {
      yMin = point.y;
    }
    if (yMax == null || point.y > yMax) {
      yMax = point.y;
    }
  }
  return Rect.fromLTRB(xMin!, yMin!, xMax!, yMax!);
}
