// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/drag_target/draggable.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Finder findContainerWith({required Finder child, required Color color}) {
    return find.ancestor(
      of: child,
      matching: find.byWidgetPredicate(
        (Widget widget) => widget is Container && widget.color == color,
      ),
    );
  }

  testWidgets('Verify initial state', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DraggableExampleApp());

    expect(find.text('Draggable Sample'), findsOneWidget);

    expect(
      findContainerWith(color: Colors.lightGreenAccent, child: find.text('Draggable')),
      findsOneWidget,
    );

    expect(
      findContainerWith(color: Colors.cyan, child: find.text('Value is updated to: 0')),
      findsOneWidget,
    );
  });

  testWidgets('Verify correct containers are displayed while dragging', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.DraggableExampleApp());

    final Finder idleContainer = findContainerWith(
      color: Colors.lightGreenAccent,
      child: find.text('Draggable'),
    );
    final Finder draggingContainer = findContainerWith(
      color: Colors.pinkAccent,
      child: find.text('Child When Dragging'),
    );
    final Finder feedbackContainer = findContainerWith(
      color: Colors.deepOrange,
      child: find.byIcon(Icons.directions_run),
    );

    expect(idleContainer, findsOneWidget);
    expect(draggingContainer, findsNothing);
    expect(feedbackContainer, findsNothing);

    final TestGesture gesture = await tester.startGesture(tester.getCenter(idleContainer));
    await tester.pump();

    expect(idleContainer, findsNothing);
    expect(draggingContainer, findsOneWidget);
    expect(feedbackContainer, findsOneWidget);

    await gesture.moveBy(const Offset(200, 0));
    await tester.pump();

    expect(idleContainer, findsNothing);
    expect(draggingContainer, findsOneWidget);
    expect(feedbackContainer, findsOneWidget);

    await gesture.up();
    await tester.pump();

    expect(idleContainer, findsOneWidget);
    expect(draggingContainer, findsNothing);
    expect(feedbackContainer, findsNothing);
  });

  testWidgets('Dropping Draggable over DragTarget updates the counter', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.DraggableExampleApp());

    final Finder draggable = find.byType(Draggable<int>);
    final Finder target = find.byType(DragTarget<int>);

    int counter = 0;

    for (int i = 0; i < 5; i++) {
      final TestGesture gesture = await tester.startGesture(tester.getCenter(draggable));
      await gesture.moveTo(tester.getCenter(target));
      await gesture.up();
      await tester.pump();

      counter += 10;

      expect(
        findContainerWith(color: Colors.cyan, child: find.text('Value is updated to: $counter')),
        findsOneWidget,
      );
    }
  });
}
