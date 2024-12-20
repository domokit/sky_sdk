// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  group('Available semantic scroll actions', () {
    // Regression tests for https://github.com/flutter/flutter/issues/52032.

    const int itemCount = 10;
    const double itemHeight = 150.0;

    testWidgets('forward vertical', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListView.builder(
            controller: controller,
            itemCount: itemCount,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(height: itemHeight, child: Text('Tile $index'));
            },
          ),
        ),
      );

      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
        ),
      );

      // Jump to the end.
      controller.jumpTo(itemCount * itemHeight);
      await tester.pumpAndSettle();
      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.scrollDown, SemanticsAction.scrollToOffset],
        ),
      );

      semantics.dispose();
    });

    testWidgets('reverse vertical', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListView.builder(
            reverse: true,
            controller: controller,
            itemCount: itemCount,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(height: itemHeight, child: Text('Tile $index'));
            },
          ),
        ),
      );

      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.scrollDown, SemanticsAction.scrollToOffset],
        ),
      );

      // Jump to the end.
      controller.jumpTo(itemCount * itemHeight);
      await tester.pumpAndSettle();
      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollToOffset],
        ),
      );

      semantics.dispose();
    });

    testWidgets('forward horizontal', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListView.builder(
            scrollDirection: Axis.horizontal,
            controller: controller,
            itemCount: itemCount,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(height: itemHeight, child: Text('Tile $index'));
            },
          ),
        ),
      );

      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.scrollLeft, SemanticsAction.scrollToOffset],
        ),
      );

      // Jump to the end.
      controller.jumpTo(itemCount * itemHeight);
      await tester.pumpAndSettle();
      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.scrollRight, SemanticsAction.scrollToOffset],
        ),
      );

      semantics.dispose();
    });

    testWidgets('reverse horizontal', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListView.builder(
            scrollDirection: Axis.horizontal,
            reverse: true,
            controller: controller,
            itemCount: itemCount,
            itemBuilder: (BuildContext context, int index) {
              return SizedBox(height: itemHeight, child: Text('Tile $index'));
            },
          ),
        ),
      );

      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.scrollRight, SemanticsAction.scrollToOffset],
        ),
      );

      // Jump to the end.
      controller.jumpTo(itemCount * itemHeight);
      await tester.pumpAndSettle();
      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.scrollLeft, SemanticsAction.scrollToOffset],
        ),
      );

      semantics.dispose();
    });
  });
}
