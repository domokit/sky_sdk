// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

const Widget testChild = SizedBox(
  height: 100.0,
  width: 100.0,
  child: DecoratedBox(
    decoration: BoxDecoration(color: CupertinoColors.activeBlue),
  ),
);

void main() {
  testWidgets('CupertinoCollapsible hides child when fully collapsed', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
            child: CupertinoCollapsible(
            isExpanded: false,
            child: testChild,
          ),
        ),
      ),
    );

    expect(find.byWidget(testChild), findsOneWidget);
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 0.0);

    semantics.dispose();
  });

  testWidgets('CupertinoCollapsible paints child when fully expanded', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoCollapsible(
            child: testChild,
          ),
        ),
      ),
    );

    expect(find.byWidget(testChild), findsOneWidget);
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 100.0);

    semantics.dispose();
  });

  testWidgets('CupertinoCollapsible paints child while animating', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    Widget buildCollapsible({bool value = false}) {
      return CupertinoApp(
        home: Center(
          child: CupertinoCollapsible(
            isExpanded: value,
            child: testChild,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCollapsible());
    expect(find.byWidget(testChild).hitTestable(), findsNothing);

    await tester.pumpWidget(buildCollapsible(value: true));
    await tester.pump(const Duration(milliseconds: 100));
    expect(find.byWidget(testChild).hitTestable(), findsOneWidget);

    semantics.dispose();
  });
  testWidgets('CupertinoCollapsible animates height', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    Widget buildCollapsible({bool value = false}) {
      return CupertinoApp(
        home: Center(
          child: CupertinoCollapsible(
            isExpanded: value,
            child: testChild,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCollapsible());
    expect(find.byWidget(testChild), findsOneWidget);
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 0.0);

    await tester.pumpWidget(buildCollapsible(value: true));
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 0.0);

    await tester.pump(const Duration(milliseconds: 100));
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 50.0);

    await tester.pump(const Duration(milliseconds: 100));
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 100.0);

    semantics.dispose();
  });

  testWidgets('CupertinoCollapsible animates height with custom duration', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    Widget buildCollapsible({bool value = false}) {
      return CupertinoApp(
        home: Center(
          child: CupertinoCollapsible(
            isExpanded: value,
            animationStyle: AnimationStyle(
              duration: const Duration(milliseconds: 100),
              curve: Curves.linear,
            ),
            child: testChild,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCollapsible());
    expect(find.byWidget(testChild), findsOneWidget);
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 0.0);

    await tester.pumpWidget(buildCollapsible(value: true));
    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 50.0);

    await tester.pump(const Duration(milliseconds: 50));
    expect(tester.getSize(find.byType(CupertinoCollapsible)).height, 100.0);

    semantics.dispose();
  });
}
