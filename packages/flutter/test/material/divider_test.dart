// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Material3 - Divider control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(theme: ThemeData(useMaterial3: true), home: const Center(child: Divider())),
    );
    final RenderBox box = tester.firstRenderObject(find.byType(Divider));
    expect(box.size.height, 16.0);
    final Container container = tester.widget(find.byType(Container));
    final BoxDecoration decoration = container.decoration! as BoxDecoration;
    expect(decoration.border!.bottom.width, 1.0);
  });

  testWidgets('Material2 - Divider control test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(theme: ThemeData(useMaterial3: false), home: const Center(child: Divider())),
    );
    final RenderBox box = tester.firstRenderObject(find.byType(Divider));
    expect(box.size.height, 16.0);
    final Container container = tester.widget(find.byType(Container));
    final BoxDecoration decoration = container.decoration! as BoxDecoration;
    expect(decoration.border!.bottom.width, 0.0);
  });

  testWidgets('Divider custom thickness', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Divider(thickness: 5.0)),
      ),
    );
    final Container container = tester.widget(find.byType(Container));
    final BoxDecoration decoration = container.decoration! as BoxDecoration;
    expect(decoration.border!.bottom.width, 5.0);
  });

  testWidgets('Horizontal divider custom indentation', (WidgetTester tester) async {
    const double customIndent = 10.0;
    Rect dividerRect;
    Rect lineRect;

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Divider(indent: customIndent)),
      ),
    );
    // The divider line is drawn with a DecoratedBox with a border
    dividerRect = tester.getRect(find.byType(Divider));
    lineRect = tester.getRect(find.byType(DecoratedBox));
    expect(lineRect.left, dividerRect.left + customIndent);
    expect(lineRect.right, dividerRect.right);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Divider(endIndent: customIndent)),
      ),
    );
    dividerRect = tester.getRect(find.byType(Divider));
    lineRect = tester.getRect(find.byType(DecoratedBox));
    expect(lineRect.left, dividerRect.left);
    expect(lineRect.right, dividerRect.right - customIndent);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: Divider(indent: customIndent, endIndent: customIndent)),
      ),
    );
    dividerRect = tester.getRect(find.byType(Divider));
    lineRect = tester.getRect(find.byType(DecoratedBox));
    expect(lineRect.left, dividerRect.left + customIndent);
    expect(lineRect.right, dividerRect.right - customIndent);
  });

  testWidgets('Material3 - Vertical Divider Test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: const Center(child: VerticalDivider()),
      ),
    );
    final RenderBox box = tester.firstRenderObject(find.byType(VerticalDivider));
    expect(box.size.width, 16.0);
    final Container container = tester.widget(find.byType(Container));
    final BoxDecoration decoration = container.decoration! as BoxDecoration;
    final Border border = decoration.border! as Border;
    expect(border.left.width, 1.0);
  });

  testWidgets('Material2 - Vertical Divider Test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Center(child: VerticalDivider()),
      ),
    );
    final RenderBox box = tester.firstRenderObject(find.byType(VerticalDivider));
    expect(box.size.width, 16.0);
    final Container container = tester.widget(find.byType(Container));
    final BoxDecoration decoration = container.decoration! as BoxDecoration;
    final Border border = decoration.border! as Border;
    expect(border.left.width, 0.0);
  });

  testWidgets('Divider custom thickness', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: VerticalDivider(thickness: 5.0)),
      ),
    );
    final Container container = tester.widget(find.byType(Container));
    final BoxDecoration decoration = container.decoration! as BoxDecoration;
    final Border border = decoration.border! as Border;
    expect(border.left.width, 5.0);
  });

  testWidgets('Vertical Divider Test 2', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Material(
          child: SizedBox(
            height: 24.0,
            child: Row(children: <Widget>[Text('Hey.'), VerticalDivider()]),
          ),
        ),
      ),
    );
    final RenderBox box = tester.firstRenderObject(find.byType(VerticalDivider));
    final RenderBox containerBox = tester.firstRenderObject(find.byType(Container).last);

    expect(box.size.width, 16.0);
    expect(containerBox.size.height, 600.0);
    expect(find.byType(VerticalDivider), paints..path(strokeWidth: 0.0));
  });

  testWidgets('Vertical divider custom indentation', (WidgetTester tester) async {
    const double customIndent = 10.0;
    Rect dividerRect;
    Rect lineRect;

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: VerticalDivider(indent: customIndent)),
      ),
    );
    // The divider line is drawn with a DecoratedBox with a border
    dividerRect = tester.getRect(find.byType(VerticalDivider));
    lineRect = tester.getRect(find.byType(DecoratedBox));
    expect(lineRect.top, dividerRect.top + customIndent);
    expect(lineRect.bottom, dividerRect.bottom);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: VerticalDivider(endIndent: customIndent)),
      ),
    );
    dividerRect = tester.getRect(find.byType(VerticalDivider));
    lineRect = tester.getRect(find.byType(DecoratedBox));
    expect(lineRect.top, dividerRect.top);
    expect(lineRect.bottom, dividerRect.bottom - customIndent);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: VerticalDivider(indent: customIndent, endIndent: customIndent)),
      ),
    );
    dividerRect = tester.getRect(find.byType(VerticalDivider));
    lineRect = tester.getRect(find.byType(DecoratedBox));
    expect(lineRect.top, dividerRect.top + customIndent);
    expect(lineRect.bottom, dividerRect.bottom - customIndent);
  });

  // Regression test for https://github.com/flutter/flutter/issues/39533
  testWidgets('createBorderSide does not throw exception with null context', (
    WidgetTester tester,
  ) async {
    // Passing a null context used to throw an exception but no longer does.
    expect(() => Divider.createBorderSide(null), isNot(throwsAssertionError));
    expect(() => Divider.createBorderSide(null), isNot(throwsNoSuchMethodError));
  });
}
