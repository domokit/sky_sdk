// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows header', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoFormSection(
            header: const Text('Header'),
            children: <Widget>[CupertinoTextFormFieldRow()],
          ),
        ),
      ),
    );

    expect(find.text('Header'), findsOneWidget);
  });

  testWidgets('Shows footer', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoFormSection(
            footer: const Text('Footer'),
            children: <Widget>[CupertinoTextFormFieldRow()],
          ),
        ),
      ),
    );

    expect(find.text('Footer'), findsOneWidget);
  });

  testWidgets('Shows long dividers in edge-to-edge section part 1', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoFormSection(children: <Widget>[CupertinoTextFormFieldRow()])),
      ),
    );

    // Since the children list is reconstructed with dividers in it, the column
    // retrieved should have 3 items for an input [children] param with 1 child.
    final Column childrenColumn = tester.widget(find.byType(Column).at(1));
    expect(childrenColumn.children.length, 3);
  });

  testWidgets('Shows long dividers in edge-to-edge section part 2', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoFormSection(
            children: <Widget>[CupertinoTextFormFieldRow(), CupertinoTextFormFieldRow()],
          ),
        ),
      ),
    );

    // Since the children list is reconstructed with dividers in it, the column
    // retrieved should have 5 items for an input [children] param with 2
    // children. Two long dividers, two rows, and one short divider.
    final Column childrenColumn = tester.widget(find.byType(Column).at(1));
    expect(childrenColumn.children.length, 5);
  });

  testWidgets('Does not show long dividers in insetGrouped section part 1', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoFormSection.insetGrouped(children: <Widget>[CupertinoTextFormFieldRow()]),
        ),
      ),
    );

    // Since the children list is reconstructed without long dividers in it, the
    // column retrieved should have 1 item for an input [children] param with 1
    // child.
    final Column childrenColumn = tester.widget(find.byType(Column).at(1));
    expect(childrenColumn.children.length, 1);
  });

  testWidgets('Does not show long dividers in insetGrouped section part 2', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        restorationScopeId: 'App',
        home: Center(
          child: CupertinoFormSection.insetGrouped(
            children: <Widget>[CupertinoTextFormFieldRow(), CupertinoTextFormFieldRow()],
          ),
        ),
      ),
    );

    // Since the children list is reconstructed with short dividers in it, the
    // column retrieved should have 3 items for an input [children] param with 2
    // children. Two long dividers, two rows, and one short divider.
    final Column childrenColumn = tester.widget(find.byType(Column).at(1));
    expect(childrenColumn.children.length, 3);
  });

  testWidgets('Sets background color for section', (WidgetTester tester) async {
    const Color backgroundColor = CupertinoColors.systemBlue;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(),
          child: CupertinoFormSection(
            backgroundColor: backgroundColor,
            children: <Widget>[CupertinoTextFormFieldRow()],
          ),
        ),
      ),
    );

    final DecoratedBox decoratedBox = tester.widget(find.byType(DecoratedBox).first);
    final BoxDecoration boxDecoration = decoratedBox.decoration as BoxDecoration;
    expect(boxDecoration.color, backgroundColor);
  });

  testWidgets('Setting clipBehavior clips children section', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoFormSection(
            clipBehavior: Clip.antiAlias,
            children: <Widget>[CupertinoTextFormFieldRow()],
          ),
        ),
      ),
    );

    expect(find.byType(ClipRRect), findsOneWidget);
  });

  testWidgets('Not setting clipBehavior does not produce a RenderClipRRect object', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoFormSection(children: <Widget>[CupertinoTextFormFieldRow()])),
      ),
    );

    final Iterable<RenderClipRRect> renderClips =
        tester.allRenderObjects.whereType<RenderClipRRect>();
    expect(renderClips, isEmpty);
  });

  testWidgets('Does not double up padding on header', (WidgetTester tester) async {
    const Widget header = Text('Header');

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoFormSection(
            header: header,
            children: <Widget>[CupertinoTextFormFieldRow()],
          ),
        ),
      ),
    );

    expect(tester.getTopLeft(find.byWidget(header)), const Offset(20, 22));
  });

  testWidgets('Does not double up padding on footer', (WidgetTester tester) async {
    const Widget footer = Text('Footer');

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoFormSection(
            footer: footer,
            children: <Widget>[CupertinoTextFormFieldRow()],
          ),
        ),
      ),
    );

    expect(
      tester.getTopLeft(find.byWidget(footer)),
      offsetMoreOrLessEquals(const Offset(20, 65), epsilon: 1),
    );
  });

  testWidgets('Sets custom margin', (WidgetTester tester) async {
    final Widget child = CupertinoTextFormFieldRow();

    const double margin = 35;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoFormSection(
            margin: const EdgeInsets.all(margin),
            children: <Widget>[child],
          ),
        ),
      ),
    );

    expect(
      tester.getTopLeft(find.byWidget(child)),
      offsetMoreOrLessEquals(const Offset(margin, 22 + margin), epsilon: 1),
    );
  });
}
