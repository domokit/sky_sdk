// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'multi_view_testing.dart';

void main() {
  testWidgets('Widgets in view update as expected', (WidgetTester tester) async {
    final Widget widget = View(view: tester.view, child: const TestWidget());

    await tester.pumpWidget(wrapWithView: false, widget);

    expect(find.text('Hello'), findsOneWidget);
    expect(tester.renderObject<RenderParagraph>(find.byType(Text)).text.toPlainText(), 'Hello');

    tester.state<TestWidgetState>(find.byType(TestWidget)).text = 'World';
    await tester.pump();
    expect(find.text('Hello'), findsNothing);
    expect(find.text('World'), findsOneWidget);
    expect(tester.renderObject<RenderParagraph>(find.byType(Text)).text.toPlainText(), 'World');

    await tester.pumpWidget(wrapWithView: false, ViewCollection(views: <Widget>[widget]));
    expect(find.text('Hello'), findsNothing);
    expect(find.text('World'), findsOneWidget);
    expect(tester.renderObject<RenderParagraph>(find.byType(Text)).text.toPlainText(), 'World');

    tester.state<TestWidgetState>(find.byType(TestWidget)).text = 'FooBar';
    await tester.pumpWidget(wrapWithView: false, widget);
    expect(find.text('World'), findsNothing);
    expect(find.text('FooBar'), findsOneWidget);
    expect(tester.renderObject<RenderParagraph>(find.byType(Text)).text.toPlainText(), 'FooBar');
  });

  testWidgets('Views in ViewCollection update as expected', (WidgetTester tester) async {
    Iterable<String> renderParagraphTexts() {
      return tester
          .renderObjectList<RenderParagraph>(find.byType(Text))
          .map((RenderParagraph r) => r.text.toPlainText());
    }

    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();
    final Widget view1 = View(view: tester.view, child: TestWidget(key: key1));
    final Widget view2 = View(view: FakeView(tester.view), child: TestWidget(key: key2));

    await tester.pumpWidget(wrapWithView: false, ViewCollection(views: <Widget>[view1, view2]));

    expect(find.text('Hello'), findsNWidgets(2));
    expect(renderParagraphTexts(), <String>['Hello', 'Hello']);

    tester.state<TestWidgetState>(find.byKey(key1)).text = 'Guten';
    tester.state<TestWidgetState>(find.byKey(key2)).text = 'Tag';
    await tester.pump();
    expect(find.text('Hello'), findsNothing);
    expect(find.text('Guten'), findsOneWidget);
    expect(find.text('Tag'), findsOneWidget);
    expect(renderParagraphTexts(), <String>['Guten', 'Tag']);

    tester.state<TestWidgetState>(find.byKey(key2)).text = 'Abend';
    await tester.pump();
    expect(find.text('Tag'), findsNothing);
    expect(find.text('Guten'), findsOneWidget);
    expect(find.text('Abend'), findsOneWidget);
    expect(renderParagraphTexts(), <String>['Guten', 'Abend']);

    tester.state<TestWidgetState>(find.byKey(key2)).text = 'Morgen';
    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          view1,
          ViewCollection(views: <Widget>[view2]),
        ],
      ),
    );
    expect(find.text('Abend'), findsNothing);
    expect(find.text('Guten'), findsOneWidget);
    expect(find.text('Morgen'), findsOneWidget);
    expect(renderParagraphTexts(), <String>['Guten', 'Morgen']);
  });

  testWidgets('Views in ViewAnchor update as expected', (WidgetTester tester) async {
    Iterable<String> renderParagraphTexts() {
      return tester
          .renderObjectList<RenderParagraph>(find.byType(Text))
          .map((RenderParagraph r) => r.text.toPlainText());
    }

    final Key insideAnchoredViewKey = UniqueKey();
    final Key outsideAnchoredViewKey = UniqueKey();
    final Widget view = View(
      view: FakeView(tester.view),
      child: TestWidget(key: insideAnchoredViewKey),
    );

    await tester.pumpWidget(ViewAnchor(view: view, child: TestWidget(key: outsideAnchoredViewKey)));

    expect(find.text('Hello'), findsNWidgets(2));
    expect(renderParagraphTexts(), <String>['Hello', 'Hello']);

    tester.state<TestWidgetState>(find.byKey(outsideAnchoredViewKey)).text = 'Guten';
    tester.state<TestWidgetState>(find.byKey(insideAnchoredViewKey)).text = 'Tag';
    await tester.pump();
    expect(find.text('Hello'), findsNothing);
    expect(find.text('Guten'), findsOneWidget);
    expect(find.text('Tag'), findsOneWidget);
    expect(renderParagraphTexts(), <String>['Guten', 'Tag']);

    tester.state<TestWidgetState>(find.byKey(insideAnchoredViewKey)).text = 'Abend';
    await tester.pump();
    expect(find.text('Tag'), findsNothing);
    expect(find.text('Guten'), findsOneWidget);
    expect(find.text('Abend'), findsOneWidget);
    expect(renderParagraphTexts(), <String>['Guten', 'Abend']);

    tester.state<TestWidgetState>(find.byKey(outsideAnchoredViewKey)).text = 'Schönen';
    await tester.pump();
    expect(find.text('Guten'), findsNothing);
    expect(find.text('Schönen'), findsOneWidget);
    expect(find.text('Abend'), findsOneWidget);
    expect(renderParagraphTexts(), <String>['Schönen', 'Abend']);

    tester.state<TestWidgetState>(find.byKey(insideAnchoredViewKey)).text = 'Tag';
    await tester.pumpWidget(
      ViewAnchor(
        view: ViewCollection(views: <Widget>[view]),
        child: TestWidget(key: outsideAnchoredViewKey),
      ),
    );
    await tester.pump();
    expect(find.text('Abend'), findsNothing);
    expect(find.text('Schönen'), findsOneWidget);
    expect(find.text('Tag'), findsOneWidget);
    expect(renderParagraphTexts(), <String>['Schönen', 'Tag']);

    tester.state<TestWidgetState>(find.byKey(insideAnchoredViewKey)).text = 'Morgen';
    await tester.pumpWidget(
      SizedBox(
        child: ViewAnchor(
          view: ViewCollection(views: <Widget>[view]),
          child: TestWidget(key: outsideAnchoredViewKey),
        ),
      ),
    );
    await tester.pump();
    expect(
      find.text('Schönen'),
      findsNothing,
    ); // The `outsideAnchoredViewKey` is not a global key, its state is lost in the move above.
    expect(find.text('Tag'), findsNothing);
    expect(find.text('Hello'), findsOneWidget);
    expect(find.text('Morgen'), findsOneWidget);
    expect(renderParagraphTexts(), <String>['Hello', 'Morgen']);
  });
}

class TestWidget extends StatefulWidget {
  const TestWidget({super.key});

  @override
  State<TestWidget> createState() => TestWidgetState();
}

class TestWidgetState extends State<TestWidget> {
  String get text => _text;
  String _text = 'Hello';
  set text(String value) {
    if (_text == value) {
      return;
    }
    setState(() {
      _text = value;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Text(text, textDirection: TextDirection.ltr);
  }
}
