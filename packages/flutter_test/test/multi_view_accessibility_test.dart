// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'multi_view_testing.dart';

void main() {
  testWidgets('Detects tap targets in all views', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await pumpViews(
      tester: tester,
      viewContents: <Widget>[
        SizedBox(width: 47.0, height: 47.0, child: GestureDetector(onTap: () {})),
        SizedBox(width: 46.0, height: 46.0, child: GestureDetector(onTap: () {})),
      ],
    );
    final Evaluation result = await androidTapTargetGuideline.evaluate(tester);
    expect(result.passed, false);
    expect(
      result.reason,
      contains('expected tap target size of at least Size(48.0, 48.0), but found Size(47.0, 47.0)'),
    );
    expect(
      result.reason,
      contains('expected tap target size of at least Size(48.0, 48.0), but found Size(46.0, 46.0)'),
    );
    handle.dispose();
  });

  testWidgets('Detects labeled tap targets in all views', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await pumpViews(
      tester: tester,
      viewContents: <Widget>[
        SizedBox(width: 47.0, height: 47.0, child: GestureDetector(onTap: () {})),
        SizedBox(width: 46.0, height: 46.0, child: GestureDetector(onTap: () {})),
      ],
    );
    final Evaluation result = await labeledTapTargetGuideline.evaluate(tester);
    expect(result.passed, false);
    final List<String> lines = const LineSplitter().convert(result.reason!);
    expect(lines, hasLength(2));
    expect(lines.first, startsWith('SemanticsNode#1(Rect.fromLTRB(0.0, 0.0, 47.0, 47.0)'));
    expect(
      lines.first,
      endsWith('expected tappable node to have semantic label, but none was found.'),
    );
    expect(lines.last, startsWith('SemanticsNode#2(Rect.fromLTRB(0.0, 0.0, 46.0, 46.0)'));
    expect(
      lines.last,
      endsWith('expected tappable node to have semantic label, but none was found.'),
    );
    handle.dispose();
  });

  testWidgets('Detects contrast problems in all views', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await pumpViews(
      tester: tester,
      viewContents: <Widget>[
        Container(
          width: 200.0,
          height: 200.0,
          color: Colors.yellow,
          child: const Text(
            'this is a test',
            style: TextStyle(fontSize: 14.0, color: Colors.yellowAccent),
          ),
        ),
        Container(
          width: 200.0,
          height: 200.0,
          color: Colors.yellow,
          child: const Text(
            'this is a test',
            style: TextStyle(fontSize: 25.0, color: Colors.yellowAccent),
          ),
        ),
      ],
    );
    final Evaluation result = await textContrastGuideline.evaluate(tester);
    expect(result.passed, false);
    expect(
      result.reason,
      contains('Expected contrast ratio of at least 4.5 but found 0.88 for a font size of 14.0.'),
    );
    expect(
      result.reason,
      contains('Expected contrast ratio of at least 3.0 but found 0.88 for a font size of 25.0.'),
    );
    handle.dispose();
  });
}

Future<void> pumpViews({required WidgetTester tester, required List<Widget> viewContents}) {
  final List<Widget> views = <Widget>[
    for (int i = 0; i < viewContents.length; i++)
      View(view: FakeView(tester.view, viewId: i + 100), child: Center(child: viewContents[i])),
  ];

  return tester.pumpWidget(
    wrapWithView: false,
    Directionality(textDirection: TextDirection.ltr, child: ViewCollection(views: views)),
  );
}
