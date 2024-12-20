// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/src/foundation/assertions.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

const List<Widget> children = <Widget>[
  SizedBox(width: 200.0, height: 150.0),
  SizedBox(width: 200.0, height: 150.0),
  SizedBox(width: 200.0, height: 150.0),
  SizedBox(width: 200.0, height: 150.0),
];

void expectRects(WidgetTester tester, List<Rect> expected) {
  final Finder finder = find.byType(SizedBox);
  final List<Rect> actual = <Rect>[];
  finder.runCached(() {
    for (int i = 0; i < expected.length; ++i) {
      final Finder current = finder.at(i);
      expect(current, findsOneWidget);
      actual.add(tester.getRect(finder.at(i)));
    }
  });
  expect(() => finder.at(expected.length), throwsRangeError);
  expect(actual, equals(expected));
}

void main() {
  testWidgets('ListBody down', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Flex(direction: Axis.vertical, children: <Widget>[ListBody(children: children)]),
    );

    expectRects(tester, <Rect>[
      const Rect.fromLTWH(0.0, 0.0, 800.0, 150.0),
      const Rect.fromLTWH(0.0, 150.0, 800.0, 150.0),
      const Rect.fromLTWH(0.0, 300.0, 800.0, 150.0),
      const Rect.fromLTWH(0.0, 450.0, 800.0, 150.0),
    ]);
  });

  testWidgets('ListBody up', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Flex(
        direction: Axis.vertical,
        children: <Widget>[ListBody(reverse: true, children: children)],
      ),
    );

    expectRects(tester, <Rect>[
      const Rect.fromLTWH(0.0, 450.0, 800.0, 150.0),
      const Rect.fromLTWH(0.0, 300.0, 800.0, 150.0),
      const Rect.fromLTWH(0.0, 150.0, 800.0, 150.0),
      const Rect.fromLTWH(0.0, 0.0, 800.0, 150.0),
    ]);
  });

  testWidgets('ListBody right', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Flex(
        textDirection: TextDirection.ltr,
        direction: Axis.horizontal,
        children: <Widget>[
          Directionality(
            textDirection: TextDirection.ltr,
            child: ListBody(mainAxis: Axis.horizontal, children: children),
          ),
        ],
      ),
    );

    expectRects(tester, <Rect>[
      const Rect.fromLTWH(0.0, 0.0, 200.0, 600.0),
      const Rect.fromLTWH(200.0, 0.0, 200.0, 600.0),
      const Rect.fromLTWH(400.0, 0.0, 200.0, 600.0),
      const Rect.fromLTWH(600.0, 0.0, 200.0, 600.0),
    ]);
  });

  testWidgets('ListBody left', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Flex(
        textDirection: TextDirection.ltr,
        direction: Axis.horizontal,
        children: <Widget>[
          Directionality(
            textDirection: TextDirection.rtl,
            child: ListBody(mainAxis: Axis.horizontal, children: children),
          ),
        ],
      ),
    );

    expectRects(tester, <Rect>[
      const Rect.fromLTWH(600.0, 0.0, 200.0, 600.0),
      const Rect.fromLTWH(400.0, 0.0, 200.0, 600.0),
      const Rect.fromLTWH(200.0, 0.0, 200.0, 600.0),
      const Rect.fromLTWH(0.0, 0.0, 200.0, 600.0),
    ]);
  });

  testWidgets('Limited space along main axis error', (WidgetTester tester) async {
    final FlutterExceptionHandler oldHandler = FlutterError.onError!;
    final List<FlutterErrorDetails> errors = <FlutterErrorDetails>[];
    FlutterError.onError = (FlutterErrorDetails error) => errors.add(error);
    try {
      await tester.pumpWidget(
        const SizedBox(
          width: 100,
          height: 100,
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: ListBody(mainAxis: Axis.horizontal, children: children),
          ),
        ),
      );
    } finally {
      FlutterError.onError = oldHandler;
    }
    expect(errors, isNotEmpty);
    expect(errors.first.exception, isFlutterError);
    expect(
      (errors.first.exception as FlutterError).toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   RenderListBody must have unlimited space along its main axis.\n'
        '   RenderListBody does not clip or resize its children, so it must\n'
        '   be placed in a parent that does not constrain the main axis.\n'
        '   You probably want to put the RenderListBody inside a\n'
        '   RenderViewport with a matching main axis.\n',
      ),
    );
  });

  testWidgets('Nested ListBody unbounded cross axis error', (WidgetTester tester) async {
    final FlutterExceptionHandler oldHandler = FlutterError.onError!;
    final List<FlutterErrorDetails> errors = <FlutterErrorDetails>[];
    FlutterError.onError = (FlutterErrorDetails error) => errors.add(error);
    try {
      await tester.pumpWidget(
        const Flex(
          textDirection: TextDirection.ltr,
          direction: Axis.horizontal,
          children: <Widget>[
            Directionality(
              textDirection: TextDirection.ltr,
              child: ListBody(
                mainAxis: Axis.horizontal,
                children: <Widget>[
                  Flex(
                    textDirection: TextDirection.ltr,
                    direction: Axis.vertical,
                    children: <Widget>[
                      Directionality(
                        textDirection: TextDirection.ltr,
                        child: ListBody(children: children),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ],
        ),
      );
    } finally {
      FlutterError.onError = oldHandler;
    }
    expect(errors, isNotEmpty);
    expect(errors.first.exception, isFlutterError);
    expect(
      (errors.first.exception as FlutterError).toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   RenderListBody must have a bounded constraint for its cross axis.\n'
        '   RenderListBody forces its children to expand to fit the\n'
        "   RenderListBody's container, so it must be placed in a parent that\n"
        '   constrains the cross axis to a finite dimension.\n'
        '   If you are attempting to nest a RenderListBody with one direction\n'
        '   inside one of another direction, you will want to wrap the inner\n'
        '   one inside a box that fixes the dimension in that direction, for\n'
        '   example, a RenderIntrinsicWidth or RenderIntrinsicHeight object.\n'
        '   This is relatively expensive, however.\n',
      ),
    );
  });
}
