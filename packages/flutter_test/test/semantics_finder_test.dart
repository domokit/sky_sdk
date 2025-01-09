// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'multi_view_testing.dart';

void main() {
  testWidgets('can find nodes in an view when no view is specified', (WidgetTester tester) async {
    final List<FlutterView> views = <FlutterView>[
      for (int i = 0; i < 3; i++) FakeView(tester.view, viewId: i + 100),
    ];
    await pumpViews(tester: tester, views: views);

    expect(find.semantics.byLabel('View0Child0'), findsOne);
    expect(find.semantics.byLabel('View1Child1'), findsOne);
    expect(find.semantics.byLabel('View2Child2'), findsOne);
  });

  testWidgets('can find nodes only in specified view', (WidgetTester tester) async {
    final List<FlutterView> views = <FlutterView>[
      for (int i = 0; i < 3; i++) FakeView(tester.view, viewId: i + 100),
    ];
    await pumpViews(tester: tester, views: views);

    expect(find.semantics.byLabel('View0Child0', view: views[0]), findsOne);
    expect(find.semantics.byLabel('View0Child0', view: views[1]), findsNothing);
    expect(find.semantics.byLabel('View0Child0', view: views[2]), findsNothing);

    expect(find.semantics.byLabel('View1Child1', view: views[0]), findsNothing);
    expect(find.semantics.byLabel('View1Child1', view: views[1]), findsOne);
    expect(find.semantics.byLabel('View1Child1', view: views[2]), findsNothing);

    expect(find.semantics.byLabel('View2Child2', view: views[0]), findsNothing);
    expect(find.semantics.byLabel('View2Child2', view: views[1]), findsNothing);
    expect(find.semantics.byLabel('View2Child2', view: views[2]), findsOne);
  });
}

Future<void> pumpViews({required WidgetTester tester, required List<FlutterView> views}) {
  final List<Widget> viewWidgets = <Widget>[
    for (int i = 0; i < 3; i++)
      View(
        view: views[i],
        child: Center(
          child: Column(
            children: <Widget>[
              for (int c = 0; c < 5; c++)
                Semantics(container: true, child: Text('View${i}Child$c')),
            ],
          ),
        ),
      ),
  ];

  return tester.pumpWidget(
    wrapWithView: false,
    Directionality(textDirection: TextDirection.ltr, child: ViewCollection(views: viewWidgets)),
  );
}
