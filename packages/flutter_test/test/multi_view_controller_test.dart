// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'multi_view_testing.dart';

void main() {
  testWidgets('simulatedAccessibilityTraversal - start and end in same view', (
    WidgetTester tester,
  ) async {
    await pumpViews(tester: tester);
    expect(
      tester.semantics
          .simulatedAccessibilityTraversal(
            start: find.text('View2Child1'),
            end: find.text('View2Child3'),
          )
          .map((SemanticsNode node) => node.label),
      <String>['View2Child1', 'View2Child2', 'View2Child3'],
    );
  });

  testWidgets('simulatedAccessibilityTraversal - start not specified', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    expect(
      tester.semantics
          .simulatedAccessibilityTraversal(end: find.text('View2Child3'))
          .map((SemanticsNode node) => node.label),
      <String>['View2Child0', 'View2Child1', 'View2Child2', 'View2Child3'],
    );
  });

  testWidgets('simulatedAccessibilityTraversal - end not specified', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    expect(
      tester.semantics
          .simulatedAccessibilityTraversal(start: find.text('View2Child1'))
          .map((SemanticsNode node) => node.label),
      <String>['View2Child1', 'View2Child2', 'View2Child3', 'View2Child4'],
    );
  });

  testWidgets('simulatedAccessibilityTraversal - nothing specified', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    expect(
      tester.semantics.simulatedAccessibilityTraversal().map((SemanticsNode node) => node.label),
      <String>['View1Child0', 'View1Child1', 'View1Child2', 'View1Child3', 'View1Child4'],
    );
    // Should be traversing over tester.view.
    expect(tester.viewOf(find.text('View1Child0')), tester.view);
  });

  testWidgets('simulatedAccessibilityTraversal - only view specified', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    expect(
      tester.semantics
          .simulatedAccessibilityTraversal(view: tester.viewOf(find.text('View2Child1')))
          .map((SemanticsNode node) => node.label),
      <String>['View2Child0', 'View2Child1', 'View2Child2', 'View2Child3', 'View2Child4'],
    );
  });

  testWidgets('simulatedAccessibilityTraversal - everything specified', (
    WidgetTester tester,
  ) async {
    await pumpViews(tester: tester);
    expect(
      tester.semantics
          .simulatedAccessibilityTraversal(
            start: find.text('View2Child1'),
            end: find.text('View2Child3'),
            view: tester.viewOf(find.text('View2Child1')),
          )
          .map((SemanticsNode node) => node.label),
      <String>['View2Child1', 'View2Child2', 'View2Child3'],
    );
  });

  testWidgets('simulatedAccessibilityTraversal - start and end not in same view', (
    WidgetTester tester,
  ) async {
    await pumpViews(tester: tester);
    expect(
      () => tester.semantics.simulatedAccessibilityTraversal(
        start: find.text('View2Child1'),
        end: find.text('View1Child3'),
      ),
      throwsA(
        isStateError.having(
          (StateError e) => e.message,
          'message',
          contains('The start and end node are in different views.'),
        ),
      ),
    );
  });

  testWidgets('simulatedAccessibilityTraversal - start is not in view', (
    WidgetTester tester,
  ) async {
    await pumpViews(tester: tester);
    expect(
      () => tester.semantics.simulatedAccessibilityTraversal(
        start: find.text('View2Child1'),
        end: find.text('View1Child3'),
        view: tester.viewOf(find.text('View1Child3')),
      ),
      throwsA(
        isStateError.having(
          (StateError e) => e.message,
          'message',
          contains('The start node is not part of the provided view.'),
        ),
      ),
    );
  });

  testWidgets('simulatedAccessibilityTraversal - end is not in view', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    expect(
      () => tester.semantics.simulatedAccessibilityTraversal(
        start: find.text('View2Child1'),
        end: find.text('View1Child3'),
        view: tester.viewOf(find.text('View2Child1')),
      ),
      throwsA(
        isStateError.having(
          (StateError e) => e.message,
          'message',
          contains('The end node is not part of the provided view.'),
        ),
      ),
    );
  });

  testWidgets('viewOf', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    expect(tester.viewOf(find.text('View0Child0')).viewId, 100);
    expect(tester.viewOf(find.text('View1Child1')).viewId, tester.view.viewId);
    expect(tester.viewOf(find.text('View2Child2')).viewId, 102);
  });

  testWidgets('layers includes layers from all views', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    const int numberOfViews = 3;
    expect(
      tester.binding.renderViews.length,
      numberOfViews,
    ); // One RenderView for each FlutterView.

    final List<Layer> layers = tester.layers;
    // Each RenderView contributes a TransformLayer and a PictureLayer.
    expect(layers, hasLength(numberOfViews * 2));
    expect(layers.whereType<TransformLayer>(), hasLength(numberOfViews));
    expect(layers.whereType<PictureLayer>(), hasLength(numberOfViews));
    expect(
      layers.whereType<TransformLayer>().map((TransformLayer l) => l.owner),
      containsAll(tester.binding.renderViews),
    );
  });

  testWidgets('hitTestOnBinding', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    // Not specifying a viewId hit tests on tester.view:
    HitTestResult result = tester.hitTestOnBinding(Offset.zero);
    expect(
      result.path.map((HitTestEntry h) => h.target).whereType<RenderView>().single.flutterView,
      tester.view,
    );
    // Specifying a viewId is respected:
    result = tester.hitTestOnBinding(Offset.zero, viewId: 100);
    expect(
      result.path
          .map((HitTestEntry h) => h.target)
          .whereType<RenderView>()
          .single
          .flutterView
          .viewId,
      100,
    );
    result = tester.hitTestOnBinding(Offset.zero, viewId: 102);
    expect(
      result.path
          .map((HitTestEntry h) => h.target)
          .whereType<RenderView>()
          .single
          .flutterView
          .viewId,
      102,
    );
  });

  testWidgets('hitTestable works in different Views', (WidgetTester tester) async {
    await pumpViews(tester: tester);
    expect(
      (find.text('View0Child0').hitTestable().evaluate().single.widget as Text).data,
      'View0Child0',
    );
    expect(
      (find.text('View1Child1').hitTestable().evaluate().single.widget as Text).data,
      'View1Child1',
    );
    expect(
      (find.text('View2Child2').hitTestable().evaluate().single.widget as Text).data,
      'View2Child2',
    );
  });

  testWidgets('simulatedAccessibilityTraversal - startNode and endNode in same view', (
    WidgetTester tester,
  ) async {
    await pumpViews(tester: tester);
    expect(
      tester.semantics
          .simulatedAccessibilityTraversal(
            startNode: find.semantics.byLabel('View2Child1'),
            endNode: find.semantics.byLabel('View2Child3'),
          )
          .map((SemanticsNode node) => node.label),
      <String>['View2Child1', 'View2Child2', 'View2Child3'],
    );
  });
}

Future<void> pumpViews({required WidgetTester tester}) {
  final List<Widget> views = <Widget>[
    for (int i = 0; i < 3; i++)
      View(
        view: i == 1 ? tester.view : FakeView(tester.view, viewId: i + 100),
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
    Directionality(textDirection: TextDirection.ltr, child: ViewCollection(views: views)),
  );
}
