// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('Performing SemanticsAction.showOnScreen does not crash if node no longer exist', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/100358.

    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Semantics(explicitChildNodes: true, child: const Text('Hello World')),
      ),
    );

    final int nodeId = tester.semantics.find(find.bySemanticsLabel('Hello World')).id;

    await tester.pumpWidget(Directionality(textDirection: TextDirection.ltr, child: Container()));

    // Node with $nodeId is no longer in the tree.
    expect(semantics, isNot(hasSemantics(TestSemantics(id: nodeId))));

    // Executing SemanticsAction.showOnScreen on that node does not crash.
    // (A platform may not have processed the semantics update yet and send
    // actions for no longer existing nodes.)
    tester.binding.performSemanticsAction(
      SemanticsActionEvent(
        type: SemanticsAction.showOnScreen,
        nodeId: nodeId,
        viewId: tester.view.viewId,
      ),
    );
    semantics.dispose();
  });
}
