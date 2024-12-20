// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('SemanticsNode ids are stable', (WidgetTester tester) async {
    // Regression test for b/151732341.
    final SemanticsTester semantics = SemanticsTester(tester);
    final TapGestureRecognizer recognizer1 = TapGestureRecognizer();
    addTearDown(recognizer1.dispose);
    final TapGestureRecognizer recognizer2 = TapGestureRecognizer();
    addTearDown(recognizer2.dispose);
    final TapGestureRecognizer recognizer3 = TapGestureRecognizer();
    addTearDown(recognizer3.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Text.rich(
          TextSpan(
            text: 'Hallo ',
            recognizer: recognizer1..onTap = () {},
            children: <TextSpan>[
              TextSpan(text: 'Welt ', recognizer: recognizer2..onTap = () {}),
              TextSpan(text: '!!!', recognizer: recognizer3..onTap = () {}),
            ],
          ),
        ),
      ),
    );
    expect(find.text('Hallo Welt !!!'), findsOneWidget);
    final SemanticsNode node = tester.getSemantics(find.text('Hallo Welt !!!'));
    final Map<String, int> labelToNodeId = <String, int>{};
    node.visitChildren((SemanticsNode node) {
      labelToNodeId[node.label] = node.id;
      return true;
    });
    expect(node.id, 1);
    expect(labelToNodeId['Hallo '], 2);
    expect(labelToNodeId['Welt '], 3);
    expect(labelToNodeId['!!!'], 4);
    expect(labelToNodeId.length, 3);

    // Rebuild semantics.
    tester.renderObject(find.text('Hallo Welt !!!')).markNeedsSemanticsUpdate();
    await tester.pump();

    final SemanticsNode nodeAfterRebuild = tester.getSemantics(find.text('Hallo Welt !!!'));
    final Map<String, int> labelToNodeIdAfterRebuild = <String, int>{};
    nodeAfterRebuild.visitChildren((SemanticsNode node) {
      labelToNodeIdAfterRebuild[node.label] = node.id;
      return true;
    });

    // Node IDs are stable.
    expect(nodeAfterRebuild.id, node.id);
    expect(labelToNodeIdAfterRebuild['Hallo '], labelToNodeId['Hallo ']);
    expect(labelToNodeIdAfterRebuild['Welt '], labelToNodeId['Welt ']);
    expect(labelToNodeIdAfterRebuild['!!!'], labelToNodeId['!!!']);
    expect(labelToNodeIdAfterRebuild.length, 3);

    final TapGestureRecognizer recognizer4 = TapGestureRecognizer();
    addTearDown(recognizer4.dispose);
    final TapGestureRecognizer recognizer5 = TapGestureRecognizer();
    addTearDown(recognizer5.dispose);

    // Remove one node.
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Text.rich(
          TextSpan(
            text: 'Hallo ',
            recognizer: recognizer4..onTap = () {},
            children: <TextSpan>[TextSpan(text: 'Welt ', recognizer: recognizer5..onTap = () {})],
          ),
        ),
      ),
    );

    final SemanticsNode nodeAfterRemoval = tester.getSemantics(find.text('Hallo Welt '));
    final Map<String, int> labelToNodeIdAfterRemoval = <String, int>{};
    nodeAfterRemoval.visitChildren((SemanticsNode node) {
      labelToNodeIdAfterRemoval[node.label] = node.id;
      return true;
    });

    // Node IDs are stable.
    expect(nodeAfterRemoval.id, node.id);
    expect(labelToNodeIdAfterRemoval['Hallo '], labelToNodeId['Hallo ']);
    expect(labelToNodeIdAfterRemoval['Welt '], labelToNodeId['Welt ']);
    expect(labelToNodeIdAfterRemoval.length, 2);

    final TapGestureRecognizer recognizer6 = TapGestureRecognizer();
    addTearDown(recognizer6.dispose);
    final TapGestureRecognizer recognizer7 = TapGestureRecognizer();
    addTearDown(recognizer7.dispose);
    final TapGestureRecognizer recognizer8 = TapGestureRecognizer();
    addTearDown(recognizer8.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Text.rich(
          TextSpan(
            text: 'Hallo ',
            recognizer: recognizer6..onTap = () {},
            children: <TextSpan>[
              TextSpan(text: 'Welt ', recognizer: recognizer7..onTap = () {}),
              TextSpan(text: '!!!', recognizer: recognizer8..onTap = () {}),
            ],
          ),
        ),
      ),
    );
    expect(find.text('Hallo Welt !!!'), findsOneWidget);
    final SemanticsNode nodeAfterAddition = tester.getSemantics(find.text('Hallo Welt !!!'));
    final Map<String, int> labelToNodeIdAfterAddition = <String, int>{};
    nodeAfterAddition.visitChildren((SemanticsNode node) {
      labelToNodeIdAfterAddition[node.label] = node.id;
      return true;
    });

    // New node gets a new ID.
    expect(nodeAfterAddition.id, node.id);
    expect(labelToNodeIdAfterAddition['Hallo '], labelToNodeId['Hallo ']);
    expect(labelToNodeIdAfterAddition['Welt '], labelToNodeId['Welt ']);
    expect(labelToNodeIdAfterAddition['!!!'], isNot(labelToNodeId['!!!']));
    expect(labelToNodeIdAfterAddition['!!!'], isNotNull);
    expect(labelToNodeIdAfterAddition.length, 3);

    semantics.dispose();
  });
}
