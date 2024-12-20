// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Semantics 4', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    //    O
    //   / \       O=root
    //  L   L      L=node with label
    //     / \     C=node with checked
    //    C   C*   *=node removed next pass
    //
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          fit: StackFit.expand,
          children: <Widget>[
            Semantics(container: true, label: 'L1'),
            Semantics(
              label: 'L2',
              container: true,
              child: Stack(
                fit: StackFit.expand,
                children: <Widget>[Semantics(checked: true), Semantics(checked: false)],
              ),
            ),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(id: 1, label: 'L1', rect: TestSemantics.fullScreen),
            TestSemantics.rootChild(
              id: 2,
              label: 'L2',
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  id: 3,
                  flags: SemanticsFlag.hasCheckedState.index | SemanticsFlag.isChecked.index,
                  rect: TestSemantics.fullScreen,
                ),
                TestSemantics(
                  id: 4,
                  flags: SemanticsFlag.hasCheckedState.index,
                  rect: TestSemantics.fullScreen,
                ),
              ],
            ),
          ],
        ),
      ),
    );

    //    O        O=root
    //   / \       L=node with label
    //  L* LC      C=node with checked
    //             *=node removed next pass
    //
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          fit: StackFit.expand,
          children: <Widget>[
            Semantics(label: 'L1', container: true),
            Semantics(
              label: 'L2',
              container: true,
              child: Stack(
                fit: StackFit.expand,
                children: <Widget>[Semantics(checked: true), Semantics()],
              ),
            ),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(id: 1, label: 'L1', rect: TestSemantics.fullScreen),
            TestSemantics.rootChild(
              id: 2,
              label: 'L2',
              flags: SemanticsFlag.hasCheckedState.index | SemanticsFlag.isChecked.index,
              rect: TestSemantics.fullScreen,
            ),
          ],
        ),
      ),
    );

    //             O=root
    //    OLC      L=node with label
    //             C=node with checked
    //
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          fit: StackFit.expand,
          children: <Widget>[
            Semantics(),
            Semantics(
              label: 'L2',
              container: true,
              child: Stack(
                fit: StackFit.expand,
                children: <Widget>[Semantics(checked: true), Semantics()],
              ),
            ),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 2,
              label: 'L2',
              flags: SemanticsFlag.hasCheckedState.index | SemanticsFlag.isChecked.index,
              rect: TestSemantics.fullScreen,
            ),
          ],
        ),
      ),
    );

    semantics.dispose();
  });
}
