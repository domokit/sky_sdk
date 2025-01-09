// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Semantics 8 - Merging with reset', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MergeSemantics(
        child: Semantics(
          container: true,
          child: Semantics(
            container: true,
            child: Stack(
              textDirection: TextDirection.ltr,
              children: <Widget>[
                Semantics(checked: true),
                Semantics(label: 'label', textDirection: TextDirection.ltr),
              ],
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              flags: SemanticsFlag.hasCheckedState.index | SemanticsFlag.isChecked.index,
              label: 'label',
              textDirection: TextDirection.ltr,
              rect: TestSemantics.fullScreen,
            ),
          ],
        ),
      ),
    );

    // switch the order of the inner Semantics node to trigger a reset
    await tester.pumpWidget(
      MergeSemantics(
        child: Semantics(
          container: true,
          child: Semantics(
            container: true,
            child: Stack(
              textDirection: TextDirection.ltr,
              children: <Widget>[
                Semantics(label: 'label', textDirection: TextDirection.ltr),
                Semantics(checked: true),
              ],
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              flags: SemanticsFlag.hasCheckedState.index | SemanticsFlag.isChecked.index,
              label: 'label',
              textDirection: TextDirection.ltr,
              rect: TestSemantics.fullScreen,
            ),
          ],
        ),
      ),
    );

    semantics.dispose();
  });
}
