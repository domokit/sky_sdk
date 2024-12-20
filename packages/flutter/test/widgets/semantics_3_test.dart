// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Semantics 3', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    // implicit annotators
    await tester.pumpWidget(
      Semantics(
        container: true,
        child: Semantics(
          label: 'test',
          textDirection: TextDirection.ltr,
          child: Semantics(checked: true),
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
              label: 'test',
              rect: TestSemantics.fullScreen,
            ),
          ],
        ),
      ),
    );

    // remove one
    await tester.pumpWidget(Semantics(container: true, child: Semantics(checked: true)));

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              flags: SemanticsFlag.hasCheckedState.index | SemanticsFlag.isChecked.index,
              rect: TestSemantics.fullScreen,
            ),
          ],
        ),
      ),
    );

    // change what it says
    await tester.pumpWidget(
      Semantics(container: true, child: Semantics(label: 'test', textDirection: TextDirection.ltr)),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              label: 'test',
              textDirection: TextDirection.ltr,
              rect: TestSemantics.fullScreen,
            ),
          ],
        ),
      ),
    );

    // add a node
    await tester.pumpWidget(
      Semantics(
        container: true,
        child: Semantics(
          checked: true,
          child: Semantics(label: 'test', textDirection: TextDirection.ltr),
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
              label: 'test',
              rect: TestSemantics.fullScreen,
            ),
          ],
        ),
      ),
    );

    int changeCount = 0;
    tester.binding.pipelineOwner.semanticsOwner!.addListener(() {
      changeCount += 1;
    });

    // make no changes
    await tester.pumpWidget(
      Semantics(
        container: true,
        child: Semantics(
          checked: true,
          child: Semantics(label: 'test', textDirection: TextDirection.ltr),
        ),
      ),
    );

    expect(changeCount, 0);

    semantics.dispose();
  });
}
