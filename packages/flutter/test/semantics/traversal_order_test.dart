// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('Traversal order handles touching elements', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: List<Widget>.generate(3, (int column) {
            return Row(
              children: List<Widget>.generate(3, (int row) {
                return Semantics(
                  child: SizedBox(width: 50.0, height: 50.0, child: Text('$column - $row')),
                );
              }),
            );
          }),
        ),
      ),
    );

    final TestSemantics expected = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics(
          id: 1,
          textDirection: TextDirection.ltr,
          children: <TestSemantics>[
            TestSemantics(
              id: 2,
              children: <TestSemantics>[
                TestSemantics(
                  id: 3,
                  flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                  children: <TestSemantics>[
                    TestSemantics(id: 4, label: '0 - 0', textDirection: TextDirection.ltr),
                    TestSemantics(id: 5, label: '0 - 1', textDirection: TextDirection.ltr),
                    TestSemantics(id: 6, label: '0 - 2', textDirection: TextDirection.ltr),
                    TestSemantics(id: 7, label: '1 - 0', textDirection: TextDirection.ltr),
                    TestSemantics(id: 8, label: '1 - 1', textDirection: TextDirection.ltr),
                    TestSemantics(id: 9, label: '1 - 2', textDirection: TextDirection.ltr),
                    TestSemantics(id: 10, label: '2 - 0', textDirection: TextDirection.ltr),
                    TestSemantics(id: 11, label: '2 - 1', textDirection: TextDirection.ltr),
                    TestSemantics(id: 12, label: '2 - 2', textDirection: TextDirection.ltr),
                  ],
                ),
              ],
            ),
          ],
        ),
      ],
    );
    expect(semantics, hasSemantics(expected, ignoreRect: true, ignoreTransform: true));
    semantics.dispose();
  });
}
