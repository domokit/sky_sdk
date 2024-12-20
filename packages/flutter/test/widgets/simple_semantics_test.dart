// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Simple tree is simple', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(const Center(child: Text('Hello!', textDirection: TextDirection.ltr)));

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              label: 'Hello!',
              textDirection: TextDirection.ltr,
              rect: const Rect.fromLTRB(0.0, 0.0, 84.0, 14.0),
              transform: Matrix4.translationValues(358.0, 293.0, 0.0),
            ),
          ],
        ),
      ),
    );

    semantics.dispose();
  });

  testWidgets('Simple tree is simple - material', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    // Not using Text widget because of https://github.com/flutter/flutter/issues/12357.
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Semantics(label: 'Hello!', child: const SizedBox(width: 10.0, height: 10.0)),
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
              rect: const Rect.fromLTWH(0.0, 0.0, 800.0, 600.0),
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  rect: const Rect.fromLTWH(0.0, 0.0, 800.0, 600.0),
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      rect: const Rect.fromLTWH(0.0, 0.0, 800.0, 600.0),
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          label: 'Hello!',
                          textDirection: TextDirection.ltr,
                          rect: const Rect.fromLTRB(0.0, 0.0, 10.0, 10.0),
                          transform: Matrix4.translationValues(395.0, 295.0, 0.0),
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
      ),
    );

    semantics.dispose();
  });
}
