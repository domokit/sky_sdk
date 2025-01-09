// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Semantics 5', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        fit: StackFit.expand,
        children: <Widget>[
          Semantics(
            // this tests that empty nodes disappear
          ),
          Semantics(
            // this tests whether you can have a container with no other semantics
            container: true,
          ),
          Semantics(
            label: 'label', // (force a fork)
            textDirection: TextDirection.ltr,
          ),
        ],
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(id: 1, rect: TestSemantics.fullScreen),
            TestSemantics.rootChild(id: 2, label: 'label', rect: TestSemantics.fullScreen),
          ],
        ),
      ),
    );

    semantics.dispose();
  });
}
