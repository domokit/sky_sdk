// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('can change semantics in a branch blocked by BlockSemantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          id: 1,
          label: 'hello',
          textDirection: TextDirection.ltr,
          rect: TestSemantics.fullScreen,
        ),
      ],
    );

    await tester.pumpWidget(buildWidget(blockedText: 'one'));

    expect(semantics, hasSemantics(expectedSemantics));

    // The purpose of the test is to ensure that this change does not throw.
    await tester.pumpWidget(buildWidget(blockedText: 'two'));

    expect(semantics, hasSemantics(expectedSemantics));

    // Ensure that the previously blocked semantics end up in the tree correctly when unblocked.
    await tester.pumpWidget(buildWidget(blockedText: 'two', blocking: false));
    expect(semantics, includesNodeWith(label: 'two', textDirection: TextDirection.ltr));

    semantics.dispose();
  });
}

Widget buildWidget({required String blockedText, bool blocking = true}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: Stack(
      fit: StackFit.expand,
      children: <Widget>[
        Semantics(container: true, child: ListView(children: <Widget>[Text(blockedText)])),
        BlockSemantics(blocking: blocking, child: Semantics(label: 'hello', container: true)),
      ],
    ),
  );
}
