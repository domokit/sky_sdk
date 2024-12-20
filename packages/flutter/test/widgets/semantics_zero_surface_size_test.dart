// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('has only root node if surface size is 0x0', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(Semantics(selected: true));

    expect(
      semantics,
      hasSemantics(
        TestSemantics(
          id: 0,
          rect: const Rect.fromLTRB(0.0, 0.0, 2400.0, 1800.0),
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
              flags: <SemanticsFlag>[SemanticsFlag.hasSelectedState, SemanticsFlag.isSelected],
            ),
          ],
        ),
        ignoreTransform: true,
      ),
    );

    await tester.binding.setSurfaceSize(Size.zero);
    await tester.pumpAndSettle();

    expect(semantics, hasSemantics(TestSemantics(id: 0, rect: Rect.zero), ignoreTransform: true));

    await tester.binding.setSurfaceSize(null);
    semantics.dispose();
  });
}
