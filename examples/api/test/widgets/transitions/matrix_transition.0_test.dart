// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/transitions/matrix_transition.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows Flutter logo inside a MatrixTransition', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MatrixTransitionExampleApp());

    final Finder transformFinder = find.ancestor(
      of: find.byType(FlutterLogo),
      matching: find.byType(MatrixTransition),
    );
    expect(transformFinder, findsOneWidget);
  });

  testWidgets('MatrixTransition animates', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MatrixTransitionExampleApp());

    final Finder transformFinder = find.ancestor(
      of: find.byType(FlutterLogo),
      matching: find.byType(Transform),
    );

    Transform transformBox = tester.widget(transformFinder);
    Matrix4 actualTransform = transformBox.transform;

    // Check initial transform.
    expect(
      actualTransform,
      Matrix4.fromList(<double>[
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.004,
        1.0,
      ])..transpose(),
    );

    // Animate half way.
    await tester.pump(const Duration(seconds: 1));
    transformBox = tester.widget(transformFinder);
    actualTransform = transformBox.transform;

    // The transform should be updated.
    expect(
      actualTransform,
      isNot(
        Matrix4.fromList(<double>[
          1.0,
          0.0,
          0.0,
          0.0,
          0.0,
          1.0,
          0.0,
          0.0,
          0.0,
          0.0,
          1.0,
          0.0,
          0.0,
          0.0,
          0.004,
          1.0,
        ])..transpose(),
      ),
    );
  });
}
