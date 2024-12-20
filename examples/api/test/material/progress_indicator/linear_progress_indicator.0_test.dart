// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/progress_indicator/linear_progress_indicator.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Determinate LinearProgressIndicator uses the provided value', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ProgressIndicatorExampleApp());
    await tester.pump(const Duration(milliseconds: 2500));

    final Finder indicatorFinder = find.byType(LinearProgressIndicator).first;
    final LinearProgressIndicator progressIndicator = tester.widget(indicatorFinder);
    expect(progressIndicator.value, equals(0.5));
  });

  testWidgets('Indeterminate LinearProgressIndicator does not have a value', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ProgressIndicatorExampleApp());
    await tester.pump(const Duration(milliseconds: 2500));

    final Finder indicatorFinder = find.byType(LinearProgressIndicator).last;
    final LinearProgressIndicator progressIndicator = tester.widget(indicatorFinder);
    expect(progressIndicator.value, null);
  });

  testWidgets('Progress indicators year2023 flag can be toggled', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ProgressIndicatorExampleApp());

    LinearProgressIndicator determinateIndicator = tester.widget<LinearProgressIndicator>(
      find.byType(LinearProgressIndicator).first,
    );
    expect(determinateIndicator.year2023, true);
    LinearProgressIndicator indeterminateIndicator = tester.widget<LinearProgressIndicator>(
      find.byType(LinearProgressIndicator).last,
    );
    expect(indeterminateIndicator.year2023, true);

    await tester.tap(find.byType(SwitchListTile));
    await tester.pump();

    determinateIndicator = tester.widget<LinearProgressIndicator>(
      find.byType(LinearProgressIndicator).first,
    );
    expect(determinateIndicator.year2023, false);
    indeterminateIndicator = tester.widget<LinearProgressIndicator>(
      find.byType(LinearProgressIndicator).last,
    );
    expect(indeterminateIndicator.year2023, false);
  });
}
