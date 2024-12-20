// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/segmented_button/segmented_button.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can use SegmentedButton.styleFrom to customize SegmentedButton', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SegmentedButtonApp());

    final Color unselectedBackgroundColor = Colors.grey[200]!;
    const Color unselectedForegroundColor = Colors.red;
    const Color selectedBackgroundColor = Colors.green;
    const Color selectedForegroundColor = Colors.white;

    Material getMaterial(String text) {
      return tester.widget<Material>(
        find.ancestor(of: find.text(text), matching: find.byType(Material)).first,
      );
    }

    // Verify the unselected button style.
    expect(getMaterial('Day').textStyle?.color, unselectedForegroundColor);
    expect(getMaterial('Day').color, unselectedBackgroundColor);

    // Verify the selected button style.
    expect(getMaterial('Week').textStyle?.color, selectedForegroundColor);
    expect(getMaterial('Week').color, selectedBackgroundColor);
  });
}
