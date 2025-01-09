// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/button_style_button/button_style_button.icon_alignment.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ButtonStyleButton.iconAlignment updates button icons alignment', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.ButtonStyleButtonIconAlignmentApp());

    Finder findButtonMaterial(String text) {
      return find.ancestor(of: find.text(text), matching: find.byType(Material)).first;
    }

    void expectedLeftIconPosition({
      required double iconOffset,
      required double textButtonIconOffset,
    }) {
      expect(
        tester.getTopLeft(findButtonMaterial('ElevatedButton')).dx,
        tester.getTopLeft(find.byIcon(Icons.sunny)).dx - iconOffset,
      );
      expect(
        tester.getTopLeft(findButtonMaterial('FilledButton')).dx,
        tester.getTopLeft(find.byIcon(Icons.beach_access)).dx - iconOffset,
      );
      expect(
        tester.getTopLeft(findButtonMaterial('FilledButton Tonal')).dx,
        tester.getTopLeft(find.byIcon(Icons.cloud)).dx - iconOffset,
      );
      expect(
        tester.getTopLeft(findButtonMaterial('OutlinedButton')).dx,
        tester.getTopLeft(find.byIcon(Icons.light)).dx - iconOffset,
      );
      expect(
        tester.getTopLeft(findButtonMaterial('TextButton')).dx,
        tester.getTopLeft(find.byIcon(Icons.flight_takeoff)).dx - textButtonIconOffset,
      );
    }

    void expectedRightIconPosition({
      required double iconOffset,
      required double textButtonIconOffset,
    }) {
      expect(
        tester.getTopRight(findButtonMaterial('ElevatedButton')).dx,
        tester.getTopRight(find.byIcon(Icons.sunny)).dx + iconOffset,
      );
      expect(
        tester.getTopRight(findButtonMaterial('FilledButton')).dx,
        tester.getTopRight(find.byIcon(Icons.beach_access)).dx + iconOffset,
      );
      expect(
        tester.getTopRight(findButtonMaterial('FilledButton Tonal')).dx,
        tester.getTopRight(find.byIcon(Icons.cloud)).dx + iconOffset,
      );
      expect(
        tester.getTopRight(findButtonMaterial('OutlinedButton')).dx,
        tester.getTopRight(find.byIcon(Icons.light)).dx + iconOffset,
      );
      expect(
        tester.getTopRight(findButtonMaterial('TextButton')).dx,
        tester.getTopRight(find.byIcon(Icons.flight_takeoff)).dx + textButtonIconOffset,
      );
    }

    // Test initial icon alignment in LTR.
    expectedLeftIconPosition(iconOffset: 16, textButtonIconOffset: 12);

    // Update icon alignment to end.
    await tester.tap(find.text('end'));
    await tester.pumpAndSettle();

    // Test icon alignment end in LTR.
    expectedRightIconPosition(iconOffset: 24, textButtonIconOffset: 16);

    // Reset icon alignment to start.
    await tester.tap(find.text('start'));
    await tester.pumpAndSettle();

    // Change text direction to RTL.
    await tester.tap(find.text('RTL'));
    await tester.pumpAndSettle();

    // Test icon alignment start in LTR.
    expectedRightIconPosition(iconOffset: 16, textButtonIconOffset: 12);

    // Update icon alignment to end.
    await tester.tap(find.text('end'));
    await tester.pumpAndSettle();

    // Test icon alignment end in LTR.
    expectedLeftIconPosition(iconOffset: 24, textButtonIconOffset: 16);
  });
}
