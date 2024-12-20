// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/chip/deletable_chip_attributes.delete_icon_box_constraints.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('RawChip.deleteIconBoxConstraints updates delete icon size constraints', (
    WidgetTester tester,
  ) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double padding = 8.0;

    await tester.pumpWidget(const example.DeleteIconBoxConstraintsApp());

    expect(tester.getSize(find.byType(RawChip).at(0)).width, equals(202.0));
    expect(tester.getSize(find.byType(RawChip).at(0)).height, equals(58.0));

    Offset chipToRight = tester.getTopRight(
      find.byWidget(
        tester.widget<Material>(
          find.descendant(of: find.byType(RawChip).at(0), matching: find.byType(Material)),
        ),
      ),
    );
    Offset deleteIconCenter = tester.getCenter(find.byIcon(Icons.cancel).at(0));
    expect(chipToRight.dx, deleteIconCenter.dx + (iconSize / 2) + padding + border);

    expect(tester.getSize(find.byType(RawChip).at(1)).width, equals(202.0));
    expect(tester.getSize(find.byType(RawChip).at(1)).height, equals(78.0));

    chipToRight = tester.getTopRight(
      find.byWidget(
        tester.widget<Material>(
          find.descendant(of: find.byType(RawChip).at(1), matching: find.byType(Material)),
        ),
      ),
    );
    deleteIconCenter = tester.getCenter(find.byIcon(Icons.cancel).at(1));
    expect(chipToRight.dx, deleteIconCenter.dx + (iconSize / 2) + padding + border);

    expect(tester.getSize(find.byType(RawChip).at(2)).width, equals(202.0));
    expect(tester.getSize(find.byType(RawChip).at(2)).height, equals(78.0));

    chipToRight = tester.getTopRight(
      find.byWidget(
        tester.widget<Material>(
          find.descendant(of: find.byType(RawChip).at(2), matching: find.byType(Material)),
        ),
      ),
    );
    deleteIconCenter = tester.getCenter(find.byIcon(Icons.cancel).at(2));
    expect(chipToRight.dx, deleteIconCenter.dx + (iconSize / 2) + padding + border);
  });
}
