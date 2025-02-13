// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/floating_action_button/floating_action_button.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FloatingActionButton variants', (WidgetTester tester) async {
    RawMaterialButton getRawMaterialButtonWidget(Finder finder) {
      return tester.widget<RawMaterialButton>(finder);
    }

    await tester.pumpWidget(const example.FloatingActionButtonExampleApp());

    final ThemeData theme = ThemeData();

    expect(find.byType(FloatingActionButton), findsNWidgets(4));
    expect(find.byIcon(Icons.add), findsNWidgets(4));

    final Finder smallFabMaterialButton = find.byType(RawMaterialButton).at(0);
    final RenderBox smallFabRenderBox = tester.renderObject(smallFabMaterialButton);
    expect(smallFabRenderBox.size, const Size(48.0, 48.0));
    expect(
      getRawMaterialButtonWidget(smallFabMaterialButton).fillColor,
      theme.colorScheme.primaryContainer,
    );
    expect(
      getRawMaterialButtonWidget(smallFabMaterialButton).shape,
      RoundedRectangleBorder(borderRadius: BorderRadius.circular(12.0)),
    );

    final Finder regularFABMaterialButton = find.byType(RawMaterialButton).at(1);
    final RenderBox regularFABRenderBox = tester.renderObject(regularFABMaterialButton);
    expect(regularFABRenderBox.size, const Size(56.0, 56.0));
    expect(
      getRawMaterialButtonWidget(regularFABMaterialButton).fillColor,
      theme.colorScheme.primaryContainer,
    );
    expect(
      getRawMaterialButtonWidget(regularFABMaterialButton).shape,
      RoundedRectangleBorder(borderRadius: BorderRadius.circular(16.0)),
    );

    final Finder largeFABMaterialButton = find.byType(RawMaterialButton).at(2);
    final RenderBox largeFABRenderBox = tester.renderObject(largeFABMaterialButton);
    expect(largeFABRenderBox.size, const Size(96.0, 96.0));
    expect(
      getRawMaterialButtonWidget(largeFABMaterialButton).fillColor,
      theme.colorScheme.primaryContainer,
    );
    expect(
      getRawMaterialButtonWidget(largeFABMaterialButton).shape,
      RoundedRectangleBorder(borderRadius: BorderRadius.circular(28.0)),
    );

    final Finder extendedFABMaterialButton = find.byType(RawMaterialButton).at(3);
    final RenderBox extendedFABRenderBox = tester.renderObject(extendedFABMaterialButton);
    expect(extendedFABRenderBox.size, within(distance: 0.01, from: const Size(110.3, 56.0)));
    expect(
      getRawMaterialButtonWidget(extendedFABMaterialButton).fillColor,
      theme.colorScheme.primaryContainer,
    );
    expect(
      getRawMaterialButtonWidget(extendedFABMaterialButton).shape,
      RoundedRectangleBorder(borderRadius: BorderRadius.circular(16.0)),
    );
  });
}
