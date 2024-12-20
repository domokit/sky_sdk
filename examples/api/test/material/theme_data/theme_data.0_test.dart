// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/theme_data/theme_data.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ThemeData basics', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ThemeDataExampleApp());

    final ColorScheme colorScheme = ColorScheme.fromSeed(seedColor: Colors.indigo);

    final Material fabMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(FloatingActionButton), matching: find.byType(Material)),
    );
    expect(fabMaterial.color, colorScheme.tertiary);

    final RichText iconRichText = tester.widget<RichText>(
      find.descendant(of: find.byIcon(Icons.add), matching: find.byType(RichText)),
    );
    expect(iconRichText.text.style!.color, colorScheme.onTertiary);

    expect(find.text('8 Points'), isNotNull);
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    expect(find.text('9 Points'), isNotNull);
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle();
    expect(find.text('10 Points'), isNotNull);
  });
}
