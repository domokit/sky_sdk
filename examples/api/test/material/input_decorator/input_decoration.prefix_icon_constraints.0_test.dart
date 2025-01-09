// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/input_decorator/input_decoration.prefix_icon_constraints.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows two TextFields decorated with prefix icon sizes matching their hint text', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.PrefixIconConstraintsExampleApp());
    expect(find.text('InputDecoration Sample'), findsOneWidget);

    expect(find.byType(TextField), findsNWidgets(2));
    expect(find.byIcon(Icons.search), findsNWidgets(2));
    expect(find.text('Normal Icon Constraints'), findsOneWidget);
    expect(find.text('Smaller Icon Constraints'), findsOneWidget);

    final Finder normalIcon = find.descendant(
      of: find.ancestor(of: find.text('Normal Icon Constraints'), matching: find.byType(TextField)),
      matching: find.byIcon(Icons.search),
    );
    final Finder smallerIcon = find.descendant(
      of: find.ancestor(
        of: find.text('Smaller Icon Constraints'),
        matching: find.byType(TextField),
      ),
      matching: find.byIcon(Icons.search),
    );

    expect(
      tester.getSize(normalIcon).longestSide,
      greaterThan(tester.getSize(smallerIcon).longestSide),
    );
  });

  testWidgets('prefixIcons are placed left of hintText', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PrefixIconConstraintsExampleApp());

    final Finder normalIcon = find.descendant(
      of: find.ancestor(of: find.text('Normal Icon Constraints'), matching: find.byType(TextField)),
      matching: find.byIcon(Icons.search),
    );
    final Finder smallerIcon = find.descendant(
      of: find.ancestor(
        of: find.text('Smaller Icon Constraints'),
        matching: find.byType(TextField),
      ),
      matching: find.byIcon(Icons.search),
    );

    expect(
      tester.getCenter(find.text('Normal Icon Constraints')).dx,
      greaterThan(tester.getCenter(normalIcon).dx),
    );
    expect(
      tester.getCenter(find.text('Smaller Icon Constraints')).dx,
      greaterThan(tester.getCenter(smallerIcon).dx),
    );
  });
}
