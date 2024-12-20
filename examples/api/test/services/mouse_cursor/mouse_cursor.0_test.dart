// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/services/mouse_cursor/mouse_cursor.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Uses Text Cursor', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MouseCursorExampleApp());

    expect(find.byType(MouseRegion), findsNWidgets(2)); // There's one in the MaterialApp
    final Finder mouseRegionFinder = find.ancestor(
      of: find.byType(Container),
      matching: find.byType(MouseRegion),
    );
    expect(mouseRegionFinder, findsOneWidget);
    expect(
      (tester.widget(mouseRegionFinder) as MouseRegion).cursor,
      equals(SystemMouseCursors.text),
    );
  });
}
