// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/selection_area/selection_area.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SelectionArea SelectionListener Example Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SelectionAreaSelectionListenerExampleApp());
    expect(find.byType(Column), findsNWidgets(2));
    expect(find.textContaining('Selection StartOffset:'), findsOneWidget);
    expect(find.textContaining('Selection EndOffset:'), findsOneWidget);
    expect(find.textContaining('Selection Status:'), findsOneWidget);
    expect(find.textContaining('Selectable Region Status:'), findsOneWidget);
    expect(
      find.textContaining('This is some text under a SelectionArea that can be selected.'),
      findsOneWidget,
    );
  });
}
