// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/cupertino/context_menu/cupertino_context_menu.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can open cupertino context menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ContextMenuApp());

    final Offset logo = tester.getCenter(find.byType(FlutterLogo));
    expect(find.text('Favorite'), findsNothing);

    await tester.startGesture(logo);
    await tester.pumpAndSettle();
    expect(find.text('Favorite'), findsOneWidget);

    await tester.tap(find.text('Favorite'));
    await tester.pumpAndSettle();
    expect(find.text('Favorite'), findsNothing);
  });
}
