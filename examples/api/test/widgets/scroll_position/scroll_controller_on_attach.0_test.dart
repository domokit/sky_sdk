// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scroll_position/scroll_controller_on_attach.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can toggle between scroll notification types', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ScrollControllerDemo());

    expect(find.byType(CustomScrollView), findsOneWidget);
    expect(find.text('Not Scrolling'), findsOneWidget);
    Material appBarMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
    );
    expect(appBarMaterial.color, Colors.redAccent[700]!.withOpacity(.85));
    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(CustomScrollView)),
    );
    await gesture.moveBy(const Offset(10.0, 10.0));
    await tester.pump();
    expect(find.text('Scrolling'), findsOneWidget);
    appBarMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
    );
    expect(appBarMaterial.color, Colors.green[800]!.withOpacity(.85));
    await gesture.up();
    await tester.pumpAndSettle();

    expect(find.text('Not Scrolling'), findsOneWidget);
    appBarMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
    );
  });
}
