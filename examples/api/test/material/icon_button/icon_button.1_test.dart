// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/icon_button/icon_button.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('IconButton', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IconButtonExampleApp());

    await tester.pumpAndSettle();

    expect(find.byIcon(Icons.android), findsOneWidget);
    final Ink ink = tester.widget<Ink>(
      find.ancestor(of: find.byIcon(Icons.android), matching: find.byType(Ink)),
    );

    final ShapeDecoration decoration = ink.decoration! as ShapeDecoration;
    expect(decoration.color, Colors.lightBlue);
    expect(decoration.shape, const CircleBorder());

    final IconButton iconButton = ink.child! as IconButton;
    expect(iconButton.color, Colors.white);
  });
}
