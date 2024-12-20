// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/text_field/cupertino_text_field.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoTextField has initial text', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoTextFieldApp());

    expect(find.byType(CupertinoTextField), findsOneWidget);
    expect(find.text('initial text'), findsOneWidget);

    await tester.enterText(find.byType(CupertinoTextField), 'new text');
    await tester.pump();

    expect(find.text('new text'), findsOneWidget);
  });
}
