// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/widgets/magnifier/cupertino_text_magnifier.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CupertinoTextMagnifier must be visible after longPress', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.CupertinoTextMagnifierApp());

    final Finder cupertinoTextFieldWidget = find.byType(CupertinoTextField);
    await tester.longPress(cupertinoTextFieldWidget);

    final Finder cupertinoTextMagnifierWidget = find.byType(CupertinoTextMagnifier);
    expect(cupertinoTextMagnifierWidget, findsOneWidget);
  });
}
