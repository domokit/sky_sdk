// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/radio/radio.toggleable.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('StreamBuilder listens to internal stream', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ToggleableExampleApp());

    expect(find.byType(Radio<int>), findsExactly(5));
    expect(find.text('Hercules Mulligan'), findsOne);
    expect(find.text('Eliza Hamilton'), findsOne);
    expect(find.text('Philip Schuyler'), findsOne);
    expect(find.text('Maria Reynolds'), findsOne);
    expect(find.text('Samuel Seabury'), findsOne);

    for (int i = 0; i < 5; i++) {
      await tester.tap(find.byType(Radio<int>).at(i));
      await tester.pump();
      expect(
        find.byWidgetPredicate((Widget widget) => widget is Radio<int> && widget.groupValue == i),
        findsExactly(5),
      );
    }
  });
}
