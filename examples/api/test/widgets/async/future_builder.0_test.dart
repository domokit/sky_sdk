// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/async/future_builder.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('StreamBuilder listens to internal stream', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FutureBuilderExampleApp());

    expect(find.byType(CircularProgressIndicator), findsOne);
    expect(find.text('Awaiting result...'), findsOne);

    await tester.pump(const Duration(seconds: 1));

    expect(find.byType(CircularProgressIndicator), findsOne);
    expect(find.text('Awaiting result...'), findsOne);

    await tester.pump(const Duration(seconds: 1));

    expect(find.byIcon(Icons.check_circle_outline), findsOne);
    expect(find.text('Result: Data Loaded'), findsOne);
  });
}
