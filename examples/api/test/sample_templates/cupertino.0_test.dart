// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/sample_templates/cupertino.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

// This is an example of a test for API example code.
//
// It only tests that the example is presenting what it is supposed to, but you
// should also test the basic functionality of the example to make sure that it
// functions as expected.

void main() {
  testWidgets('Example app has a placeholder', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SampleApp());

    expect(find.byType(Placeholder), findsOneWidget);
  });
}
