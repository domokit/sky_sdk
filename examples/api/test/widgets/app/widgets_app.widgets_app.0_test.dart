// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/app/widgets_app.widgets_app.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('WidgetsApp test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.WidgetsAppExampleApp());

    expect(find.text('Hello World'), findsOneWidget);
  });
}
