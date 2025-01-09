// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/app_lifecycle_listener/app_lifecycle_listener.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AppLifecycleListener example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AppLifecycleListenerExample());

    expect(find.textContaining('Current State:'), findsOneWidget);
    expect(find.textContaining('State History:'), findsOneWidget);
  });
}
