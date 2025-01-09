// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:splash/main.dart' as entrypoint;

void main() {
  testWidgets('Displays flutter logo and message', (WidgetTester tester) async {
    entrypoint.main();

    expect(find.byType(FlutterLogo), findsOneWidget);
    expect(
      find.text('This app is only meant to be run under the Flutter debugger'),
      findsOneWidget,
    );
  });
}
