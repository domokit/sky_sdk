// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/clip_rrect.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ClipRRect fits to its child', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ClipRRectApp());

    final Finder clipRRectFinder = find.byType(ClipRRect);
    final Finder logoFinder = find.byType(FlutterLogo);
    expect(clipRRectFinder, findsOneWidget);
    expect(logoFinder, findsOneWidget);

    final Rect clipRect = tester.getRect(clipRRectFinder);
    final Rect containerRect = tester.getRect(logoFinder);
    expect(clipRect, equals(containerRect));
  });
}
