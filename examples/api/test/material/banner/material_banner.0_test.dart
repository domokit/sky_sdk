// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/banner/material_banner.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Shows all elements', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MaterialBannerExampleApp());

    expect(find.byType(MaterialBanner), findsOneWidget);
    expect(find.byType(AppBar), findsOneWidget);
    expect(find.byType(TextButton), findsNWidgets(2));
    expect(find.text('Hello, I am a Material Banner'), findsOneWidget);
    expect(find.text('The MaterialBanner is below'), findsOneWidget);
    expect(find.text('OPEN'), findsOneWidget);
    expect(find.text('DISMISS'), findsOneWidget);
    expect(find.byIcon(Icons.agriculture_outlined), findsOneWidget);
  });

  testWidgets('BottomNavigationBar Updates Screen Content', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MaterialBannerExampleApp());

    expect(find.byType(MaterialBanner), findsOne);
    expect(find.text('Hello, I am a Material Banner'), findsOne);
    expect(find.byIcon(Icons.agriculture_outlined), findsOne);
    expect(find.widgetWithText(TextButton, 'OPEN'), findsOne);
    expect(find.widgetWithText(TextButton, 'DISMISS'), findsOne);
  });

  testWidgets('The banner is below the text saying so', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MaterialBannerExampleApp());

    expect(find.byType(MaterialBanner), findsOneWidget);
    expect(find.text('The MaterialBanner is below'), findsOneWidget);
    final double bannerY = tester.getCenter(find.byType(MaterialBanner)).dy;
    final double textY = tester.getCenter(find.text('The MaterialBanner is below')).dy;
    expect(bannerY, greaterThan(textY));
  });
}
