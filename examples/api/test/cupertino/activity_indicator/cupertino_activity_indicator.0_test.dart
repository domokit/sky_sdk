// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/activity_indicator/cupertino_activity_indicator.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Default and customized cupertino activity indicators', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoIndicatorApp());

    // Cupertino activity indicator with default properties.
    final Finder firstIndicator = find.byType(CupertinoActivityIndicator).at(0);
    expect(tester.widget<CupertinoActivityIndicator>(firstIndicator).animating, true);
    expect(tester.widget<CupertinoActivityIndicator>(firstIndicator).radius, 10.0);

    // Cupertino activity indicator with custom radius and color.
    final Finder secondIndicator = find.byType(CupertinoActivityIndicator).at(1);
    expect(tester.widget<CupertinoActivityIndicator>(secondIndicator).animating, true);
    expect(tester.widget<CupertinoActivityIndicator>(secondIndicator).radius, 20.0);
    expect(
      tester.widget<CupertinoActivityIndicator>(secondIndicator).color,
      CupertinoColors.activeBlue,
    );

    // Cupertino activity indicator with custom radius and disabled animation.
    final Finder thirdIndicator = find.byType(CupertinoActivityIndicator).at(2);
    expect(tester.widget<CupertinoActivityIndicator>(thirdIndicator).animating, false);
    expect(tester.widget<CupertinoActivityIndicator>(thirdIndicator).radius, 20.0);
  });
}
