// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/services/system_chrome/system_chrome.set_system_u_i_overlay_style.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AppBar.systemOverlayStyle can change system overlays styles.', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.SystemOverlayStyleApp());

    final SystemUiOverlayStyle? firstStyle = SystemChrome.latestStyle;

    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();
    final SystemUiOverlayStyle? secondStyle = SystemChrome.latestStyle;
    expect(secondStyle?.statusBarColor, isNot(firstStyle?.statusBarColor));
    expect(secondStyle?.systemNavigationBarColor, isNot(firstStyle?.systemNavigationBarColor));

    await tester.tap(find.byType(ElevatedButton));
    await tester.pump();
    final SystemUiOverlayStyle? thirdStyle = SystemChrome.latestStyle;
    expect(thirdStyle?.statusBarColor, isNot(secondStyle?.statusBarColor));
    expect(thirdStyle?.systemNavigationBarColor, isNot(secondStyle?.systemNavigationBarColor));
  });
}
