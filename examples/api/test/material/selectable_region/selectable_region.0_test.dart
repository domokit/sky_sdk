// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/selectable_region/selectable_region.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Future<void> sendKeyCombination(WidgetTester tester, LogicalKeyboardKey key) async {
    final LogicalKeyboardKey modifier = switch (defaultTargetPlatform) {
      TargetPlatform.iOS || TargetPlatform.macOS => LogicalKeyboardKey.meta,
      _ => LogicalKeyboardKey.control,
    };
    await tester.sendKeyDownEvent(modifier);
    await tester.sendKeyDownEvent(key);
    await tester.sendKeyUpEvent(key);
    await tester.sendKeyUpEvent(modifier);
    await tester.pump();
  }

  testWidgets('The icon can be selected with the text', (WidgetTester tester) async {
    String? clipboard;
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
      MethodCall methodCall,
    ) async {
      if (methodCall.method == 'Clipboard.setData') {
        clipboard = (methodCall.arguments as Map<String, dynamic>)['text'] as String;
      }
      return null;
    });
    addTearDown(() {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        null,
      );
    });

    await tester.pumpWidget(const example.SelectableRegionExampleApp());

    await tester.tap(find.byIcon(Icons.key)); // Focus the application.
    await tester.pump();

    // Keyboard select all.
    await sendKeyCombination(tester, LogicalKeyboardKey.keyA);
    // Keyboard copy.
    await sendKeyCombination(tester, LogicalKeyboardKey.keyC);

    expect(clipboard, 'SelectableRegion SampleSelect this iconCustom Text');
  }, variant: TargetPlatformVariant.all());
}
