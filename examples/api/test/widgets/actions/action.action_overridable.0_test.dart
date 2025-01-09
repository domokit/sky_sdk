// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/actions/action.action_overridable.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final _MockClipboard mockClipboard = _MockClipboard();

  testWidgets('Copies text on Ctrl-C', (WidgetTester tester) async {
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      mockClipboard.handleMethodCall,
    );
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: Center(child: example.VerificationCodeGenerator()))),
    );

    expect(primaryFocus, isNotNull);
    expect(mockClipboard.clipboardData, isNull);

    await tester.sendKeyDownEvent(LogicalKeyboardKey.control);
    await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.control);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);

    expect(mockClipboard.clipboardData?['text'], '111222333');
  });
}

class _MockClipboard {
  _MockClipboard();

  Map<String, dynamic>? clipboardData;

  Future<Object?> handleMethodCall(MethodCall methodCall) async {
    switch (methodCall.method) {
      case 'Clipboard.setData':
        clipboardData = methodCall.arguments as Map<String, dynamic>;
        return null;
    }
    if (methodCall.method.startsWith('Clipboard')) {
      throw StateError('unrecognized method call: ${methodCall.method}');
    }
    return null;
  }
}
