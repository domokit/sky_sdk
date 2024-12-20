// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('toString control test', (WidgetTester tester) async {
    final Widget widget = Title(
      color: const Color(0xFF00FF00),
      title: 'Awesome app',
      child: Container(),
    );
    expect(widget.toString, isNot(throwsException));
  });

  testWidgets('should handle having no title', (WidgetTester tester) async {
    final Title widget = Title(color: const Color(0xFF00FF00), child: Container());
    expect(widget.toString, isNot(throwsException));
    expect(widget.title, equals(''));
    expect(widget.color, equals(const Color(0xFF00FF00)));
  });

  testWidgets('should not allow non-opaque color', (WidgetTester tester) async {
    expect(() => Title(color: const Color(0x00000000), child: Container()), throwsAssertionError);
  });

  testWidgets('should not pass "null" to setApplicationSwitcherDescription', (
    WidgetTester tester,
  ) async {
    final List<MethodCall> log = <MethodCall>[];

    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    await tester.pumpWidget(Title(color: const Color(0xFF00FF00), child: Container()));

    expect(log, hasLength(1));
    expect(
      log.single,
      isMethodCall(
        'SystemChrome.setApplicationSwitcherDescription',
        arguments: <String, dynamic>{'label': '', 'primaryColor': 4278255360},
      ),
    );
  });
}
