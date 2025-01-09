// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_widgets.dart';

class TestCustomPainter extends CustomPainter {
  TestCustomPainter({required this.log, required this.name});

  final List<String> log;
  final String name;

  @override
  void paint(Canvas canvas, Size size) {
    log.add(name);
  }

  @override
  bool shouldRepaint(TestCustomPainter oldPainter) {
    return name != oldPainter.name || log != oldPainter.log;
  }
}

void main() {
  testWidgets('Do we paint when coming back from a navigation', (WidgetTester tester) async {
    final List<String> log = <String>[];
    log.add('0');
    await tester.pumpWidget(
      MaterialApp(
        routes: <String, WidgetBuilder>{
          '/':
              (BuildContext context) => RepaintBoundary(
                child: RepaintBoundary(
                  child: FlipWidget(
                    left: CustomPaint(painter: TestCustomPainter(log: log, name: 'left')),
                    right: CustomPaint(painter: TestCustomPainter(log: log, name: 'right')),
                  ),
                ),
              ),
          '/second': (BuildContext context) => Container(),
        },
      ),
    );
    log.add('1');
    final NavigatorState navigator = tester.state<NavigatorState>(find.byType(Navigator));
    navigator.pushNamed('/second');
    log.add('2');
    expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 2);
    log.add('3');
    flipStatefulWidget(tester, skipOffstage: false);
    log.add('4');
    navigator.pop();
    log.add('5');
    expect(await tester.pumpAndSettle(const Duration(minutes: 1)), 2);
    log.add('6');
    flipStatefulWidget(tester);
    expect(await tester.pumpAndSettle(), 1);
    log.add('7');
    expect(log, <String>['0', 'left', '1', '2', '3', '4', '5', 'right', '6', 'left', '7']);
  });
}
