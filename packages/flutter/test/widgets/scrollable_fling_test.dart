// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

const TextStyle testFont = TextStyle(color: Color(0xFF00FF00));

Future<void> pumpTest(WidgetTester tester, TargetPlatform platform) async {
  await tester.pumpWidget(Container());
  await tester.pumpWidget(
    MaterialApp(
      theme: ThemeData(platform: platform),
      home: ColoredBox(
        color: const Color(0xFF111111),
        child: ListView.builder(
          dragStartBehavior: DragStartBehavior.down,
          itemBuilder: (BuildContext context, int index) {
            return Text('$index', style: testFont);
          },
        ),
      ),
    ),
  );
}

const double dragOffset = 213.82;

void main() {
  testWidgets('Flings on different platforms', (WidgetTester tester) async {
    double getCurrentOffset() {
      return tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels;
    }

    await pumpTest(tester, TargetPlatform.android);
    await tester.fling(find.byType(ListView), const Offset(0.0, -dragOffset), 1000.0);
    expect(getCurrentOffset(), dragOffset);
    await tester.pump(); // trigger fling
    expect(getCurrentOffset(), dragOffset);
    await tester.pump(const Duration(seconds: 5));
    final double androidResult = getCurrentOffset();
    // Regression test for https://github.com/flutter/flutter/issues/83632
    // Before changing these values, ensure the fling results in a distance that
    // makes sense. See issue for more context.
    expect(androidResult, greaterThan(408.0));
    expect(androidResult, lessThan(409.0));

    await pumpTest(tester, TargetPlatform.linux);
    await tester.fling(find.byType(ListView), const Offset(0.0, -dragOffset), 1000.0);
    expect(getCurrentOffset(), dragOffset);
    await tester.pump(); // trigger fling
    expect(getCurrentOffset(), dragOffset);
    await tester.pump(const Duration(seconds: 5));
    final double linuxResult = getCurrentOffset();

    await pumpTest(tester, TargetPlatform.windows);
    await tester.fling(find.byType(ListView), const Offset(0.0, -dragOffset), 1000.0);
    expect(getCurrentOffset(), dragOffset);
    await tester.pump(); // trigger fling
    expect(getCurrentOffset(), dragOffset);
    await tester.pump(const Duration(seconds: 5));
    final double windowsResult = getCurrentOffset();

    await pumpTest(tester, TargetPlatform.iOS);
    await tester.fling(find.byType(ListView), const Offset(0.0, -dragOffset), 1000.0);
    // Scroll starts ease into the scroll on iOS.
    expect(getCurrentOffset(), moreOrLessEquals(210.71026666666666));
    await tester.pump(); // trigger fling
    expect(getCurrentOffset(), moreOrLessEquals(210.71026666666666));
    await tester.pump(const Duration(seconds: 5));
    final double iOSResult = getCurrentOffset();

    await pumpTest(tester, TargetPlatform.macOS);
    await tester.fling(find.byType(ListView), const Offset(0.0, -dragOffset), 1000.0);
    // Scroll starts ease into the scroll on iOS.
    expect(getCurrentOffset(), moreOrLessEquals(210.71026666666666));
    await tester.pump(); // trigger fling
    expect(getCurrentOffset(), moreOrLessEquals(210.71026666666666));
    await tester.pump(const Duration(seconds: 5));
    final double macOSResult = getCurrentOffset();

    expect(androidResult, lessThan(iOSResult)); // iOS is slipperier than Android
    expect(macOSResult, lessThan(iOSResult)); // iOS is slipperier than macOS
    expect(macOSResult, lessThan(androidResult)); // Android is slipperier than macOS
    expect(linuxResult, lessThan(iOSResult)); // iOS is slipperier than Linux
    expect(macOSResult, lessThan(linuxResult)); // Linux is slipperier than macOS
    expect(windowsResult, lessThan(iOSResult)); // iOS is slipperier than Windows
    expect(macOSResult, lessThan(windowsResult)); // Windows is slipperier than macOS
    expect(windowsResult, equals(androidResult));
    expect(windowsResult, equals(androidResult));
    expect(linuxResult, equals(androidResult));
    expect(linuxResult, equals(androidResult));
  });

  testWidgets('fling and tap to stop', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          dragStartBehavior: DragStartBehavior.down,
          children: List<Widget>.generate(
            250,
            (int i) => GestureDetector(
              onTap: () {
                log.add('tap $i');
              },
              child: Text('$i', style: testFont),
            ),
          ),
        ),
      ),
    );

    expect(log, equals(<String>[]));
    await tester.tap(find.byType(Scrollable));
    await tester.pump(const Duration(milliseconds: 50));
    expect(log, equals(<String>['tap 21']));
    await tester.fling(find.byType(Scrollable), const Offset(0.0, -200.0), 1000.0);
    await tester.pump(const Duration(milliseconds: 50));
    expect(log, equals(<String>['tap 21']));
    await tester.tap(find.byType(Scrollable)); // should stop the fling but not tap anything
    await tester.pump(const Duration(milliseconds: 50));
    expect(log, equals(<String>['tap 21']));
    await tester.tap(find.byType(Scrollable));
    await tester.pump(const Duration(milliseconds: 50));
    expect(log, equals(<String>['tap 21', 'tap 35']));
  });

  testWidgets('fling and wait and tap', (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          dragStartBehavior: DragStartBehavior.down,
          children: List<Widget>.generate(
            250,
            (int i) => GestureDetector(
              onTap: () {
                log.add('tap $i');
              },
              child: Text('$i', style: testFont),
            ),
          ),
        ),
      ),
    );

    expect(log, equals(<String>[]));
    await tester.tap(find.byType(Scrollable));
    await tester.pump(const Duration(milliseconds: 50));
    expect(log, equals(<String>['tap 21']));
    await tester.fling(find.byType(Scrollable), const Offset(0.0, -200.0), 1000.0);
    await tester.pump(const Duration(milliseconds: 50));
    expect(log, equals(<String>['tap 21']));
    await tester.pump(
      const Duration(seconds: 50),
    ); // long wait, so the fling will have ended at the end of it
    expect(log, equals(<String>['tap 21']));
    await tester.tap(find.byType(Scrollable));
    await tester.pump(const Duration(milliseconds: 50));
    expect(log, equals(<String>['tap 21', 'tap 49']));
  });
}
