// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/cupertino.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Activity indicator animate property works', (WidgetTester tester) async {
    await tester.pumpWidget(buildCupertinoActivityIndicator());
    expect(SchedulerBinding.instance.transientCallbackCount, equals(1));

    await tester.pumpWidget(buildCupertinoActivityIndicator(false));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));

    await tester.pumpWidget(Container());

    await tester.pumpWidget(buildCupertinoActivityIndicator(false));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));

    await tester.pumpWidget(buildCupertinoActivityIndicator());
    expect(SchedulerBinding.instance.transientCallbackCount, equals(1));
  });

  testWidgets('Activity indicator dark mode', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: MediaQuery(
          data: const MediaQueryData(),
          child: RepaintBoundary(
            key: key,
            child: const ColoredBox(
              color: CupertinoColors.white,
              child: CupertinoActivityIndicator(animating: false, radius: 35),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('activityIndicator.paused.light.png'));

    await tester.pumpWidget(
      Center(
        child: MediaQuery(
          data: const MediaQueryData(platformBrightness: Brightness.dark),
          child: RepaintBoundary(
            key: key,
            child: const ColoredBox(
              color: CupertinoColors.black,
              child: CupertinoActivityIndicator(animating: false, radius: 35),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('activityIndicator.paused.dark.png'));
  });

  testWidgets('Activity indicator 0% in progress', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: const ColoredBox(
            color: CupertinoColors.white,
            child: CupertinoActivityIndicator.partiallyRevealed(progress: 0),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('activityIndicator.inprogress.0.0.png'));
  });

  testWidgets('Activity indicator 30% in progress', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: const ColoredBox(
            color: CupertinoColors.white,
            child: CupertinoActivityIndicator.partiallyRevealed(progress: 0.5),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('activityIndicator.inprogress.0.3.png'));
  });

  testWidgets('Activity indicator 100% in progress', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: const ColoredBox(
            color: CupertinoColors.white,
            child: CupertinoActivityIndicator.partiallyRevealed(),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('activityIndicator.inprogress.1.0.png'));
  });

  // Regression test for https://github.com/flutter/flutter/issues/41345.
  testWidgets('has the correct corner radius', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoActivityIndicator(animating: false, radius: 100));

    // An earlier implementation for the activity indicator started drawing
    // the ticks at 9 o'clock, however, in order to support partially revealed
    // indicator (https://github.com/flutter/flutter/issues/29159), the
    // first tick was changed to be at 12 o'clock.
    expect(
      find.byType(CupertinoActivityIndicator),
      paints..rrect(rrect: const RRect.fromLTRBXY(-10, -100 / 3, 10, -100, 10, 10)),
    );
  });

  testWidgets('Can specify color', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: const ColoredBox(
            color: CupertinoColors.white,
            child: CupertinoActivityIndicator(
              animating: false,
              color: Color(0xFF5D3FD3),
              radius: 100,
            ),
          ),
        ),
      ),
    );

    expect(
      find.byType(CupertinoActivityIndicator),
      paints..rrect(
        rrect: const RRect.fromLTRBXY(-10, -100 / 3, 10, -100, 10, 10),
        color: const Color(0x935d3fd3),
      ),
    );
  });
}

Widget buildCupertinoActivityIndicator([bool? animating]) {
  return MediaQuery(
    data: const MediaQueryData(),
    child: CupertinoActivityIndicator(animating: animating ?? true),
  );
}
