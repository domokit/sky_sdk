// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

// This file is for tests for WidgetsBinding that require a `LiveTestWidgetsFlutterBinding`.
void main() {
  LiveTestWidgetsFlutterBinding();
  testWidgets('ReportTiming callback records the sendFramesToEngine when it was scheduled', (
    WidgetTester tester,
  ) async {
    // Addresses https://github.com/flutter/flutter/issues/144261
    // This test needs LiveTestWidgetsFlutterBinding for multiple reasons.
    //
    // First, this was the environment that this bug was discovered.
    //
    // Second, unlike `AutomatedTestWidgetsFlutterBinding`, which overrides
    // `scheduleWarmUpFrame` to execute the handlers synchronously,
    // `LiveTestWidgetsFlutterBinding` still calls them asynchronously. This
    // allows `runApp`, which also schedules a warm-up frame, to bind a widget
    // without rendering a frame, which is needed to for `deferFirstFrame` to
    // take effect.

    // Before `testWidgets` executes the test body, it pumps a frame with a
    // fixed dummy widget, then calls `resetFirstFrameSent`. The pumped frame
    // schedules a reportTiming call that has yet to arrive.
    //
    // This puts the test in an inconsistent state: a reportTiming callback is
    // supposed to happen only after a frame is rendered, but due to
    // `resetFirstFrameSent`, the framework thinks no frames have been rendered.

    expect(tester.binding.sendFramesToEngine, true);
    // Push the widget with `runApp` instead of `tester.pump`, avoiding
    // rendering a frame, which is needed for `deferFirstFrame` later to work.
    runApp(const DummyWidget());
    // Verify that no widget tree is built and nothing is rendered.
    expect(find.text('First frame'), findsNothing);
    // Defer the first frame, making `sendFramesToEngine` false, so that widget
    // tree will be built but not sent to the engine.
    tester.binding.deferFirstFrame();
    expect(tester.binding.sendFramesToEngine, false);
    // Pump a frame, letting the reportTiming callback to run. If the
    // reportTiming callback were to assume that `sendFramesToEngine` is true,
    // the callback would crash.
    await tester.pump(const Duration(milliseconds: 1));
    await tester.binding.waitUntilFirstFrameRasterized;
    expect(find.text('First frame'), findsOne);
  }, skip: kIsWeb); // [intended] Web doesn't use LiveTestWidgetsFlutterBinding
}

class DummyWidget extends StatelessWidget {
  const DummyWidget({super.key});

  @override
  Widget build(BuildContext context) {
    return const Directionality(
      textDirection: TextDirection.ltr,
      child: Center(child: Text('First frame')),
    );
  }
}
