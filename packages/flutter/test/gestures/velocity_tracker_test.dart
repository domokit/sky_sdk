// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';
import 'velocity_tracker_data.dart';

bool _withinTolerance(double actual, double expected) {
  const double kTolerance = 0.001; // Within .1% of expected value
  final double diff = (actual - expected) / expected;
  return diff.abs() < kTolerance;
}

bool _checkVelocity(Velocity actual, Offset expected) {
  return _withinTolerance(actual.pixelsPerSecond.dx, expected.dx) &&
      _withinTolerance(actual.pixelsPerSecond.dy, expected.dy);
}

void main() {
  const List<Offset> expected = <Offset>[
    Offset(219.59280094228163, 1304.701682306001),
    Offset(355.71046950050845, 967.2112857054104),
    Offset(12.657970884022308, -36.90447839251946),
    Offset(714.1399654786744, -2561.534447931869),
    Offset(-19.668121066218564, -2910.105747052462),
    Offset(646.8690114934209, 2976.977762577527),
    Offset(396.6988447819592, 2106.225572911095),
    Offset(298.31594440044495, -3660.8315955215294),
    Offset(-1.7334232785165882, -3288.13174127454),
    Offset(384.6361280392334, -2645.6612524779835),
    Offset(176.37900397918557, 2711.2542876273264),
    Offset(396.9328560260098, 4280.651578291764),
    Offset(-71.51939428321249, 3716.7385187526947),
  ];

  testWidgets('Velocity tracker gives expected results', (WidgetTester tester) async {
    final VelocityTracker tracker = VelocityTracker.withKind(PointerDeviceKind.touch);
    int i = 0;
    for (final PointerEvent event in velocityEventData) {
      if (event is PointerDownEvent || event is PointerMoveEvent) {
        tracker.addPosition(event.timeStamp, event.position);
      }
      if (event is PointerUpEvent) {
        expect(_checkVelocity(tracker.getVelocity(), expected[i]), isTrue);
        i += 1;
      }
    }
  });

  testWidgets('Velocity control test', (WidgetTester tester) async {
    const Velocity velocity1 = Velocity(pixelsPerSecond: Offset(7.0, 0.0));
    const Velocity velocity2 = Velocity(pixelsPerSecond: Offset(12.0, 0.0));
    expect(velocity1, equals(const Velocity(pixelsPerSecond: Offset(7.0, 0.0))));
    expect(velocity1, isNot(equals(velocity2)));
    expect(velocity2 - velocity1, equals(const Velocity(pixelsPerSecond: Offset(5.0, 0.0))));
    expect((-velocity1).pixelsPerSecond, const Offset(-7.0, 0.0));
    expect(velocity1 + velocity2, equals(const Velocity(pixelsPerSecond: Offset(19.0, 0.0))));
    expect(velocity1.hashCode, isNot(equals(velocity2.hashCode)));
    expect(velocity1, hasOneLineDescription);
  });

  testWidgets('Interrupted velocity estimation', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/pull/7510
    final VelocityTracker tracker = VelocityTracker.withKind(PointerDeviceKind.touch);
    for (final PointerEvent event in interruptedVelocityEventData) {
      if (event is PointerDownEvent || event is PointerMoveEvent) {
        tracker.addPosition(event.timeStamp, event.position);
      }
      if (event is PointerUpEvent) {
        expect(_checkVelocity(tracker.getVelocity(), const Offset(649.5, 3890.3)), isTrue);
      }
    }
  });

  testWidgets('No data velocity estimation', (WidgetTester tester) async {
    final VelocityTracker tracker = VelocityTracker.withKind(PointerDeviceKind.touch);
    expect(tracker.getVelocity(), Velocity.zero);
  });

  testWidgets('FreeScrollStartVelocityTracker.getVelocity throws when no points', (
    WidgetTester tester,
  ) async {
    final IOSScrollViewFlingVelocityTracker tracker = IOSScrollViewFlingVelocityTracker(
      PointerDeviceKind.touch,
    );
    AssertionError? exception;
    try {
      tracker.getVelocity();
    } on AssertionError catch (e) {
      exception = e;
    }

    expect(exception?.toString(), contains('at least 1 point'));
  });

  testWidgets(
    'FreeScrollStartVelocityTracker.getVelocity throws when the new point precedes the previous point',
    (WidgetTester tester) async {
      final IOSScrollViewFlingVelocityTracker tracker = IOSScrollViewFlingVelocityTracker(
        PointerDeviceKind.touch,
      );
      AssertionError? exception;

      tracker.addPosition(const Duration(hours: 1), Offset.zero);
      try {
        tracker.getVelocity();
        tracker.addPosition(const Duration(seconds: 1), Offset.zero);
      } on AssertionError catch (e) {
        exception = e;
      }

      expect(exception?.toString(), contains('has a smaller timestamp'));
    },
  );

  testWidgets('Estimate does not throw when there are more than 1 point', (
    WidgetTester tester,
  ) async {
    final IOSScrollViewFlingVelocityTracker tracker = IOSScrollViewFlingVelocityTracker(
      PointerDeviceKind.touch,
    );
    Offset position = Offset.zero;
    Duration time = Duration.zero;
    const Offset positionDelta = Offset(0, -1);
    const Duration durationDelta = Duration(seconds: 1);
    AssertionError? exception;

    for (int i = 0; i < 5; i += 1) {
      position += positionDelta;
      time += durationDelta;
      tracker.addPosition(time, position);

      try {
        tracker.getVelocity();
      } on AssertionError catch (e) {
        exception = e;
      }
      expect(exception, isNull);
    }
  });

  testWidgets('Makes consistent velocity estimates with consistent velocity', (
    WidgetTester tester,
  ) async {
    final IOSScrollViewFlingVelocityTracker tracker = IOSScrollViewFlingVelocityTracker(
      PointerDeviceKind.touch,
    );
    Offset position = Offset.zero;
    Duration time = Duration.zero;
    const Offset positionDelta = Offset(0, -1);
    const Duration durationDelta = Duration(seconds: 1);

    for (int i = 0; i < 10; i += 1) {
      position += positionDelta;
      time += durationDelta;
      tracker.addPosition(time, position);

      if (i >= 3) {
        expect(tracker.getVelocity().pixelsPerSecond, positionDelta);
      }
    }
  });

  testWidgets('Assume zero velocity when there are no recent samples - base VelocityTracker', (
    WidgetTester tester,
  ) async {
    final VelocityTracker tracker = VelocityTracker.withKind(PointerDeviceKind.touch);
    Offset position = Offset.zero;
    Duration time = Duration.zero;
    const Offset positionDelta = Offset(0, -1);
    const Duration durationDelta = Duration(seconds: 1);

    for (int i = 0; i < 10; i += 1) {
      position += positionDelta;
      time += durationDelta;
      tracker.addPosition(time, position);
    }
    await tester.pumpAndSettle();

    expect(tracker.getVelocity().pixelsPerSecond, Offset.zero);
  });

  testWidgets('Assume zero velocity when there are no recent samples - IOS', (
    WidgetTester tester,
  ) async {
    final IOSScrollViewFlingVelocityTracker tracker = IOSScrollViewFlingVelocityTracker(
      PointerDeviceKind.touch,
    );
    Offset position = Offset.zero;
    Duration time = Duration.zero;
    const Offset positionDelta = Offset(0, -1);
    const Duration durationDelta = Duration(seconds: 1);

    for (int i = 0; i < 10; i += 1) {
      position += positionDelta;
      time += durationDelta;
      tracker.addPosition(time, position);
    }
    await tester.pumpAndSettle();

    expect(tracker.getVelocity().pixelsPerSecond, Offset.zero);
  });

  testWidgets('Assume zero velocity when there are no recent samples - MacOS', (
    WidgetTester tester,
  ) async {
    final MacOSScrollViewFlingVelocityTracker tracker = MacOSScrollViewFlingVelocityTracker(
      PointerDeviceKind.touch,
    );
    Offset position = Offset.zero;
    Duration time = Duration.zero;
    const Offset positionDelta = Offset(0, -1);
    const Duration durationDelta = Duration(seconds: 1);

    for (int i = 0; i < 10; i += 1) {
      position += positionDelta;
      time += durationDelta;
      tracker.addPosition(time, position);
    }
    await tester.pumpAndSettle();

    expect(tracker.getVelocity().pixelsPerSecond, Offset.zero);
  });
}
