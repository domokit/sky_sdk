// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Animates forward when built', (WidgetTester tester) async {
    final List<int> values = <int>[];
    int endCount = 0;
    await tester.pumpWidget(
      TweenAnimationBuilder<int>(
        duration: const Duration(seconds: 1),
        tween: IntTween(begin: 10, end: 110),
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
        onEnd: () {
          endCount++;
        },
      ),
    );
    expect(endCount, 0);
    expect(values, <int>[10]);

    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[10, 60]);

    await tester.pump(const Duration(milliseconds: 501));
    expect(endCount, 1);
    expect(values, <int>[10, 60, 110]);

    await tester.pump(const Duration(milliseconds: 500));
    expect(endCount, 1);
    expect(values, <int>[10, 60, 110]);
  });

  testWidgets('No initial animation when begin=null', (WidgetTester tester) async {
    final List<int> values = <int>[];
    int endCount = 0;
    await tester.pumpWidget(
      TweenAnimationBuilder<int>(
        duration: const Duration(seconds: 1),
        tween: IntTween(end: 100),
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
        onEnd: () {
          endCount++;
        },
      ),
    );
    expect(endCount, 0);
    expect(values, <int>[100]);
    await tester.pump(const Duration(seconds: 2));
    expect(endCount, 0);
    expect(values, <int>[100]);
  });

  testWidgets('No initial animation when begin=end', (WidgetTester tester) async {
    final List<int> values = <int>[];
    int endCount = 0;
    await tester.pumpWidget(
      TweenAnimationBuilder<int>(
        duration: const Duration(seconds: 1),
        tween: IntTween(begin: 100, end: 100),
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
        onEnd: () {
          endCount++;
        },
      ),
    );
    expect(endCount, 0);
    expect(values, <int>[100]);
    await tester.pump(const Duration(seconds: 2));
    expect(endCount, 0);
    expect(values, <int>[100]);
  });

  testWidgets('Replace tween animates new tween', (WidgetTester tester) async {
    final List<int> values = <int>[];
    Widget buildWidget({required IntTween tween}) {
      return TweenAnimationBuilder<int>(
        duration: const Duration(seconds: 1),
        tween: tween,
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
      );
    }

    await tester.pumpWidget(buildWidget(tween: IntTween(begin: 0, end: 100)));
    expect(values, <int>[0]);
    await tester.pump(const Duration(seconds: 2)); // finish first animation.
    expect(values, <int>[0, 100]);

    await tester.pumpWidget(buildWidget(tween: IntTween(begin: 100, end: 200)));
    expect(values, <int>[0, 100, 100]);

    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[0, 100, 100, 150]);

    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[0, 100, 100, 150, 200]);
  });

  testWidgets('Curve is respected', (WidgetTester tester) async {
    final List<int> values = <int>[];
    Widget buildWidget({required IntTween tween, required Curve curve}) {
      return TweenAnimationBuilder<int>(
        duration: const Duration(seconds: 1),
        tween: tween,
        curve: curve,
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
      );
    }

    await tester.pumpWidget(
      buildWidget(tween: IntTween(begin: 0, end: 100), curve: Curves.easeInExpo),
    );
    expect(values, <int>[0]);
    await tester.pump(const Duration(milliseconds: 500));
    expect(values.last, lessThan(50));
    expect(values.last, greaterThan(0));

    await tester.pump(const Duration(seconds: 2)); // finish animation.

    values.clear();
    // Update curve (and tween to re-trigger animation).
    await tester.pumpWidget(
      buildWidget(tween: IntTween(begin: 100, end: 200), curve: Curves.linear),
    );
    expect(values, <int>[100]);
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[100, 150]);
  });

  testWidgets('Duration is respected', (WidgetTester tester) async {
    final List<int> values = <int>[];
    Widget buildWidget({required IntTween tween, required Duration duration}) {
      return TweenAnimationBuilder<int>(
        tween: tween,
        duration: duration,
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
      );
    }

    await tester.pumpWidget(
      buildWidget(tween: IntTween(begin: 0, end: 100), duration: const Duration(seconds: 1)),
    );
    expect(values, <int>[0]);
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[0, 50]);

    await tester.pump(const Duration(seconds: 2)); // finish animation.

    values.clear();
    // Update duration (and tween to re-trigger animation).
    await tester.pumpWidget(
      buildWidget(tween: IntTween(begin: 100, end: 200), duration: const Duration(seconds: 2)),
    );
    expect(values, <int>[100]);
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[100, 125]);
  });

  testWidgets('Child is integrated into tree', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: TweenAnimationBuilder<int>(
          tween: IntTween(begin: 0, end: 100),
          duration: const Duration(seconds: 1),
          builder: (BuildContext context, int i, Widget? child) {
            return child!;
          },
          child: const Text('Hello World'),
        ),
      ),
    );

    expect(find.text('Hello World'), findsOneWidget);
  });

  group('Change tween gapless while', () {
    testWidgets('running forward', (WidgetTester tester) async {
      final List<int> values = <int>[];
      Widget buildWidget({required IntTween tween}) {
        return TweenAnimationBuilder<int>(
          tween: tween,
          duration: const Duration(seconds: 1),
          builder: (BuildContext context, int i, Widget? child) {
            values.add(i);
            return const Placeholder();
          },
        );
      }

      await tester.pumpWidget(buildWidget(tween: IntTween(begin: 0, end: 100)));
      expect(values, <int>[0]);
      await tester.pump(const Duration(milliseconds: 500));
      expect(values, <int>[0, 50]);

      // Change tween
      await tester.pumpWidget(buildWidget(tween: IntTween(begin: 200, end: 300)));
      expect(values, <int>[0, 50, 50]); // gapless: animation continues where it left off.

      await tester.pump(const Duration(milliseconds: 500));
      expect(values, <int>[0, 50, 50, 175]); // 175 = halfway between 50 and new target 300.

      // Run animation to end
      await tester.pump(const Duration(seconds: 2));
      expect(values, <int>[0, 50, 50, 175, 300]);
      values.clear();
    });

    testWidgets('running forward and then reverse with same tween instance', (
      WidgetTester tester,
    ) async {
      final List<int> values = <int>[];
      Widget buildWidget({required IntTween tween}) {
        return TweenAnimationBuilder<int>(
          tween: tween,
          duration: const Duration(seconds: 1),
          builder: (BuildContext context, int i, Widget? child) {
            values.add(i);
            return const Placeholder();
          },
        );
      }

      final IntTween tween1 = IntTween(begin: 0, end: 100);
      final IntTween tween2 = IntTween(begin: 200, end: 300);

      await tester.pumpWidget(buildWidget(tween: tween1));
      await tester.pump(const Duration(milliseconds: 500));
      await tester.pumpWidget(buildWidget(tween: tween2));
      await tester.pump(const Duration(milliseconds: 500));
      await tester.pump(const Duration(seconds: 2));
      expect(values, <int>[0, 50, 50, 175, 300]);
      values.clear();
    });
  });

  testWidgets('Changing tween while gapless tween change is in progress', (
    WidgetTester tester,
  ) async {
    final List<int> values = <int>[];
    Widget buildWidget({required IntTween tween}) {
      return TweenAnimationBuilder<int>(
        tween: tween,
        duration: const Duration(seconds: 1),
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
      );
    }

    final IntTween tween1 = IntTween(begin: 0, end: 100);
    final IntTween tween2 = IntTween(begin: 200, end: 300);
    final IntTween tween3 = IntTween(begin: 400, end: 501);

    await tester.pumpWidget(buildWidget(tween: tween1));
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[0, 50]);
    values.clear();

    // Change tween
    await tester.pumpWidget(buildWidget(tween: tween2));
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[50, 175]);
    values.clear();

    await tester.pumpWidget(buildWidget(tween: tween3));
    await tester.pump(const Duration(milliseconds: 500));
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[175, 338, 501]);
  });

  testWidgets('Changing curve while no animation is running does not trigger animation', (
    WidgetTester tester,
  ) async {
    final List<int> values = <int>[];
    Widget buildWidget({required Curve curve}) {
      return TweenAnimationBuilder<int>(
        tween: IntTween(begin: 0, end: 100),
        curve: curve,
        duration: const Duration(seconds: 1),
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
      );
    }

    await tester.pumpWidget(buildWidget(curve: Curves.linear));
    await tester.pump(const Duration(seconds: 2));
    expect(values, <int>[0, 100]);
    values.clear();

    await tester.pumpWidget(buildWidget(curve: Curves.easeInExpo));
    expect(values, <int>[100]);
    await tester.pump(const Duration(seconds: 2));
    expect(values, <int>[100]);
  });

  testWidgets('Setting same tween and direction does not trigger animation', (
    WidgetTester tester,
  ) async {
    final List<int> values = <int>[];
    Widget buildWidget({required IntTween tween}) {
      return TweenAnimationBuilder<int>(
        tween: tween,
        duration: const Duration(seconds: 1),
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
      );
    }

    await tester.pumpWidget(buildWidget(tween: IntTween(begin: 0, end: 100)));
    await tester.pump(const Duration(milliseconds: 500));
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[0, 50, 100]);
    values.clear();

    await tester.pumpWidget(buildWidget(tween: IntTween(begin: 0, end: 100)));
    await tester.pump(const Duration(milliseconds: 500));
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, everyElement(100));
  });

  testWidgets('Setting same tween and direction while gapless animation is in progress works', (
    WidgetTester tester,
  ) async {
    final List<int> values = <int>[];
    Widget buildWidget({required IntTween tween}) {
      return TweenAnimationBuilder<int>(
        tween: tween,
        duration: const Duration(seconds: 1),
        builder: (BuildContext context, int i, Widget? child) {
          values.add(i);
          return const Placeholder();
        },
      );
    }

    await tester.pumpWidget(buildWidget(tween: IntTween(begin: 0, end: 100)));
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[0, 50]);
    await tester.pumpWidget(buildWidget(tween: IntTween(begin: 200, end: 300)));
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[0, 50, 50, 175]);

    await tester.pumpWidget(buildWidget(tween: IntTween(begin: 200, end: 300)));
    expect(values, <int>[0, 50, 50, 175, 175]);
    await tester.pump(const Duration(milliseconds: 500));
    expect(values, <int>[0, 50, 50, 175, 175, 300]);

    values.clear();
    await tester.pump(const Duration(seconds: 2));
    expect(values, everyElement(300));
  });

  testWidgets('Works with nullable tweens', (WidgetTester tester) async {
    final List<Size?> values = <Size?>[];
    await tester.pumpWidget(
      TweenAnimationBuilder<Size?>(
        duration: const Duration(seconds: 1),
        tween: SizeTween(end: const Size(10, 10)),
        builder: (BuildContext context, Size? s, Widget? child) {
          values.add(s);
          return const Placeholder();
        },
      ),
    );
    expect(values, <Size>[const Size(10, 10)]);
    await tester.pump(const Duration(seconds: 2));
    expect(values, <Size>[const Size(10, 10)]);
  });
}
