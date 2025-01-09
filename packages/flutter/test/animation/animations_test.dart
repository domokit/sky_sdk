// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../scheduler/scheduler_tester.dart';

class BogusCurve extends Curve {
  const BogusCurve();

  @override
  double transform(double t) => 100.0;
}

void main() {
  setUp(() {
    WidgetsFlutterBinding.ensureInitialized();
    WidgetsBinding.instance
      ..resetEpoch()
      ..platformDispatcher.onBeginFrame = null
      ..platformDispatcher.onDrawFrame = null;
  });

  test('toString control test', () {
    expect(kAlwaysCompleteAnimation, hasOneLineDescription);
    expect(kAlwaysDismissedAnimation, hasOneLineDescription);
    expect(const AlwaysStoppedAnimation<double>(0.5), hasOneLineDescription);
    CurvedAnimation curvedAnimation = CurvedAnimation(
      parent: kAlwaysDismissedAnimation,
      curve: Curves.ease,
    );
    expect(curvedAnimation, hasOneLineDescription);
    curvedAnimation.reverseCurve = Curves.elasticOut;
    expect(curvedAnimation, hasOneLineDescription);
    final AnimationController controller = AnimationController(
      duration: const Duration(milliseconds: 500),
      vsync: const TestVSync(),
    );
    controller
      ..value = 0.5
      ..reverse();
    curvedAnimation = CurvedAnimation(
      parent: controller,
      curve: Curves.ease,
      reverseCurve: Curves.elasticOut,
    );
    expect(curvedAnimation, hasOneLineDescription);
    controller.stop();
  });

  test('ProxyAnimation.toString control test', () {
    final ProxyAnimation animation = ProxyAnimation();
    expect(animation.value, 0.0);
    expect(animation.status, AnimationStatus.dismissed);
    expect(animation, hasOneLineDescription);
    animation.parent = kAlwaysDismissedAnimation;
    expect(animation, hasOneLineDescription);
  });

  test('ProxyAnimation set parent generates value changed', () {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    controller.value = 0.5;
    bool didReceiveCallback = false;
    final ProxyAnimation animation =
        ProxyAnimation()..addListener(() {
          didReceiveCallback = true;
        });
    expect(didReceiveCallback, isFalse);
    animation.parent = controller;
    expect(didReceiveCallback, isTrue);
    didReceiveCallback = false;
    expect(didReceiveCallback, isFalse);
    controller.value = 0.6;
    expect(didReceiveCallback, isTrue);
  });

  test('ReverseAnimation calls listeners', () {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    controller.value = 0.5;
    bool didReceiveCallback = false;
    void listener() {
      didReceiveCallback = true;
    }

    final ReverseAnimation animation = ReverseAnimation(controller)..addListener(listener);
    expect(didReceiveCallback, isFalse);
    controller.value = 0.6;
    expect(didReceiveCallback, isTrue);
    didReceiveCallback = false;
    animation.removeListener(listener);
    expect(didReceiveCallback, isFalse);
    controller.value = 0.7;
    expect(didReceiveCallback, isFalse);
    expect(animation, hasOneLineDescription);
  });

  test('TrainHoppingAnimation', () {
    final AnimationController currentTrain = AnimationController(vsync: const TestVSync());
    final AnimationController nextTrain = AnimationController(vsync: const TestVSync());
    currentTrain.value = 0.5;
    nextTrain.value = 0.75;
    bool didSwitchTrains = false;
    final TrainHoppingAnimation animation = TrainHoppingAnimation(
      currentTrain,
      nextTrain,
      onSwitchedTrain: () {
        didSwitchTrains = true;
      },
    );
    expect(didSwitchTrains, isFalse);
    expect(animation.value, 0.5);
    expect(animation, hasOneLineDescription);
    nextTrain.value = 0.25;
    expect(didSwitchTrains, isTrue);
    expect(animation.value, 0.25);
    expect(animation, hasOneLineDescription);
    expect(animation.toString(), contains('no next'));
  });

  test('TrainHoppingAnimation dispatches memory events', () async {
    await expectLater(
      await memoryEvents(
        () =>
            TrainHoppingAnimation(
              const AlwaysStoppedAnimation<double>(1),
              const AlwaysStoppedAnimation<double>(1),
            ).dispose(),
        TrainHoppingAnimation,
      ),
      areCreateAndDispose,
    );
  });

  test('AnimationMean control test', () {
    final AnimationController left = AnimationController(value: 0.5, vsync: const TestVSync());
    final AnimationController right = AnimationController(vsync: const TestVSync());

    final AnimationMean mean = AnimationMean(left: left, right: right);

    expect(mean, hasOneLineDescription);
    expect(mean.value, equals(0.25));

    final List<double> log = <double>[];
    void logValue() {
      log.add(mean.value);
    }

    mean.addListener(logValue);

    right.value = 1.0;

    expect(mean.value, equals(0.75));
    expect(log, equals(<double>[0.75]));
    log.clear();

    mean.removeListener(logValue);

    left.value = 0.0;

    expect(mean.value, equals(0.50));
    expect(log, isEmpty);
  });

  test('AnimationMax control test', () {
    final AnimationController first = AnimationController(value: 0.5, vsync: const TestVSync());
    final AnimationController second = AnimationController(vsync: const TestVSync());

    final AnimationMax<double> max = AnimationMax<double>(first, second);

    expect(max, hasOneLineDescription);
    expect(max.value, equals(0.5));

    final List<double> log = <double>[];
    void logValue() {
      log.add(max.value);
    }

    max.addListener(logValue);

    second.value = 1.0;

    expect(max.value, equals(1.0));
    expect(log, equals(<double>[1.0]));
    log.clear();

    max.removeListener(logValue);

    first.value = 0.0;

    expect(max.value, equals(1.0));
    expect(log, isEmpty);
  });

  test('AnimationMin control test', () {
    final AnimationController first = AnimationController(value: 0.5, vsync: const TestVSync());
    final AnimationController second = AnimationController(vsync: const TestVSync());

    final AnimationMin<double> min = AnimationMin<double>(first, second);

    expect(min, hasOneLineDescription);
    expect(min.value, equals(0.0));

    final List<double> log = <double>[];
    void logValue() {
      log.add(min.value);
    }

    min.addListener(logValue);

    second.value = 1.0;

    expect(min.value, equals(0.5));
    expect(log, equals(<double>[0.5]));
    log.clear();

    min.removeListener(logValue);

    first.value = 0.25;

    expect(min.value, equals(0.25));
    expect(log, isEmpty);
  });

  test('CurvedAnimation with bogus curve', () {
    final AnimationController controller = AnimationController(vsync: const TestVSync());
    final CurvedAnimation curved = CurvedAnimation(parent: controller, curve: const BogusCurve());
    FlutterError? error;
    try {
      curved.value;
    } on FlutterError catch (e) {
      error = e;
    }
    expect(error, isNotNull);
    expect(
      error!.toStringDeep(),
      // RegExp matcher is required here due to flutter web and flutter mobile generating
      // slightly different floating point numbers
      // in Flutter web 0.0 sometimes just appears as 0. or 0
      matches(
        RegExp(r'''
FlutterError
   Invalid curve endpoint at \d+(\.\d*)?\.
   Curves must map 0\.0 to near zero and 1\.0 to near one but
   BogusCurve mapped \d+(\.\d*)? to \d+(\.\d*)?, which is near \d+(\.\d*)?\.
''', multiLine: true),
      ),
    );
  });

  test('CurvedAnimation running with different forward and reverse durations.', () {
    final AnimationController controller = AnimationController(
      duration: const Duration(milliseconds: 100),
      reverseDuration: const Duration(milliseconds: 50),
      vsync: const TestVSync(),
    );
    final CurvedAnimation curved = CurvedAnimation(
      parent: controller,
      curve: Curves.linear,
      reverseCurve: Curves.linear,
    );

    controller.forward();
    tick(Duration.zero);
    tick(const Duration(milliseconds: 10));
    expect(curved.value, moreOrLessEquals(0.1));
    tick(const Duration(milliseconds: 20));
    expect(curved.value, moreOrLessEquals(0.2));
    tick(const Duration(milliseconds: 30));
    expect(curved.value, moreOrLessEquals(0.3));
    tick(const Duration(milliseconds: 40));
    expect(curved.value, moreOrLessEquals(0.4));
    tick(const Duration(milliseconds: 50));
    expect(curved.value, moreOrLessEquals(0.5));
    tick(const Duration(milliseconds: 60));
    expect(curved.value, moreOrLessEquals(0.6));
    tick(const Duration(milliseconds: 70));
    expect(curved.value, moreOrLessEquals(0.7));
    tick(const Duration(milliseconds: 80));
    expect(curved.value, moreOrLessEquals(0.8));
    tick(const Duration(milliseconds: 90));
    expect(curved.value, moreOrLessEquals(0.9));
    tick(const Duration(milliseconds: 100));
    expect(curved.value, moreOrLessEquals(1.0));
    controller.reverse();
    tick(const Duration(milliseconds: 110));
    expect(curved.value, moreOrLessEquals(1.0));
    tick(const Duration(milliseconds: 120));
    expect(curved.value, moreOrLessEquals(0.8));
    tick(const Duration(milliseconds: 130));
    expect(curved.value, moreOrLessEquals(0.6));
    tick(const Duration(milliseconds: 140));
    expect(curved.value, moreOrLessEquals(0.4));
    tick(const Duration(milliseconds: 150));
    expect(curved.value, moreOrLessEquals(0.2));
    tick(const Duration(milliseconds: 160));
    expect(curved.value, moreOrLessEquals(0.0));
  });

  test('CurvedAnimation stops listening to parent when disposed.', () async {
    const Interval forwardCurve = Interval(0.0, 0.5);
    const Interval reverseCurve = Interval(0.5, 1.0);

    final AnimationController controller = AnimationController(
      duration: const Duration(milliseconds: 100),
      reverseDuration: const Duration(milliseconds: 100),
      vsync: const TestVSync(),
    );
    final CurvedAnimation curved = CurvedAnimation(
      parent: controller,
      curve: forwardCurve,
      reverseCurve: reverseCurve,
    );

    expect(forwardCurve.transform(0.5), 1.0);
    expect(reverseCurve.transform(0.5), 0.0);

    controller.forward(from: 0.5);
    expect(controller.status, equals(AnimationStatus.forward));
    expect(curved.value, equals(1.0));

    controller.value = 1.0;
    expect(controller.status, equals(AnimationStatus.completed));

    controller.reverse(from: 0.5);
    expect(controller.status, equals(AnimationStatus.reverse));
    expect(curved.value, equals(0.0));

    expect(curved.isDisposed, isFalse);
    curved.dispose();
    expect(curved.isDisposed, isTrue);

    controller.value = 0.0;
    expect(controller.status, equals(AnimationStatus.dismissed));

    controller.forward(from: 0.5);
    expect(controller.status, equals(AnimationStatus.forward));
    expect(curved.value, equals(0.0));
  });

  test('ReverseAnimation running with different forward and reverse durations.', () {
    final AnimationController controller = AnimationController(
      duration: const Duration(milliseconds: 100),
      reverseDuration: const Duration(milliseconds: 50),
      vsync: const TestVSync(),
    );
    final ReverseAnimation reversed = ReverseAnimation(
      CurvedAnimation(parent: controller, curve: Curves.linear, reverseCurve: Curves.linear),
    );

    controller.forward();
    tick(Duration.zero);
    tick(const Duration(milliseconds: 10));
    expect(reversed.value, moreOrLessEquals(0.9));
    tick(const Duration(milliseconds: 20));
    expect(reversed.value, moreOrLessEquals(0.8));
    tick(const Duration(milliseconds: 30));
    expect(reversed.value, moreOrLessEquals(0.7));
    tick(const Duration(milliseconds: 40));
    expect(reversed.value, moreOrLessEquals(0.6));
    tick(const Duration(milliseconds: 50));
    expect(reversed.value, moreOrLessEquals(0.5));
    tick(const Duration(milliseconds: 60));
    expect(reversed.value, moreOrLessEquals(0.4));
    tick(const Duration(milliseconds: 70));
    expect(reversed.value, moreOrLessEquals(0.3));
    tick(const Duration(milliseconds: 80));
    expect(reversed.value, moreOrLessEquals(0.2));
    tick(const Duration(milliseconds: 90));
    expect(reversed.value, moreOrLessEquals(0.1));
    tick(const Duration(milliseconds: 100));
    expect(reversed.value, moreOrLessEquals(0.0));
    controller.reverse();
    tick(const Duration(milliseconds: 110));
    expect(reversed.value, moreOrLessEquals(0.0));
    tick(const Duration(milliseconds: 120));
    expect(reversed.value, moreOrLessEquals(0.2));
    tick(const Duration(milliseconds: 130));
    expect(reversed.value, moreOrLessEquals(0.4));
    tick(const Duration(milliseconds: 140));
    expect(reversed.value, moreOrLessEquals(0.6));
    tick(const Duration(milliseconds: 150));
    expect(reversed.value, moreOrLessEquals(0.8));
    tick(const Duration(milliseconds: 160));
    expect(reversed.value, moreOrLessEquals(1.0));
  });

  test('TweenSequence', () {
    final AnimationController controller = AnimationController(vsync: const TestVSync());

    final Animation<double> animation = TweenSequence<double>(<TweenSequenceItem<double>>[
      TweenSequenceItem<double>(tween: Tween<double>(begin: 5.0, end: 10.0), weight: 4.0),
      TweenSequenceItem<double>(tween: ConstantTween<double>(10.0), weight: 2.0),
      TweenSequenceItem<double>(tween: Tween<double>(begin: 10.0, end: 5.0), weight: 4.0),
    ]).animate(controller);

    expect(animation.value, 5.0);

    controller.value = 0.2;
    expect(animation.value, 7.5);

    controller.value = 0.4;
    expect(animation.value, 10.0);

    controller.value = 0.6;
    expect(animation.value, 10.0);

    controller.value = 0.8;
    expect(animation.value, 7.5);

    controller.value = 1.0;
    expect(animation.value, 5.0);
  });

  test('TweenSequence with curves', () {
    final AnimationController controller = AnimationController(vsync: const TestVSync());

    final Animation<double> animation = TweenSequence<double>(<TweenSequenceItem<double>>[
      TweenSequenceItem<double>(
        tween: Tween<double>(
          begin: 5.0,
          end: 10.0,
        ).chain(CurveTween(curve: const Interval(0.5, 1.0))),
        weight: 4.0,
      ),
      TweenSequenceItem<double>(
        tween: ConstantTween<double>(
          10.0,
        ).chain(CurveTween(curve: Curves.linear)), // linear is a no-op
        weight: 2.0,
      ),
      TweenSequenceItem<double>(
        tween: Tween<double>(
          begin: 10.0,
          end: 5.0,
        ).chain(CurveTween(curve: const Interval(0.0, 0.5))),
        weight: 4.0,
      ),
    ]).animate(controller);

    expect(animation.value, 5.0);

    controller.value = 0.2;
    expect(animation.value, 5.0);

    controller.value = 0.4;
    expect(animation.value, 10.0);

    controller.value = 0.6;
    expect(animation.value, 10.0);

    controller.value = 0.8;
    expect(animation.value, 5.0);

    controller.value = 1.0;
    expect(animation.value, 5.0);
  });

  test('TweenSequence, one tween', () {
    final AnimationController controller = AnimationController(vsync: const TestVSync());

    final Animation<double> animation = TweenSequence<double>(<TweenSequenceItem<double>>[
      TweenSequenceItem<double>(tween: Tween<double>(begin: 5.0, end: 10.0), weight: 1.0),
    ]).animate(controller);

    expect(animation.value, 5.0);

    controller.value = 0.5;
    expect(animation.value, 7.5);

    controller.value = 1.0;
    expect(animation.value, 10.0);
  });

  test('$CurvedAnimation dispatches memory events', () async {
    await expectLater(
      await memoryEvents(
        () =>
            CurvedAnimation(
              parent: AnimationController(
                duration: const Duration(milliseconds: 100),
                vsync: const TestVSync(),
              ),
              curve: Curves.linear,
            ).dispose(),
        CurvedAnimation,
      ),
      areCreateAndDispose,
    );
  });
}
