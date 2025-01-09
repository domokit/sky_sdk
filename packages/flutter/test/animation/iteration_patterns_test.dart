// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  setUp(() {
    WidgetsFlutterBinding.ensureInitialized();
    WidgetsBinding.instance.resetEpoch();
  });

  test('AnimationController with mutating listener', () {
    final AnimationController controller = AnimationController(
      duration: const Duration(milliseconds: 100),
      vsync: const TestVSync(),
    );
    final List<String> log = <String>[];

    void listener1() {
      log.add('listener1');
    }

    void listener3() {
      log.add('listener3');
    }

    void listener4() {
      log.add('listener4');
    }

    void listener2() {
      log.add('listener2');
      controller.removeListener(listener1);
      controller.removeListener(listener3);
      controller.addListener(listener4);
    }

    controller.addListener(listener1);
    controller.addListener(listener2);
    controller.addListener(listener3);
    controller.value = 0.2;
    expect(log, <String>['listener1', 'listener2']);
    log.clear();

    controller.value = 0.3;
    expect(log, <String>['listener2', 'listener4']);
    log.clear();

    controller.value = 0.4;
    expect(log, <String>['listener2', 'listener4']);
    log.clear();
  });

  test('AnimationController with mutating status listener', () {
    final AnimationController controller = AnimationController(
      duration: const Duration(milliseconds: 100),
      vsync: const TestVSync(),
    );
    final List<String> log = <String>[];

    void listener1(AnimationStatus status) {
      log.add('listener1');
    }

    void listener3(AnimationStatus status) {
      log.add('listener3');
    }

    void listener4(AnimationStatus status) {
      log.add('listener4');
    }

    void listener2(AnimationStatus status) {
      log.add('listener2');
      controller.removeStatusListener(listener1);
      controller.removeStatusListener(listener3);
      controller.addStatusListener(listener4);
    }

    controller.addStatusListener(listener1);
    controller.addStatusListener(listener2);
    controller.addStatusListener(listener3);
    controller.forward();
    expect(log, <String>['listener1', 'listener2']);
    log.clear();

    controller.reverse();
    expect(log, <String>['listener2', 'listener4']);
    log.clear();

    controller.forward();
    expect(log, <String>['listener2', 'listener4', 'listener4']);
    log.clear();

    controller.dispose();
  });

  testWidgets('AnimationController with throwing listener', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(
      duration: const Duration(milliseconds: 100),
      vsync: const TestVSync(),
    );
    addTearDown(controller.dispose);
    final List<String> log = <String>[];

    void listener1() {
      log.add('listener1');
    }

    void badListener() {
      log.add('badListener');
      throw ArgumentError();
    }

    void listener2() {
      log.add('listener2');
    }

    controller.addListener(listener1);
    controller.addListener(badListener);
    controller.addListener(listener2);
    controller.value = 0.2;
    expect(log, <String>['listener1', 'badListener', 'listener2']);
    expect(tester.takeException(), isArgumentError);
    log.clear();
  });

  testWidgets('AnimationController with throwing status listener', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(
      duration: const Duration(milliseconds: 100),
      vsync: const TestVSync(),
    );
    final List<String> log = <String>[];

    void listener1(AnimationStatus status) {
      log.add('listener1');
    }

    void badListener(AnimationStatus status) {
      log.add('badListener');
      throw ArgumentError();
    }

    void listener2(AnimationStatus status) {
      log.add('listener2');
    }

    controller.addStatusListener(listener1);
    controller.addStatusListener(badListener);
    controller.addStatusListener(listener2);
    controller.forward();
    expect(log, <String>['listener1', 'badListener', 'listener2']);
    expect(tester.takeException(), isArgumentError);
    log.clear();
    controller.dispose();
  });
}
