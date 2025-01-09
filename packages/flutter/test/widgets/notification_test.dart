// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

class MyNotification extends Notification {}

void main() {
  testWidgets('Notification basics - toString', (WidgetTester tester) async {
    expect(MyNotification(), hasOneLineDescription);
  });

  testWidgets('Notification basics - dispatch', (WidgetTester tester) async {
    final List<dynamic> log = <dynamic>[];
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      NotificationListener<MyNotification>(
        onNotification: (MyNotification value) {
          log.add('a');
          log.add(value);
          return true;
        },
        child: NotificationListener<MyNotification>(
          onNotification: (MyNotification value) {
            log.add('b');
            log.add(value);
            return false;
          },
          child: Container(key: key),
        ),
      ),
    );
    expect(log, isEmpty);
    final Notification notification = MyNotification();
    expect(() {
      notification.dispatch(key.currentContext);
    }, isNot(throwsException));
    expect(log, <dynamic>['b', notification, 'a', notification]);
  });

  testWidgets('Notification basics - cancel', (WidgetTester tester) async {
    final List<dynamic> log = <dynamic>[];
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      NotificationListener<MyNotification>(
        onNotification: (MyNotification value) {
          log.add('a - error');
          log.add(value);
          return true;
        },
        child: NotificationListener<MyNotification>(
          onNotification: (MyNotification value) {
            log.add('b');
            log.add(value);
            return true;
          },
          child: Container(key: key),
        ),
      ),
    );
    expect(log, isEmpty);
    final Notification notification = MyNotification();
    expect(() {
      notification.dispatch(key.currentContext);
    }, isNot(throwsException));
    expect(log, <dynamic>['b', notification]);
  });

  testWidgets('Notification basics - listener null return value', (WidgetTester tester) async {
    final List<Type> log = <Type>[];
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      NotificationListener<MyNotification>(
        onNotification: (MyNotification value) {
          log.add(value.runtimeType);
          return false;
        },
        child: NotificationListener<MyNotification>(
          onNotification: (MyNotification value) => false,
          child: Container(key: key),
        ),
      ),
    );
    expect(() {
      MyNotification().dispatch(key.currentContext);
    }, isNot(throwsException));
    expect(log, <Type>[MyNotification]);
  });

  testWidgets('Notification basics - listener null return value', (WidgetTester tester) async {
    await tester.pumpWidget(const Placeholder());
    final ScrollMetricsNotification n1 = ScrollMetricsNotification(
      metrics: FixedScrollMetrics(
        minScrollExtent: 1.0,
        maxScrollExtent: 2.0,
        pixels: 3.0,
        viewportDimension: 4.0,
        axisDirection: AxisDirection.down,
        devicePixelRatio: 5.0,
      ),
      context: tester.allElements.first,
    );
    expect(n1.metrics.pixels, 3.0);
    final ScrollUpdateNotification n2 = n1.asScrollUpdate();
    expect(n2.metrics.pixels, 3.0);
  });
}
