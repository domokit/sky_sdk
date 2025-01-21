// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  testWidgets('Restoration Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());

    expect(findRoute('home', count: 0), findsOneWidget);
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    await tester.restartAndRestore();
    expect(findRoute('home', count: 1), findsOneWidget);

    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 2), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 3), findsOneWidget);

    await tester.restoreFrom(data);
    expect(findRoute('home', count: 2), findsOneWidget);
  });

  testWidgets('restorablePushNamed', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo', arguments: 3);
    await tester.pumpAndSettle();

    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('Foo', count: 0, arguments: 3), findsOneWidget);

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2, arguments: 3), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar', arguments: 4);
    await tester.pumpAndSettle();
    expect(findRoute('Bar', arguments: 4), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);
    expect(findRoute('Bar'), findsNothing);
  });

  testWidgets('restorablePushReplacementNamed', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePushReplacementNamed('Foo', arguments: 3);
    await tester.pumpAndSettle();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0, arguments: 3), findsOneWidget);

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2, arguments: 3), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar', arguments: 4);
    await tester.pumpAndSettle();
    expect(findRoute('Bar', arguments: 4), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);
    expect(findRoute('Bar'), findsNothing);
  });

  testWidgets('restorablePopAndPushNamed', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePopAndPushNamed('Foo', arguments: 3);
    await tester.pumpAndSettle();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0, arguments: 3), findsOneWidget);

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2, arguments: 3), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar', arguments: 4);
    await tester.pumpAndSettle();
    expect(findRoute('Bar', arguments: 4), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);
    expect(findRoute('Bar'), findsNothing);
  });

  testWidgets('restorablePushNamedAndRemoveUntil', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePushNamedAndRemoveUntil('Foo', (Route<dynamic> _) => false, arguments: 3);
    await tester.pumpAndSettle();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0, arguments: 3), findsOneWidget);

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2, arguments: 3), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar', arguments: 4);
    await tester.pumpAndSettle();
    expect(findRoute('Bar', arguments: 4), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1, arguments: 3), findsOneWidget);
    expect(findRoute('Bar'), findsNothing);
  });

  testWidgets('restorablePush', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePush(_routeBuilder, arguments: 'Foo');
    await tester.pumpAndSettle();

    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('Foo', count: 0), findsOneWidget);

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 1), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('Foo', count: 1), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar');
    await tester.pumpAndSettle();
    expect(findRoute('Bar'), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('Foo', count: 1), findsOneWidget);
    expect(findRoute('Bar'), findsNothing);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets('restorablePush adds route on all platforms', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePush(_routeBuilder, arguments: 'Foo');
    await tester.pumpAndSettle();
    expect(findRoute('Foo'), findsOneWidget);
  });

  testWidgets('restorablePushReplacement', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home', count: 0), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePushReplacement(_routeBuilder, arguments: 'Foo');
    await tester.pumpAndSettle();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0), findsOneWidget);

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 1), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar');
    await tester.pumpAndSettle();
    expect(findRoute('Bar'), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1), findsOneWidget);
    expect(findRoute('Bar'), findsNothing);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets('restorablePushReplacement adds route on all platforms', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePushReplacement(_routeBuilder, arguments: 'Foo');
    await tester.pumpAndSettle();
    expect(findRoute('Foo'), findsOneWidget);
  });

  testWidgets('restorablePushAndRemoveUntil', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home', count: 0), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePushAndRemoveUntil(_routeBuilder, (Route<dynamic> _) => false, arguments: 'Foo');
    await tester.pumpAndSettle();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0), findsOneWidget);

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 1), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar');
    await tester.pumpAndSettle();
    expect(findRoute('Bar'), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1), findsOneWidget);
    expect(findRoute('Bar'), findsNothing);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets('restorablePushAndRemoveUntil adds route on all platforms', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const TestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePushAndRemoveUntil(_routeBuilder, (Route<dynamic> _) => false, arguments: 'Foo');
    await tester.pumpAndSettle();
    expect(findRoute('Foo'), findsOneWidget);
  });

  testWidgets('restorableReplace', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home', count: 0), findsOneWidget);

    final Route<Object> oldRoute = ModalRoute.of(tester.element(find.text('Route: home')))!;
    expect(oldRoute.settings.name, 'home');

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorableReplace(newRouteBuilder: _routeBuilder, arguments: 'Foo', oldRoute: oldRoute);
    await tester.pumpAndSettle();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0), findsOneWidget);

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 1), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar');
    await tester.pumpAndSettle();
    expect(findRoute('Bar'), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 1), findsOneWidget);
    expect(findRoute('Bar'), findsNothing);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets('restorableReplace adds route on all platforms', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home', count: 0), findsOneWidget);

    final Route<Object> oldRoute = ModalRoute.of(tester.element(find.text('Route: home')))!;
    expect(oldRoute.settings.name, 'home');

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorableReplace(newRouteBuilder: _routeBuilder, arguments: 'Foo', oldRoute: oldRoute);
    await tester.pumpAndSettle();
    expect(findRoute('Foo'), findsOneWidget);
  });

  testWidgets('restorableReplaceRouteBelow', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home', count: 0), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Anchor');
    await tester.pumpAndSettle();

    await tapRouteCounter('Anchor', tester);
    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 0, skipOffstage: false), findsOneWidget);
    expect(findRoute('Anchor', count: 1), findsOneWidget);

    final Route<Object> anchor = ModalRoute.of(tester.element(find.text('Route: Anchor')))!;
    expect(anchor.settings.name, 'Anchor');

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorableReplaceRouteBelow(
          newRouteBuilder: _routeBuilder,
          arguments: 'Foo',
          anchorRoute: anchor,
        );
    await tester.pumpAndSettle();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0, skipOffstage: false), findsOneWidget);
    expect(findRoute('Anchor', count: 1), findsOneWidget);

    await tapRouteCounter('Anchor', tester);
    expect(findRoute('Anchor', count: 2), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0, skipOffstage: false), findsOneWidget);
    expect(findRoute('Anchor', count: 2), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Anchor', tester);
    expect(findRoute('Anchor', count: 3), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar');
    await tester.pumpAndSettle();
    expect(findRoute('Bar'), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('home', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 0, skipOffstage: false), findsOneWidget);
    expect(findRoute('Anchor', count: 2), findsOneWidget);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets('restorableReplaceRouteBelow adds route on all platforms', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home', count: 0), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Anchor');
    await tester.pumpAndSettle();

    await tapRouteCounter('Anchor', tester);
    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 0, skipOffstage: false), findsOneWidget);
    expect(findRoute('Anchor', count: 1), findsOneWidget);

    final Route<Object> anchor = ModalRoute.of(tester.element(find.text('Route: Anchor')))!;
    expect(anchor.settings.name, 'Anchor');

    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorableReplaceRouteBelow(
          newRouteBuilder: _routeBuilder,
          arguments: 'Foo',
          anchorRoute: anchor,
        );
    await tester.pumpAndSettle();
    expect(findRoute('Foo', skipOffstage: false), findsOneWidget);
  });

  testWidgets('restoring a popped route', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo');
    await tester.pumpAndSettle();

    await tapRouteCounter('Foo', tester);
    await tapRouteCounter('Foo', tester);
    expect(findRoute('home'), findsNothing);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('Foo', count: 2), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();

    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 3), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('Foo'), findsNothing);

    await tester.restoreFrom(data);

    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('Foo', count: 2), findsOneWidget);
  });

  testWidgets('popped routes are not restored', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo');
    await tester.pumpAndSettle();
    expect(findRoute('Foo'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar');
    await tester.pumpAndSettle();
    expect(findRoute('Bar'), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();

    expect(findRoute('Bar'), findsNothing);
    expect(findRoute('Foo'), findsOneWidget);
    expect(findRoute('home', skipOffstage: false), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('Bar'), findsNothing);
    expect(findRoute('Foo'), findsOneWidget);
    expect(findRoute('home', skipOffstage: false), findsOneWidget);
  });

  testWidgets('routes that are in the process of push are restored', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo');
    await tester.pump();
    await tester.pump();
    expect(findRoute('Foo'), findsOneWidget);

    // Push is in progress.
    final ModalRoute<Object> route1 = ModalRoute.of(tester.element(find.text('Route: Foo')))!;
    final String route1id = route1.restorationScopeId.value!;
    expect(route1id, isNotNull);
    expect(route1.settings.name, 'Foo');
    expect(route1.animation!.isCompleted, isFalse);
    expect(route1.animation!.isDismissed, isFalse);
    expect(route1.isActive, isTrue);

    await tester.restartAndRestore();

    expect(findRoute('Foo'), findsOneWidget);
    expect(findRoute('home', skipOffstage: false), findsOneWidget);
    final ModalRoute<Object> route2 = ModalRoute.of(tester.element(find.text('Route: Foo')))!;
    expect(route2, isNot(same(route1)));
    expect(route1.restorationScopeId.value, route1id);
    expect(route2.animation!.isCompleted, isTrue);
    expect(route2.isActive, isTrue);
  });

  testWidgets('routes that are in the process of pop are not restored', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const TestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo');
    await tester.pumpAndSettle();

    final ModalRoute<Object> route1 = ModalRoute.of(tester.element(find.text('Route: Foo')))!;
    int notifyCount = 0;
    route1.restorationScopeId.addListener(() {
      notifyCount++;
    });
    expect(route1.isActive, isTrue);
    expect(route1.restorationScopeId.value, isNotNull);
    expect(route1.animation!.isCompleted, isTrue);
    expect(notifyCount, 0);

    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    expect(notifyCount, 1);
    await tester.pump();
    await tester.pump();

    // Pop is in progress.
    expect(route1.restorationScopeId.value, isNull);
    expect(route1.settings.name, 'Foo');
    expect(route1.animation!.isCompleted, isFalse);
    expect(route1.animation!.isDismissed, isFalse);
    expect(route1.isActive, isFalse);

    await tester.restartAndRestore();

    expect(findRoute('Foo', skipOffstage: false), findsNothing);
    expect(findRoute('home', count: 1), findsOneWidget);
    expect(notifyCount, 1);
  });

  testWidgets('routes are restored in the right order', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route1');
    await tester.pumpAndSettle();
    expect(findRoute('route1'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route2');
    await tester.pumpAndSettle();
    expect(findRoute('route2'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route3');
    await tester.pumpAndSettle();
    expect(findRoute('route3'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route4');
    await tester.pumpAndSettle();
    expect(findRoute('route4'), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('route4'), findsOneWidget);
    expect(findRoute('route3', skipOffstage: false), findsOneWidget);
    expect(findRoute('route2', skipOffstage: false), findsOneWidget);
    expect(findRoute('route1', skipOffstage: false), findsOneWidget);
    expect(findRoute('home', skipOffstage: false), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('route3'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('route2'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('route1'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('home'), findsOneWidget);
  });

  testWidgets('all routes up to first unrestorable are restored', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route1');
    await tester.pumpAndSettle();
    expect(findRoute('route1'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route2');
    await tester.pumpAndSettle();
    expect(findRoute('route2'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('route3');
    await tester.pumpAndSettle();
    expect(findRoute('route3'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route4');
    await tester.pumpAndSettle();
    expect(findRoute('route4'), findsOneWidget);
    tester
        .state<NavigatorState>(find.byType(Navigator))
        .restorablePush(_routeBuilder, arguments: 'route5');
    await tester.pumpAndSettle();
    expect(findRoute('route5'), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('route5', skipOffstage: false), findsNothing);
    expect(findRoute('route4', skipOffstage: false), findsNothing);
    expect(findRoute('route3', skipOffstage: false), findsNothing);

    expect(findRoute('route2'), findsOneWidget);
    expect(findRoute('route1', skipOffstage: false), findsOneWidget);
    expect(findRoute('home', skipOffstage: false), findsOneWidget);
  });

  testWidgets('removing unrestorable routes restores all of them', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route1');
    await tester.pumpAndSettle();
    expect(findRoute('route1'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route2');
    await tester.pumpAndSettle();
    expect(findRoute('route2'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('route3');
    await tester.pumpAndSettle();
    expect(findRoute('route3'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route4');
    await tester.pumpAndSettle();
    expect(findRoute('route4'), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('route5');
    await tester.pumpAndSettle();
    expect(findRoute('route5'), findsOneWidget);

    final Route<Object> route =
        ModalRoute.of(tester.element(find.text('Route: route3', skipOffstage: false)))!;
    expect(route.settings.name, 'route3');
    tester.state<NavigatorState>(find.byType(Navigator)).removeRoute(route);
    await tester.pumpAndSettle();

    await tester.restartAndRestore();

    expect(findRoute('route5'), findsOneWidget);
    expect(findRoute('route4', skipOffstage: false), findsOneWidget);
    expect(findRoute('route3', skipOffstage: false), findsNothing);
    expect(findRoute('route2', skipOffstage: false), findsOneWidget);
    expect(findRoute('route1', skipOffstage: false), findsOneWidget);
    expect(findRoute('home', skipOffstage: false), findsOneWidget);
  });

  testWidgets('RestorableRouteFuture', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePush(_routeFutureBuilder);
    await tester.pumpAndSettle();
    expect(find.text('Return value: null'), findsOneWidget);

    final RestorableRouteFuture<int> routeFuture =
        tester.state<RouteFutureWidgetState>(find.byType(RouteFutureWidget)).routeFuture;
    expect(routeFuture.route, isNull);
    expect(routeFuture.isPresent, isFalse);
    expect(routeFuture.enabled, isFalse);

    routeFuture.present('Foo');
    await tester.pumpAndSettle();
    expect(find.text('Route: Foo'), findsOneWidget);
    expect(routeFuture.route!.settings.name, 'Foo');
    expect(routeFuture.isPresent, isTrue);
    expect(routeFuture.enabled, isTrue);

    await tester.restartAndRestore();

    expect(find.text('Route: Foo'), findsOneWidget);
    final RestorableRouteFuture<int> restoredRouteFuture =
        tester
            .state<RouteFutureWidgetState>(find.byType(RouteFutureWidget, skipOffstage: false))
            .routeFuture;
    expect(restoredRouteFuture.route!.settings.name, 'Foo');
    expect(restoredRouteFuture.isPresent, isTrue);
    expect(restoredRouteFuture.enabled, isTrue);

    tester.state<NavigatorState>(find.byType(Navigator)).pop(10);
    await tester.pumpAndSettle();
    expect(find.text('Return value: 10'), findsOneWidget);
    expect(restoredRouteFuture.route, isNull);
    expect(restoredRouteFuture.isPresent, isFalse);
    expect(restoredRouteFuture.enabled, isFalse);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets('RestorableRouteFuture in unrestorable context', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    expect(findRoute('home'), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('unrestorable');
    await tester.pumpAndSettle();
    expect(findRoute('unrestorable'), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePush(_routeFutureBuilder);
    await tester.pumpAndSettle();
    expect(find.text('Return value: null'), findsOneWidget);

    final RestorableRouteFuture<int> routeFuture =
        tester.state<RouteFutureWidgetState>(find.byType(RouteFutureWidget)).routeFuture;
    expect(routeFuture.route, isNull);
    expect(routeFuture.isPresent, isFalse);
    expect(routeFuture.enabled, isFalse);

    routeFuture.present('Foo');
    await tester.pumpAndSettle();
    expect(find.text('Route: Foo'), findsOneWidget);
    expect(routeFuture.route!.settings.name, 'Foo');
    expect(routeFuture.isPresent, isTrue);
    expect(routeFuture.enabled, isFalse);

    await tester.restartAndRestore();

    expect(findRoute('home'), findsOneWidget);
  });

  testWidgets('Illegal arguments throw', (WidgetTester tester) async {
    await tester.pumpWidget(const TestWidget());
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Bar');
    await tester.pumpAndSettle();

    final Route<Object> oldRoute = ModalRoute.of(tester.element(find.text('Route: Bar')))!;
    expect(oldRoute.settings.name, 'Bar');

    final Matcher throwsArgumentsAssertionError = throwsA(
      isAssertionError.having(
        (AssertionError e) => e.message,
        'message',
        'The arguments object must be serializable via the StandardMessageCodec.',
      ),
    );
    final Matcher throwsBuilderAssertionError = throwsA(
      isAssertionError.having(
        (AssertionError e) => e.message,
        'message',
        'The provided routeBuilder must be a static function.',
      ),
    );

    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePushNamed('Foo', arguments: Object()),
      throwsArgumentsAssertionError,
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePushReplacementNamed('Foo', arguments: Object()),
      throwsArgumentsAssertionError,
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePopAndPushNamed('Foo', arguments: Object()),
      throwsArgumentsAssertionError,
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePushNamedAndRemoveUntil(
            'Foo',
            (Route<Object?> _) => false,
            arguments: Object(),
          ),
      throwsArgumentsAssertionError,
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePush(_routeBuilder, arguments: Object()),
      throwsArgumentsAssertionError,
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePushReplacement(_routeBuilder, arguments: Object()),
      throwsArgumentsAssertionError,
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePushAndRemoveUntil(
            _routeBuilder,
            (Route<Object?> _) => false,
            arguments: Object(),
          ),
      throwsArgumentsAssertionError,
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorableReplace(
            newRouteBuilder: _routeBuilder,
            oldRoute: oldRoute,
            arguments: Object(),
          ),
      throwsArgumentsAssertionError,
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorableReplaceRouteBelow(
            newRouteBuilder: _routeBuilder,
            anchorRoute: oldRoute,
            arguments: Object(),
          ),
      throwsArgumentsAssertionError,
    );

    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePush((BuildContext _, Object? _) => FakeRoute()),
      throwsBuilderAssertionError,
      skip: isBrowser, // https://github.com/flutter/flutter/issues/33615
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePushReplacement((BuildContext _, Object? _) => FakeRoute()),
      throwsBuilderAssertionError,
      skip: isBrowser, // https://github.com/flutter/flutter/issues/33615
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorablePushAndRemoveUntil(
            (BuildContext _, Object? _) => FakeRoute(),
            (Route<Object?> _) => false,
          ),
      throwsBuilderAssertionError,
      skip: isBrowser, // https://github.com/flutter/flutter/issues/33615
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorableReplace(
            newRouteBuilder: (BuildContext _, Object? _) => FakeRoute(),
            oldRoute: oldRoute,
          ),
      throwsBuilderAssertionError,
      skip: isBrowser, // https://github.com/flutter/flutter/issues/33615
    );
    expect(
      () => tester
          .state<NavigatorState>(find.byType(Navigator))
          .restorableReplaceRouteBelow(
            newRouteBuilder: (BuildContext _, Object? _) => FakeRoute(),
            anchorRoute: oldRoute,
          ),
      throwsBuilderAssertionError,
      skip: isBrowser, // https://github.com/flutter/flutter/issues/33615
    );
  });

  testWidgets('Moving scopes', (WidgetTester tester) async {
    await tester.pumpWidget(
      const RootRestorationScope(restorationId: 'root', child: TestWidget(restorationId: null)),
    );
    await tapRouteCounter('home', tester);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo');
    await tester.pumpAndSettle();
    expect(findRoute('Foo'), findsOneWidget);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);

    // Nothing is restored.
    await tester.restartAndRestore();
    expect(findRoute('Foo'), findsNothing);
    expect(findRoute('home', count: 0), findsOneWidget);

    await tapRouteCounter('home', tester);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo');
    await tester.pumpAndSettle();

    // Move navigator into restoration scope.
    await tester.pumpWidget(const RootRestorationScope(restorationId: 'root', child: TestWidget()));

    expect(findRoute('Foo'), findsOneWidget);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);

    // Everything is restored.
    await tester.restartAndRestore();
    expect(findRoute('Foo'), findsOneWidget);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);

    // Move navigator out of restoration scope.
    await tester.pumpWidget(
      const RootRestorationScope(restorationId: 'root', child: TestWidget(restorationId: null)),
    );

    expect(findRoute('Foo'), findsOneWidget);
    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);

    // Nothing is restored.
    await tester.restartAndRestore();
    expect(findRoute('Foo'), findsNothing);
    expect(findRoute('home', count: 0), findsOneWidget);
  });

  testWidgets('Restoring pages', (WidgetTester tester) async {
    await tester.pumpWidget(const PagedTestWidget());
    expect(findRoute('home', count: 0), findsOneWidget);
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo');
    await tester.pumpAndSettle();
    await tapRouteCounter('Foo', tester);
    await tapRouteCounter('Foo', tester);
    expect(findRoute('Foo', count: 2), findsOneWidget);

    final TestRestorationData data = await tester.getRestorationData();
    await tester.restartAndRestore();

    expect(findRoute('Foo', count: 2), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('home', count: 1), findsOneWidget);

    tester.state<PagedTestNavigatorState>(find.byType(PagedTestNavigator)).addPage('bar');
    await tester.pumpAndSettle();
    await tapRouteCounter('bar', tester);
    expect(findRoute('bar', count: 1), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('Foo');
    await tester.pumpAndSettle();
    expect(findRoute('Foo', count: 0), findsOneWidget);

    await tester.restoreFrom(data);

    expect(findRoute('bar', skipOffstage: false), findsNothing);
    expect(findRoute('Foo', count: 2), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('home', count: 1), findsOneWidget);

    tester.state<PagedTestNavigatorState>(find.byType(PagedTestNavigator)).addPage('bar');
    await tester.pumpAndSettle();
    expect(findRoute('bar', count: 0), findsOneWidget);
  });

  testWidgets('Unrestorable pages', (WidgetTester tester) async {
    await tester.pumpWidget(const PagedTestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);
    tester.state<PagedTestNavigatorState>(find.byType(PagedTestNavigator)).addPage('p1');
    await tester.pumpAndSettle();
    await tapRouteCounter('p1', tester);
    expect(findRoute('p1', count: 1), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('r1');
    await tester.pumpAndSettle();
    await tapRouteCounter('r1', tester);
    expect(findRoute('r1', count: 1), findsOneWidget);

    tester
        .state<PagedTestNavigatorState>(find.byType(PagedTestNavigator))
        .addPage('p2', restoreState: false);
    await tester.pumpAndSettle();
    await tapRouteCounter('p2', tester);
    expect(findRoute('p2', count: 1), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('r2');
    await tester.pumpAndSettle();
    await tapRouteCounter('r2', tester);
    expect(findRoute('r2', count: 1), findsOneWidget);

    tester.state<PagedTestNavigatorState>(find.byType(PagedTestNavigator)).addPage('p3');
    await tester.pumpAndSettle();
    await tapRouteCounter('p3', tester);
    expect(findRoute('p3', count: 1), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('r3');
    await tester.pumpAndSettle();
    await tapRouteCounter('r3', tester);
    expect(findRoute('r3', count: 1), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('r2', skipOffstage: false), findsNothing);
    expect(findRoute('r3', count: 1), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('p3', count: 1), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('p2', count: 0), findsOneWidget); // Page did not restore its state!
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('r1', count: 1), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('p1', count: 1), findsOneWidget);
    tester.state<NavigatorState>(find.byType(Navigator)).pop();
    await tester.pumpAndSettle();
    expect(findRoute('home', count: 1), findsOneWidget);
  });

  testWidgets('removed page is not restored', (WidgetTester tester) async {
    await tester.pumpWidget(const PagedTestWidget());
    await tapRouteCounter('home', tester);
    expect(findRoute('home', count: 1), findsOneWidget);

    tester.state<PagedTestNavigatorState>(find.byType(PagedTestNavigator)).addPage('p1');
    await tester.pumpAndSettle();
    await tapRouteCounter('p1', tester);
    expect(findRoute('p1', count: 1), findsOneWidget);

    tester.state<NavigatorState>(find.byType(Navigator)).restorablePushNamed('r1');
    await tester.pumpAndSettle();
    await tapRouteCounter('r1', tester);
    expect(findRoute('r1', count: 1), findsOneWidget);

    tester.state<PagedTestNavigatorState>(find.byType(PagedTestNavigator)).addPage('p2');
    await tester.pumpAndSettle();
    await tapRouteCounter('p2', tester);
    expect(findRoute('p2', count: 1), findsOneWidget);

    tester.state<PagedTestNavigatorState>(find.byType(PagedTestNavigator)).removePage('p1');
    await tester.pumpAndSettle();

    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('p1', count: 1, skipOffstage: false), findsNothing);
    expect(findRoute('r1', count: 1, skipOffstage: false), findsNothing);
    expect(findRoute('p2', count: 1), findsOneWidget);

    await tester.restartAndRestore();

    expect(findRoute('home', count: 1, skipOffstage: false), findsOneWidget);
    expect(findRoute('p1', count: 1, skipOffstage: false), findsNothing);
    expect(findRoute('r1', count: 1, skipOffstage: false), findsNothing);
    expect(findRoute('p2', count: 1), findsOneWidget);

    tester.state<PagedTestNavigatorState>(find.byType(PagedTestNavigator)).addPage('p1');
    await tester.pumpAndSettle();
    expect(findRoute('p1', count: 0), findsOneWidget);
  });

  testWidgets(
    'Helpful assert thrown all routes in onGenerateInitialRoutes are not restorable',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          restorationScopeId: 'material_app',
          initialRoute: '/',
          routes: <String, WidgetBuilder>{'/': (BuildContext context) => Container()},
          onGenerateInitialRoutes: (String initialRoute) {
            return <MaterialPageRoute<void>>[
              MaterialPageRoute<void>(builder: (BuildContext context) => Container()),
            ];
          },
        ),
      );
      await tester.restartAndRestore();
      final dynamic exception = tester.takeException();
      expect(exception, isAssertionError);
      expect(
        (exception as AssertionError).message,
        contains('All routes returned by onGenerateInitialRoutes are not restorable.'),
      );

      // The previous assert leaves the widget tree in a broken state, so the
      // following code catches any remaining exceptions from attempting to build
      // new widget tree.
      final FlutterExceptionHandler? oldHandler = FlutterError.onError;
      dynamic remainingException;
      FlutterError.onError = (FlutterErrorDetails details) {
        remainingException ??= details.exception;
      };
      await tester.pumpWidget(Container(key: UniqueKey()));
      FlutterError.onError = oldHandler;
      expect(remainingException, isAssertionError);
    },
  );
}

@pragma('vm:entry-point')
Route<void> _routeBuilder(BuildContext context, Object? arguments) {
  return MaterialPageRoute<void>(
    builder: (BuildContext context) {
      return RouteWidget(name: arguments! as String);
    },
  );
}

@pragma('vm:entry-point')
Route<void> _routeFutureBuilder(BuildContext context, Object? arguments) {
  return MaterialPageRoute<void>(
    builder: (BuildContext context) {
      return const RouteFutureWidget();
    },
  );
}

class PagedTestWidget extends StatelessWidget {
  const PagedTestWidget({super.key, this.restorationId = 'app'});

  final String restorationId;

  @override
  Widget build(BuildContext context) {
    return RootRestorationScope(
      restorationId: restorationId,
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: MediaQueryData.fromView(View.of(context)),
          child: const PagedTestNavigator(),
        ),
      ),
    );
  }
}

class PagedTestNavigator extends StatefulWidget {
  const PagedTestNavigator({super.key});

  @override
  State<PagedTestNavigator> createState() => PagedTestNavigatorState();
}

class PagedTestNavigatorState extends State<PagedTestNavigator> with RestorationMixin {
  final RestorableString _routes = RestorableString('r-home');

  void addPage(String name, {bool restoreState = true, int? index}) {
    assert(!name.contains(','));
    assert(!name.startsWith('r-'));
    final List<String> routes = _routes.value.split(',');
    name = restoreState ? 'r-$name' : name;
    if (index != null) {
      routes.insert(index, name);
    } else {
      routes.add(name);
    }
    setState(() {
      _routes.value = routes.join(',');
    });
  }

  bool removePage(String name) {
    final List<String> routes = _routes.value.split(',');
    if (routes.remove(name) || routes.remove('r-$name')) {
      setState(() {
        _routes.value = routes.join(',');
      });
      return true;
    }
    return false;
  }

  @override
  Widget build(BuildContext context) {
    return Navigator(
      restorationScopeId: 'nav',
      onPopPage: (Route<dynamic> route, dynamic result) {
        if (route.didPop(result)) {
          removePage(route.settings.name!);
          return true;
        }
        return false;
      },
      pages:
          _routes.value.isEmpty
              ? const <Page<Object?>>[]
              : _routes.value.split(',').map((String name) {
                if (name.startsWith('r-')) {
                  name = name.substring(2);
                  return TestPage(name: name, restorationId: name, key: ValueKey<String>(name));
                }
                return TestPage(name: name, key: ValueKey<String>(name));
              }).toList(),
      onGenerateRoute: (RouteSettings settings) {
        return MaterialPageRoute<int>(
          settings: settings,
          builder: (BuildContext context) {
            return RouteWidget(name: settings.name!, arguments: settings.arguments);
          },
        );
      },
    );
  }

  @override
  String get restorationId => 'router';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_routes, 'routes');
  }

  @override
  void dispose() {
    super.dispose();
    _routes.dispose();
  }
}

class TestPage extends Page<void> {
  const TestPage({super.key, required String super.name, super.restorationId});

  @override
  Route<void> createRoute(BuildContext context) {
    return MaterialPageRoute<void>(
      settings: this,
      builder: (BuildContext context) {
        return RouteWidget(name: name!);
      },
    );
  }
}

class TestWidget extends StatelessWidget {
  const TestWidget({super.key, this.restorationId = 'app'});

  final String? restorationId;

  @override
  Widget build(BuildContext context) {
    return RootRestorationScope(
      restorationId: restorationId,
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: MediaQueryData.fromView(View.of(context)),
          child: Navigator(
            initialRoute: 'home',
            restorationScopeId: 'app',
            onGenerateRoute: (RouteSettings settings) {
              return MaterialPageRoute<int>(
                settings: settings,
                builder: (BuildContext context) {
                  return RouteWidget(name: settings.name!, arguments: settings.arguments);
                },
              );
            },
          ),
        ),
      ),
    );
  }
}

class RouteWidget extends StatefulWidget {
  const RouteWidget({super.key, required this.name, this.arguments});

  final String name;
  final Object? arguments;

  @override
  State<RouteWidget> createState() => RouteWidgetState();
}

class RouteWidgetState extends State<RouteWidget> with RestorationMixin {
  final RestorableInt counter = RestorableInt(0);

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(counter, 'counter');
  }

  @override
  String get restorationId => 'stateful';

  @override
  void dispose() {
    super.dispose();
    counter.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        children: <Widget>[
          GestureDetector(
            child: Text('Route: ${widget.name}'),
            onTap: () {
              setState(() {
                counter.value++;
              });
            },
          ),
          if (widget.arguments != null) Text('Arguments(home): ${widget.arguments}'),
          Text('Counter(${widget.name}): ${counter.value}'),
        ],
      ),
    );
  }
}

class RouteFutureWidget extends StatefulWidget {
  const RouteFutureWidget({super.key});

  @override
  State<RouteFutureWidget> createState() => RouteFutureWidgetState();
}

class RouteFutureWidgetState extends State<RouteFutureWidget> with RestorationMixin {
  late RestorableRouteFuture<int> routeFuture;
  int? value;

  @override
  void initState() {
    super.initState();
    routeFuture = RestorableRouteFuture<int>(
      onPresent: (NavigatorState navigatorState, Object? arguments) {
        return navigatorState.restorablePushNamed(arguments! as String);
      },
      onComplete: (int i) {
        setState(() {
          value = i;
        });
      },
    );
  }

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(routeFuture, 'routeFuture');
  }

  @override
  String get restorationId => 'routefuturewidget';

  @override
  void dispose() {
    super.dispose();
    routeFuture.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(child: Text('Return value: $value'));
  }
}

Finder findRoute(String name, {Object? arguments, int? count, bool skipOffstage = true}) =>
    _RouteFinder(name, arguments: arguments, count: count, skipOffstage: skipOffstage);

Future<void> tapRouteCounter(String name, WidgetTester tester) async {
  await tester.tap(find.text('Route: $name'));
  await tester.pump();
}

class _RouteFinder extends MatchFinder {
  _RouteFinder(this.name, {this.arguments, this.count, super.skipOffstage});

  final String name;
  final Object? arguments;
  final int? count;

  @override
  String get description {
    String result = 'Route(name: $name';
    if (arguments != null) {
      result += ', arguments: $arguments';
    }
    if (count != null) {
      result += ', count: $count';
    }
    return result;
  }

  @override
  bool matches(Element candidate) {
    final Widget widget = candidate.widget;
    if (widget is RouteWidget) {
      if (widget.name != name) {
        return false;
      }
      if (arguments != null && widget.arguments != arguments) {
        return false;
      }
      final RouteWidgetState state = (candidate as StatefulElement).state as RouteWidgetState;
      if (count != null && state.counter.value != count) {
        return false;
      }
      return true;
    }
    return false;
  }
}

class FakeRoute extends Fake implements Route<void> {}
