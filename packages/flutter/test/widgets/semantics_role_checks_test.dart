// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('All semantics roles has checks', () {
    expect(SemanticsRole.values.toSet(), DebugSemanticsRoleChecks.kChecks.keys.toSet());
  });

  group('tab', () {
    testWidgets('failure case, empty', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Semantics(role: SemanticsRole.tab, child: const Text('a tab')),
        ),
      );
      final Object? exception = tester.takeException();
      expect(exception, isFlutterError);
      final FlutterError error = exception! as FlutterError;
      expect(error.message, 'A tab needs selected states');
    });

    testWidgets('failure case, no tap', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Semantics(role: SemanticsRole.tab, selected: false, child: const Text('a tab')),
        ),
      );
      final Object? exception = tester.takeException();
      expect(exception, isFlutterError);
      final FlutterError error = exception! as FlutterError;
      expect(error.message, 'A tab must have a tap action');
    });

    testWidgets('success case', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Semantics(
            role: SemanticsRole.tab,
            selected: false,
            onTap: () {},
            child: const Text('a tab'),
          ),
        ),
      );
      expect(tester.takeException(), isNull);
    });
  });

  group('tabBar', () {
    testWidgets('failure case, empty child', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Semantics(
            role: SemanticsRole.tabBar,
            child: const ExcludeSemantics(child: Text('something')),
          ),
        ),
      );
      final Object? exception = tester.takeException();
      expect(exception, isFlutterError);
      final FlutterError error = exception! as FlutterError;
      expect(error.message, 'a TabBar cannot be empty');
    });

    testWidgets('failure case, non tab child', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Semantics(
            role: SemanticsRole.tabBar,
            explicitChildNodes: true,
            child: Semantics(child: const Text('some child')),
          ),
        ),
      );
      final Object? exception = tester.takeException();
      expect(exception, isFlutterError);
      final FlutterError error = exception! as FlutterError;
      expect(error.message, 'Children of TabBar must have the tab role');
    });

    testWidgets('Success case', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Semantics(
            role: SemanticsRole.tabBar,
            explicitChildNodes: true,
            child: Semantics(
              role: SemanticsRole.tab,
              selected: false,
              onTap: () {},
              child: const Text('some child'),
            ),
          ),
        ),
      );
      expect(tester.takeException(), isNull);
    });
  });
}
