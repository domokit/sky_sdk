// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';
import 'package:macrobenchmarks/common.dart';
import 'package:macrobenchmarks/main.dart' as app;

typedef ControlCallback = Future<void> Function(WidgetController controller);

class ScrollableButtonRoute {
  ScrollableButtonRoute(this.listViewKey, this.buttonKey);

  final String listViewKey;
  final String buttonKey;
}

void macroPerfTestE2E(
  String testName,
  String routeName, {
  Duration? pageDelay,
  Duration duration = const Duration(seconds: 3),
  ControlCallback? body,
  ControlCallback? setup,
}) {
  macroPerfTestMultiPageE2E(
    testName,
    <ScrollableButtonRoute>[ScrollableButtonRoute(kScrollableName, routeName)],
    pageDelay: pageDelay,
    duration: duration,
    body: body,
    setup: setup,
  );
}

void macroPerfTestMultiPageE2E(
  String testName,
  List<ScrollableButtonRoute> routes, {
  Duration? pageDelay,
  Duration duration = const Duration(seconds: 3),
  ControlCallback? body,
  ControlCallback? setup,
}) {
  final WidgetsBinding widgetsBinding = IntegrationTestWidgetsFlutterBinding.ensureInitialized();
  assert(widgetsBinding is IntegrationTestWidgetsFlutterBinding);
  final IntegrationTestWidgetsFlutterBinding binding =
      widgetsBinding as IntegrationTestWidgetsFlutterBinding;
  binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive;

  testWidgets(
    testName,
    (WidgetTester tester) async {
      assert(tester.binding == binding);
      app.main();
      await tester.pumpAndSettle();

      // The slight initial delay avoids starting the timing during a
      // period of increased load on the device. Without this delay, the
      // benchmark has greater noise.
      // See: https://github.com/flutter/flutter/issues/19434
      await tester.binding.delayed(const Duration(microseconds: 250));

      for (final ScrollableButtonRoute route in routes) {
        expect(route.listViewKey, startsWith('/'));
        expect(route.buttonKey, startsWith('/'));

        // Make sure each list view page is settled
        await tester.pumpAndSettle();

        final Finder listView = find.byKey(ValueKey<String>(route.listViewKey));
        // ListView is not a Scrollable, but it contains one
        final Finder scrollable = find.descendant(of: listView, matching: find.byType(Scrollable));
        // scrollable should find one widget as soon as the page is loaded
        expect(scrollable, findsOneWidget);

        final Finder button = find.byKey(ValueKey<String>(route.buttonKey), skipOffstage: false);
        // button may or may not find a widget right away until we scroll to it
        await tester.scrollUntilVisible(button, 50, scrollable: scrollable);
        // After scrolling, button should find one Widget
        expect(button, findsOneWidget);

        // Allow scrolling to settle
        await tester.pumpAndSettle();
        await tester.tap(button);
        // Cannot be pumpAndSettle because some tests have infinite animation.
        await tester.pump(const Duration(milliseconds: 20));
      }

      if (pageDelay != null) {
        // Wait for the page to load
        await tester.binding.delayed(pageDelay);
      }

      if (setup != null) {
        await setup(tester);
      }

      await binding.watchPerformance(() async {
        final Future<void> durationFuture = tester.binding.delayed(duration);
        if (body != null) {
          await body(tester);
        }
        await durationFuture;
      });
    },
    semanticsEnabled: false,
    timeout: Timeout.none,
  );
}
