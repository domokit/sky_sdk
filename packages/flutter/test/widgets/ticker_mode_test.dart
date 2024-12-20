// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Nested TickerMode cannot turn tickers back on', (WidgetTester tester) async {
    int outerTickCount = 0;
    int innerTickCount = 0;

    Widget nestedTickerModes({required bool innerEnabled, required bool outerEnabled}) {
      return Directionality(
        textDirection: TextDirection.rtl,
        child: TickerMode(
          enabled: outerEnabled,
          child: Row(
            children: <Widget>[
              _TickingWidget(
                onTick: () {
                  outerTickCount++;
                },
              ),
              TickerMode(
                enabled: innerEnabled,
                child: _TickingWidget(
                  onTick: () {
                    innerTickCount++;
                  },
                ),
              ),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(nestedTickerModes(outerEnabled: false, innerEnabled: true));

    expect(outerTickCount, 0);
    expect(innerTickCount, 0);
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(outerTickCount, 0);
    expect(innerTickCount, 0);

    await tester.pumpWidget(nestedTickerModes(outerEnabled: true, innerEnabled: false));
    outerTickCount = 0;
    innerTickCount = 0;
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(outerTickCount, 4);
    expect(innerTickCount, 0);

    await tester.pumpWidget(nestedTickerModes(outerEnabled: true, innerEnabled: true));
    outerTickCount = 0;
    innerTickCount = 0;
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(outerTickCount, 4);
    expect(innerTickCount, 4);

    await tester.pumpWidget(nestedTickerModes(outerEnabled: false, innerEnabled: false));
    outerTickCount = 0;
    innerTickCount = 0;
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    await tester.pump(const Duration(seconds: 1));
    expect(outerTickCount, 0);
    expect(innerTickCount, 0);
  });

  testWidgets('Changing TickerMode does not rebuild widgets with SingleTickerProviderStateMixin', (
    WidgetTester tester,
  ) async {
    Widget widgetUnderTest({required bool tickerEnabled}) {
      return TickerMode(enabled: tickerEnabled, child: const _TickingWidget());
    }

    _TickingWidgetState state() => tester.state<_TickingWidgetState>(find.byType(_TickingWidget));

    await tester.pumpWidget(widgetUnderTest(tickerEnabled: true));
    expect(state().ticker.isTicking, isTrue);
    expect(state().buildCount, 1);

    await tester.pumpWidget(widgetUnderTest(tickerEnabled: false));
    expect(state().ticker.isTicking, isFalse);
    expect(state().buildCount, 1);

    await tester.pumpWidget(widgetUnderTest(tickerEnabled: true));
    expect(state().ticker.isTicking, isTrue);
    expect(state().buildCount, 1);
  });

  testWidgets('Changing TickerMode does not rebuild widgets with TickerProviderStateMixin', (
    WidgetTester tester,
  ) async {
    Widget widgetUnderTest({required bool tickerEnabled}) {
      return TickerMode(enabled: tickerEnabled, child: const _MultiTickingWidget());
    }

    _MultiTickingWidgetState state() =>
        tester.state<_MultiTickingWidgetState>(find.byType(_MultiTickingWidget));

    await tester.pumpWidget(widgetUnderTest(tickerEnabled: true));
    expect(state().ticker.isTicking, isTrue);
    expect(state().buildCount, 1);

    await tester.pumpWidget(widgetUnderTest(tickerEnabled: false));
    expect(state().ticker.isTicking, isFalse);
    expect(state().buildCount, 1);

    await tester.pumpWidget(widgetUnderTest(tickerEnabled: true));
    expect(state().ticker.isTicking, isTrue);
    expect(state().buildCount, 1);
  });

  testWidgets(
    'Moving widgets with SingleTickerProviderStateMixin to a new TickerMode ancestor works',
    (WidgetTester tester) async {
      final GlobalKey tickingWidgetKey = GlobalKey();
      Widget widgetUnderTest({required LocalKey tickerModeKey, required bool tickerEnabled}) {
        return TickerMode(
          key: tickerModeKey,
          enabled: tickerEnabled,
          child: _TickingWidget(key: tickingWidgetKey),
        );
      }

      // Using different local keys to simulate changing TickerMode ancestors.
      await tester.pumpWidget(widgetUnderTest(tickerEnabled: true, tickerModeKey: UniqueKey()));
      final State tickerModeState = tester.state(find.byType(TickerMode));
      final _TickingWidgetState tickingState = tester.state<_TickingWidgetState>(
        find.byType(_TickingWidget),
      );
      expect(tickingState.ticker.isTicking, isTrue);

      await tester.pumpWidget(widgetUnderTest(tickerEnabled: false, tickerModeKey: UniqueKey()));
      expect(tester.state(find.byType(TickerMode)), isNot(same(tickerModeState)));
      expect(tickingState, same(tester.state<_TickingWidgetState>(find.byType(_TickingWidget))));
      expect(tickingState.ticker.isTicking, isFalse);
    },
  );

  testWidgets('Moving widgets with TickerProviderStateMixin to a new TickerMode ancestor works', (
    WidgetTester tester,
  ) async {
    final GlobalKey tickingWidgetKey = GlobalKey();
    Widget widgetUnderTest({required LocalKey tickerModeKey, required bool tickerEnabled}) {
      return TickerMode(
        key: tickerModeKey,
        enabled: tickerEnabled,
        child: _MultiTickingWidget(key: tickingWidgetKey),
      );
    }

    // Using different local keys to simulate changing TickerMode ancestors.
    await tester.pumpWidget(widgetUnderTest(tickerEnabled: true, tickerModeKey: UniqueKey()));
    final State tickerModeState = tester.state(find.byType(TickerMode));
    final _MultiTickingWidgetState tickingState = tester.state<_MultiTickingWidgetState>(
      find.byType(_MultiTickingWidget),
    );
    expect(tickingState.ticker.isTicking, isTrue);

    await tester.pumpWidget(widgetUnderTest(tickerEnabled: false, tickerModeKey: UniqueKey()));
    expect(tester.state(find.byType(TickerMode)), isNot(same(tickerModeState)));
    expect(
      tickingState,
      same(tester.state<_MultiTickingWidgetState>(find.byType(_MultiTickingWidget))),
    );
    expect(tickingState.ticker.isTicking, isFalse);
  });

  testWidgets('Ticking widgets in old route do not rebuild when new route is pushed', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        routes: <String, WidgetBuilder>{'/foo': (BuildContext context) => const Text('New route')},
        home: const Row(
          children: <Widget>[_TickingWidget(), _MultiTickingWidget(), Text('Old route')],
        ),
      ),
    );

    _MultiTickingWidgetState multiTickingState() => tester.state<_MultiTickingWidgetState>(
      find.byType(_MultiTickingWidget, skipOffstage: false),
    );
    _TickingWidgetState tickingState() =>
        tester.state<_TickingWidgetState>(find.byType(_TickingWidget, skipOffstage: false));

    expect(find.text('Old route'), findsOneWidget);
    expect(find.text('New route'), findsNothing);

    expect(multiTickingState().ticker.isTicking, isTrue);
    expect(multiTickingState().buildCount, 1);
    expect(tickingState().ticker.isTicking, isTrue);
    expect(tickingState().buildCount, 1);

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/foo');
    await tester.pumpAndSettle();
    expect(find.text('Old route'), findsNothing);
    expect(find.text('New route'), findsOneWidget);

    expect(multiTickingState().ticker.isTicking, isFalse);
    expect(multiTickingState().buildCount, 1);
    expect(tickingState().ticker.isTicking, isFalse);
    expect(tickingState().buildCount, 1);
  });
}

class _TickingWidget extends StatefulWidget {
  const _TickingWidget({super.key, this.onTick});

  final VoidCallback? onTick;

  @override
  State<_TickingWidget> createState() => _TickingWidgetState();
}

class _TickingWidgetState extends State<_TickingWidget> with SingleTickerProviderStateMixin {
  late Ticker ticker;
  int buildCount = 0;

  @override
  void initState() {
    super.initState();
    ticker = createTicker((Duration _) {
      widget.onTick?.call();
    })..start();
  }

  @override
  Widget build(BuildContext context) {
    buildCount += 1;
    return Container();
  }

  @override
  void dispose() {
    ticker.dispose();
    super.dispose();
  }
}

class _MultiTickingWidget extends StatefulWidget {
  const _MultiTickingWidget({super.key});

  @override
  State<_MultiTickingWidget> createState() => _MultiTickingWidgetState();
}

class _MultiTickingWidgetState extends State<_MultiTickingWidget> with TickerProviderStateMixin {
  late Ticker ticker;
  int buildCount = 0;

  @override
  void initState() {
    super.initState();
    ticker = createTicker((Duration _) {})..start();
  }

  @override
  Widget build(BuildContext context) {
    buildCount += 1;
    return Container();
  }

  @override
  void dispose() {
    ticker.dispose();
    super.dispose();
  }
}
