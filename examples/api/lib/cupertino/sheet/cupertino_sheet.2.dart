// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoSheetRoute] with restorable state and nested navigation.

void main() => runApp(const RestorableSheetExampleApp());

class RestorableSheetExampleApp extends StatelessWidget {
  const RestorableSheetExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      restorationScopeId: 'sheet-app',
      title: 'Restorable Sheet',
      home: RestorableSheet(restorationId: 'sheet'),
    );
  }
}

class RestorableSheet extends StatefulWidget {
  const RestorableSheet({super.key, this.restorationId});

  final String? restorationId;

  @override
  State<RestorableSheet> createState() => _RestorableSheetState();
}

@pragma('vm:entry-point')
class _RestorableSheetState extends State<RestorableSheet> with RestorationMixin {
  final RestorableInt _counter = RestorableInt(0);
  late RestorableRouteFuture<int?> _restorableSheetRouteFuture;

  @override
  void initState() {
    super.initState();
    _restorableSheetRouteFuture = RestorableRouteFuture<int?>(
      onComplete: _changeCounter,
      onPresent: (NavigatorState navigator, Object? arguments) {
        return navigator.restorablePush(_counterSheetBuilder, arguments: _counter.value);
      },
    );
  }

  @override
  String? get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_counter, 'count');
    registerForRestoration(_restorableSheetRouteFuture, 'sheet_route_future');
  }

  @override
  void dispose() {
    _counter.dispose();
    super.dispose();
  }

  @pragma('vm:entry-point')
  static Route<void> _counterSheetBuilder(BuildContext context, Object? arguments) {
    return CupertinoSheetRoute<int?>(
      builder: (BuildContext context) {
        return Navigator(
          restorationScopeId: 'nested-nav',
          onGenerateRoute: (RouteSettings settings) {
            return CupertinoPageRoute<void>(
              settings: settings,
              builder: (BuildContext context) {
                return PopScope(
                  canPop: settings.name != '/',
                  onPopInvokedWithResult: (bool didPop, Object? result) {
                    if (didPop) {
                      return;
                    }
                    Navigator.of(context).pop();
                  },
                  child: CounterSheetScaffold(counter: arguments! as int),
                );
              },
            );
          },
        );
      },
    );
  }

  void _changeCounter(int? newCounter) {
    if (newCounter != null) {
      setState(() {
        _counter.value = newCounter;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(
        middle: Text('Sheet Example'),
        automaticBackgroundVisibility: false,
      ),
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Counter current value:'),
            Text('${_counter.value}'),
            CupertinoButton(
              child: const Text('Open Sheet'),
              onPressed: () {
                _restorableSheetRouteFuture.present();
              },
            ),
          ],
        ),
      ),
    );
  }
}

class CounterSheetScaffold extends StatefulWidget {
  const CounterSheetScaffold({super.key, required this.counter});

  final int counter;

  @override
  State<CounterSheetScaffold> createState() => _CounterSheetScaffoldState();
}

class _CounterSheetScaffoldState extends State<CounterSheetScaffold> with RestorationMixin {
  late RestorableInt _counter;
  late RestorableRouteFuture<int?> _multiplicationRouteFuture;

  @override
  void initState() {
    super.initState();
    _counter = RestorableInt(widget.counter);
    _multiplicationRouteFuture = RestorableRouteFuture<int?>(
      onComplete: _changeCounter,
      onPresent: (NavigatorState navigator, Object? arguments) {
        return navigator.restorablePush(_multiplicationRouteBuilder, arguments: _counter.value);
      },
    );
  }

  @pragma('vm:entry-point')
  static Route<void> _multiplicationRouteBuilder(BuildContext context, Object? arguments) {
    return CupertinoPageRoute<int?>(
      settings: const RouteSettings(name: '/multiplication'),
      builder: (BuildContext context) {
        return MultiplicationPage(counter: arguments! as int);
      },
    );
  }

  void _changeCounter(int? newCounter) {
    if (newCounter != null) {
      setState(() {
        _counter.value = newCounter;
      });
    }
  }

  @override
  String? get restorationId => 'sheet_scaffold';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_counter, 'sheet_counter');
    registerForRestoration(_multiplicationRouteFuture, 'multiplication_route');
    if (!_counter.enabled) {
      _counter = RestorableInt(widget.counter);
    }
  }

  @override
  void dispose() {
    _counter.dispose();
    _multiplicationRouteFuture.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text('Current Count: ${_counter.value}'),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                CupertinoButton(
                  onPressed: () {
                    setState(() => _counter.value = _counter.value - 1);
                  },
                  child: const Text('Decrease'),
                ),
                CupertinoButton(
                  onPressed: () {
                    setState(() => _counter.value = _counter.value + 1);
                  },
                  child: const Text('Increase'),
                ),
              ],
            ),
            CupertinoButton(
              onPressed: () => _multiplicationRouteFuture.present(),
              child: const Text('Go to Multiplication Page'),
            ),
            CupertinoButton(
              onPressed: () => Navigator.of(context, rootNavigator: true).pop(_counter.value),
              child: const Text('Pop Sheet'),
            ),
          ],
        ),
      ),
    );
  }
}

class MultiplicationPage extends StatefulWidget {
  const MultiplicationPage({super.key, required this.counter});

  final int counter;

  @override
  State<MultiplicationPage> createState() => _MultiplicationPageState();
}

class _MultiplicationPageState extends State<MultiplicationPage> with RestorationMixin {
  late final RestorableInt _counter = RestorableInt(widget.counter);

  @override
  String? get restorationId => 'multiplication_page';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_counter, 'multi_counter');
  }

  @override
  void dispose() {
    _counter.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Current Count'),
            Text(_counter.value.toString()),
            CupertinoButton(
              onPressed: () {
                setState(() => _counter.value = _counter.value * 2);
              },
              child: const Text('Double it'),
            ),
            CupertinoButton(
              onPressed: () => Navigator.pop(context, _counter.value),
              child: const Text('Pass it on to the last sheet'),
            ),
          ],
        ),
      ),
    );
  }
}
