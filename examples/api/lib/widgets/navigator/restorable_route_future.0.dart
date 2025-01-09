// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [RestorableRouteFuture].

void main() => runApp(const RestorableRouteFutureExampleApp());

class RestorableRouteFutureExampleApp extends StatelessWidget {
  const RestorableRouteFutureExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      restorationScopeId: 'app',
      home: Scaffold(
        appBar: AppBar(title: const Text('RestorableRouteFuture Example')),
        body: const MyHome(),
      ),
    );
  }
}

class MyHome extends StatefulWidget {
  const MyHome({super.key});

  @override
  State<MyHome> createState() => _MyHomeState();
}

@pragma('vm:entry-point')
class _MyHomeState extends State<MyHome> with RestorationMixin {
  final RestorableInt _lastCount = RestorableInt(0);
  late RestorableRouteFuture<int> _counterRoute;

  @override
  String get restorationId => 'home';

  @override
  void initState() {
    super.initState();
    _counterRoute = RestorableRouteFuture<int>(
      onPresent: (NavigatorState navigator, Object? arguments) {
        // Defines what route should be shown (and how it should be added
        // to the navigator) when `RestorableRouteFuture.present` is called.
        return navigator.restorablePush(_counterRouteBuilder, arguments: arguments);
      },
      onComplete: (int count) {
        // Defines what should happen with the return value when the route
        // completes.
        setState(() {
          _lastCount.value = count;
        });
      },
    );
  }

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    // Register the `RestorableRouteFuture` with the state restoration framework.
    registerForRestoration(_counterRoute, 'route');
    registerForRestoration(_lastCount, 'count');
  }

  @override
  void dispose() {
    super.dispose();
    _lastCount.dispose();
    _counterRoute.dispose();
  }

  // A static `RestorableRouteBuilder` that can re-create the route during
  // state restoration.
  @pragma('vm:entry-point')
  static Route<int> _counterRouteBuilder(BuildContext context, Object? arguments) {
    return MaterialPageRoute<int>(
      builder: (BuildContext context) => MyCounter(title: arguments!.toString()),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text('Last count: ${_lastCount.value}'),
          ElevatedButton(
            onPressed: () {
              // Show the route defined by the `RestorableRouteFuture`.
              _counterRoute.present('Awesome Counter');
            },
            child: const Text('Open Counter'),
          ),
        ],
      ),
    );
  }
}

// Widget for the route pushed by the `RestorableRouteFuture`.
class MyCounter extends StatefulWidget {
  const MyCounter({super.key, required this.title});

  final String title;

  @override
  State<MyCounter> createState() => _MyCounterState();
}

class _MyCounterState extends State<MyCounter> with RestorationMixin {
  final RestorableInt _count = RestorableInt(0);

  @override
  String get restorationId => 'counter';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_count, 'count');
  }

  @override
  void dispose() {
    super.dispose();
    _count.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
        leading: BackButton(
          onPressed: () {
            // Return the current count of the counter from this route.
            Navigator.of(context).pop(_count.value);
          },
        ),
      ),
      body: Center(child: Text('Count: ${_count.value}')),
      floatingActionButton: FloatingActionButton(
        child: const Icon(Icons.add),
        onPressed: () {
          setState(() {
            _count.value++;
          });
        },
      ),
    );
  }
}
