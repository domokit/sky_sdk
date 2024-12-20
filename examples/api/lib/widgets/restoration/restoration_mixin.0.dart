// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [RestorationMixin].

void main() => runApp(const RestorationExampleApp());

class RestorationExampleApp extends StatelessWidget {
  const RestorationExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      restorationScopeId: 'app',
      title: 'Restorable Counter',
      home: RestorableCounter(restorationId: 'counter'),
    );
  }
}

class RestorableCounter extends StatefulWidget {
  const RestorableCounter({super.key, this.restorationId});

  final String? restorationId;

  @override
  State<RestorableCounter> createState() => _RestorableCounterState();
}

// The [State] object uses the [RestorationMixin] to make the current value
// of the counter restorable.
class _RestorableCounterState extends State<RestorableCounter> with RestorationMixin {
  // The current value of the counter is stored in a [RestorableProperty].
  // During state restoration it is automatically restored to its old value.
  // If no restoration data is available to restore the counter from, it is
  // initialized to the specified default value of zero.
  final RestorableInt _counter = RestorableInt(0);

  // In this example, the restoration ID for the mixin is passed in through
  // the [StatefulWidget]'s constructor.
  @override
  String? get restorationId => widget.restorationId;

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    // All restorable properties must be registered with the mixin. After
    // registration, the counter either has its old value restored or is
    // initialized to its default value.
    registerForRestoration(_counter, 'count');
  }

  void _incrementCounter() {
    setState(() {
      // The current value of the property can be accessed and modified via
      // the value getter and setter.
      _counter.value++;
    });
  }

  @override
  void dispose() {
    _counter.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Restorable Counter')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('You have pushed the button this many times:'),
            Text('${_counter.value}', style: Theme.of(context).textTheme.headlineMedium),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _incrementCounter,
        tooltip: 'Increment',
        child: const Icon(Icons.add),
      ),
    );
  }
}
