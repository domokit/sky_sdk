// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ValueListenableBuilder].

void main() => runApp(const ValueListenableBuilderExampleApp());

class ValueListenableBuilderExampleApp extends StatelessWidget {
  const ValueListenableBuilderExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: ValueListenableBuilderExample());
  }
}

class ValueListenableBuilderExample extends StatefulWidget {
  const ValueListenableBuilderExample({super.key});

  @override
  State<ValueListenableBuilderExample> createState() => _ValueListenableBuilderExampleState();
}

class _ValueListenableBuilderExampleState extends State<ValueListenableBuilderExample> {
  final ValueNotifier<int> _counter = ValueNotifier<int>(0);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('ValueListenableBuilder Sample')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('You have pushed the button this many times:'),
            ValueListenableBuilder<int>(
              builder: (BuildContext context, int value, Widget? child) {
                // This builder will only get called when the _counter
                // is updated.
                return Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[CountDisplay(count: value), child!],
                );
              },
              valueListenable: _counter,
              // The child parameter is most helpful if the child is
              // expensive to build and does not depend on the value from
              // the notifier.
              child: const Padding(
                padding: EdgeInsets.all(10.0),
                child: SizedBox(width: 40, height: 40, child: FlutterLogo(size: 40)),
              ),
            ),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        child: const Icon(Icons.plus_one),
        onPressed: () => _counter.value += 1,
      ),
    );
  }
}

class CountDisplay extends StatelessWidget {
  const CountDisplay({super.key, required this.count});

  final int count;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: 100,
      height: 100,
      padding: const EdgeInsetsDirectional.all(10),
      child: Text('$count', style: Theme.of(context).textTheme.headlineMedium),
    );
  }
}
