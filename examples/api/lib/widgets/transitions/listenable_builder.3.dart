// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'package:flutter/material.dart';

/// Flutter code sample for a [ChangeNotifier] with a [ListenableBuilder].

void main() {
  runApp(const ListenableBuilderExample());
}

class ListModel with ChangeNotifier {
  final List<int> _values = <int>[];
  List<int> get values => _values.toList(); // O(N), makes a new copy each time.

  void add(int value) {
    _values.add(value);
    notifyListeners();
  }
}

class ListenableBuilderExample extends StatefulWidget {
  const ListenableBuilderExample({super.key});

  @override
  State<ListenableBuilderExample> createState() => _ListenableBuilderExampleState();
}

class _ListenableBuilderExampleState extends State<ListenableBuilderExample> {
  final ListModel _listNotifier = ListModel();
  final math.Random _random = math.Random(0); // fixed seed for reproducibility

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ListenableBuilder Example')),
        body: ListBody(listNotifier: _listNotifier),
        floatingActionButton: FloatingActionButton(
          onPressed:
              () => _listNotifier.add(
                _random.nextInt(1 << 31),
              ), // 1 << 31 is the maximum supported value
          child: const Icon(Icons.add),
        ),
      ),
    );
  }
}

class ListBody extends StatelessWidget {
  const ListBody({super.key, required this.listNotifier});

  final ListModel listNotifier;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: <Widget>[
          const Text('Current values:'),
          Expanded(
            child: ListenableBuilder(
              listenable: listNotifier,
              builder: (BuildContext context, Widget? child) {
                // We rebuild the ListView each time the list changes,
                // so that the framework knows to update the rendering.
                final List<int> values = listNotifier.values; // copy the list
                return ListView.builder(
                  itemBuilder:
                      (BuildContext context, int index) =>
                          ListTile(title: Text('${values[index]}')),
                  itemCount: values.length,
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}
