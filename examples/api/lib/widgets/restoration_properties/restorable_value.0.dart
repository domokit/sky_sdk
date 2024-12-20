// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [RestorableValue].

void main() => runApp(const RestorableValueExampleApp());

class RestorableValueExampleApp extends StatelessWidget {
  const RestorableValueExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('RestorableValue Sample')),
        body: const RestorableValueExample(restorationId: 'main'),
      ),
    );
  }
}

class RestorableValueExample extends StatefulWidget {
  const RestorableValueExample({super.key, this.restorationId});

  final String? restorationId;

  @override
  State<RestorableValueExample> createState() => _RestorableValueExampleState();
}

/// RestorationProperty objects can be used because of RestorationMixin.
class _RestorableValueExampleState extends State<RestorableValueExample> with RestorationMixin {
  // In this example, the restoration ID for the mixin is passed in through
  // the [StatefulWidget]'s constructor.
  @override
  String? get restorationId => widget.restorationId;

  // The current value of the answer is stored in a [RestorableProperty].
  // During state restoration it is automatically restored to its old value.
  // If no restoration data is available to restore the answer from, it is
  // initialized to the specified default value, in this case 42.
  final RestorableInt _answer = RestorableInt(42);

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    // All restorable properties must be registered with the mixin. After
    // registration, the answer either has its old value restored or is
    // initialized to its default value.
    registerForRestoration(_answer, 'answer');
  }

  void _incrementAnswer() {
    setState(() {
      // The current value of the property can be accessed and modified via
      // the value getter and setter.
      _answer.value += 1;
    });
  }

  @override
  void dispose() {
    // Properties must be disposed when no longer used.
    _answer.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: OutlinedButton(onPressed: _incrementAnswer, child: Text('${_answer.value}')),
    );
  }
}
