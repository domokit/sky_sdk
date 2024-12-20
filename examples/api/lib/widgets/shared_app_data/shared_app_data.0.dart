// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SharedAppData].

void main() {
  runApp(const SharedAppDataExampleApp());
}

class SharedAppDataExampleApp extends StatelessWidget {
  const SharedAppDataExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: SharedAppDataExample());
  }
}

// Demonstrates that changes to the SharedAppData _only_ cause the dependent
// widgets to be rebuilt. In this case that's the ShowSharedValue widget that's
// displaying the value of a key whose value has been updated.
class SharedAppDataExample extends StatefulWidget {
  const SharedAppDataExample({super.key});

  @override
  State<SharedAppDataExample> createState() => _SharedAppDataExampleState();
}

class _SharedAppDataExampleState extends State<SharedAppDataExample> {
  int _fooVersion = 0;
  int _barVersion = 0;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('SharedAppData Sample')),
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            const ShowSharedValue(appDataKey: 'foo'),
            const SizedBox(height: 16),
            const ShowSharedValue(appDataKey: 'bar'),
            const SizedBox(height: 16),
            ElevatedButton(
              child: const Text('change foo'),
              onPressed: () {
                _fooVersion += 1;
                // Changing the SharedAppData's value for 'foo' key causes the
                // widgets that depend on 'foo' key to be rebuilt.
                SharedAppData.setValue<String, String?>(
                  context,
                  'foo',
                  'FOO $_fooVersion',
                ); // No need to call setState().
              },
            ),
            const SizedBox(height: 16),
            ElevatedButton(
              child: const Text('change bar'),
              onPressed: () {
                _barVersion += 1;
                // Changing the SharedAppData's value for 'bar' key causes the
                // widgets that depend on 'bar' key to be rebuilt.
                SharedAppData.setValue<String, String?>(
                  context,
                  'bar',
                  'BAR $_barVersion',
                ); // No need to call setState().
              },
            ),
          ],
        ),
      ),
    );
  }
}

class ShowSharedValue extends StatelessWidget {
  const ShowSharedValue({super.key, required this.appDataKey});

  final String appDataKey;

  @override
  Widget build(BuildContext context) {
    // The SharedAppData.getValue() call causes this widget to depend on the
    // value of the SharedAppData's key. If it's changed, with
    // SharedAppData.setValue(), then this widget will be rebuilt.
    final String value = SharedAppData.getValue<String, String>(
      context,
      appDataKey,
      () => 'initial',
    );

    return Text('$appDataKey: $value');
  }
}
