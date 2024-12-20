// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [NavigatorState.restorablePushReplacement].

void main() => runApp(const RestorablePushReplacementExampleApp());

class RestorablePushReplacementExampleApp extends StatelessWidget {
  const RestorablePushReplacementExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const RootRestorationScope(
      restorationId: 'app',
      child: MaterialApp(restorationScopeId: 'app', home: RestorablePushReplacementExample()),
    );
  }
}

class RestorablePushReplacementExample extends StatefulWidget {
  const RestorablePushReplacementExample({this.wasPushed = false, super.key});

  final bool wasPushed;

  @override
  State<RestorablePushReplacementExample> createState() => _RestorablePushReplacementExampleState();
}

@pragma('vm:entry-point')
class _RestorablePushReplacementExampleState extends State<RestorablePushReplacementExample> {
  @pragma('vm:entry-point')
  static Route<void> _myRouteBuilder(BuildContext context, Object? arguments) {
    return MaterialPageRoute<void>(
      builder: (BuildContext context) => const RestorablePushReplacementExample(wasPushed: true),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Sample Code')),
      body: Center(
        child:
            widget.wasPushed
                ? const Text('This is a new route.')
                : const Text('This is the initial route.'),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () => Navigator.of(context).restorablePushReplacement(_myRouteBuilder),
        tooltip: 'Increment Counter',
        child: const Icon(Icons.add),
      ),
    );
  }
}
