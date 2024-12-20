// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [NavigatorState.restorablePushAndRemoveUntil].

void main() => runApp(const RestorablePushAndRemoveUntilExampleApp());

class RestorablePushAndRemoveUntilExampleApp extends StatelessWidget {
  const RestorablePushAndRemoveUntilExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const RootRestorationScope(
      restorationId: 'app',
      child: MaterialApp(restorationScopeId: 'app', home: RestorablePushAndRemoveUntilExample()),
    );
  }
}

class RestorablePushAndRemoveUntilExample extends StatefulWidget {
  const RestorablePushAndRemoveUntilExample({super.key});

  @override
  State<RestorablePushAndRemoveUntilExample> createState() =>
      _RestorablePushAndRemoveUntilExampleState();
}

@pragma('vm:entry-point')
class _RestorablePushAndRemoveUntilExampleState extends State<RestorablePushAndRemoveUntilExample> {
  @pragma('vm:entry-point')
  static Route<void> _myRouteBuilder(BuildContext context, Object? arguments) {
    return MaterialPageRoute<void>(
      builder: (BuildContext context) => const RestorablePushAndRemoveUntilExample(),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Sample Code')),
      floatingActionButton: FloatingActionButton(
        onPressed:
            () => Navigator.of(
              context,
            ).restorablePushAndRemoveUntil(_myRouteBuilder, ModalRoute.withName('/')),
        tooltip: 'Increment Counter',
        child: const Icon(Icons.add),
      ),
    );
  }
}
