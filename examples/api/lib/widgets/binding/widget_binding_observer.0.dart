// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [WidgetsBindingObserver].

void main() => runApp(const WidgetBindingObserverExampleApp());

class WidgetBindingObserverExampleApp extends StatelessWidget {
  const WidgetBindingObserverExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('WidgetBindingsObserver Sample')),
        body: const WidgetBindingsObserverSample(),
      ),
    );
  }
}

class WidgetBindingsObserverSample extends StatefulWidget {
  const WidgetBindingsObserverSample({super.key});

  @override
  State<WidgetBindingsObserverSample> createState() => _WidgetBindingsObserverSampleState();
}

class _WidgetBindingsObserverSampleState extends State<WidgetBindingsObserverSample>
    with WidgetsBindingObserver {
  final List<AppLifecycleState> _stateHistoryList = <AppLifecycleState>[];

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    if (WidgetsBinding.instance.lifecycleState != null) {
      _stateHistoryList.add(WidgetsBinding.instance.lifecycleState!);
    }
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    setState(() {
      _stateHistoryList.add(state);
    });
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (_stateHistoryList.isNotEmpty) {
      return ListView.builder(
        key: const ValueKey<String>('stateHistoryList'),
        itemCount: _stateHistoryList.length,
        itemBuilder: (BuildContext context, int index) {
          return Text('state is: ${_stateHistoryList[index]}');
        },
      );
    }

    return const Center(child: Text('There are no AppLifecycleStates to show.'));
  }
}
