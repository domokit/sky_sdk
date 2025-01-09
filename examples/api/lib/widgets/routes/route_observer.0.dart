// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [RouteObserver].

final RouteObserver<ModalRoute<void>> routeObserver = RouteObserver<ModalRoute<void>>();

void main() {
  runApp(const RouteObserverApp());
}

class RouteObserverApp extends StatelessWidget {
  const RouteObserverApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      navigatorObservers: <NavigatorObserver>[routeObserver],
      home: const RouteObserverExample(),
    );
  }
}

class RouteObserverExample extends StatefulWidget {
  const RouteObserverExample({super.key});

  @override
  State<RouteObserverExample> createState() => _RouteObserverExampleState();
}

class _RouteObserverExampleState extends State<RouteObserverExample> with RouteAware {
  List<String> log = <String>[];

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    routeObserver.subscribe(this, ModalRoute.of(context)!);
  }

  @override
  void dispose() {
    routeObserver.unsubscribe(this);
    super.dispose();
  }

  @override
  void didPush() {
    // Route was pushed onto navigator and is now the topmost route.
    log.add('didPush');
  }

  @override
  void didPopNext() {
    // Covering route was popped off the navigator.
    log.add('didPopNext');
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            Text('RouteObserver log:', style: Theme.of(context).textTheme.headlineSmall),
            ConstrainedBox(
              constraints: const BoxConstraints(maxHeight: 300.0),
              child: ListView.builder(
                itemCount: log.length,
                itemBuilder: (BuildContext context, int index) {
                  if (log.isEmpty) {
                    return const SizedBox.shrink();
                  }
                  return Text(log[index], textAlign: TextAlign.center);
                },
              ),
            ),
            OutlinedButton(
              onPressed: () {
                Navigator.of(context).push<void>(
                  MaterialPageRoute<void>(builder: (BuildContext context) => const NextPage()),
                );
              },
              child: const Text('Go to next page'),
            ),
          ],
        ),
      ),
    );
  }
}

class NextPage extends StatelessWidget {
  const NextPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: FilledButton(
          onPressed: () {
            Navigator.of(context).pop();
          },
          child: const Text('Go back to RouteAware page'),
        ),
      ),
    );
  }
}
