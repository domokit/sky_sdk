// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() {
  runApp(const PlatformView());
}

class PlatformView extends StatelessWidget {
  const PlatformView({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Platform View',
      theme: ThemeData(primarySwatch: Colors.grey),
      home: const MyHomePage(title: 'Platform View'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  static const MethodChannel _methodChannel = MethodChannel('samples.flutter.io/platform_view');

  int _counter = 0;

  void _incrementCounter() {
    setState(() {
      _counter++;
    });
  }

  static Widget get _buttonText => switch (defaultTargetPlatform) {
    TargetPlatform.android => const Text('Continue in Android view'),
    TargetPlatform.iOS => const Text('Continue in iOS view'),
    TargetPlatform.windows => const Text('Continue in Windows view'),
    TargetPlatform.macOS => const Text('Continue in macOS view'),
    TargetPlatform.linux => const Text('Continue in Linux view'),
    TargetPlatform.fuchsia => throw UnimplementedError('Platform not yet implemented'),
  };

  Future<void> _launchPlatformCount() async {
    final int? platformCounter = await _methodChannel.invokeMethod('switchView', _counter);
    setState(() {
      _counter = platformCounter!;
    });
  }

  @override
  Widget build(BuildContext context) => Scaffold(
    appBar: AppBar(title: Text(widget.title)),
    body: Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: <Widget>[
        Expanded(
          child: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                Text(
                  'Button tapped $_counter time${_counter == 1 ? '' : 's'}.',
                  style: const TextStyle(fontSize: 17.0),
                ),
                Padding(
                  padding: const EdgeInsets.all(18.0),
                  child: ElevatedButton(onPressed: _launchPlatformCount, child: _buttonText),
                ),
              ],
            ),
          ),
        ),
        Container(
          padding: const EdgeInsets.only(bottom: 15.0, left: 5.0),
          child: Row(
            children: <Widget>[
              Image.asset('assets/flutter-mark-square-64.png', scale: 1.5),
              const Text('Flutter', style: TextStyle(fontSize: 30.0)),
            ],
          ),
        ),
      ],
    ),
    floatingActionButton: FloatingActionButton(
      onPressed: _incrementCounter,
      tooltip: 'Increment',
      child: const Icon(Icons.add),
    ),
  );
}
