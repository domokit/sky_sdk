// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

import 'component1.dart' deferred as component1;

void main() {
  enableFlutterDriverExtension();
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Deferred Components Example',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: const MyHomePage(),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  MyHomePageState createState() => MyHomePageState();
}

class MyHomePageState extends State<MyHomePage> {
  Future<void>? libraryFuture;

  Widget postLoadDisplayWidget = const Text('placeholder', key: Key('PlaceholderText'));

  @override
  void initState() {
    // Automatically trigger load for release test without driver.
    Future<void>.delayed(const Duration(milliseconds: 3000), () {
      _pressHandler();
    });
    super.initState();
  }

  void _pressHandler() {
    if (libraryFuture == null) {
      setState(() {
        libraryFuture = component1.loadLibrary().then((dynamic _) {
          // Delay to give debug runs more than one frame to capture
          // the placeholder text.
          Future<void>.delayed(const Duration(milliseconds: 750), () {
            setState(() {
              postLoadDisplayWidget = component1.LogoScreen();
            });
          });
        });
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    final Widget testWidget =
        libraryFuture == null
            ? const Text('preload', key: Key('PreloadText'))
            : FutureBuilder<void>(
              future: libraryFuture,
              builder: (BuildContext context, AsyncSnapshot<void> snapshot) {
                if (snapshot.connectionState == ConnectionState.done) {
                  if (snapshot.hasError) {
                    return Text('Error: ${snapshot.error}');
                  }
                  return postLoadDisplayWidget;
                }
                return postLoadDisplayWidget;
              },
            );
    return Scaffold(
      appBar: AppBar(title: const Text('Deferred components test')),
      body: Center(child: testWidget),
      floatingActionButton: FloatingActionButton(
        key: const Key('FloatingActionButton'),
        onPressed: _pressHandler,
        tooltip: 'Load',
        child: const Icon(Icons.add),
      ),
    );
  }
}
