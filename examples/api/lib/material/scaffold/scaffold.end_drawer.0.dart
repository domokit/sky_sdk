// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Scaffold.endDrawer].

void main() => runApp(const EndDrawerExampleApp());

class EndDrawerExampleApp extends StatelessWidget {
  const EndDrawerExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: EndDrawerExample());
  }
}

class EndDrawerExample extends StatefulWidget {
  const EndDrawerExample({super.key});

  @override
  State<EndDrawerExample> createState() => _EndDrawerExampleState();
}

class _EndDrawerExampleState extends State<EndDrawerExample> {
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();

  void _openEndDrawer() {
    _scaffoldKey.currentState!.openEndDrawer();
  }

  void _closeEndDrawer() {
    Navigator.of(context).pop();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      key: _scaffoldKey,
      appBar: AppBar(title: const Text('Drawer Demo')),
      body: Center(
        child: ElevatedButton(onPressed: _openEndDrawer, child: const Text('Open End Drawer')),
      ),
      endDrawer: Drawer(
        child: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              const Text('This is the Drawer'),
              ElevatedButton(onPressed: _closeEndDrawer, child: const Text('Close Drawer')),
            ],
          ),
        ),
      ),
      // Disable opening the end drawer with a swipe gesture.
      endDrawerEnableOpenDragGesture: false,
    );
  }
}
