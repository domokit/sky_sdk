// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SliverOpacity].

void main() => runApp(const SliverOpacityExampleApp());

class SliverOpacityExampleApp extends StatelessWidget {
  const SliverOpacityExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: SliverOpacityExample());
  }
}

class SliverOpacityExample extends StatefulWidget {
  const SliverOpacityExample({super.key});

  @override
  State<SliverOpacityExample> createState() => _SliverOpacityExampleState();
}

class _SliverOpacityExampleState extends State<SliverOpacityExample> {
  static const List<Widget> _listItems = <Widget>[
    ListTile(title: Text('Now you see me,')),
    ListTile(title: Text("Now you don't!")),
  ];

  bool _visible = true;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('SliverOpacity demo')),
      body: CustomScrollView(
        slivers: <Widget>[
          const SliverToBoxAdapter(
            child: ListTile(title: Text('Press on the button to toggle the list visibility.')),
          ),
          const SliverToBoxAdapter(child: ListTile(title: Text('Before the list...'))),
          SliverOpacity(
            opacity: _visible ? 1.0 : 0.0,
            sliver: SliverList(delegate: SliverChildListDelegate(_listItems)),
          ),
          const SliverToBoxAdapter(child: ListTile(title: Text('Before the list...'))),
        ],
      ),
      floatingActionButton: FloatingActionButton(
        child: const Icon(Icons.disabled_visible),
        onPressed: () {
          setState(() {
            _visible = !_visible;
          });
        },
      ),
    );
  }
}
