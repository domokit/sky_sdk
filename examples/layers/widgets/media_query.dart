// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class AdaptedListItem extends StatelessWidget {
  const AdaptedListItem({super.key, required this.name});

  final String name;

  @override
  Widget build(BuildContext context) {
    return Row(
      children: <Widget>[
        Container(
          width: 32.0,
          height: 32.0,
          margin: const EdgeInsets.all(8.0),
          color: Colors.lightBlueAccent.shade100,
        ),
        Text(name),
      ],
    );
  }
}

class AdaptedGridItem extends StatelessWidget {
  const AdaptedGridItem({super.key, required this.name});

  final String name;

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Column(
        children: <Widget>[
          Expanded(child: Container(color: Colors.lightBlueAccent.shade100)),
          Container(
            margin: const EdgeInsets.only(left: 8.0),
            child: Row(
              children: <Widget>[
                Expanded(child: Text(name)),
                const IconButton(icon: Icon(Icons.more_vert), onPressed: null),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

const double _kListItemExtent = 50.0;
const double _kMaxTileWidth = 150.0;
const double _kGridViewBreakpoint = 450.0;

class AdaptiveContainer extends StatelessWidget {
  const AdaptiveContainer({super.key, required this.names});

  final List<String> names;

  @override
  Widget build(BuildContext context) {
    if (MediaQuery.of(context).size.width < _kGridViewBreakpoint) {
      return ListView(
        itemExtent: _kListItemExtent,
        children: names.map<Widget>((String name) => AdaptedListItem(name: name)).toList(),
      );
    } else {
      return GridView.extent(
        maxCrossAxisExtent: _kMaxTileWidth,
        children: names.map<Widget>((String name) => AdaptedGridItem(name: name)).toList(),
      );
    }
  }
}

List<String> _initNames() => List<String>.generate(30, (int i) => 'Item $i');

final List<String> _kNames = _initNames();

void main() {
  runApp(
    MaterialApp(
      title: 'Media Query Example',
      home: Scaffold(
        appBar: AppBar(title: const Text('Media Query Example')),
        body: Material(child: AdaptiveContainer(names: _kNames)),
      ),
    ),
  );
}
