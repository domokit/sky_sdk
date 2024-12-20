// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SliverAnimatedOpacity].

void main() => runApp(const SliverAnimatedOpacityExampleApp());

class SliverAnimatedOpacityExampleApp extends StatelessWidget {
  const SliverAnimatedOpacityExampleApp({super.key});

  static const Duration duration = Duration(milliseconds: 500);
  static const Curve curve = Curves.easeInOut;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SliverAnimatedOpacity Sample')),
        body: const Center(child: SliverAnimatedOpacityExample(duration: duration, curve: curve)),
      ),
    );
  }
}

class SliverAnimatedOpacityExample extends StatefulWidget {
  const SliverAnimatedOpacityExample({required this.duration, required this.curve, super.key});

  final Duration duration;

  final Curve curve;

  @override
  State<SliverAnimatedOpacityExample> createState() => _SliverAnimatedOpacityExampleState();
}

class _SliverAnimatedOpacityExampleState extends State<SliverAnimatedOpacityExample>
    with SingleTickerProviderStateMixin {
  bool _visible = true;

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      slivers: <Widget>[
        SliverAnimatedOpacity(
          opacity: _visible ? 1.0 : 0.0,
          duration: widget.duration,
          curve: widget.curve,
          sliver: SliverFixedExtentList(
            itemExtent: 100.0,
            delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
              return Container(color: index.isEven ? Colors.indigo[200] : Colors.orange[200]);
            }, childCount: 5),
          ),
        ),
        SliverToBoxAdapter(
          child: FloatingActionButton(
            onPressed: () {
              setState(() {
                _visible = !_visible;
              });
            },
            tooltip: 'Toggle opacity',
            child: const Icon(Icons.flip),
          ),
        ),
      ],
    );
  }
}
