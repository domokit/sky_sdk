// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code example for [DecoratedSliver]
/// with clipping turned off in a parent [CustomScrollView].

void main() => runApp(const DecoratedSliverClipExampleApp());

class DecoratedSliverClipExampleApp extends StatelessWidget {
  const DecoratedSliverClipExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'DecoratedSliver Clip Example',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        useMaterial3: true,
      ),
      home: const DecoratedSliverClipExample(),
    );
  }
}

class DecoratedSliverClipExample extends StatefulWidget {
  const DecoratedSliverClipExample({super.key});

  @override
  State<DecoratedSliverClipExample> createState() => _DecoratedSliverClipExampleState();
}

class _DecoratedSliverClipExampleState extends State<DecoratedSliverClipExample> {
  double _height = 225.0;
  bool _isClipped = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF1C1C1C),
      body: Column(
        children: <Widget>[
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: <Widget>[
              Switch(
                inactiveTrackColor: Colors.cyan,
                activeColor: Colors.pink,
                onChanged: (bool value) {
                  setState(() {
                    _isClipped = value;
                  });
                },
                value: _isClipped,
              ),
              Slider(
                activeColor: Colors.pink,
                inactiveColor: Colors.cyan,
                onChanged: (double value) {
                  setState(() {
                    _height = value;
                  });
                },
                value: _height,
                min: 150,
                max: 225,
              ),
            ],
          ),
          const SizedBox(height: 20.0),
          Stack(
            children: <Widget>[
              Padding(
                padding: const EdgeInsets.all(24.0),
                child: SizedBox(
                  width: 400,
                  height: _height,
                  child: ResizableCustomScrollView(isClipped: _isClipped),
                ),
              ),
              Positioned(
                top: _height,
                left: 0,
                right: 0,
                child: SizedBox(
                  height: MediaQuery.of(context).size.height - _height,
                  width: double.infinity,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class ResizableCustomScrollView extends StatelessWidget {
  const ResizableCustomScrollView({super.key, required this.isClipped});

  final bool isClipped;

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      // The clip behavior defaults to Clip.hardEdge if no argument is provided.
      clipBehavior: isClipped ? Clip.hardEdge : Clip.none,
      slivers: <Widget>[
        DecoratedSliver(
          decoration: const ShapeDecoration(
            color: Color(0xFF2C2C2C),
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(6))),
            shadows: <BoxShadow>[
              BoxShadow(color: Colors.cyan, offset: Offset(3, 3), blurRadius: 24),
            ],
          ),
          sliver: SliverList.builder(
            itemCount: 5,
            itemBuilder:
                (_, int index) => Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: Row(
                    children: <Widget>[
                      const Icon(Icons.add_box, color: Color(0xFFA8A8A8)),
                      Flexible(
                        child: Text(
                          'Item $index',
                          style: const TextStyle(color: Color(0xFFA8A8A8)),
                        ),
                      ),
                    ],
                  ),
                ),
          ),
        ),
      ],
    );
  }
}
