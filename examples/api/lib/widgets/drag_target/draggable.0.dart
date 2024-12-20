// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Draggable].

void main() => runApp(const DraggableExampleApp());

class DraggableExampleApp extends StatelessWidget {
  const DraggableExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Draggable Sample')),
        body: const DraggableExample(),
      ),
    );
  }
}

class DraggableExample extends StatefulWidget {
  const DraggableExample({super.key});

  @override
  State<DraggableExample> createState() => _DraggableExampleState();
}

class _DraggableExampleState extends State<DraggableExample> {
  int acceptedData = 0;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: <Widget>[
        Draggable<int>(
          // Data is the value this Draggable stores.
          data: 10,
          feedback: Container(
            color: Colors.deepOrange,
            height: 100,
            width: 100,
            child: const Icon(Icons.directions_run),
          ),
          childWhenDragging: Container(
            height: 100.0,
            width: 100.0,
            color: Colors.pinkAccent,
            child: const Center(child: Text('Child When Dragging')),
          ),
          child: Container(
            height: 100.0,
            width: 100.0,
            color: Colors.lightGreenAccent,
            child: const Center(child: Text('Draggable')),
          ),
        ),
        DragTarget<int>(
          builder: (BuildContext context, List<dynamic> accepted, List<dynamic> rejected) {
            return Container(
              height: 100.0,
              width: 100.0,
              color: Colors.cyan,
              child: Center(child: Text('Value is updated to: $acceptedData')),
            );
          },
          onAcceptWithDetails: (DragTargetDetails<int> details) {
            setState(() {
              acceptedData += details.data;
            });
          },
        ),
      ],
    );
  }
}
