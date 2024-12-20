// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Focus].

void main() => runApp(const FocusExampleApp());

class FocusExampleApp extends StatelessWidget {
  const FocusExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: FocusExample());
  }
}

class FocusExample extends StatefulWidget {
  const FocusExample({super.key});

  @override
  State<FocusExample> createState() => _FocusExampleState();
}

class _FocusExampleState extends State<FocusExample> {
  int focusedChild = 0;
  List<Widget> children = <Widget>[];
  List<FocusNode> childFocusNodes = <FocusNode>[];

  @override
  void initState() {
    super.initState();
    // Add the first child.
    _addChild();
  }

  @override
  void dispose() {
    for (final FocusNode node in childFocusNodes) {
      node.dispose();
    }
    super.dispose();
  }

  void _addChild() {
    // Calling requestFocus here creates a deferred request for focus, since the
    // node is not yet part of the focus tree.
    childFocusNodes.add(FocusNode(debugLabel: 'Child ${children.length}')..requestFocus());

    children.add(
      Padding(
        padding: const EdgeInsets.all(2.0),
        child: ActionChip(
          focusNode: childFocusNodes.last,
          label: Text('CHILD ${children.length}'),
          onPressed: () {},
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(child: Wrap(children: children)),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          setState(() {
            focusedChild = children.length;
            _addChild();
          });
        },
        child: const Icon(Icons.add),
      ),
    );
  }
}
