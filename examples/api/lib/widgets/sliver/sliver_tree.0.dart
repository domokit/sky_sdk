// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [TreeSliver].

void main() => runApp(const TreeSliverExampleApp());

class TreeSliverExampleApp extends StatelessWidget {
  const TreeSliverExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: TreeSliverExample());
  }
}

class TreeSliverExample extends StatefulWidget {
  const TreeSliverExample({super.key});

  @override
  State<TreeSliverExample> createState() => _TreeSliverExampleState();
}

class _TreeSliverExampleState extends State<TreeSliverExample> {
  TreeSliverNode<String>? _selectedNode;
  final TreeSliverController controller = TreeSliverController();
  final List<TreeSliverNode<String>> _tree = <TreeSliverNode<String>>[
    TreeSliverNode<String>('First'),
    TreeSliverNode<String>(
      'Second',
      children: <TreeSliverNode<String>>[
        TreeSliverNode<String>(
          'alpha',
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>('uno'),
            TreeSliverNode<String>('dos'),
            TreeSliverNode<String>('tres'),
          ],
        ),
        TreeSliverNode<String>('beta'),
        TreeSliverNode<String>('kappa'),
      ],
    ),
    TreeSliverNode<String>(
      'Third',
      expanded: true,
      children: <TreeSliverNode<String>>[
        TreeSliverNode<String>('gamma'),
        TreeSliverNode<String>('delta'),
        TreeSliverNode<String>('epsilon'),
      ],
    ),
    TreeSliverNode<String>('Fourth'),
  ];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('TreeSliver Demo')),
      body: CustomScrollView(
        slivers: <Widget>[
          TreeSliver<String>(
            tree: _tree,
            controller: controller,
            treeNodeBuilder: (
              BuildContext context,
              TreeSliverNode<Object?> node,
              AnimationStyle animationStyle,
            ) {
              Widget child = GestureDetector(
                behavior: HitTestBehavior.translucent,
                onTap: () {
                  setState(() {
                    controller.toggleNode(node);
                    _selectedNode = node as TreeSliverNode<String>;
                  });
                },
                child: TreeSliver.defaultTreeNodeBuilder(context, node, animationStyle),
              );
              if (_selectedNode == node as TreeSliverNode<String>) {
                child = ColoredBox(color: Colors.purple[100]!, child: child);
              }
              return child;
            },
          ),
        ],
      ),
    );
  }
}
