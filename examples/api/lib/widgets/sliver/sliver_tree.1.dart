// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

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
  final List<TreeSliverNode<String>> tree = <TreeSliverNode<String>>[
    TreeSliverNode<String>('README.md'),
    TreeSliverNode<String>('analysis_options.yaml'),
    TreeSliverNode<String>(
      'lib',
      children: <TreeSliverNode<String>>[
        TreeSliverNode<String>(
          'src',
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>(
              'widgets',
              children: <TreeSliverNode<String>>[
                TreeSliverNode<String>('about.dart.dart'),
                TreeSliverNode<String>('app.dart'),
                TreeSliverNode<String>('basic.dart'),
                TreeSliverNode<String>('constants.dart'),
              ],
            ),
          ],
        ),
        TreeSliverNode<String>('widgets.dart'),
      ],
    ),
    TreeSliverNode<String>('pubspec.lock'),
    TreeSliverNode<String>('pubspec.yaml'),
    TreeSliverNode<String>(
      'test',
      children: <TreeSliverNode<String>>[
        TreeSliverNode<String>(
          'widgets',
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>('about_test.dart'),
            TreeSliverNode<String>('app_test.dart'),
            TreeSliverNode<String>('basic_test.dart'),
            TreeSliverNode<String>('constants_test.dart'),
          ],
        ),
      ],
    ),
  ];

  Widget _treeNodeBuilder(
    BuildContext context,
    TreeSliverNode<Object?> node,
    AnimationStyle toggleAnimationStyle,
  ) {
    final bool isParentNode = node.children.isNotEmpty;
    final BorderSide border = BorderSide(width: 2, color: Colors.purple[300]!);
    return TreeSliver.wrapChildToToggleNode(
      node: node,
      child: Row(
        children: <Widget>[
          // Custom indentation
          SizedBox(width: 10.0 * node.depth! + 8.0),
          DecoratedBox(
            decoration: BoxDecoration(
              border: node.parent != null ? Border(left: border, bottom: border) : null,
            ),
            child: const SizedBox(height: 50.0, width: 20.0),
          ),
          // Leading icon for parent nodes
          if (isParentNode)
            DecoratedBox(
              decoration: BoxDecoration(border: Border.all()),
              child: SizedBox.square(
                dimension: 20.0,
                child: Icon(node.isExpanded ? Icons.remove : Icons.add, size: 14),
              ),
            ),
          // Spacer
          const SizedBox(width: 8.0),
          // Content
          Text(node.content.toString()),
        ],
      ),
    );
  }

  Widget _getTree() {
    return DecoratedSliver(
      decoration: BoxDecoration(border: Border.all()),
      sliver: TreeSliver<String>(
        tree: tree,
        onNodeToggle: (TreeSliverNode<Object?> node) {
          setState(() {
            _selectedNode = node as TreeSliverNode<String>;
          });
        },
        treeNodeBuilder: _treeNodeBuilder,
        treeRowExtentBuilder: (
          TreeSliverNode<Object?> node,
          SliverLayoutDimensions layoutDimensions,
        ) {
          // This gives more space to parent nodes.
          return node.children.isNotEmpty ? 60.0 : 50.0;
        },
        // No internal indentation, the custom treeNodeBuilder applies its
        // own indentation to decorate in the indented space.
        indentation: TreeSliverIndentationType.none,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    // This example is assumes the full screen is available.
    final Size screenSize = MediaQuery.sizeOf(context);
    final List<Widget> selectedChildren = <Widget>[];
    if (_selectedNode != null) {
      selectedChildren.addAll(<Widget>[
        const Spacer(),
        Icon(_selectedNode!.children.isEmpty ? Icons.file_open_outlined : Icons.folder_outlined),
        const SizedBox(height: 16.0),
        Text(_selectedNode!.content),
        const Spacer(),
      ]);
    }
    return Scaffold(
      body: Row(
        children: <Widget>[
          SizedBox(
            width: screenSize.width / 2,
            height: double.infinity,
            child: CustomScrollView(slivers: <Widget>[_getTree()]),
          ),
          DecoratedBox(
            decoration: BoxDecoration(border: Border.all()),
            child: SizedBox(
              width: screenSize.width / 2,
              height: double.infinity,
              child: Center(child: Column(children: selectedChildren)),
            ),
          ),
        ],
      ),
    );
  }
}
