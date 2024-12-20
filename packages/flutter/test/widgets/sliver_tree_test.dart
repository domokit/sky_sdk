// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

List<TreeSliverNode<String>> simpleNodeSet = <TreeSliverNode<String>>[
  TreeSliverNode<String>('Root 0'),
  TreeSliverNode<String>(
    'Root 1',
    expanded: true,
    children: <TreeSliverNode<String>>[
      TreeSliverNode<String>('Child 1:0'),
      TreeSliverNode<String>('Child 1:1'),
    ],
  ),
  TreeSliverNode<String>(
    'Root 2',
    children: <TreeSliverNode<String>>[
      TreeSliverNode<String>('Child 2:0'),
      TreeSliverNode<String>('Child 2:1'),
    ],
  ),
  TreeSliverNode<String>('Root 3'),
];

void main() {
  group('TreeSliverNode', () {
    setUp(() {
      // Reset node conditions for each test.
      simpleNodeSet = <TreeSliverNode<String>>[
        TreeSliverNode<String>('Root 0'),
        TreeSliverNode<String>(
          'Root 1',
          expanded: true,
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>('Child 1:0'),
            TreeSliverNode<String>('Child 1:1'),
          ],
        ),
        TreeSliverNode<String>(
          'Root 2',
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>('Child 2:0'),
            TreeSliverNode<String>('Child 2:1'),
          ],
        ),
        TreeSliverNode<String>('Root 3'),
      ];
    });

    test('getters, toString', () {
      final List<TreeSliverNode<String>> children = <TreeSliverNode<String>>[
        TreeSliverNode<String>('child'),
      ];
      final TreeSliverNode<String> node = TreeSliverNode<String>(
        'parent',
        children: children,
        expanded: true,
      );
      expect(node.content, 'parent');
      expect(node.children, children);
      expect(node.isExpanded, isTrue);
      expect(node.children.first.content, 'child');
      expect(node.children.first.children.isEmpty, isTrue);
      expect(node.children.first.isExpanded, isFalse);
      // Set by TreeSliver when built for tree integrity
      expect(node.depth, isNull);
      expect(node.parent, isNull);
      expect(node.children.first.depth, isNull);
      expect(node.children.first.parent, isNull);

      expect(node.toString(), 'TreeSliverNode: parent, depth: null, parent, expanded: true');
      expect(node.children.first.toString(), 'TreeSliverNode: child, depth: null, leaf');
    });

    testWidgets('TreeSliverNode sets ups parent and depth properties', (WidgetTester tester) async {
      final List<TreeSliverNode<String>> children = <TreeSliverNode<String>>[
        TreeSliverNode<String>('child'),
      ];
      final TreeSliverNode<String> node = TreeSliverNode<String>(
        'parent',
        children: children,
        expanded: true,
      );
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[
              TreeSliver<String>(tree: <TreeSliverNode<String>>[node]),
            ],
          ),
        ),
      );
      expect(node.content, 'parent');
      expect(node.children, children);
      expect(node.isExpanded, isTrue);
      expect(node.children.first.content, 'child');
      expect(node.children.first.children.isEmpty, isTrue);
      expect(node.children.first.isExpanded, isFalse);
      // Set by TreeSliver when built for tree integrity
      expect(node.depth, 0);
      expect(node.parent, isNull);
      expect(node.children.first.depth, 1);
      expect(node.children.first.parent, node);

      expect(node.toString(), 'TreeSliverNode: parent, depth: root, parent, expanded: true');
      expect(node.children.first.toString(), 'TreeSliverNode: child, depth: 1, leaf');
    });
  });

  group('TreeController', () {
    setUp(() {
      // Reset node conditions for each test.
      simpleNodeSet = <TreeSliverNode<String>>[
        TreeSliverNode<String>('Root 0'),
        TreeSliverNode<String>(
          'Root 1',
          expanded: true,
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>('Child 1:0'),
            TreeSliverNode<String>('Child 1:1'),
          ],
        ),
        TreeSliverNode<String>(
          'Root 2',
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>('Child 2:0'),
            TreeSliverNode<String>('Child 2:1'),
          ],
        ),
        TreeSliverNode<String>('Root 3'),
      ];
    });

    testWidgets('Can set controller on TreeSliver', (WidgetTester tester) async {
      final TreeSliverController controller = TreeSliverController();
      TreeSliverController? returnedController;
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[
              TreeSliver<String>(
                tree: simpleNodeSet,
                controller: controller,
                treeNodeBuilder: (
                  BuildContext context,
                  TreeSliverNode<Object?> node,
                  AnimationStyle toggleAnimationStyle,
                ) {
                  returnedController ??= TreeSliverController.of(context);
                  return TreeSliver.defaultTreeNodeBuilder(context, node, toggleAnimationStyle);
                },
              ),
            ],
          ),
        ),
      );
      expect(controller, returnedController);
    });

    testWidgets('Can get default controller on TreeSliver', (WidgetTester tester) async {
      TreeSliverController? returnedController;
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[
              TreeSliver<String>(
                tree: simpleNodeSet,
                treeNodeBuilder: (
                  BuildContext context,
                  TreeSliverNode<Object?> node,
                  AnimationStyle toggleAnimationStyle,
                ) {
                  returnedController ??= TreeSliverController.maybeOf(context);
                  return TreeSliver.defaultTreeNodeBuilder(context, node, toggleAnimationStyle);
                },
              ),
            ],
          ),
        ),
      );
      expect(returnedController, isNotNull);
    });

    testWidgets('Can get node for TreeSliverNode.content', (WidgetTester tester) async {
      final TreeSliverController controller = TreeSliverController();
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[TreeSliver<String>(tree: simpleNodeSet, controller: controller)],
          ),
        ),
      );

      expect(controller.getNodeFor('Root 0'), simpleNodeSet[0]);
    });

    testWidgets('Can get isExpanded for a node', (WidgetTester tester) async {
      final TreeSliverController controller = TreeSliverController();
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[TreeSliver<String>(tree: simpleNodeSet, controller: controller)],
          ),
        ),
      );
      expect(controller.isExpanded(simpleNodeSet[0]), isFalse);
      expect(controller.isExpanded(simpleNodeSet[1]), isTrue);
    });

    testWidgets('Can get isActive for a node', (WidgetTester tester) async {
      final TreeSliverController controller = TreeSliverController();
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[TreeSliver<String>(tree: simpleNodeSet, controller: controller)],
          ),
        ),
      );
      expect(controller.isActive(simpleNodeSet[0]), isTrue);
      expect(controller.isActive(simpleNodeSet[1]), isTrue);
      // The parent 'Root 2' is not expanded, so its children are not active.
      expect(controller.isExpanded(simpleNodeSet[2]), isFalse);
      expect(controller.isActive(simpleNodeSet[2].children[0]), isFalse);
    });

    testWidgets('Can toggleNode, to collapse or expand', (WidgetTester tester) async {
      final TreeSliverController controller = TreeSliverController();
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[TreeSliver<String>(tree: simpleNodeSet, controller: controller)],
          ),
        ),
      );

      // The parent 'Root 2' is not expanded, so its children are not active.
      expect(controller.isExpanded(simpleNodeSet[2]), isFalse);
      expect(controller.isActive(simpleNodeSet[2].children[0]), isFalse);
      // Toggle 'Root 2' to expand it
      controller.toggleNode(simpleNodeSet[2]);
      expect(controller.isExpanded(simpleNodeSet[2]), isTrue);
      expect(controller.isActive(simpleNodeSet[2].children[0]), isTrue);

      // The parent 'Root 1' is expanded, so its children are active.
      expect(controller.isExpanded(simpleNodeSet[1]), isTrue);
      expect(controller.isActive(simpleNodeSet[1].children[0]), isTrue);
      // Collapse 'Root 1'
      controller.toggleNode(simpleNodeSet[1]);
      expect(controller.isExpanded(simpleNodeSet[1]), isFalse);
      expect(controller.isActive(simpleNodeSet[1].children[0]), isTrue);
      // Nodes are not removed from the active list until the collapse animation
      // completes. The parent expansion state also updates.
      await tester.pumpAndSettle();
      expect(controller.isExpanded(simpleNodeSet[1]), isFalse);
      expect(controller.isActive(simpleNodeSet[1].children[0]), isFalse);
    });

    testWidgets('Can expandNode, then collapseAll', (WidgetTester tester) async {
      final TreeSliverController controller = TreeSliverController();
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[TreeSliver<String>(tree: simpleNodeSet, controller: controller)],
          ),
        ),
      );

      // The parent 'Root 2' is not expanded, so its children are not active.
      expect(controller.isExpanded(simpleNodeSet[2]), isFalse);
      expect(controller.isActive(simpleNodeSet[2].children[0]), isFalse);
      // Expand 'Root 2'
      controller.expandNode(simpleNodeSet[2]);
      expect(controller.isExpanded(simpleNodeSet[2]), isTrue);
      expect(controller.isActive(simpleNodeSet[2].children[0]), isTrue);

      // Both parents from our simple node set are expanded.
      // 'Root 1'
      expect(controller.isExpanded(simpleNodeSet[1]), isTrue);
      // 'Root 2'
      expect(controller.isExpanded(simpleNodeSet[2]), isTrue);
      // Collapse both.
      controller.collapseAll();
      await tester.pumpAndSettle();
      // Both parents from our simple node set have collapsed.
      // 'Root 1'
      expect(controller.isExpanded(simpleNodeSet[1]), isFalse);
      // 'Root 2'
      expect(controller.isExpanded(simpleNodeSet[2]), isFalse);
    });

    testWidgets('Can collapseNode, then expandAll', (WidgetTester tester) async {
      final TreeSliverController controller = TreeSliverController();
      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[TreeSliver<String>(tree: simpleNodeSet, controller: controller)],
          ),
        ),
      );

      // The parent 'Root 1' is expanded, so its children are active.
      expect(controller.isExpanded(simpleNodeSet[1]), isTrue);
      expect(controller.isActive(simpleNodeSet[1].children[0]), isTrue);
      // Collapse 'Root 1'
      controller.collapseNode(simpleNodeSet[1]);
      expect(controller.isExpanded(simpleNodeSet[1]), isFalse);
      expect(controller.isActive(simpleNodeSet[1].children[0]), isTrue);
      // Nodes are not removed from the active list until the collapse animation
      // completes.
      await tester.pumpAndSettle();
      expect(controller.isActive(simpleNodeSet[1].children[0]), isFalse);

      // Both parents from our simple node set are collapsed.
      // 'Root 1'
      expect(controller.isExpanded(simpleNodeSet[1]), isFalse);
      // 'Root 2'
      expect(controller.isExpanded(simpleNodeSet[2]), isFalse);
      // Expand both.
      controller.expandAll();
      // Both parents from our simple node set are expanded.
      // 'Root 1'
      expect(controller.isExpanded(simpleNodeSet[1]), isTrue);
      // 'Root 2'
      expect(controller.isExpanded(simpleNodeSet[2]), isTrue);
    });
  });

  test('TreeSliverIndentationType values are properly reflected', () {
    double value = TreeSliverIndentationType.standard.value;
    expect(value, 10.0);

    value = TreeSliverIndentationType.none.value;
    expect(value, 0.0);

    value = TreeSliverIndentationType.custom(50.0).value;
    expect(value, 50.0);
  });

  testWidgets('.toggleNodeWith, onNodeToggle', (WidgetTester tester) async {
    simpleNodeSet = <TreeSliverNode<String>>[
      TreeSliverNode<String>('Root 0'),
      TreeSliverNode<String>(
        'Root 1',
        expanded: true,
        children: <TreeSliverNode<String>>[
          TreeSliverNode<String>('Child 1:0'),
          TreeSliverNode<String>('Child 1:1'),
        ],
      ),
      TreeSliverNode<String>(
        'Root 2',
        children: <TreeSliverNode<String>>[
          TreeSliverNode<String>('Child 2:0'),
          TreeSliverNode<String>('Child 2:1'),
        ],
      ),
      TreeSliverNode<String>('Root 3'),
    ];

    final TreeSliverController controller = TreeSliverController();
    // The default node builder wraps the leading icon with toggleNodeWith.
    bool toggled = false;
    TreeSliverNode<String>? toggledNode;
    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            TreeSliver<String>(
              tree: simpleNodeSet,
              controller: controller,
              onNodeToggle: (TreeSliverNode<Object?> node) {
                toggled = true;
                toggledNode = node as TreeSliverNode<String>;
              },
            ),
          ],
        ),
      ),
    );
    expect(controller.isExpanded(simpleNodeSet[1]), isTrue);
    await tester.tap(find.byType(Icon).first);
    await tester.pump();
    expect(controller.isExpanded(simpleNodeSet[1]), isFalse);
    expect(toggled, isTrue);
    expect(toggledNode, simpleNodeSet[1]);
    await tester.pumpAndSettle();
    expect(controller.isExpanded(simpleNodeSet[1]), isFalse);
    toggled = false;
    toggledNode = null;
    // Use toggleNodeWith to make the whole row trigger the node state.
    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            TreeSliver<String>(
              tree: simpleNodeSet,
              controller: controller,
              onNodeToggle: (TreeSliverNode<Object?> node) {
                toggled = true;
                toggledNode = node as TreeSliverNode<String>;
              },
              treeNodeBuilder: (
                BuildContext context,
                TreeSliverNode<Object?> node,
                AnimationStyle toggleAnimationStyle,
              ) {
                final Duration animationDuration =
                    toggleAnimationStyle.duration ?? TreeSliver.defaultAnimationDuration;
                final Curve animationCurve =
                    toggleAnimationStyle.curve ?? TreeSliver.defaultAnimationCurve;
                // This makes the whole row trigger toggling.
                return TreeSliver.wrapChildToToggleNode(
                  node: node,
                  child: Padding(
                    padding: const EdgeInsets.all(8.0),
                    child: Row(
                      children: <Widget>[
                        // Icon for parent nodes
                        SizedBox.square(
                          dimension: 30.0,
                          child:
                              node.children.isNotEmpty
                                  ? AnimatedRotation(
                                    turns: node.isExpanded ? 0.25 : 0.0,
                                    duration: animationDuration,
                                    curve: animationCurve,
                                    child: const Icon(IconData(0x25BA), size: 14),
                                  )
                                  : null,
                        ),
                        // Spacer
                        const SizedBox(width: 8.0),
                        // Content
                        Text(node.content.toString()),
                      ],
                    ),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
    // Still collapsed from earlier
    expect(controller.isExpanded(simpleNodeSet[1]), isFalse);
    // Tapping on the text instead of the Icon.
    await tester.tap(find.text('Root 1'));
    await tester.pump();
    expect(controller.isExpanded(simpleNodeSet[1]), isTrue);
    expect(toggled, isTrue);
    expect(toggledNode, simpleNodeSet[1]);
  });

  testWidgets('AnimationStyle is piped through to node builder', (WidgetTester tester) async {
    simpleNodeSet = <TreeSliverNode<String>>[
      TreeSliverNode<String>('Root 0'),
      TreeSliverNode<String>(
        'Root 1',
        expanded: true,
        children: <TreeSliverNode<String>>[
          TreeSliverNode<String>('Child 1:0'),
          TreeSliverNode<String>('Child 1:1'),
        ],
      ),
      TreeSliverNode<String>(
        'Root 2',
        children: <TreeSliverNode<String>>[
          TreeSliverNode<String>('Child 2:0'),
          TreeSliverNode<String>('Child 2:1'),
        ],
      ),
      TreeSliverNode<String>('Root 3'),
    ];

    AnimationStyle? style;
    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            TreeSliver<String>(
              tree: simpleNodeSet,
              treeNodeBuilder: (
                BuildContext context,
                TreeSliverNode<Object?> node,
                AnimationStyle toggleAnimationStyle,
              ) {
                style ??= toggleAnimationStyle;
                return Text(node.content.toString());
              },
            ),
          ],
        ),
      ),
    );
    // Default
    expect(style, TreeSliver.defaultToggleAnimationStyle);

    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            TreeSliver<String>(
              tree: simpleNodeSet,
              toggleAnimationStyle: AnimationStyle.noAnimation,
              treeNodeBuilder: (
                BuildContext context,
                TreeSliverNode<Object?> node,
                AnimationStyle toggleAnimationStyle,
              ) {
                style = toggleAnimationStyle;
                return Text(node.content.toString());
              },
            ),
          ],
        ),
      ),
    );
    expect(style, isNotNull);
    expect(style!.curve, isNull);
    expect(style!.duration, Duration.zero);
    style = null;

    await tester.pumpWidget(
      MaterialApp(
        home: CustomScrollView(
          slivers: <Widget>[
            TreeSliver<String>(
              tree: simpleNodeSet,
              toggleAnimationStyle: AnimationStyle(
                curve: Curves.easeIn,
                duration: const Duration(milliseconds: 200),
              ),
              treeNodeBuilder: (
                BuildContext context,
                TreeSliverNode<Object?> node,
                AnimationStyle toggleAnimationStyle,
              ) {
                style ??= toggleAnimationStyle;
                return Text(node.content.toString());
              },
            ),
          ],
        ),
      ),
    );
    expect(style, isNotNull);
    expect(style!.curve, Curves.easeIn);
    expect(style!.duration, const Duration(milliseconds: 200));
  });

  testWidgets('Adding more root TreeViewNodes are reflected in the tree', (
    WidgetTester tester,
  ) async {
    simpleNodeSet = <TreeSliverNode<String>>[
      TreeSliverNode<String>('Root 0'),
      TreeSliverNode<String>(
        'Root 1',
        expanded: true,
        children: <TreeSliverNode<String>>[
          TreeSliverNode<String>('Child 1:0'),
          TreeSliverNode<String>('Child 1:1'),
        ],
      ),
      TreeSliverNode<String>(
        'Root 2',
        children: <TreeSliverNode<String>>[
          TreeSliverNode<String>('Child 2:0'),
          TreeSliverNode<String>('Child 2:1'),
        ],
      ),
      TreeSliverNode<String>('Root 3'),
    ];
    final TreeSliverController controller = TreeSliverController();
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: CustomScrollView(
                slivers: <Widget>[TreeSliver<String>(tree: simpleNodeSet, controller: controller)],
              ),
              floatingActionButton: FloatingActionButton(
                onPressed: () {
                  setState(() {
                    simpleNodeSet.add(TreeSliverNode<String>('Added root'));
                  });
                },
              ),
            );
          },
        ),
      ),
    );
    await tester.pump();

    expect(find.text('Root 0'), findsOneWidget);
    expect(find.text('Root 1'), findsOneWidget);
    expect(find.text('Child 1:0'), findsOneWidget);
    expect(find.text('Child 1:1'), findsOneWidget);
    expect(find.text('Root 2'), findsOneWidget);
    expect(find.text('Child 2:0'), findsNothing);
    expect(find.text('Child 2:1'), findsNothing);
    expect(find.text('Root 3'), findsOneWidget);
    expect(find.text('Added root'), findsNothing);

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();

    expect(find.text('Root 0'), findsOneWidget);
    expect(find.text('Root 1'), findsOneWidget);
    expect(find.text('Child 1:0'), findsOneWidget);
    expect(find.text('Child 1:1'), findsOneWidget);
    expect(find.text('Root 2'), findsOneWidget);
    expect(find.text('Child 2:0'), findsNothing);
    expect(find.text('Child 2:1'), findsNothing);
    expect(find.text('Root 3'), findsOneWidget);
    // Node was added
    expect(find.text('Added root'), findsOneWidget);
  });

  testWidgets('Adding more TreeViewNodes below the root are reflected in the tree', (
    WidgetTester tester,
  ) async {
    simpleNodeSet = <TreeSliverNode<String>>[
      TreeSliverNode<String>('Root 0'),
      TreeSliverNode<String>(
        'Root 1',
        expanded: true,
        children: <TreeSliverNode<String>>[
          TreeSliverNode<String>('Child 1:0'),
          TreeSliverNode<String>('Child 1:1'),
        ],
      ),
      TreeSliverNode<String>(
        'Root 2',
        children: <TreeSliverNode<String>>[
          TreeSliverNode<String>('Child 2:0'),
          TreeSliverNode<String>('Child 2:1'),
        ],
      ),
      TreeSliverNode<String>('Root 3'),
    ];
    final TreeSliverController controller = TreeSliverController();
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: CustomScrollView(
                slivers: <Widget>[TreeSliver<String>(tree: simpleNodeSet, controller: controller)],
              ),
              floatingActionButton: FloatingActionButton(
                onPressed: () {
                  setState(() {
                    simpleNodeSet[1].children.add(TreeSliverNode<String>('Added child'));
                  });
                },
              ),
            );
          },
        ),
      ),
    );
    await tester.pump();
    expect(find.text('Root 0'), findsOneWidget);
    expect(find.text('Root 1'), findsOneWidget);
    expect(find.text('Child 1:0'), findsOneWidget);
    expect(find.text('Child 1:1'), findsOneWidget);
    expect(find.text('Added child'), findsNothing);
    expect(find.text('Root 2'), findsOneWidget);
    expect(find.text('Child 2:0'), findsNothing);
    expect(find.text('Child 2:1'), findsNothing);
    expect(find.text('Root 3'), findsOneWidget);
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump();
    expect(find.text('Root 0'), findsOneWidget);
    expect(find.text('Root 1'), findsOneWidget);
    expect(find.text('Child 1:0'), findsOneWidget);
    expect(find.text('Child 1:1'), findsOneWidget);
    // Child node was added
    expect(find.text('Added child'), findsOneWidget);
    expect(find.text('Root 2'), findsOneWidget);
    expect(find.text('Child 2:0'), findsNothing);
    expect(find.text('Child 2:1'), findsNothing);
    expect(find.text('Root 3'), findsOneWidget);
  });

  testWidgets(
    'TreeSliverNode should close all children when collapsed when animation is disabled',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/153889
      final TreeSliverController controller = TreeSliverController();
      final List<TreeSliverNode<String>> tree = <TreeSliverNode<String>>[
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

      await tester.pumpWidget(
        MaterialApp(
          home: CustomScrollView(
            slivers: <Widget>[
              TreeSliver<String>(
                tree: tree,
                controller: controller,
                toggleAnimationStyle: AnimationStyle.noAnimation,
                treeNodeBuilder: (
                  BuildContext context,
                  TreeSliverNode<Object?> node,
                  AnimationStyle animationStyle,
                ) {
                  final Widget child = GestureDetector(
                    behavior: HitTestBehavior.translucent,
                    onTap: () => controller.toggleNode(node),
                    child: TreeSliver.defaultTreeNodeBuilder(context, node, animationStyle),
                  );

                  return child;
                },
              ),
            ],
          ),
        ),
      );

      expect(find.text('First'), findsOneWidget);
      expect(find.text('Second'), findsOneWidget);
      expect(find.text('Third'), findsOneWidget);
      expect(find.text('Fourth'), findsOneWidget);
      expect(find.text('alpha'), findsNothing);
      expect(find.text('beta'), findsNothing);
      expect(find.text('kappa'), findsNothing);
      expect(find.text('gamma'), findsOneWidget);
      expect(find.text('delta'), findsOneWidget);
      expect(find.text('epsilon'), findsOneWidget);
      expect(find.text('uno'), findsNothing);
      expect(find.text('dos'), findsNothing);
      expect(find.text('tres'), findsNothing);

      await tester.tap(find.text('Second'));
      await tester.pumpAndSettle();

      expect(find.text('alpha'), findsOneWidget);

      await tester.tap(find.text('alpha'));
      await tester.pumpAndSettle();

      expect(find.text('uno'), findsOneWidget);
      expect(find.text('dos'), findsOneWidget);
      expect(find.text('tres'), findsOneWidget);

      await tester.tap(find.text('alpha'));
      await tester.pumpAndSettle();

      expect(find.text('uno'), findsNothing);
      expect(find.text('dos'), findsNothing);
      expect(find.text('tres'), findsNothing);
    },
  );

  testWidgets(
    'TreeSliverNode should close all children when collapsed when animation is completed',
    (WidgetTester tester) async {
      final TreeSliverController controller = TreeSliverController();
      final List<TreeSliverNode<String>> tree = <TreeSliverNode<String>>[
        TreeSliverNode<String>(
          'First',
          expanded: true,
          children: <TreeSliverNode<String>>[
            TreeSliverNode<String>(
              'alpha',
              expanded: true,
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
      ];

      Widget buildTreeSliver(TreeSliverController controller) {
        return MaterialApp(
          home: Scaffold(
            body: CustomScrollView(
              shrinkWrap: true,
              slivers: <Widget>[
                TreeSliver<String>(
                  tree: tree,
                  controller: controller,
                  toggleAnimationStyle: AnimationStyle(
                    curve: Curves.easeInOut,
                    duration: const Duration(milliseconds: 200),
                  ),
                  treeNodeBuilder: (
                    BuildContext context,
                    TreeSliverNode<Object?> node,
                    AnimationStyle animationStyle,
                  ) {
                    final Widget child = GestureDetector(
                      key: ValueKey<String>(node.content! as String),
                      behavior: HitTestBehavior.translucent,
                      onTap: () => controller.toggleNode(node),
                      child: TreeSliver.defaultTreeNodeBuilder(context, node, animationStyle),
                    );
                    return child;
                  },
                ),
              ],
            ),
          ),
        );
      }

      await tester.pumpWidget(buildTreeSliver(controller));

      expect(find.text('alpha'), findsOneWidget);
      expect(find.text('uno'), findsOneWidget);
      expect(find.text('dos'), findsOneWidget);
      expect(find.text('tres'), findsOneWidget);

      // Using runAsync to handle collapse and animations properly.
      await tester.runAsync(() async {
        await tester.tap(find.text('alpha'));
        await tester.pumpAndSettle();

        expect(find.text('uno'), findsNothing);
        expect(find.text('dos'), findsNothing);
        expect(find.text('tres'), findsNothing);
      });
    },
  );
}
