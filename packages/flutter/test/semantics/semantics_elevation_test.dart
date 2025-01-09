// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('SemanticsNodes overlapping in z', (WidgetTester tester) async {
    // Cards are semantic boundaries that always own their own SemanticNode,
    // PhysicalModels merge their semantics information into parent.
    //
    // Side view of the widget tree:
    //
    //          Card('abs. elevation: 30') ---------------
    //                                            | 8                  ----------- Card('abs. elevation 25')
    //          Card('abs. elevation: 22') ---------------                  |
    //                                            | 7                       |
    // PhysicalModel('abs. elevation: 15') ---------------                  | 15
    //                                            | 5                       |
    //                                     --------------------------------------- Card('abs. elevation: 10')
    //                                                           | 10
    //                                                           |
    //                                     --------------------------------------- 'ground'
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      const MaterialApp(
        home: Column(
          children: <Widget>[
            Text('ground'),
            Card(
              elevation: 10.0,
              child: Column(
                children: <Widget>[
                  Text('absolute elevation: 10'),
                  PhysicalModel(
                    elevation: 5.0,
                    color: Colors.black,
                    child: Column(
                      children: <Widget>[
                        Text('absolute elevation: 15'),
                        Card(
                          elevation: 7.0,
                          child: Column(
                            children: <Widget>[
                              Text('absolute elevation: 22'),
                              Card(elevation: 8.0, child: Text('absolute elevation: 30')),
                            ],
                          ),
                        ),
                      ],
                    ),
                  ),
                  Card(elevation: 15.0, child: Text('absolute elevation: 25')),
                ],
              ),
            ),
          ],
        ),
      ),
    );

    final SemanticsNode ground = tester.getSemantics(find.text('ground'));
    expect(ground.thickness, 0.0);
    expect(ground.elevation, 0.0);
    expect(ground.label, 'ground');

    final SemanticsNode elevation10 = tester.getSemantics(find.text('absolute elevation: 10'));
    final SemanticsNode elevation15 = tester.getSemantics(find.text('absolute elevation: 15'));
    expect(elevation10, same(elevation15)); // configs got merged into each other.
    expect(elevation10.thickness, 15.0);
    expect(elevation10.elevation, 0.0);
    expect(elevation10.label, 'absolute elevation: 10\nabsolute elevation: 15');

    final SemanticsNode elevation22 = tester.getSemantics(find.text('absolute elevation: 22'));
    expect(elevation22.thickness, 7.0);
    expect(elevation22.elevation, 15.0);
    expect(elevation22.label, 'absolute elevation: 22');

    final SemanticsNode elevation25 = tester.getSemantics(find.text('absolute elevation: 25'));
    expect(elevation25.thickness, 15.0);
    expect(elevation25.elevation, 10.0);
    expect(elevation22.label, 'absolute elevation: 22');

    final SemanticsNode elevation30 = tester.getSemantics(find.text('absolute elevation: 30'));
    expect(elevation30.thickness, 8.0);
    expect(elevation30.elevation, 7.0);
    expect(elevation30.label, 'absolute elevation: 30');

    semantics.dispose();
  });

  testWidgets('SemanticsNodes overlapping in z with switched children', (
    WidgetTester tester,
  ) async {
    // Same as 'SemanticsNodes overlapping in z', but the order of children
    // is reversed

    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      const MaterialApp(
        home: Column(
          children: <Widget>[
            Text('ground'),
            Card(
              elevation: 10.0,
              child: Column(
                children: <Widget>[
                  Card(elevation: 15.0, child: Text('absolute elevation: 25')),
                  PhysicalModel(
                    elevation: 5.0,
                    color: Colors.black,
                    child: Column(
                      children: <Widget>[
                        Text('absolute elevation: 15'),
                        Card(
                          elevation: 7.0,
                          child: Column(
                            children: <Widget>[
                              Text('absolute elevation: 22'),
                              Card(elevation: 8.0, child: Text('absolute elevation: 30')),
                            ],
                          ),
                        ),
                      ],
                    ),
                  ),
                  Text('absolute elevation: 10'),
                ],
              ),
            ),
          ],
        ),
      ),
    );

    final SemanticsNode ground = tester.getSemantics(find.text('ground'));
    expect(ground.thickness, 0.0);
    expect(ground.elevation, 0.0);
    expect(ground.label, 'ground');

    final SemanticsNode elevation10 = tester.getSemantics(find.text('absolute elevation: 10'));
    final SemanticsNode elevation15 = tester.getSemantics(find.text('absolute elevation: 15'));
    expect(elevation10, same(elevation15)); // configs got merged into each other.
    expect(elevation10.thickness, 15.0);
    expect(elevation10.elevation, 0.0);
    expect(elevation10.label, 'absolute elevation: 15\nabsolute elevation: 10');

    final SemanticsNode elevation22 = tester.getSemantics(find.text('absolute elevation: 22'));
    expect(elevation22.thickness, 7.0);
    expect(elevation22.elevation, 15.0);
    expect(elevation22.label, 'absolute elevation: 22');

    final SemanticsNode elevation25 = tester.getSemantics(find.text('absolute elevation: 25'));
    expect(elevation25.thickness, 15.0);
    expect(elevation25.elevation, 10.0);
    expect(elevation22.label, 'absolute elevation: 22');

    final SemanticsNode elevation30 = tester.getSemantics(find.text('absolute elevation: 30'));
    expect(elevation30.thickness, 8.0);
    expect(elevation30.elevation, 7.0);
    expect(elevation30.label, 'absolute elevation: 30');

    semantics.dispose();
  });

  testWidgets('single node thickness', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(home: Center(child: Material(elevation: 24.0, child: Text('Hello')))),
    );

    final SemanticsNode node = tester.getSemantics(find.text('Hello'));
    expect(node.thickness, 0.0);
    expect(node.elevation, 24.0);
    expect(node.label, 'Hello');

    semantics.dispose();
  });

  testWidgets('force-merge', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Card(
          elevation: 10.0,
          child: Column(
            children: <Widget>[
              const Text('abs. elevation: 10.0'),
              MergeSemantics(
                child: Semantics(
                  explicitChildNodes:
                      true, // just to be sure that it's going to be an explicit merge
                  child: const Column(
                    children: <Widget>[
                      Card(elevation: 15.0, child: Text('abs. elevation 25.0')),
                      Card(elevation: 5.0, child: Text('abs. elevation 15.0')),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );

    final SemanticsNode elevation10 = tester.getSemantics(find.text('abs. elevation: 10.0'));
    expect(elevation10.thickness, 10.0);
    expect(elevation10.elevation, 0.0);
    expect(elevation10.label, 'abs. elevation: 10.0');
    expect(elevation10.childrenCount, 1);

    // TODO(goderbauer): remove awkward workaround when accessing force-merged
    //   SemanticsData becomes easier, https://github.com/flutter/flutter/issues/25669
    SemanticsData? mergedChildData;
    elevation10.visitChildren((SemanticsNode child) {
      expect(mergedChildData, isNull);
      mergedChildData = child.getSemanticsData();
      return true;
    });

    expect(mergedChildData!.thickness, 15.0);
    expect(mergedChildData!.elevation, 10.0);
    expect(mergedChildData!.label, 'abs. elevation 25.0\nabs. elevation 15.0');

    semantics.dispose();
  });

  testWidgets('force-merge with inversed children', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Card(
          elevation: 10.0,
          child: Column(
            children: <Widget>[
              const Text('abs. elevation: 10.0'),
              MergeSemantics(
                child: Semantics(
                  explicitChildNodes:
                      true, // just to be sure that it's going to be an explicit merge
                  child: const Column(
                    children: <Widget>[
                      Card(elevation: 5.0, child: Text('abs. elevation 15.0')),
                      Card(elevation: 15.0, child: Text('abs. elevation 25.0')),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );

    final SemanticsNode elevation10 = tester.getSemantics(find.text('abs. elevation: 10.0'));
    expect(elevation10.thickness, 10.0);
    expect(elevation10.elevation, 0.0);
    expect(elevation10.label, 'abs. elevation: 10.0');
    expect(elevation10.childrenCount, 1);

    // TODO(goderbauer): remove awkward workaround when accessing force-merged
    //   SemanticsData becomes easier, https://github.com/flutter/flutter/issues/25669
    SemanticsData? mergedChildData;
    elevation10.visitChildren((SemanticsNode child) {
      expect(mergedChildData, isNull);
      mergedChildData = child.getSemanticsData();
      return true;
    });

    expect(mergedChildData!.thickness, 15.0);
    expect(mergedChildData!.elevation, 10.0);
    expect(mergedChildData!.label, 'abs. elevation 15.0\nabs. elevation 25.0');

    semantics.dispose();
  });
}
