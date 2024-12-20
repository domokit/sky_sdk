// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Traversal Order of SliverList', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final ScrollController controller = ScrollController(initialScrollOffset: 3000.0);
    addTearDown(controller.dispose);

    final List<Widget> listChildren = List<Widget>.generate(30, (int i) {
      return SizedBox(
        height: 200.0,
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            Semantics(container: true, child: Text('Item ${i}a')),
            Semantics(container: true, child: Text('item ${i}b')),
          ],
        ),
      );
    });
    await tester.pumpWidget(
      Semantics(
        textDirection: TextDirection.ltr,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: CustomScrollView(
              controller: controller,
              semanticChildCount: 30,
              slivers: <Widget>[SliverList(delegate: SliverChildListDelegate(listChildren))],
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      scrollIndex: 15,
                      scrollChildren: 30,
                      flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                      actions: <SemanticsAction>[
                        SemanticsAction.scrollUp,
                        SemanticsAction.scrollDown,
                        SemanticsAction.scrollToOffset,
                      ],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 13a',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'item 13b',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 14a',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'item 14b',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(label: 'Item 15a', textDirection: TextDirection.ltr),
                            TestSemantics(label: 'item 15b', textDirection: TextDirection.ltr),
                          ],
                        ),
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(label: 'Item 16a', textDirection: TextDirection.ltr),
                            TestSemantics(label: 'item 16b', textDirection: TextDirection.ltr),
                          ],
                        ),
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(label: 'Item 17a', textDirection: TextDirection.ltr),
                            TestSemantics(label: 'item 17b', textDirection: TextDirection.ltr),
                          ],
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 18a',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'item 18b',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 19a',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'item 19b',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Traversal Order of SliverFixedExtentList', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final ScrollController controller = ScrollController(initialScrollOffset: 3000.0);
    addTearDown(controller.dispose);

    final List<Widget> listChildren = List<Widget>.generate(30, (int i) {
      return SizedBox(
        height: 200.0,
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            Semantics(container: true, child: Text('Item ${i}a')),
            Semantics(container: true, child: Text('item ${i}b')),
          ],
        ),
      );
    });
    await tester.pumpWidget(
      Semantics(
        textDirection: TextDirection.ltr,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: CustomScrollView(
              controller: controller,
              slivers: <Widget>[
                SliverFixedExtentList(
                  itemExtent: 200.0,
                  delegate: SliverChildListDelegate(listChildren, addSemanticIndexes: false),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                      actions: <SemanticsAction>[
                        SemanticsAction.scrollUp,
                        SemanticsAction.scrollDown,
                        SemanticsAction.scrollToOffset,
                      ],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 13a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'item 13b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 14a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'item 14b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(label: 'Item 15a', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'item 15b', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 16a', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'item 16b', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 17a', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'item 17b', textDirection: TextDirection.ltr),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 18a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'item 18b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 19a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'item 19b',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Traversal Order of SliverGrid', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final ScrollController controller = ScrollController(initialScrollOffset: 1600.0);
    addTearDown(controller.dispose);

    final List<Widget> listChildren = List<Widget>.generate(30, (int i) {
      return SizedBox(height: 200.0, child: Text('Item $i'));
    });
    await tester.pumpWidget(
      Semantics(
        textDirection: TextDirection.ltr,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: CustomScrollView(
              controller: controller,
              slivers: <Widget>[
                SliverGrid.count(
                  crossAxisCount: 2,
                  crossAxisSpacing: 400.0,
                  children: listChildren,
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                      actions: <SemanticsAction>[
                        SemanticsAction.scrollUp,
                        SemanticsAction.scrollDown,
                        SemanticsAction.scrollToOffset,
                      ],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 12',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 13',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 14',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 15',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(label: 'Item 16', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 17', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 18', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 19', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 20', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 21', textDirection: TextDirection.ltr),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 22',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 23',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 24',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 25',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Traversal Order of List of individual slivers', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final ScrollController controller = ScrollController(initialScrollOffset: 3000.0);
    addTearDown(controller.dispose);

    final List<Widget> listChildren = List<Widget>.generate(30, (int i) {
      return SliverToBoxAdapter(
        child: SizedBox(
          height: 200.0,
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: <Widget>[
              Semantics(container: true, child: Text('Item ${i}a')),
              Semantics(container: true, child: Text('item ${i}b')),
            ],
          ),
        ),
      );
    });
    await tester.pumpWidget(
      Semantics(
        textDirection: TextDirection.ltr,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: CustomScrollView(controller: controller, slivers: listChildren),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                      actions: <SemanticsAction>[
                        SemanticsAction.scrollUp,
                        SemanticsAction.scrollDown,
                        SemanticsAction.scrollToOffset,
                      ],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 13a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'item 13b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 14a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'item 14b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(label: 'Item 15a', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'item 15b', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 16a', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'item 16b', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'Item 17a', textDirection: TextDirection.ltr),
                        TestSemantics(label: 'item 17b', textDirection: TextDirection.ltr),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 18a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'item 18b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'Item 19a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: 'item 19b',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Traversal Order of in a SingleChildScrollView', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final ScrollController controller = ScrollController(initialScrollOffset: 3000.0);
    addTearDown(controller.dispose);

    final List<Widget> listChildren = List<Widget>.generate(30, (int i) {
      return SizedBox(
        height: 200.0,
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            Semantics(container: true, child: Text('Item ${i}a')),
            Semantics(container: true, child: Text('item ${i}b')),
          ],
        ),
      );
    });
    await tester.pumpWidget(
      Semantics(
        textDirection: TextDirection.ltr,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: SingleChildScrollView(
              controller: controller,
              child: Column(children: listChildren),
            ),
          ),
        ),
      ),
    );

    final List<TestSemantics> children = <TestSemantics>[];
    for (int index = 0; index < 30; index += 1) {
      final bool isHidden = index < 15 || index > 17;
      children.add(
        TestSemantics(
          flags: isHidden ? <SemanticsFlag>[SemanticsFlag.isHidden] : 0,
          label: 'Item ${index}a',
          textDirection: TextDirection.ltr,
        ),
      );
      children.add(
        TestSemantics(
          flags: isHidden ? <SemanticsFlag>[SemanticsFlag.isHidden] : 0,
          label: 'item ${index}b',
          textDirection: TextDirection.ltr,
        ),
      );
    }

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                  actions: <SemanticsAction>[
                    SemanticsAction.scrollUp,
                    SemanticsAction.scrollDown,
                    SemanticsAction.scrollToOffset,
                  ],
                  children: children,
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Traversal Order with center child', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Semantics(
        textDirection: TextDirection.ltr,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: const MediaQueryData(),
            child: Scrollable(
              viewportBuilder: (BuildContext context, ViewportOffset offset) {
                return Viewport(
                  offset: offset,
                  center: const ValueKey<int>(0),
                  slivers: List<Widget>.generate(30, (int i) {
                    final int item = i - 15;
                    return SliverToBoxAdapter(
                      key: ValueKey<int>(item),
                      child: SizedBox(
                        height: 200.0,
                        child: Row(
                          crossAxisAlignment: CrossAxisAlignment.stretch,
                          children: <Widget>[
                            Semantics(container: true, child: Text('${item}a')),
                            Semantics(container: true, child: Text('${item}b')),
                          ],
                        ),
                      ),
                    );
                  }),
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                      actions: <SemanticsAction>[
                        SemanticsAction.scrollUp,
                        SemanticsAction.scrollDown,
                        SemanticsAction.scrollToOffset,
                      ],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: '-2a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: '-2b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: '-1a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: '-1b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(label: '0a', textDirection: TextDirection.ltr),
                        TestSemantics(label: '0b', textDirection: TextDirection.ltr),
                        TestSemantics(label: '1a', textDirection: TextDirection.ltr),
                        TestSemantics(label: '1b', textDirection: TextDirection.ltr),
                        TestSemantics(label: '2a', textDirection: TextDirection.ltr),
                        TestSemantics(label: '2b', textDirection: TextDirection.ltr),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: '3a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: '3b',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: '4a',
                          textDirection: TextDirection.ltr,
                        ),
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                          label: '4b',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });
}
