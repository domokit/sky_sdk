// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('FlexibleSpaceBar centers title on iOS', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android),
        home: Scaffold(appBar: AppBar(flexibleSpace: const FlexibleSpaceBar(title: Text('X')))),
      ),
    );

    final Finder title = find.text('X');
    Offset center = tester.getCenter(title);
    Size size = tester.getSize(title);
    expect(center.dx, lessThan(400.0 - size.width / 2.0));

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      // Clear the widget tree to avoid animating between platforms.
      await tester.pumpWidget(Container(key: UniqueKey()));

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: platform),
          home: Scaffold(appBar: AppBar(flexibleSpace: const FlexibleSpaceBar(title: Text('X')))),
        ),
      );

      center = tester.getCenter(title);
      size = tester.getSize(title);
      expect(center.dx, greaterThan(400.0 - size.width / 2.0));
      expect(center.dx, lessThan(400.0 + size.width / 2.0));
    }
  });

  testWidgets('Material3 - FlexibleSpaceBarSettings provides settings to a FlexibleSpaceBar', (
    WidgetTester tester,
  ) async {
    const double minExtent = 100.0;
    const double initExtent = 200.0;
    const double maxExtent = 300.0;
    const double alpha = 0.5;

    final FlexibleSpaceBarSettings customSettings =
        FlexibleSpaceBar.createSettings(
              currentExtent: initExtent,
              minExtent: minExtent,
              maxExtent: maxExtent,
              toolbarOpacity: alpha,
              child: AppBar(
                flexibleSpace: const FlexibleSpaceBar(
                  title: Text('title'),
                  background: Text('X2'),
                  collapseMode: CollapseMode.pin,
                ),
              ),
            )
            as FlexibleSpaceBarSettings;

    const Key dragTarget = Key('orange box');

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            key: dragTarget,
            primary: true,
            slivers: <Widget>[
              SliverPersistentHeader(
                floating: true,
                pinned: true,
                delegate: TestDelegate(settings: customSettings),
              ),
              SliverToBoxAdapter(child: Container(height: 1200.0, color: Colors.orange[400])),
            ],
          ),
        ),
      ),
    );

    final RenderBox clipRect = tester.renderObject(find.byType(ClipRect).at(1));
    final Transform transform = tester.firstWidget(
      find.descendant(of: find.byType(FlexibleSpaceBar), matching: find.byType(Transform)),
    );

    // The current (200) is half way between the min (100) and max (300) and the
    // lerp values used to calculate the scale are 1 and 1.5, so we check for 1.25.
    expect(transform.transform.getMaxScaleOnAxis(), 1.25);

    // The space bar rect always starts fully expanded.
    expect(clipRect.size.height, maxExtent);

    final Element actionTextBox = tester.element(find.text('title'));
    final Text textWidget = actionTextBox.widget as Text;
    final DefaultTextStyle defaultTextStyle = DefaultTextStyle.of(actionTextBox);

    final TextStyle effectiveStyle = defaultTextStyle.style.merge(textWidget.style);
    expect(effectiveStyle.color?.alpha, 128); // Which is alpha of .5

    // We drag up to fully collapse the space bar.
    await tester.drag(find.byKey(dragTarget), const Offset(0, -400.0));
    await tester.pumpAndSettle();

    expect(clipRect.size.height, minExtent);
  });

  testWidgets('Material2 - FlexibleSpaceBarSettings provides settings to a FlexibleSpaceBar', (
    WidgetTester tester,
  ) async {
    const double minExtent = 100.0;
    const double initExtent = 200.0;
    const double maxExtent = 300.0;
    const double alpha = 0.5;

    final FlexibleSpaceBarSettings customSettings =
        FlexibleSpaceBar.createSettings(
              currentExtent: initExtent,
              minExtent: minExtent,
              maxExtent: maxExtent,
              toolbarOpacity: alpha,
              child: AppBar(
                flexibleSpace: const FlexibleSpaceBar(
                  title: Text('title'),
                  background: Text('X2'),
                  collapseMode: CollapseMode.pin,
                ),
              ),
            )
            as FlexibleSpaceBarSettings;

    const Key dragTarget = Key('orange box');

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: CustomScrollView(
            key: dragTarget,
            primary: true,
            slivers: <Widget>[
              SliverPersistentHeader(
                floating: true,
                pinned: true,
                delegate: TestDelegate(settings: customSettings),
              ),
              SliverToBoxAdapter(child: Container(height: 1200.0, color: Colors.orange[400])),
            ],
          ),
        ),
      ),
    );

    final RenderBox clipRect = tester.renderObject(find.byType(ClipRect).first);
    final Transform transform = tester.firstWidget(
      find.descendant(of: find.byType(FlexibleSpaceBar), matching: find.byType(Transform)),
    );

    // The current (200) is half way between the min (100) and max (300) and the
    // lerp values used to calculate the scale are 1 and 1.5, so we check for 1.25.
    expect(transform.transform.getMaxScaleOnAxis(), 1.25);

    // The space bar rect always starts fully expanded.
    expect(clipRect.size.height, maxExtent);

    final Element actionTextBox = tester.element(find.text('title'));
    final Text textWidget = actionTextBox.widget as Text;
    final DefaultTextStyle defaultTextStyle = DefaultTextStyle.of(actionTextBox);

    final TextStyle effectiveStyle = defaultTextStyle.style.merge(textWidget.style);
    expect(effectiveStyle.color?.alpha, 128); // Which is alpha of .5

    // We drag up to fully collapse the space bar.
    await tester.drag(find.byKey(dragTarget), const Offset(0, -400.0));
    await tester.pumpAndSettle();

    expect(clipRect.size.height, minExtent);
  });

  testWidgets(
    'FlexibleSpaceBar.background is visible when using height other than kToolbarHeight',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/80451
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            appBar: AppBar(
              toolbarHeight: 300,
              flexibleSpace: const FlexibleSpaceBar(
                title: Text('Title'),
                background: Text('Background'),
                collapseMode: CollapseMode.pin,
              ),
            ),
            body: CustomScrollView(
              primary: true,
              slivers: <Widget>[
                SliverToBoxAdapter(child: Container(height: 1200.0, color: Colors.orange[400])),
              ],
            ),
          ),
        ),
      );

      final dynamic backgroundOpacity = tester.firstWidget(
        find.byWidgetPredicate(
          (Widget widget) => widget.runtimeType.toString() == '_FlexibleSpaceHeaderOpacity',
        ),
      );
      // accessing private type member.
      // ignore: avoid_dynamic_calls
      expect(backgroundOpacity.opacity, 1.0);
    },
  );

  testWidgets('Material3 - Collapsed FlexibleSpaceBar has correct semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const double expandedHeight = 200;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              const SliverAppBar(
                pinned: true,
                expandedHeight: expandedHeight,
                title: Text('Title'),
                flexibleSpace: FlexibleSpaceBar(background: Text('Expanded title')),
              ),
              SliverList(
                delegate: SliverChildListDelegate(<Widget>[
                  for (int i = 0; i < 50; i++)
                    SizedBox(height: 200, child: Center(child: Text('Item $i'))),
                ]),
              ),
            ],
          ),
        ),
      ),
    );

    TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          id: 1,
          rect: TestSemantics.fullScreen,
          children: <TestSemantics>[
            TestSemantics(
              id: 2,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  id: 3,
                  rect: TestSemantics.fullScreen,
                  flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 4,
                      rect: TestSemantics.fullScreen,
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 9,
                          rect: const Rect.fromLTRB(0.0, 0.0, 800.0, expandedHeight),
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 12,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 13,
                                  rect: const Rect.fromLTRB(0.0, 0.0, 110.0, 28.0),
                                  flags: <SemanticsFlag>[
                                    SemanticsFlag.isHeader,
                                    SemanticsFlag.namesRoute,
                                  ],
                                  label: 'Title',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                            TestSemantics(
                              id: 10,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 11,
                                  rect: const Rect.fromLTRB(0.0, 0.0, 800.0, expandedHeight),
                                  label: 'Expanded title',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                          ],
                        ),
                        TestSemantics(
                          id: 14,
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          rect: TestSemantics.fullScreen,
                          actions: <SemanticsAction>[
                            SemanticsAction.scrollUp,
                            SemanticsAction.scrollToOffset,
                          ],
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 5,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              label: 'Item 0',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 6,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              label: 'Item 1',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 7,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 2',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 8,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 3',
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
      ],
    );

    expect(semantics, hasSemantics(expectedSemantics, ignoreTransform: true));

    // We drag up to fully collapse the space bar.
    await tester.drag(find.text('Item 1'), const Offset(0, -600.0));
    await tester.pumpAndSettle();

    expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          id: 1,
          rect: TestSemantics.fullScreen,
          children: <TestSemantics>[
            TestSemantics(
              id: 2,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  id: 3,
                  rect: TestSemantics.fullScreen,
                  flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 4,
                      rect: TestSemantics.fullScreen,
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 9,
                          // The app bar is collapsed.
                          rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 56.0),
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 12,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 56.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 13,
                                  rect: const Rect.fromLTRB(0.0, 0.0, 110.0, 28.0),
                                  flags: <SemanticsFlag>[
                                    SemanticsFlag.isHeader,
                                    SemanticsFlag.namesRoute,
                                  ],
                                  label: 'Title',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                            // The flexible space bar still persists in the
                            // semantic tree even if it is collapsed.
                            TestSemantics(
                              id: 10,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 56.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 11,
                                  rect: const Rect.fromLTRB(0.0, 36.0, 800.0, 92.0),
                                  label: 'Expanded title',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                          ],
                        ),
                        TestSemantics(
                          id: 14,
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          rect: TestSemantics.fullScreen,
                          actions: <SemanticsAction>[
                            SemanticsAction.scrollUp,
                            SemanticsAction.scrollDown,
                            SemanticsAction.scrollToOffset,
                          ],
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 5,
                              rect: const Rect.fromLTRB(0.0, 150.0, 800.0, 200.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 0',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 6,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 1',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 7,
                              rect: const Rect.fromLTRB(0.0, 56.0, 800.0, 200.0),
                              label: 'Item 2',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 8,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              label: 'Item 3',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 15,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              label: 'Item 4',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 16,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 5',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 17,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 6',
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
      ],
    );

    expect(semantics, hasSemantics(expectedSemantics, ignoreTransform: true));

    semantics.dispose();
  });

  testWidgets('Material2 - Collapsed FlexibleSpaceBar has correct semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const double expandedHeight = 200;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              const SliverAppBar(
                pinned: true,
                expandedHeight: expandedHeight,
                title: Text('Title'),
                flexibleSpace: FlexibleSpaceBar(background: Text('Expanded title')),
              ),
              SliverList(
                delegate: SliverChildListDelegate(<Widget>[
                  for (int i = 0; i < 50; i++)
                    SizedBox(height: 200, child: Center(child: Text('Item $i'))),
                ]),
              ),
            ],
          ),
        ),
      ),
    );

    TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          id: 1,
          rect: TestSemantics.fullScreen,
          children: <TestSemantics>[
            TestSemantics(
              id: 2,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  id: 3,
                  rect: TestSemantics.fullScreen,
                  flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 4,
                      rect: TestSemantics.fullScreen,
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 9,
                          rect: const Rect.fromLTRB(0.0, 0.0, 800.0, expandedHeight),
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 12,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 13,
                                  rect: const Rect.fromLTRB(0.0, 0.0, 100.0, 20.0),
                                  flags: <SemanticsFlag>[
                                    SemanticsFlag.isHeader,
                                    SemanticsFlag.namesRoute,
                                  ],
                                  label: 'Title',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                            TestSemantics(
                              id: 10,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 11,
                                  rect: const Rect.fromLTRB(0.0, 0.0, 800.0, expandedHeight),
                                  label: 'Expanded title',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                          ],
                        ),
                        TestSemantics(
                          id: 14,
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          rect: TestSemantics.fullScreen,
                          actions: <SemanticsAction>[
                            SemanticsAction.scrollUp,
                            SemanticsAction.scrollToOffset,
                          ],
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 5,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              label: 'Item 0',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 6,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              label: 'Item 1',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 7,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 2',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 8,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 3',
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
      ],
    );

    expect(semantics, hasSemantics(expectedSemantics, ignoreTransform: true));

    // We drag up to fully collapse the space bar.
    await tester.drag(find.text('Item 1'), const Offset(0, -600.0));
    await tester.pumpAndSettle();

    expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          id: 1,
          rect: TestSemantics.fullScreen,
          children: <TestSemantics>[
            TestSemantics(
              id: 2,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  id: 3,
                  rect: TestSemantics.fullScreen,
                  flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 4,
                      rect: TestSemantics.fullScreen,
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 9,
                          // The app bar is collapsed.
                          rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 56.0),
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 12,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 56.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 13,
                                  rect: const Rect.fromLTRB(0.0, 0.0, 100.0, 20.0),
                                  flags: <SemanticsFlag>[
                                    SemanticsFlag.isHeader,
                                    SemanticsFlag.namesRoute,
                                  ],
                                  label: 'Title',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                            // The flexible space bar still persists in the
                            // semantic tree even if it is collapsed.
                            TestSemantics(
                              id: 10,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 56.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 11,
                                  rect: const Rect.fromLTRB(0.0, 36.0, 800.0, 92.0),
                                  label: 'Expanded title',
                                  textDirection: TextDirection.ltr,
                                ),
                              ],
                            ),
                          ],
                        ),
                        TestSemantics(
                          id: 14,
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          rect: TestSemantics.fullScreen,
                          actions: <SemanticsAction>[
                            SemanticsAction.scrollUp,
                            SemanticsAction.scrollDown,
                            SemanticsAction.scrollToOffset,
                          ],
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 5,
                              rect: const Rect.fromLTRB(0.0, 150.0, 800.0, 200.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 0',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 6,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 1',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 7,
                              rect: const Rect.fromLTRB(0.0, 56.0, 800.0, 200.0),
                              label: 'Item 2',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 8,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              label: 'Item 3',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 15,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              label: 'Item 4',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 16,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 5',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              id: 17,
                              rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0),
                              flags: <SemanticsFlag>[SemanticsFlag.isHidden],
                              label: 'Item 6',
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
      ],
    );

    expect(semantics, hasSemantics(expectedSemantics, ignoreTransform: true));

    semantics.dispose();
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/14227
  testWidgets('Material3 - FlexibleSpaceBar sets width constraints for the title', (
    WidgetTester tester,
  ) async {
    const double titleFontSize = 20.0;
    const double height = 300.0;
    late double width;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              width = MediaQuery.sizeOf(context).width;
              return CustomScrollView(
                slivers: <Widget>[
                  SliverAppBar(
                    expandedHeight: height,
                    pinned: true,
                    stretch: true,
                    flexibleSpace: FlexibleSpaceBar(
                      titlePadding: EdgeInsets.zero,
                      title: Text(
                        'X' * 2000,
                        maxLines: 1,
                        overflow: TextOverflow.ellipsis,
                        style: const TextStyle(fontSize: titleFontSize, height: 1.0),
                      ),
                      centerTitle: false,
                    ),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    final double textWidth = width;
    // The title is scaled and transformed to be 1.5 times bigger, when the
    // FlexibleSpaceBar is fully expanded, thus we expect the width to be
    // 1.5 times smaller than the full width. The height of the text is the same
    // as the font size, with 10 dps bottom margin.
    expect(
      tester.getRect(find.byType(Text)),
      rectMoreOrLessEquals(
        Rect.fromLTRB(0, height - titleFontSize - 10, textWidth, height),
        epsilon: 0.0001,
      ),
    );
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/14227
  testWidgets('Material2 - FlexibleSpaceBar sets width constraints for the title', (
    WidgetTester tester,
  ) async {
    const double titleFontSize = 20.0;
    const double height = 300.0;
    late double width;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              width = MediaQuery.sizeOf(context).width;
              return CustomScrollView(
                slivers: <Widget>[
                  SliverAppBar(
                    expandedHeight: height,
                    pinned: true,
                    stretch: true,
                    flexibleSpace: FlexibleSpaceBar(
                      titlePadding: EdgeInsets.zero,
                      title: Text(
                        'X' * 2000,
                        maxLines: 1,
                        overflow: TextOverflow.ellipsis,
                        style: const TextStyle(fontSize: titleFontSize),
                      ),
                      centerTitle: false,
                    ),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    final double textWidth = width;
    // The title is scaled and transformed to be 1.5 times bigger, when the
    // FlexibleSpaceBar is fully expanded, thus we expect the width to be
    // 1.5 times smaller than the full width. The height of the text is the same
    // as the font size, with 10 dps bottom margin.
    expect(
      tester.getRect(find.byType(Text)),
      rectMoreOrLessEquals(
        Rect.fromLTRB(0, height - titleFontSize - 10, textWidth, height),
        epsilon: 0.0001,
      ),
    );
  });

  testWidgets(
    'Material3 - FlexibleSpaceBar sets constraints for the title - override expandedTitleScale',
    (WidgetTester tester) async {
      const double titleFontSize = 20.0;
      const double height = 300.0;
      const double expandedTitleScale = 3.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: CustomScrollView(
              slivers: <Widget>[
                SliverAppBar(
                  expandedHeight: height,
                  pinned: true,
                  stretch: true,
                  flexibleSpace: FlexibleSpaceBar(
                    expandedTitleScale: expandedTitleScale,
                    titlePadding: EdgeInsets.zero,
                    title: Text(
                      'X' * 41,
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: const TextStyle(fontSize: titleFontSize, height: 1.0),
                    ),
                    centerTitle: false,
                  ),
                ),
                SliverList(
                  delegate: SliverChildListDelegate(<Widget>[
                    for (int i = 0; i < 3; i++)
                      SizedBox(height: 200.0, child: Center(child: Text('Item $i'))),
                  ]),
                ),
              ],
            ),
          ),
        ),
      );

      // We drag up to fully collapse the space bar.
      await tester.drag(find.text('Item 0'), const Offset(0, -600.0));
      await tester.pumpAndSettle();

      final Finder title = find.byType(Text).first;
      final double collapsedWidth = tester.getRect(title).width;

      // We drag down to fully expand the space bar.
      await tester.drag(find.text('Item 2'), const Offset(0, 600.0));
      await tester.pumpAndSettle();

      // The title is shifted by this margin to maintain the position of the
      // bottom edge.
      const double bottomMargin = titleFontSize * (expandedTitleScale - 1);

      final double textWidth = collapsedWidth;
      // The title is scaled and transformed to be 3 times bigger, when the
      // FlexibleSpaceBar is fully expanded, thus we expect the width to be
      // 3 times smaller than the full width. The height of the text is the same
      // as the font size, with 40 dps bottom margin to maintain its bottom position.
      expect(
        tester.getRect(title),
        rectMoreOrLessEquals(
          Rect.fromLTRB(0, height - titleFontSize - bottomMargin, textWidth, height),
          epsilon: 0.0001,
        ),
      );
    },
  );

  testWidgets(
    'Material2 - FlexibleSpaceBar sets constraints for the title - override expandedTitleScale',
    (WidgetTester tester) async {
      const double titleFontSize = 20.0;
      const double height = 300.0;
      const double expandedTitleScale = 3.0;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: CustomScrollView(
              slivers: <Widget>[
                SliverAppBar(
                  expandedHeight: height,
                  pinned: true,
                  stretch: true,
                  flexibleSpace: FlexibleSpaceBar(
                    expandedTitleScale: expandedTitleScale,
                    titlePadding: EdgeInsets.zero,
                    title: Text(
                      'X' * 41,
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: const TextStyle(fontSize: titleFontSize),
                    ),
                    centerTitle: false,
                  ),
                ),
                SliverList(
                  delegate: SliverChildListDelegate(<Widget>[
                    for (int i = 0; i < 3; i++)
                      SizedBox(height: 200.0, child: Center(child: Text('Item $i'))),
                  ]),
                ),
              ],
            ),
          ),
        ),
      );

      // We drag up to fully collapse the space bar.
      await tester.drag(find.text('Item 0'), const Offset(0, -600.0));
      await tester.pumpAndSettle();

      final Finder title = find.byType(Text).first;
      final double collapsedWidth = tester.getRect(title).width;

      // We drag down to fully expand the space bar.
      await tester.drag(find.text('Item 2'), const Offset(0, 600.0));
      await tester.pumpAndSettle();

      // The title is shifted by this margin to maintain the position of the
      // bottom edge.
      const double bottomMargin = titleFontSize * (expandedTitleScale - 1);

      final double textWidth = collapsedWidth;
      // The title is scaled and transformed to be 3 times bigger, when the
      // FlexibleSpaceBar is fully expanded, thus we expect the width to be
      // 3 times smaller than the full width. The height of the text is the same
      // as the font size, with 40 dps bottom margin to maintain its bottom position.
      expect(
        tester.getRect(title),
        rectMoreOrLessEquals(
          Rect.fromLTRB(0, height - titleFontSize - bottomMargin, textWidth, height),
          epsilon: 0.0001,
        ),
      );
    },
  );

  testWidgets('Material3 - FlexibleSpaceBar scaled title', (WidgetTester tester) async {
    const double titleFontSize = 20.0;
    const double height = 300.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              const SliverAppBar(
                expandedHeight: height,
                pinned: true,
                stretch: true,
                flexibleSpace: RepaintBoundary(
                  child: FlexibleSpaceBar(
                    title: Text('X', style: TextStyle(fontSize: titleFontSize)),
                    centerTitle: false,
                  ),
                ),
              ),
              SliverList(
                delegate: SliverChildListDelegate(<Widget>[
                  for (int i = 0; i < 3; i += 1)
                    SizedBox(height: 200.0, child: Center(child: Text('Item $i'))),
                ]),
              ),
            ],
          ),
        ),
      ),
    );

    // We drag up to fully collapse the space bar.
    await tester.drag(find.text('Item 0'), const Offset(0, -600.0));
    await tester.pumpAndSettle();

    final Finder flexibleSpaceBar = find.ancestor(
      of: find.byType(FlexibleSpaceBar),
      matching: find.byType(RepaintBoundary).first,
    );
    await expectLater(
      flexibleSpaceBar,
      matchesGoldenFile('flexible_space_bar.m3.expanded_title_scale_default.collapsed.png'),
    );

    // We drag down to fully expand the space bar.
    await tester.drag(find.text('Item 2'), const Offset(0, 600.0));
    await tester.pumpAndSettle();

    await expectLater(
      flexibleSpaceBar,
      matchesGoldenFile('flexible_space_bar.m3.expanded_title_scale_default.expanded.png'),
    );
  });

  testWidgets('Material2 - FlexibleSpaceBar scaled title', (WidgetTester tester) async {
    const double titleFontSize = 20.0;
    const double height = 300.0;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              const SliverAppBar(
                expandedHeight: height,
                pinned: true,
                stretch: true,
                flexibleSpace: RepaintBoundary(
                  child: FlexibleSpaceBar(
                    title: Text('X', style: TextStyle(fontSize: titleFontSize)),
                    centerTitle: false,
                  ),
                ),
              ),
              SliverList(
                delegate: SliverChildListDelegate(<Widget>[
                  for (int i = 0; i < 3; i += 1)
                    SizedBox(height: 200.0, child: Center(child: Text('Item $i'))),
                ]),
              ),
            ],
          ),
        ),
      ),
    );

    // We drag up to fully collapse the space bar.
    await tester.drag(find.text('Item 0'), const Offset(0, -600.0));
    await tester.pumpAndSettle();

    final Finder flexibleSpaceBar = find.ancestor(
      of: find.byType(FlexibleSpaceBar),
      matching: find.byType(RepaintBoundary).first,
    );
    await expectLater(
      flexibleSpaceBar,
      matchesGoldenFile('flexible_space_bar.m2.expanded_title_scale_default.collapsed.png'),
    );

    // We drag down to fully expand the space bar.
    await tester.drag(find.text('Item 2'), const Offset(0, 600.0));
    await tester.pumpAndSettle();

    await expectLater(
      flexibleSpaceBar,
      matchesGoldenFile('flexible_space_bar.m2.expanded_title_scale_default.expanded.png'),
    );
  });

  testWidgets('Material3 - FlexibleSpaceBar scaled title - override expandedTitleScale', (
    WidgetTester tester,
  ) async {
    const double titleFontSize = 20.0;
    const double height = 300.0;
    const double expandedTitleScale = 3.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              const SliverAppBar(
                expandedHeight: height,
                pinned: true,
                stretch: true,
                flexibleSpace: RepaintBoundary(
                  child: FlexibleSpaceBar(
                    title: Text('X', style: TextStyle(fontSize: titleFontSize)),
                    centerTitle: false,
                    expandedTitleScale: expandedTitleScale,
                  ),
                ),
              ),
              SliverList(
                delegate: SliverChildListDelegate(<Widget>[
                  for (int i = 0; i < 3; i += 1)
                    SizedBox(height: 200.0, child: Center(child: Text('Item $i'))),
                ]),
              ),
            ],
          ),
        ),
      ),
    );

    // We drag up to fully collapse the space bar.
    await tester.drag(find.text('Item 0'), const Offset(0, -600.0));
    await tester.pumpAndSettle();

    final Finder flexibleSpaceBar = find.ancestor(
      of: find.byType(FlexibleSpaceBar),
      matching: find.byType(RepaintBoundary).first,
    );
    // This should match the default behavior
    await expectLater(
      flexibleSpaceBar,
      matchesGoldenFile('flexible_space_bar.m3.expanded_title_scale_default.collapsed.png'),
    );

    // We drag down to fully expand the space bar.
    await tester.drag(find.text('Item 2'), const Offset(0, 600.0));
    await tester.pumpAndSettle();

    await expectLater(
      flexibleSpaceBar,
      matchesGoldenFile('flexible_space_bar.m3.expanded_title_scale_override.expanded.png'),
    );
  });

  testWidgets('Material2 - FlexibleSpaceBar scaled title - override expandedTitleScale', (
    WidgetTester tester,
  ) async {
    const double titleFontSize = 20.0;
    const double height = 300.0;
    const double expandedTitleScale = 3.0;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              const SliverAppBar(
                expandedHeight: height,
                pinned: true,
                stretch: true,
                flexibleSpace: RepaintBoundary(
                  child: FlexibleSpaceBar(
                    title: Text('X', style: TextStyle(fontSize: titleFontSize)),
                    centerTitle: false,
                    expandedTitleScale: expandedTitleScale,
                  ),
                ),
              ),
              SliverList(
                delegate: SliverChildListDelegate(<Widget>[
                  for (int i = 0; i < 3; i += 1)
                    SizedBox(height: 200.0, child: Center(child: Text('Item $i'))),
                ]),
              ),
            ],
          ),
        ),
      ),
    );

    // We drag up to fully collapse the space bar.
    await tester.drag(find.text('Item 0'), const Offset(0, -600.0));
    await tester.pumpAndSettle();

    final Finder flexibleSpaceBar = find.ancestor(
      of: find.byType(FlexibleSpaceBar),
      matching: find.byType(RepaintBoundary).first,
    );
    // This should match the default behavior
    await expectLater(
      flexibleSpaceBar,
      matchesGoldenFile('flexible_space_bar.m2.expanded_title_scale_default.collapsed.png'),
    );

    // We drag down to fully expand the space bar.
    await tester.drag(find.text('Item 2'), const Offset(0, 600.0));
    await tester.pumpAndSettle();

    await expectLater(
      flexibleSpaceBar,
      matchesGoldenFile('flexible_space_bar.m2.expanded_title_scale_override.expanded.png'),
    );
  });

  testWidgets('Material3 - FlexibleSpaceBar titlePadding defaults', (WidgetTester tester) async {
    Widget buildFrame(TargetPlatform platform, bool? centerTitle) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Scaffold(
          appBar: AppBar(
            flexibleSpace: FlexibleSpaceBar(centerTitle: centerTitle, title: const Text('X')),
          ),
        ),
      );
    }

    final Finder title = find.text('X');
    final Finder flexibleSpaceBar = find.byType(FlexibleSpaceBar);
    Offset getTitleBottomLeft() {
      return Offset(
        tester.getTopLeft(title).dx,
        tester.getBottomRight(flexibleSpaceBar).dy - tester.getBottomRight(title).dy,
      );
    }

    await tester.pumpWidget(buildFrame(TargetPlatform.android, null));
    expect(getTitleBottomLeft(), const Offset(72.0, 16.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.android, true));
    expect(getTitleBottomLeft(), const Offset(389.0, 16.0));

    // Clear the widget tree to avoid animating between Android and iOS.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.iOS, null));
    expect(getTitleBottomLeft(), const Offset(389.0, 16.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.iOS, false));
    expect(getTitleBottomLeft(), const Offset(72.0, 16.0));

    // Clear the widget tree to avoid animating between iOS and macOS.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.macOS, null));
    expect(getTitleBottomLeft(), const Offset(389.0, 16.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.macOS, false));
    expect(getTitleBottomLeft(), const Offset(72.0, 16.0));
  });

  testWidgets('Material2 - FlexibleSpaceBar titlePadding defaults', (WidgetTester tester) async {
    Widget buildFrame(TargetPlatform platform, bool? centerTitle) {
      return MaterialApp(
        theme: ThemeData(platform: platform, useMaterial3: false),
        home: Scaffold(
          appBar: AppBar(
            flexibleSpace: FlexibleSpaceBar(centerTitle: centerTitle, title: const Text('X')),
          ),
        ),
      );
    }

    final Finder title = find.text('X');
    final Finder flexibleSpaceBar = find.byType(FlexibleSpaceBar);
    Offset getTitleBottomLeft() {
      return Offset(
        tester.getTopLeft(title).dx,
        tester.getBottomRight(flexibleSpaceBar).dy - tester.getBottomRight(title).dy,
      );
    }

    await tester.pumpWidget(buildFrame(TargetPlatform.android, null));
    expect(getTitleBottomLeft(), const Offset(72.0, 16.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.android, true));
    expect(getTitleBottomLeft(), const Offset(390.0, 16.0));

    // Clear the widget tree to avoid animating between Android and iOS.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.iOS, null));
    expect(getTitleBottomLeft(), const Offset(390.0, 16.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.iOS, false));
    expect(getTitleBottomLeft(), const Offset(72.0, 16.0));

    // Clear the widget tree to avoid animating between iOS and macOS.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.macOS, null));
    expect(getTitleBottomLeft(), const Offset(390.0, 16.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.macOS, false));
    expect(getTitleBottomLeft(), const Offset(72.0, 16.0));
  });

  testWidgets('Material3 - FlexibleSpaceBar titlePadding override', (WidgetTester tester) async {
    Widget buildFrame(TargetPlatform platform, bool? centerTitle) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Scaffold(
          appBar: AppBar(
            flexibleSpace: FlexibleSpaceBar(
              titlePadding: EdgeInsets.zero,
              centerTitle: centerTitle,
              title: const Text('X'),
            ),
          ),
        ),
      );
    }

    final Finder title = find.text('X');
    final Finder flexibleSpaceBar = find.byType(FlexibleSpaceBar);
    Offset getTitleBottomLeft() {
      return Offset(
        tester.getTopLeft(title).dx,
        tester.getBottomRight(flexibleSpaceBar).dy - tester.getBottomRight(title).dy,
      );
    }

    await tester.pumpWidget(buildFrame(TargetPlatform.android, null));
    expect(getTitleBottomLeft(), Offset.zero);

    await tester.pumpWidget(buildFrame(TargetPlatform.android, true));
    expect(getTitleBottomLeft(), const Offset(389.0, 0.0));

    // Clear the widget tree to avoid animating between platforms.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.iOS, null));
    expect(getTitleBottomLeft(), const Offset(389.0, 0.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.iOS, false));
    expect(getTitleBottomLeft(), Offset.zero);

    // Clear the widget tree to avoid animating between platforms.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.macOS, null));
    expect(getTitleBottomLeft(), const Offset(389.0, 0.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.macOS, false));
    expect(getTitleBottomLeft(), Offset.zero);

    // Clear the widget tree to avoid animating between platforms.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.windows, null));
    expect(getTitleBottomLeft(), Offset.zero);

    await tester.pumpWidget(buildFrame(TargetPlatform.windows, true));
    expect(getTitleBottomLeft(), const Offset(389.0, 0.0));

    // Clear the widget tree to avoid animating between platforms.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.linux, null));
    expect(getTitleBottomLeft(), Offset.zero);

    await tester.pumpWidget(buildFrame(TargetPlatform.linux, true));
    expect(getTitleBottomLeft(), const Offset(389.0, 0.0));
  });

  testWidgets('Material2 - FlexibleSpaceBar titlePadding override', (WidgetTester tester) async {
    Widget buildFrame(TargetPlatform platform, bool? centerTitle) {
      return MaterialApp(
        theme: ThemeData(platform: platform, useMaterial3: false),
        home: Scaffold(
          appBar: AppBar(
            flexibleSpace: FlexibleSpaceBar(
              titlePadding: EdgeInsets.zero,
              centerTitle: centerTitle,
              title: const Text('X'),
            ),
          ),
        ),
      );
    }

    final Finder title = find.text('X');
    final Finder flexibleSpaceBar = find.byType(FlexibleSpaceBar);
    Offset getTitleBottomLeft() {
      return Offset(
        tester.getTopLeft(title).dx,
        tester.getBottomRight(flexibleSpaceBar).dy - tester.getBottomRight(title).dy,
      );
    }

    await tester.pumpWidget(buildFrame(TargetPlatform.android, null));
    expect(getTitleBottomLeft(), Offset.zero);

    await tester.pumpWidget(buildFrame(TargetPlatform.android, true));
    expect(getTitleBottomLeft(), const Offset(390.0, 0.0));

    // Clear the widget tree to avoid animating between platforms.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.iOS, null));
    expect(getTitleBottomLeft(), const Offset(390.0, 0.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.iOS, false));
    expect(getTitleBottomLeft(), Offset.zero);

    // Clear the widget tree to avoid animating between platforms.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.macOS, null));
    expect(getTitleBottomLeft(), const Offset(390.0, 0.0));

    await tester.pumpWidget(buildFrame(TargetPlatform.macOS, false));
    expect(getTitleBottomLeft(), Offset.zero);

    // Clear the widget tree to avoid animating between platforms.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.windows, null));
    expect(getTitleBottomLeft(), Offset.zero);

    await tester.pumpWidget(buildFrame(TargetPlatform.windows, true));
    expect(getTitleBottomLeft(), const Offset(390.0, 0.0));

    // Clear the widget tree to avoid animating between platforms.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildFrame(TargetPlatform.linux, null));
    expect(getTitleBottomLeft(), Offset.zero);

    await tester.pumpWidget(buildFrame(TargetPlatform.linux, true));
    expect(getTitleBottomLeft(), const Offset(390.0, 0.0));
  });

  testWidgets('FlexibleSpaceBar rebuilds when scrolling.', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: SubCategoryScreenView()));

    expect(RenderRebuildTracker.count, 1);
    expect(
      tester.layers.lastWhere((Layer element) => element is OpacityLayer),
      isA<OpacityLayer>().having((OpacityLayer p0) => p0.alpha, 'alpha', 255),
    );

    // We drag up to fully collapse the space bar.
    for (int i = 0; i < 9; i++) {
      await tester.drag(find.byKey(SubCategoryScreenView.scrollKey), const Offset(0, -50.0));
      await tester.pumpAndSettle();
    }

    expect(
      tester.layers.lastWhere((Layer element) => element is OpacityLayer),
      isA<OpacityLayer>().having((OpacityLayer p0) => p0.alpha, 'alpha', lessThan(255)),
    );

    for (int i = 0; i < 11; i++) {
      await tester.drag(find.byKey(SubCategoryScreenView.scrollKey), const Offset(0, -50.0));
      await tester.pumpAndSettle();
    }

    expect(RenderRebuildTracker.count, greaterThan(1));
    expect(tester.layers.whereType<OpacityLayer>(), isEmpty);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/132030.
  testWidgets('FlexibleSpaceBarSettings.hasLeading provides a gap between leading and title', (
    WidgetTester tester,
  ) async {
    final FlexibleSpaceBarSettings customSettings =
        FlexibleSpaceBar.createSettings(
              currentExtent: 200.0,
              hasLeading: true,
              child: AppBar(
                leading: const Icon(Icons.menu),
                flexibleSpace: FlexibleSpaceBar(
                  title: Text('title ' * 10),
                  // Set centerTitle to false to create a gap between the leading widget
                  // and the long title.
                  centerTitle: false,
                ),
              ),
            )
            as FlexibleSpaceBarSettings;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            slivers: <Widget>[
              SliverPersistentHeader(
                floating: true,
                pinned: true,
                delegate: TestDelegate(settings: customSettings),
              ),
              SliverToBoxAdapter(child: Container(height: 1200.0, color: Colors.orange[400])),
            ],
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();
    expect(tester.getTopLeft(find.byType(Text)).dx, closeTo(72.0, 0.01));
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/135698.
  testWidgets(
    '_FlexibleSpaceHeaderOpacity with near zero opacity avoids compositing',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: NestedScrollView(
              headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
                return <Widget>[
                  SliverOverlapAbsorber(
                    handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                    sliver: const SliverAppBar(
                      pinned: true,
                      expandedHeight: 200.0,
                      collapsedHeight: 56.0,
                      flexibleSpace: FlexibleSpaceBar(background: SizedBox()),
                    ),
                  ),
                ];
              },
              body: const SingleChildScrollView(
                child: Column(children: <Widget>[Placeholder(fallbackHeight: 300.0)]),
              ),
            ),
          ),
        ),
      );

      // Drag the scroll view to the top to collapse the sliver app bar.
      // Ensure collapsed height - current extent is near zero for the
      // FlexibleSpaceBar to avoid compositing.
      await tester.drag(
        find.byType(SingleChildScrollView),
        const Offset(0, -(200.0 - 56.08787892026129)),
      );
      await tester.pumpAndSettle();

      expect(tester.takeException(), isNull);
    },
    variant: TargetPlatformVariant.mobile(),
  );

  // This is a regression test for https://github.com/flutter/flutter/issues/138608.
  testWidgets('FlexibleSpaceBar centers title with a leading widget', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: CustomScrollView(
            slivers: <Widget>[
              SliverAppBar(
                leading: Icon(Icons.menu),
                flexibleSpace: FlexibleSpaceBar(centerTitle: true, title: Text('X')),
              ),
            ],
          ),
        ),
      ),
    );

    final Offset appBarCenter = tester.getCenter(find.byType(AppBar));
    final Offset titleCenter = tester.getCenter(find.text('X'));
    expect(appBarCenter.dx, titleCenter.dx);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/138296.
  testWidgets('Material3 - Default title color', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme, // Provide the expected theme data.
        home: const Material(
          child: CustomScrollView(
            slivers: <Widget>[SliverAppBar(flexibleSpace: FlexibleSpaceBar(title: Text('Title')))],
          ),
        ),
      ),
    );

    final DefaultTextStyle textStyle = DefaultTextStyle.of(tester.element(find.text('Title')));
    expect(textStyle.style.color, theme.textTheme.titleLarge!.color);
  });
}

class TestDelegate extends SliverPersistentHeaderDelegate {
  const TestDelegate({required this.settings});

  final FlexibleSpaceBarSettings settings;

  @override
  double get maxExtent => settings.maxExtent;

  @override
  double get minExtent => settings.minExtent;

  @override
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent) {
    return settings;
  }

  @override
  bool shouldRebuild(TestDelegate oldDelegate) => false;
}

class RebuildTracker extends SingleChildRenderObjectWidget {
  const RebuildTracker({super.key});

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderRebuildTracker();
  }
}

class RenderRebuildTracker extends RenderProxyBox {
  static int count = 0;

  @override
  void paint(PaintingContext context, Offset offset) {
    count++;
    super.paint(context, offset);
  }
}

class SubCategoryScreenView extends StatefulWidget {
  const SubCategoryScreenView({super.key});

  static const Key scrollKey = Key('orange box');

  @override
  State<SubCategoryScreenView> createState() => _SubCategoryScreenViewState();
}

class _SubCategoryScreenViewState extends State<SubCategoryScreenView>
    with TickerProviderStateMixin {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(centerTitle: true, title: const Text('Test')),
      body: CustomScrollView(
        key: SubCategoryScreenView.scrollKey,
        slivers: <Widget>[
          SliverAppBar(
            leading: const SizedBox(),
            expandedHeight: MediaQuery.of(context).size.width / 1.7,
            collapsedHeight: 0,
            toolbarHeight: 0,
            titleSpacing: 0,
            leadingWidth: 0,
            flexibleSpace: const FlexibleSpaceBar(
              background: AspectRatio(aspectRatio: 1.7, child: RebuildTracker()),
            ),
          ),
          const SliverToBoxAdapter(child: SizedBox(height: 12)),
          SliverGrid.builder(
            gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
            itemCount: 300,
            itemBuilder: (BuildContext context, int index) {
              return Card(color: Colors.amber, child: Center(child: Text('$index')));
            },
          ),
          const SliverToBoxAdapter(child: SizedBox(height: 12)),
        ],
      ),
    );
  }
}
