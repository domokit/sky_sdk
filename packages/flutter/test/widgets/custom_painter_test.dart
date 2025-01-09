// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  group(CustomPainter, () {
    setUp(() {
      debugResetSemanticsIdCounter();
      _PainterWithSemantics.shouldRebuildSemanticsCallCount = 0;
      _PainterWithSemantics.buildSemanticsCallCount = 0;
      _PainterWithSemantics.semanticsBuilderCallCount = 0;
    });

    _defineTests();
  });
}

void _defineTests() {
  testWidgets('builds no semantics by default', (WidgetTester tester) async {
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    await tester.pumpWidget(CustomPaint(painter: _PainterWithoutSemantics()));

    expect(semanticsTester, hasSemantics(TestSemantics.root()));

    semanticsTester.dispose();
  });

  testWidgets('provides foreground semantics', (WidgetTester tester) async {
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    await tester.pumpWidget(
      CustomPaint(
        foregroundPainter: _PainterWithSemantics(
          semantics: const CustomPainterSemantics(
            rect: Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
            properties: SemanticsProperties(label: 'foreground', textDirection: TextDirection.rtl),
          ),
        ),
      ),
    );

    expect(
      semanticsTester,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  label: 'foreground',
                  rect: const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
                ),
              ],
            ),
          ],
        ),
      ),
    );

    semanticsTester.dispose();
  });

  testWidgets('provides background semantics', (WidgetTester tester) async {
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    await tester.pumpWidget(
      CustomPaint(
        painter: _PainterWithSemantics(
          semantics: const CustomPainterSemantics(
            rect: Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
            properties: SemanticsProperties(label: 'background', textDirection: TextDirection.rtl),
          ),
        ),
      ),
    );

    expect(
      semanticsTester,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  label: 'background',
                  rect: const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
                ),
              ],
            ),
          ],
        ),
      ),
    );

    semanticsTester.dispose();
  });

  testWidgets('combines background, child and foreground semantics', (WidgetTester tester) async {
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    await tester.pumpWidget(
      CustomPaint(
        painter: _PainterWithSemantics(
          semantics: const CustomPainterSemantics(
            rect: Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
            properties: SemanticsProperties(label: 'background', textDirection: TextDirection.rtl),
          ),
        ),
        foregroundPainter: _PainterWithSemantics(
          semantics: const CustomPainterSemantics(
            rect: Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
            properties: SemanticsProperties(label: 'foreground', textDirection: TextDirection.rtl),
          ),
        ),
        child: Semantics(
          container: true,
          child: const Text('Hello', textDirection: TextDirection.ltr),
        ),
      ),
    );

    expect(
      semanticsTester,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  id: 3,
                  label: 'background',
                  rect: const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
                ),
                TestSemantics(
                  id: 2,
                  label: 'Hello',
                  rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
                ),
                TestSemantics(
                  id: 4,
                  label: 'foreground',
                  rect: const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
                ),
              ],
            ),
          ],
        ),
      ),
    );

    semanticsTester.dispose();
  });

  testWidgets('applies $SemanticsProperties', (WidgetTester tester) async {
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    await tester.pumpWidget(
      CustomPaint(
        painter: _PainterWithSemantics(
          semantics: const CustomPainterSemantics(
            key: ValueKey<int>(1),
            rect: Rect.fromLTRB(1.0, 2.0, 3.0, 4.0),
            properties: SemanticsProperties(
              checked: false,
              selected: false,
              button: false,
              label: 'label-before',
              value: 'value-before',
              increasedValue: 'increase-before',
              decreasedValue: 'decrease-before',
              hint: 'hint-before',
              textDirection: TextDirection.rtl,
            ),
          ),
        ),
      ),
    );

    expect(
      semanticsTester,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  rect: const Rect.fromLTRB(1.0, 2.0, 3.0, 4.0),
                  id: 2,
                  flags: <SemanticsFlag>[
                    SemanticsFlag.hasCheckedState,
                    SemanticsFlag.hasSelectedState,
                  ],
                  label: 'label-before',
                  value: 'value-before',
                  increasedValue: 'increase-before',
                  decreasedValue: 'decrease-before',
                  hint: 'hint-before',
                  textDirection: TextDirection.rtl,
                ),
              ],
            ),
          ],
        ),
      ),
    );

    await tester.pumpWidget(
      CustomPaint(
        painter: _PainterWithSemantics(
          semantics: CustomPainterSemantics(
            key: const ValueKey<int>(1),
            rect: const Rect.fromLTRB(5.0, 6.0, 7.0, 8.0),
            properties: SemanticsProperties(
              checked: true,
              selected: true,
              button: true,
              label: 'label-after',
              value: 'value-after',
              increasedValue: 'increase-after',
              decreasedValue: 'decrease-after',
              hint: 'hint-after',
              textDirection: TextDirection.ltr,
              onScrollDown: () {},
              onLongPress: () {},
              onDecrease: () {},
              onIncrease: () {},
              onScrollLeft: () {},
              onScrollRight: () {},
              onScrollUp: () {},
              onTap: () {},
            ),
          ),
        ),
      ),
    );

    expect(
      semanticsTester,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              rect: TestSemantics.fullScreen,
              children: <TestSemantics>[
                TestSemantics(
                  rect: const Rect.fromLTRB(5.0, 6.0, 7.0, 8.0),
                  actions: 255,
                  id: 2,
                  flags: <SemanticsFlag>[
                    SemanticsFlag.hasCheckedState,
                    SemanticsFlag.isChecked,
                    SemanticsFlag.hasSelectedState,
                    SemanticsFlag.isSelected,
                    SemanticsFlag.isButton,
                  ],
                  label: 'label-after',
                  value: 'value-after',
                  increasedValue: 'increase-after',
                  decreasedValue: 'decrease-after',
                  hint: 'hint-after',
                  textDirection: TextDirection.ltr,
                ),
              ],
            ),
          ],
        ),
      ),
    );

    semanticsTester.dispose();
  });

  testWidgets('Can toggle semantics on, off, on without crash', (WidgetTester tester) async {
    await tester.pumpWidget(
      CustomPaint(
        painter: _PainterWithSemantics(
          semantics: const CustomPainterSemantics(
            key: ValueKey<int>(1),
            rect: Rect.fromLTRB(1.0, 2.0, 3.0, 4.0),
            properties: SemanticsProperties(
              checked: false,
              selected: false,
              button: false,
              label: 'label-before',
              value: 'value-before',
              increasedValue: 'increase-before',
              decreasedValue: 'decrease-before',
              hint: 'hint-before',
              textDirection: TextDirection.rtl,
            ),
          ),
        ),
      ),
    );

    // Start with semantics off.
    expect(tester.binding.pipelineOwner.semanticsOwner, isNull);

    // Semantics on
    SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpAndSettle();
    expect(tester.binding.pipelineOwner.semanticsOwner, isNotNull);

    // Semantics off
    semantics.dispose();
    await tester.pumpAndSettle();
    expect(tester.binding.pipelineOwner.semanticsOwner, isNull);

    // Semantics on
    semantics = SemanticsTester(tester);
    await tester.pumpAndSettle();
    expect(tester.binding.pipelineOwner.semanticsOwner, isNotNull);

    semantics.dispose();
  }, semanticsEnabled: false);

  testWidgets('Supports all actions', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final List<SemanticsAction> performedActions = <SemanticsAction>[];

    await tester.pumpWidget(
      CustomPaint(
        painter: _PainterWithSemantics(
          semantics: CustomPainterSemantics(
            key: const ValueKey<int>(1),
            rect: const Rect.fromLTRB(1.0, 2.0, 3.0, 4.0),
            properties: SemanticsProperties(
              onDismiss: () => performedActions.add(SemanticsAction.dismiss),
              onTap: () => performedActions.add(SemanticsAction.tap),
              onLongPress: () => performedActions.add(SemanticsAction.longPress),
              onScrollLeft: () => performedActions.add(SemanticsAction.scrollLeft),
              onScrollRight: () => performedActions.add(SemanticsAction.scrollRight),
              onScrollUp: () => performedActions.add(SemanticsAction.scrollUp),
              onScrollDown: () => performedActions.add(SemanticsAction.scrollDown),
              onIncrease: () => performedActions.add(SemanticsAction.increase),
              onDecrease: () => performedActions.add(SemanticsAction.decrease),
              onCopy: () => performedActions.add(SemanticsAction.copy),
              onCut: () => performedActions.add(SemanticsAction.cut),
              onPaste: () => performedActions.add(SemanticsAction.paste),
              onMoveCursorForwardByCharacter:
                  (bool _) => performedActions.add(SemanticsAction.moveCursorForwardByCharacter),
              onMoveCursorBackwardByCharacter:
                  (bool _) => performedActions.add(SemanticsAction.moveCursorBackwardByCharacter),
              onMoveCursorForwardByWord:
                  (bool _) => performedActions.add(SemanticsAction.moveCursorForwardByWord),
              onMoveCursorBackwardByWord:
                  (bool _) => performedActions.add(SemanticsAction.moveCursorBackwardByWord),
              onSetSelection:
                  (TextSelection _) => performedActions.add(SemanticsAction.setSelection),
              onSetText: (String text) => performedActions.add(SemanticsAction.setText),
              onDidGainAccessibilityFocus:
                  () => performedActions.add(SemanticsAction.didGainAccessibilityFocus),
              onDidLoseAccessibilityFocus:
                  () => performedActions.add(SemanticsAction.didLoseAccessibilityFocus),
              onFocus: () => performedActions.add(SemanticsAction.focus),
            ),
          ),
        ),
      ),
    );
    final Set<SemanticsAction> allActions =
        SemanticsAction.values.toSet()
          ..remove(SemanticsAction.customAction) // customAction is not user-exposed.
          ..remove(SemanticsAction.showOnScreen) // showOnScreen is not user-exposed
          ..remove(SemanticsAction.scrollToOffset); // scrollToOffset is not user-exposed

    const int expectedId = 2;
    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          id: 1,
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: expectedId,
              rect: TestSemantics.fullScreen,
              actions: allActions.fold<int>(
                0,
                (int previous, SemanticsAction action) => previous | action.index,
              ),
            ),
          ],
        ),
      ],
    );
    expect(semantics, hasSemantics(expectedSemantics, ignoreRect: true, ignoreTransform: true));

    // Do the actions work?
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    int expectedLength = 1;
    for (final SemanticsAction action in allActions) {
      switch (action) {
        case SemanticsAction.moveCursorBackwardByCharacter:
        case SemanticsAction.moveCursorForwardByCharacter:
        case SemanticsAction.moveCursorBackwardByWord:
        case SemanticsAction.moveCursorForwardByWord:
          semanticsOwner.performAction(expectedId, action, true);
        case SemanticsAction.setSelection:
          semanticsOwner.performAction(expectedId, action, <String, int>{'base': 4, 'extent': 5});
        case SemanticsAction.setText:
          semanticsOwner.performAction(expectedId, action, 'text');
        case SemanticsAction.copy:
        case SemanticsAction.customAction:
        case SemanticsAction.cut:
        case SemanticsAction.decrease:
        case SemanticsAction.didGainAccessibilityFocus:
        case SemanticsAction.didLoseAccessibilityFocus:
        case SemanticsAction.dismiss:
        case SemanticsAction.increase:
        case SemanticsAction.longPress:
        case SemanticsAction.paste:
        case SemanticsAction.scrollDown:
        case SemanticsAction.scrollLeft:
        case SemanticsAction.scrollRight:
        case SemanticsAction.scrollUp:
        case SemanticsAction.scrollToOffset:
        case SemanticsAction.showOnScreen:
        case SemanticsAction.tap:
        case SemanticsAction.focus:
          semanticsOwner.performAction(expectedId, action);
      }
      expect(performedActions.length, expectedLength);
      expect(performedActions.last, action);
      expectedLength += 1;
    }

    semantics.dispose();
  });

  testWidgets('Supports all flags', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    // checked state and toggled state are mutually exclusive.
    await tester.pumpWidget(
      CustomPaint(
        painter: _PainterWithSemantics(
          semantics: const CustomPainterSemantics(
            key: ValueKey<int>(1),
            rect: Rect.fromLTRB(1.0, 2.0, 3.0, 4.0),
            properties: SemanticsProperties(
              enabled: true,
              checked: true,
              selected: true,
              hidden: true,
              button: true,
              slider: true,
              keyboardKey: true,
              link: true,
              textField: true,
              readOnly: true,
              focused: true,
              focusable: true,
              inMutuallyExclusiveGroup: true,
              header: true,
              obscured: true,
              multiline: true,
              scopesRoute: true,
              namesRoute: true,
              image: true,
              liveRegion: true,
              toggled: true,
              expanded: true,
            ),
          ),
        ),
      ),
    );
    List<SemanticsFlag> flags = SemanticsFlag.values.toList();
    // [SemanticsFlag.hasImplicitScrolling] isn't part of [SemanticsProperties]
    // therefore it has to be removed.
    flags
      ..remove(SemanticsFlag.hasImplicitScrolling)
      ..remove(SemanticsFlag.isCheckStateMixed);
    TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          id: 1,
          children: <TestSemantics>[
            TestSemantics.rootChild(id: 2, rect: TestSemantics.fullScreen, flags: flags),
          ],
        ),
      ],
    );
    expect(semantics, hasSemantics(expectedSemantics, ignoreRect: true, ignoreTransform: true));

    await tester.pumpWidget(
      CustomPaint(
        painter: _PainterWithSemantics(
          semantics: const CustomPainterSemantics(
            key: ValueKey<int>(1),
            rect: Rect.fromLTRB(1.0, 2.0, 3.0, 4.0),
            properties: SemanticsProperties(
              enabled: true,
              checked: false,
              mixed: true,
              toggled: true,
              selected: true,
              hidden: true,
              button: true,
              slider: true,
              keyboardKey: true,
              link: true,
              textField: true,
              readOnly: true,
              focused: true,
              focusable: true,
              inMutuallyExclusiveGroup: true,
              header: true,
              obscured: true,
              multiline: true,
              scopesRoute: true,
              namesRoute: true,
              image: true,
              liveRegion: true,
              expanded: true,
            ),
          ),
        ),
      ),
    );
    flags = SemanticsFlag.values.toList();
    // [SemanticsFlag.hasImplicitScrolling] isn't part of [SemanticsProperties]
    // therefore it has to be removed.
    flags
      ..remove(SemanticsFlag.hasImplicitScrolling)
      ..remove(SemanticsFlag.isChecked);
    expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics.rootChild(
          id: 1,
          children: <TestSemantics>[
            TestSemantics.rootChild(id: 2, rect: TestSemantics.fullScreen, flags: flags),
          ],
        ),
      ],
    );
    expect(semantics, hasSemantics(expectedSemantics, ignoreRect: true, ignoreTransform: true));
    semantics.dispose();
  });

  group('diffing', () {
    testWidgets('complains about duplicate keys', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);
      await tester.pumpWidget(CustomPaint(painter: _SemanticsDiffTest(<String>['a-k', 'a-k'])));
      expect(tester.takeException(), isFlutterError);
      semanticsTester.dispose();
    });

    _testDiff('adds one item to an empty list', (_DiffTester tester) async {
      await tester.diff(from: <String>[], to: <String>['a']);
    });

    _testDiff('removes the last item from the list', (_DiffTester tester) async {
      await tester.diff(from: <String>['a'], to: <String>[]);
    });

    _testDiff('appends one item at the end of a non-empty list', (_DiffTester tester) async {
      await tester.diff(from: <String>['a'], to: <String>['a', 'b']);
    });

    _testDiff('prepends one item at the beginning of a non-empty list', (_DiffTester tester) async {
      await tester.diff(from: <String>['b'], to: <String>['a', 'b']);
    });

    _testDiff('inserts one item in the middle of a list', (_DiffTester tester) async {
      await tester.diff(from: <String>['a-k', 'c-k'], to: <String>['a-k', 'b-k', 'c-k']);
    });

    _testDiff('removes one item from the middle of a list', (_DiffTester tester) async {
      await tester.diff(from: <String>['a-k', 'b-k', 'c-k'], to: <String>['a-k', 'c-k']);
    });

    _testDiff('swaps two items', (_DiffTester tester) async {
      await tester.diff(from: <String>['a-k', 'b-k'], to: <String>['b-k', 'a-k']);
    });

    _testDiff('finds and moved one keyed item', (_DiffTester tester) async {
      await tester.diff(from: <String>['a-k', 'b', 'c'], to: <String>['b', 'c', 'a-k']);
    });
  });

  testWidgets('rebuilds semantics upon resize', (WidgetTester tester) async {
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    final _PainterWithSemantics painter = _PainterWithSemantics(
      semantics: const CustomPainterSemantics(
        rect: Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
        properties: SemanticsProperties(label: 'background', textDirection: TextDirection.rtl),
      ),
    );

    final CustomPaint paint = CustomPaint(painter: painter);

    await tester.pumpWidget(SizedBox(height: 20.0, width: 20.0, child: paint));
    expect(_PainterWithSemantics.shouldRebuildSemanticsCallCount, 0);
    expect(_PainterWithSemantics.buildSemanticsCallCount, 1);
    expect(_PainterWithSemantics.semanticsBuilderCallCount, 4);

    await tester.pumpWidget(SizedBox(height: 20.0, width: 20.0, child: paint));
    expect(_PainterWithSemantics.shouldRebuildSemanticsCallCount, 0);
    expect(_PainterWithSemantics.buildSemanticsCallCount, 1);
    expect(_PainterWithSemantics.semanticsBuilderCallCount, 4);

    await tester.pumpWidget(SizedBox(height: 40.0, width: 40.0, child: paint));
    expect(_PainterWithSemantics.shouldRebuildSemanticsCallCount, 0);
    expect(_PainterWithSemantics.buildSemanticsCallCount, 2);
    expect(_PainterWithSemantics.semanticsBuilderCallCount, 4);

    semanticsTester.dispose();
  });

  testWidgets('does not rebuild when shouldRebuildSemantics is false', (WidgetTester tester) async {
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    const CustomPainterSemantics testSemantics = CustomPainterSemantics(
      rect: Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
      properties: SemanticsProperties(label: 'background', textDirection: TextDirection.rtl),
    );

    await tester.pumpWidget(CustomPaint(painter: _PainterWithSemantics(semantics: testSemantics)));
    expect(_PainterWithSemantics.shouldRebuildSemanticsCallCount, 0);
    expect(_PainterWithSemantics.buildSemanticsCallCount, 1);
    expect(_PainterWithSemantics.semanticsBuilderCallCount, 4);

    await tester.pumpWidget(CustomPaint(painter: _PainterWithSemantics(semantics: testSemantics)));
    expect(_PainterWithSemantics.shouldRebuildSemanticsCallCount, 1);
    expect(_PainterWithSemantics.buildSemanticsCallCount, 1);
    expect(_PainterWithSemantics.semanticsBuilderCallCount, 4);

    const CustomPainterSemantics testSemantics2 = CustomPainterSemantics(
      rect: Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
      properties: SemanticsProperties(label: 'background', textDirection: TextDirection.rtl),
    );

    await tester.pumpWidget(CustomPaint(painter: _PainterWithSemantics(semantics: testSemantics2)));
    expect(_PainterWithSemantics.shouldRebuildSemanticsCallCount, 2);
    expect(_PainterWithSemantics.buildSemanticsCallCount, 1);
    expect(_PainterWithSemantics.semanticsBuilderCallCount, 4);

    semanticsTester.dispose();
  });
}

void _testDiff(String description, Future<void> Function(_DiffTester tester) testFunction) {
  testWidgets(description, (WidgetTester tester) async {
    await testFunction(_DiffTester(tester));
  });
}

class _DiffTester {
  _DiffTester(this.tester);

  final WidgetTester tester;

  /// Creates an initial semantics list using the `from` list, then updates the
  /// list to the `to` list. This causes [RenderCustomPaint] to diff the two
  /// lists and apply the changes. This method asserts the changes were
  /// applied correctly, specifically:
  ///
  /// - checks that initial and final configurations are in the desired states.
  /// - checks that keyed nodes have stable IDs.
  Future<void> diff({required List<String> from, required List<String> to}) async {
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    TestSemantics createExpectations(List<String> labels) {
      return TestSemantics.root(
        children: <TestSemantics>[
          TestSemantics.rootChild(
            rect: TestSemantics.fullScreen,
            children: <TestSemantics>[
              for (final String label in labels)
                TestSemantics(rect: const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0), label: label),
            ],
          ),
        ],
      );
    }

    await tester.pumpWidget(CustomPaint(painter: _SemanticsDiffTest(from)));
    expect(semanticsTester, hasSemantics(createExpectations(from), ignoreId: true));

    SemanticsNode root = RendererBinding.instance.renderView.debugSemantics!;
    final Map<Key, int> idAssignments = <Key, int>{};
    root.visitChildren((SemanticsNode firstChild) {
      firstChild.visitChildren((SemanticsNode node) {
        if (node.key != null) {
          idAssignments[node.key!] = node.id;
        }
        return true;
      });
      return true;
    });

    await tester.pumpWidget(CustomPaint(painter: _SemanticsDiffTest(to)));
    await tester.pumpAndSettle();
    expect(semanticsTester, hasSemantics(createExpectations(to), ignoreId: true));

    root = RendererBinding.instance.renderView.debugSemantics!;
    root.visitChildren((SemanticsNode firstChild) {
      firstChild.visitChildren((SemanticsNode node) {
        if (node.key != null && idAssignments[node.key] != null) {
          expect(
            idAssignments[node.key],
            node.id,
            reason:
                'Node with key ${node.key} was previously assigned ID ${idAssignments[node.key]}. '
                'After diffing the child list, its ID changed to ${node.id}. IDs must be stable.',
          );
        }
        return true;
      });
      return true;
    });

    semanticsTester.dispose();
  }
}

class _SemanticsDiffTest extends CustomPainter {
  _SemanticsDiffTest(this.data);

  final List<String> data;

  @override
  void paint(Canvas canvas, Size size) {
    // We don't test painting.
  }

  @override
  SemanticsBuilderCallback get semanticsBuilder => buildSemantics;

  List<CustomPainterSemantics> buildSemantics(Size size) {
    final List<CustomPainterSemantics> semantics = <CustomPainterSemantics>[];
    for (final String label in data) {
      Key? key;
      if (label.endsWith('-k')) {
        key = ValueKey<String>(label);
      }
      semantics.add(
        CustomPainterSemantics(
          rect: const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
          key: key,
          properties: SemanticsProperties(label: label, textDirection: TextDirection.rtl),
        ),
      );
    }
    return semantics;
  }

  @override
  bool shouldRepaint(_SemanticsDiffTest oldPainter) => true;
}

class _PainterWithSemantics extends CustomPainter {
  _PainterWithSemantics({required this.semantics});

  final CustomPainterSemantics semantics;

  static int semanticsBuilderCallCount = 0;
  static int buildSemanticsCallCount = 0;
  static int shouldRebuildSemanticsCallCount = 0;

  @override
  void paint(Canvas canvas, Size size) {
    // We don't test painting.
  }

  @override
  SemanticsBuilderCallback get semanticsBuilder {
    semanticsBuilderCallCount += 1;
    return buildSemantics;
  }

  List<CustomPainterSemantics> buildSemantics(Size size) {
    buildSemanticsCallCount += 1;
    return <CustomPainterSemantics>[semantics];
  }

  @override
  bool shouldRepaint(_PainterWithSemantics oldPainter) {
    return true;
  }

  @override
  bool shouldRebuildSemantics(_PainterWithSemantics oldPainter) {
    shouldRebuildSemanticsCallCount += 1;
    return !identical(oldPainter.semantics, semantics);
  }
}

class _PainterWithoutSemantics extends CustomPainter {
  _PainterWithoutSemantics();

  @override
  void paint(Canvas canvas, Size size) {
    // We don't test painting.
  }

  @override
  bool shouldRepaint(_PainterWithSemantics oldPainter) => true;
}
