// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/src/foundation/constants.dart';
import 'package:flutter_test/flutter_test.dart';

class _TestSliverPersistentHeaderDelegate extends SliverPersistentHeaderDelegate {
  _TestSliverPersistentHeaderDelegate({
    required this.minExtent,
    required this.maxExtent,
    required this.child,
    this.vsync = const TestVSync(),
  });

  final Widget child;

  @override
  final double maxExtent;

  @override
  final double minExtent;

  @override
  final TickerProvider? vsync;

  @override
  final PersistentHeaderShowOnScreenConfiguration showOnScreenConfiguration =
      const PersistentHeaderShowOnScreenConfiguration();

  @override
  Widget build(BuildContext context, double shrinkOffset, bool overlapsContent) => child;

  @override
  bool shouldRebuild(_TestSliverPersistentHeaderDelegate oldDelegate) => true;
}

void main() {
  const TextStyle textStyle = TextStyle();
  const Color cursorColor = Color.fromARGB(0xFF, 0xFF, 0x00, 0x00);

  late TextEditingController controller;
  late FocusNode focusNode;

  setUp(() {
    controller = TextEditingController();
    focusNode = FocusNode();
  });

  tearDown(() {
    controller.dispose();
    focusNode.dispose();
  });

  testWidgets('tapping on a partly visible editable brings it fully on screen', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SizedBox(
            height: 300.0,
            child: ListView(
              controller: scrollController,
              children: <Widget>[
                EditableText(
                  backgroundCursorColor: Colors.grey,
                  controller: controller,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                ),
                const SizedBox(height: 350.0),
              ],
            ),
          ),
        ),
      ),
    );

    // Scroll the EditableText half off screen.
    final RenderBox render = tester.renderObject(find.byType(EditableText));
    scrollController.jumpTo(render.size.height / 2);
    await tester.pumpAndSettle();
    expect(scrollController.offset, render.size.height / 2);

    await tester.showKeyboard(find.byType(EditableText));
    await tester.pumpAndSettle();
    expect(scrollController.offset, 0.0);
  });

  testWidgets('tapping on a partly visible editable brings it fully on screen with scrollInsets', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SizedBox(
            height: 300.0,
            child: ListView(
              controller: scrollController,
              children: <Widget>[
                const SizedBox(height: 200.0),
                EditableText(
                  backgroundCursorColor: Colors.grey,
                  scrollPadding: const EdgeInsets.all(50.0),
                  controller: controller,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                ),
                const SizedBox(height: 850.0),
              ],
            ),
          ),
        ),
      ),
    );

    // Scroll the EditableText half off screen.
    final RenderBox render = tester.renderObject(find.byType(EditableText));
    scrollController.jumpTo(200 + render.size.height / 2);
    await tester.pumpAndSettle();
    expect(scrollController.offset, 200 + render.size.height / 2);

    await tester.showKeyboard(find.byType(EditableText));
    await tester.pumpAndSettle();
    // Container above the text is 200 in height, the scrollInsets are 50
    // Tolerance of 5 units (The actual value was 152.0 in the current tests instead of 150.0)
    expect(scrollController.offset, lessThan(200.0 - 50.0 + 5.0));
    expect(scrollController.offset, greaterThan(200.0 - 50.0 - 5.0));
  });

  testWidgets('editable comes back on screen when entering text while it is off-screen', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController(initialScrollOffset: 100.0);
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SizedBox(
            height: 300.0,
            child: ListView(
              controller: scrollController,
              children: <Widget>[
                const SizedBox(height: 350.0),
                EditableText(
                  backgroundCursorColor: Colors.grey,
                  controller: controller,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                ),
                const SizedBox(height: 350.0),
              ],
            ),
          ),
        ),
      ),
    );

    // Focus the EditableText and scroll it off screen.
    await tester.showKeyboard(find.byType(EditableText));
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isTrue);
    scrollController.jumpTo(0.0);
    await tester.pumpAndSettle();
    expect(scrollController.offset, 0.0);
    expect(find.byType(EditableText), findsNothing);

    // Entering text brings it back on screen.
    tester.testTextInput.enterText('Hello');
    await tester.pumpAndSettle();
    expect(scrollController.offset, greaterThan(0.0));
    expect(find.byType(EditableText), findsOneWidget);
  });

  testWidgets('entering text does not scroll when scrollPhysics.allowImplicitScrolling = false', (
    WidgetTester tester,
  ) async {
    // regression test for https://github.com/flutter/flutter/issues/19523
    final ScrollController scrollController = ScrollController(initialScrollOffset: 100.0);
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SizedBox(
            height: 300.0,
            child: ListView(
              physics: const NoImplicitScrollPhysics(),
              controller: scrollController,
              children: <Widget>[
                const SizedBox(height: 350.0),
                EditableText(
                  backgroundCursorColor: Colors.grey,
                  controller: controller,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                ),
                const SizedBox(height: 350.0),
              ],
            ),
          ),
        ),
      ),
    );

    // Focus the EditableText and scroll it off screen.
    await tester.showKeyboard(find.byType(EditableText));
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isTrue);
    scrollController.jumpTo(0.0);
    await tester.pumpAndSettle();
    expect(scrollController.offset, 0.0);
    expect(find.byType(EditableText), findsNothing);

    // Entering text brings it not back on screen.
    tester.testTextInput.enterText('Hello');
    await tester.pumpAndSettle();
    expect(scrollController.offset, 0.0);
    expect(find.byType(EditableText), findsNothing);
  });

  testWidgets('entering text does not scroll a surrounding PageView', (WidgetTester tester) async {
    // regression test for https://github.com/flutter/flutter/issues/19523
    final PageController pageController = PageController(initialPage: 1);
    addTearDown(pageController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: PageView(
                controller: pageController,
                children: <Widget>[
                  Container(color: Colors.red),
                  ColoredBox(color: Colors.green, child: TextField(controller: controller)),
                  Container(color: Colors.red),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    await tester.showKeyboard(find.byType(EditableText));
    await tester.pumpAndSettle();
    expect(controller.text, '');
    tester.testTextInput.enterText('H');
    final int frames = await tester.pumpAndSettle();

    // The text input should not trigger any animations, which would indicate
    // that the surrounding PageView is incorrectly scrolling back-and-forth.
    expect(frames, 1);

    expect(controller.text, 'H');
  });

  testWidgets('focused multi-line editable scrolls caret back into view when typing', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    controller.text = "Start${'\n' * 39}End";

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SizedBox(
            height: 300.0,
            child: ListView(
              controller: scrollController,
              children: <Widget>[
                EditableText(
                  backgroundCursorColor: Colors.grey,
                  maxLines: null, // multiline
                  controller: controller,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                ),
              ],
            ),
          ),
        ),
      ),
    );

    // Bring keyboard up and verify that end of EditableText is not on screen.
    await tester.showKeyboard(find.byType(EditableText));
    await tester.pumpAndSettle();
    scrollController.jumpTo(0.0);
    await tester.pumpAndSettle();
    final RenderBox render = tester.renderObject(find.byType(EditableText));
    expect(render.size.height, greaterThan(500.0));
    expect(scrollController.offset, 0.0);

    // Enter text at end, which is off-screen.
    final String textToEnter = '${controller.text} HELLO';
    tester.testTextInput.updateEditingValue(
      TextEditingValue(
        text: textToEnter,
        selection: TextSelection.collapsed(offset: textToEnter.length),
      ),
    );
    await tester.pumpAndSettle();

    // Caret scrolls into view.
    expect(find.byType(EditableText), findsOneWidget);
    expect(render.size.height, greaterThan(500.0));
    expect(scrollController.offset, greaterThan(0.0));
  });

  testWidgets(
    'focused multi-line editable does not scroll to old position when non-collapsed selection set',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      final String text = "Start${'\n' * 39}End";
      controller.value = TextEditingValue(
        text: text,
        selection: TextSelection.collapsed(offset: text.length - 3),
      );

      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: SizedBox(
              height: 300.0,
              child: ListView(
                controller: scrollController,
                children: <Widget>[
                  EditableText(
                    backgroundCursorColor: Colors.grey,
                    maxLines: null, // multiline
                    controller: controller,
                    focusNode: focusNode,
                    style: textStyle,
                    cursorColor: cursorColor,
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      // Bring keyboard up and verify that end of EditableText is not on screen.
      await tester.showKeyboard(find.byType(EditableText));
      await tester.pumpAndSettle();

      scrollController.jumpTo(0.0);
      await tester.pumpAndSettle();
      final RenderBox render = tester.renderObject(find.byType(EditableText));
      expect(render.size.height, greaterThan(500.0));
      expect(scrollController.offset, 0.0);

      // Change selection to non-collapsed so that cursor isn't shown
      // and the location requires a bit of scroll.
      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: text,
          selection: const TextSelection(baseOffset: 26, extentOffset: 27),
        ),
      );
      await tester.pumpAndSettle();

      // Selection extent scrolls into view.
      expect(find.byType(EditableText), findsOneWidget);
      expect(render.size.height, greaterThan(500.0));
      expect(scrollController.offset, 28.0);
    },
  );

  testWidgets('scrolls into view with scrollInserts after the keyboard pops up', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    const Key container = Key('container');

    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.bottomCenter,
          child: SizedBox(
            height: 300.0,
            child: ListView(
              controller: scrollController,
              children: <Widget>[
                const SizedBox(key: container, height: 200.0),
                EditableText(
                  backgroundCursorColor: Colors.grey,
                  scrollPadding: const EdgeInsets.only(bottom: 300.0),
                  controller: controller,
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                ),
                const SizedBox(height: 400.0),
              ],
            ),
          ),
        ),
      ),
    );

    expect(scrollController.offset, 0.0);

    await tester.showKeyboard(find.byType(EditableText));
    await tester.pumpAndSettle();
    expect(scrollController.offset, greaterThan(0.0));
    expect(find.byKey(container), findsNothing);
  });

  testWidgets(
    'A pinned persistent header should not scroll when its descendant EditableText gains focus',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/25507.
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      const Key headerKey = Key('header');

      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: SizedBox(
              height: 600.0,
              width: 600.0,
              child: CustomScrollView(
                controller: scrollController,
                slivers: List<Widget>.generate(50, (int i) {
                  return i == 10
                      ? SliverPersistentHeader(
                        pinned: true,
                        delegate: _TestSliverPersistentHeaderDelegate(
                          minExtent: 50,
                          maxExtent: 50,
                          child: Container(
                            alignment: Alignment.topCenter,
                            child: EditableText(
                              key: headerKey,
                              backgroundCursorColor: Colors.grey,
                              controller: controller,
                              focusNode: focusNode,
                              style: textStyle,
                              cursorColor: cursorColor,
                            ),
                          ),
                        ),
                      )
                      : SliverToBoxAdapter(child: SizedBox(height: 100.0, child: Text('Tile $i')));
                }),
              ),
            ),
          ),
        ),
      );

      // The persistent header should now be pinned at the top.
      scrollController.jumpTo(100.0 * 15);
      await tester.pumpAndSettle();
      expect(scrollController.offset, 100.0 * 15);

      focusNode.requestFocus();
      await tester.pumpAndSettle();
      // The scroll offset should remain the same.
      expect(scrollController.offset, 100.0 * 15);
    },
  );

  testWidgets(
    'A pinned persistent header should not scroll when its descendant EditableText gains focus (no animation)',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/25507.
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);

      const Key headerKey = Key('header');
      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: SizedBox(
              height: 600.0,
              width: 600.0,
              child: CustomScrollView(
                controller: scrollController,
                slivers: List<Widget>.generate(50, (int i) {
                  return i == 10
                      ? SliverPersistentHeader(
                        pinned: true,
                        delegate: _TestSliverPersistentHeaderDelegate(
                          minExtent: 50,
                          maxExtent: 50,
                          vsync: null,
                          child: Container(
                            alignment: Alignment.topCenter,
                            child: EditableText(
                              key: headerKey,
                              backgroundCursorColor: Colors.grey,
                              controller: controller,
                              focusNode: focusNode,
                              style: textStyle,
                              cursorColor: cursorColor,
                            ),
                          ),
                        ),
                      )
                      : SliverToBoxAdapter(child: SizedBox(height: 100.0, child: Text('Tile $i')));
                }),
              ),
            ),
          ),
        ),
      );

      // The persistent header should now be pinned at the top.
      scrollController.jumpTo(100.0 * 15);
      await tester.pumpAndSettle();
      expect(scrollController.offset, 100.0 * 15);

      focusNode.requestFocus();
      await tester.pumpAndSettle();
      // The scroll offset should remain the same.
      expect(scrollController.offset, 100.0 * 15);
    },
  );

  void testShowCaretOnScreen({required bool readOnly}) {
    group('EditableText._showCaretOnScreen, readOnly=$readOnly', () {
      final TextInputFormatter rejectEverythingFormatter = TextInputFormatter.withFunction(
        (TextEditingValue old, TextEditingValue value) => old,
      );

      bool isCaretOnScreen(WidgetTester tester) {
        final EditableTextState state = tester.state<EditableTextState>(
          find.byType(EditableText, skipOffstage: false),
        );
        final RenderEditable renderEditable = state.renderEditable;
        final Rect localRect = renderEditable.getLocalRectForCaret(
          state.textEditingValue.selection.base,
        );
        final Offset caretOrigin = renderEditable.localToGlobal(localRect.topLeft);
        final Rect caretRect = caretOrigin & localRect.size;
        return const Rect.fromLTWH(0, 0, 800, 600).intersect(caretRect) == caretRect;
      }

      Widget buildEditableText({
        required bool rejectUserInputs,
        ScrollController? scrollController,
        ScrollController? editableScrollController,
      }) {
        return MaterialApp(
          home: Scaffold(
            body: ListView(
              controller: scrollController,
              cacheExtent: 1000,
              children: <Widget>[
                // The text field is not fully visible.
                const SizedBox(height: 599),
                EditableText(
                  backgroundCursorColor: Colors.grey,
                  controller: controller,
                  scrollController: editableScrollController,
                  inputFormatters: <TextInputFormatter>[
                    if (rejectUserInputs) rejectEverythingFormatter,
                  ],
                  focusNode: focusNode,
                  style: textStyle,
                  cursorColor: cursorColor,
                  readOnly: readOnly,
                ),
              ],
            ),
          ),
        );
      }

      testWidgets('focus-triggered showCaretOnScreen', (WidgetTester tester) async {
        controller.text = 'a' * 100;
        controller.selection = const TextSelection.collapsed(offset: 100);
        final ScrollController scrollController = ScrollController();
        addTearDown(scrollController.dispose);
        final ScrollController editableScrollController = ScrollController();
        addTearDown(editableScrollController.dispose);

        await tester.pumpWidget(
          buildEditableText(
            rejectUserInputs: false,
            scrollController: scrollController,
            editableScrollController: editableScrollController,
          ),
        );

        focusNode.requestFocus();
        await tester.pumpAndSettle();

        if (kIsWeb) {
          await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
          await tester.pump();
        }

        // On web, the entire field is selected, and only part of that selection
        // is visible on the screen.
        expect(isCaretOnScreen(tester), !readOnly && !kIsWeb);
        expect(scrollController.offset, readOnly ? 0.0 : greaterThan(0.0));
        expect(editableScrollController.offset, readOnly ? 0.0 : greaterThan(0.0));
      });

      testWidgets('selection-triggered showCaretOnScreen: virtual keyboard', (
        WidgetTester tester,
      ) async {
        controller.text = 'a' * 100;
        controller.selection = const TextSelection.collapsed(offset: 80);
        final ScrollController scrollController = ScrollController();
        addTearDown(scrollController.dispose);
        final ScrollController editableScrollController = ScrollController();
        addTearDown(editableScrollController.dispose);

        await tester.pumpWidget(
          buildEditableText(
            rejectUserInputs: false,
            scrollController: scrollController,
            editableScrollController: editableScrollController,
          ),
        );

        focusNode.requestFocus();
        await tester.pumpAndSettle();

        // Ensure the caret is not fully visible and the text field is focused.
        scrollController.jumpTo(0);
        editableScrollController.jumpTo(0);
        await tester.pumpAndSettle();
        expect(isCaretOnScreen(tester), isFalse);

        final EditableTextState state = tester.state<EditableTextState>(
          find.byType(EditableText, skipOffstage: false),
        );

        // Change the selection. Show caret on screen when readyOnly is true,
        // as a read-only text field rejects everything from the software
        // keyboard (except for web).
        state.updateEditingValue(
          state.textEditingValue.copyWith(selection: const TextSelection.collapsed(offset: 90)),
        );
        await tester.pumpAndSettle();
        expect(isCaretOnScreen(tester), !readOnly || kIsWeb);
        expect(scrollController.offset, readOnly && !kIsWeb ? 0.0 : greaterThan(0.0));
        expect(editableScrollController.offset, readOnly && !kIsWeb ? 0.0 : greaterThan(0.0));

        // Reject user input.
        await tester.pumpWidget(
          buildEditableText(
            rejectUserInputs: true,
            scrollController: scrollController,
            editableScrollController: editableScrollController,
          ),
        );

        // Ensure the caret is not fully visible and the text field is focused.
        scrollController.jumpTo(0);
        editableScrollController.jumpTo(0);
        await tester.pumpAndSettle();
        expect(isCaretOnScreen(tester), isFalse);

        state.updateEditingValue(
          state.textEditingValue.copyWith(selection: const TextSelection.collapsed(offset: 100)),
        );
        await tester.pumpAndSettle();
        expect(isCaretOnScreen(tester), !readOnly || kIsWeb);
        expect(scrollController.offset, readOnly && !kIsWeb ? 0.0 : greaterThan(0.0));
        expect(editableScrollController.offset, readOnly && !kIsWeb ? 0.0 : greaterThan(0.0));
      });

      testWidgets('selection-triggered showCaretOnScreen: text selection delegate', (
        WidgetTester tester,
      ) async {
        controller.text = 'a' * 100;
        controller.selection = const TextSelection.collapsed(offset: 80);
        final ScrollController scrollController = ScrollController();
        addTearDown(scrollController.dispose);
        final ScrollController editableScrollController = ScrollController();
        addTearDown(editableScrollController.dispose);

        await tester.pumpWidget(
          buildEditableText(
            rejectUserInputs: false,
            scrollController: scrollController,
            editableScrollController: editableScrollController,
          ),
        );

        focusNode.requestFocus();
        await tester.pumpAndSettle();

        // Ensure the caret is not fully visible and the text field is focused.
        scrollController.jumpTo(0);
        editableScrollController.jumpTo(0);
        await tester.pumpAndSettle();
        expect(isCaretOnScreen(tester), isFalse);

        final EditableTextState state = tester.state<EditableTextState>(
          find.byType(EditableText, skipOffstage: false),
        );

        // Change the selection. Show caret on screen even when readyOnly is
        // false.
        state.userUpdateTextEditingValue(
          state.textEditingValue.copyWith(selection: const TextSelection.collapsed(offset: 90)),
          null,
        );
        await tester.pumpAndSettle();
        expect(isCaretOnScreen(tester), isTrue);
        expect(scrollController.offset, greaterThan(0.0));
        expect(editableScrollController.offset, greaterThan(0.0));

        // Rejects user input.
        await tester.pumpWidget(
          buildEditableText(
            rejectUserInputs: true,
            scrollController: scrollController,
            editableScrollController: editableScrollController,
          ),
        );

        // Ensure the caret is not fully visible and the text field is focused.
        scrollController.jumpTo(0);
        editableScrollController.jumpTo(0);
        await tester.pumpAndSettle();
        expect(isCaretOnScreen(tester), isFalse);

        state.userUpdateTextEditingValue(
          state.textEditingValue.copyWith(selection: const TextSelection.collapsed(offset: 100)),
          null,
        );
        await tester.pumpAndSettle();
        expect(isCaretOnScreen(tester), isTrue);
        expect(scrollController.offset, greaterThan(0.0));
        expect(editableScrollController.offset, greaterThan(0.0));
      });

      // Regression text for https://github.com/flutter/flutter/pull/74722.
      testWidgets('does NOT randomly trigger when cursor blinks', (WidgetTester tester) async {
        controller.text = 'a' * 100;
        controller.selection = const TextSelection.collapsed(offset: 0);
        final ScrollController editableScrollController = ScrollController();
        addTearDown(editableScrollController.dispose);
        final bool deterministicCursor = EditableText.debugDeterministicCursor;
        EditableText.debugDeterministicCursor = false;

        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: EditableText(
                backgroundCursorColor: Colors.grey,
                controller: controller,
                scrollController: editableScrollController,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
              ),
            ),
          ),
        );

        final EditableTextState state = tester.state<EditableTextState>(
          find.byType(EditableText, skipOffstage: false),
        );

        // Ensure the text was initially visible.
        expect(isCaretOnScreen(tester), true);
        expect(editableScrollController.offset, 0.0);

        // Change the text but keep the cursor location.
        state.updateEditingValue(controller.value.copyWith(text: 'a' * 101));

        await tester.pumpAndSettle();

        // The caret should stay where it was, since the selection didn't change.
        expect(isCaretOnScreen(tester), true);
        expect(editableScrollController.offset, 0.0);

        // Now move to hide the cursor.
        editableScrollController.jumpTo(100.0);

        // Does not trigger showCaretOnScreen.
        await tester.pump();
        await tester.pumpAndSettle();
        expect(editableScrollController.offset, 100.0);
        expect(isCaretOnScreen(tester), isFalse);

        EditableText.debugDeterministicCursor = deterministicCursor;
      });
    });
  }

  testShowCaretOnScreen(readOnly: true);
  testShowCaretOnScreen(readOnly: false);
}

class NoImplicitScrollPhysics extends AlwaysScrollableScrollPhysics {
  const NoImplicitScrollPhysics({super.parent});

  @override
  bool get allowImplicitScrolling => false;

  @override
  NoImplicitScrollPhysics applyTo(ScrollPhysics? ancestor) {
    return NoImplicitScrollPhysics(parent: buildParent(ancestor));
  }
}
