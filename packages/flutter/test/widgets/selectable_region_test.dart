// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'clipboard_utils.dart';
import 'keyboard_utils.dart';
import 'process_text_utils.dart';
import 'semantics_tester.dart';

Offset textOffsetToPosition(RenderParagraph paragraph, int offset) {
  const Rect caret = Rect.fromLTWH(0.0, 0.0, 2.0, 20.0);
  final Offset localOffset =
      paragraph.getOffsetForCaret(TextPosition(offset: offset), caret) +
      Offset(0.0, paragraph.preferredLineHeight);
  return paragraph.localToGlobal(localOffset) + const Offset(kIsWeb ? 1.0 : 0.0, -2.0);
}

Offset globalize(Offset point, RenderBox box) {
  return box.localToGlobal(point);
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final MockClipboard mockClipboard = MockClipboard();

  setUp(() async {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      mockClipboard.handleMethodCall,
    );
    await Clipboard.setData(const ClipboardData(text: 'empty'));
  });

  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  });

  group('SelectableRegion', () {
    testWidgets('mouse selection single click sends correct events', (WidgetTester tester) async {
      final UniqueKey spy = UniqueKey();
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      final TestGesture gesture = await tester.startGesture(
        const Offset(200.0, 200.0),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pumpAndSettle();
      renderSelectionSpy.events.clear();

      await gesture.moveTo(const Offset(200.0, 100.0));
      expect(renderSelectionSpy.events.length, 2);
      expect(renderSelectionSpy.events[0].type, SelectionEventType.startEdgeUpdate);
      final SelectionEdgeUpdateEvent startEdge =
          renderSelectionSpy.events[0] as SelectionEdgeUpdateEvent;
      expect(startEdge.globalPosition, const Offset(200.0, 200.0));
      expect(renderSelectionSpy.events[1].type, SelectionEventType.endEdgeUpdate);
      SelectionEdgeUpdateEvent endEdge = renderSelectionSpy.events[1] as SelectionEdgeUpdateEvent;
      expect(endEdge.globalPosition, const Offset(200.0, 100.0));
      renderSelectionSpy.events.clear();

      await gesture.moveTo(const Offset(100.0, 100.0));
      expect(renderSelectionSpy.events.length, 1);
      expect(renderSelectionSpy.events[0].type, SelectionEventType.endEdgeUpdate);
      endEdge = renderSelectionSpy.events[0] as SelectionEdgeUpdateEvent;
      expect(endEdge.globalPosition, const Offset(100.0, 100.0));

      await gesture.up();
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/102410.

    testWidgets('mouse double click sends select-word event', (WidgetTester tester) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      final TestGesture gesture = await tester.startGesture(
        const Offset(200.0, 200.0),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      renderSelectionSpy.events.clear();
      await gesture.down(const Offset(200.0, 200.0));
      await tester.pump();
      await gesture.up();
      expect(renderSelectionSpy.events.length, 1);
      expect(renderSelectionSpy.events[0], isA<SelectWordSelectionEvent>());
      final SelectWordSelectionEvent selectionEvent =
          renderSelectionSpy.events[0] as SelectWordSelectionEvent;
      expect(selectionEvent.globalPosition, const Offset(200.0, 200.0));
    });

    testWidgets('touch double click sends select-word event', (WidgetTester tester) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      final TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      renderSelectionSpy.events.clear();
      await gesture.down(const Offset(200.0, 200.0));
      await tester.pump();
      await gesture.up();
      expect(renderSelectionSpy.events.length, 1);
      expect(renderSelectionSpy.events[0], isA<SelectWordSelectionEvent>());
      final SelectWordSelectionEvent selectionEvent =
          renderSelectionSpy.events[0] as SelectWordSelectionEvent;
      expect(selectionEvent.globalPosition, const Offset(200.0, 200.0));
    });

    testWidgets('Does not crash when using Navigator pages', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/119776
      await tester.pumpWidget(
        MaterialApp(
          home: Navigator(
            pages: <Page<void>>[
              MaterialPage<void>(
                child: Column(
                  children: <Widget>[
                    const Text('How are you?'),
                    SelectableRegion(
                      selectionControls: materialTextSelectionControls,
                      child: const SelectAllWidget(child: SizedBox(width: 100, height: 100)),
                    ),
                    const Text('Fine, thank you.'),
                  ],
                ),
              ),
              const MaterialPage<void>(child: Scaffold(body: Text('Foreground Page'))),
            ],
            onPopPage: (_, __) => false,
          ),
        ),
      );

      expect(tester.takeException(), isNull);
    });

    testWidgets('can draw handles when they are at rect boundaries', (WidgetTester tester) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              const Text('How are you?'),
              SelectableRegion(
                selectionControls: materialTextSelectionControls,
                child: SelectAllWidget(key: spy, child: const SizedBox(width: 100, height: 100)),
              ),
              const Text('Fine, thank you.'),
            ],
          ),
        ),
      );
      await tester.pumpAndSettle();

      final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byKey(spy)));
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump();

      final RenderSelectAll renderSpy = tester.renderObject<RenderSelectAll>(find.byKey(spy));
      expect(renderSpy.startHandle, isNotNull);
      expect(renderSpy.endHandle, isNotNull);
    });

    testWidgets('touch does not accept drag', (WidgetTester tester) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      final TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));
      addTearDown(gesture.removePointer);
      await gesture.moveTo(const Offset(200.0, 100.0));
      await gesture.up();
      expect(
        renderSelectionSpy.events.every((SelectionEvent element) => element is ClearSelectionEvent),
        isTrue,
      );
    });

    testWidgets('does not merge semantics node of the children', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: Scaffold(
              body: Center(
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    const Text('Line one'),
                    const Text('Line two'),
                    ElevatedButton(onPressed: () {}, child: const Text('Button')),
                  ],
                ),
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
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(label: 'Line one', textDirection: TextDirection.ltr),
                          TestSemantics(label: 'Line two', textDirection: TextDirection.ltr),
                          TestSemantics(
                            flags: <SemanticsFlag>[
                              SemanticsFlag.isButton,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                            ],
                            actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                            label: 'Button',
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

    testWidgets(
      'Horizontal PageView beats SelectionArea child touch drag gestures on iOS',
      (WidgetTester tester) async {
        final PageController pageController = PageController();
        const String testValue = 'abc def ghi jkl mno pqr stu vwx yz';
        addTearDown(pageController.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: PageView(
              controller: pageController,
              children: <Widget>[
                Center(
                  child: SelectableRegion(
                    selectionControls: materialTextSelectionControls,
                    child: const Text(testValue),
                  ),
                ),
                const SizedBox(height: 200.0, child: Center(child: Text('Page 2'))),
              ],
            ),
          ),
        );

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text(testValue), matching: find.byType(RichText)),
        );
        final Offset gPos = textOffsetToPosition(paragraph, testValue.indexOf('g'));
        final Offset pPos = textOffsetToPosition(paragraph, testValue.indexOf('p'));

        // A double tap + drag should take precedence over parent drags.
        final TestGesture gesture = await tester.startGesture(gPos);
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        await gesture.down(gPos);
        await tester.pumpAndSettle();
        await gesture.moveTo(pPos);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph.selections, isNotEmpty);
        expect(
          paragraph.selections[0],
          TextSelection(
            baseOffset: testValue.indexOf('g'),
            extentOffset: testValue.indexOf('p') + 3,
          ),
        );

        expect(pageController.page, isNotNull);
        expect(pageController.page, 0.0);
        // A horizontal drag directly on the SelectableRegion should move the page
        // view to the next page.
        final Rect selectableTextRect = tester.getRect(find.byType(SelectableRegion));
        await tester.dragFrom(
          selectableTextRect.centerRight - const Offset(0.1, 0.0),
          const Offset(-500.0, 0.0),
        );
        await tester.pumpAndSettle();
        expect(pageController.page, isNotNull);
        expect(pageController.page, 1.0);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.iOS),
      skip: kIsWeb, // https://github.com/flutter/flutter/issues/125582.
    );

    testWidgets(
      'Vertical PageView beats SelectionArea child touch drag gestures',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/150897.
        final PageController pageController = PageController();
        const String testValue = 'abc def ghi jkl mno pqr stu vwx yz';
        addTearDown(pageController.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: PageView(
              scrollDirection: Axis.vertical,
              controller: pageController,
              children: <Widget>[
                Center(
                  child: SelectableRegion(
                    selectionControls: materialTextSelectionControls,
                    child: const Text(testValue),
                  ),
                ),
                const SizedBox(height: 200.0, child: Center(child: Text('Page 2'))),
              ],
            ),
          ),
        );

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text(testValue), matching: find.byType(RichText)),
        );
        final Offset gPos = textOffsetToPosition(paragraph, testValue.indexOf('g'));
        final Offset pPos = textOffsetToPosition(paragraph, testValue.indexOf('p'));

        // A double tap + drag should take precedence over parent drags.
        final TestGesture gesture = await tester.startGesture(gPos);
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        await gesture.down(gPos);
        await tester.pumpAndSettle();
        await gesture.moveTo(pPos);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph.selections, isNotEmpty);
        expect(
          paragraph.selections[0],
          TextSelection(
            baseOffset: testValue.indexOf('g'),
            extentOffset: testValue.indexOf('p') + 3,
          ),
        );

        expect(pageController.page, isNotNull);
        expect(pageController.page, 0.0);
        // A vertical drag directly on the SelectableRegion should move the page
        // view to the next page.
        final Rect selectableTextRect = tester.getRect(find.byType(SelectableRegion));
        // Simulate a pan by drag vertically first.
        await gesture.down(selectableTextRect.center);
        await tester.pump();
        await gesture.moveTo(selectableTextRect.center + const Offset(0.0, -200.0));
        // Introduce horizontal movement.
        await gesture.moveTo(selectableTextRect.center + const Offset(5.0, -300.0));
        await gesture.moveTo(selectableTextRect.center + const Offset(-10.0, -400.0));
        // Continue dragging vertically.
        await gesture.moveTo(selectableTextRect.center + const Offset(0.0, -500.0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(pageController.page, isNotNull);
        expect(pageController.page, 1.0);
      },
      variant: TargetPlatformVariant.mobile(),
      skip: kIsWeb, // https://github.com/flutter/flutter/issues/125582.
    );

    testWidgets('mouse single-click selection collapses the selection', (
      WidgetTester tester,
    ) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      final TestGesture gesture = await tester.startGesture(
        const Offset(200.0, 200.0),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(renderSelectionSpy.events.length, 2);
      expect(renderSelectionSpy.events[0], isA<SelectionEdgeUpdateEvent>());
      expect(
        (renderSelectionSpy.events[0] as SelectionEdgeUpdateEvent).type,
        SelectionEventType.startEdgeUpdate,
      );
      expect(renderSelectionSpy.events[1], isA<SelectionEdgeUpdateEvent>());
      expect(
        (renderSelectionSpy.events[1] as SelectionEdgeUpdateEvent).type,
        SelectionEventType.endEdgeUpdate,
      );
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/102410.

    testWidgets('touch long press sends select-word event', (WidgetTester tester) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      renderSelectionSpy.events.clear();
      final TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      expect(renderSelectionSpy.events.length, 1);
      expect(renderSelectionSpy.events[0], isA<SelectWordSelectionEvent>());
      final SelectWordSelectionEvent selectionEvent =
          renderSelectionSpy.events[0] as SelectWordSelectionEvent;
      expect(selectionEvent.globalPosition, const Offset(200.0, 200.0));
    });

    testWidgets('touch long press and drag sends correct events', (WidgetTester tester) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      renderSelectionSpy.events.clear();
      final TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      expect(renderSelectionSpy.events.length, 1);
      expect(renderSelectionSpy.events[0], isA<SelectWordSelectionEvent>());
      final SelectWordSelectionEvent selectionEvent =
          renderSelectionSpy.events[0] as SelectWordSelectionEvent;
      expect(selectionEvent.globalPosition, const Offset(200.0, 200.0));

      renderSelectionSpy.events.clear();
      await gesture.moveTo(const Offset(200.0, 50.0));
      await gesture.up();
      expect(renderSelectionSpy.events.length, 1);
      expect(renderSelectionSpy.events[0].type, SelectionEventType.endEdgeUpdate);
      final SelectionEdgeUpdateEvent edgeEvent =
          renderSelectionSpy.events[0] as SelectionEdgeUpdateEvent;
      expect(edgeEvent.globalPosition, const Offset(200.0, 50.0));
      expect(edgeEvent.granularity, TextGranularity.word);
    });

    testWidgets('touch long press cancel does not send ClearSelectionEvent', (
      WidgetTester tester,
    ) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      renderSelectionSpy.events.clear();
      final TestGesture gesture = await tester.startGesture(const Offset(200.0, 200.0));

      addTearDown(gesture.removePointer);

      await tester.pump(const Duration(milliseconds: 500));
      await gesture.cancel();
      expect(
        renderSelectionSpy.events.any((SelectionEvent element) => element is ClearSelectionEvent),
        isFalse,
      );
    });

    testWidgets('scrolling after the selection does not send ClearSelectionEvent', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/128765
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SizedBox(
            height: 750,
            child: SingleChildScrollView(
              child: SizedBox(
                height: 2000,
                child: SelectableRegion(
                  selectionControls: materialTextSelectionControls,
                  child: SelectionSpy(key: spy),
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      renderSelectionSpy.events.clear();
      final TestGesture selectGesture = await tester.startGesture(const Offset(200.0, 200.0));
      addTearDown(selectGesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await selectGesture.up();
      expect(renderSelectionSpy.events.length, 1);
      expect(renderSelectionSpy.events[0], isA<SelectWordSelectionEvent>());

      renderSelectionSpy.events.clear();
      final TestGesture scrollGesture = await tester.startGesture(const Offset(250.0, 850.0));
      await tester.pump(const Duration(milliseconds: 500));
      await scrollGesture.moveTo(Offset.zero);
      await scrollGesture.up();
      await tester.pumpAndSettle();
      expect(renderSelectionSpy.events.length, 0);
    });

    testWidgets('mouse long press does not send select-word event', (WidgetTester tester) async {
      final UniqueKey spy = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: SelectionSpy(key: spy),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderSelectionSpy renderSelectionSpy = tester.renderObject<RenderSelectionSpy>(
        find.byKey(spy),
      );
      renderSelectionSpy.events.clear();
      final TestGesture gesture = await tester.startGesture(
        const Offset(200.0, 200.0),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      expect(
        renderSelectionSpy.events.every(
          (SelectionEvent element) => element is SelectionEdgeUpdateEvent,
        ),
        isTrue,
      );
    });
  });

  testWidgets('Can extend StaticSelectionContainerDelegate', (WidgetTester tester) async {
    SelectedContent? content;

    // Inserts a new line between selected content of children selectables.
    final ColumnSelectionContainerDelegate selectionDelegate = ColumnSelectionContainerDelegate();

    addTearDown(selectionDelegate.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: SelectableRegion(
          onSelectionChanged: (SelectedContent? selectedContent) => content = selectedContent,
          selectionControls: materialTextSelectionControls,
          child: SelectionContainer(
            delegate: selectionDelegate,
            child: const Center(
              child: Column(children: <Widget>[Text('Hello World!'), Text('How are you!')]),
            ),
          ),
        ),
      ),
    );

    final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Hello World!'), matching: find.byType(RichText)),
    );
    final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('How are you!'), matching: find.byType(RichText)),
    );
    final TestGesture mouseGesture = await tester.startGesture(
      textOffsetToPosition(paragraph, 4),
      kind: PointerDeviceKind.mouse,
    );

    expect(content, isNull);
    addTearDown(mouseGesture.removePointer);
    await tester.pump();

    // Move selection to second paragraph.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph2, 10));
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'o World!\nHow are yo');
    await mouseGesture.up();
    await tester.pump();
  });

  testWidgets(
    'dragging handle or selecting word triggers haptic feedback on Android',
    (WidgetTester tester) async {
      final List<MethodCall> log = <MethodCall>[];
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
        MethodCall methodCall,
      ) async {
        log.add(methodCall);
        return null;
      });
      addTearDown(() {
        tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.platform,
          mockClipboard.handleMethodCall,
        );
      });

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Text('How are you?'),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 6),
      ); // at the 'r'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 500));
      // `are` is selected.
      expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));
      expect(
        log.last,
        isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.selectionClick'),
      );
      log.clear();
      final List<TextBox> boxes = paragraph.getBoxesForSelection(paragraph.selections[0]);
      expect(boxes.length, 1);
      final Offset handlePos = globalize(boxes[0].toRect().bottomRight, paragraph);
      await gesture.down(handlePos);
      final Offset endPos = Offset(textOffsetToPosition(paragraph, 8).dx, handlePos.dy);

      // Select 1 more character by dragging end handle to trigger feedback.
      await gesture.moveTo(endPos);
      expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 8));
      // Only Android vibrate when dragging the handle.
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
          expect(
            log.last,
            isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.selectionClick'),
          );
        case TargetPlatform.fuchsia:
        case TargetPlatform.iOS:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(log, isEmpty);
      }
      await gesture.up();
    },
    variant: TargetPlatformVariant.all(),
  );

  group('SelectionArea integration', () {
    testWidgets(
      'selection is not cleared when app loses focus on desktop',
      (WidgetTester tester) async {
        Future<void> setAppLifecycleState(AppLifecycleState state) async {
          final ByteData? message = const StringCodec().encodeMessage(state.toString());
          await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
              .handlePlatformMessage('flutter/lifecycle', message, (_) {});
        }

        final FocusNode focusNode = FocusNode();
        final GlobalKey selectableKey = GlobalKey();
        addTearDown(focusNode.dispose);
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              key: selectableKey,
              focusNode: focusNode,
              selectionControls: materialTextSelectionControls,
              child: const Center(child: Text('How are you')),
            ),
          ),
        );
        await setAppLifecycleState(AppLifecycleState.resumed);
        await tester.pumpAndSettle();

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));
        expect(focusNode.hasFocus, isTrue);

        // Setting the app lifecycle state to AppLifecycleState.inactive to simulate
        // a lose of window focus.
        await setAppLifecycleState(AppLifecycleState.inactive);
        await tester.pumpAndSettle();
        expect(focusNode.hasFocus, isFalse);
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));
      },
      variant: TargetPlatformVariant.desktop(),
    );

    testWidgets(
      'touch can select word-by-word on double tap drag on mobile platforms',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Center(child: Text('How are you')),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph, 2));
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.moveTo(textOffsetToPosition(paragraph, 3));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 4));

        await gesture.moveTo(textOffsetToPosition(paragraph, 4));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));

        await gesture.moveTo(textOffsetToPosition(paragraph, 7));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 8));

        await gesture.moveTo(textOffsetToPosition(paragraph, 8));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));

        // Check backward selection.
        await gesture.moveTo(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        // Start a new double-click drag.
        await gesture.up();
        await tester.pump();
        await gesture.down(textOffsetToPosition(paragraph, 5));
        await tester.pump();
        await gesture.up();
        expect(paragraph.selections.isEmpty, isFalse);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 5));
        await tester.pump(kDoubleTapTimeout);

        // Double-click.
        await gesture.down(textOffsetToPosition(paragraph, 5));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        await gesture.down(textOffsetToPosition(paragraph, 5));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

        // Selecting across line should select to the end.
        await gesture.moveTo(textOffsetToPosition(paragraph, 5) + const Offset(0.0, 200.0));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 11));
        await gesture.up();
      },
      variant: TargetPlatformVariant.mobile(),
      // [intended] Web does not support double tap + drag gestures on all of the tested platforms.
      skip: kIsWeb,
    );

    testWidgets(
      'touch can select multiple widgets on double tap drag on mobile platforms',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph1, 2));
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph1, 2));
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
        await tester.pump();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));

        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
        // Should select the rest of paragraph 1.
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));

        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));

        await gesture.up();
      },
      variant: TargetPlatformVariant.mobile(),
      // [intended] Web does not support double tap + drag gestures on all of the tested platforms.
      skip: kIsWeb,
    );

    testWidgets(
      'touch can select multiple widgets on double tap drag and return to origin word on mobile platforms',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph1, 2));
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph1, 2));
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
        await tester.pump();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));

        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
        // Should select the rest of paragraph 1.
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));

        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));

        await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
        // Should clear the selection on paragraph 3.
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));
        expect(paragraph3.selections.isEmpty, isTrue);

        await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
        // Should clear the selection on paragraph 2.
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));
        expect(paragraph2.selections.isEmpty, isTrue);
        expect(paragraph3.selections.isEmpty, isTrue);

        await gesture.up();
      },
      variant: TargetPlatformVariant.mobile(),
      // [intended] Web does not support double tap + drag gestures on all of the tested platforms.
      skip: kIsWeb,
    );

    testWidgets(
      'touch can reverse selection across multiple widgets on double tap drag on mobile platforms',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph3, 10));
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph3, 10));
        await tester.pumpAndSettle();
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 11));

        await gesture.moveTo(textOffsetToPosition(paragraph3, 4));
        await tester.pump();
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 11, extentOffset: 4));

        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 11, extentOffset: 0));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 14, extentOffset: 5));

        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 11, extentOffset: 0));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 14, extentOffset: 0));
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 12, extentOffset: 4));

        await gesture.up();
      },
      variant: TargetPlatformVariant.mobile(),
      // [intended] Web does not support double tap + drag gestures on all of the tested platforms.
      skip: kIsWeb,
    );

    testWidgets(
      'touch cannot triple tap or triple tap drag on Android and iOS',
      (WidgetTester tester) async {
        const String longText =
            'Hello world this is some long piece of text '
            'that will represent a long paragraph, when triple clicking this block '
            'of text all of it will be selected.\n'
            'This will be the start of a new line. When triple clicking this block '
            'of text all of it should be selected.';

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Center(child: Text(longText)),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text(longText), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 150));

        await gesture.moveTo(textOffsetToPosition(paragraph, 155));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 257));

        await gesture.moveTo(textOffsetToPosition(paragraph, 170));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 257));

        // Check backward selection.
        await gesture.moveTo(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 150));

        // Start a new triple-click drag.
        await gesture.up();
        await tester.pumpAndSettle(kDoubleTapTimeout);
        await gesture.down(textOffsetToPosition(paragraph, 151));
        await tester.pumpAndSettle();
        await gesture.up();
        expect(paragraph.selections.isNotEmpty, isTrue);
        expect(paragraph.selections.length, 1);
        expect(paragraph.selections.first, const TextSelection.collapsed(offset: 151));
        await tester.pump(kDoubleTapTimeout);

        // Triple-click.
        await gesture.down(textOffsetToPosition(paragraph, 151));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        await gesture.down(textOffsetToPosition(paragraph, 151));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        await gesture.down(textOffsetToPosition(paragraph, 151));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 150, extentOffset: 257));
        await gesture.up();
        await tester.pumpAndSettle();

        // Reset selection.
        await tester.tapAt(textOffsetToPosition(paragraph, 0));
        await tester.pumpAndSettle(kDoubleTapTimeout);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 0));

        // Trying to triple-click with a touch gesture should not work.
        final TestGesture touchGesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
        );
        addTearDown(touchGesture.removePointer);
        await tester.pump();
        await touchGesture.up();
        await tester.pump();

        await touchGesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pump();
        await touchGesture.up();
        await tester.pump();

        await touchGesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pump();
        await touchGesture.up();
        await tester.pumpAndSettle();
        // The selection is collapsed on Android because the max consecutive tap count
        // on native Android is 2 when the pointer device kind is not precise like
        // for a touch.
        //
        // On iOS the selection is maintained because the tap occurred on the active
        // selection.
        expect(
          paragraph.selections[0],
          defaultTargetPlatform == TargetPlatform.iOS
              ? const TextSelection(baseOffset: 0, extentOffset: 5)
              : const TextSelection.collapsed(offset: 2),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.iOS,
      }),
      // [intended] Web does not support double tap + drag gestures on all of the tested platforms.
      skip: kIsWeb,
    );

    testWidgets(
      'touch cannot select word-by-word on double tap drag when on Android web',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Center(child: Text('How are you')),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph, 2));
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        // Dragging should not change the selection.
        await gesture.moveTo(textOffsetToPosition(paragraph, 3));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.moveTo(textOffsetToPosition(paragraph, 4));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.moveTo(textOffsetToPosition(paragraph, 7));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.moveTo(textOffsetToPosition(paragraph, 8));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        // Check backward selection.
        await gesture.moveTo(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));
        await gesture.up();
        await tester.pumpAndSettle();
      },
      skip: !kIsWeb, // [intended] This test verifies web behavior.
    );

    testWidgets(
      'touch can double tap + drag on iOS web',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Center(child: Text('How are you')),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph, 2));
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 2));

        // A double tap should not change the selection.
        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 2));

        // Dragging should change the selection.
        await gesture.moveTo(textOffsetToPosition(paragraph, 3));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 4));

        await gesture.moveTo(textOffsetToPosition(paragraph, 4));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));

        await gesture.moveTo(textOffsetToPosition(paragraph, 7));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 8));

        await gesture.moveTo(textOffsetToPosition(paragraph, 8));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));

        // Check backward selection.
        await gesture.moveTo(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));
        await gesture.up();
        await tester.pumpAndSettle();
      },
      variant: TargetPlatformVariant.only(TargetPlatform.iOS),
      skip: true, // https://github.com/flutter/flutter/issues/125582.
    );

    testWidgets(
      'touch cannot double tap on iOS web',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Center(child: Text('How are you')),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph, 2));
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 2));

        // A double tap should not change the selection.
        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 2));

        await gesture.up();
        await tester.pumpAndSettle();
      },
      variant: TargetPlatformVariant.only(TargetPlatform.iOS),
      skip: !kIsWeb, // [intended] This test verifies web behavior.
    );

    testWidgets(
      'RenderParagraph should invalidate cachedRect on window size change',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/155143.
        addTearDown(tester.view.reset);
        const String testString = 'How are you doing today? Good, and you?';

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Center(child: Text(testString)),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.textContaining('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();

        await gesture.moveTo(textOffsetToPosition(paragraph, testString.length));
        await tester.pumpAndSettle();
        expect(
          paragraph.selections[0],
          const TextSelection(baseOffset: 2, extentOffset: testString.length),
        );
        await gesture.up();
        await tester.pumpAndSettle();

        // Change the size of the window.
        tester.view.physicalSize = const Size(800.0, 400.0);
        await tester.pumpAndSettle();

        // Start a new drag.
        await gesture.down(textOffsetToPosition(paragraph, 0));
        await tester.pumpAndSettle();
        await gesture.up();
        await tester.pumpAndSettle(kDoubleTapTimeout);
        expect(paragraph.selections.isEmpty, isFalse);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 0));

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pumpAndSettle();

        // Select to the end.
        await gesture.moveTo(textOffsetToPosition(paragraph, testString.length));
        await tester.pump();
        expect(
          paragraph.selections[0],
          const TextSelection(baseOffset: 2, extentOffset: testString.length),
        );

        await gesture.up();
        await tester.pumpAndSettle();
      },
      variant: TargetPlatformVariant.all(),
      skip: kIsWeb, // https://github.com/flutter/flutter/issues/125582.
    );

    testWidgets('RenderParagraph should invalidate cached bounding boxes', (
      WidgetTester tester,
    ) async {
      final UniqueKey outerText = UniqueKey();
      addTearDown(tester.view.reset);

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: Scaffold(
              body: Center(child: Text('How are you doing today? Good, and you?', key: outerText)),
            ),
          ),
        ),
      );
      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
      );
      final SelectableRegionState state = tester.state<SelectableRegionState>(
        find.byType(SelectableRegion),
      );

      // Double click to select word at position.
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 27),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await gesture.down(textOffsetToPosition(paragraph, 27));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      // Should select "Good".
      expect(paragraph.selections[0], const TextSelection(baseOffset: 25, extentOffset: 29));

      // Change the size of the window.
      tester.view.physicalSize = const Size(800.0, 400.0);
      await tester.pumpAndSettle();
      state.clearSelection();
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(paragraph.selections.isEmpty, isTrue);

      // Double click at the same position.
      await gesture.down(textOffsetToPosition(paragraph, 27));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await gesture.down(textOffsetToPosition(paragraph, 27));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      // Should select "Good" again.
      expect(paragraph.selections.isEmpty, isFalse);
      expect(paragraph.selections[0], const TextSelection(baseOffset: 25, extentOffset: 29));
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/125582.

    testWidgets('mouse can select single text on desktop platforms', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Center(child: Text('How are you')),
          ),
        ),
      );
      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();

      await gesture.moveTo(textOffsetToPosition(paragraph, 4));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 2, extentOffset: 4));

      await gesture.moveTo(textOffsetToPosition(paragraph, 6));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 2, extentOffset: 6));

      // Check backward selection.
      await gesture.moveTo(textOffsetToPosition(paragraph, 1));
      await tester.pump();
      expect(paragraph.selections.isEmpty, isFalse);
      expect(paragraph.selections[0], const TextSelection(baseOffset: 2, extentOffset: 1));

      // Start a new drag.
      await gesture.up();
      await tester.pumpAndSettle();

      await gesture.down(textOffsetToPosition(paragraph, 5));
      await tester.pumpAndSettle();
      expect(paragraph.selections.isEmpty, isFalse);
      expect(paragraph.selections[0], const TextSelection.collapsed(offset: 5));

      // Selecting across line should select to the end.
      await gesture.moveTo(textOffsetToPosition(paragraph, 5) + const Offset(0.0, 200.0));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 5, extentOffset: 11));

      await gesture.up();
    }, variant: TargetPlatformVariant.desktop());

    testWidgets('mouse can select single text on mobile platforms', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Center(child: Text('How are you')),
          ),
        ),
      );
      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();

      await gesture.moveTo(textOffsetToPosition(paragraph, 4));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 2, extentOffset: 4));

      await gesture.moveTo(textOffsetToPosition(paragraph, 6));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 2, extentOffset: 6));

      // Check backward selection.
      await gesture.moveTo(textOffsetToPosition(paragraph, 1));
      await tester.pump();
      expect(paragraph.selections.isEmpty, isFalse);
      expect(paragraph.selections[0], const TextSelection(baseOffset: 2, extentOffset: 1));

      // Start a new drag.
      await gesture.up();
      await tester.pumpAndSettle();

      await gesture.down(textOffsetToPosition(paragraph, 5));
      await tester.pumpAndSettle();
      await gesture.moveTo(textOffsetToPosition(paragraph, 6));
      await tester.pump();
      expect(paragraph.selections.isEmpty, isFalse);
      expect(paragraph.selections[0], const TextSelection(baseOffset: 5, extentOffset: 6));

      // Selecting across line should select to the end.
      await gesture.moveTo(textOffsetToPosition(paragraph, 5) + const Offset(0.0, 200.0));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 5, extentOffset: 11));

      await gesture.up();
    }, variant: TargetPlatformVariant.mobile());

    testWidgets('mouse drag finalizes the selection', (WidgetTester tester) async {
      SelectableRegionSelectionStatus? selectionStatus;
      final GlobalKey textKey = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: Center(child: Text(key: textKey, 'How are you')),
          ),
        ),
      );
      await tester.pumpAndSettle();
      expect(textKey.currentContext, isNotNull);
      final ValueListenable<SelectableRegionSelectionStatus>? selectionStatusNotifier =
          SelectableRegionSelectionStatusScope.maybeOf(textKey.currentContext!);
      void onSelectionStatusChange() {
        selectionStatus = selectionStatusNotifier?.value;
      }

      selectionStatusNotifier?.addListener(onSelectionStatusChange);
      addTearDown(() {
        selectionStatusNotifier?.removeListener(onSelectionStatusChange);
      });
      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();

      await gesture.moveTo(textOffsetToPosition(paragraph, 4));
      await tester.pump();
      expect(selectionStatus, SelectableRegionSelectionStatus.changing);
      await gesture.up();
      await tester.pump();

      expect(paragraph.selections.length, 1);
      expect(selectionStatus, SelectableRegionSelectionStatus.finalized);
    }, variant: TargetPlatformVariant.all());

    testWidgets(
      'touch drag does not finalize selection on mobile platforms',
      (WidgetTester tester) async {
        SelectableRegionSelectionStatus? selectionStatus;
        final GlobalKey textKey = GlobalKey();
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Center(child: Text(key: textKey, 'How are you')),
            ),
          ),
        );
        await tester.pumpAndSettle();
        expect(textKey.currentContext, isNotNull);
        final ValueListenable<SelectableRegionSelectionStatus>? selectionStatusNotifier =
            SelectableRegionSelectionStatusScope.maybeOf(textKey.currentContext!);
        void onSelectionStatusChange() {
          selectionStatus = selectionStatusNotifier?.value;
        }

        selectionStatusNotifier?.addListener(onSelectionStatusChange);
        addTearDown(() {
          selectionStatusNotifier?.removeListener(onSelectionStatusChange);
        });
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph, 2));
        addTearDown(gesture.removePointer);
        await tester.pump();

        await gesture.moveTo(textOffsetToPosition(paragraph, 4));
        await tester.pump();
        await gesture.up();
        await tester.pump();

        expect(paragraph.selections.length, 0);
        expect(selectionStatus, isNull);
      },
      variant: TargetPlatformVariant.mobile(),
    );

    testWidgets('mouse can select word-by-word on double click drag', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Center(child: Text('How are you')),
          ),
        ),
      );
      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph, 2));
      await tester.pumpAndSettle();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

      await gesture.moveTo(textOffsetToPosition(paragraph, 3));
      await tester.pumpAndSettle();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 4));

      await gesture.moveTo(textOffsetToPosition(paragraph, 4));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));

      await gesture.moveTo(textOffsetToPosition(paragraph, 7));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 8));

      await gesture.moveTo(textOffsetToPosition(paragraph, 8));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));

      // Check backward selection.
      await gesture.moveTo(textOffsetToPosition(paragraph, 1));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

      // Start a new double-click drag.
      await gesture.up();
      await tester.pump();
      await gesture.down(textOffsetToPosition(paragraph, 5));
      await tester.pump();
      await gesture.up();
      expect(paragraph.selections.isEmpty, isFalse);
      expect(paragraph.selections[0], const TextSelection.collapsed(offset: 5));
      await tester.pump(kDoubleTapTimeout);

      // Double-click.
      await gesture.down(textOffsetToPosition(paragraph, 5));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await gesture.down(textOffsetToPosition(paragraph, 5));
      await tester.pumpAndSettle();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

      // Selecting across line should select to the end.
      await gesture.moveTo(textOffsetToPosition(paragraph, 5) + const Offset(0.0, 200.0));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 11));
      await gesture.up();
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/125582.

    testWidgets('mouse can select multiple widgets on double click drag', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph1, 2));
      await tester.pumpAndSettle();
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

      await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
      await tester.pump();
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
      // Should select the rest of paragraph 1.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));

      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));

      await gesture.up();
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/125582.

    testWidgets(
      'mouse can select multiple widgets on double click drag and return to origin word',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph1, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph1, 2));
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
        await tester.pump();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));

        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
        // Should select the rest of paragraph 1.
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));

        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));

        await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
        // Should clear the selection on paragraph 3.
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));
        expect(paragraph3.selections.isEmpty, isTrue);

        await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
        // Should clear the selection on paragraph 2.
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));
        expect(paragraph2.selections.isEmpty, isTrue);
        expect(paragraph3.selections.isEmpty, isTrue);

        await gesture.up();
      },
      // https://github.com/flutter/flutter/issues/125582.
      skip: kIsWeb,
    );

    testWidgets('mouse can reverse selection across multiple widgets on double click drag', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph3, 10),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph3, 10));
      await tester.pumpAndSettle();
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 11));

      await gesture.moveTo(textOffsetToPosition(paragraph3, 4));
      await tester.pump();
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 11, extentOffset: 4));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 11, extentOffset: 0));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 14, extentOffset: 5));

      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 11, extentOffset: 0));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 14, extentOffset: 0));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 12, extentOffset: 4));

      await gesture.up();
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/125582.

    testWidgets('mouse can select paragraph-by-paragraph on triple click drag', (
      WidgetTester tester,
    ) async {
      const String longText =
          'Hello world this is some long piece of text '
          'that will represent a long paragraph, when triple clicking this block '
          'of text all of it will be selected.\n'
          'This will be the start of a new line. When triple clicking this block '
          'of text all of it should be selected.';

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Center(child: Text(longText)),
          ),
        ),
      );
      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text(longText), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph, 2));
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph, 2));
      await tester.pumpAndSettle();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 150));

      await gesture.moveTo(textOffsetToPosition(paragraph, 155));
      await tester.pumpAndSettle();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 257));

      await gesture.moveTo(textOffsetToPosition(paragraph, 170));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 257));

      // Check backward selection.
      await gesture.moveTo(textOffsetToPosition(paragraph, 1));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 150));

      // Start a new triple-click drag.
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);
      await gesture.down(textOffsetToPosition(paragraph, 151));
      await tester.pumpAndSettle();
      await gesture.up();
      expect(paragraph.selections.isNotEmpty, isTrue);
      expect(paragraph.selections.length, 1);
      expect(paragraph.selections.first, const TextSelection.collapsed(offset: 151));
      await tester.pump(kDoubleTapTimeout);

      // Triple-click.
      await gesture.down(textOffsetToPosition(paragraph, 151));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await gesture.down(textOffsetToPosition(paragraph, 151));
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await gesture.down(textOffsetToPosition(paragraph, 151));
      await tester.pumpAndSettle();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 150, extentOffset: 257));

      // Selecting across line should select to the end.
      await gesture.moveTo(textOffsetToPosition(paragraph, 5) + const Offset(0.0, -200.0));
      await tester.pump();
      expect(paragraph.selections[0], const TextSelection(baseOffset: 257, extentOffset: 0));
      await gesture.up();
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/125582.

    testWidgets(
      'mouse can select multiple widgets on triple click drag when selecting inside a WidgetSpan',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Text.rich(
                WidgetSpan(
                  child: Column(
                    children: <Widget>[
                      Text('Text widget A.'),
                      Text('Text widget B.'),
                      Text('Text widget C.'),
                      Text('Text widget D.'),
                      Text('Text widget E.'),
                    ],
                  ),
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraphC = tester.renderObject<RenderParagraph>(
          find.descendant(
            of: find.textContaining('Text widget C.'),
            matching: find.byType(RichText),
          ),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraphC, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraphC, 2));
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraphC, 2));
        await tester.pumpAndSettle();
        expect(paragraphC.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));

        await gesture.moveTo(textOffsetToPosition(paragraphC, 7));
        await tester.pump();
        expect(paragraphC.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));

        final RenderParagraph paragraphE = tester.renderObject<RenderParagraph>(
          find.descendant(
            of: find.textContaining('Text widget E.'),
            matching: find.byType(RichText),
          ),
        );
        final RenderParagraph paragraphD = tester.renderObject<RenderParagraph>(
          find.descendant(
            of: find.textContaining('Text widget D.'),
            matching: find.byType(RichText),
          ),
        );
        await gesture.moveTo(textOffsetToPosition(paragraphE, 5));
        // Should select line C-E.
        expect(paragraphC.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraphD.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraphE.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));

        await gesture.up();
      },
      skip: kIsWeb, // https://github.com/flutter/flutter/issues/125582.
    );

    testWidgets('mouse can select multiple widgets on triple click drag', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?\nThis is the first text widget.'),
                Text('Good, and you?\nThis is the second text widget.'),
                Text('Fine, thank you.\nThis is the third text widget.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(
          of: find.textContaining('first text widget'),
          matching: find.byType(RichText),
        ),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph1, 2));
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph1, 2));
      await tester.pumpAndSettle();
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 13));

      await gesture.moveTo(textOffsetToPosition(paragraph1, 14));
      await tester.pump();
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 43));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(
          of: find.textContaining('second text widget'),
          matching: find.byType(RichText),
        ),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
      // Should select line 1 of text widget 2.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 43));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 15));

      await gesture.moveTo(textOffsetToPosition(paragraph2, 16));
      // Should select the rest of text widget 2.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 43));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 46));

      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(
          of: find.textContaining('third text widget'),
          matching: find.byType(RichText),
        ),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
      // Should select line 1 of text widget 3.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 43));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 46));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 17));

      await gesture.moveTo(textOffsetToPosition(paragraph3, 18));
      // Should select the rest of text widget 3.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 43));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 46));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 47));

      await gesture.up();
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/125582.

    testWidgets(
      'mouse can select multiple widgets on triple click drag and return to origin paragraph',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?\nThis is the first text widget.'),
                  Text('Good, and you?\nThis is the second text widget.'),
                  Text('Fine, thank you.\nThis is the third text widget.'),
                ],
              ),
            ),
          ),
        );
        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(
            of: find.textContaining('second text widget'),
            matching: find.byType(RichText),
          ),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph2, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph2, 2));
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph2, 2));
        await tester.pumpAndSettle();
        // Should select line 1 of text widget 2.
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 15));

        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(
            of: find.textContaining('first text widget'),
            matching: find.byType(RichText),
          ),
        );

        // Should select line 2 of text widget 1.
        await gesture.moveTo(textOffsetToPosition(paragraph1, 14));
        await tester.pump();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 43, extentOffset: 13));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 15, extentOffset: 0));

        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(
            of: find.textContaining('third text widget'),
            matching: find.byType(RichText),
          ),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph1, 5));
        // Should select rest of text widget 1.
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 43, extentOffset: 0));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 15, extentOffset: 0));

        await gesture.moveTo(textOffsetToPosition(paragraph2, 2));
        // Should clear the selection on paragraph 1 and return to the origin paragraph.
        expect(paragraph1.selections.isEmpty, true);
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 15, extentOffset: 0));

        await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
        // Should select line 1 of text widget 3.
        expect(paragraph1.selections.isEmpty, true);
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 46));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 17));

        await gesture.moveTo(textOffsetToPosition(paragraph3, 18));
        // Should select line 2 of text widget 3.
        expect(paragraph1.selections.isEmpty, true);
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 46));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 47));

        await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
        // Should clear the selection on paragraph 3 and return to the origin paragraph.
        expect(paragraph1.selections.isEmpty, true);
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 15));
        expect(paragraph3.selections.isEmpty, true);

        await gesture.up();
      },
      skip: kIsWeb, // https://github.com/flutter/flutter/issues/125582.
    );

    testWidgets('mouse can reverse selection across multiple widgets on triple click drag', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?\nThis is the first text widget.'),
                Text('Good, and you?\nThis is the second text widget.'),
                Text('Fine, thank you.\nThis is the third text widget.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(
          of: find.textContaining('Fine, thank you.'),
          matching: find.byType(RichText),
        ),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph3, 18),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph3, 18));
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(textOffsetToPosition(paragraph3, 18));
      await tester.pumpAndSettle();
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 17, extentOffset: 47));

      await gesture.moveTo(textOffsetToPosition(paragraph3, 4));
      await tester.pump();
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 47, extentOffset: 0));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.textContaining('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 47, extentOffset: 0));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 46, extentOffset: 0));

      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.textContaining('How are you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 47, extentOffset: 0));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 46, extentOffset: 0));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 43, extentOffset: 0));

      await gesture.up();
    }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/125582.

    testWidgets('mouse can select multiple widgets', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();

      await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
      await tester.pump();
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 4));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
      // Should select the rest of paragraph 1.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 12));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));

      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 12));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));

      await gesture.up();
    });

    testWidgets(
      'mouse shift + click holds the selection start in place and moves the end',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph1, 9),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection.collapsed(offset: 9));

        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        await gesture.down(textOffsetToPosition(paragraph2, 5));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 9, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));

        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        await gesture.down(textOffsetToPosition(paragraph3, 13));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 9, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 13));

        await gesture.down(textOffsetToPosition(paragraph1, 4));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 9, extentOffset: 4));
        expect(paragraph2.selections.isEmpty, isTrue);
        expect(paragraph3.selections.isEmpty, isTrue);

        await gesture.down(textOffsetToPosition(paragraph1, 0));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 9, extentOffset: 0));
        expect(paragraph2.selections.isEmpty, isTrue);
        expect(paragraph3.selections.isEmpty, isTrue);
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      },
      variant: TargetPlatformVariant.desktop(),
    );

    testWidgets(
      'mouse shift + click collapses the selection when it has not been initialized',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph1, 9),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph1.selections[0], const TextSelection.collapsed(offset: 9));
        expect(paragraph2.selections.isEmpty, isTrue);
        expect(paragraph3.selections.isEmpty, isTrue);
        await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      },
      variant: TargetPlatformVariant.desktop(),
    );

    testWidgets('collapsing selection should clear selection of all other selectables', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(paragraph1.selections[0], const TextSelection.collapsed(offset: 2));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.down(textOffsetToPosition(paragraph2, 5));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(paragraph1.selections.isEmpty, isTrue);
      expect(paragraph2.selections[0], const TextSelection.collapsed(offset: 5));

      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      await gesture.down(textOffsetToPosition(paragraph3, 13));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      expect(paragraph1.selections.isEmpty, isTrue);
      expect(paragraph2.selections.isEmpty, isTrue);
      expect(paragraph3.selections[0], const TextSelection.collapsed(offset: 13));
    });

    testWidgets('mouse can work with disabled container', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                SelectionContainer.disabled(child: Text('Good, and you?')),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();

      await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
      await tester.pump();
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 4));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
      // Should select the rest of paragraph 1.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 12));
      // paragraph2 is in a disabled container.
      expect(paragraph2.selections.isEmpty, isTrue);

      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 12));
      expect(paragraph2.selections.isEmpty, isTrue);
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));

      await gesture.up();
    });

    testWidgets('mouse can reverse selection', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph3, 10),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();

      await gesture.moveTo(textOffsetToPosition(paragraph3, 4));
      await tester.pump();
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 10, extentOffset: 4));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 10, extentOffset: 0));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 14, extentOffset: 5));

      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 10, extentOffset: 0));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 14, extentOffset: 0));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 12, extentOffset: 6));

      await gesture.up();
    });

    testWidgets(
      'long press selection overlay behavior on iOS and Android',
      (WidgetTester tester) async {
        // This test verifies that all platforms wait until long press end to
        // show the context menu, and only Android waits until long press end to
        // show the selection handles.
        final bool isPlatformAndroid = defaultTargetPlatform == TargetPlatform.android;
        Set<ContextMenuButtonType> buttonTypes = <ContextMenuButtonType>{};
        final UniqueKey toolbarKey = UniqueKey();
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionHandleControls,
              contextMenuBuilder: (
                BuildContext context,
                SelectableRegionState selectableRegionState,
              ) {
                buttonTypes =
                    selectableRegionState.contextMenuButtonItems
                        .map((ContextMenuButtonItem buttonItem) => buttonItem.type)
                        .toSet();
                return SizedBox.shrink(key: toolbarKey);
              },
              child: const Text('How are you?'),
            ),
          ),
        );

        expect(buttonTypes.isEmpty, true);
        expect(find.byKey(toolbarKey), findsNothing);

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph, 2));
        addTearDown(gesture.removePointer);
        await tester.pump(const Duration(milliseconds: 500));
        await tester.pumpAndSettle();

        // All platform except Android should show the selection handles when the
        // long press starts.
        List<FadeTransition> transitions =
            find
                .descendant(
                  of: find.byWidgetPredicate(
                    (Widget w) => '${w.runtimeType}' == '_SelectionHandleOverlay',
                  ),
                  matching: find.byType(FadeTransition),
                )
                .evaluate()
                .map((Element e) => e.widget)
                .cast<FadeTransition>()
                .toList();
        expect(transitions.length, isPlatformAndroid ? 0 : 2);
        FadeTransition? left;
        FadeTransition? right;
        if (!isPlatformAndroid) {
          left = transitions[0];
          right = transitions[1];
          expect(left.opacity.value, equals(1.0));
          expect(right.opacity.value, equals(1.0));
        }
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));
        expect(find.byKey(toolbarKey), findsNothing);

        await gesture.moveTo(textOffsetToPosition(paragraph, 8));
        await tester.pumpAndSettle();
        transitions =
            find
                .descendant(
                  of: find.byWidgetPredicate(
                    (Widget w) => '${w.runtimeType}' == '_SelectionHandleOverlay',
                  ),
                  matching: find.byType(FadeTransition),
                )
                .evaluate()
                .map((Element e) => e.widget)
                .cast<FadeTransition>()
                .toList();
        // All platform except Android should show the selection handles while doing
        // a long press drag.
        expect(transitions.length, isPlatformAndroid ? 0 : 2);
        if (!isPlatformAndroid) {
          left = transitions[0];
          right = transitions[1];
          expect(left.opacity.value, equals(1.0));
          expect(right.opacity.value, equals(1.0));
        }
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));
        expect(find.byKey(toolbarKey), findsNothing);

        await gesture.up();
        await tester.pumpAndSettle();
        transitions =
            find
                .descendant(
                  of: find.byWidgetPredicate(
                    (Widget w) => '${w.runtimeType}' == '_SelectionHandleOverlay',
                  ),
                  matching: find.byType(FadeTransition),
                )
                .evaluate()
                .map((Element e) => e.widget)
                .cast<FadeTransition>()
                .toList();
        expect(transitions.length, 2);
        left = transitions[0];
        right = transitions[1];

        // All platforms should show the selection handles and context menu when
        // the long press ends.
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));
        expect(left.opacity.value, equals(1.0));
        expect(right.opacity.value, equals(1.0));
        expect(find.byKey(toolbarKey), findsOneWidget);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.iOS,
      }),
      skip: kIsWeb, // [intended] Web uses its native context menu.
    );

    testWidgets(
      'single tap on the previous selection toggles the toolbar on iOS',
      (WidgetTester tester) async {
        Set<ContextMenuButtonType> buttonTypes = <ContextMenuButtonType>{};
        final UniqueKey toolbarKey = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionHandleControls,
              contextMenuBuilder: (
                BuildContext context,
                SelectableRegionState selectableRegionState,
              ) {
                buttonTypes =
                    selectableRegionState.contextMenuButtonItems
                        .map((ContextMenuButtonItem buttonItem) => buttonItem.type)
                        .toSet();
                return SizedBox.shrink(key: toolbarKey);
              },
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );

        expect(buttonTypes.isEmpty, true);
        expect(find.byKey(toolbarKey), findsNothing);

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(textOffsetToPosition(paragraph, 2));
        addTearDown(gesture.removePointer);
        await tester.pump(const Duration(milliseconds: 500));
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));
        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));
        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsNothing);

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));
        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Collapse selection.
        await tester.tapAt(textOffsetToPosition(paragraph, 9));
        await tester.pump();
        expect(paragraph.selections.isEmpty, isFalse);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 9));
        expect(find.byKey(toolbarKey), findsNothing);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.iOS),
      skip: kIsWeb, // [intended] Web uses its native context menu.
    );

    testWidgets(
      'right-click mouse can select word at position on Apple platforms',
      (WidgetTester tester) async {
        Set<ContextMenuButtonType> buttonTypes = <ContextMenuButtonType>{};
        final UniqueKey toolbarKey = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionHandleControls,
              contextMenuBuilder: (
                BuildContext context,
                SelectableRegionState selectableRegionState,
              ) {
                buttonTypes =
                    selectableRegionState.contextMenuButtonItems
                        .map((ContextMenuButtonItem buttonItem) => buttonItem.type)
                        .toSet();
                return SizedBox.shrink(key: toolbarKey);
              },
              child: const Center(child: Text('How are you')),
            ),
          ),
        );

        expect(buttonTypes.isEmpty, true);
        expect(find.byKey(toolbarKey), findsNothing);

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture primaryMouseButtonGesture = await tester.createGesture(
          kind: PointerDeviceKind.mouse,
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        addTearDown(primaryMouseButtonGesture.removePointer);
        addTearDown(gesture.removePointer);
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.up();
        await tester.pump();

        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        await gesture.down(textOffsetToPosition(paragraph, 6));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

        await gesture.up();
        await tester.pump();

        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        await gesture.down(textOffsetToPosition(paragraph, 9));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 8, extentOffset: 11));

        await gesture.up();
        await tester.pump();

        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Collapse selection.
        await primaryMouseButtonGesture.down(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        await primaryMouseButtonGesture.up();
        await tester.pumpAndSettle();
        // Selection is collapsed.
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 1));
        expect(find.byKey(toolbarKey), findsNothing);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
      skip: kIsWeb, // [intended] Web uses its native context menu.
    );

    testWidgets(
      'right-click mouse on an active selection does not clear the selection in other selectables on Apple platforms',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/150268.
        Set<ContextMenuButtonType> buttonTypes = <ContextMenuButtonType>{};
        final UniqueKey toolbarKey = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionHandleControls,
              contextMenuBuilder: (
                BuildContext context,
                SelectableRegionState selectableRegionState,
              ) {
                buttonTypes =
                    selectableRegionState.contextMenuButtonItems
                        .map((ContextMenuButtonItem buttonItem) => buttonItem.type)
                        .toSet();
                return SizedBox.shrink(key: toolbarKey);
              },
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );

        expect(buttonTypes.isEmpty, true);
        expect(find.byKey(toolbarKey), findsNothing);

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
        );
        final TestGesture secondaryMouseButtonGesture = await tester.createGesture(
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        addTearDown(secondaryMouseButtonGesture.removePointer);
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.moveTo(textOffsetToPosition(paragraph3, 5));
        await tester.pump();
        await gesture.up();
        await tester.pumpAndSettle();
        expect(paragraph.selections, isNotEmpty);
        expect(paragraph2.selections, isNotEmpty);
        expect(paragraph3.selections, isNotEmpty);
        expect(paragraph.selections[0], const TextSelection(baseOffset: 2, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));

        // Right-clicking on the active selection should retain the selection.
        await secondaryMouseButtonGesture.down(textOffsetToPosition(paragraph2, 7));
        await tester.pump();
        await secondaryMouseButtonGesture.up();
        await tester.pumpAndSettle();
        expect(paragraph.selections, isNotEmpty);
        expect(paragraph2.selections, isNotEmpty);
        expect(paragraph3.selections, isNotEmpty);
        expect(paragraph.selections[0], const TextSelection(baseOffset: 2, extentOffset: 12));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));

        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
      skip: kIsWeb, // [intended] Web uses its native context menu.
    );

    testWidgets(
      'right-click mouse at the same position as previous right-click toggles the context menu on macOS',
      (WidgetTester tester) async {
        Set<ContextMenuButtonType> buttonTypes = <ContextMenuButtonType>{};
        final UniqueKey toolbarKey = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionHandleControls,
              contextMenuBuilder: (
                BuildContext context,
                SelectableRegionState selectableRegionState,
              ) {
                buttonTypes =
                    selectableRegionState.contextMenuButtonItems
                        .map((ContextMenuButtonItem buttonItem) => buttonItem.type)
                        .toSet();
                return SizedBox.shrink(key: toolbarKey);
              },
              child: const Center(child: Text('How are you')),
            ),
          ),
        );

        expect(buttonTypes.isEmpty, true);
        expect(find.byKey(toolbarKey), findsNothing);

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        final TestGesture primaryMouseButtonGesture = await tester.createGesture(
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(primaryMouseButtonGesture.removePointer);
        addTearDown(gesture.removePointer);
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.up();
        await tester.pump();

        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

        await gesture.up();
        await tester.pump();

        // Right-click at same position will toggle the context menu off.
        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsNothing);

        await gesture.down(textOffsetToPosition(paragraph, 9));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 8, extentOffset: 11));

        await gesture.up();
        await tester.pump();

        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        await gesture.down(textOffsetToPosition(paragraph, 9));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 8, extentOffset: 11));

        await gesture.up();
        await tester.pump();

        // Right-click at same position will toggle the context menu off.
        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsNothing);

        await gesture.down(textOffsetToPosition(paragraph, 6));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

        await gesture.up();
        await tester.pump();

        expect(buttonTypes, contains(ContextMenuButtonType.copy));
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Collapse selection.
        await primaryMouseButtonGesture.down(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        await primaryMouseButtonGesture.up();
        await tester.pumpAndSettle();
        // Selection is collapsed.
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 1));
        expect(find.byKey(toolbarKey), findsNothing);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.macOS),
      skip: kIsWeb, // [intended] Web uses its native context menu.
    );

    testWidgets(
      'right-click mouse shows the context menu at position on Android, Fuchsia, and Windows',
      (WidgetTester tester) async {
        Set<ContextMenuButtonType> buttonTypes = <ContextMenuButtonType>{};
        final UniqueKey toolbarKey = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionHandleControls,
              contextMenuBuilder: (
                BuildContext context,
                SelectableRegionState selectableRegionState,
              ) {
                buttonTypes =
                    selectableRegionState.contextMenuButtonItems
                        .map((ContextMenuButtonItem buttonItem) => buttonItem.type)
                        .toSet();
                return SizedBox.shrink(key: toolbarKey);
              },
              child: const Center(child: Text('How are you')),
            ),
          ),
        );

        expect(buttonTypes.isEmpty, true);
        expect(find.byKey(toolbarKey), findsNothing);

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        final TestGesture primaryMouseButtonGesture = await tester.createGesture(
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(primaryMouseButtonGesture.removePointer);
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        // Selection is collapsed.
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 2));
        expect(buttonTypes.length, 1);
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        await gesture.down(textOffsetToPosition(paragraph, 6));
        await tester.pump();
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 6));

        await gesture.up();
        await tester.pump();

        expect(buttonTypes.length, 1);
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        await gesture.down(textOffsetToPosition(paragraph, 9));
        await tester.pump();
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 9));

        await gesture.up();
        await tester.pump();

        expect(buttonTypes.length, 1);
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Collapse selection.
        await primaryMouseButtonGesture.down(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        await primaryMouseButtonGesture.up();
        await tester.pumpAndSettle(kDoubleTapTimeout);
        // Selection is collapsed.
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 1));
        expect(find.byKey(toolbarKey), findsNothing);

        // Create an uncollapsed selection by dragging.
        await primaryMouseButtonGesture.down(textOffsetToPosition(paragraph, 0));
        await tester.pump();
        await primaryMouseButtonGesture.moveTo(textOffsetToPosition(paragraph, 5));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));
        await primaryMouseButtonGesture.up();
        await tester.pump();

        // Right click on previous selection should not collapse the selection.
        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Right click anywhere outside previous selection should collapse the
        // selection.
        await gesture.down(textOffsetToPosition(paragraph, 7));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 7));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Collapse selection.
        await primaryMouseButtonGesture.down(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        await primaryMouseButtonGesture.up();
        await tester.pumpAndSettle();
        // Selection is collapsed.
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 1));
        expect(find.byKey(toolbarKey), findsNothing);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.fuchsia,
        TargetPlatform.windows,
      }),
      skip: kIsWeb, // [intended] Web uses its native context menu.
    );

    testWidgets(
      'right-click mouse toggles the context menu on Linux',
      (WidgetTester tester) async {
        Set<ContextMenuButtonType> buttonTypes = <ContextMenuButtonType>{};
        final UniqueKey toolbarKey = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionHandleControls,
              contextMenuBuilder: (
                BuildContext context,
                SelectableRegionState selectableRegionState,
              ) {
                buttonTypes =
                    selectableRegionState.contextMenuButtonItems
                        .map((ContextMenuButtonItem buttonItem) => buttonItem.type)
                        .toSet();
                return SizedBox.shrink(key: toolbarKey);
              },
              child: const Center(child: Text('How are you')),
            ),
          ),
        );

        expect(buttonTypes.isEmpty, true);
        expect(find.byKey(toolbarKey), findsNothing);

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        final TestGesture primaryMouseButtonGesture = await tester.createGesture(
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(primaryMouseButtonGesture.removePointer);
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        // Selection is collapsed.
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 2));

        // Context menu toggled on.
        expect(buttonTypes.length, 1);
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        await gesture.down(textOffsetToPosition(paragraph, 6));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 2));

        // Context menu toggled off. Selection remains the same.
        expect(find.byKey(toolbarKey), findsNothing);

        await gesture.down(textOffsetToPosition(paragraph, 9));
        await tester.pump();
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 9));

        await gesture.up();
        await tester.pump();

        // Context menu toggled on.
        expect(buttonTypes.length, 1);
        expect(buttonTypes, contains(ContextMenuButtonType.selectAll));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Collapse selection.
        await primaryMouseButtonGesture.down(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        await primaryMouseButtonGesture.up();
        await tester.pumpAndSettle(kDoubleTapTimeout);
        // Selection is collapsed.
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 1));
        expect(find.byKey(toolbarKey), findsNothing);

        await primaryMouseButtonGesture.down(textOffsetToPosition(paragraph, 0));
        await tester.pump();
        await primaryMouseButtonGesture.moveTo(textOffsetToPosition(paragraph, 5));
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));
        await primaryMouseButtonGesture.up();
        await tester.pump();

        // Right click on previous selection should not collapse the selection.
        await gesture.down(textOffsetToPosition(paragraph, 2));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Right click anywhere outside previous selection should first toggle the context
        // menu off.
        await gesture.down(textOffsetToPosition(paragraph, 7));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(paragraph.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));
        expect(find.byKey(toolbarKey), findsNothing);

        // Right click again should collapse the selection and toggle the context
        // menu on.
        await gesture.down(textOffsetToPosition(paragraph, 7));
        await tester.pump();
        await gesture.up();
        await tester.pump();
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 7));
        expect(find.byKey(toolbarKey), findsOneWidget);

        // Collapse selection.
        await primaryMouseButtonGesture.down(textOffsetToPosition(paragraph, 1));
        await tester.pump();
        await primaryMouseButtonGesture.up();
        await tester.pumpAndSettle();
        // Selection is collapsed.
        expect(paragraph.selections.isEmpty, false);
        expect(paragraph.selections[0], const TextSelection.collapsed(offset: 1));
        expect(find.byKey(toolbarKey), findsNothing);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.linux),
      skip: kIsWeb, // [intended] Web uses its native context menu.
    );

    testWidgets(
      'can copy a selection made with the mouse',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        // Select from offset 2 of paragraph 1 to offset 6 of paragraph3.
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph1, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();

        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
        await gesture.up();

        // keyboard copy.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyC, control: true),
        );

        final Map<String, dynamic> clipboardData =
            mockClipboard.clipboardData as Map<String, dynamic>;
        expect(clipboardData['text'], 'w are you?Good, and you?Fine, ');
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.windows,
        TargetPlatform.linux,
        TargetPlatform.fuchsia,
      }),
    );

    testWidgets(
      'does not override TextField keyboard shortcuts if the TextField is focused - non apple',
      (WidgetTester tester) async {
        final TextEditingController controller = TextEditingController(
          text: 'I am fine, thank you.',
        );
        addTearDown(controller.dispose);
        final FocusNode selectableRegionFocus = FocusNode();
        addTearDown(selectableRegionFocus.dispose);
        final FocusNode textFieldFocus = FocusNode();
        addTearDown(textFieldFocus.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: SelectableRegion(
                focusNode: selectableRegionFocus,
                selectionControls: materialTextSelectionControls,
                child: Column(
                  children: <Widget>[
                    const Text('How are you?'),
                    const Text('Good, and you?'),
                    TextField(controller: controller, focusNode: textFieldFocus),
                  ],
                ),
              ),
            ),
          ),
        );
        textFieldFocus.requestFocus();
        await tester.pump();

        // Make sure keyboard select all works on TextField.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyA, control: true),
        );
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 21));

        // Make sure no selection in SelectableRegion.
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        expect(paragraph1.selections.isEmpty, isTrue);
        expect(paragraph2.selections.isEmpty, isTrue);

        // Focus selectable region.
        selectableRegionFocus.requestFocus();
        await tester.pump();

        // Reset controller selection once the TextField is unfocused.
        controller.selection = const TextSelection.collapsed(offset: -1);

        // Make sure keyboard select all will be handled by selectable region now.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyA, control: true),
        );
        expect(controller.selection, const TextSelection.collapsed(offset: -1));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.windows,
        TargetPlatform.linux,
        TargetPlatform.fuchsia,
      }),
      skip: kIsWeb, // [intended] the web handles this on its own.
    );

    testWidgets(
      'does not override TextField keyboard shortcuts if the TextField is focused - apple',
      (WidgetTester tester) async {
        final TextEditingController controller = TextEditingController(
          text: 'I am fine, thank you.',
        );
        addTearDown(controller.dispose);
        final FocusNode selectableRegionFocus = FocusNode();
        addTearDown(selectableRegionFocus.dispose);
        final FocusNode textFieldFocus = FocusNode();
        addTearDown(textFieldFocus.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: SelectableRegion(
                focusNode: selectableRegionFocus,
                selectionControls: materialTextSelectionControls,
                child: Column(
                  children: <Widget>[
                    const Text('How are you?'),
                    const Text('Good, and you?'),
                    TextField(controller: controller, focusNode: textFieldFocus),
                  ],
                ),
              ),
            ),
          ),
        );
        textFieldFocus.requestFocus();
        await tester.pump();

        // Make sure keyboard select all works on TextField.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyA, meta: true),
        );
        expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 21));

        // Make sure no selection in SelectableRegion.
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        expect(paragraph1.selections.isEmpty, isTrue);
        expect(paragraph2.selections.isEmpty, isTrue);

        // Focus selectable region.
        selectableRegionFocus.requestFocus();
        await tester.pump();

        // Reset controller selection once the TextField is unfocused.
        controller.selection = const TextSelection.collapsed(offset: -1);

        // Make sure keyboard select all will be handled by selectable region now.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyA, meta: true),
        );
        expect(controller.selection, const TextSelection.collapsed(offset: -1));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
      skip: kIsWeb, // [intended] the web handles this on its own.
    );

    testWidgets(
      'select all',
      (WidgetTester tester) async {
        final FocusNode focusNode = FocusNode();
        addTearDown(focusNode.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              focusNode: focusNode,
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        await tester.pumpAndSettle();
        focusNode.requestFocus();

        // keyboard select all.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyA, control: true),
        );

        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 16));
        expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
        expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.windows,
        TargetPlatform.linux,
        TargetPlatform.fuchsia,
      }),
    );

    testWidgets(
      'mouse selection can handle widget span',
      (WidgetTester tester) async {
        final UniqueKey outerText = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Center(
                child: Text.rich(
                  const TextSpan(
                    children: <InlineSpan>[
                      TextSpan(text: 'How are you?'),
                      WidgetSpan(child: Text('Good, and you?')),
                      TextSpan(text: 'Fine, thank you.'),
                    ],
                  ),
                  key: outerText,
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.moveTo(textOffsetToPosition(paragraph, 17)); // right after `Fine`.
        await gesture.up();

        // keyboard copy.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyC, control: true),
        );
        final Map<String, dynamic> clipboardData =
            mockClipboard.clipboardData as Map<String, dynamic>;
        expect(clipboardData['text'], 'w are you?Good, and you?Fine');
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.windows,
        TargetPlatform.linux,
        TargetPlatform.fuchsia,
      }),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets(
      'double click + drag mouse selection can handle widget span',
      (WidgetTester tester) async {
        final UniqueKey outerText = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Center(
                child: Text.rich(
                  const TextSpan(
                    children: <InlineSpan>[
                      TextSpan(text: 'How are you?'),
                      WidgetSpan(child: Text('Good, and you?')),
                      TextSpan(text: 'Fine, thank you.'),
                    ],
                  ),
                  key: outerText,
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 0),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph, 0));
        await tester.pump();
        await gesture.moveTo(textOffsetToPosition(paragraph, 17)); // right after `Fine`.
        await gesture.up();

        // keyboard copy.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyC, control: true),
        );
        final Map<String, dynamic> clipboardData =
            mockClipboard.clipboardData as Map<String, dynamic>;
        expect(clipboardData['text'], 'How are you?Good, and you?Fine,');
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.windows,
        TargetPlatform.linux,
        TargetPlatform.fuchsia,
      }),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets(
      'double click + drag mouse selection can handle widget span - multiline',
      (WidgetTester tester) async {
        final UniqueKey outerText = UniqueKey();
        final UniqueKey innerText = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Center(
                child: Text.rich(
                  TextSpan(
                    children: <InlineSpan>[
                      const TextSpan(text: 'How are you\n?'),
                      WidgetSpan(child: Text('Good, and you?', key: innerText)),
                      const TextSpan(text: 'Fine, thank you.'),
                    ],
                  ),
                  key: outerText,
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
        );
        final RenderParagraph innerParagraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(innerText), matching: find.byType(RichText)).first,
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 0),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        await gesture.down(textOffsetToPosition(paragraph, 0));
        await tester.pump();
        await gesture.moveTo(textOffsetToPosition(innerParagraph, 2)); // on `Good`.

        // Should not crash.
        expect(tester.takeException(), isNull);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.windows,
        TargetPlatform.linux,
        TargetPlatform.fuchsia,
      }),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets(
      'select word event can select inline widget',
      (WidgetTester tester) async {
        final UniqueKey outerText = UniqueKey();
        final UniqueKey innerText = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Center(
                child: Text.rich(
                  TextSpan(
                    children: <InlineSpan>[
                      const TextSpan(text: 'How are\n you?'),
                      WidgetSpan(child: Text('Good, and you?', key: innerText)),
                      const TextSpan(text: 'Fine, thank you.'),
                    ],
                  ),
                  key: outerText,
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
        );
        final RenderParagraph innerParagraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(innerText), matching: find.byType(RichText)).first,
        );
        final TestGesture gesture = await tester.startGesture(
          tester.getCenter(find.byKey(innerText)),
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        // Should select "and".
        expect(paragraph.selections.isEmpty, isTrue);
        expect(innerParagraph.selections[0], const TextSelection(baseOffset: 6, extentOffset: 9));
      },
      variant: TargetPlatformVariant.only(TargetPlatform.macOS),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets(
      'select word event should not crash when its position is at an unselectable inline element',
      (WidgetTester tester) async {
        final FocusNode focusNode = FocusNode();
        final UniqueKey flutterLogo = UniqueKey();
        addTearDown(focusNode.dispose);

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Scaffold(
                body: Center(
                  child: Text.rich(
                    TextSpan(
                      children: <InlineSpan>[
                        const TextSpan(
                          text:
                              'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.',
                        ),
                        WidgetSpan(child: FlutterLogo(key: flutterLogo)),
                        const TextSpan(text: 'Hello, world.'),
                      ],
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
        final Offset gestureOffset = tester.getCenter(find.byKey(flutterLogo).first);

        // Right click on unselectable element.
        final TestGesture gesture = await tester.startGesture(
          gestureOffset,
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();

        // Should not crash.
        expect(tester.takeException(), isNull);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.macOS),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets(
      'can select word when a selectables rect is completely inside of another selectables rect',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/127076.
        final UniqueKey outerText = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Scaffold(
                body: Center(
                  child: Text.rich(
                    const TextSpan(
                      children: <InlineSpan>[
                        TextSpan(
                          text:
                              'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.',
                        ),
                        WidgetSpan(child: Text('Some text in a WidgetSpan. ')),
                        TextSpan(text: 'Hello, world.'),
                      ],
                    ),
                    key: outerText,
                  ),
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
        );

        // Right click to select word at position.
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 125),
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        // Should select "Hello".
        expect(paragraph.selections[0], const TextSelection(baseOffset: 124, extentOffset: 129));
      },
      variant: TargetPlatformVariant.only(TargetPlatform.macOS),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets(
      'can select word when selectable is broken up by an unselectable WidgetSpan',
      (WidgetTester tester) async {
        final UniqueKey outerText = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Scaffold(
                body: Center(
                  child: Text.rich(
                    const TextSpan(
                      children: <InlineSpan>[
                        TextSpan(
                          text:
                              'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.',
                        ),
                        WidgetSpan(child: SizedBox.shrink()),
                        TextSpan(text: 'Hello, world.'),
                      ],
                    ),
                    key: outerText,
                  ),
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
        );

        // Right click to select word at position.
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 125),
          kind: PointerDeviceKind.mouse,
          buttons: kSecondaryMouseButton,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.up();
        await tester.pump();
        // Should select "Hello".
        expect(paragraph.selections[0], const TextSelection(baseOffset: 124, extentOffset: 129));
      },
      variant: TargetPlatformVariant.only(TargetPlatform.macOS),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets(
      'widget span is ignored if it does not contain text - non Apple',
      (WidgetTester tester) async {
        final UniqueKey outerText = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Center(
                child: Text.rich(
                  const TextSpan(
                    children: <InlineSpan>[
                      TextSpan(text: 'How are you?'),
                      WidgetSpan(child: Placeholder()),
                      TextSpan(text: 'Fine, thank you.'),
                    ],
                  ),
                  key: outerText,
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.moveTo(textOffsetToPosition(paragraph, 17)); // right after `Fine`.
        await gesture.up();

        // keyboard copy.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyC, control: true),
        );
        final Map<String, dynamic> clipboardData =
            mockClipboard.clipboardData as Map<String, dynamic>;
        expect(clipboardData['text'], 'w are you?Fine');
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.windows,
        TargetPlatform.linux,
        TargetPlatform.fuchsia,
      }),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets(
      'widget span is ignored if it does not contain text - Apple',
      (WidgetTester tester) async {
        final UniqueKey outerText = UniqueKey();

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: Center(
                child: Text.rich(
                  const TextSpan(
                    children: <InlineSpan>[
                      TextSpan(text: 'How are you?'),
                      WidgetSpan(child: Placeholder()),
                      TextSpan(text: 'Fine, thank you.'),
                    ],
                  ),
                  key: outerText,
                ),
              ),
            ),
          ),
        );
        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.moveTo(textOffsetToPosition(paragraph, 17)); // right after `Fine`.
        await gesture.up();

        // keyboard copy.
        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.keyC, meta: true),
        );
        final Map<String, dynamic> clipboardData =
            mockClipboard.clipboardData as Map<String, dynamic>;
        expect(clipboardData['text'], 'w are you?Fine');
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
      skip: isBrowser, // https://github.com/flutter/flutter/issues/61020
    );

    testWidgets('mouse can select across bidi text', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('جيد وانت؟', textDirection: TextDirection.rtl),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();

      await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
      await tester.pump();
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 4));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('جيد وانت؟'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
      // Should select the rest of paragraph 1.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 12));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 5));

      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      // Add a little offset to cross the boundary between paragraph 2 and 3.
      await gesture.moveTo(textOffsetToPosition(paragraph3, 6) + const Offset(0, 1));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 2, extentOffset: 12));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 9));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));

      await gesture.up();
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/61020

    testWidgets('long press and drag touch moves selection word by word', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 6),
      ); // at the 'r'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      // `are` is selected.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(textOffsetToPosition(paragraph2, 7));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 4, extentOffset: 12));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 9));
      await gesture.up();
    });

    testWidgets('can drag end handle when not covering entire screen', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/104620.

      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              const Text('How are you?'),
              SelectableRegion(
                selectionControls: materialTextSelectionControls,
                child: const Text('Good, and you?'),
              ),
              const Text('Fine, thank you.'),
            ],
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph2, 7),
      ); // at the 'a'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 500));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 6, extentOffset: 9));
      final List<TextBox> boxes = paragraph2.getBoxesForSelection(paragraph2.selections[0]);
      expect(boxes.length, 1);

      final Offset handlePos = globalize(boxes[0].toRect().bottomRight, paragraph2);
      await gesture.down(handlePos);

      await gesture.moveTo(
        textOffsetToPosition(paragraph2, 11) + Offset(0, paragraph2.size.height / 2),
      );
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 6, extentOffset: 11));
      await gesture.up();
    });

    testWidgets('can drag start handle when not covering entire screen', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/104620.

      await tester.pumpWidget(
        MaterialApp(
          home: Column(
            children: <Widget>[
              const Text('How are you?'),
              SelectableRegion(
                selectionControls: materialTextSelectionControls,
                child: const Text('Good, and you?'),
              ),
              const Text('Fine, thank you.'),
            ],
          ),
        ),
      );
      await tester.pumpAndSettle();
      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph2, 7),
      ); // at the 'a'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 500));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 6, extentOffset: 9));
      final List<TextBox> boxes = paragraph2.getBoxesForSelection(paragraph2.selections[0]);
      expect(boxes.length, 1);

      final Offset handlePos = globalize(boxes[0].toRect().bottomLeft, paragraph2);
      await gesture.down(handlePos);

      await gesture.moveTo(
        textOffsetToPosition(paragraph2, 11) + Offset(0, paragraph2.size.height / 2),
      );
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 11, extentOffset: 9));
      await gesture.up();
    });

    testWidgets('can drag start selection handle', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph3, 7),
      ); // at the 'h'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 500));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 11));
      final List<TextBox> boxes = paragraph3.getBoxesForSelection(paragraph3.selections[0]);
      expect(boxes.length, 1);

      final Offset handlePos = globalize(boxes[0].toRect().bottomLeft, paragraph3);
      await gesture.down(handlePos);
      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(
        textOffsetToPosition(paragraph2, 5) + Offset(0, paragraph2.size.height / 2),
      );
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 5, extentOffset: 14));

      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      await gesture.moveTo(
        textOffsetToPosition(paragraph1, 6) + Offset(0, paragraph1.size.height / 2),
      );
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 6, extentOffset: 12));
      await gesture.up();
    });

    testWidgets('can drag start selection handle across end selection handle', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph3, 7),
      ); // at the 'h'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 500));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 11));
      final List<TextBox> boxes = paragraph3.getBoxesForSelection(paragraph3.selections[0]);
      expect(boxes.length, 1);

      final Offset handlePos = globalize(boxes[0].toRect().bottomLeft, paragraph3);
      await gesture.down(handlePos);
      await gesture.moveTo(
        textOffsetToPosition(paragraph3, 14) + Offset(0, paragraph3.size.height / 2),
      );
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 14, extentOffset: 11));

      await gesture.moveTo(
        textOffsetToPosition(paragraph3, 4) + Offset(0, paragraph3.size.height / 2),
      );
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 4, extentOffset: 11));
      await gesture.up();
    });

    testWidgets('can drag end selection handle across start selection handle', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph3, 7),
      ); // at the 'h'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 500));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 11));
      final List<TextBox> boxes = paragraph3.getBoxesForSelection(paragraph3.selections[0]);
      expect(boxes.length, 1);

      final Offset handlePos = globalize(boxes[0].toRect().bottomRight, paragraph3);
      await gesture.down(handlePos);
      await gesture.moveTo(
        textOffsetToPosition(paragraph3, 4) + Offset(0, paragraph3.size.height / 2),
      );
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 4));

      await gesture.moveTo(
        textOffsetToPosition(paragraph3, 12) + Offset(0, paragraph3.size.height / 2),
      );
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 12));
      await gesture.up();
    });

    testWidgets('can select all from toolbar', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph3, 7),
      ); // at the 'h'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 500));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 11));
      expect(find.text('Select all'), findsOneWidget);

      await tester.tap(find.text('Select all'));
      await tester.pump();

      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 16));
      expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
    }, skip: kIsWeb); // [intended] Web uses its native context menu.

    testWidgets('can copy from toolbar', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph3, 7),
      ); // at the 'h'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 500));
      expect(paragraph3.selections[0], const TextSelection(baseOffset: 6, extentOffset: 11));
      expect(find.text('Copy'), findsOneWidget);

      await tester.tap(find.text('Copy'));
      await tester.pump();

      // Selection should be cleared.
      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      expect(paragraph3.selections.isEmpty, isTrue);
      expect(paragraph2.selections.isEmpty, isTrue);
      expect(paragraph1.selections.isEmpty, isTrue);

      final Map<String, dynamic> clipboardData =
          mockClipboard.clipboardData as Map<String, dynamic>;
      expect(clipboardData['text'], 'thank');
    }, skip: kIsWeb); // [intended] Web uses its native context menu.

    testWidgets(
      'can use keyboard to granularly extend selection - character',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        // Select from offset 2 of paragraph1 to offset 6 of paragraph1.
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph1, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
        await gesture.up();
        await tester.pump();

        // Ho[w ar]e you?
        // Good, and you?
        // Fine, thank you.
        expect(paragraph1.selections.length, 1);
        expect(paragraph1.selections[0].start, 2);
        expect(paragraph1.selections[0].end, 6);

        await sendKeyCombination(
          tester,
          const SingleActivator(LogicalKeyboardKey.arrowRight, shift: true),
        );
        await tester.pump();
        // Ho[w are] you?
        // Good, and you?
        // Fine, thank you.
        expect(paragraph1.selections.length, 1);
        expect(paragraph1.selections[0].start, 2);
        expect(paragraph1.selections[0].end, 7);

        for (int i = 0; i < 5; i += 1) {
          await sendKeyCombination(
            tester,
            const SingleActivator(LogicalKeyboardKey.arrowRight, shift: true),
          );
          await tester.pump();
          expect(paragraph1.selections.length, 1);
          expect(paragraph1.selections[0].start, 2);
          expect(paragraph1.selections[0].end, 8 + i);
        }

        for (int i = 0; i < 5; i += 1) {
          await sendKeyCombination(
            tester,
            const SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true),
          );
          await tester.pump();
          expect(paragraph1.selections.length, 1);
          expect(paragraph1.selections[0].start, 2);
          expect(paragraph1.selections[0].end, 11 - i);
        }
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets('can use keyboard to granularly extend selection - word', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      // Select from offset 2 of paragraph1 to offset 6 of paragraph1.
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
      await gesture.up();
      await tester.pump();

      final bool alt;
      final bool control;
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          alt = false;
          control = true;
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          alt = true;
          control = false;
      }

      // Ho[w ar]e you?
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 6);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, alt: alt, control: control),
      );
      await tester.pump();
      // Ho[w are] you?
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 7);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, alt: alt, control: control),
      );
      await tester.pump();
      // Ho[w are you]?
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 11);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, alt: alt, control: control),
      );
      await tester.pump();
      // Ho[w are you?]
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 12);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, alt: alt, control: control),
      );
      await tester.pump();
      // Ho[w are you?
      // Good], and you?
      // Fine, thank you.
      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 12);
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 0);
      expect(paragraph2.selections[0].end, 4);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true, alt: alt, control: control),
      );
      await tester.pump();
      // Ho[w are you?
      // ]Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 12);
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 0);
      expect(paragraph2.selections[0].end, 0);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true, alt: alt, control: control),
      );
      await tester.pump();
      // Ho[w are ]you?
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 8);
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 0);
      expect(paragraph2.selections[0].end, 0);
    }, variant: TargetPlatformVariant.all());

    testWidgets('can use keyboard to granularly extend selection - line', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      // Select from offset 2 of paragraph1 to offset 6 of paragraph1.
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
      await gesture.up();
      await tester.pump();

      final bool alt;
      final bool meta;
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          meta = false;
          alt = true;
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          meta = true;
          alt = false;
      }

      // Ho[w ar]e you?
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 6);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, alt: alt, meta: meta),
      );
      await tester.pump();
      // Ho[w are you?]
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 12);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, alt: alt, meta: meta),
      );
      await tester.pump();
      // Ho[w are you?
      // Good, and you?]
      // Fine, thank you.
      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 12);
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 0);
      expect(paragraph2.selections[0].end, 14);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true, alt: alt, meta: meta),
      );
      await tester.pump();
      // Ho[w are you?]
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 12);
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 0);
      expect(paragraph2.selections[0].end, 0);

      await sendKeyCombination(
        tester,
        SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true, alt: alt, meta: meta),
      );
      await tester.pump();
      // [Ho]w are you?
      // Good, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 0);
      expect(paragraph1.selections[0].end, 2);
    }, variant: TargetPlatformVariant.all());

    testWidgets(
      'can use keyboard to granularly extend selection - document',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              selectionControls: materialTextSelectionControls,
              child: const Column(
                children: <Widget>[
                  Text('How are you?'),
                  Text('Good, and you?'),
                  Text('Fine, thank you.'),
                ],
              ),
            ),
          ),
        );
        // Select from offset 2 of paragraph1 to offset 6 of paragraph1.
        final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
        );
        final TestGesture gesture = await tester.startGesture(
          textOffsetToPosition(paragraph1, 2),
          kind: PointerDeviceKind.mouse,
        );
        addTearDown(gesture.removePointer);
        await tester.pump();
        await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
        await gesture.up();
        await tester.pump();

        final bool alt;
        final bool meta;
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            meta = false;
            alt = true;
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            meta = true;
            alt = false;
        }

        // Ho[w ar]e you?
        // Good, and you?
        // Fine, thank you.
        expect(paragraph1.selections.length, 1);
        expect(paragraph1.selections[0].start, 2);
        expect(paragraph1.selections[0].end, 6);

        await sendKeyCombination(
          tester,
          SingleActivator(LogicalKeyboardKey.arrowDown, shift: true, meta: meta, alt: alt),
        );
        await tester.pump();
        // Ho[w are you?
        // Good, and you?
        // Fine, thank you.]
        final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
        );
        final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
        );
        expect(paragraph1.selections.length, 1);
        expect(paragraph1.selections[0].start, 2);
        expect(paragraph1.selections[0].end, 12);
        expect(paragraph2.selections.length, 1);
        expect(paragraph2.selections[0].start, 0);
        expect(paragraph2.selections[0].end, 14);
        expect(paragraph3.selections.length, 1);
        expect(paragraph3.selections[0].start, 0);
        expect(paragraph3.selections[0].end, 16);

        await sendKeyCombination(
          tester,
          SingleActivator(LogicalKeyboardKey.arrowUp, shift: true, meta: meta, alt: alt),
        );
        await tester.pump();
        // [Ho]w are you?
        // Good, and you?
        // Fine, thank you.
        expect(paragraph1.selections.length, 1);
        expect(paragraph1.selections[0].start, 0);
        expect(paragraph1.selections[0].end, 2);
        expect(paragraph2.selections.length, 1);
        expect(paragraph2.selections[0].start, 0);
        expect(paragraph2.selections[0].end, 0);
        expect(paragraph3.selections.length, 1);
        expect(paragraph3.selections[0].start, 0);
        expect(paragraph3.selections[0].end, 0);
      },
      variant: TargetPlatformVariant.all(),
    );

    testWidgets('can use keyboard to directionally extend selection', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Column(
              children: <Widget>[
                Text('How are you?'),
                Text('Good, and you?'),
                Text('Fine, thank you.'),
              ],
            ),
          ),
        ),
      );
      // Select from offset 2 of paragraph2 to offset 6 of paragraph2.
      final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph2, 2),
        kind: PointerDeviceKind.mouse,
      );
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.moveTo(textOffsetToPosition(paragraph2, 6));
      await gesture.up();
      await tester.pump();

      // How are you?
      // Go[od, ]and you?
      // Fine, thank you.
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 2);
      expect(paragraph2.selections[0].end, 6);

      await sendKeyCombination(
        tester,
        const SingleActivator(LogicalKeyboardKey.arrowDown, shift: true),
      );
      await tester.pump();
      // How are you?
      // Go[od, and you?
      // Fine, t]hank you.
      final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
      );
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 2);
      expect(paragraph2.selections[0].end, 14);
      expect(paragraph3.selections.length, 1);
      expect(paragraph3.selections[0].start, 0);
      expect(paragraph3.selections[0].end, 7);

      await sendKeyCombination(
        tester,
        const SingleActivator(LogicalKeyboardKey.arrowDown, shift: true),
      );
      await tester.pump();
      // How are you?
      // Go[od, and you?
      // Fine, thank you.]
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 2);
      expect(paragraph2.selections[0].end, 14);
      expect(paragraph3.selections.length, 1);
      expect(paragraph3.selections[0].start, 0);
      expect(paragraph3.selections[0].end, 16);

      await sendKeyCombination(
        tester,
        const SingleActivator(LogicalKeyboardKey.arrowUp, shift: true),
      );
      await tester.pump();
      // How are you?
      // Go[od, ]and you?
      // Fine, thank you.
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 2);
      expect(paragraph2.selections[0].end, 6);
      expect(paragraph3.selections.length, 1);
      expect(paragraph3.selections[0].start, 0);
      expect(paragraph3.selections[0].end, 0);

      await sendKeyCombination(
        tester,
        const SingleActivator(LogicalKeyboardKey.arrowUp, shift: true),
      );
      await tester.pump();
      // How a[re you?
      // Go]od, and you?
      // Fine, thank you.
      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 5);
      expect(paragraph1.selections[0].end, 12);
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 0);
      expect(paragraph2.selections[0].end, 2);

      await sendKeyCombination(
        tester,
        const SingleActivator(LogicalKeyboardKey.arrowUp, shift: true),
      );
      await tester.pump();
      // [How are you?
      // Go]od, and you?
      // Fine, thank you.
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 0);
      expect(paragraph1.selections[0].end, 12);
      expect(paragraph2.selections.length, 1);
      expect(paragraph2.selections[0].start, 0);
      expect(paragraph2.selections[0].end, 2);
    }, variant: TargetPlatformVariant.all());

    group('magnifier', () {
      late ValueNotifier<MagnifierInfo> magnifierInfo;
      final Widget fakeMagnifier = Container(key: UniqueKey());

      testWidgets('Can drag handles to show, unshow, and update magnifier', (
        WidgetTester tester,
      ) async {
        const String text = 'Monkeys and rabbits in my soup';

        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              magnifierConfiguration: TextMagnifierConfiguration(
                magnifierBuilder: (
                  _,
                  MagnifierController controller,
                  ValueNotifier<MagnifierInfo> localMagnifierInfo,
                ) {
                  magnifierInfo = localMagnifierInfo;
                  return fakeMagnifier;
                },
              ),
              selectionControls: materialTextSelectionControls,
              child: const Text(text),
            ),
          ),
        );
        await tester.pumpAndSettle();

        final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
          find.descendant(of: find.text(text), matching: find.byType(RichText)),
        );

        // Show the selection handles.
        final TestGesture activateSelectionGesture = await tester.startGesture(
          textOffsetToPosition(paragraph, text.length ~/ 2),
        );
        addTearDown(activateSelectionGesture.removePointer);
        await tester.pump(const Duration(milliseconds: 500));
        await activateSelectionGesture.up();
        await tester.pump(const Duration(milliseconds: 500));

        // Drag the handle around so that the magnifier shows.
        final TextBox selectionBox =
            paragraph.getBoxesForSelection(paragraph.selections.first).first;
        final Offset leftHandlePos = globalize(selectionBox.toRect().bottomLeft, paragraph);
        final TestGesture gesture = await tester.startGesture(leftHandlePos);
        await gesture.moveTo(textOffsetToPosition(paragraph, text.length - 2));
        await tester.pump();

        // Expect the magnifier to show and then store it's position.
        expect(find.byKey(fakeMagnifier.key!), findsOneWidget);
        final Offset firstDragGesturePosition = magnifierInfo.value.globalGesturePosition;

        await gesture.moveTo(textOffsetToPosition(paragraph, text.length));
        await tester.pump();

        // Expect the position the magnifier gets to have moved.
        expect(firstDragGesturePosition, isNot(magnifierInfo.value.globalGesturePosition));

        // Lift the pointer and expect the magnifier to disappear.
        await gesture.up();
        await tester.pump();

        expect(find.byKey(fakeMagnifier.key!), findsNothing);
      });
    });
  });

  testWidgets(
    'toolbar is hidden on Android and iOS when orientation changes',
    (WidgetTester tester) async {
      addTearDown(tester.view.reset);

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionControls,
            child: const Text('How are you?'),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph1, 6),
      ); // at the 'r'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      // `are` is selected.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));
      await tester.pumpAndSettle();

      await gesture.up();
      await tester.pumpAndSettle();
      // Text selection toolbar has appeared.
      expect(find.text('Copy'), findsOneWidget);

      // Hide the toolbar by changing orientation.
      tester.view.physicalSize = const Size(1800.0, 2400.0);
      await tester.pumpAndSettle();
      expect(find.text('Copy'), findsNothing);

      // Handles should be hidden as well on Android
      expect(
        find.descendant(
          of: find.byType(CompositedTransformFollower),
          matching: find.byType(Padding),
        ),
        defaultTargetPlatform == TargetPlatform.android ? findsNothing : findsNWidgets(2),
      );
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
    skip: kIsWeb, // [intended] Web uses its native context menu.
  );

  // Regression test for https://github.com/flutter/flutter/issues/121053.
  testWidgets(
    'Ensure SelectionArea does not affect the layout of its children',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: <Widget>[SelectionArea(child: Text('row 1')), Text('row 2')],
          ),
        ),
      );
      await tester.pumpAndSettle();
      final double xOffset1 = tester.getTopLeft(find.text('row 1')).dx;
      final double xOffset2 = tester.getTopLeft(find.text('row 2')).dx;
      expect(xOffset1, xOffset2);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'the selection behavior when clicking `Copy` item in mobile platforms',
    (WidgetTester tester) async {
      List<ContextMenuButtonItem> buttonItems = <ContextMenuButtonItem>[];

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionHandleControls,
            contextMenuBuilder: (
              BuildContext context,
              SelectableRegionState selectableRegionState,
            ) {
              buttonItems = selectableRegionState.contextMenuButtonItems;
              return const SizedBox.shrink();
            },
            child: const Text('How are you?'),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      await tester.longPressAt(textOffsetToPosition(paragraph1, 6)); // at the 'r'
      await tester.pump(kLongPressTimeout);
      // `are` is selected.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

      // Press `Copy` item.
      expect(buttonItems[0].type, ContextMenuButtonType.copy);
      buttonItems[0].onPressed?.call();

      final SelectableRegionState regionState = tester.state<SelectableRegionState>(
        find.byType(SelectableRegion),
      );

      // In Android copy should clear the selection.
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          expect(regionState.selectionOverlay, isNull);
          expect(regionState.selectionOverlay?.startHandleLayerLink, isNull);
          expect(regionState.selectionOverlay?.endHandleLayerLink, isNull);
        case TargetPlatform.iOS:
          expect(regionState.selectionOverlay, isNotNull);
          expect(regionState.selectionOverlay?.startHandleLayerLink, isNotNull);
          expect(regionState.selectionOverlay?.endHandleLayerLink, isNotNull);
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          // Test doesn't run these platforms.
          break;
      }
    },
    variant: TargetPlatformVariant.mobile(),
    skip: kIsWeb, // [intended] Web uses its native context menu.
  );

  testWidgets(
    'the handles do not disappear when clicking `Select all` item in mobile platforms',
    (WidgetTester tester) async {
      List<ContextMenuButtonItem> buttonItems = <ContextMenuButtonItem>[];

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionHandleControls,
            contextMenuBuilder: (
              BuildContext context,
              SelectableRegionState selectableRegionState,
            ) {
              buttonItems = selectableRegionState.contextMenuButtonItems;
              return const SizedBox.shrink();
            },
            child: const Text('How are you?'),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      await tester.longPressAt(textOffsetToPosition(paragraph1, 6)); // at the 'r'
      await tester.pump(kLongPressTimeout);
      // `are` is selected.
      expect(paragraph1.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

      late ContextMenuButtonItem selectAllButton;
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
          // On Android, the select all button is after the share button.
          expect(buttonItems[2].type, ContextMenuButtonType.selectAll);
          selectAllButton = buttonItems[2];
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(buttonItems[1].type, ContextMenuButtonType.selectAll);
          selectAllButton = buttonItems[1];
      }

      // Press `Select All` item.
      selectAllButton.onPressed?.call();

      final SelectableRegionState regionState = tester.state<SelectableRegionState>(
        find.byType(SelectableRegion),
      );

      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
          expect(regionState.selectionOverlay, isNotNull);
          expect(regionState.selectionOverlay?.startHandleLayerLink, isNotNull);
          expect(regionState.selectionOverlay?.endHandleLayerLink, isNotNull);
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          // Test doesn't run these platforms.
          break;
      }
    },
    variant: TargetPlatformVariant.mobile(),
    skip: kIsWeb, // [intended] Web uses its native context menu.
  );

  testWidgets(
    'Selection behavior when clicking the `Share` button on Android',
    (WidgetTester tester) async {
      List<ContextMenuButtonItem> buttonItems = <ContextMenuButtonItem>[];

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionHandleControls,
            contextMenuBuilder: (
              BuildContext context,
              SelectableRegionState selectableRegionState,
            ) {
              buttonItems = selectableRegionState.contextMenuButtonItems;
              return const SizedBox.shrink();
            },
            child: const Text('How are you?'),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      await tester.longPressAt(textOffsetToPosition(paragraph, 6)); // at the 'r'
      await tester.pump(kLongPressTimeout);

      // `are` is selected.
      expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

      String? lastShare;
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          if (methodCall.method == 'Share.invoke') {
            expect(methodCall.arguments, isA<String>());
            lastShare = methodCall.arguments as String;
          }
          return null;
        },
      );
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.platform,
          null,
        ),
      );

      final SelectableRegionState regionState = tester.state<SelectableRegionState>(
        find.byType(SelectableRegion),
      );

      // Press the `Share` button.
      expect(buttonItems[1].type, ContextMenuButtonType.share);
      buttonItems[1].onPressed?.call();
      expect(lastShare, 'are');
      // On Android, share should clear the selection.
      expect(regionState.selectionOverlay, isNull);
      expect(regionState.selectionOverlay?.startHandleLayerLink, isNull);
      expect(regionState.selectionOverlay?.endHandleLayerLink, isNull);
    },
    skip: kIsWeb, // [intended] Web uses its native context menu.
  );

  testWidgets(
    'builds the correct button items',
    (WidgetTester tester) async {
      List<ContextMenuButtonItem> buttonItems = <ContextMenuButtonItem>[];

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionHandleControls,
            contextMenuBuilder: (
              BuildContext context,
              SelectableRegionState selectableRegionState,
            ) {
              buttonItems = selectableRegionState.contextMenuButtonItems;
              return const SizedBox.shrink();
            },
            child: const Text('How are you?'),
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 6),
      ); // at the 'r'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      // `are` is selected.
      expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

      await gesture.up();
      await tester.pumpAndSettle();

      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
          // On Android, the share button is before the select all button.
          expect(buttonItems.length, 3);
          expect(buttonItems[0].type, ContextMenuButtonType.copy);
          expect(buttonItems[1].type, ContextMenuButtonType.share);
          expect(buttonItems[2].type, ContextMenuButtonType.selectAll);
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(buttonItems.length, 2);
          expect(buttonItems[0].type, ContextMenuButtonType.copy);
          expect(buttonItems[1].type, ContextMenuButtonType.selectAll);
      }
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended] Web uses its native context menu.
  );

  testWidgets('can clear selection through SelectableRegionState', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: SelectableRegion(
          selectionControls: materialTextSelectionControls,
          child: const Column(
            children: <Widget>[
              Text('How are you?'),
              Text('Good, and you?'),
              Text('Fine, thank you.'),
            ],
          ),
        ),
      ),
    );

    final SelectableRegionState state = tester.state<SelectableRegionState>(
      find.byType(SelectableRegion),
    );
    final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
    );
    final TestGesture gesture = await tester.startGesture(
      textOffsetToPosition(paragraph1, 2),
      kind: PointerDeviceKind.mouse,
    );
    addTearDown(gesture.removePointer);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    await gesture.down(textOffsetToPosition(paragraph1, 2));
    await tester.pumpAndSettle();
    expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 3));

    await gesture.moveTo(textOffsetToPosition(paragraph1, 4));
    await tester.pump();
    expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 7));

    final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
    );
    await gesture.moveTo(textOffsetToPosition(paragraph2, 5));
    // Should select the rest of paragraph 1.
    expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
    expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 6));

    final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
    );
    await gesture.moveTo(textOffsetToPosition(paragraph3, 6));
    expect(paragraph1.selections[0], const TextSelection(baseOffset: 0, extentOffset: 12));
    expect(paragraph2.selections[0], const TextSelection(baseOffset: 0, extentOffset: 14));
    expect(paragraph3.selections[0], const TextSelection(baseOffset: 0, extentOffset: 11));
    await gesture.up();
    await tester.pumpAndSettle();

    // Clear selection programmatically.
    state.clearSelection();
    expect(paragraph1.selections, isEmpty);
    expect(paragraph2.selections, isEmpty);
    expect(paragraph3.selections, isEmpty);
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/125582.

  testWidgets(
    'Text processing actions are added to the toolbar',
    (WidgetTester tester) async {
      final MockProcessTextHandler mockProcessTextHandler = MockProcessTextHandler();
      TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.processText,
        mockProcessTextHandler.handleMethodCall,
      );
      addTearDown(
        () => tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.processText,
          null,
        ),
      );

      Set<String?> buttonLabels = <String?>{};

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableRegion(
            selectionControls: materialTextSelectionHandleControls,
            contextMenuBuilder: (
              BuildContext context,
              SelectableRegionState selectableRegionState,
            ) {
              buttonLabels =
                  selectableRegionState.contextMenuButtonItems
                      .map((ContextMenuButtonItem buttonItem) => buttonItem.label)
                      .toSet();
              return const SizedBox.shrink();
            },
            child: const Text('How are you?'),
          ),
        ),
      );
      await tester.pumpAndSettle();

      final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
        find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
      );
      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(paragraph, 6),
      ); // at the 'r'
      addTearDown(gesture.removePointer);
      await tester.pump(const Duration(milliseconds: 500));
      // `are` is selected.
      expect(paragraph.selections[0], const TextSelection(baseOffset: 4, extentOffset: 7));

      await gesture.up();
      await tester.pumpAndSettle();

      // The text processing actions are available on Android only.
      final bool areTextActionsSupported = defaultTargetPlatform == TargetPlatform.android;
      expect(buttonLabels.contains(fakeAction1Label), areTextActionsSupported);
      expect(buttonLabels.contains(fakeAction2Label), areTextActionsSupported);
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended] Web uses its native context menu.
  );

  testWidgets('SelectionListener onSelectionChanged is accurate with WidgetSpans', (
    WidgetTester tester,
  ) async {
    final List<String> dataModel = <String>['Hello world, ', 'how are you today.'];
    final SelectionListenerNotifier selectionNotifier = SelectionListenerNotifier();
    addTearDown(selectionNotifier.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: SelectableRegion(
          selectionControls: materialTextSelectionControls,
          child: SelectionListener(
            selectionNotifier: selectionNotifier,
            child: Column(
              children: <Widget>[
                Text.rich(
                  TextSpan(
                    text: dataModel[0],
                    children: <InlineSpan>[WidgetSpan(child: Text(dataModel[1]))],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
      find.descendant(
        of: find.textContaining('Hello world'),
        matching: find.byType(RichText).first,
      ),
    );
    final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('how are you today.'), matching: find.byType(RichText)),
    );
    final TestGesture mouseGesture = await tester.startGesture(
      textOffsetToPosition(paragraph1, 0),
      kind: PointerDeviceKind.mouse,
    );

    addTearDown(mouseGesture.removePointer);
    await tester.pump();

    SelectedContentRange? selectedRange;

    // Selection on paragraph1.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph1, 1));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 0);
    expect(selectedRange.endOffset, 1);

    // Selection on paragraph1.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph1, 10));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 0);
    expect(selectedRange.endOffset, 10);

    // Selection on paragraph1 and paragraph2.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph2, 10));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 0);
    expect(selectedRange.endOffset, 23);
    await mouseGesture.up();
    await tester.pump();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 0);
    expect(selectedRange.endOffset, 23);

    // Collapsed selection.
    await mouseGesture.down(textOffsetToPosition(paragraph2, 3));
    await tester.pump();
    await mouseGesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    expect(selectionNotifier.selection.status, SelectionStatus.collapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 16);
    expect(selectedRange.endOffset, 16);

    // Backwards selection.
    await mouseGesture.down(textOffsetToPosition(paragraph2, 4));
    await tester.pump();
    await mouseGesture.moveTo(textOffsetToPosition(paragraph1, 0));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 17);
    expect(selectedRange.endOffset, 0);
    await mouseGesture.up();
    await tester.pump();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 17);
    expect(selectedRange.endOffset, 0);

    // Collapsed selection.
    await mouseGesture.down(textOffsetToPosition(paragraph1, 0));
    await tester.pumpAndSettle();
    await mouseGesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    expect(selectionNotifier.selection.status, SelectionStatus.collapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 0);
    expect(selectedRange.endOffset, 0);
  });

  testWidgets('onSelectionChanged SelectedContentRange is accurate', (WidgetTester tester) async {
    final List<String> dataModel = <String>['How are you?', 'Good, and you?', 'Fine, thank you.'];
    final SelectionListenerNotifier selectionNotifier = SelectionListenerNotifier();
    SelectedContentRange? selectedRange;
    addTearDown(selectionNotifier.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: SelectableRegion(
          selectionControls: materialTextSelectionControls,
          child: SelectionListener(
            selectionNotifier: selectionNotifier,
            child: Column(
              children: <Widget>[Text(dataModel[0]), Text(dataModel[1]), Text(dataModel[2])],
            ),
          ),
        ),
      ),
    );

    final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
    );
    final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
    );
    final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
    );
    final TestGesture mouseGesture = await tester.startGesture(
      textOffsetToPosition(paragraph1, 4),
      kind: PointerDeviceKind.mouse,
    );

    addTearDown(mouseGesture.removePointer);
    await tester.pump();

    // Selection on paragraph1.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph1, 7));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 4);
    expect(selectedRange.endOffset, 7);

    // Selection on paragraph1.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph1, 10));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 4);
    expect(selectedRange.endOffset, 10);

    // Selection on paragraph1 and paragraph2.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph2, 10));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 4);
    expect(selectedRange.endOffset, 22);

    // Selection on paragraph1, paragraph2, and paragraph3.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph3, 10));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 4);
    expect(selectedRange.endOffset, 36);
    await mouseGesture.up();
    await tester.pump();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 4);
    expect(selectedRange.endOffset, 36);

    // Collapsed selection.
    await mouseGesture.down(textOffsetToPosition(paragraph1, 3));
    await tester.pump();
    await mouseGesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    expect(selectionNotifier.selection.status, SelectionStatus.collapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 3);
    expect(selectedRange.endOffset, 3);

    // Backwards selection.
    await mouseGesture.down(textOffsetToPosition(paragraph3, 4));
    await tester.pump();
    await mouseGesture.moveTo(textOffsetToPosition(paragraph1, 0));
    await tester.pumpAndSettle();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 30);
    expect(selectedRange.endOffset, 0);
    await mouseGesture.up();
    await tester.pump();
    expect(selectionNotifier.selection.status, SelectionStatus.uncollapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 30);
    expect(selectedRange.endOffset, 0);

    // Collapsed selection.
    await mouseGesture.down(textOffsetToPosition(paragraph1, 0));
    await tester.pumpAndSettle();
    await mouseGesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    expect(selectionNotifier.selection.status, SelectionStatus.collapsed);
    selectedRange = selectionNotifier.selection.range;
    expect(selectedRange, isNotNull);
    expect(selectedRange!.startOffset, 0);
    expect(selectedRange.endOffset, 0);
  });

  testWidgets('onSelectionChange is called when the selection changes through gestures', (
    WidgetTester tester,
  ) async {
    SelectedContent? content;

    await tester.pumpWidget(
      MaterialApp(
        home: SelectableRegion(
          onSelectionChanged: (SelectedContent? selectedContent) => content = selectedContent,
          selectionControls: materialTextSelectionControls,
          child: const Center(child: Text('How are you')),
        ),
      ),
    );

    final RenderParagraph paragraph = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('How are you'), matching: find.byType(RichText)),
    );
    final TestGesture mouseGesture = await tester.startGesture(
      textOffsetToPosition(paragraph, 4),
      kind: PointerDeviceKind.mouse,
    );
    final TestGesture touchGesture = await tester.createGesture();

    expect(content, isNull);
    addTearDown(mouseGesture.removePointer);
    addTearDown(touchGesture.removePointer);
    await tester.pump();

    // Called on drag.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph, 7));
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'are');

    // Updates on drag.
    await mouseGesture.moveTo(textOffsetToPosition(paragraph, 10));
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'are yo');

    // Called on drag end.
    await mouseGesture.up();
    await tester.pump();
    expect(content, isNotNull);
    expect(content!.plainText, 'are yo');

    // Backwards selection.
    await mouseGesture.down(textOffsetToPosition(paragraph, 3));
    await tester.pump();
    await mouseGesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    expect(content, isNotNull);
    expect(content!.plainText, '');

    await mouseGesture.down(textOffsetToPosition(paragraph, 3));
    await tester.pump();

    await mouseGesture.moveTo(textOffsetToPosition(paragraph, 0));
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'How');

    await mouseGesture.up();
    await tester.pump();
    expect(content, isNotNull);
    expect(content!.plainText, 'How');

    // Called on double tap.
    await mouseGesture.down(textOffsetToPosition(paragraph, 6));
    await tester.pump();
    await mouseGesture.up();
    await tester.pump();
    await mouseGesture.down(textOffsetToPosition(paragraph, 6));
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'are');
    await mouseGesture.up();
    await tester.pumpAndSettle();

    // Called on tap.
    await mouseGesture.down(textOffsetToPosition(paragraph, 0));
    await tester.pumpAndSettle();
    await mouseGesture.up();
    await tester.pumpAndSettle(kDoubleTapTimeout);
    expect(content, isNotNull);
    expect(content!.plainText, '');

    // With touch gestures.

    // Called on long press start.
    await touchGesture.down(textOffsetToPosition(paragraph, 0));
    await tester.pumpAndSettle(kLongPressTimeout);
    expect(content, isNotNull);
    expect(content!.plainText, 'How');

    // Called on long press update.
    await touchGesture.moveTo(textOffsetToPosition(paragraph, 5));
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'How are');

    // Called on long press end.
    await touchGesture.up();
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'How are');

    // Long press to select 'you'.
    await touchGesture.down(textOffsetToPosition(paragraph, 9));
    await tester.pumpAndSettle(kLongPressTimeout);
    expect(content, isNotNull);
    expect(content!.plainText, 'you');
    await touchGesture.up();
    await tester.pumpAndSettle();

    // Called while moving selection handles.
    final List<TextBox> boxes = paragraph.getBoxesForSelection(paragraph.selections[0]);
    expect(boxes.length, 1);
    final Offset startHandlePos = globalize(boxes[0].toRect().bottomLeft, paragraph);
    final Offset endHandlePos = globalize(boxes[0].toRect().bottomRight, paragraph);
    final Offset startPos = Offset(textOffsetToPosition(paragraph, 4).dx, startHandlePos.dy);
    final Offset endPos = Offset(textOffsetToPosition(paragraph, 6).dx, endHandlePos.dy);

    // Start handle.
    await touchGesture.down(startHandlePos);
    await touchGesture.moveTo(startPos);
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'are you');
    await touchGesture.up();
    await tester.pumpAndSettle();

    // End handle.
    await touchGesture.down(endHandlePos);
    await touchGesture.moveTo(endPos);
    await tester.pumpAndSettle();
    expect(content, isNotNull);
    expect(content!.plainText, 'ar');
    await touchGesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('onSelectionChange is called when the selection changes through keyboard actions', (
    WidgetTester tester,
  ) async {
    SelectedContent? content;

    await tester.pumpWidget(
      MaterialApp(
        home: SelectableRegion(
          onSelectionChanged: (SelectedContent? selectedContent) => content = selectedContent,
          selectionControls: materialTextSelectionControls,
          child: const Column(
            children: <Widget>[
              Text('How are you?'),
              Text('Good, and you?'),
              Text('Fine, thank you.'),
            ],
          ),
        ),
      ),
    );

    expect(content, isNull);
    await tester.pump();

    final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('How are you?'), matching: find.byType(RichText)),
    );
    final RenderParagraph paragraph2 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Good, and you?'), matching: find.byType(RichText)),
    );
    final RenderParagraph paragraph3 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.text('Fine, thank you.'), matching: find.byType(RichText)),
    );
    final TestGesture gesture = await tester.startGesture(
      textOffsetToPosition(paragraph1, 2),
      kind: PointerDeviceKind.mouse,
    );
    addTearDown(gesture.removePointer);
    await tester.pump();
    await gesture.moveTo(textOffsetToPosition(paragraph1, 6));
    await gesture.up();
    await tester.pump();

    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0].start, 2);
    expect(paragraph1.selections[0].end, 6);
    expect(content, isNotNull);
    expect(content!.plainText, 'w ar');

    await sendKeyCombination(
      tester,
      const SingleActivator(LogicalKeyboardKey.arrowRight, shift: true),
    );
    await tester.pump();
    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0].start, 2);
    expect(paragraph1.selections[0].end, 7);
    expect(content, isNotNull);
    expect(content!.plainText, 'w are');

    for (int i = 0; i < 5; i += 1) {
      await sendKeyCombination(
        tester,
        const SingleActivator(LogicalKeyboardKey.arrowRight, shift: true),
      );
      await tester.pump();
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 8 + i);
      expect(content, isNotNull);
    }
    expect(content, isNotNull);
    expect(content!.plainText, 'w are you?');

    for (int i = 0; i < 5; i += 1) {
      await sendKeyCombination(
        tester,
        const SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true),
      );
      await tester.pump();
      expect(paragraph1.selections.length, 1);
      expect(paragraph1.selections[0].start, 2);
      expect(paragraph1.selections[0].end, 11 - i);
      expect(content, isNotNull);
    }
    expect(content, isNotNull);
    expect(content!.plainText, 'w are');

    await sendKeyCombination(
      tester,
      const SingleActivator(LogicalKeyboardKey.arrowDown, shift: true),
    );
    await tester.pump();
    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0].start, 2);
    expect(paragraph1.selections[0].end, 12);
    expect(paragraph2.selections.length, 1);
    expect(paragraph2.selections[0].start, 0);
    expect(paragraph2.selections[0].end, 8);
    expect(content, isNotNull);
    expect(content!.plainText, 'w are you?Good, an');

    await sendKeyCombination(
      tester,
      const SingleActivator(LogicalKeyboardKey.arrowDown, shift: true),
    );
    await tester.pump();
    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0].start, 2);
    expect(paragraph1.selections[0].end, 12);
    expect(paragraph2.selections.length, 1);
    expect(paragraph2.selections[0].start, 0);
    expect(paragraph2.selections[0].end, 14);
    expect(paragraph3.selections.length, 1);
    expect(paragraph3.selections[0].start, 0);
    expect(paragraph3.selections[0].end, 9);
    expect(content, isNotNull);
    expect(content!.plainText, 'w are you?Good, and you?Fine, tha');

    await sendKeyCombination(
      tester,
      const SingleActivator(LogicalKeyboardKey.arrowDown, shift: true),
    );
    await tester.pump();
    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0].start, 2);
    expect(paragraph1.selections[0].end, 12);
    expect(paragraph2.selections.length, 1);
    expect(paragraph2.selections[0].start, 0);
    expect(paragraph2.selections[0].end, 14);
    expect(paragraph3.selections.length, 1);
    expect(paragraph3.selections[0].start, 0);
    expect(paragraph3.selections[0].end, 16);
    expect(content, isNotNull);
    expect(content!.plainText, 'w are you?Good, and you?Fine, thank you.');

    await sendKeyCombination(
      tester,
      const SingleActivator(LogicalKeyboardKey.arrowUp, shift: true),
    );
    await tester.pump();
    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0].start, 2);
    expect(paragraph1.selections[0].end, 12);
    expect(paragraph2.selections.length, 1);
    expect(paragraph2.selections[0].start, 0);
    expect(paragraph2.selections[0].end, 8);
    expect(paragraph3.selections.length, 1);
    expect(content, isNotNull);
    expect(content!.plainText, 'w are you?Good, an');

    await sendKeyCombination(
      tester,
      const SingleActivator(LogicalKeyboardKey.arrowUp, shift: true),
    );
    await tester.pump();
    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0].start, 2);
    expect(paragraph1.selections[0].end, 7);
    expect(paragraph2.selections.length, 1);
    expect(paragraph3.selections.length, 1);
    expect(content, isNotNull);
    expect(content!.plainText, 'w are');

    await sendKeyCombination(
      tester,
      const SingleActivator(LogicalKeyboardKey.arrowUp, shift: true),
    );
    await tester.pump();
    expect(paragraph1.selections.length, 1);
    expect(paragraph1.selections[0].start, 0);
    expect(paragraph1.selections[0].end, 2);
    expect(paragraph2.selections.length, 1);
    expect(paragraph3.selections.length, 1);
    expect(content, isNotNull);
    expect(content!.plainText, 'Ho');
  });

  group('BrowserContextMenu', () {
    setUp(() async {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.contextMenu,
        (MethodCall call) {
          // Just complete successfully, so that BrowserContextMenu thinks that
          // the engine successfully received its call.
          return Future<void>.value();
        },
      );
      await BrowserContextMenu.disableContextMenu();
    });

    tearDown(() async {
      await BrowserContextMenu.enableContextMenu();
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.contextMenu,
        null,
      );
    });

    testWidgets(
      'web can show flutter context menu when the browser context menu is disabled',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: SelectableRegion(
              onSelectionChanged: (SelectedContent? selectedContent) {},
              selectionControls: materialTextSelectionControls,
              child: const Center(child: Text('How are you')),
            ),
          ),
        );
        await tester.pumpAndSettle();

        final SelectableRegionState state = tester.state<SelectableRegionState>(
          find.byType(SelectableRegion),
        );
        expect(find.text('Copy'), findsNothing);

        state.selectAll(SelectionChangedCause.toolbar);
        await tester.pumpAndSettle();
        expect(find.text('Copy'), findsOneWidget);

        state.hideToolbar();
        await tester.pumpAndSettle();
        expect(find.text('Copy'), findsNothing);
      },
      skip: !kIsWeb, // [intended]
    );
  });

  testWidgets('Multiple selectables on a single line should be in screen order', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/127942.
    final UniqueKey outerText = UniqueKey();
    const TextStyle textStyle = TextStyle(fontSize: 10);

    await tester.pumpWidget(
      MaterialApp(
        home: SelectableRegion(
          selectionControls: materialTextSelectionControls,
          child: Scaffold(
            body: Center(
              child: Text.rich(
                const TextSpan(
                  children: <InlineSpan>[
                    TextSpan(text: 'Hello my name is ', style: textStyle),
                    WidgetSpan(
                      child: Text('Dash', style: textStyle),
                      alignment: PlaceholderAlignment.middle,
                    ),
                    TextSpan(text: '.', style: textStyle),
                  ],
                ),
                key: outerText,
              ),
            ),
          ),
        ),
      ),
    );
    final RenderParagraph paragraph1 = tester.renderObject<RenderParagraph>(
      find.descendant(of: find.byKey(outerText), matching: find.byType(RichText)).first,
    );
    final TestGesture gesture = await tester.startGesture(
      textOffsetToPosition(paragraph1, 0),
      kind: PointerDeviceKind.mouse,
    );
    addTearDown(gesture.removePointer);
    await tester.pump();
    await gesture.up();

    // Select all.
    await sendKeyCombination(tester, const SingleActivator(LogicalKeyboardKey.keyA, control: true));

    // keyboard copy.
    await sendKeyCombination(tester, const SingleActivator(LogicalKeyboardKey.keyC, control: true));

    final Map<String, dynamic> clipboardData = mockClipboard.clipboardData as Map<String, dynamic>;
    expect(clipboardData['text'], 'Hello my name is Dash.');
  });
}

class ColumnSelectionContainerDelegate extends StaticSelectionContainerDelegate {
  /// Copies the selected contents of all [Selectable]s, separating their
  /// contents with a new line.
  @override
  SelectedContent? getSelectedContent() {
    final List<SelectedContent> selections = <SelectedContent>[
      for (final Selectable selectable in selectables)
        if (selectable.getSelectedContent() case final SelectedContent data) data,
    ];
    if (selections.isEmpty) {
      return null;
    }
    return SelectedContent(
      plainText: selections
          .map((SelectedContent selectedContent) => selectedContent.plainText)
          .join('\n'),
    );
  }
}

class SelectionSpy extends LeafRenderObjectWidget {
  const SelectionSpy({super.key});

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderSelectionSpy(SelectionContainer.maybeOf(context));
  }

  @override
  void updateRenderObject(BuildContext context, covariant RenderObject renderObject) {}
}

class RenderSelectionSpy extends RenderProxyBox with Selectable, SelectionRegistrant {
  RenderSelectionSpy(SelectionRegistrar? registrar) {
    this.registrar = registrar;
  }

  final Set<VoidCallback> listeners = <VoidCallback>{};
  List<SelectionEvent> events = <SelectionEvent>[];

  @override
  Size get size => _size;
  Size _size = Size.zero;

  @override
  List<Rect> get boundingBoxes => <Rect>[paintBounds];

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    _size = Size(constraints.maxWidth, constraints.maxHeight);
    return _size;
  }

  @override
  void addListener(VoidCallback listener) => listeners.add(listener);

  @override
  void removeListener(VoidCallback listener) => listeners.remove(listener);

  @override
  SelectionResult dispatchSelectionEvent(SelectionEvent event) {
    events.add(event);
    return SelectionResult.end;
  }

  @override
  SelectedContent? getSelectedContent() {
    return const SelectedContent(plainText: 'content');
  }

  @override
  SelectedContentRange? getSelection() {
    return null;
  }

  @override
  int get contentLength => 1;

  @override
  final SelectionGeometry value = const SelectionGeometry(
    hasContent: true,
    status: SelectionStatus.uncollapsed,
    startSelectionPoint: SelectionPoint(
      localPosition: Offset.zero,
      lineHeight: 0.0,
      handleType: TextSelectionHandleType.left,
    ),
    endSelectionPoint: SelectionPoint(
      localPosition: Offset.zero,
      lineHeight: 0.0,
      handleType: TextSelectionHandleType.left,
    ),
  );

  @override
  void pushHandleLayers(LayerLink? startHandle, LayerLink? endHandle) {}
}

class SelectAllWidget extends SingleChildRenderObjectWidget {
  const SelectAllWidget({super.key, super.child});

  @override
  RenderObject createRenderObject(BuildContext context) {
    return RenderSelectAll(SelectionContainer.maybeOf(context));
  }

  @override
  void updateRenderObject(BuildContext context, covariant RenderObject renderObject) {}
}

class RenderSelectAll extends RenderProxyBox with Selectable, SelectionRegistrant {
  RenderSelectAll(SelectionRegistrar? registrar) {
    this.registrar = registrar;
  }

  @override
  List<Rect> get boundingBoxes => <Rect>[paintBounds];

  final Set<VoidCallback> listeners = <VoidCallback>{};
  LayerLink? startHandle;
  LayerLink? endHandle;

  @override
  void addListener(VoidCallback listener) => listeners.add(listener);

  @override
  void removeListener(VoidCallback listener) => listeners.remove(listener);

  @override
  SelectionResult dispatchSelectionEvent(SelectionEvent event) {
    value = SelectionGeometry(
      hasContent: true,
      status: SelectionStatus.uncollapsed,
      startSelectionPoint: SelectionPoint(
        localPosition: Offset(0, size.height),
        lineHeight: 0.0,
        handleType: TextSelectionHandleType.left,
      ),
      endSelectionPoint: SelectionPoint(
        localPosition: Offset(size.width, size.height),
        lineHeight: 0.0,
        handleType: TextSelectionHandleType.left,
      ),
    );
    return SelectionResult.end;
  }

  @override
  SelectedContent? getSelectedContent() {
    return const SelectedContent(plainText: 'content');
  }

  @override
  SelectedContentRange? getSelection() {
    return null;
  }

  @override
  int get contentLength => 1;

  @override
  SelectionGeometry get value => _value;
  SelectionGeometry _value = const SelectionGeometry(
    hasContent: true,
    status: SelectionStatus.uncollapsed,
    startSelectionPoint: SelectionPoint(
      localPosition: Offset.zero,
      lineHeight: 0.0,
      handleType: TextSelectionHandleType.left,
    ),
    endSelectionPoint: SelectionPoint(
      localPosition: Offset.zero,
      lineHeight: 0.0,
      handleType: TextSelectionHandleType.left,
    ),
  );
  set value(SelectionGeometry other) {
    if (other == _value) {
      return;
    }
    _value = other;
    for (final VoidCallback callback in listeners) {
      callback();
    }
  }

  @override
  void pushHandleLayers(LayerLink? startHandle, LayerLink? endHandle) {
    this.startHandle = startHandle;
    this.endHandle = endHandle;
  }
}
